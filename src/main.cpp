// Copyright (c) 2022 Trevor Makes

#include "core/io.hpp"
#include "core/io/bus.hpp"
#include "core/cli.hpp"
#include "core/mon.hpp"

#include <Arduino.h>

using core::cli::Args;
using core::mon::parse_unsigned;
using core::serial::StreamEx;
using CLI = core::cli::CLI<>;

// Create command line interface around Arduino Serial
StreamEx serialEx(Serial);
CLI serialCli(serialEx);

using core::io::ActiveLow;
using core::io::ActiveHigh;
using core::io::BitExtend;
using core::io::WordExtend;
using core::io::Latch;

// EEPROM needs at least 70 ns before reading data after pulling chip select low
// Wrap the data port with this template to insert the required delay
// TODO maybe this should be integrated into core::io::Control or Bus
template <typename DATA>
struct DelayRead : DATA {
  using TYPE = typename DATA::TYPE;
  static inline TYPE read() {
    // Each NOP delays only 65 ns at 16 MHz, so we need two
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    return DATA::read();
  }
};

#ifdef __AVR_ATmega328P__

// Create Port* wrappers around AVR ports B, C, D
CORE_PORT(B)
CORE_PORT(C)
CORE_PORT(D)

// 8-bit data bus [D7 D6 D5 D4 B3 B2 B1 B0]
using DataPort = BitExtend<PortD::Mask<0xF0>, PortB::Mask<0x0F>>;

// Latch upper and lower bytes of address from data port
using MSBLatch = ActiveHigh<PortC::Bit<5>>;
using LSBLatch = ActiveHigh<PortC::Bit<4>>;
using AddressMSB = Latch<DataPort, MSBLatch>;
using AddressLSB = Latch<DataPort, LSBLatch>;
using AddressPort = WordExtend<AddressMSB, AddressLSB>;

// Bus control lines
using ReadEnable = ActiveLow<PortC::Bit<3>>;
using WriteEnable = ActiveLow<PortC::Bit<2>>;
using Control = core::io::Control<ReadEnable, WriteEnable>;

using Bus = core::io::Bus<AddressPort, DelayRead<DataPort>, Control>;

#else
#error Need to provide configuration for current platform. See __AVR_ATmega328P__ configuration above.
#endif

void setup() {
  Bus::config_write();

  // Establish serial connection with computer
  Serial.begin(9600);
  while (!Serial) {}
}

void set_lsb(Args);
void set_msb(Args);
void set_data(Args);
void write_bus(Args);
void read_bus(Args);

void measure_page_write(Args);

// TODO make this a wrapper around Bus?
// static constexpr auto& write_byte = eeprom_write;
void eeprom_write(uint16_t address, uint8_t data) {
  // TODO would need page_buf from write -and- flush, so should be static member of Bus wrapper struct?
  // TODO need a bitfield  to handle partial page writes?
  static uint8_t page_buf[64];
  //static uint16_t last_page = 0; // ????
  uint16_t page = address & ~63;
  uint8_t index = address & 63;
  page_buf[index] = data;
  // TODO instead of flushing on index 63 (assumes write in ascending order)
  // - when page != last_page
  // - when explicit flush() is called
  if (index == 63) {
    /*if (last write maybe pending) {
      // poll until last write complete (max 10ms)
      while (Bus::read_byte(last_address) != last_data) {}
    }*/
    Bus::config_write(); // TODO config should be done by mon
    for (uint8_t i = 0; i < 64; ++i) { 
      Bus::write_byte(page + i, page_buf[i]);
    }
    Bus::config_read(); // TODO config should be done by mon
  }
}

// Replace write_byte to measure IHX import rate
// static constexpr auto& write_byte = measure_import;
void measure_import(uint16_t address, uint8_t data) {
  static auto t1 = millis();
  if ((address & 63) == 63) {
    auto t2 = millis();
    serialEx.println(t2 - t1);
    t1 = t2;
  }
}

// Define interface for core::mon function templates
struct API : core::mon::Base<API> {
  // TODO API/Base should use something like `using Bus = Bus;`
  // - mon should use config_read/config_write internally
  // - maybe add a flush_write to help with EEPROM buffering?
  static constexpr auto& read_byte = Bus::read_byte;
  static constexpr auto& write_byte = eeprom_write;
  static StreamEx& get_stream() { return serialEx; }
  static CLI& get_cli() { return serialCli; }
};

void loop() {
  static const core::cli::Command commands[] = {
    { "lsb", set_lsb },
    { "msb", set_msb },
    { "data", set_data },
    { "write", write_bus },
    { "read", read_bus },
    { "hex", core::mon::cmd_hex<API> },
    { "export", core::mon::cmd_export<API> },
    { "import", core::mon::cmd_import<API> },
    { "measure", measure_page_write },
  };

  serialCli.run_once(commands);
}

void set_lsb(Args args) {
  uint8_t data = 0;
  parse_unsigned(data, args.next());
  AddressLSB::config_output();
  AddressLSB::write(data);
}

void set_msb(Args args) {
  uint8_t data = 0;
  parse_unsigned(data, args.next());
  AddressMSB::config_output();
  AddressMSB::write(data);
}

void set_data(Args args) {
  uint8_t data = 0;
  parse_unsigned(data, args.next());
  DataPort::config_output();
  DataPort::write(data);
}

void write_bus(Args args) {
  uint16_t addr = 0;
  uint8_t data = 0;
  parse_unsigned(addr, args.next());
  parse_unsigned(data, args.next());
  Bus::config_write();
  Bus::write_byte(addr, data);

  // Measure data polling to completion
  Bus::config_read();
  auto t1 = millis();
  while (Bus::read_byte(addr) != data) {}
  auto t2 = millis();
  serialEx.println(t2 - t1);
}

void read_bus(Args args) {
  uint16_t addr = 0;
  parse_unsigned(addr, args.next());
  Bus::config_read();
  uint8_t data = Bus::read_byte(addr);
  serialEx.println(data, HEX);
}

// Measure EEPROM page load/write timing
void measure_page_write(Args) {
  Bus::config_write();

  // Measure 64 byte page load
  auto t1 = micros();
  for (uint16_t addr = 0x140; addr < 0x180; ++addr) {
    Bus::write_byte(addr, addr);
  }
  auto t2 = micros();

  // Measure data polling to completion
  Bus::config_read();
  while (Bus::read_byte(0x17F) != 0x7F) {}
  auto t3 = micros();

  serialEx.println(t2 - t1);
  serialEx.println(t3 - t2);
}
