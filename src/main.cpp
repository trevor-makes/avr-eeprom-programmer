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

template <typename BUS, uint8_t PAGE_SIZE = 64>
struct PagedWrite : BUS {
  static_assert(core::util::is_power_of_two(PAGE_SIZE), "");
  using typename BUS::DATA_TYPE;
  using typename BUS::ADDRESS_TYPE;

  static DATA_TYPE page_data[PAGE_SIZE];
  static bool page_mask[PAGE_SIZE]; // TODO compact bitfield instead of bool array?
  static ADDRESS_TYPE last_page;
  // TODO maybe store last address (instead of page) and data for polling

  static void write_byte(ADDRESS_TYPE address, DATA_TYPE data) {
    ADDRESS_TYPE page = address & ~(PAGE_SIZE - 1);
    uint8_t index = address & (PAGE_SIZE - 1);
    if (page != last_page) {
      flush_write();
      last_page = page;
    }
    page_data[index] = data;
    page_mask[index] = true;
  }

  static void flush_write() {
    /*if (last write maybe pending) {
      // poll until last write complete (max 10ms)
      Bus::config_read();
      while (Bus::read_byte(last_address) != last_data) {}
    }*/
    BUS::config_write();
    for (uint8_t i = 0; i < PAGE_SIZE; ++i) { 
      if (page_mask[i]) {
        BUS::write_byte(last_page + i, page_data[i]);
        page_mask[i] = false;
      }
    }
    BUS::flush_write();
  }
};

template <typename BUS, uint8_t PAGE_SIZE>
typename BUS::DATA_TYPE PagedWrite<BUS, PAGE_SIZE>::page_data[PAGE_SIZE];

template <typename BUS, uint8_t PAGE_SIZE>
bool PagedWrite<BUS, PAGE_SIZE>::page_mask[PAGE_SIZE] = {}; // zero-initialized

template <typename BUS, uint8_t PAGE_SIZE>
typename BUS::ADDRESS_TYPE PagedWrite<BUS, PAGE_SIZE>::last_page = 0;

// Hook write_byte to measure IHX transfer rate
template <typename BUS>
struct MeasureImport : BUS {
  using typename BUS::DATA_TYPE;
  using typename BUS::ADDRESS_TYPE;

  static void write_byte(ADDRESS_TYPE address, DATA_TYPE data) {
    static auto t1 = millis();
    if ((address & 63) == 63) {
      auto t2 = millis();
      serialEx.println(t2 - t1);
      t1 = t2;
    }
  }
};

// Define interface for core::mon function templates
struct API : core::mon::Base<API> {
  using BUS = PagedWrite<Bus>;
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
    { "validate", core::mon::cmd_validate<API> },
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
