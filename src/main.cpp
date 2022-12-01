// Copyright (c) 2022 Trevor Makes

#include "core/io.hpp"
#include "core/io/bus.hpp"
#include "core/cli.hpp"
#include "core/mon/format.hpp"

#include <Arduino.h>

using core::cli::Args;
core::serial::StreamEx serialEx(Serial);
core::cli::CLI<> serialCli(serialEx);

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
using DataPort = core::io::BitExtend<PortD::Mask<0xF0>, PortB::Mask<0x0F>>;

// Latch upper and lower bytes of address from data port
using MSBLatch = core::io::ActiveHigh<PortC::Bit<5>>;
using LSBLatch = core::io::ActiveHigh<PortC::Bit<4>>;
using AddressMSB = core::io::Latch<DataPort, MSBLatch>;
using AddressLSB = core::io::Latch<DataPort, LSBLatch>;
using AddressPort = core::io::WordExtend<AddressMSB, AddressLSB>;

// Bus control lines
using ReadEnable = core::io::ActiveLow<PortC::Bit<3>>;
using WriteEnable = core::io::ActiveLow<PortC::Bit<2>>;
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

void loop() {
  static const core::cli::Command commands[] = {
    { "lsb", set_lsb },
    { "msb", set_msb },
    { "data", set_data },
    { "write", write_bus },
    { "read", read_bus },
  };

  serialCli.run_once(commands);
}

void set_lsb(Args args) {
  uint8_t data = 0;
  core::mon::parse_unsigned(data, args.next());
  AddressLSB::config_output();
  AddressLSB::write(data);
}

void set_msb(Args args) {
  uint8_t data = 0;
  core::mon::parse_unsigned(data, args.next());
  AddressMSB::config_output();
  AddressMSB::write(data);
}

void set_data(Args args) {
  uint8_t data = 0;
  core::mon::parse_unsigned(data, args.next());
  DataPort::config_output();
  DataPort::write(data);
}

void write_bus(Args args) {
  uint16_t addr = 0;
  uint8_t data = 0;
  core::mon::parse_unsigned(addr, args.next());
  core::mon::parse_unsigned(data, args.next());
  Bus::config_write();
  Bus::write_byte(addr, data);
}

void read_bus(Args args) {
  uint16_t addr = 0;
  core::mon::parse_unsigned(addr, args.next());
  Bus::config_read();
  uint8_t data = Bus::read_byte(addr);
  serialEx.println(data, HEX);
}
