// Copyright (c) 2022 Trevor Makes

#include "debug.hpp"
#include "bus.hpp"

#include "core/io.hpp"
#include "core/io/bus.hpp"
#include "core/cli.hpp"
#include "core/mon.hpp"

#include <Arduino.h>

using core::io::ActiveLow;
using core::io::ActiveHigh;
using core::io::BitExtend;
using core::io::WordExtend;
using core::io::Latch;

// Arduino Nano and Uno
#ifdef __AVR_ATmega328P__

// Create Port* wrappers around AVR ports B, C, D
CORE_PORT(B)
CORE_PORT(C)
CORE_PORT(D)

// LSB Latch - D2 |         | x
// MSB Latch - D3 |         | x
//    Data 4 - D4 |         | C5 - Write Enable (active low)
//    Data 5 - D5 |         | C4 - Mode Select (input)
//    Data 6 - D6 | Arduino | C3 - Read Enable (active low)
//    Data 7 - D7 |   NANO  | C2 - Chip Enable (active low)
//    Data 0 - B0 |         | C1 - *
//    Data 1 - B1 |         | C0 - *
//    Data 2 - B2 |   ___   | x
//    Data 3 - B3 |  |USB|  | x
//         * - B4 |__|___|__| B5 - *
// * unused digital pins

// 8-bit data bus [D7 D6 D5 D4 B3 B2 B1 B0]
using DataPort = BitExtend<PortD::Mask<0xF0>, PortB::Mask<0x0F>>;

// Latch upper and lower bytes of 16-bit address from data port
using MSBLatch = ActiveHigh<PortD::Bit<3>>;
using LSBLatch = ActiveHigh<PortD::Bit<2>>;
using AddressMSB = Latch<DataPort, MSBLatch>;
using AddressLSB = Latch<DataPort, LSBLatch>;
using AddressPort = WordExtend<AddressMSB, AddressLSB>;

// Alternate data port for 2364 address MSB latch [- - - B3 C2 B2 B1 B0]
// From the 28C pinout, 2364 ROM swaps A12 for A11 and A11 for /CE (C2)
// TODO make variadic templates for BitExtend and Overlay
using DataMSB_ROM = BitExtend<PortB::Bit<3>, PortC::Bit<2>, PortB::Mask<0x07>>;
using AddressMSB_ROM = Latch<DataMSB_ROM, MSBLatch>;
using AddressPort_ROM = WordExtend<AddressMSB_ROM, AddressLSB>;

// Bus control lines
using ChipEnable = ActiveLow<PortC::Bit<2>>;
using ReadEnable = ActiveLow<PortC::Bit<3>>;
using ModeSelect = PortC::Bit<4>;
using WriteEnable = ActiveLow<PortC::Bit<5>>;

#else
#error Need to provide configuration for current platform. See __AVR_ATmega328P__ configuration above.
#endif

// See read and write timings in bus.hpp
using Bus = PortBus<AddressPort, DataPort, ReadEnable, WriteEnable>;
using Bus_ROM = PortBus<AddressPort_ROM, DataPort, ReadEnable, WriteEnable>;

using core::serial::StreamEx;
using CLI = core::cli::CLI<>;
using core::cli::Args;

// Create command line interface around Arduino Serial
StreamEx serialEx(Serial);
CLI serialCli(serialEx);

// Define interface for core::mon function templates
struct API : core::mon::Base<API> {
  static StreamEx& get_stream() { return serialEx; }
  static CLI& get_cli() { return serialCli; }
  // Wrap bus with paged write adapter to meet EEPROM write timings
  using BUS = PagedWrite<Bus>;
};

struct API_ROM : core::mon::Base<API_ROM> {
  static StreamEx& get_stream() { return serialEx; }
  static CLI& get_cli() { return serialCli; }
  // Wrap bus with paged write adapter to meet EEPROM write timings
  using BUS = Bus_ROM;
};

void set_baud(Args args) {
  CORE_EXPECT_UINT(API, uint32_t, baud, args, return)
  // https://forum.arduino.cc/t/change-baud-rate-at-runtime/368191
  Serial.flush();
  Serial.begin(baud);
  while (Serial.available()) Serial.read();
  // NOTE in PlatformIO terminal, type `ctrl-t b` to enter matching baud rate
}

void erase(Args) {
  // Special command sequence to erase all bytes to FF
  // Writes need to be sequential; don't use paged write
  Bus::config_write();
  Bus::write_bus(0x5555, 0xAA);
  Bus::write_bus(0xAAAA, 0x55);
  Bus::write_bus(0x5555, 0x80);
  Bus::write_bus(0x5555, 0xAA);
  Bus::write_bus(0xAAAA, 0x55);
  Bus::write_bus(0x5555, 0x10);
  delay(20); // erase takes up to 20 ms
}

void unlock(Args) {
  // Special command sequence to disable software data protection
  // Writes need to be sequential; don't use paged write
  Bus::config_write();
  Bus::write_bus(0x5555, 0xAA);
  Bus::write_bus(0xAAAA, 0x55);
  Bus::write_bus(0x5555, 0x80);
  Bus::write_bus(0x5555, 0xAA);
  Bus::write_bus(0xAAAA, 0x55);
  Bus::write_bus(0x5555, 0x20);
  delay(10); // unlock takes up to 10 ms
}

void lock(Args) {
  // Special command sequence to enable software data protection
  // Writes need to be sequential; don't use paged write
  Bus::config_write();
  Bus::write_bus(0x5555, 0xAA);
  Bus::write_bus(0xAAAA, 0x55);
  Bus::write_bus(0x5555, 0xA0);
  delay(10); // lock takes up to 10 ms
}

template <typename BUS>
void debug_bus(Args args) {
  CORE_EXPECT_UINT(API, uint16_t, addr, args, return)
  CORE_EXPECT_UINT(API, uint8_t, data, args, return)
  BUS::config_write();
  BUS::write_bus(addr, data);
}

void loop_eeprom() {
  static const core::cli::Command commands[] = {
    { "baud", set_baud },
    { "hex", core::mon::cmd_hex<API> },
    { "set", core::mon::cmd_set<API> },
    { "fill", core::mon::cmd_fill<API> },
    { "move", core::mon::cmd_move<API> },
    { "export", core::mon::cmd_export<API> },
    { "import", core::mon::cmd_import<API> },
    { "verify", core::mon::cmd_verify<API> },
    { "erase", erase },
    { "unlock", unlock },
    { "lock", lock },
    { "debug", debug_bus<Bus> },
  };

  serialCli.run_once(commands);
}

void loop_rom() {
  static const core::cli::Command commands[] = {
    { "baud", set_baud },
    { "hex", core::mon::cmd_hex<API_ROM> },
    { "export", core::mon::cmd_export<API_ROM> },
    { "verify", core::mon::cmd_verify<API_ROM> },
    { "debug", debug_bus<Bus_ROM> },
  };

  serialCli.run_once(commands);
}

void (*loop_fn)();

void setup() {
  // Establish serial connection with computer
  Serial.begin(9600);
  while (!Serial) {}

  ModeSelect::config_input();
  AddressMSB::config_output();
  AddressMSB::write(0); // set A13 low

  // Read if jumper set to Vcc or A13 (low)
  if (ModeSelect::is_set()) {
    Serial.println("2364 ROM mode");
    loop_fn = loop_rom;
  } else {
    // TODO set A13 high and check ModeSelect now set
    Serial.println("28C EEPROM mode");
    ChipEnable::config_output();
    ChipEnable::enable();
    loop_fn = loop_eeprom;
  }
}

void loop() {
  loop_fn();
}
