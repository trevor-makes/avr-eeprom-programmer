// Copyright (c) 2022 Trevor Makes

#include "debug.hpp"
#include "bus.hpp"

#include "core/io.hpp"
#include "core/io/bus.hpp"
#include "core/cli.hpp"
#include "core/mon.hpp"

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

//      * - D2 |         | x
//      * - D3 |         | x
// Data 4 - D4 |         | C5 - MSB Latch Enable (active high)
// Data 5 - D5 |         | C4 - LSB Latch Enable (active high)
// Data 6 - D6 | Arduino | C3 - Bus Read Enable (active low)
// Data 7 - D7 |   NANO  | C2 - Bus Write Enable (active low)
// Data 0 - B0 |         | C1 - *
// Data 1 - B1 |         | C0 - *
// Data 2 - B2 |   ___   | x
// Data 3 - B3 |  |USB|  | x
//      * - B4 |__|___|__| B5 - *
// * unused digital pins

// 8-bit data bus [D7 D6 D5 D4 B3 B2 B1 B0]
using DataPort = BitExtend<PortD::Mask<0xF0>, PortB::Mask<0x0F>>;

// Latch upper and lower bytes of 16-bit address from data port
using MSBLatch = ActiveHigh<PortC::Bit<5>>;
using LSBLatch = ActiveHigh<PortC::Bit<4>>;
using AddressMSB = Latch<DataPort, MSBLatch>;
using AddressLSB = Latch<DataPort, LSBLatch>;
using AddressPort = WordExtend<AddressMSB, AddressLSB>;

// Bus control lines
using ReadEnable = ActiveLow<PortC::Bit<3>>;
using WriteEnable = ActiveLow<PortC::Bit<2>>;

#else
#error Need to provide configuration for current platform. See __AVR_ATmega328P__ configuration above.
#endif

// See read and write timings in bus.hpp
using Bus = PortBus<AddressPort, DataPort, ReadEnable, WriteEnable>;

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

void setup() {
  // Establish serial connection with computer
  Serial.begin(9600);
  while (!Serial) {}
}

void set_baud(Args args) {
  CORE_EXPECT_UINT(API, uint32_t, baud, args, return)
  // NOTE in PlatformIO terminal, type `ctrl-t b` to enter matching baud rate
  Serial.print(F("Type: ctrl-t b "));
  Serial.println(baud);
  // https://forum.arduino.cc/t/change-baud-rate-at-runtime/368191
  Serial.flush();
  Serial.begin(baud);
  while (Serial.available()) Serial.read();
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

void loop() {
  static const core::cli::Command commands[] = {
    { F("baud"), set_baud },
    { F("hex"), core::mon::cmd_hex<API> },
    { F("set"), core::mon::cmd_set<API> },
    { F("fill"), core::mon::cmd_fill<API> },
    { F("move"), core::mon::cmd_move<API> },
    { F("export"), core::mon::cmd_export<API> },
    { F("import"), core::mon::cmd_import<API> },
    { F("verify"), core::mon::cmd_verify<API> },
    { F("erase"), erase },
    { F("unlock"), unlock },
    { F("lock"), lock },
    // Debug commands for measuring read/write throughput
    //{ "write", write_bus<API> },
    //{ "read", read_bus<API> },
    //{ "pagew", page_write<API> },
    //{ "pager", page_read<API> },
  };

  serialCli.prompt(commands);
}
