// Copyright (c) 2022 Trevor Makes

#include "core/io.hpp"
#include "core/io/bus.hpp"
#include "core/cli.hpp"
#include "core/mon/format.hpp"

#include <Arduino.h>

using core::cli::Args;
core::serial::StreamEx serialEx(Serial);
core::cli::CLI<> serialCli(serialEx);

template <typename PIN>
struct ActiveLow {
  static inline void config_output() {
    PIN::config_output();
    disable();
  }
  static inline void enable() {
    PIN::clear();
  }
  static inline void disable() {
    PIN::set();
  }
};

template <typename PIN>
struct ActiveHigh {
  static inline void config_output() {
    PIN::config_output();
    disable();
  }
  static inline void enable() {
    PIN::set();
  }
  static inline void disable() {
    PIN::clear();
  }
};

#ifdef __AVR_ATmega328P__
CORE_PORT(B)
CORE_PORT(C)
CORE_PORT(D)

using MSBLatch = ActiveHigh<PortC::Bit<5>>;
using LSBLatch = ActiveHigh<PortC::Bit<4>>;
using ReadEnable = PortC::Bit<3>;
using WriteEnable = PortC::Bit<2>;

// [D7 D6 D5 D4 B3 B2 B1 B0]
using DataBus = core::io::BitExtend<PortD::Mask<0xF0>, PortB::Mask<0x0F>>;
#else
#error Need to provide configuration for current platform. See __AVR_ATmega328P__ configuration above.
#endif

// TODO move into core::io?
template <typename DATA, typename LATCH_ENABLE/*, typename OUTPUT_ENABLE*/>
struct Latch {
  using TYPE = typename DATA::TYPE;
  static inline void config_output() {
    // TODO enable OUTPUT_ENABLE
    // NOTE don't config DATA until write()
    LATCH_ENABLE::config_output();
  }
  static inline void config_input() {
    // TODO disable OUTPUT_ENABLE
    //LATCH_ENABLE::config_float_disable();
  }
  static inline void write(TYPE data) {
    auto config = DATA::save_config();
    DATA::config_output();
    DATA::write(data);
    LATCH_ENABLE::enable();
    LATCH_ENABLE::disable();
    DATA::restore_config(config);
  }
};

using AddressMSB = Latch<DataBus, MSBLatch>;
using AddressLSB = Latch<DataBus, LSBLatch>;
using AddressBus = core::io::WordExtend<AddressMSB, AddressLSB>;

using Control = core::io::Control<ReadEnable, WriteEnable>;
using Bus = core::io::Bus<AddressBus, DataBus, Control>;

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
  DataBus::config_output();
  DataBus::write(data);
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
