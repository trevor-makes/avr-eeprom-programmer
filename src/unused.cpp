// Copyright (c) 2022 Trevor Makes

#if 0

using core::cli::Args;
using core::mon::parse_unsigned;

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
  Bus::write_data(addr, data);

  // Measure data polling to completion
  Bus::config_read();
  auto t1 = micros();
  while (Bus::read_data(addr) != data) {}
  auto t2 = micros();
  serialEx.println(t2 - t1);
}

void read_bus(Args args) {
  uint16_t addr = 0;
  parse_unsigned(addr, args.next());
  Bus::config_read();
  uint8_t data = Bus::read_data(addr);
  serialEx.println(data, HEX);
}

// Measure EEPROM page load/write timing
void measure_page_write(Args) {
  Bus::config_write();

  // Measure 64 byte page load
  auto t1 = micros();
  for (uint16_t addr = 0x140; addr < 0x180; ++addr) {
    Bus::write_data(addr, addr);
  }
  auto t2 = micros();

  // Measure data polling to completion
  Bus::config_read();
  while (Bus::read_data(0x17F) != 0x7F) {}
  auto t3 = micros();

  serialEx.println(t2 - t1);
  serialEx.println(t3 - t2);
}

#endif
