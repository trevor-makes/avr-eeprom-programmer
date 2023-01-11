// Copyright (c) 2022 Trevor Makes

#pragma once

#include "core/cli.hpp"
#include "core/mon/format.hpp"

template <typename API, typename LSB>
void set_lsb(core::cli::Args args) {
  CORE_EXPECT_UINT(API, uint8_t, lsb, args, return)
  LSB::config_output();
  LSB::write(lsb);
}

template <typename API, typename MSB>
void set_msb(core::cli::Args args) {
  CORE_EXPECT_UINT(API, uint8_t, msb, args, return)
  MSB::config_output();
  MSB::write(msb);
}

template <typename API, typename DATA>
void set_data(core::cli::Args args) {
  CORE_EXPECT_UINT(API, uint8_t, data, args, return)
  DATA::config_output();
  DATA::write(data);
}

template <typename API>
void write_bus(core::cli::Args args) {
  using BUS = typename API::BUS;
  using DATA = typename BUS::DATA_TYPE;
  using ADDRESS = typename BUS::ADDRESS_TYPE;

  CORE_EXPECT_UINT(API, ADDRESS, addr, args, return)
  CORE_EXPECT_UINT(API, DATA, data, args, return)
  BUS::config_write();

  auto t1 = micros();
  BUS::write_bus(addr, data);
  BUS::flush_write();
  auto t2 = micros();
  BUS::flush_write();
  auto t3 = micros();

  API::get_stream().println(t2 - t1); // write_bus
  API::get_stream().println(t3 - t2); // flush_write
  API::get_stream().println(t3 - t1); // write_bus + flush_write
}

template <typename API>
void read_bus(core::cli::Args args) {
  using BUS = typename API::BUS;
  using ADDRESS = typename BUS::ADDRESS_TYPE;

  CORE_EXPECT_UINT(API, ADDRESS, addr, args, return)
  BUS::config_read();

  auto t1 = micros();
  uint8_t data = BUS::read_bus(addr);
  auto t2 = micros();

  API::get_stream().println(data, HEX);
  API::get_stream().println(t2 - t1); // read_bus
}

template <typename API, uint8_t PAGE_SIZE = 64>
void page_write(core::cli::Args args) {
  using BUS = typename API::BUS;
  using ADDRESS = typename BUS::ADDRESS_TYPE;

  CORE_EXPECT_UINT(API, ADDRESS, page, args, return)
  BUS::config_write();

  // Mask off just the page from the given address
  const ADDRESS INDEX_MASK = PAGE_SIZE - 1;
  const ADDRESS PAGE_MASK = ~INDEX_MASK;
  page &= PAGE_MASK;

  // Measure time to load page data
  auto t1 = micros();
  for (uint8_t index = 0; index < PAGE_SIZE; ++index) {
    BUS::write_bus(page + index, index);
  }
  BUS::flush_write();
  auto t2 = micros();

  // Measure data polling to completion
  BUS::flush_write();
  auto t3 = micros();

  API::get_stream().println(t2 - t1); // write_bus x PAGE_SIZE
  API::get_stream().println(t3 - t2); // flush_write
  API::get_stream().println(t3 - t1); // write_bus + flush_write
}

template <typename API, uint8_t PAGE_SIZE = 64>
void page_read(core::cli::Args args) {
  using BUS = typename API::BUS;
  using ADDRESS = typename BUS::ADDRESS_TYPE;

  CORE_OPTION_UINT(API, ADDRESS, page, 0, args, return)
  BUS::config_read();

  // Mask off just the page from the given address
  const ADDRESS INDEX_MASK = PAGE_SIZE - 1;
  const ADDRESS PAGE_MASK = ~INDEX_MASK;
  page &= PAGE_MASK;

  // Measure time to load page data
  uint8_t checksum = 0;
  auto t1 = micros();
  for (uint8_t index = 0; index < PAGE_SIZE; ++index) {
    checksum += BUS::read_bus(page + index);
  }
  auto t2 = micros();

  API::get_stream().println(checksum, HEX);
  API::get_stream().println(t2 - t1); // read_bus x PAGE_SIZE
}
