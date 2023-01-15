// Copyright (c) 2022 Trevor Makes

#pragma once

#include "core/util.hpp"
#include "core/io/bus.hpp"

#include <stdint.h>

template <typename ADDRESS, typename DATA, typename RE, typename WE>
struct PortBus : core::io::BaseBus {
  using ADDRESS_TYPE = typename ADDRESS::TYPE;
  using DATA_TYPE = typename DATA::TYPE;

  static void config_write() {
    ADDRESS::config_output();
    DATA::config_output();
    RE::config_output();
    WE::config_output();
  }

  static void write_bus(ADDRESS_TYPE addr, DATA_TYPE data) {
    ADDRESS::write(addr);
    WE::enable();
    DATA::write(data);
    WE::disable();
  }

  static void config_read() {
    ADDRESS::config_output();
    DATA::config_input();
    RE::config_output();
    WE::config_output();
  }

  static DATA_TYPE read_bus(ADDRESS_TYPE addr) {
    // Latch address from data port
    DATA::config_output();
    ADDRESS::write(addr);
    // Begin read sequence
    DATA::config_input();
    RE::enable();
    // AT28C64B tOE max (latency from output enable to output) is 70 ns
    // ATmega328p tpd max (port read latency) is 1.5 cycles (93.75 ns @ 16 MHz)
    // Empirically found 2 NOP cycles (125 ns) to add just enough delay
    __asm__ __volatile__ ("nop"); // delay 62.5 ns
    __asm__ __volatile__ ("nop"); // delay 62.5 ns
    const DATA_TYPE data = DATA::read();
    // End read sequence
    RE::disable();
    return data;
  }
};

// 28 series EEPROM chips support paged write where a full page may be written in one programming cycle
// This requires less than 150 us between writes, followed by up to 10 ms of inactivity while programming
// This adapter caches a page on the Arduino and flushes all writes at once to meet timing requirements
template <typename BUS, uint8_t PAGE_SIZE = 64>
struct PagedWrite : BUS {
  using typename BUS::DATA_TYPE;
  using typename BUS::ADDRESS_TYPE;

  static_assert(core::util::is_power_of_two(PAGE_SIZE), "");
  static constexpr const ADDRESS_TYPE INDEX_MASK = PAGE_SIZE - 1;
  static constexpr const ADDRESS_TYPE PAGE_MASK = ~INDEX_MASK;

  static DATA_TYPE page_data[PAGE_SIZE];
  static bool page_mask[PAGE_SIZE]; // TODO compact bitfield instead of bool array?
  static ADDRESS_TYPE cached_page;

  static void write_bus(ADDRESS_TYPE address, DATA_TYPE data) {
    // If last address was on a different page, flush the last page
    ADDRESS_TYPE page = address & PAGE_MASK;
    if (page != cached_page) {
      flush_write();
      cached_page = page;
    }

    // Cache data until page is flushed
    uint8_t index = address & INDEX_MASK;
    page_data[index] = data;
    page_mask[index] = true;
  }

  static ADDRESS_TYPE poll_address;
  static DATA_TYPE poll_data;
  static bool is_flushing;

  static void flush_write() {
    // Poll until last BUS write is complete (max 10ms)
    if (is_flushing) {
      BUS::config_read();
      auto start = millis();
      while (BUS::read_bus(poll_address) != poll_data) {
        // Break after 10 ms in case polling failed (data protection enabled)
        if (millis() - start >= 10) break;
      }
      is_flushing = false;
    }

    // Flush cached page data to BUS
    BUS::config_write();
    for (uint8_t i = 0; i < PAGE_SIZE; ++i) { 
      if (page_mask[i]) {
        ADDRESS_TYPE address = cached_page + i;
        DATA_TYPE data = page_data[i];
        BUS::write_bus(address, data);
        page_mask[i] = false;
        // Save last written data so next flush can poll until EEPROM is ready
        poll_address = address;
        poll_data = data;
        is_flushing = true;
      }
    }

    // Propagate flush to parent type in case it does something
    BUS::flush_write();
  }
};

template <typename BUS, uint8_t PAGE_SIZE>
typename BUS::DATA_TYPE PagedWrite<BUS, PAGE_SIZE>::page_data[PAGE_SIZE];

template <typename BUS, uint8_t PAGE_SIZE>
bool PagedWrite<BUS, PAGE_SIZE>::page_mask[PAGE_SIZE] = {}; // zero-initialized

template <typename BUS, uint8_t PAGE_SIZE>
typename BUS::ADDRESS_TYPE PagedWrite<BUS, PAGE_SIZE>::cached_page;

template <typename BUS, uint8_t PAGE_SIZE>
typename BUS::ADDRESS_TYPE PagedWrite<BUS, PAGE_SIZE>::poll_address;

template <typename BUS, uint8_t PAGE_SIZE>
typename BUS::DATA_TYPE PagedWrite<BUS, PAGE_SIZE>::poll_data;

template <typename BUS, uint8_t PAGE_SIZE>
bool PagedWrite<BUS, PAGE_SIZE>::is_flushing = false;
