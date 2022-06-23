// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include "simple_system_common.h"

int main(int argc, char **argv) {
  pcount_enable(0);
  pcount_reset();
  pcount_enable(1);
  puts("DBG \n");
  puts("TSYS LOADED ME HERE :/\n");
  puthex(0xDEADBEEF);
  putchar('\n');
  puthex(0xBAADF00D);
  putchar('\n');

  pcount_enable(0);

  // Enable periodic timer interrupt
  // (the actual timebase is a bit meaningless in simulation)
  timer_enable(2000);

  uint64_t last_elapsed_time = get_elapsed_time();
  int count=0;
  while (last_elapsed_time <= 100) {
    uint64_t cur_time = get_elapsed_time();
    if (cur_time != last_elapsed_time) {
      last_elapsed_time = cur_time;
      puthex(count++);
      putchar('\n');
      // if (last_elapsed_time & 1) {
      //   puts("Tick!\n");
      // } else {
      //   puts("Tock!\n");
      // }
    }
    asm volatile("wfi");
  }

  return 0;
}