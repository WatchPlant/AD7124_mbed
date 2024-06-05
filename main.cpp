/*
 * Copyright (c) 2006-2020 Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 */
#include "DigitalInOut.h"
#include "mbed.h"
#include "hal/include/hal/spi_api.h"
#include "mstd_iterator"
#include <cstdint>
#include <cstdio>
#include <ad7124-defs.h>
#include <utils.h>
#include <AD7124.h>
#include "mbed_trace.h"


DigitalInOut LED = LED1;

int main()
{
    //char msg[] = "Echoes back to the screen anything you type\n";
    //char *buff = new char[1];
    //pc1.write(msg, sizeof(msg));


    mbed_trace_init();
   // TRACE_ACTIVE_LEVEL_ALL, TRACE_ACTIVE_LEVEL_DEBUG (same as ALL), TRACE_ACTIVE_LEVEL_INFO, TRACE_ACTIVE_LEVEL_WARN, 
   //TRACE_ACTIVE_LEVEL_ERROR, TRACE_ACTIVE_LEVEL_CMD and TRACE_LEVEL_NONE.",
    mbed_trace_config_set(TRACE_ACTIVE_LEVEL_ALL);

     // Chip must be deselected
    cs = 1;
    AD7124 adc;  // mosi, miso, sclk, cs
    // Setup the spi for 8 bit data, high steady state clock,
    // second edge capture, with a 1MHz clock rate
    spi.format(8, 3);
    spi.frequency(800000);

    bool trace = true;
    char read = 1;
    char write = 0;

    adc.init(true, true, AD7124_CH0_MAP_REG);

    adc.read_thread();
    
}