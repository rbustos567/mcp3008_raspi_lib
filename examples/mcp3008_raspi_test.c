/*
 * mcp3008_raspi_test.c:
 *      Copyright (c) 2020 Ricardo Bustos
 ***********************************************************************
 *
 *    mcp3008 raspberry pi 3+  library is a free software: you can redistribute 
 *    it and/or modify it under the terms of the GNU Lesser General Public 
 *    License as published by the Free Software Foundation, either version 
 *    3 of the License, or (at your option) any later version.
 *
 *    mcp3008 raspberry pi 3+ library is distributed in the hope that it 
 *    will be useful, but WITHOUT ANY WARRANTY; without even the implied 
 *    warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
 *    See the GNU Lesser General Public License for more details.
 *
 ***********************************************************************
 */

#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
#include <stdbool.h>
#include "../lib/mcp3008_raspi.h"

int main (void) {

    short channel = 2; // channel 2 selected
    bool single = true; // single-ended mode 
    unsigned int raw; // raw output 
    float volts; // voltage output 
    int ret;

    printf("Initializing mcp3008 lib...\n");
    ret = init_mcp3008_device();

    if (ret < MCP3008_NOER) {
        printf("%s\n", get_mcp3008_error(ret));
        exit(0);
    }
	
    // Obtaning raw data from channel 0 of mcp3008 chip
    ret = get_raw_input_from_mcp3008_channel(channel, single, &raw);

    if (ret < MCP3008_NOER) {
        printf("%s\n", get_mcp3008_error(ret));
	exit(0);
    }

    printf("CHANNEL %d: output raw = %d\n", channel, raw); 
            
    // Obtaning voltage level from channel 0 of mcp3008 chip 
    ret = get_volt_input_from_mcp3008_channel(channel, single, &volts);
	
    if (ret < MCP3008_NOER) {
        printf("%s\n", get_mcp3008_error(ret));
        exit(0);
    }
 
    printf("CHANNEL %d: output volts  = %f\n", channel, volts);

    printf("Ending mcp3008 lib...\n");
    ret = end_mcp3008_device();
    
    if (ret < MCP3008_NOER) {
        printf("%s\n", get_mcp3008_error(ret));
        exit(0);
    }

    return 0; 
}
