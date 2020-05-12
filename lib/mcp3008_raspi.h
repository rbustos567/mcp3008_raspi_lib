/*
 * mcp3008_raspi.h:
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
#include "mcp3008_errors.h"

#ifndef MCP3008_RASPI_H__
#define MCP3008_RASPI_H__

#define MCP3008_LIB_NAM     "libmcp3008_raspi"

#define MCP3008_DEBUG       0  // Set to enable debug

#define MCP3008_INI_HEX     0x01
#define MCP3008_CLR_HEX     0x00
#define MCP3008_S_MODE_HEX  0x80

#define MCP3008_MAX_CHN     7

#define MCP3008_SIZE_BUF    4
#define MCP3008_MAX_PATH    64
#define MCP3008_DEF_DEV     "/dev/spidev0.0"

#define MCP3008_SHIFT_4_B   4
#define MCP3008_SHIFT_8_B   8

#define MCP3008_DEF_SPI_M   0
#define MCP3008_DEF_BITS    8
#define MCP3008_DEF_SPEED   1000000
#define MCP3008_DEF_DELAY   0
#define MCP3008_DEF_CS_CH   0
#define MCP3008_DEF_TX_NB   0
#define MCP3008_DEF_RX_NB   0
#define MCP3008_DEF_PAD     0

#define MCP3008_MAX_RAW     262143 // Max value: 0x03FFFF
#define MCP3008_MAX_V_REF   3.3

#if defined(MCP3008_DEBUG) && (MCP3008_DEBUG > 0)
    #define debug_print(fmt, ...) do { fprintf(stderr, "DEBUG: %s:%s:%d:%s() ret=[%d]\n" fmt, \
                                 MCP3008_LIB_NAM, __FILE__, __LINE__, __func__,  __VA_ARGS__); } while (0)
#else
    #define debug_print(fmt, ...) // Do nothing
#endif

struct spidev_msg_conf {
    uint8_t mode;
    uint8_t bits;
    uint32_t speed;
    uint16_t delay;
    unsigned cs_change;
};

struct mcp3008_config {
    char *dev_path;
    struct spidev_msg_conf *spi_msg_c;
};

struct mcp3008_ioctl_transfer {
    int fd;
    struct spi_ioc_transfer *tr_msg;
    uint8_t s_tx[MCP3008_SIZE_BUF];
    uint8_t s_rx[MCP3008_SIZE_BUF];
};

extern int init_mcp3008_device(void);
extern int end_mcp3008_device(void);
extern int set_mcp3008_device_file(const char *dev);
extern int set_mcp3008_mode(uint8_t mode);
extern int set_mcp3008_bits_per_word(uint8_t bits);
extern int set_mcp3008_speed(uint32_t speed);
extern int set_mcp3008_delay(uint16_t delay);
extern int set_mcp3008_cs_change(bool set);
extern int get_raw_input_from_mcp3008_channel(unsigned short, bool, unsigned int *);
extern int get_volt_input_from_mcp3008_channel(unsigned short, bool, float *);
extern const char *get_mcp3008_error(int);

#endif // MCP3008_RASPI_H__
