##############################################################################
#  Makefile:
#      Copyright (c) 2020 Ricardo Bustos
#
#    mcp3008 raspberry pi 3+  library is a free software: you can redistribute 
#    it and/or modify it under the terms of the GNU Lesser General Public 
#    License as published by the Free Software Foundation, either version 
#    3 of the License, or (at your option) any later version.
#
#    mcp3008 raspberry pi 3+ library is distributed in the hope that it 
#    will be useful, but WITHOUT ANY WARRANTY; without even the implied 
#    warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
#    See the GNU Lesser General Public License for more details.
#
###############################################################################

CC = gcc
LIB = mcp3008_raspi
LIB_DIR = ../lib 
CFLAGS = -L $(LIB_DIR) -Wl,-rpath=$(LIB_DIR) -Wall
LFLAGS = -l $(LIB)
#DEBUG = -g
RM = rm -f

TARGET_BIN = mcp3008_raspi_test.bin
SRCS = mcp3008_raspi_test.c

OBJS = $(SRCS:.c=.o)


$(TARGET_BIN): $(SRCS)
	$(CC) $< $(CFLAGS) -o $@ $(LFLAGS)
#	$(CC) $< $(CFLAGS) $(DEBUG) -o $@ $(LFLAGS)

clean:
	${RM} $(TARGET_BIN)
