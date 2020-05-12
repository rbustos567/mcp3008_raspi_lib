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

VERSION = $(shell cat ../VERSION)

CC = gcc
LIB = lib
CFLAGS = -std=c99 -Wall -Wextra -fPIC
#DEBUG = -g
LDFLAGS = -shared
LN = ln
RM = rm -f

LIB_NAME = mcp3008_raspi
TARGET_LIB  = $(LIB)$(LIB_NAME).so.$(VERSION)
SYM_LIB = $(LIB)$(LIB_NAME).so 

DEPS = mcp3008_raspi.h mcp3008_errors.h
OBJS = mcp3008_raspi.o mcp3008_errors.o

.PHONY: all

all: ${TARGET_LIB}
	$(LN) -s $(TARGET_LIB) $(SYM_LIB) 

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)
#	$(CC) -c -o $@ $< $(CFLAGS) $(DEBUG)

$(TARGET_LIB): $(OBJS)
	$(CC) -o $@ $^ $(CFLAGS) $(LDFLAGS)
#	$(CC) -o $@ $^ $(CFLAGS) $(DEBUG) $(LDFLAGS)


.PHONY: clean

clean:
	${RM} $(SYM_LIB) ${TARGET_LIB} ${OBJS}
