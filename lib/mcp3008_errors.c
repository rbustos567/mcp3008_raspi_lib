/*
 * mcp3008_errors.c:
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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

const char *mcp3008_errors_table[] = {
    "No error",
    "Error: Pointer is null",
    "Error: No access to file",
    "Error: Buffer size exceed",
    "Error: Returned error from ioctl() system call",
    "Error: File descriptor unopened",
    "Error: Unable to close file descriptor",
    "Error: Unable to allocate dynamic memory",
    "Error: Pointer is not null",
    "Error: Argument value not allowed"
};
