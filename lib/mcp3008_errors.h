/*
 * mcp3008_errors.h:
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

#ifndef MCP3008_ERRORS_H__
#define MCP3008_ERRORS_H__

#define MCP3008_NOER        0
#define MCP3008_ENUP       -1
#define MCP3008_ENAC       -2 
#define MCP3008_ESOV       -3
#define MCP3008_EIOC       -4
#define MCP3008_EFUO       -5
#define MCP3008_EFUC       -6
#define MCP3008_EUAM       -7
#define MCP3008_ENNP       -8
#define MCP3008_EANA       -9

extern const char *mcp3008_errors_table[]; 

#endif // MCP3008_ERRORS_H__
