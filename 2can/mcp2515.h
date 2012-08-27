/******************************************************************************
* File:              mcp2515.h
* Author:            Kevin Day
* Date:              December, 2011
* Description:       
*                    
*                    
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
*                    
* Copyright (c) 2011 Kevin Day
* All rights reserved.
*******************************************************************************/
#ifndef _MCP2515_H
#define _MCP2515_H

extern task_t mcp_taskinfo;

task_t *mcp_task_create();

#endif
