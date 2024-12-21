/*  generic_modbus.h - Driver for generic UPS connected via modbus RIO
 *
 *  Copyright (C)
 *    2021 Dimitris Economou <dimitris.s.economou@gmail.com>
 *    2024 Andy Oakley <andy@andyoakley.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#ifndef NUT_VICTRON_MODBUS_H
#define NUT_VICTRON_MODBUS_H

/* UPS device details */
#define DEVICE_MFR  "Victron"
#define DEVICE_MODEL ""

/*
 * modbus response and byte timeouts
 * us: 1 - 999999
 */
#define MODRESP_TIMEOUT_s 0
#define MODRESP_TIMEOUT_us 200000
#define MODBYTE_TIMEOUT_s 0
#define MODBYTE_TIMEOUT_us 50000

/* modbus access parameters */
#define MODBUS_SLAVE_ID 100

/* definition of register type */
enum regtype {
	COIL = 0,
	INPUT_B,
	INPUT_R,
	HOLDING
};
typedef enum regtype regtype_t;

enum registers {
	AC_L1_W = 817,
	ACTIVE_SOURCE = 826,
	STATE_OF_CHARGE = 843,
	BATTERY_STATE = 844
};
typedef enum registers registers_t;



#endif /* NUT_VICTRON_MODBUS_H */
