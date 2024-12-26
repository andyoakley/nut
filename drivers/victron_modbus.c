/*  victron_modbus.c - Driver for Victron Cerbo GX via modbus
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

#include "main.h"
#include "victron_modbus.h"
#include <modbus.h>
#include "timehead.h"
#include "nut_stdint.h"

#define DRIVER_NAME "NUT Victron Modbus driver"
#define DRIVER_VERSION  "0.01"

/* variables */
static modbus_t *mbctx = NULL;                             /* modbus memory context */
static int errcnt = 0;                                     /* modbus access error counter */

static char *device_mfr = DEVICE_MFR;                      /* device manufacturer */
static char *device_model = DEVICE_MODEL;                  /* device model */
static int rio_slave_id = MODBUS_SLAVE_ID;                 /* set device ID to default value */
static uint32_t mod_resp_to_s = MODRESP_TIMEOUT_s;         /* set the modbus response time out (s) */
static uint32_t mod_resp_to_us = MODRESP_TIMEOUT_us;       /* set the modbus response time out (us) */
static uint32_t mod_byte_to_s = MODBYTE_TIMEOUT_s;         /* set the modbus byte time out (us) */
static uint32_t mod_byte_to_us = MODBYTE_TIMEOUT_us;       /* set the modbus byte time out (us) */

static int last_soc = -1;		                           /* previous SOC, used to debounce spurious zero readings */

/* get config vars set by -x or defined in ups.conf driver section */
void get_config_vars(void);

/* create a new modbus context based on connection type (serial or TCP) */
modbus_t *modbus_new(const char *port);

/* reconnect upon communication error */
void modbus_reconnect(void);

/* modbus register read function */
int register_read(modbus_t *mb, int addr, regtype_t type, void *data);

/* driver description structure */
upsdrv_info_t upsdrv_info = {
	DRIVER_NAME,
	DRIVER_VERSION,
	"Andy Oakley <andy@andyoakley.com>",
	DRV_BETA,
	{NULL}
};

/*
 * driver functions
 */

/* initialize ups driver information */
void upsdrv_initinfo(void) {
	upsdebugx(2, "upsdrv_initinfo");

	/* set device information */
	dstate_setinfo("device.mfr", "%s", device_mfr);
	dstate_setinfo("device.model", "%s", device_model);
}

/* open serial connection and connect to modbus RIO */
void upsdrv_initups(void)
{
	int rval;
	upsdebugx(2, "upsdrv_initups");

	get_config_vars();

	/* open communication port */
	mbctx = modbus_new(device_path);
	if (mbctx == NULL) {
		fatalx(EXIT_FAILURE, "modbus_new_rtu: Unable to open communication port context");
	}

	/* set slave ID */
	rval = modbus_set_slave(mbctx, rio_slave_id);
	if (rval < 0) {
		modbus_free(mbctx);
		fatalx(EXIT_FAILURE, "modbus_set_slave: Invalid modbus slave ID %d", rio_slave_id);
	}

	/* connect to modbus device  */
	if (modbus_connect(mbctx) == -1) {
		modbus_free(mbctx);
		fatalx(EXIT_FAILURE, "modbus_connect: unable to connect: error(%s)", modbus_strerror(errno));
	}

	/* set modbus response timeout */
#if (defined NUT_MODBUS_TIMEOUT_ARG_sec_usec_uint32) || (defined NUT_MODBUS_TIMEOUT_ARG_sec_usec_uint32_cast_timeval_fields)
	rval = modbus_set_response_timeout(mbctx, mod_resp_to_s, mod_resp_to_us);
	if (rval < 0) {
		modbus_free(mbctx);
		fatalx(EXIT_FAILURE, "modbus_set_response_timeout: error(%s)", modbus_strerror(errno));
	}
#elif (defined NUT_MODBUS_TIMEOUT_ARG_timeval_numeric_fields)
	{
		/* Older libmodbus API (with timeval), and we have
		 * checked at configure time that we can put uint32_t
		 * into its fields. They are probably "long" on many
		 * systems as respectively time_t and suseconds_t -
		 * but that is not guaranteed; for more details see
		 * https://pubs.opengroup.org/onlinepubs/9699919799/basedefs/sys_time.h.html
		 */
		struct timeval to;
		memset(&to, 0, sizeof(struct timeval));
		to.tv_sec = mod_resp_to_s;
		to.tv_usec = mod_resp_to_us;
		/* void */ modbus_set_response_timeout(mbctx, &to);
	}
/* #elif (defined NUT_MODBUS_TIMEOUT_ARG_timeval) // some un-castable type in fields */
#else
# error "Can not use libmodbus API for timeouts"
#endif /* NUT_MODBUS_TIMEOUT_ARG_* */

	/* set modbus byte time out */
#if (defined NUT_MODBUS_TIMEOUT_ARG_sec_usec_uint32) || (defined NUT_MODBUS_TIMEOUT_ARG_sec_usec_uint32_cast_timeval_fields)
	rval = modbus_set_byte_timeout(mbctx, mod_byte_to_s, mod_byte_to_us);
	if (rval < 0) {
		modbus_free(mbctx);
		fatalx(EXIT_FAILURE, "modbus_set_byte_timeout: error(%s)", modbus_strerror(errno));
	}
#elif (defined NUT_MODBUS_TIMEOUT_ARG_timeval_numeric_fields)
	{   /* see comments above */
		struct timeval to;
		memset(&to, 0, sizeof(struct timeval));
		to.tv_sec = mod_byte_to_s;
		to.tv_usec = mod_byte_to_us;
		/* void */ modbus_set_byte_timeout(mbctx, &to);
	}
/* #elif (defined NUT_MODBUS_TIMEOUT_ARG_timeval) // some un-castable type in fields */
#endif /* NUT_MODBUS_TIMEOUT_ARG_* */
}

/* update UPS signal state */
void upsdrv_updateinfo(void)
{
	int rval;
	int reg_val;
	errcnt = 0;

	upsdebugx(2, "upsdrv_updateinfo");
	status_init();      /* initialize ups.status update */
	alarm_init();       /* initialize ups.alarm update */
	
	/* CHARGER STATUS */
	rval = register_read(mbctx, BATTERY_STATE, HOLDING, &reg_val);
	if (rval > -1) {
		upsdebugx(2, "battery state %d", reg_val);
		switch (reg_val) {
			case 0:
				dstate_setinfo("battery.charger.status", "resting");
				break;
			case 1:
				dstate_setinfo("battery.charger.status", "charging");
				status_set("CHRG");
				break;
			case 2:
				dstate_setinfo("battery.charger.status", "discharging");
				status_set("DISCHRG");
				break;
		}
	}
	else {
		upsdebugx(2, "Could not read charger status");
		errcnt++;
	}
	
	/* ACTIVE SOURCE */
	rval = register_read(mbctx, ACTIVE_SOURCE, HOLDING, &reg_val);
	if (rval > -1) {
		upsdebugx(2, "Active source %d", reg_val);
		switch (reg_val) {
			case 0:
				dstate_setinfo("input.source", "unknown");
				status_set("OL");
				break;
			case 1:
				dstate_setinfo("input.source", "grid");
				status_set("OL");
				break;
			case 2:
				dstate_setinfo("input.source", "generator");
				status_set("OL");
				break;
			case 3:
				dstate_setinfo("input.source", "shore");
				status_set("OL");
				break;
			case 240:
				dstate_setinfo("input.source", "battery");
				status_set("OB");
				break;
		}
	}
	else {
		upsdebugx(2, "Could not read active source");
		errcnt++;
	}

	/* STATE OF CHARGE */
	rval = register_read(mbctx, STATE_OF_CHARGE, HOLDING, &reg_val);
	if (rval > -1) {
		upsdebugx(2, "State of charge %d %%", reg_val);
		dstate_setinfo("battery.charge", "%d", reg_val);
		if (reg_val == 0 && last_soc > 20) {
			/* Occasional zero readings can be spurious.
			 * Mark this update as stale and wait for two in a row before triggering a low battery warning
			 */
			upsdebugx(2, "State of charge dropped to zero unexpectedly");
			errcnt++;
		}
		else if (reg_val < 20) {
			status_set("LB");
			alarm_set("Low Battery");
		}
		last_soc = reg_val;
	}
	else {
		upsdebugx(2, "Could not read state of charge");
		errcnt++;
	}

	/* check for communication errors */
	if (errcnt == 0) {
		alarm_commit();
		status_commit();
		dstate_dataok();
	} else {
		upsdebugx(2,"Communication errors: %d", errcnt);
		dstate_datastale();
	}
}


/* shutdown UPS */
void upsdrv_shutdown(void)
{
}

/* print driver usage info */
void upsdrv_help(void)
{
}

/* list flags and values that you want to receive via -x */
void upsdrv_makevartable(void)
{
	addvar(VAR_VALUE, "device_model", "device model");
}

/* close modbus connection and free modbus context allocated memory */
void upsdrv_cleanup(void)
{
	if (mbctx != NULL) {
		modbus_close(mbctx);
		modbus_free(mbctx);
	}
}


/*
 * driver support functions
 */

/* Read a modbus register */
int register_read(modbus_t *mb, int addr, regtype_t type, void *data)
{
	int rval = -1;

	/* register bit masks */
	uint16_t mask8 = 0x000F;
	uint16_t mask16 = 0x00FF;

	switch (type) {
		case COIL:
			rval = modbus_read_bits(mb, addr, 1, (uint8_t *)data);
			*(uint16_t *)data = *(uint16_t *)data & mask8;
			break;
		case INPUT_B:
			rval = modbus_read_input_bits(mb, addr, 1, (uint8_t *)data);
			*(uint16_t *)data = *(uint16_t *)data & mask8;
			break;
		case INPUT_R:
			rval = modbus_read_input_registers(mb, addr, 1, (uint16_t *)data);
			*(uint16_t *)data = *(uint16_t *)data & mask16;
			break;
		case HOLDING:
			rval = modbus_read_registers(mb, addr, 1, (uint16_t *)data);
			*(uint16_t *)data = *(uint16_t *)data & mask16;
			break;

#if (defined HAVE_PRAGMA_GCC_DIAGNOSTIC_PUSH_POP) && ( (defined HAVE_PRAGMA_GCC_DIAGNOSTIC_IGNORED_COVERED_SWITCH_DEFAULT) || (defined HAVE_PRAGMA_GCC_DIAGNOSTIC_IGNORED_UNREACHABLE_CODE) )
# pragma GCC diagnostic push
#endif
#ifdef HAVE_PRAGMA_GCC_DIAGNOSTIC_IGNORED_COVERED_SWITCH_DEFAULT
# pragma GCC diagnostic ignored "-Wcovered-switch-default"
#endif
#ifdef HAVE_PRAGMA_GCC_DIAGNOSTIC_IGNORED_UNREACHABLE_CODE
# pragma GCC diagnostic ignored "-Wunreachable-code"
#endif
/* Older CLANG (e.g. clang-3.4) seems to not support the GCC pragmas above */
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunreachable-code"
# pragma clang diagnostic ignored "-Wcovered-switch-default"
#endif
		/* All enum cases defined as of the time of coding
		 * have been covered above. Handle later definitions,
		 * memory corruptions and buggy inputs below...
		 */
		default:
			upsdebugx(2,"ERROR: register_read: invalid register type %d\n", type);
			break;
#ifdef __clang__
# pragma clang diagnostic pop
#endif
#if (defined HAVE_PRAGMA_GCC_DIAGNOSTIC_PUSH_POP) && ( (defined HAVE_PRAGMA_GCC_DIAGNOSTIC_IGNORED_COVERED_SWITCH_DEFAULT) || (defined HAVE_PRAGMA_GCC_DIAGNOSTIC_IGNORED_UNREACHABLE_CODE) )
# pragma GCC diagnostic pop
#endif
	}
	if (rval == -1) {
		upslogx(LOG_ERR,"ERROR:(%s) modbus_read: addr:0x%x, type:%8s, path:%s\n",
			modbus_strerror(errno),
			addr,
			(type == COIL) ? "COIL" :
			(type == INPUT_B) ? "INPUT_B" :
			(type == INPUT_R) ? "INPUT_R" : "HOLDING",
			device_path
		);

		/* on BROKEN PIPE error try to reconnect */
		if (errno == EPIPE || errno == EBADF || errno == EMBBADDATA || errno == ETIMEDOUT) {
			upsdebugx(2, "register_read: error(%s)", modbus_strerror(errno));
			modbus_reconnect();
		}
	}
	upsdebugx(3, "register addr: 0x%x, register type: %d read: %u",addr, type, *(unsigned int *)data);
	return rval;
}


/* get driver configuration parameters */
void get_config_vars(void)
{
	/* check if device model is set ang get the value */
	if (testvar("device_model")) {
		device_model = getval("device_model");
	}
	upsdebugx(2, "device_model %s", device_model);

	/* check if response time out (s) is set ang get the value */
	if (testvar("mod_resp_to_s")) {
		mod_resp_to_s = (uint32_t)strtol(getval("mod_resp_to_s"), NULL, 10);
	}
	upsdebugx(2, "mod_resp_to_s %d", mod_resp_to_s);

	/* check if response time out (us) is set ang get the value */
	if (testvar("mod_resp_to_us")) {
		mod_resp_to_us = (uint32_t) strtol(getval("mod_resp_to_us"), NULL, 10);
		if (mod_resp_to_us > 999999) {
			fatalx(EXIT_FAILURE, "get_config_vars: Invalid mod_resp_to_us %d", mod_resp_to_us);
		}
	}
	upsdebugx(2, "mod_resp_to_us %d", mod_resp_to_us);

	/* check if byte time out (s) is set ang get the value */
	if (testvar("mod_byte_to_s")) {
		mod_byte_to_s = (uint32_t)strtol(getval("mod_byte_to_s"), NULL, 10);
	}
	upsdebugx(2, "mod_byte_to_s %d", mod_byte_to_s);

	/* check if byte time out (us) is set ang get the value */
	if (testvar("mod_byte_to_us")) {
		mod_byte_to_us = (uint32_t) strtol(getval("mod_byte_to_us"), NULL, 10);
		if (mod_byte_to_us > 999999) {
			fatalx(EXIT_FAILURE, "get_config_vars: Invalid mod_byte_to_us %d", mod_byte_to_us);
		}
	}
	upsdebugx(2, "mod_byte_to_us %d", mod_byte_to_us);

}

/* create a new modbus context over TCP */
modbus_t *modbus_new(const char *port)
{
	modbus_t *mb;

	mb = modbus_new_tcp(port, 502);
	if (mb == NULL) {
		upslogx(LOG_ERR, "modbus_new_tcp: Unable to connect to %s\n", port);
	}
	return mb;
}

/* reconnect to modbus server upon connection error */
void modbus_reconnect(void)
{
	int rval;

	upsdebugx(2, "modbus_reconnect, trying to reconnect to modbus server");
	dstate_setinfo("driver.state", "reconnect.trying");

	/* clear current modbus context */
	modbus_close(mbctx);
	modbus_free(mbctx);

	/* open communication port */
	mbctx = modbus_new(device_path);
	if (mbctx == NULL) {
		fatalx(EXIT_FAILURE, "modbus_new_rtu: Unable to open communication port context");
	}

	/* set slave ID */
	rval = modbus_set_slave(mbctx, rio_slave_id);
	if (rval < 0) {
		modbus_free(mbctx);
		fatalx(EXIT_FAILURE, "modbus_set_slave: Invalid modbus slave ID %d", rio_slave_id);
	}

	/* connect to modbus device  */
	if (modbus_connect(mbctx) == -1 && errno != EINPROGRESS) {
		modbus_free(mbctx);
		fatalx(EXIT_FAILURE, "modbus_connect: unable to connect: %s", modbus_strerror(errno));
	}

	/* set modbus response timeout */
#if (defined NUT_MODBUS_TIMEOUT_ARG_sec_usec_uint32) || (defined NUT_MODBUS_TIMEOUT_ARG_sec_usec_uint32_cast_timeval_fields)
	rval = modbus_set_response_timeout(mbctx, mod_resp_to_s, mod_resp_to_us);
	if (rval < 0) {
		modbus_free(mbctx);
		fatalx(EXIT_FAILURE, "modbus_set_response_timeout: error(%s)", modbus_strerror(errno));
	}
#elif (defined NUT_MODBUS_TIMEOUT_ARG_timeval_numeric_fields)
	{   /* see comments above */
		struct timeval to;
		memset(&to, 0, sizeof(struct timeval));
		to.tv_sec = mod_resp_to_s;
		to.tv_usec = mod_resp_to_us;
		/* void */ modbus_set_response_timeout(mbctx, &to);
	}
/* #elif (defined NUT_MODBUS_TIMEOUT_ARG_timeval) // some un-castable type in fields */
#endif /* NUT_MODBUS_TIMEOUT_ARG_* */

	/* set modbus byte timeout */
#if (defined NUT_MODBUS_TIMEOUT_ARG_sec_usec_uint32) || (defined NUT_MODBUS_TIMEOUT_ARG_sec_usec_uint32_cast_timeval_fields)
	rval = modbus_set_byte_timeout(mbctx, mod_byte_to_s, mod_byte_to_us);
	if (rval < 0) {
		modbus_free(mbctx);
		fatalx(EXIT_FAILURE, "modbus_set_byte_timeout: error(%s)", modbus_strerror(errno));
	}
#elif (defined NUT_MODBUS_TIMEOUT_ARG_timeval_numeric_fields)
	{   /* see comments above */
		struct timeval to;
		memset(&to, 0, sizeof(struct timeval));
		to.tv_sec = mod_byte_to_s;
		to.tv_usec = mod_byte_to_us;
		/* void */ modbus_set_byte_timeout(mbctx, &to);
	}
/* #elif (defined NUT_MODBUS_TIMEOUT_ARG_timeval) // some un-castable type in fields */
#endif /* NUT_MODBUS_TIMEOUT_ARG_* */

	dstate_setinfo("driver.state", "quiet");
}
