/**
 ******************************************************************************
 * @file    i2c_comm.h
 * @brief   I2C1 slave register-map interface to the master board. Replaces
 *          the original driver's BaSyTec ASCII protocol (dropped per
 *          request) with a compact binary register map, since this is a new
 *          requirement rather than a port of existing logic.
 *
 * Protocol
 * --------
 * PM acts as an I2C slave at I2C_SLAVE_ADDR_7BIT (see i2c_comm.c - pick/
 * confirm the actual address expected by the master, then either change the
 * #define or set it as "Own Address 1" on I2C1 in the .ioc, whichever this
 * project prefers to keep as the source of truth).
 *
 *   Master write:  [register address][data bytes...]  (Sr/P)
 *     - writes the given bytes starting at that register address; a
 *       write of just the address (no data bytes) only moves the read
 *       pointer, it does not modify anything.
 *   Master read:   [register address](Sr)[data bytes...] (P)
 *     - i.e. a write of the address, repeated-START, then a read: returns
 *       the register map starting at that address.
 *
 * All multi-byte fields are little-endian, signed unless noted. Registers
 * marked RO ignore writes.
 *
 *   Addr  Name        R/W  Type    Scale / meaning
 *   0x00  STATUS       RO  uint8   bit0 running, bit1 error, bit2 direction (0=heating,1=cooling)
 *   0x01  CONTROL      RW  uint8   write: bit0=start, bit1=stop, bit2=clear error (self-clearing, reads back 0)
 *   0x02  ERROR_CODE   RO  uint8   0 none, 1 startup overcurrent, 2 runtime overcurrent, 3 sanity error
 *   0x03  CHANNEL_SEL  RW  uint8   0 external, 1..3 = TEMP_CHANNEL_1..3 (selects PID input)
 *   0x04  T_SET        RW  int16   deg C x100
 *   0x06  T_EXT        RW  int16   deg C x100 (used while CHANNEL_SEL = external)
 *   0x08  T_ACTUAL     RO  int16   deg C x100 (channel currently feeding the PID)
 *   0x0A  T_CH1        RO  int16   deg C x100
 *   0x0C  T_CH2        RO  int16   deg C x100
 *   0x0E  T_CH3        RO  int16   deg C x100
 *   0x10  KP           RW  uint16  x1000
 *   0x12  KI           RW  uint16  x1000
 *   0x14  KD           RW  uint16  x1000
 *   0x16  I_LIMIT      RW  uint16  mA
 *   0x18  I_ACTUAL     RO  uint16  mA (always >= 0, see current_sensor.h)
 *   0x1A  DUTY_CYCLE   RO  uint16  x1000 (0..1000 = 0..100.0%)
 *
 * Register map is 0x1C (28) bytes total.
 ******************************************************************************
 */
#ifndef I2C_COMM_H
#define I2C_COMM_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* One-time setup: sets the slave address and arms address-match listening.
 * Call once from main() before the main loop, after MX_I2C1_Init(). */
void I2cComm_Init(void);

/* Refreshes the read-only telemetry registers from the live control/sensor
 * state and applies any register writes that arrived since the last call.
 * Call once per main loop iteration. Deliberately does the actual work here
 * (main-loop context) rather than inside the I2C ISR callbacks, which only
 * copy bytes and set flags. */
void I2cComm_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* I2C_COMM_H */
