/*
 * Copyright(C) 2011 FUJITSU LIMITED
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011-2012
/*----------------------------------------------------------------------------*/
#ifndef _COMPASS_H
#define _COMPASS_H
#ifdef __KERNEL__
#define COMPASS_DEBUG
#else /* __KERNEL__ */
#include <stdint.h>
#include <sys/time.h>
#endif /* __KERNEL__ */
#include <linux/ioctl.h>

/*--------------------------------------------------------------------
Device Number
--------------------------------------------------------------------*/
#define COMPASS_DEVNO_MAJOR 0 // Major Number (Automatic Numbering)
#define COMPASS_DEVNO_MINOR 0 // Minor Number

#define COMPASS_DEVNO MKDEV(COMPASS_DEVNO_MAJOR, COMPASS_DEVNO_MINOR)

#define COMPASS_NDEVICES    1 // Number of Devices

/*--------------------------------------------------------------------
magnetism measurement data
--------------------------------------------------------------------*/
typedef struct compass_data {
  struct timeval time;    // Measuring Time

  // Measurement data of cmps
  int8_t temperature;    // Temperature magnetic sensors (TEMP)
  int16_t magnetism[3];   // Magnetic vector (DATA[XYZ])

  // condition of case
  uint8_t case_open;     // condition of case (open <=> true)

  // Number of errors after device opened.
  u_long total_error_count; // Number of errors
  u_long total_retry_count; // Number of retry

  int8_t measurement_range;  // Measurement range

#if defined(CONFIG_MACH_F12NAD)  /* FUJITSU:2012-03-28 NAD change start */
#define HSCD_MEASUREMENT_RANGE_12    0  // 12bit
#define HSCD_MEASUREMENT_RANGE_13    1  // 13bit
#else
#define CMPS_MEASUREMENT_RANGE_12    0  // 12bit
#define CMPS_MEASUREMENT_RANGE_13    1  // 13bit
#endif  /* FUJITSU:2012-03-28 NAD change end */

} compass_data_t;

/*--------------------------------------------------------------------
cmps Register
--------------------------------------------------------------------*/
// Parameters for 3-axis
#if defined(CONFIG_MACH_F12NAD)  /* FUJITSU:2012-03-28 NAD change start */
typedef union hscd_xyz {
#else
typedef union cmps_xyz {
#endif  /* FUJITSU:2012-03-28 NAD change end */
  int16_t xyz[3];
  struct { int16_t x, y, z; };
#if defined(CONFIG_MACH_F12NAD)  /* FUJITSU:2012-03-28 NAD change start */
} hscd_xyz_t;
#else
} cmps_xyz_t;

typedef union cmps_fuserom{
	uint8_t xyz[3];
	struct { uint8_t x, y, z; };
} cmps_fuse_t;
#endif  /* FUJITSU:2012-03-28 NAD change end */

// Register all of cmps
//   Interrupt Source INS 
//   Interrupt latches INL 
//   Interrupt Control INC 
//   Interrupt Threshold Register ITHR
//   not use the interrupt line is not connected.
#if defined(CONFIG_MACH_F12NAD)  /* FUJITSU:2012-03-28 NAD change start */
typedef struct hscd_registers {
#else
typedef struct cmps_registers {
#endif  /* FUJITSU:2012-03-28 NAD change end */
  uint8_t mask;     // Select read / write registers.
#if defined(CONFIG_MACH_F12NAD)  /* FUJITSU:2012-03-28 NAD change start */
#define HSCD_REGMASK_DATA     0x01 // Measurement data (TEMP,DATA[XYZ])
#define HSCD_REGMASK_OFFSET   0x02 // Sensor offset (OFF[XYZ])
#define HSCD_REGMASK_AMPGAIN  0x04 // Sensor amplifier gain (AMP)
#define HSCD_REGMASK_ADJUST   \
  (HSCD_REGMASK_OFFSET | HSCD_REGMASK_AMPGAIN)
#define HSCD_REGMASK_ALL      \
  (HSCD_REGMASK_DATA | HSCD_REGMASK_ADJUST)
#else
#define CMPS_REGMASK_DATA     0x01 // Measurement data (TEMP,DATA[XYZ])
#define CMPS_REGMASK_OFFSET   0x02 // Sensor offset (OFF[XYZ])
#define CMPS_REGMASK_AMPGAIN  0x04 // Sensor amplifier gain (AMP)
#define CMPS_REGMASK_ADJUST   \
  (CMPS_REGMASK_OFFSET | CMPS_REGMASK_AMPGAIN)
#define CMPS_REGMASK_ALL      \
  (CMPS_REGMASK_DATA | CMPS_REGMASK_ADJUST)
#define CMPS_REGMASK_FUSEROM  0x05 // Sensor fuserom
#endif  /* FUJITSU:2012-03-28 NAD change end */

  // Status and sensor measurements
  int8_t temp;     // Temperature sensor measurements (TEMP)
#if defined(CONFIG_MACH_F12NAD)  /* FUJITSU:2012-03-28 NAD change start */
  hscd_xyz_t data;  //magnetism sensor measurements (DATAX, DATAY, DATAZ)
#else
  cmps_xyz_t data;  //magnetism sensor measurements (DATAX, DATAY, DATAZ)
  cmps_fuse_t fuse;   //sensor fuseROM (ASAX, ASAY, ASAZ)
#endif  /* FUJITSU:2012-03-28 NAD change end */

  // Mode and set value register
  uint8_t ins;      // Interrupt Source (INS Unused)
  uint8_t stat;     // Status
  uint8_t inl;      // Interrupt latches (INL Unused)
  uint8_t cntl1;    // Mode (CNTL1)
  uint8_t cntl2;    // Mode (CNTL2)
  uint8_t cntl3;    // Mode (CNTL3)
  uint8_t cntl4;    // Mode (CNTL4)
  uint8_t inc;      // Interrupt Control (INC Unused)
  uint16_t ithr;    // Interrupt Threshold (ITHR Unused)
  uint8_t pret;     // Preset time (PRET)
	
  uint8_t amp;     // magnetism sensor amplifier gain (AMP)
#if defined(CONFIG_MACH_F12NAD)  /* FUJITSU:2012-03-28 NAD change start */
  hscd_xyz_t off;   // magnetism sensor offset (OFFX, OFFY, OFFZ)
#else
  cmps_xyz_t off;   // magnetism sensor offset (OFFX, OFFY, OFFZ)
#endif  /* FUJITSU:2012-03-28 NAD change end */

  // condition of case
  uint8_t case_open;     // condition of case (open <=> true)

  struct i2c_client* st_client;  //I2C Client information

#if defined(CONFIG_MACH_F12NAD)  /* FUJITSU:2012-03-28 NAD change start */
} hscd_registers_t;
#else
} cmps_registers_t;
#endif  /* FUJITSU:2012-03-28 NAD change end */

/*--------------------------------------------------------------------
Self Test
--------------------------------------------------------------------*/
typedef struct selftest_data {
  // setting value
  int8_t test_mode;    // test mode(A or B)
#define MODE_A           0  // test mode A
#define MODE_B           1  // test mode B
	
  // return value
  int8_t test_ret;     // test result(pass or fail)
#define SELF_TEST_PASS   0  // pass
#define SELF_TEST_FAIL   1  // fail

  uint8_t err_code;     // error code(Bmode fail only)
	
} selftest_data_t;

/*--------------------------------------------------------------------
IOCTL
--------------------------------------------------------------------*/
#define COMPASS_IOC_MAGIC   0xA2

#define COMPASS_IOC_GETDATA     _IOWR(COMPASS_IOC_MAGIC, 0x01, compass_data_t)
#define COMPASS_IOC_TEMP_COR    _IOWR(COMPASS_IOC_MAGIC, 0x02, compass_data_t)
#if defined(CONFIG_MACH_F12NAD)  /* FUJITSU:2012-03-28 NAD change start */
#define COMPASS_IOC_OFFCAL      _IOW(COMPASS_IOC_MAGIC, 0x03, hscd_registers_t)
#define COMPASS_IOC_GETREGS     _IOR(COMPASS_IOC_MAGIC, 0x04, hscd_registers_t)
#define COMPASS_IOC_SETREGS     _IOW(COMPASS_IOC_MAGIC, 0x05, hscd_registers_t)
#else
#define COMPASS_IOC_OFFCAL      _IOW(COMPASS_IOC_MAGIC, 0x03, cmps_registers_t)
#define COMPASS_IOC_GETREGS     _IOR(COMPASS_IOC_MAGIC, 0x04, cmps_registers_t)
#define COMPASS_IOC_SETREGS     _IOW(COMPASS_IOC_MAGIC, 0x05, cmps_registers_t)
#endif  /* FUJITSU:2012-03-28 NAD change end */
#define COMPASS_IOC_GETDATASTOP _IOW(COMPASS_IOC_MAGIC, 0x06, compass_data_t)
#define COMPASS_IOC_SELFTEST    _IOWR(COMPASS_IOC_MAGIC, 0x07, selftest_data_t)

/*====================== Definition for driver =====================*/
#ifdef __KERNEL__
/*--------------------------------------------------------------------
General-purpose macro definition
--------------------------------------------------------------------*/
#define OK      0

/*--------------------------------------------------------------------
Function  :Structure member's address is returned.
@param    : (1) type:Structure member's type.
            (2) base:Base address of structure.
            (3) offset:Byte offset of the structure member.
@retval   :Member's address.
--------------------------------------------------------------------*/
#define STRUCT_MEMBER(type, base, offset) \
  ((type*)((char*)(base) + (offset)))

/*--------------------------------------------------------------------
Log output
--------------------------------------------------------------------*/
#define compass_printk(logLevel, format, arg...) \
  printk("%s" format, logLevel, ##arg)

#endif /* __KERNEL__ */

/*--------------------------------------------------------------------
--------------------------------------------------------------------*/

#endif /* _LINUX_COMPASS_H */
