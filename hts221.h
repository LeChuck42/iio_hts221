#ifndef __HTS221_H__
#define __HTS221_H__

#define HTS221_CMD_WRITE        0x00
#define HTS221_CMD_READ         0x80

#define HTS221_CMD_NOINCR       0x00
#define HTS221_CMD_INCR         0x40

#define HTS221_ADDR_MASK        0x3f

/* Address registers */
#define REG_WHOAMI_ADDR         (0x0f)  /** Who am i address register */
#define REG_AVG_ADDR            (0x10)  /** Averaging conf register */
#define REG_CNTRL1_ADDR         (0x20)  /** CNTRL1 address register */
#define REG_CNTRL2_ADDR         (0x21)  /** CNTRL2 address register */
#define REG_H_OUT_L             (0x28)  /** OUT humidity address register */
#define REG_T_OUT_L             (0x2A)  /** OUT temperature address register */
#define REG_0RH_CAL_X_H         (0X36)  /** Calibration H 0 address register */
#define REG_1RH_CAL_X_H         (0X3a)  /** Calibration H 1 address register */
#define REG_0RH_CAL_Y_H         (0x30)  /** Calibration H 0 RH address register */
#define REG_1RH_CAL_Y_H         (0x31)  /** Calibration H 1 RH address register */
#define REG_0T_CAL_X_L          (0x3c)  /** Calibration T 0 address register */
#define REG_1T_CAL_X_L          (0x3e)  /** Calibration T 1 address register */
#define REG_0T_CAL_Y_H          (0x32)  /** Calibration T 0 C address register */
#define REG_1T_CAL_Y_H          (0x33)  /** Calibration T 1 C address register */
#define REG_STATUS              (0x27)  /** Status address register */

#define REG_T1_T0_CAL_Y_H       (0x35)  /** Calibration T0 and T! Address register  **/

#define CNTRL1_BIT_ODR0         (1<<0)
#define CNTRL1_BIT_ODR1         (1<<1)
#define CNTRL1_BIT_BDU          (1<<2)
#define CNTRL1_BIT_PD           (1<<7)

#define CNTRL2_BIT_ONE_SHOT     (1<<0)
#define CNTRL2_BIT_HEATER       (1<<1)
#define CNTRL2_BIT_BOOT         (1<<7)

/* Enable/Disable Sensor */
#define MASK_ENABLE             (0x80)
#define ENABLE_SENSOR           (0x80)
#define DISABLE_SENSOR          (0x00)

/* Sensor Average */
#define HTS221_H_AVG             0
#define HTS221_T_AVG             1

/* Default values */
#define WHOIAM_VALUE            (0xbc)  /** Who Am I default value */

/* Humidity and Termometer output data rate ODR */
#define	ODR_ONESH   0x0000	/* one shot 	*/
#define	ODR_1       0x0001	/*  1  Hz 	*/
#define	ODR_7		0x0002	/*  7  Hz 	*/
#define	ODR_12      0x0003	/* 12.5Hz 	*/
#define ODR_MASK    0x0003


#define HTS221_TEMP_FACTOR       3  // log2
#define HTS221_HUM_FACTOR        1  // log2

#endif
