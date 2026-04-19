/*
 *******************************************************************************
 *                           Includes
 *******************************************************************************
 */
#ifndef _USER_MAIN_H
#define _USER_MAIN_H

/* Includes */
#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <assert.h>
/* Includes */
#include "stm32f10x.h"
#include "types.h"
#include "delay.h"
#include "dataoperation.h"
#include "port.h"
#include "fm24cxx.h"
#include "uart.h"
#include "uart1.h"
#include "pwm.h"
#include "OLED_I2C.h"
#include "adc.h"
#include "usart1process.h"

#define   VERSIONID     (100012)

#undef CLOCKS_PER_SEC
#define CLOCKS_PER_SEC      1000


#endif  //_USER_MAIN_H
