/* *********************************************
   File:        Adapanel.h
   Title:       Adafruit panel controller
   Purpose:     STM32F4 Discovery
                Header for 32x16 LED panel
   Business:    HSA Elektrotechnik
   Compiler:    MDK-ARM
   Author/Date: F. Haunstetter / 23.10.2013
   Comment:     new
   Author/Date: F. Haunstetter / 10.11.2013
   Comment:     Supports project as of Nov 10, 2013.
   Author/Date: F. Haunstetter / 17.11.2013
   Comment:     Adaptations to sources and constants cleanup.
   *********************************************
*/

#ifndef __ADAPANEL_H__
#define __ADAPANEL_H__

/* global includes */
#include <stdlib.h>
#include <rt_heap.h>
#include <arm_math.h>						// CMSIS
#include <stm32f4xx.h>					// CMSIS

/* global types */
enum color {RED, GREEN, BLUE, COLORS, PINC=1};			// pixel memory implementation layout
enum strip {LOWER, UPPER, STRIPS, SI=1};
enum line {TOP, L1=TOP, LINES=16, LINC=1};
enum column {LEFT, C1=LEFT, COLUMNS=32, CINC=1};

// frame buffer type:
// - each element contains 1 double pixel
// - upper line pixel: B[2:0], lower line pixel: B[5:3]
typedef uint8_t buffer_t[LINES][COLUMNS];

/* global macros */
#define BIT(n)			(1 << n)
#define MASK(n, s)	((BIT(n) - 1) << s)

// dynamic system constants; frame update rate should be above 60 Hz
#define FREQ        (60*LINES*SHADES)

// picture attribute constants
#define SHADES      16
#define PIXEL(s, l, c) ((s)*LINES*COLUMNS+(l)*COLUMNS+(c))

/* common externals */
extern uint32_t SystemCoreClock;

/* global function prototypes */
void dbgWait_Ada32( volatile int w );							  // Adafruit32x16.c
void init_Ada32(void);
void clr_Ada32(void);
void bck_Ada32( uint32_t color );
void point( uint32_t x, uint32_t y, uint32_t color );

#endif // __ADAPANEL_H__
