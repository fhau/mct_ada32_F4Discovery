/* *********************************************
   File:        Adafruit32x16
   Title:       Adafruit panel controller
   Purpose:     STM32F4 Discovery
                Controller for 32x16 LED panel
   Business:    HSA Elektrotechnik
   Compiler:    MDK-ARM
   Author/Date: F. Haunstetter / 25.10.2013
   Comment:     new
   Author/Date: F. Haunstetter / 10.11.2013
   Comment:     More reliable signals, latency adjustment.
	              Support for 16 brightness levels.
								Now each pixel has 4 Parameters in 24 bits.
   Author/Date: F. Haunstetter / 17.11.2013
   Comment:     Added double buffer architecture for animations.
   *********************************************
   Each driver file supports just 1 device, and consists of:
   - at least 1 initialization routine (init_xxx),
   - driver capability routines as needed (use friendly names),
   - a shutdown / unload routine if necessary,
   - interrupt service routines as needed (xxxISR).
   *********************************************
*/

/* includes */
#include "Adapanel.h"

/* private macros */
#define TIM2DIV     (SystemCoreClock/(2.0*FREQ)+0.5)

/* private function prototypes */
uint16_t dld_Ada32( uint32_t l, uint32_t s );			// download frame buffer lines l and l+8 to panel line buffers 
void cpy_Ada32(void);															// propagate prepared to displayable frame buffer

/* global variables and buffers */
//static uint8_t* frame;														// module local current frame buffer pointer
static uint8_t* prep_frame;												// module local second frame buffer
static uint8_t* disp_frame;												// module local first frame buffer

uint32_t line_done;																// 1: display of current line done (preparation of next)
uint32_t pic_done;																// 1: display of current picture done (preparation of next)

//
// Initialize clock and frame buffers.
//
void init_Ada32()
{
	// get 2 heap segments, each with a stack of pixel buffers for picture frames
	disp_frame = (uint8_t*) malloc( sizeof(buffer_t[SHADES]) );	// frame for display
	prep_frame = (uint8_t*) malloc( sizeof(buffer_t[SHADES]) );	// frame to prepare next display

	// Portmap:
	// R1 = PE10 G1 = PE11 B1 = PE12 (upper)
	// R2 = PE13 G2 = PE14 B2 = PE15 (lower)
	// L2 = PE9 L1 = PE8 L0 = PE7
	// CLK = PB2
	// STB = PB11 (Timer 2/Ch4)
	// OE = PB12 (inverted)
	// PB10 must be n/c

	// reset all frame buffers, and the panel's line buffers to "all LEDs off"
	clr_Ada32();
	cpy_Ada32();
	dld_Ada32( 0, 0 );
	
	// setup of strobe timer
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOBEN;	// turn port E and B on to enable panel control
	
	// configure LED port pins for low speed output using read-modify-write, push-pull is default
	GPIOE->MODER = (GPIOE->MODER
		& ~(GPIO_MODER_MODER11 | GPIO_MODER_MODER12 | GPIO_MODER_MODER13 | GPIO_MODER_MODER14 | GPIO_MODER_MODER15\
		| GPIO_MODER_MODER6 | GPIO_MODER_MODER7 | GPIO_MODER_MODER8 | GPIO_MODER_MODER9 | GPIO_MODER_MODER10))
		| (GPIO_MODER_MODER11_0 | GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0 | GPIO_MODER_MODER14_0 | GPIO_MODER_MODER15_0\
		| GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0 | GPIO_MODER_MODER10_0);
	GPIOE->OSPEEDR = (GPIOE->OSPEEDR
		& ~(GPIO_OSPEEDER_OSPEEDR11 | GPIO_OSPEEDER_OSPEEDR12 | GPIO_OSPEEDER_OSPEEDR13 | GPIO_OSPEEDER_OSPEEDR14 | GPIO_OSPEEDER_OSPEEDR15\
		| GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7 | GPIO_OSPEEDER_OSPEEDR8 | GPIO_OSPEEDER_OSPEEDR9 | GPIO_OSPEEDER_OSPEEDR10));

	// configure OutputEnable as output, Strobe as timer 2 port, and Serial Data Clock as output
	GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER12 | GPIO_MODER_MODER11 | GPIO_MODER_MODER2))
		| (GPIO_MODER_MODER12_0 | GPIO_MODER_MODER11_1 | GPIO_MODER_MODER2_0);
	GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~(GPIO_OSPEEDER_OSPEEDR12 | GPIO_OSPEEDER_OSPEEDR11 | GPIO_OSPEEDER_OSPEEDR2))
		/*| (GPIO_OSPEEDER_OSPEEDR11_0 | GPIO_OSPEEDER_OSPEEDR2_0)*/; // lowest speed necessary for driving the signals directly
	GPIOB->AFR[11/8] |= 1 << (11%8)*4;										// STB = Tim2/Ch4, that is AF1 on Bit 11
	GPIOB->ODR &= ~BIT(12);																// /OE should currently always kept aktive
	
	// configure strobe timer TIM2
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;										// turn timer TIM2 on for strobe pulse generation
	TIM2->CCMR2 =  TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;		// do PWM style output
	TIM2->ARR = (unsigned int) TIM2DIV;										// counter repetition rate, fixed for flicker free displaying
	TIM2->CCR4 = (unsigned int) TIM2DIV-3;								// initiate display by opening the latch (pulse width ~25ns)
	TIM2->CCR1 = (unsigned int) 15;												// interrupt routine must be called with latency correction to Ch4 change
	TIM2->CCER |= TIM_CCER_CC4E | TIM_CCER_CC4P;					// enable CC4 output functions, rising edge strobe pulse (active low)
	TIM2->DIER |= TIM_DIER_CC1IE;													// enable trailing edge compare match interrupts, with correction 
	TIM2->CR1 |= TIM_CR1_CEN |  TIM_CR1_URS |  TIM_CR1_DIR;// run the timer: update is flagged on underflow, counting down

	// configure TIM2 interrupts (Int 28, Prio 1)
	NVIC_SetPriority( TIM2_IRQn, 1 );
	NVIC_ClearPendingIRQ( TIM2_IRQn );
	NVIC_EnableIRQ( TIM2_IRQn );
}

//
// Operate panel while latch is already open:
// Display buffered line pair at any line number by appying the address.
// Then prepare the line buffer for the next pair of lines to display.
// Any of the above MAY NOT BE INTERRUPTED by concurrend tasks.
// Match function name with the Interrupt Vector Table, in file startup_... .s
//
void TIM2_IRQHandler()																	// CAUTION: interrupt latency at max. speed is appx. 450ns
{
	static uint32_t disp_line = TOP;											// local subscript of line to display
	static uint16_t disp_port = 0;												// prepared port bit sequence for panel line number
	static uint32_t current_shade = 0;										// local subscript of shade to display
	
	GPIOE->ODR = disp_port;																// display 2 lines of current content, not wasting time

	disp_line++;																					// compute line and
	line_done = 1;																				// synchronize line with main loop
	if (disp_line == LINES)
	{
		disp_line = TOP;
		current_shade = (current_shade + 1) % SHADES;				// shade, next to display
		pic_done = 1;																				// synchronize picture with main loop
		if (current_shade == 0)															// after a picture is complete
			cpy_Ada32();																			// propagate frame buffers to not disrupt animations
	}
	
	disp_port = dld_Ada32( disp_line, current_shade );		// prepare the next 2 lines to display

	TIM2->SR &= ~(BIT(0) | BIT(1) | BIT(4));							// must clear interrupt flags

	GPIOD->ODR ^= BIT(12);
}


/* *********************************************
   Download display frame buffer line pair (l) from
   frame shade (s) to the panel's line buffers. *critical*
	 *********************************************/
uint16_t dld_Ada32( uint32_t l, uint32_t s )
{
	uint32_t c;

	for (c = LEFT; c < COLUMNS; c++)
	{
		// output pixel data of 1 panel line, l and l+8 implicitely, from frame buffer
		GPIOE->ODR = (GPIOE->ODR & ~MASK(6,10)) | (disp_frame[PIXEL(s,l,c)] << 10);

		GPIOB->ODR |= BIT(2);																// output a CLK pulse
		__asm {																							// ~25ns puls width @ 168MHz
			nop
			nop
			nop
		};
		GPIOB->ODR &= ~BIT(2);
	}
	
	return (GPIOE->ODR & ~MASK(3,7)) | l << 7;						// prepare matching line number
}


/* *********************************************
   Copy prepared frame to display frame. *critical*
	 *********************************************/
void cpy_Ada32()
{
	uint32_t c;																						// column counter
	uint32_t l;																						// line counter (line and line+8)
	uint32_t s;																						// shades counter

	for (s = 0; s < SHADES; s++)
		for (l = 0; l < LINES; l++)
			for (c = 0; c < COLUMNS; c++)
				disp_frame[PIXEL(s,l,c)] = prep_frame[PIXEL(s,l,c)];
}


/* *********************************************
   Clear frame buffers to all LEDs off.
	 *********************************************/
void clr_Ada32()
{
	uint32_t c;																						// column counter
	uint32_t l;																						// line counter (line and line+8)
	uint32_t s;																						// shades counter

	for (s = 0; s < SHADES; s++)
		for (l = 0; l < LINES; l++)
			for (c = 0; c < COLUMNS; c++)
				prep_frame[PIXEL(s,l,c)] = (uint8_t)(0);
}


/* *********************************************
   Set frame buffers for a background color.
   0: all LEDs off
	 *********************************************/
void bck_Ada32( uint32_t color )
{
	uint32_t c;																						// column counter
	uint32_t l;																						// line counter (line and line+8)
	uint32_t s;																						// shade buffer counter
	uint32_t r = (color & MASK(8, 16)) >> 16;							// LED color brightness decoding
	uint32_t g = (color & MASK(8, 8)) >> 8;
	uint32_t b =  color & MASK(8, 0);
	uint32_t rgb;																					// frame buffer temporary pixel
	
	for (s = 0; s < SHADES; s++)													// filling the shade buffers according to brightness
	{
		rgb  = (r > 0)? r--,BIT(0): 0;
		rgb |= (g > 0)? g--,BIT(1): 0;
		rgb |= (b > 0)? b--,BIT(2): 0;

		for (l = 0; l < LINES; l++)
			for (c = 0; c < COLUMNS; c++)
				prep_frame[PIXEL(s,l,c)] = (uint8_t)(rgb | rgb << 3);
	}
}


/* *********************************************
   Add point at position with color.
   x: 0 .. 31, y: 0 .. 15, color: 0x00:r(0 .. 255):g(0 .. 255):b(0 .. 255)
   caveat: r,g,b currently supported only 0 .. 15
	 *********************************************/
void point( uint32_t x, uint32_t y, uint32_t color )
{
	uint32_t ls = y < 8? 0: 3;
	uint32_t s;																						// shade buffer counter
	uint32_t r = (color & MASK(8, 16)) >> 16;							// LED color brightness decoding
	uint32_t g = (color & MASK(8, 8)) >> 8;
	uint32_t b =  color & MASK(8, 0);
	uint32_t rgb;																					// frame buffer temporary pixel
	
	for (s = 0; s < SHADES; s++)													// setting the shade buffers according to brightness
	{
		rgb  = (r > 0)? r--,BIT(0): 0;
		rgb |= (g > 0)? g--,BIT(1): 0;
		rgb |= (b > 0)? b--,BIT(2): 0;
		prep_frame[PIXEL(s, y&(LINES-1), x&(COLUMNS-1))] =
			(prep_frame[PIXEL(s, y&(LINES-1), x&(COLUMNS-1))] & ~MASK(3, ls)) | (uint8_t)(rgb << ls);
	}
//	frame[s][y&0x7][x&0x1F] = (uint8_t)(rgb << ls);
}


/* *********************************************
   Inserts some time, for debug purposes only.
	 *********************************************/
void dbgWait_Ada32( volatile int w )
{
	for (;w;w--);
}

