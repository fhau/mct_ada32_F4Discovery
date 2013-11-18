/* *********************************************
   File:        main.c
   Title:       Adafruit panel controller
   Purpose:     STM32F4 Discovery
                Controller for 32x16 LED panel
   Business:    HSA Elektrotechnik
   Compiler:    MDK-ARM
   Author/Date: F. Haunstetter / 12.10.2013
   Comment:     new
   Author/Date: F. Haunstetter / 17.10.2013
	 Comment:     Heap in low RAM
   Author/Date: F. Haunstetter / 18.10.2013
	 Comment:     Picture buffer organization.
   Author/Date: F. Haunstetter / 10.11.2013
	 Comment:     Reduced overhead, prepared for animations.
	              Project doesn't implement animations yet.
   Author/Date: F. Haunstetter / 17.11.2013
	 Comment:     Added tests for animations.
   *********************************************
   A 'main' programm file consists just of the project's only main routine.
   *********************************************
*/

/* includes */
#include "Adapanel.h"

/* private macros */
void Electric(void);
void Colors(void);
void AniDimm(void);
void Bavaria(void);
void Centrifuge(void);

/* private function prototypes */
void setupMCO(void);
void init_heap( void );														// initialize heap support
void init_board_leds( void );											// initialize on board visualization

void SysTick_Handler(void);

/* global variables and buffers */
extern uint32_t line_done;												// global line synchronization flag
extern uint32_t pic_done;													// global picture synchronization flag

/* TESTING AND DEBUGING SUPPORT */
struct tf {
	void (*fu) (void);				// pointer to function to invoke
	uint32_t repeat;					// if >0: number of ticks between adjacent runs
	uint32_t run_once;				// if >0: number of ticks from beginning, to run one or first time
	uint32_t timetick;				// timer tick counter (initially always set to zero)
};
void test( struct tf* p_tf );

static struct tf erase = {&clr_Ada32,     0,  9000, 0};
static struct tf fu1 =   {&AniDimm,     500,   500, 0};
static struct tf fu21 =  {&Colors,    20000, 10000, 0};
static struct tf fu22 =  {&Colors,    20000, 15000, 0};
static struct tf fu3 =   {&Electric,  20000,     1, 0};
static struct tf fu4 =   {&Bavaria,      10,     1, 0};
static struct tf fu5 =   {&Centrifuge,    7,     1, 0};
/* END TEST AND DEBUG */

/* *********************************************
		main routine
	 *********************************************/
int main()
{
	unsigned int ticks = 														// ticks for desired animation timebase
		(unsigned int) (SystemCoreClock/1000.0 +0.5);

	NVIC_SetPriority( SVCall_IRQn, 0 );							// CIMSIS: Supervisor call with max. priority
	NVIC_SetPriority( SysTick_IRQn, 3 );						// CIMSIS: run tick timer with medium priority
	
  init_heap();																		// initialize heap support
	init_board_leds();															// debug output with on-board LEDs

	init_Ada32();																		// initialize buffer and signals for panel strobe

	if (SysTick_Config( ticks ))										// CIMSIS: start tick timer for animations
		__asm { SVC #15 };														// ... on TT-error
	

	// main loop: waiting for interrupts, looping endlessly
	for(;;)
	{
		if (line_done)																// after every line:
		{
			line_done = 0;
		}
		
		if (pic_done)																	// after every picture:
		{
			pic_done = 0;
		}
	}
}

/* *********************************************
		ISR SysTick: overrides weak label in vtable
		write to frame buffer to display a picture
	 *********************************************/
void SysTick_Handler()
{
//	test( &fu1 );
//	test( &fu21 );
//	test( &fu22 );
//	test( &fu3 );
//	test( &fu4 );
	test( &fu5 );
	
	GPIOD->ODR ^= BIT(14);
}


/* *********************************************
		DEMO: Electric power waves
	 *********************************************/
void Electric()
{
	static int x = 0;
	static int y = 0;
	static const int col[] = {0x00080201, 0x00080000, 0x000A0200, 0x00010101, 0x00010101, 0x00010101, 0x00010101, 0};

	clr_Ada32();																		// erase panel before display

	for (x = 0; x < 32; x++)
	{
		y = (int)((-sin(((double)x/32 + (double)0/3)*2*3.14157) + 1)*7.4 +0.5);
		point( x, y, col[0] );
		y = (int)((-sin(((double)x/32 + (double)1/3)*2*3.14157) + 1)*7.4 +0.5);
		point( x, y, col[1] );
		y = (int)((-sin(((double)x/32 + (double)2/3)*2*3.14157) + 1)*7.4 +0.5);
		point( x, y, col[2] );
	}
}


/* *********************************************
		DEMO: Colors
	 *********************************************/
void Colors()
{
	enum {X0, Y0=0, EDGE=16, RED_SH=16, GREEN_SH=8, BLUE_SH=0};
	static int x = 0;
	static int y = 0;
	static uint32_t i = 0;

	clr_Ada32();																		// erase panel before display

	for (y = Y0; y < (Y0+EDGE); y++)
	{
		if (i == 0)
		{
			for (x = X0; x < EDGE; x++)
					point( x, y, (int)((double)(x%EDGE)/EDGE*SHADES+.5)<<RED_SH   | (int)((double)(y-Y0)/EDGE*SHADES+.5)<<GREEN_SH );
			for (x = X0+EDGE; x < (X0+2*EDGE); x++)
					point( x, y, (int)((double)(x%EDGE)/EDGE*SHADES+.5)<<RED_SH   | (int)((double)(y-Y0)/EDGE*SHADES+.5)<<BLUE_SH );
		}
		else
		{
			for (x = X0; x < EDGE; x++)
					point( x, y, (int)((double)(x%EDGE)/EDGE*SHADES+.5)<<GREEN_SH | (int)((double)(y-Y0)/EDGE*SHADES+.5)<<BLUE_SH );
			for (x = X0+EDGE; x < (X0+2*EDGE); x++)
					point( x, y, (int)((double)(x%EDGE)/EDGE*SHADES+.5)<<RED_SH  | (int)((double)(y-Y0)/EDGE*SHADES+.5)<<GREEN_SH | (int)((double)(y-Y0)/EDGE*SHADES+.5) );
		}
	}
	i ^= 1;
}


/* *********************************************
		DEMO: Animated dimming of colors
	 *********************************************/
void AniDimm()
{
	enum {X0=11, Y0=3, EDGE=10, RED_SH=16, GREEN_SH=8, BLUE_SH=0};
	int x, y, m;
	static int n = 1;

	for (y = Y0, m = 1; y < (Y0+EDGE); y++, m++)
	{
		if (m <= n)
		for (x = X0; x < (X0+EDGE); x++)
			point( x, y, 1<<RED_SH | 1<<GREEN_SH );
	}
	n++;
	if (n > EDGE)
	{
		n = 1;
		clr_Ada32();
	}
}


/* *********************************************
		DEMO: Flashing blue
	 *********************************************/
void Bavaria()
{
	static uint32_t c = 1;
	static int add = 1;
	
	bck_Ada32( c );																	// paint a background
	
	c += add;
	if (c == 16 || c == 0)
	{
		add = -add;
		c += 2*add;
	}
}


/* *********************************************
		DEMO: Rotation
	 *********************************************/
void Centrifuge()
{
	#define PI 3.14157
	#define N  60
	enum {RED_SH=16, GREEN_SH=8, BLUE_SH=0};

	static double phi = 0;
	double x = 0, y = 0;
	
//	bck_Ada32( 0x00010001 );												// paint a background
	clr_Ada32();												// paint a background
	
	point( (uint32_t)(x+.5), (uint32_t)(y+.5), 0 );
	x = ((COLUMNS-1) + (LINES-1)*cos(phi))/2.0;
	y = (LINES-1)/2.0*(1.0 - sin(phi));
	point( (uint32_t)(x+.5), (uint32_t)(y+.5), 1<<GREEN_SH );
	
	phi = phi + 2*PI/N;
	if (phi > 2*PI)
		phi = 0;
}

/* *** Generic Utiliy Routines *****************/

/* *********************************************
		PLL: system clock at Microcontroller Clock Out 2 (PC9)
		to be called from startup
	 *********************************************/
void setupMCO(void)
{
	/* Configure the GPIO pins */
	RCC->AHB1ENR    |= RCC_AHB1ENR_GPIOCEN;			// turn port C on to enable MCO2
	RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_MCO2PRE | RCC_CFGR_MCO2)) | (0x38000000); // SYSCLK and output divisor 5
	
	GPIOC->MODER    = (GPIOC->MODER & ~GPIO_MODER_MODER9) |  GPIO_MODER_MODER9_1; // alternate function
	GPIOC->OSPEEDR  |= GPIO_OSPEEDER_OSPEEDR9;	// PC9 = 100MHz Fast Speed
	GPIOC->OTYPER   &= ~GPIO_OTYPER_OT_9;				// output Push-Pull
	GPIOC->PUPDR    &= ~GPIO_PUPDR_PUPDR9;   		// no pull resistors
}


/* *********************************************
   Initialize heap management with parameters from Startup.s
	 *********************************************/
__asm void init_heap( void )
{
  IMPORT __user_initial_stackheap
  IMPORT _init_alloc
  
  push  {r12, lr}
  ldr   r12, = __user_initial_stackheap   		// get parameters by calling Startup.s
  blx   r12
  ldr   r12, = _init_alloc                		// init heap manager: r0 = base:8, r1 = top:8
  mov   r1, r2
  blx   r12
  pop   {r12, pc}

  ALIGN
}


/* *********************************************
   Initialize Discovery Board LED quadrupel
	 *********************************************/
void init_board_leds( void )
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;				// turn port D on to enable LED control
	
	// configure LED port pins for high speed output using read-modify-write, push-pull is default
	GPIOD->MODER = (GPIOD->MODER
		& ~(GPIO_MODER_MODER12 | GPIO_MODER_MODER13 | GPIO_MODER_MODER14 | GPIO_MODER_MODER15))
		| (GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0 | GPIO_MODER_MODER14_0 | GPIO_MODER_MODER15_0);
	GPIOD->OSPEEDR = (GPIOD->OSPEEDR
		& ~(GPIO_OSPEEDER_OSPEEDR12 | GPIO_OSPEEDER_OSPEEDR13 | GPIO_OSPEEDER_OSPEEDR14 | GPIO_OSPEEDER_OSPEEDR15))
		| (GPIO_OSPEEDER_OSPEEDR12_0 | GPIO_OSPEEDER_OSPEEDR13_0 | GPIO_OSPEEDER_OSPEEDR14_0 | GPIO_OSPEEDER_OSPEEDR15_0);
}


/* *********************************************
   Invoke test functions without parameters, returning nothing
	 *********************************************/
void test( struct tf* p_tf )									// parameter contains test properties, see declaration, must be static
{
	if (p_tf->run_once != 0)										// functions to be run once (repeat == 0), or to be started with delay
	{
		if (p_tf->run_once == 1)									// "1" means: run immediately
			(p_tf->fu)();
		p_tf->run_once--;
	}

	else if (p_tf->repeat != 0)									// functions to be run repeatedly
	{
		if (p_tf->timetick == p_tf->repeat)				// every function has its own tick counter
		{
			p_tf->timetick = 0;											// tick counters may never overflow
			(p_tf->fu)();
		}
		else
			p_tf->timetick++;
	}
}
