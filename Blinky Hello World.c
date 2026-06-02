#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "MKL46Z4.h"


#define BaseDutyLeft  5500
#define BaseDutyRight 5800
#define Sp_SetPoint 20   // desired encoder pulses per 10ms
#define Kp 5             // proportional gain
#define Ki 0
#define Kd 0

/*  ============================================================
    GLOBAL VARIABLES
    ============================================================ */
volatile int state = 0;
volatile int moveCount = 0;   // counts 10 ms periods during 6-second move
volatile int  left_pulsesCount = 0;  // Counting the pulses on the left  pulse.
volatile int  right_pulsesCount = 0; // Counting the pulses on the right pulse.
volatile int  Pv_leftWheel = 0;      // left wheel current speed or process variable
volatile int  Pv_rightWheel = 0;    // right wheel current speed or process variable
volatile int left_sumError = 0;
volatile int right_sumError = 0;
volatile int left_prevError = 0;
volatile int right_prevError = 0;
/* ============================================================
   TRIM TUNING VALUES (YOU ADJUST THESE)
   ============================================================ */

/* ============================================================
   MOTOR HELPER FUNCTIONS
   These functions just set GPIO pins to control motor direction.
   ============================================================ */

/*
  Left_Forward():
  - Sets the left motor driver pins for forward direction.
  - AI1 and AI2 control direction.
  - For this driver wiring:
      AI1=0, AI2=1 -> forward
*/
static inline void Left_Forward(void) {
    GPIOB->PCOR = (1u << 1);   // PTB1 low  -> AI1 = 0
    GPIOB->PSOR = (1u << 0);   // PTB0 high -> AI2 = 1
  }

/*
  Left_Reverse():
  - Sets the left motor direction to reverse.
  - AI1=1, AI2=0 -> reverse
*/
 static inline void Left_Reverse(void) {
    GPIOB->PSOR = (1u << 1);   // PTB1 high -> AI1 = 1
    GPIOB->PCOR = (1u << 0);   // PTB0 low  -> AI2 = 0
  }

/*
  Right_Forward():
  - Sets the right motor driver pins for forward direction.
  - BI1 and BI2 control direction.
  - For this driver wiring:
      BI1=0, BI2=1 -> forward
*/
 static inline void Right_Forward(void) {
    GPIOC->PCOR = (1 << 1);   // PTC1 low  -> BI1 = 0
    GPIOC->PSOR = (1 << 2);   // PTC2 high -> BI2 = 1
  }

/*
  Right_Reverse():
 - Sets the right motor direction to reverse.
 - BI1=1, BI2=0 -> reverse
 */

  static inline void Right_Reverse (void) {
    GPIOC->PSOR = (1 << 1);   // PTC1 high -> BI1 = 1
    GPIOC->PCOR = (1 << 2);   // PTC2 low  -> BI2 = 0
}

/*
  Left_Stop():
  - Stops left motor direction command by making both direction pins 0.
  - When both direction pins are 0, the driver stops (coast/stop depending on driver).
*/
static inline void Left_Stop(void) {
    GPIOB->PCOR = (1 << 0);   // PTB0 low -> AI2 = 0
    GPIOB->PCOR = (1 << 1);   // PTB1 low -> AI1 = 0
}

/*
  Right_Stop():
  - Stops right motor direction command by making both direction pins 0.
*/
static inline void Right_Stop (void) {
    GPIOC->PCOR = (1 << 1);   // PTC1 low -> BI1 = 0
    GPIOC->PCOR = (1 << 2);   // PTC2 low -> BI2 = 0
}


/*
  Delay():
  - Simple busy-wait delay (burns CPU cycles).
  - This is NOT an accurate real-time delay, but good enough for basic testing.
*/
static inline void Delay(volatile unsigned int t) {
    while (t--) {
        __asm volatile ("nop");  // no operation (waste one CPU cycle)
    }
 }

/* ============================================================
   PWM SPEED CONTROL HELPERS (SIMPLE VERSION)
   ============================================================ */

/*
  LeftMotorSpeed(speed):
  - Sets duty cycle for TPM2 channel 0 (left motor PWM pin).
  - speed is the compare value CnV (0..MOD).
  - bigger speed -> more duty cycle -> faster motor.
*/
static inline void LeftMotorSpeed (int speed) {
    TPM2->CONTROLS[0].CnV = speed; // left motor PWM duty compare value
 }

/*
  RightMotorSpeed(speed):
  - Sets duty cycle for TPM2 channel 1 (right motor PWM pin).
*/
static inline void RightMotorSpeed(int speed) {
    TPM2->CONTROLS[1].CnV = speed; // right motor PWM duty compare value
 }

 /*
  StopBothMotors():
  - Stops the motors in two ways:
      (1) PWM duty = 0 (no power)
      (2) Direction pins set to 0 (stop direction command)
 */
   static inline void StopBothMotors(void) {
    // Stop PWM output (no “push” to motors)
    TPM2->CONTROLS[0].CnV = 0; //defines how long the signal stays ON inside a period (the duty cycle).
    TPM2->CONTROLS[1].CnV = 0;

    // Stop direction pins (no direction command)
    Left_Stop();
    Right_Stop();
}

 /*
  TPM2GenPWM():
  - Configures TPM2 to generate PWM signals on PTB2 (TPM2_CH0) and PTB3 (TPM2_CH1).
  - Also selects OSCERCLK as the TPM clock source.
  - This sets up:
      * Pin muxing for PTB2/PTB3 to TPM2 function
      * MOD value (PWM period)
      * PWM mode in CnSC
      * Starts TPM2 counter (TPM2->SC)
*/
static void TPM2GenPWM(void) {

    // Debug print to show we entered PWM setup
    PRINTF("Entered TPM2GenPWM\r\n");

    // 1) Enable OSCERCLK output (external reference clock enable)
    OSC0->CR |= (1u << 7);   // ERCLKEN

    // 2) Configure oscillator settings for 8MHz crystal usage
    MCG->C2 |= (1 << 2);    // EREFS0=1 (use crystal)
    MCG->C2 |= (2 << 4);    // RANGE0=2 (high frequency range)

    PRINTF("Before wait: MCG->S=0x%08x OSC0->CR=0x%08x MCG->C2=0x%08x\r\n",
      (unsigned int)MCG->S, (unsigned int)OSC0->CR, (unsigned int)MCG->C2);

    // 3) Wait for oscillator to initialize (timeout prevents infinite loop)
    volatile unsigned int timeout = 5000000;
    while (!(MCG->S & (1u << 1)) && timeout--) { }   // OSCINIT0 is bit 1

    PRINTF("After wait: OSCINIT0=%d MCG->S=0x%08x\r\n",
           (MCG->S & (1 << 1)) ? 1 : 0,
           (unsigned int)MCG-> S);

    // Enable TPM2 clock gate (TPM2 must have clock to work)
     SIM->SCGC6 |=  (1 << 26);

    // Enable PORTB clock (needed for PTB2/PTB3 pin mux)
    SIM->SCGC5  |=   (1  << 10);

    /*
      Configure PTB2/PTB3 pin mux to TPM2 function (ALT3).
      Steps:
      - Clear MUX bits [10:8] first
      - Set them to ALT3 (0b011) -> 0x300
    */
    PORTB->PCR[2]  &= ~0x700;  // clear MUX bits for PTB2
    PORTB->PCR[3]  &= ~0x700;  // clear MUX bits for PTB3
    PORTB->PCR[2]  |= (0x300); // PTB2 -> TPM2_CH0
    PORTB->PCR[3]  |= (0x300); // PTB3 -> TPM2_CH1


    /*
      Select TPM clock source = OSCERCLK
      SIM_SOPT2[25:24] = 10 (binary)
    */
    SIM->SOPT2 &= ~(3 << 24); // clear bits first
    SIM->SOPT2 |=  (2 << 24); // set TPMSRC=OSCERCLK

    // Stop TPM2 before configuring
    TPM2->SC  = 0;  // stop counter
    TPM2->CNT = 0;  // reset counter value

    /*
      Set MOD = 7999
      - MOD controls the PWM period (frequency).
      - PWM frequency depends on TPM clock and MOD.
    */
    TPM2 -> MOD = 7999;

    /*
      Configure PWM mode for channels:
      Using (2<<4)|(2<<2) sets the proper bits for edge-aligned PWM.
      (This config chooses the channel output behavior.)
    */
    TPM2->CONTROLS[0].CnSC = (2 << 4) | (2 << 2); // channel 0 PWM mode
    TPM2->CONTROLS[1].CnSC = (2 << 4) | (2 << 2); // channel 1 PWM mode

    // Initial duty cycle values (initial “speed” values)
    TPM2->CONTROLS[0].CnV = 5600;   // left initial PWM compare
    TPM2->CONTROLS[1].CnV = 5600;   // right initial PWM compare

    /*
      Start TPM2:
      - CMOD=01 (counter runs)
      - PS= 000 (prescaler /1)
      Here they are inside TPM2->SC.
    */
    TPM2->SC = (1 << 3) | (0 << 0);

    // Debug: wait a bit and print CNT to prove counter is running
    Delay(200000);
    PRINTF("CNT now = %u\r\n", (int)TPM2->CNT);
 }

   /* ============================================================
      OPEN LOOP
   ============================================================ */

 static inline void OpenLoopDrivestraight(void) {
	Left_Forward();
	Right_Forward();
	LeftMotorSpeed(BaseDutyLeft);
	RightMotorSpeed(BaseDutyRight);

   }
static  inline int adjustPWM (int pwm) {
    if (pwm > 7999) {
	    pwm = 7999;
       }
     if (pwm < 0) {
	   pwm = 0; // set back to zero
     }
    return pwm;
   }
  /* ============================================================
   TIMER INTERRUPT
   Runs every ~10 ms
   ============================================================ */
  void TPM0_IRQHandler (void)  {

	// TOF = 1  → timer overflow happened.
	// TOF = 0  → no overflow
    if (TPM0->SC & (1 << 7))  {

    	  if (state == 1)    { // only control while robot is moving

    /* -------- LEFT WHEEL CONTROL -------- */
    	// STEP 1: measured speed
    	Pv_leftWheel = left_pulsesCount;  // P_leftWheel = measured wheel speed
    	left_pulsesCount = 0;  // reset for next period

        // STEP 2: WE COMPUTE THE ERROR
        int current_Left_Error = Sp_SetPoint - Pv_leftWheel; // proportiona_Error = desired speed − measured speed
        left_sumError += current_Left_Error;  //  Accumulated error (Integral)
        int left_diffError =  current_Left_Error - left_prevError; // Difference in error (Derivative)

       // STEP 3: Proportional correction
          int left_correction = Kp   *  current_Left_Error  // proportiona_Error * Kp
          	  	  	  	  	  + (Ki  *  left_sumError)  // +   Ki * Integral
                              + (Kd  *  left_diffError); // +   Kd * Derivative
       // STEP 4: store the previous error
           left_prevError  = current_Left_Error;

       // STEP 5: compute PWM
       int left_pwm = BaseDutyLeft + left_correction;

       // STEP 6: called adjustPWM & limited PWM range.
       left_pwm =  adjustPWM (left_pwm);

        // STEP 7: apply the change on the left wheels
          LeftMotorSpeed (left_pwm);

   /* ------------- RIGHT WHEEL CONTROL -------- */
       // STEP 1: measured speed
       Pv_rightWheel = right_pulsesCount;  // mesuring the right pulse count
       right_pulsesCount = 0;

       // STEP 2: Compute the Error

       int current_right_Error = Sp_SetPoint - Pv_rightWheel;  // e(t) = SP - PV --> prop_Error = desired speed − measured speed
       right_sumError  += current_right_Error;   // integral error
       int right_diffError =  current_right_Error - right_prevError; // Difference in error (Derivative)


       // STEP 3: proportional correction.
         int right_corection =  (Kp * current_right_Error)
        		    			 + (Ki *  right_sumError)
								 + (Kd * right_diffError);
         // STEP 4: store the previous error
         right_prevError= current_right_Error;
       // STEP 5: ADJUST THE MOTOR SPEED(PWM)
		   int right_pwm  =  BaseDutyRight + right_corection;

		   // STEP 5: clamp PWM to valid range
		   right_pwm = adjustPWM (right_pwm);
		   // STEP 6: apply the change on the right wheels
		   RightMotorSpeed(right_pwm);

		   /* -------- 6-second movement timer -------- */
		    moveCount++;
		    if (moveCount >= 600)   { // 600 × 10 ms = 6 sec
		    StopBothMotors();
		    state = 2;   // finished
		    }
		 }
		 // clear timer interrupt flag
		 TPM0->SC |= (1 << 7);
    }
  }
  // Encoder interrupt ISR (counts pulses)
 void PORTA_IRQHandler (void)  {
	if  (PORTA -> ISFR &  (1 << 6) ) {  // check if PTA pin 6 caused interrupt [PTA6: left-Encoder]
		PORTA  -> ISFR =  (1 << 6);    //   clear the interrupt if there is 1.
		left_pulsesCount++;        // increment count
	}
	if (PORTA -> ISFR  &  (1 << 14) ) { // check if PTA pin 14 caused interrupt [PTA14: left-Encoder]
		PORTA -> ISFR  =  (1 << 14);   // if there is 1 clear it
		right_pulsesCount++;
	 }
  }

   /* ============================================================
      MAIN:
      ============================================================ */

int main (void) {

    /*
      These three MCUXpresso-generated calls set up:
      - pins
      - system clocks
      - peripherals that were configured in the project settings
    */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();

	/* ===============================
	   LEFT ENCODER SETUP (PTA6/PTA7)
	   =============================== */
	SIM->SCGC5 |= (1 << 9);   // enable PORTA clock

	PORTA->PCR[6] = (1 << 8) | (1 << 1) | (1 << 0) | PORT_PCR_IRQC(0x0B);  // left A = interrupt
	PORTA->PCR[7] = (1 << 8) | (1 << 1) | (1 << 0);                        // left B = input only.

	/* ===============================
	   RIGHT ENCODER SETUP (PTA14/PTA15)
	   =============================== */
	PORTA->PCR[14] = (1 << 8) | (1 << 1) | (1 << 0) | PORT_PCR_IRQC(0x0B); // right A = interrupt
	PORTA->PCR[15] = (1 << 8) | (1 << 1) | (1 << 0);                        // right B = input only

	GPIOA->PDDR &= ~((1 << 6) | (1 << 7) | (1 << 14) | (1 << 15));         // all encoder pins = inputs

	NVIC_EnableIRQ(PORTA_IRQn);   // enable PORTA interrupt

	#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
	BOARD_InitDebugConsole();
	#endif

    // Proof that the program is running
    PRINTF("Hello World\r\n");


    /* ===============================
       TPM0 TIMER SETUP (~10 ms) TPM0 configuration
       =============================== */

    SIM->  SCGC6  |=  (1 << 24);     // enable TPM0 clock
    SIM->  SOPT2  &= ~(3 << 24);     // CLEAR THE MUX FIRST
    SIM->  SOPT2  |=  (2 << 24);     // TPM clock source = OSCERCLK clock

    TPM0->SC  = 0;               // stop timer while configuring
    TPM0->CNT = 0;               // reset counter

    TPM0->MOD = 40000;           // ~10 ms period (approx)

    TPM0->SC |= (1 << 6);        // enable overflow interrupt (TOIE)

    NVIC_EnableIRQ(TPM0_IRQn);   // allow TPM0 interrupt

    TPM0->SC |= (1 << 3)| (1 << 0);        // start timer (CMOD = 1)
    /* ========================================================
       LED SETUP (PTD5)
       ======================================================== */

    /*
      Setup PTD5 as a GPIO output (board LED pin).
      Note: on FRDM-KL46Z PTD5 LED is active-low:
        - PCOR turns LED ON
        - PSOR turns LED OFF
    */
    SIM->SCGC5    |= (1 << 12);    // enable PORTD clock
    PORTD->PCR[5] &=    ~0x700;        // clear MUX bits
    PORTD->PCR[5] |=  (1 << 8);      // set MUX=GPIO
    GPIOD->PDDR   |=  (1 << 5);      // make PTD5 output

    /* ========================================================
       SWITCH SETUP (PORTC)
       SW1 and SW3 are active-low.
       ======================================================== */

    SIM->SCGC5 |= (1 << 11);       // enable PORTC clock

    /*
      SW1 = PTC3
      - set as GPIO
      - enable pull resistor
      - select pull-up
      - set direction as INPUT
    */
    PORTC->PCR[3] &=    ~0x700;     // GPIO mux
    PORTC->PCR[3] |=  (1 << 8);
    PORTC->PCR[3] |=  (1 << 1);  // pull enable
    PORTC->PCR[3] |=  (1 << 0);  // pull-up selected
    GPIOC->PDDR   &= ~(1 << 3); // input

    /*
      SW3 = PTC12 (same idea as SW1)
    */
    PORTC->PCR[12] &=     ~0x700;
    PORTC->PCR[12] |=   (1 << 8);
    PORTC->PCR[12] |=   (1 << 1);
    PORTC->PCR[12] |=   (1 << 0);
    GPIOC->PDDR    &= ~(1 << 12);

    /* ========================================================
       MOTOR GPIO SETUP (direction pins)
       ======================================================== */

    /*
      Enable PORTB and TPM2 clocks.
      (TPM2 used for PWM, PORTB used for direction pins and PWM pins)
    */
    SIM->SCGC5 |= (1 << 10);       // PORTB clock
    SIM->SCGC6 |= (1 << 26);       // TPM2 clock
    /*
      Configure direction pins as GPIO:
      - PTB1 and PTB0 for left motor (AI1/AI2)
      - PTC1 and PTC2 for right motor (BI1/BI2)
    */
    PORTB->PCR[1] &=    ~0x700;
    PORTB->PCR[1] |= (1u << 8);   // PTB1 GPIO

    PORTB->PCR[0] &=   ~0x700;
    PORTB->PCR[0] |= (1u << 8);   // PTB0 GPIO

    PORTC->PCR[1] &=   ~0x700;
    PORTC->PCR[1] |= (1 << 8);    // PTC1 GPIO

    PORTC->PCR[2] &=    ~0x700;
    PORTC->PCR[2] |= (1u << 8);   // PTC2 GPIO

    // Set these direction pins as OUTPUTS
    GPIOB->PDDR   |= (1 << 1) | (1 << 0);
    GPIOC->PDDR   |= (1 << 1) | (1 << 2);

    /*
      Also sets PTB2/PTB3 as outputs.
      Note: these are used as PWM pins when muxed to TPM2_CH0/CH1.
    */

    GPIOB->PDDR   |= (1 << 2) | (1 << 3);

    /* ========================================================
       TEST 0: Toggle direction pins (quick hardware test)
       ======================================================== */
    /*
      This section is just to prove that:
      - the direction pins physically toggle HIGH/LOW
      - the code is executing
      It does NOT run PWM, only direction pins.
    */
    PRINTF("Toggling direction pins...\r\n");

    // Set all direction pins HIGH briefly
    GPIOB->PSOR = (1 << 0) | (1 << 1);
    GPIOC->PSOR = (1 << 1) | (1 << 2);
    Delay(200000);

    // Set all direction pins LOW briefly
    GPIOB->PCOR = (1 << 0) | (1 << 1);
    GPIOC->PCOR = (1 << 1) | (1 << 2);
    Delay(200000);

    PRINTF("Done toggling.\r\n");
    PRINTF("Reached after toggling.\r\n");

    /* ========================================================
       Enable PWM generation for motors
       ======================================================== */
    /*
    Setup TPM2 PWM on PTB2/PTB3, then make sure motors are stopped.
    */
    TPM2GenPWM();
    StopBothMotors();
    Delay(200000);
    //OpenLoopDrivestraight();   // start forward motion

    /* ========================================================
       Variables for button event detection
       ======================================================== */

    // i is just a loop counter (for debugging / tracking)
    volatile unsigned int i = 0;

    /*
      sw1_last and sw3_last store the last “pressed state” so we can detect a NEW press.
      This is called “edge detection”:
        event = pressed_now == 1 AND last == 0
    */

    /* ========================================================
       MAIN LOOP
       ======================================================== */

    while (1) {
        i++;  // count loops

        /* -------- READ BUTTON PINS -------- */

        /*
          Read raw pin values:
          - If button not pressed (active-low), pin reads HIGH (nonzero).
          - If button pressed, pin reads LOW (0).
        */
        int sw1_pin = GPIOC->PDIR & (1 << 3);      // SW1 raw



        // directly copy hardware value
         int SW1_Pressed =sw1_pin;



        /* -------- DEBUG LED -------- */

        // if SW1 pressed ( active-low → 0 = pressed)
        if (SW1_Pressed == 0 && state == 0) {
        	GPIOD -> PCOR = (1 << 5);    // LED ON (Active-low LED)
        	state = 1;  // robot starts moving
        	moveCount = 0;
        	OpenLoopDrivestraight();
        } else {
        	GPIOD->PSOR = (1 << 5);      // LED OFF
        }
    }
}



