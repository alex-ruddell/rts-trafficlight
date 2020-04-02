/*
 * Assignment 1 -- Traffic Light Control System
 * A spiritual journey
 * Created: 09/04/2019
 * Author : arud699 and rhug194
 */ 
#define F_CPU 1000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// =================
// MACRO DEFINITIONS
// =================

#define HALF_SECOND 1953
#define ONE_SECOND 3906

// =====================
// FUNCTION DECLARATIONS
// =====================

void normalMode(void);
void configurationMode(void);
void redLightCamera(void);
void switchPoll(void);

// ============================
// Global variable declarations
// ============================

// Variable structure for tracking light colour
enum COLOURS { RED, YELLOW, GREEN };
enum COLOURS colour; 
// Counter variables -- used as time delay counters
uint16_t normCount = ONE_SECOND;
uint16_t configCount = 0;
uint16_t redLightCount = 0;
uint16_t speedCount = 0;
// State variables -- describe the state of the system
uint8_t configMode = 0; // System begins in normal mode
uint8_t step = 1; // Initialise in default state
uint8_t redLightMode = 0; // System begins in normal mode
// Switch variables -- describe the switch state
uint8_t SW0 = 0; // Switch0 debouncing variable
uint8_t SW5 = 0; // Conditional for performing speed barrier function
uint8_t SW7 = 0; // Switch7 debouncing variable
// Other variables
uint8_t redLightFlash = 0; // Tracks number of red light camera LED flashes
uint16_t numVehicles = 0; // Tracks the number of vehicles to trigger the red light camera
uint16_t barrierVelocity = 0; // Stores the calculated velocity of the car
uint8_t count; // Variable used to flash the configMode LED.
uint16_t adcInput; // Initialise the storage variable for ADC input.

// ========================
// Interrupt implementation
// ========================
// Hardware interrupt 0:
// Used as a sensor to trigger Light Barrier 1 (LB1) being crossed.
ISR(INT0_vect)
{
	SW5 = 1; // LB1 is set as crossed.
	speedCount = 0; // Resets the number of overflows for this one.
	OCR2 = 0;
}

// Hardware interrupt 1:
// Used as a sensor to trigger Light Barrier 2 (LB2) being crossed.
ISR(INT1_vect) 
{
	// LB2 is only expected to have been crossed if LB1 has been prior.
	if (SW5) {
		// LB1 must once again be pressed before entering this function again, so set as 0 or "false".
		SW5 = 0;
		// Velocity calculation:
		// Velocity = distance / time elapsed = 20 / (numOverflows / numOverflows per second)
		// Multiplied by 3.6 to convert to km/hr, and scaled by 2.55 to convert to a value out of 255.
		barrierVelocity = (20*3.6*2.55 * ONE_SECOND) / speedCount;
		if (barrierVelocity > 255) {
			barrierVelocity = 255;
		}
		OCR2 = barrierVelocity;
	}
}

// Timer 0 overflow interrupt:
// This timer overflows every ~250us. Global counter variables are reset and measured at different 
// times to time our system.
ISR(TIMER0_OVF_vect)
{	
	normCount++; 		// Counter used for timing normal mode operation.
	configCount++;		// Counter used for timing configuration mode operation.
	redLightCount++;		// Counter used for timing red light mode operation.
	speedCount++;		// Counter used for calculation of the velocity between LB1 and LB2.
}


int main(void)
{
	// Global interrupt enable flag.
	sei();
	
	// =================
	// TIMER SET UP CODE
	
	// Timer 0:
	TIMSK |= (1<<TOIE0); // Enable timer0 overflow interrupt.
	TCCR0 |= (1<<CS00); // Sets the prescaler to 1.
	// Timer 1:
	TCCR1A |= (1<<WGM10)|(1<<WGM11); // WGM13:WGM10 = 0011 for 10-bit fast PWM mode.
	TCCR1B &= ~(1<<WGM12);
	TCCR1B &= ~(1<<WGM13);
	TCCR1A |= (1<<COM1A1);	// OCR1A PWM set as non-inverting.
	TCCR1A &= ~(1<<COM1A0);
	TCCR1B |= (1<<COM1B1);
	TCCR1B &= ~(1<<COM1B0);	// OCR1B PWM set as non-inverting.
	TCCR1B |= (1<<CS11)|(1<<CS10);		// Prescalar set as 64.
	OCR1A = 0; // Initialised as a 0% duty cycle.
	// Timer 2:
	TCCR2 |= (1<<CS20); // No prescalar used.
	TCCR2 |= (1<<WGM20)|(1<<WGM21); // Set as fast PWM mode.
	TCCR2 |= (1<<COM21); // Sets compare to match, non-inverting.
	OCR2 = 0; // Initialised as a 0% duty cycle.

	// ===============
	// ADC SET UP CODE

	ADMUX = 0; // Set MUX3:0 as 0:0.
	ADCSRA |= (1<<ADFR); // Enable free-running mode.
	ADCSRA &= ~(1<<ADPS2); //ADPS2:0 = 011, prescalar of 8.
	ADCSRA |= (1<<ADPS1);
	ADCSRA |= (1<<ADPS0);
	ADCSRA |= (1<<ADEN); // Enable the ADC functionality.
	ADCSRA |= (1<<ADSC); // Start the ADC conversion.

	// ================
	// PORT SET UP CODE

	// Port D used as input port.
	// PD0 used for switch0 (config mode switch), PD1 used for switch7 (LB3).
	// PD2 (int0) used for switch5 (LB1), PD3 (int1) used for switch6 (LB2).
	DDRD = 0x00;
	
	// Port C used as output port.
	// PC1 through PC5 are used for LED0 through LED4.
	// PC0, GND, and VTG are used for the ADC.
	//DDRC = 0xFF; 
	DDRC = 254;
	PORTC = 0xFF; // Initialises all LEDs as off.
	
	
	// Port B output ports for output PWMs
	// PB1 (OCR1A), PB2 (OCR1B [not using currently]), and PB3 (OCR2) used as PWM outputs.
	DDRB = 0xFF;
	PORTB = 0xFF; // Initialise as high.

	// ==============================
    // HARDWARE INTERRUPT SET UP CODE

	GICR = (1<<INT0)|(1<<INT1); // Enable the hardware interrupts.
	MCUCR &= ~(1<<ISC00); // Falling edge interrupt trigger
	MCUCR |= (1<<ISC01); 
	MCUCR &= ~(1<<ISC10); // Falling edge interrupt trigger
	MCUCR |= (1<<ISC11); 

	// =========
	// SUPERLOOP

	colour = RED; // Start operation from in RED mode. 
	while (1) {
		switchPoll(); // Poll for switches being pressed.
		redLightCamera(); // Constantly running red light camera functionality in background.
		normalMode(); // A single, loopless function called repetetively. 
	}
	
	return 0;
}

// NORMAL MODE FUNCTION
// Run in the main while loop as the "superloop" code functionality.
// Performs the base system function.
void normalMode(void) {	
	if (normCount >= ONE_SECOND*step) { // Iterate the system state every number of seconds, defined by step.
		switch(colour) {
			case YELLOW: {			// Case when YELLOW is on.
				PORTC |= (1<<PC2); 	// TURN OFF YELLOW.
				PORTC &= ~(1<<PC1); // TURN ON RED.
				colour = RED;		// Set next system state.
				break;
			}
			case GREEN: {			// Case when GREEN is on.
				PORTC |= (1<<PC3); 	// TURN OFF GREEN.
				PORTC &= ~(1<<PC2); // TURN ON YELLOW.
				colour = YELLOW;	// Set next system state.
				break;
			}
			case RED: {				// Case when RED is on.
				configurationMode();// Function only runs when configMode == 1.
				PORTC |= (1<<PC1); 	// TURN OFF RED.
				PORTC &= ~(1<<PC3); // TURN ON GREEN.
				colour = GREEN;		// Set next system state.
				break;
			}
		}
		normCount = 0; // Reset the normal mode counting variable.
	}
}

// CONFIGURATION MODE FUNCTION
// Used to configure the frequency of the traffic light phases.
void configurationMode(void) {
	PORTC |= (1<<PC5); // Initialise the configMode LED as off.
	while(configMode) {
		adcInput = ADC; // Read the value from the ADC.
		step = (adcInput / 256) + 1; // Time period delay calculation.
		count = step * 2; // Set number of flashes required.
		configCount = 0; // Initialise the counting variable.
		while ((count > 0) && (configMode)) {
			switchPoll(); // Always polling to exit configuration mode.
			redLightCamera(); // Red light camera always working in background.
			if (configCount >= HALF_SECOND) { // Loop to track LED flashes.
				PORTC ^= (1<<PC5); // Toggle the configMode LED.
				count--;
				configCount = 0;
			}
		}
		configCount = 0;
		PORTC |= (1<<PC5); // Ensure the LED is off.
		while ((configCount <= ONE_SECOND*3) && (configMode)) { // Delay between flashes.
			switchPoll(); // Always polling to exit configuration mode.
			redLightCamera(); // Red light camera always working in background.
		}
	}
}

// RED LIGHT CAMERA FUNCTION
// Performs functionality of the red light camera.
// Used to set the PWM values and give an output.
void redLightCamera(void) {
	if (redLightMode) {
		if (numVehicles > 100) { // Saturation value of 100 is used. 
			OCR1A = 0x3FF; // 100% PWM.
		} else {
			OCR1A = numVehicles*10; // Number of vehicles is scaled to give a PWM percentage.
		}
		if (redLightCount >= HALF_SECOND) { // Performes the signature two flashes for the camera.
			redLightFlash++; // Tracks how many toggles have occurred.
			PORTC ^= (1<<PC4);
			redLightCount = 0;
			if (redLightFlash >= 3) { // Want two flashes only.
				redLightMode = 0; // Disable the flashing once flashes complete.
				redLightFlash = 0; // Reset tracking variable.
				PORTC |= (1<<PC4); // Turn the light off.
			}
		}
	}
}

// SWITCH POLL FUNCTION
// Used to monitor and debounce the switches for polling.
// Also includes some minor functionality relating to redLightCamera() for efficiency purposes.
void switchPoll(void) {
	if ((PIND & (1<<PD0)) != 0) { // When the switch is not pressed.
		SW0 = 0; // Sets switch testing variable as "off".
	}
	if (((PIND & (1<<PD0)) == 0) && (SW0 == 0)) { // Debouncing, only changes if previously "off".
		SW0 = 1; // If pressed, sets switch testing variable as "on".
		configMode = !configMode; // Toggles configuration mode.
	}
	
	if ((PIND & (1<<PD1)) != 0) { // When the switch is not pressed.
		SW7 = 0;	 // Sets switch testing variable as "off".
	}
	if (((PIND & (1<<PD1)) == 0) && (SW7 == 0)) { // Debouncing, only changes if previously "off".
		SW7 = 1; // If pressed, sets switch testing variable as "on".
		if (colour == RED) { // Red light mode is incurred only when on a red light phasae.
			PORTC ^= (1<<PC4); // Instant red light flash response.
			redLightCount = 0; 
			redLightMode = 1; // Enables the signature red light flashing.
			numVehicles++; // Increments the number of vehicles to have passed the barrier.
		}
	}
}
