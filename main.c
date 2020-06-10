#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>


// Define main clock frequency
#define  F_CPU   1000000UL
#include <util/delay.h>

static volatile uint16_t cntISR = 0;
const int LED = 0;
const int SIGNAL = 2;

#define SET_LED_ON()   PORTB &= ~(_BV( LED ))
#define SET_LED_OFF()  PORTB |= _BV( LED )
#define TOGGLE_LED()   PINB  |= _BV( LED )

#define SET_SIGNAL_ON()   PORTB |= _BV( SIGNAL )
#define SET_SIGNAL_OFF()  PORTB &= ~(_BV( SIGNAL ))

// Watchdog Interrupt Service / is executed when watchdog timed out
ISR(WDT_vect) {
    WDTCR |= _BV( WDIE );
    cntISR++;
    // Stay asleep for 4 hours
    if( cntISR == 3200 )
        cntISR = 0;
}
 
void system_sleep( )
{
  // Choose power down sleep mode
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  cli();
  if( 1 ) {
      sleep_enable();
      sleep_bod_disable();
      sei();
      sleep_cpu();
      sleep_disable();
  }
  sei();
}

void config_watchdog( )
{
    cli();
    wdt_reset();
    WDTCR |= (1<<WDCE)|(1<<WDE);

    // Set Watchodg Timer to 8 seconds
    //WDTCR |= (1<<WDIE)|(1<<WDE)|(1<<WDP3)|(1<<WDP0);
    // Set watchdog timer to 4 seconds
    WDTCR |= (1<<WDIE)|(1<<WDE)|(1<<WDP3);
    sei();
}

void config_io( )
{
    // Enable Pull up on all pins to reduce power consumption
    PORTB = 0xFF;
 
    // Configure output pin
    DDRB |= _BV( LED ) | _BV( SIGNAL );
}
 
int main( int argc, char **argv )
{
    // Init
    config_io();
    config_watchdog();
   
    // Halt clock signals to reduce power consumption
    PRR = _BV( PRTIM1 ) | _BV( PRTIM0 ) | _BV( PRUSI ) | _BV( PRADC );

    SET_LED_OFF();
    // Main loop
    while( 1 ) {
        if( cntISR == 0 )
        {
                SET_LED_ON();
                SET_SIGNAL_ON();
                // Leave the signal ON for 12 seconds
                while( cntISR < 3 );
                SET_SIGNAL_OFF();
                SET_LED_OFF();
        }
        // Return to sleep
        system_sleep( );
    }
}

