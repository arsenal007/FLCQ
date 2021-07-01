/*
 * File:   main.c
 * Author: Vasyl
 *
 * Created on March 29, 2020, 8:41 AM
 */

#define _XTAL_FREQ 20000000
#define BAUDRATE 9600

#include <pic16f628a.h>
#include <stdint.h>

#define FW_VERSION 1
#define HW_VERSION 1



extern void leds_init( void );
extern void leds( uint8_t );
extern void leds_set_end( uint8_t );
extern void leds_over( void );
extern void main_asm(void);
extern void SendByteSerially( unsigned char Byte );
extern void ds18b20_get_temperature(void);
extern void frequency_measure_begin( void );
extern uint8_t eeprom_read( uint8_t address );
extern void eeprom_write( void );

void uart_init( void )
{
  TRISB2 = 0;  // TX Pin
  TRISB1 = 1;  // RX Pin
  SPBRG = 32;  // 9600
  BRGH = 0;    // Fast baudrate
  SYNC = 0;    // Asynchronous
  SPEN = 1;    // Enable serial port pins
  CREN = 1;    // Enable reception
  SREN = 0;    // No effect
  TXIE = 0;    // Disable tx interrupts
  RCIE = 1;    // Enable rx interrupts
  TX9 = 0;     // 8-bit transmission
  RX9 = 0;     // 8-bit reception
  TXEN = 0;    // Reset transmitter
  TXEN = 1;    // Enable the transmitter
}




/*void SendByteSerially( unsigned char Byte )  // Writes a character to the serial port
{
  while ( !TXIF )
    ;  // wait for previous transmission to finish
  TXREG = Byte;
}*/

unsigned char ReceiveByteSerially( void )  // Reads a character from the serial port
{
  if ( OERR )  // If over run error, then reset the receiver
  {
    CREN = 0;
    CREN = 1;
  }

  while ( !RCIF )
    ;  // Wait for transmission to receive

  return RCREG;
}

typedef union {
  uint16_t entire;
  struct
  {
    unsigned _RX_FULL_ : 1;           // 0
    unsigned _TIMER0_INTERRUPT_ : 1;  // 1
    unsigned _PRESCALER_SET_ : 1;             // 2
    unsigned _LED_YELLOW_ : 1;        // 3
    unsigned _LED_BLUE_ : 1;          // 4
    unsigned _LED_GREEN_ : 1;         // 5
    unsigned _HICKUP_FREQUENCY_ : 1;        // 6
    unsigned _CONTINUE_MODE_ : 1;        // 7
    unsigned _UNDIFINED2_ : 1;        // 8
    unsigned _UNDIFINED3_ : 1;        // 9
    unsigned _UNDIFINED4_ : 1;        // 10
    unsigned _UNDIFINED5_ : 1;        // 11
    unsigned _UNDIFINED6_ : 1;        // 12
    unsigned _UNDIFINED7_ : 1;        // 13
    unsigned _UNDIFINED8_ : 1;        // 14
    unsigned _UNDIFINED9_ : 1;        // 15
  };
} FLCQ_STATUS_t;


extern uint8_t leds_end;
extern uint8_t leds_reg;
extern uint8_t f_mode;
extern uint32_t timer0_overflows;
extern uint8_t temperature[2];
#define UART_PACKET_SIZE 16u
extern uint8_t uart_id;
extern uint8_t uart[ UART_PACKET_SIZE ];
extern FLCQ_STATUS_t b;
extern uint8_t serie_counter;
extern const uint8_t option_reg[ 5 ];
extern uint8_t timer0_prescaler;


#define RX_FULL b._RX_FULL_
#define TIMER1_INTERRUPT b._TIMER0_INTERRUPT_
#define LED_GREEN b._LED_GREEN_
#define LED_BLUE b._LED_BLUE_
#define LED_YELLOW b._LED_YELLOW_
#define PRESCALER b._PRESCALER_SET_
#define HICKUP_FREQUENCY b._HICKUP_FREQUENCY_
#define CONTINUE_MODE b._CONTINUE_MODE_





void SetPrescaler( uint8_t _option_reg );
void PrescalerOff( void );



void freq_mesure_init( void )
{
  T0SE = 1;  // bit 4 TMR0 Source Edge Select bit 0 = low/high 1 = high/low
  timer0_overflows = 0;

  // Timer1 Registers:
  // Prescaler=1:8; TMR1 Preset=0;
  // aprox: Freq=9.53674Hz; Period=0.1048576 s
  T1CKPS1 = 1;  // bits 5-4  Prescaler Rate Select bits
  T1CKPS0 = 1;
  T1OSCEN = 0;  // bit 3 Timer1 Oscillator Enable Control: bit 1=on
  NOT_T1SYNC =
      1;  // bit 2 Timer1 External Clock Input Synchronization Control bit: 1=Do not synchronize external clock input
  TMR1CS =
      0;  // bit 1 Timer1 Clock Source Select bit: 0=Internal clock (FOSC/4) / 1 = External clock from pin T1CKI (on the rising edge)
  TMR1IE = 1;
  TMR1H = 0;   // preset for timer1 MSB register
  TMR1L = 0;   // preset for timer1 LSB register
               //
  TMR1ON = 1;  // bit 0 enables timer

  TMR0 = 0;
  T0IE = 1;

  TRISA4 = 1;  // TRISA3 = 1 enable it as input
}

void freq_mesure( void )
{
  if ( timer0_prescaler )
    SetPrescaler( option_reg[ --timer0_prescaler ] );
  else
    PrescalerOff();
  freq_mesure_init();
}


/*{
  timer0_prescaler = 5;
  freq_mesure();
}*/

void answer( uint8_t N );

void uart_freq( void )
{
  uart[ 0 ] = 0x06;
  uart[ 1 ] = timer0_prescaler;
  uart[ 2 ] = TMR0;
  uart[ 3 ] = timer0_overflows & 0xff;
  uart[ 4 ] = ( timer0_overflows >> 8 ) & 0xff;
  uart[ 5 ] = ( timer0_overflows >> 16 ) & 0xff;
  uart[ 6 ] = ( timer0_overflows >> 24 ) & 0xff;
  uart[ 7 ] = 0xFF;
  uart[ 8 ] = 0xFF;
  answer( 9 );
}

void frequency_end( void ) {
 if (PRESCALER) {
  PRESCALER = 0;
  freq_mesure_init();
 } else {
  uart_freq();
  if ( serie_counter ) {
    serie_counter--;
    freq_mesure_init();
  }
  else {
    leds_over();
  }
 }
}

// calls after interrupt of timer1
// register TMR0, timer0_overflows and timer0_prescaler
// contains raw data
void timer1_int( void )
{
  // Max input frequency 4Mhz * 0.1048576 = 419430.4
  // 419430.4 = 1638 * 256 + 102.4
  // 1638 maximal overflow count for timer0
  // we have good resolution,
  // when we are measuring at input frequencies 2..4Mhz
  // so when our overflow counter is between 1638/2=819...1638

  if ( timer0_prescaler == 0 )
    frequency_end();
  else if ( timer0_overflows < 819 )
    freq_mesure();  // freq_mesure does always prescaler decrement
  else
    frequency_end();
  TIMER1_INTERRUPT = 0;
}


void isr( void ) 
{
  if ( TMR1IF )
  {
    if ( HICKUP_FREQUENCY ) {
       TRISA4 = 0;
       TMR1ON = 0;
       TMR1IF = 0;
       T0IE = 0;
       TIMER1_INTERRUPT = 1;
    } else if ( CONTINUE_MODE ) {
       if ( PRESCALER ) {
          TRISA4 = 0;
          TMR1ON = 0;
          T0IE = 0;
          TIMER1_INTERRUPT = 1;
       } else if ( serie_counter ) {
          serie_counter--;
       } else {
          TRISA4 = 0;
          TMR1ON = 0;
          T0IE = 0;
          TIMER1_INTERRUPT = 1;
       }
       TMR1IF = 0;
    }
  }
  else if ( TMR0IF )
  {
    TMR0IF = 0;
    timer0_overflows++;
  }
  else if ( RCIF )
  {
    if ( RX_FULL )
    {
      ReceiveByteSerially();  // dummy read
    }
    else
    {
      uart[ uart_id ] = ReceiveByteSerially();
      uart_id++;
      uart_id &= 0b00001111;
      if ( ( 3 < uart_id ) && ( uart[ uart_id - 1 ] == 0xFF ) && ( uart[ uart_id - 2 ] == 0xFF ) ) RX_FULL = 1;
    }
  }

}



void answer( uint8_t N )
{
  uint8_t i;
  for ( i = 0; i < N; i++ )
    SendByteSerially( uart[ i ] );
}

void send_eeprom_value_to_host( uint8_t a )
{
  uart[ 0 ] = 0x04;
  uart[ 1 ] = eeprom_read( a );
  uart[ 2 ] = a;
  uart[ 3 ] = 0xFF;
  uart[ 4 ] = 0xFF;
  answer( 5 );
}

void send_temperature( void )
{
  uart[ 0 ] = 0x0A;
  uart[ 1 ] = temperature[1];
  uart[ 2 ] = temperature[0];
  uart[ 3 ] = 0xFF;
  uart[ 4 ] = 0xFF;
  answer( 5 );
}

void send_fw_version( void )
{
  uart[ 0 ] = 0x0E;
  uart[ 1 ] = FW_VERSION;
  uart[ 2 ] = HW_VERSION;
  uart[ 3 ] = 0xFF;
  uart[ 4 ] = 0xFF;
  answer( 5 );
}

void frequency_measure( void )  {
      leds( uart[ 1 ] );
      leds_set_end( uart[ 2 ] );
      uart[ 3 ]--;
      serie_counter = uart[ 3 ];
      frequency_measure_begin();
      PRESCALER = 1;
}


void uart_rx_packet( void )
{

  switch ( uart[ 0 ] )
  {
    // write eeprom
    case ( 0x03 ):
    {
      uart[ 2 ] &= 0x7F;  //128 max
      if ( uart_id == 5 )
      {
        eeprom_write();
        send_eeprom_value_to_host(uart[2]);
      }
      break;
    }
    // read eeprom
    case ( 0x05 ):
    {

      uart[ 1 ] &= 0x7F;  //128 max
      send_eeprom_value_to_host(uart[1]);
      break;
    }
    case ( 0x07 ):
    {
      CONTINUE_MODE = 0;
      HICKUP_FREQUENCY = 1;
      frequency_measure();
      break;
    }
    case (0x09):
    {
      leds( uart[ 1 ] );
      leds_set_end( uart[ 2 ] );
      ds18b20_get_temperature();
      send_temperature();
      leds_over();      
      break;
    }
    case (0x0B):
    {
      CONTINUE_MODE = 1;
      HICKUP_FREQUENCY = 0;
      frequency_measure();
      break;
    }
    case (0x0D):
    {
       leds( uart[ 1 ] );
       send_fw_version();
       break;
    }
  }

  uint8_t i;
  for ( i = 0; i < UART_PACKET_SIZE; i++ )
    uart[ i ] = 0;
  uart_id = 0;
  RX_FULL = 0;
}
