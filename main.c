/*
 * File:   main.c
 * Author: Vasyl
 *
 * Created on March 29, 2020, 8:41 AM
 */

#define _XTAL_FREQ 20000000
#define BAUDRATE 57600

#include <pic16f628a.h>
#include <stdint.h>

extern uint8_t leds_end;
extern uint8_t leds_reg;

extern void leds_init( void );
extern void leds( uint8_t );
extern void leds_set_end( uint8_t );
extern void leds_over( void );

extern uint8_t temperature[2];

extern void ds18b20_get_temperature(void);


#define t RA1          //sensor pin
#define ti TRISA1 = 1  //sensor is output
#define to TRISA1 = 0  //sensor is input

void cmnd_w( unsigned char cmnd )
{  //send command to temperature sensor
  unsigned char i;

  for ( i = 0; i < 8; i++ )
  {  //8 bits
    if ( cmnd & ( 1 << i ) )
    {
      to;
      t = 0;
      //__delay_us( 2 );
      ti;
      //__delay_us( 80 );
    }
    else
    {
      to;
      t = 0;
      //__delay_us( 80 );
      ti;
      //__delay_us( 2 );
    }  //hold output low if bit is low
  }
  ti;
}

unsigned char sensor_rst( void )
{  //reset the temperature

  to;
  t = 0;
  //__delay_us( 600 );
  ti;
  //__delay_us( 100 );
  //__delay_us( 600 );
  return t;  //return 0 for sensor present
}

unsigned char reply( void )
{  //reply from sensor
  unsigned char ret = 0, i;

  for ( i = 0; i < 8; i++ )
  {  //read 8 bits
    to;
    t = 0;
    //delay_us( 2 );
    ti;
    //__delay_us( 6 );
    if ( t )
    {
      ret += 1 << i;
    }
    //__delay_us( 80 );  //output high=bit is high
  }
  ti;
  return ret;
}

void uart_init( void )
{
  TRISB2 = 0;  // TX Pin
  TRISB1 = 1;  // RX Pin
  SPBRG = ( ( _XTAL_FREQ / 16 ) / BAUDRATE ) - 1;
  BRGH = 1;  // Fast baudrate
  SYNC = 0;  // Asynchronous
  SPEN = 1;  // Enable serial port pins
  CREN = 1;  // Enable reception
  SREN = 0;  // No effect
  TXIE = 0;  // Disable tx interrupts
  RCIE = 1;  // Enable rx interrupts
  TX9 = 0;   // 8-bit transmission
  RX9 = 0;   // 8-bit reception
  TXEN = 0;  // Reset transmitter
  TXEN = 1;  // Enable the transmitter
}

uint16_t timer0_overflows;
uint8_t timer0_prescaler = 5;

void SendByteSerially( unsigned char Byte )  // Writes a character to the serial port
{
  while ( !TXIF )
    ;  // wait for previous transmission to finish
  TXREG = Byte;
}

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

#define UART_PACKET_SIZE 16u
uint32_t end;
uint8_t uart[ UART_PACKET_SIZE ];
uint8_t uart_id;

typedef union {
  uint8_t entire;
  struct
  {
    unsigned _RX_FULL_ : 1;           // 0
    unsigned _TIMER0_INTERRUPT_ : 1;  // 1
    unsigned _SERIE_ : 1;             // 2
    unsigned _LED_YELLOW_ : 1;        // 3
    unsigned _LED_BLUE_ : 1;          // 4
    unsigned _LED_GREEN_ : 1;         // 5
    unsigned _UNDIFINED1_ : 1;        // 6
    unsigned _UNDIFINED0_ : 1;        // 7
  };
} FLCQ_STATUS_t;

extern FLCQ_STATUS_t b;

#define RX_FULL b._RX_FULL_
#define TIMER1_INTERRUPT b._TIMER0_INTERRUPT_
#define LED_GREEN b._LED_GREEN_
#define LED_BLUE b._LED_BLUE_
#define LED_YELLOW b._LED_YELLOW_
#define SERIE b._SERIE_

void SetPrescaler( uint8_t _option_reg );
void PrescalerOff( void );

const uint8_t option_reg[ 5 ] = {
  0b00100000,  // PSC_DIV_BY_2
  0b00100001,  // PSC_DIV_BY_4
  0b00100010,  // PSC_DIV_BY_8
  0b00100011,  // PSC_DIV_BY_16
  0b00100100   // PSC_DIV_BY_32
};

void freq_mesure_init( void )
{
  T0SE = 0;  // bit 4 TMR0 Source Edge Select bit 0 = low/high 1 = high/low
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

  TRISA3 = 1;  // TRISA3 = 1 enable it as input
}

void freq_mesure( void )
{
  if ( timer0_prescaler )
    SetPrescaler( option_reg[ --timer0_prescaler ] );
  else
    PrescalerOff();
  freq_mesure_init();
}

void freq_mesure1( void )
{
  timer0_prescaler = 5;
  freq_mesure();
}

void answer( uint8_t N );

uint8_t serie_counter;

void uart_freq( void )
{
  uart[ 0 ] = 0x06;
  uart[ 1 ] = timer0_prescaler;
  uart[ 2 ] = TMR0;
  uart[ 3 ] = ( timer0_overflows >> 8 ) & 0xff;
  uart[ 4 ] = timer0_overflows & 0xff;
  uart[ 5 ] = 0xFF;
  uart[ 6 ] = 0xFF;
  answer( 7 );
  if ( serie_counter )
  {
    serie_counter--;
    freq_mesure_init();
  }
  else
  {
    leds_over();
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
    uart_freq();
  else if ( timer0_overflows < 819 )
    freq_mesure();  // freq_mesure does always prescaler decrement
  else
    uart_freq();
  TIMER1_INTERRUPT = 0;
}

//void __interrupt() isr( void )
void isr( void ) __interrupt 0
{
  // If UART Rx Interrupt
  if ( RCIF )
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
      if ( ( 4 <= uart_id ) && ( uart[ uart_id - 1 ] == 0xFF ) && ( uart[ uart_id - 2 ] == 0xFF ) ) RX_FULL = 1;
    }
  }
  else if ( TMR0IF )
  {
    TMR0IF = 0;
    timer0_overflows++;
  }
  else if ( TMR1IF )
  {
    TRISA3 = 0;
    TMR1ON = 0;  // disable timer
    TMR1IF = 0;
    T0IE = 0;
    TIMER1_INTERRUPT = 1;
  }
}

void SendStringSerially( const unsigned char* st )
{
  while ( *st )
    SendByteSerially( *st++ );
}

extern uint8_t read_eeprom( uint8_t address );
extern void write_eeprom( void );
/*
void write_eeprom( char address, char data )
{
  EEADR = address;
  EEDATA = data;
  WREN = 1;
  GIE = 0;
  EECON2 = 0x55;
  EECON2 = 0xAA;
  WR = 1;
  GIE = 1;
  WREN = 0;
  while ( WR )
    ;
}
*/

void answer( uint8_t N )
{
  uint8_t i;
  for ( i = 0; i < N; i++ )
    SendByteSerially( uart[ i ] );
}

void send_eeprom_value_to_host( void )
{
  uart[ 0 ] = 0x04;
  uart[ 2 ] = read_eeprom( uart[ 1 ] );
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

void uart_rx_packet( void )
{
  switch ( uart[ 0 ] )
  {
    // write eeprom
    case ( 0x03 ):
    {
      uart[ 1 ] &= 0x7F;  //128 max
      if ( uart_id == 5 )
      {
        write_eeprom();
        send_eeprom_value_to_host();
      }
      break;
    }
    // read eeprom
    case ( 0x05 ):
    {
      send_eeprom_value_to_host();
      break;
    }
    case ( 0x07 ):
    {
      leds( uart[ 1 ] );
      leds_set_end( uart[ 2 ] );
      uart[ 3 ]--;
      serie_counter = uart[ 3 ];
      freq_mesure1();
      break;
    }
    case (0x09):
    {
      leds( uart[ 1 ] );
      leds_set_end( uart[ 2 ] );
      ds18b20_get_temperature();
      send_temperature();
      leds_over();      
    }
  }

  uint8_t i;
  for ( i = 0; i < UART_PACKET_SIZE; i++ )
    uart[ i ] = 0;
  uart_id = 0;
  RX_FULL = 0;
}

extern void main_asm(void);

void main(void) {
    main_asm();
  }
