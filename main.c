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
    //__delay_us( 2 );
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

void InitUART( void )
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

uint8_t timer0 = 0;

uint8_t prescaler_timer0 = 0;

void initTimer0( void )
{
  T0IE = 0;
  //Timer0 Registers Prescaler= 256 - TMR0 Preset = 255 - Freq = 19531.25 Hz - Period = 0.000051 seconds
  T0CS = 1;  // bit 5  TMR0 Clock Source Select bit...0 = Internal Clock (CLKO) 1 = Transition on T0CKI pin
  T0SE = 0;  // bit 4 TMR0 Source Edge Select bit 0 = low/high 1 = high/low
  PSA = 0;   // bit 3  Prescaler Assignment bit...0 = Prescaler is assigned to the Timer0

  if ( prescaler_timer0 & ( 1 << ( 2 ) ) )
  {
    PS2 = 1;  // bits 2-0  PS2:PS0: Prescaler Rate Select bits
  }
  else
  {
    PS2 = 0;
  }

  if ( prescaler_timer0 & ( 1 << ( 1 ) ) )
  {
    PS1 = 1;  // bits 2-0  PS2:PS0: Prescaler Rate Select bits
  }
  else
  {
    PS1 = 0;
  }

  if ( prescaler_timer0 & 1 )
  {
    PS0 = 1;  // bits 2-0  PS2:PS0: Prescaler Rate Select bits
  }
  else
  {
    PS0 = 0;
  }

  timer0 = 0;
  TMR0 = 0;
  T0IE = 1;
}

void initTimer1( void )
{
  // Timer1 Registers:
  // Prescaler=1:1; TMR1 Preset=65036; Freq=10,000.00Hz; Period=100,000 ns
  //T1CON.T1CKPS1 = 0;  // bits 5-4  Prescaler Rate Select bits

  //T1CON.T1CKPS0 = 0 T1CON.T1OSCEN = 1;  // bit 3 Timer1 Oscillator Enable Control: bit 1=on
  //T1CON.T1SYNC =
  //    1;  // bit 2 Timer1 External Clock Input Synchronization Control bit: 1=Do not synchronize external clock input
  //T1CON.TMR1CS =
  //   0;  // bit 1 Timer1 Clock Source Select bit: 0=Internal clock (FOSC/4) / 1 = External clock from pin T1CKI (on the rising edge)
  //T1CON.TMR1ON = 1;  // bit 0 enables timer
  //TMR1H = 0xFE;      // preset for timer1 MSB register
  //TMR1L = 0x0C;      // preset for timer1 LSB register
}

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
#define RX_FULL 7
uint8_t uart_rx[ UART_PACKET_SIZE ];
uint8_t uart_id;
uint8_t status;

//void __interrupt() isr( void )
void isr(void) __interrupt 1 {
  // If UART Rx Interrupt
  if ( RCIF )
  {
    uint8_t rx = ReceiveByteSerially();
    if ( rx == 0xBE )
    {
      uart_id = 0;
    }
    else if ( rx == 0xFE )
    {
      status |= ( 1 << RX_FULL );
    }
    else
    {
      uart_rx[ uart_id ] = rx;
      uart_id++;
      uart_id &= 0b00001111;
    }
  }
  else if ( TMR0IF )
  {
    TMR0IF = 0;
    TMR0 = 0;
    timer0++;
  }
}

void SendStringSerially( const unsigned char* st )
{
  while ( *st )
    SendByteSerially( *st++ );
}

void blue_enable( void )
{
  RB5 = 1;
}

void blue_disable( void )
{
  RB5 = 0;
}

uint8_t eeprom_address;
uint8_t eeprom_data;
uint8_t tempL, tempH;

uint8_t read_eeprom( uint8_t address );

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

void answer( const unsigned char* st )
{
  while ( *st != 0xFE )
    SendByteSerially( *st++ );
  //0xFE
  SendByteSerially( *st );
}

void send_eeprom_value_to_host( uint8_t address )
{
  uint8_t p[] = { 0xBE, 0x04, 0x00, 0x00, 0xFE };
  p[ 2 ] = address;
  p[ 3 ] = read_eeprom( address );
  answer( p );
}

void uart_rx_packet( void )
{
  switch ( uart_rx[ 0u ] )
  {
    // write eeprom
    case ( 0x03 ):
    {
      write_eeprom( uart_rx[ 1u ], uart_rx[ 2u ] );
      send_eeprom_value_to_host( uart_rx[ 1u ] );
      break;
    }
    // read eeprom
    case ( 0x05 ):
    {
      send_eeprom_value_to_host( uart_rx[ 1u ] );
      break;
    }
  }
  status &= ( 1 << RX_FULL );
  uint8_t i;
  for ( i = 0; i < UART_PACKET_SIZE; i++ )
    uart_rx[ i ] = 0;
}

void main( void )
{
  uint8_t lauflicht = 1;
  TRISB3 = 0;
  TRISB4 = 0;
  TRISB5 = 0;
  RB3 = 0;
  RB4 = 0;
  RB5 = 0;

  InitUART();  // Initialize UART

  //sensor_rst();     //sensor init
  //cmnd_w( 0xCC );   //skip ROM command
  //cmnd_w( 0xBE );   //read pad command
  //tempL = reply();  //LSB of temp
  //tempH = reply();  //MSB of temp
  //sensor_rst();
  //sensor_rst();
  //cmnd_w( 0xCC );  //skip ROM command
  //cmnd_w( 0x44 );  //start convertion command

  SendStringSerially( "FLCQ ver. 1.0\n" );  // Send string on UART

  GIE = 1;   // Enable global interrupts
  PEIE = 1;  // Enable Peripheral Interrupts

  while ( 1 )
  {
    lauflicht <<= 1;

    if ( ( status >> RX_FULL ) & 1U ) uart_rx_packet();

    if ( lauflicht )
    {
      lauflicht = 0;
      RB3 = 1;
      RB4 = 1;
      blue_enable();
    }
    else
    {
      lauflicht = 1;
      RB3 = 0;
      RB4 = 0;
      blue_disable();
    }

    //__delay_ms( 1000 );
  }
}
