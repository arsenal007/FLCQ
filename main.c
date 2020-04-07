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

void led_yellow_enable( void )
{
  RB3 = 1;
}

void led_yellow_disable( void )
{
  RB3 = 0;
}

void led_green_enable( void )
{
  RB4 = 1;
}

void led_green_disable( void )
{
  RB4 = 0;
}

void led_blue_enable( void )
{
  RB5 = 1;
}

void led_blue_disable( void )
{
  RB5 = 0;
}

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
  // Prescaler=1:8; TMR1 Preset=59286; Freq=100.00Hz; Period=10,000,000 ns
  T1CKPS1 = 1;  // bits 5-4  Prescaler Rate Select bits
  T1CKPS0 = 1;
  T1OSCEN = 0;  // bit 3 Timer1 Oscillator Enable Control: bit 1=on
  NOT_T1SYNC =
      1;  // bit 2 Timer1 External Clock Input Synchronization Control bit: 1=Do not synchronize external clock input
  TMR1CS =
      0;  // bit 1 Timer1 Clock Source Select bit: 0=Internal clock (FOSC/4) / 1 = External clock from pin T1CKI (on the rising edge)
  TMR1IE = 1;
  TMR1ON = 1;    // bit 0 enables timer
  TMR1H = 0xE7;  // preset for timer1 MSB register
  TMR1L = 0x96;  // preset for timer1 LSB register
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
uint32_t end;
uint8_t uart[ UART_PACKET_SIZE ];
uint8_t uart_id;
uint8_t status;
uint16_t timer1;
uint8_t lauflicht;

void timer1_int( void )
{
  if ( timer1 == 100u )
  {
    if ( lauflicht )
    {
      lauflicht = 0;
      led_yellow_enable();
    }
    else
    {
      lauflicht = 1;
      led_yellow_disable();
    }
    timer1 = 0;
  }
  else
  {
    timer1++;
  }
}

//void __interrupt() isr( void )
void isr( void ) __interrupt 0
{
  // If UART Rx Interrupt
  if ( RCIF )
  {
    if ( ( ( status >> RX_FULL ) & 1U ) == 0 )
    {
      uart[ uart_id ] = ReceiveByteSerially();
      uart_id++;
      uart_id &= 0b00001111;
      if ( ( 4 <= uart_id ) && ( uart[ uart_id - 1 ] == 0xFF ) && ( uart[ uart_id - 2 ] == 0xFF ) )
      {
        status |= ( 1 << RX_FULL );
      }
    }
    else
    {
      ReceiveByteSerially();
    }
  }
  else if ( TMR0IF )
  {
    TMR0IF = 0;
    TMR0 = 0;
    timer0++;
  }
  else if ( TMR1IF )
  {
    TMR1IF = 0;
    TMR1H = 0xE7;  // preset for timer1 MSB register
    TMR1L = 0x96;  // preset for timer1 LSB register
    timer1_int();
  }
}

void SendStringSerially( const unsigned char* st )
{
  while ( *st )
    SendByteSerially( *st++ );
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

void uart_rx_packet( void )
{
  switch ( uart[ 0 ] )
  {
    // write eeprom
    case ( 0x03 ):
    {
      led_yellow_enable();
      uart[ 1 ] &= 0x7F;  //128 max
      if ( uart_id == 5 )
      {
        write_eeprom( uart[ 1 ], uart[ 2 ] );
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
  }
  led_blue_enable();
  status &= ~( 1 << RX_FULL );
  uint8_t i;
  for ( i = 0; i < UART_PACKET_SIZE; i++ )
    uart[ i ] = 0;
  uart_id = 0;
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
  initTimer1();
  //sensor_rst();     //sensor init
  //cmnd_w( 0xCC );   //skip ROM command
  //cmnd_w( 0xBE );   //read pad command
  //tempL = reply();  //LSB of temp
  //tempH = reply();  //MSB of temp
  //sensor_rst();
  //sensor_rst();
  //cmnd_w( 0xCC );  //skip ROM command
  //cmnd_w( 0x44 );  //start convertion command

  //SendStringSerially( "FLCQ ver. 1.0\n" );  // Send string on UART

  GIE = 1;   // Enable global interrupts
  PEIE = 1;  // Enable Peripheral Interrupts
  led_yellow_disable();
  led_green_disable();
  led_blue_disable();
  while ( 1 )
  {
    if ( ( status >> RX_FULL ) & 1U ) uart_rx_packet();
    //__delay_ms( 1000 );
  }
}
