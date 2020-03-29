/*
 * File:   main.c
 * Author: Vasyl
 *
 * Created on March 29, 2020, 8:41 AM
 */

#define _XTAL_FREQ 20000000
#define BAUDRATE 57600

// CONFIG
#pragma config FOSC = \
    HS  // Oscillator Selection bits (HS oscillator: High-speed crystal/resonator on RA6/OSC2/CLKOUT and RA7/OSC1/CLKIN)
#pragma config WDTE = OFF   // Watchdog Timer Enable bit (WDT enabled)
#pragma config PWRTE = OFF  // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = \
    OFF  // RA5/MCLR/VPP Pin Function Select bit (RA5/MCLR/VPP pin function is digital input, MCLR internally tied to VDD)
#pragma config BOREN = OFF  // Brown-out Detect Enable bit (BOD disabled)
#pragma config LVP = \
    OFF  // Low-Voltage Programming Enable bit (RB4/PGM pin has PGM function, low-voltage programming enabled)
#pragma config CPD = OFF  // Data EE Memory Code Protection bit (Data memory code protection off)
#pragma config CP = OFF   // Flash Program Memory Code Protection bit (Code protection off)

// #pragma config statements should precede project file includes.

#include <xc.h>

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
      __delay_us( 2 );
      ti;
      __delay_us( 80 );
    }
    else
    {
      to;
      t = 0;
      __delay_us( 80 );
      ti;
      __delay_us( 2 );
    }  //hold output low if bit is low
  }
  ti;
}

unsigned char sensor_rst( void )
{  //reset the temperature

  to;
  t = 0;
  __delay_us( 600 );
  ti;
  __delay_us( 100 );
  __delay_us( 600 );
  return t;  //return 0 for sensor present
}

unsigned char reply( void )
{  //reply from sensor
  unsigned char ret = 0, i;

  for ( i = 0; i < 8; i++ )
  {  //read 8 bits
    to;
    t = 0;
    __delay_us( 2 );
    ti;
    __delay_us( 6 );
    if ( t )
    {
      ret += 1 << i;
    }
    __delay_us( 80 );  //output high=bit is high
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

uint8_t uart_rx = 0;
uint8_t uart_recive = 0;

void __interrupt() isr( void )
{
  // If UART Rx Interrupt
  if ( RCIF )
  {
    uart_rx = ReceiveByteSerially();
    uart_recive |= 1u;
  }
}

void SendStringSerially( const unsigned char* st )
{
  while ( *st )
    SendByteSerially( *st++ );
}

void blue_enable( void )
{
  PORTBbits.RB5 = 1;
}

void blue_disable( void )
{
  PORTBbits.RB5 = 0;
}

uint8_t eeprom_address;
uint8_t eeprom_data;

uint8_t tempL, tempH;

void main( void )
{
  uint8_t lauflicht = 1;
  TRISBbits.TRISB3 = 0;
  TRISBbits.TRISB4 = 0;
  TRISBbits.TRISB5 = 0;
  PORTBbits.RB3 = 0;
  PORTBbits.RB4 = 0;
  PORTBbits.RB5 = 0;

  InitUART();  // Initialize UART

  sensor_rst();     //sensor init
  cmnd_w( 0xCC );   //skip ROM command
  cmnd_w( 0xBE );   //read pad command
  tempL = reply();  //LSB of temp
  tempH = reply();  //MSB of temp
  sensor_rst();
  sensor_rst();
  cmnd_w( 0xCC );  //skip ROM command
  cmnd_w( 0x44 );  //start convertion command

  SendStringSerially( "FLCQ ver. 1.0\n" );  // Send string on UART

  GIE = 1;   // Enable global interrupts
  PEIE = 1;  // Enable Peripheral Interrupts

  while ( 1 )
  {
    lauflicht <<= 1;

    switch ( uart_recive )
    {
      case ( 1u ):
      {
        if ( uart_rx == 0xBE ) uart_recive = 2u;
        break;
      }
      case ( 3u ):
      {
        switch ( uart_rx )
        {
            // Write EEPROM
          case ( 0x01 ):
          {
            uart_recive = 4u;
            break;
          }
          case ( 0x02 ):
          {
            break;
          }
        }
        break;
      }
      case ( 5u ):
      {
        eeprom_address = uart_recive;
        uart_recive = 6u;
        break;
      }
      case ( 7u ):
      {
        eeprom_data = uart_recive;
        uart_recive = 8u;
        break;
      }
      case ( 9u ):
      {
        if ( uart_recive == 0xEF ) eeprom_write( eeprom_address, eeprom_data );
        while ( WR )
          continue;
        break;
      }
    }

    if ( lauflicht )
    {
      lauflicht = 0;
      PORTBbits.RB3 = 1;
      PORTBbits.RB4 = 1;
      blue_enable();
    }
    else
    {
      lauflicht = 1;
      PORTBbits.RB3 = 0;
      PORTBbits.RB4 = 0;
      blue_disable();
    }

    __delay_ms( 1000 );
  }
}
