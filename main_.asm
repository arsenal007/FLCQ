list P=16F628A
#include <P16F628A.inc>      ; processor specific definitions
              __CONFIG _CP_OFF&_WDT_OFF&_PWRTE_ON&_HS_OSC&_LVP_OFF&_BODEN_OFF&_MCLRE_OFF
              extern _ds18b20_init
              extern _leds_init
              extern _uart_init
              extern _uart_rx_packet
              extern _timer1_int
              global _main_asm
              global _b

UD_MAIN_0        udata  0x68
_b               res 1

RX_FULL                equ 0
TIMER1_INTERRUPT       equ 1
              code
_main_asm:
              BANKSEL _b
              CLRF   _b
              CALL  _ds18b20_init
              CALL  _leds_init
              CALL  _uart_init
              BSF INTCON, GIE
              BSF INTCON, PEIE
              BANKSEL _b          ; BANK 0
MAIN_LOOP:
              BTFSC _b, RX_FULL              
              CALL  _uart_rx_packet
              BTFSC _b, TIMER1_INTERRUPT
              CALL  _timer1_int
              GOTO MAIN_LOOP
              RETURN    
              END