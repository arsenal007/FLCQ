list P=16F628A
#include <P16F628A.inc>      ; processor specific definitions

LEDS_PORT      equ  PORTB
LEDS_IO        equ  TRISB
			   global _leds_end
		       global _leds_reg
UD_LEDS_0      udata  0x6E
_leds_end      res	1
UD_LEDS_1      udata  0x6F
_leds_reg      res	1
		
		       global _leds
		       global _leds_over
		       global _leds_set_end
               global _leds_init
               code
_leds_init:
	BANKSEL	LEDS_IO
	BCF	LEDS_IO,3     ; configure as output YELLOW
	BCF	LEDS_IO,4     ; configure as output GREEN
	BCF	LEDS_IO,5     ; configure as output RED
	BANKSEL LEDS_PORT
	BCF	LEDS_PORT,3
	BCF	LEDS_PORT,4
	BSF	LEDS_PORT,5		
	RETURN

_leds_set_end:
	BANKSEL	_leds_end
	MOVWF _leds_end
	RETURN

_leds_over:
	BANKSEL	_leds_end
	MOVF _leds_end,W
_leds:
	BANKSEL	_leds_reg
	MOVWF _leds_reg
NEXT_LED_YELLOW:	
	BTFSS	_leds_reg,3
	GOTO LED_YELLOW_DISABLE
	BSF	PORTB,3
	GOTO NEXT_LED_RED
LED_YELLOW_DISABLE:
	BCF	PORTB,3
NEXT_LED_RED:
	BTFSS	_leds_reg,4
	GOTO LED_RED_DISABLE
	BSF LEDS_PORT,4
	GOTO NEXT_LED_GREEN
LED_RED_DISABLE:	
	BCF	PORTB,4
NEXT_LED_GREEN:
	BTFSS	_leds_reg,5
	GOTO LED_GREEN_DISABLE
	BSF	PORTB,5
	RETURN
LED_GREEN_DISABLE:
	BCF	PORTB,5
	RETURN
	end
