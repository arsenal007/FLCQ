list P=16F628A
#include <P16F628A.inc>      ; processor specific definitions
#include <timer0.inc>

UD_DS18B20_0           udata  0x6C
_delay_counter_ms   res	1
UD_DS18B20_1           udata  0x6D
_delay_counter_us   res	1

__delay_us:  ; This routine can provide delays from 1uS to 256uS 
			 ; depending on the number moved to the Working register 
			 ; prior to calling the delay
        MOVWF   _delay_counter_us               ; Move integer to GPR
_loop_us:        
        NOP                             ; No Operation
        NOP                             ; No Operation
        NOP	                            ; No Operation
        NOP	                            ; No Operation
        DECFSZ  _delay_counter_us, F    ; Decrement GPR and place back in itself
        GOTO    _loop_us                ; Not finished, loop
        RETURN                          ; Finished, Return

__delay_ms:  ; This routine can provide delays from 1uS to 256uS 
			 ; depending on the number moved to the Working register 
			 ; prior to calling the delay
	    MOVWF   _delay_counter_ms 
_loop_ms:
        MOVLW   D'200'                  ; Move integer to W
        CALL __delay_us
        MOVLW   D'200'                  ; Move integer to W
        CALL __delay_us                 ; No Operation
        MOVLW   D'200'                  ; Move integer to W
        CALL __delay_us
        MOVLW   D'200'                  ; Move integer to W
        CALL __delay_us
        MOVLW   D'200'                  ; Move integer to W
        CALL __delay_us
        DECFSZ  _delay_counter_ms, F       ; Decrement GPR and place back in itself
        GOTO    _loop                   ; Not finished, loop
        RETURN                          ; Finished, Return