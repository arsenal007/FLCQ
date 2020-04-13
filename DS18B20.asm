list P=16F628A
#include <P16F628A.inc>      ; processor specific definitions
#include <timer0.inc>


       global _temperature
UD_DS18B20_0        udata  0x69
COUNT               res 1

UD_DS18B20_1        udata  0x6A
_temperature        res 2

UD_DS18B20_2        udata  0x6C
_delay_counter_0    res 1

UD_DS18B20_3        udata  0x6D
SHIFT               res 1
       extern _leds_over
       global _ds18b20_get_temperature
       global _ds18b20_init
       code
; Dallas DS18B20 Temperature Sensor 'One Wire' Communication Routines

DELAY:                                  ; This routine can provide delays from 
                                        ; 10uS to 669uS depending on the number moved to the Working register prior to calling the delay
        MOVWF   _delay_counter_0        ; [1]    Move integer to GPR
        GOTO    $+D'1'                  ; [2]    No Operation
        GOTO    $+D'1'                  ; [2]    No Operation
        GOTO    $+D'1'                  ; [2]    No Operation
        GOTO    $+D'1'                  ; [2]    No Operation
        GOTO    $+D'1'                  ; [2]    No Operation
        DECFSZ  _delay_counter_0, F     ; [1]    Decrement GPR and place back in itself
        GOTO    $-D'6'                  ; [2](1) Not finished, GOTO here - 3 instructions
        RETURN                          ; [2]    Finished, Return


DQ_HIZ:                                 ; This routine forces the DQ Line to an Input / High Impedance state
        BSF     STATUS, RP0             ; Bank 1
        BSF     TRISA, 1                ; Make Pin 3 an input, Pull-up resistor forces line to logic 1, unless DS18B20 pulls it low
        BCF     STATUS, RP0             ; Bank 0
        RETURN

DQ_LL:                                  ; This routine forces the DQ Line to Logic Low
        BCF     PORTA, 1                ; Clear output latch
        BSF     STATUS, RP0             ; Bank 1
        BCF     TRISA, 1                ; Make Pin 3 an output
        BCF     STATUS, RP0             ; Bank 0
        RETURN



DS18B20_RESET:                          ; This routine Resets the DS18B20
        CALL    DQ_LL                   ; Force the DQ Line to Logic Low
                                        ; (2+1+2*5+1+(2+2*5+1)*184+1+2)*0.2 = 481.8uS      
        MOVLW   D'184'                  ; Reset pulse must be held a minimum of 480uS. 
        CALL    DELAY             
        CALL    DQ_HIZ                  ; Release DQ Line
                                        ; (2+1+2*5+1+(2+2*5+1)*22+1+2)*0.2 = 60.6uS
        MOVLW   D'22'                   ; Wait for recovery
        CALL    DELAY                   ;
        BTFSC   PORTA, 1                ; Test for 'Presence Pulse'
        RETURN
                                        ; (2+1+2*5+1+(2+2*5+1)*180+1+2)*0.2 = 471.4uS
        MOVLW   D'180'                  ; Must wait a minimum of 480uS from when DQ line is released before moving on
        CALL    DELAY                   ;
        RETURN 

DS18B20_WRITE_BIT:                      ; This routine writes one bit of data to the DS18B20
        BCF     INTCON, GIE             ; Disable Global Interrupts
        CALL    DQ_LL                   ; Force the DQ Line to Logic Low
        BTFSS   STATUS, C               ; Test Carry Flag
        GOTO    $+D'2'                  ; If zero, leave DQ Line Logic Low
        CALL    DQ_HIZ                  ; If not zero, release DQ Line to Logic High
                                        ; (2+1+2*5+1+(2+2*5+1)*22+1+2)*0.2 = 60.6uS
        MOVLW   D'18'                   ; Hold Logic Low, write 0
        CALL    DELAY                   ; 
        CALL    DQ_HIZ                  ; If not zero, release DQ Line to Logic High, write 1
        BSF     INTCON, GIE             ; Enable Global Interrupts
        RETURN


DS18B20_WRITE_BYTE:                     ; This routine writes a byte of data to the DS18B20
        MOVWF   SHIFT                   ; Move data to shift into DS18B20 to SHIFT GPR
        MOVLW   D'08'                   ; Amount of bits to shift in
        MOVWF   COUNT                   ; Store in COUNT GPR
        RRF     SHIFT, F                ; Rotate valid data into Carry Flag
        CALL    DS18B20_WRITE_BIT       ; Write bit
        DECFSZ  COUNT, F                ; Decrement COUNT
        GOTO    $-D'3'                  ; If not zero, read next bit
        RETURN                          ; If zero, RETURN
 
DS18B20_READ_BIT:                       ; This routine reads one bit of data from the DS18B20
        BCF     INTCON, GIE             ; Disable Global Interrupts
        CALL    DQ_LL                   ; Force the DQ Line to Logic Low
        CALL    DQ_HIZ                  ; Release DQ Line
                                        ; (2+1+2*5+1+(2+2*5+1)*1+1+2)*0.2 = 6uS
        MOVLW   D'1'+D'1'               ; Hold Logic Low, write 0
        CALL    DELAY 
        BSF     STATUS, C               ; Pre-set Carry Flag
        BTFSS   PORTA, 1                ; Test DQ Line
        BCF     STATUS, C               ; If zero, Clear Carry Flag
                                        ; (2+1+2*5+1+(2+2*5+1)*22+1+2)*0.2 = 50.2uS
        MOVLW   D'18'+D'1'              ; Wait for Time Slot to end
        CALL    DELAY                   ; 
        BSF     INTCON, GIE             ; Enable Global Interrupts
        RETURN


DS18B20_READ_BYTE:                      ; This routine reads a byte of data from the DS18B20
        MOVLW   H'08'                   ; Amount of bits to shift out
        MOVWF   COUNT                   ; Store in COUNT GPR
        CALL    DS18B20_READ_BIT        ; Read bit
        RRF     SHIFT, F                ; Rotate bit out of carry into SHIFT GPR
        DECFSZ  COUNT, F                ; Decrement COUNT
        GOTO    $-D'3'                  ; If not zero, read next bit
        MOVF    SHIFT, W                ; If zero, move SHIFT GPR to W
        RETURN                          ;
  
_ds18b20_init:
        CLRF    PORTA       ; Initialize PORTA by ;setting;output data latches
        MOVLW   0x07        ; Turn comparators off and
        MOVWF   CMCON       ; enable pins for I/O 
                            ; functions
        RETURN   

_ds18b20_get_temperature:
        BANKSEL PORTA
        CALL    DS18B20_RESET           ; Reset

        MOVLW   H'CC'                   ; Skiprom
        CALL    DS18B20_WRITE_BYTE      ; Skiprom
        MOVLW   H'44'                   ; Convert T
        CALL    DS18B20_WRITE_BYTE      ; Convert T
        MOVLW   D'143'
        CALL    DELAY
        MOVLW   D'143'
        CALL    DELAY
        
        
        CALL    DS18B20_RESET           ; Reset
        MOVLW   H'CC'                   ; Skiprom
        CALL    DS18B20_WRITE_BYTE      ; Skiprom
        MOVLW   H'BE'                   ; Read Scratchpad
        CALL    DS18B20_WRITE_BYTE      ; Read Scratchpad

        CALL    DS18B20_READ_BYTE       ; Move Scrachpad Byte 0 to W
        MOVWF   _temperature            ; Store in Celsius TEMPLO GPR
        CALL    DS18B20_READ_BYTE       ; Move Scrachpad Byte 1 to W
        MOVWF   _temperature+1          ; Store in Celsius TEMPHI GPR
        RETURN
        END
