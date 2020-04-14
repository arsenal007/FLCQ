list P=16F628A
#include <P16F628A.inc>      ; processor specific definitions
#include <timer0.inc>
;--------------------------------------------------------------------------
;  Configure the prescaler for TIMER 0 in the PIC's OPTION register .
;--------------------------------------------------------------------------
; Description of the OPTION register, from the PIC16F628 data sheet:
; bit 7: RBPU: PORTB Pull-up Enable bit
;        1 = PORTB pull-ups are disabled
;        0 = PORTB pull-ups are enabled by individual port latch values
; bit 6: INTEDG: Interrupt Edge Select bit
;        1 = Interrupt on rising edge of RB0/INT pin
;        0 = Interrupt on falling edge of RB0/INT pin
; bit 5: T0CS: TMR0 Clock Source Select bit
;        1 = Transition on RA4/T0CKI pin
;        0 = Internal instruction cycle clock (CLKOUT)
; bit 4: T0SE: TMR0 Source Edge Select bit
;        1 = Increment on high-to-low transition on RA4/T0CKI pin
;        0 = Increment on low-to-high transition on RA4/T0CKI pin
; bit 3: PSA: Prescaler Assignment bit
;        1 = Prescaler is assigned to the WDT
;        0 = Prescaler is assigned to the Timer0 module
; bit 2-0: PS2:PS0: Prescaler Rate Select bits, here shown for TMR0 :
;     000  = 1 : 2
; ... 111  = 1 : 256
;        Note: to count EVERY pulse (1 : 1) with TMR0, the prescaler
;              must be assigned to the WATCHDOG TIMER (WDT) !
; Some examples (for the OPTION register, parameter in W for SetPrescaler):
;PSC_DIV_BY_2   equ  b'00100000'   ; let prescaler divide TMR0 by two
;PSC_DIV_BY_4   equ  b'00100001'   ; let prescaler divide TMR0 by   4
;PSC_DIV_BY_8   equ  b'00100010'   ; let prescaler divide TMR0 by   8
;PSC_DIV_BY_16  equ  b'00100011'   ; let prescaler divide TMR0 by  16
;PSC_DIV_BY_32  equ  b'00100100'   ; let prescaler divide TMR0 by  32
;PSC_DIV_BY_64  equ  b'00100101'   ; let prescaler divide TMR0 by  64
;PSC_DIV_BY_128 equ  b'00100110'   ; let prescaler divide TMR0 by 128
;PSC_DIV_BY_256 equ  b'00100111'   ; let prescaler divide TMR0 by 256
          code
          global _SetPrescaler
          global _PrescalerOff
_SetPrescaler:                          ; copy W into OPTION register, avoid watchdog trouble
          ;clrwdt                       ; recommended by Microchip ("switching prescaler assignment") 
;          errorlevel -302              ; Turn off banking message for the next few instructions..

          BANKSEL OPTION_REG            ;! setting RP0 enables access to OPTION reg
                                        ; option register is in bank1. i know. thanks for the warning.
          MOVWF OPTION_REG              ;! ex: "option" command (yucc)
                                        ;! clearing RP0 for normal register access
          RETLW 0
 
 
_PrescalerOff:                         ; turn the prescaler for TMR0 "off" 
                                       ; (actually done by assigning the prescaler to the watchdog timer )
          ;clrwdt                      ; clear watchdog timer
           clrf  TMR0                  ; clear timer 0 AND PRESCALER(!)
          errorlevel -302              ; Turn off banking message for the next few instructions..
          BANKSEL OPTION_REG           ;! setting RP0 enables access to OPTION reg
                                       ; option register is in bank1. i know. thanks for the warning.
          movlw b'00100111'            ;! recommended by Microchip when
                                       ;! changing prescaler assignment from TMR0 to WDT
          movwf OPTION_REG             ;! ex: "option" command (yucc)
          clrwdt                       ;! clear watchdog again
          movlw b'00101111'            ;! bit 3 set means PS assigned to WDT now
          movwf OPTION_REG             ;! ex: "option" command (yucc)

          retlw 0
          end
