list P=16F628A
#include <P16F628A.inc>      ; processor specific definitions
              global _eeprom_read
              global _eeprom_write
              extern _uart
              code
; W has address to read from
_eeprom_read:
              BANKSEL   EEADR        ; select memory bank 1
              MOVWF     EEADR        ; Скопировать 02h, из регистра W,
                                     ; в регистр EEAdr.
              BSF       EECON1, RD   ; Инициализировать чтение.
              MOVF      EEDATA, W    ; W = EEDATA
              RETURN
; _uart+1 has address to read from
; _uart+2 has a data to write
_eeprom_write:
              BANKSEL	  _uart        ; select bank 0
              MOVF      (_uart+1), W ; set address
              BANKSEL   EEADR        ; select Bank 1
              MOVWF	    EEADR
              BANKSEL	  _uart        ; select bank 0
              MOVF	    (_uart+2), W ; set data byte
              BANKSEL   EEDATA       ; select bank 1
              MOVWF	    EEDATA
              BSF       EECON1, WREN ; enable write
              BCF       INTCON, GIE  ; disable global interrupts
              BTFSC     INTCON, GIE  ; see AN576
              GOTO      $-D'2'
              MOVLW     0x55         ;
              MOVWF     EECON2       ; write 55h
              MOVLW     0xAA         ;
              MOVWF     EECON2       ; write AAh
              BSF       EECON1, WR   ; Set WR bit
                                     ; begin write
              BSF       INTCON, GIE  ; enable global interrupts
              BCF       EECON1, WREN
              BTFSC     EECON1, WR
              GOTO      $-D'1'
              RETURN	               ; exit point of _write_eeprom             
              END
