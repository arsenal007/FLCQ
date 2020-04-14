list P=16F628A
#include <P16F628A.inc>      ; processor specific definitions
              global _read_eeprom
              global _write_eeprom
              extern _uart
              code
; W has address to read from
_read_eeprom:
              BANKSEL   EEADR        ; jump to bank 1
              MOVWF     EEADR        ; Скопировать 02h, из регистра W,
                                     ; в регистр EEAdr.
              BSF       EECON1, 0    ; Инициализировать чтение.
              MOVF      EEDATA, W    ; W = EEDATA
              RETURN
; _uart+1 has address to read from
; _uart+2 has a data to write
_write_eeprom:
              BANKSEL	_uart        ; select bank 0?
              MOVF      _uart+1, W   ; set address
              BSF       STATUS, RP0  ; select Bank 1
              MOVWF	    EEADR
              BANKSEL	_uart
	          MOVF	    _uart+2, W    ; set data byte
	          BSF       STATUS, RP0  ; select Bank 1
              MOVWF	    EEDATA
              BSF       EECON1, WREN ; enable write
              BCF       INTCON, GIE  ; disable global interrupts
              BTFSC     INTCON, GIE  ; see AN576
              GOTO      $-D'2'
              MOVLW     H'55'        ;
              MOVWF     EECON2       ; write 55h
              MOVLW     H'AA'        ;
              MOVWF     EECON2       ; write AAh
              BSF       EECON1, WR   ; Set WR bit
                                     ; begin write
              BSF       INTCON, GIE  ; enable global interrupts
              RETURN	             ; exit point of _write_eeprom             
              END
