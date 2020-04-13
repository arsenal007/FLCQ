list P=16F628A
#include <P16F628A.inc>      ; processor specific definitions
              global _read_eeprom
              code
_read_eeprom:
              BANKSEL   EEADR    ; Переход в первый банк.
              MOVWF     EEADR    ; Скопировать 02h, из регистра W,
                                 ; в регистр EEAdr.
              bsf   EECON1, 0    ; Инициализировать чтение.
              movf  EEDATA, W    ; W = EEDATA
              RETURN              
              end
