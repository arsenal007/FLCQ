list P=16F628A
#include <P16F628A.inc>      ; processor specific definitions
              __CONFIG _CP_OFF&_WDT_OFF&_PWRTE_ON&_HS_OSC&_LVP_OFF&_BODEN_OFF&_MCLRE_OFF
              global _read_eeprom
              code
_read_eeprom  BANKSEL EEADR  ; Переход в первый банк.
              movwf EEADR        ; Скопировать 02h, из регистра W,
                                  ; в регистр EEAdr.
              bsf   EECON1, 0   ; Инициализировать чтение.
              movf  EEDATA, W    ; W = EEDATA
              return              
              end
