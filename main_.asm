list P=16F628A
#include <P16F628A.inc>      ; processor specific definitions
              __CONFIG _CP_OFF&_WDT_OFF&_PWRTE_ON&_HS_OSC&_LVP_OFF&_BODEN_OFF&_MCLRE_OFF
              extern _ds18b20_init
              extern _leds_init
              extern _uart_init
              extern _uart_rx_packet
              extern _timer1_int
              extern _ReceiveByteSerially
              extern _freq_mesure
              extern _leds
              global _SendByteSerially
              global _main
              global _b
              global _timer0_overflows
              global _uart
              global _uart_id
              global _frequency_measure_begin
              global _timer0_prescaler
              global _option_reg
              global PSAVE
              global SSAVE
              global WSAVE
              global FSAVE
              global STK12
              global STK11
              global STK10
              global STK09
              global STK08
              global STK07
              global STK06
              global STK05
              global STK04
              global STK03
              global STK02
              global STK01
              global STK00

UD_MAIN_0           udata  0x52
_timer0_prescaler   res 1

UD_MAIN_1           udata  0x53
_timer0_overflows   res 2

UD_MAIN_2           udata  0x55
packet              res 1

UD_MAIN_3           udata  0x56
uart_address        res 1

UD_MAIN_4           udata  0x57
uart_byte           res 1

UD_MAIN_5           udata  0x58
_b                  res 1

UD_UART_6           udata  0x59
_uart_id            res 1

UD_UART_7           udata  0x60
_uart               res 8


sharebank udata_ovr 0x0070
PSAVE               res 1
SSAVE               res 1
WSAVE               res 1
FSAVE               res 1
STK12               res 1
STK11               res 1
STK10               res 1
STK09               res 1
STK08               res 1
STK07               res 1
STK06               res 1
STK05               res 1
STK04               res 1
STK03               res 1
STK02               res 1
STK01               res 1
STK00               res 1



RX_FULL                equ 0
TIMER1_INTERRUPT       equ 1
STARTUP       code 0x0000
              goto    __sdcc_gsinit_startup
c_interrupt   code   0x0004
              GOTO    INTERRUPT_F
              code
_option_reg
       retlw 0x20    ; 32
       retlw 0x21    ; 33
       retlw 0x22    ; 34
       retlw 0x23    ; 35
       retlw 0x24    ; 36
__sdcc_gsinit_startup:
              GOTO    _main
_SendByteSerially:
              BANKSEL PIR1              ; select Bank 0
              BTFSS   PIR1, TXIF
              GOTO    $-D'1'
              MOVWF   TXREG
              RETURN 
_freq_mesure_asm:
              BANKSEL _timer0_prescaler
              MOVF    _timer0_prescaler, W
              BTFSC   STATUS, Z
;       GOTO   _00121_DS_
;      .line  144; "main.c" SetPrescaler( option_reg[ --timer0_prescaler ] );
;       DECF   _timer0_prescaler,F
;       MOVF   _timer0_prescaler,W
;       ADDLW  (_option_reg + 0)
;      BANKSEL       r0x1010
;      MOVWF  r0x1010
;      MOVLW  high (_option_reg + 0)
;      BTFSC  STATUS,0
;       ADDLW  0x01
;       MOVWF  r0x1011
;       MOVF   r0x1010,W
;       MOVWF  STK01
;       MOVF   r0x1011,W
;       MOVWF  STK00
       MOVLW  0x80
;       PAGESEL       __gptrget1
;       CALL   __gptrget1
 ;      PAGESEL       $
;;1    MOVWF  r0x1012
;     PAGESEL       _SetPrescaler
;       CALL   _SetPrescaler
 ;      PAGESEL       $
       GOTO   _00122_DS_
_00121_DS_:
;      .line  146; "main.c" PrescalerOff();
 ;      PAGESEL       _PrescalerOff
;       CALL   _PrescalerOff
 ;      PAGESEL       $
_00122_DS_:
;      .line  147; "main.c" freq_mesure_init();
 ;      PAGESEL       _freq_mesure_init
  ;     CALL   _freq_mesure_init
  ;     PAGESEL       $
;      .line  148; "main.c" }
;       RETURN 

_frequency_measure_begin:
              MOVLW  0x05
              MOVWF  _timer0_prescaler
              GOTO   _freq_mesure
_main:
              BANKSEL _b
              CLRF    _b
              CLRF    packet
              CALL    _ds18b20_init
              CALL    _leds_init
              CALL    _uart_init
              BSF     INTCON, GIE
              BSF INTCON, PEIE
              BANKSEL _b          ; BANK 0
MAIN_LOOP:
              BTFSC _b, RX_FULL              
              CALL  _uart_rx_packet
              BTFSC _b, TIMER1_INTERRUPT
              CALL  _timer1_int
              GOTO MAIN_LOOP
              RETURN    
;--------------------------------------------------------
; interrupt and initialization code
;--------------------------------------------------------

INTERRUPT_F:
              MOVWF   WSAVE
              SWAPF   STATUS,W
              CLRF    STATUS
              MOVWF   SSAVE
              MOVF    PCLATH,W
              CLRF    PCLATH
              MOVWF   PSAVE
              MOVF    FSR,W
              MOVWF   FSAVE
              BANKSEL PIR1
              BTFSS   PIR1, RCIF
              GOTO    SKIP_UART_RX_EVENT
;==========================================================
; UART RX EVENT              
;==========================================================
              CALL    _ReceiveByteSerially              ; read UART
              BTFSC   _b, RX_FULL
              GOTO    EXIT
              MOVWF   uart_byte
              ;;unsigned compare: left < lit(0x4=4), size=1
;      .line  275; "main.c" if ( ( 4 <= uart_id ) && ( uart[ uart_id - 1 ] == 0xFF ) && ( uart[ uart_id - 2 ] == 0xFF ) ) RX_FULL = 1;
              MOVLW   D'253'
              ADDWF   _uart_id, W         ; test if uart packets size is bigger or equeal to 3
              RLF     packet, F
              MOVF    uart_byte, W
              XORLW   0xff
              BTFSC   STATUS, Z
              BSF     STATUS, C
              RLF     packet, F
              MOVF    packet, W
              ANDLW   0x07
              MOVWF   packet
              SUBLW   0x07
              BSF     _b, RX_FULL
              BTFSC   STATUS, Z           ; Three of low sagnificant bit are in packet set?
              GOTO    EXIT                ; if so -> go to exit
                                          ;
              BCF     _b, RX_FULL         ; clear RX_FULL FLAG, means that not all symbols in packet are received 
              MOVF    _uart_id, W         ; W = uart_id
              ADDLW   _uart               ; uart_ptr + uart_id = pointer to actual register in uart array
              MOVWF   FSR
              MOVF    uart_byte, W
              MOVWF   INDF
              INCF    _uart_id, F
              MOVLW   0x07
              ANDWF   _uart_id, F
              GOTO    EXIT
SKIP_UART_RX_EVENT:
;_00237_DS_:
;      .line  278; "main.c" else if ( TMR0IF )
              BTFSS   INTCON, TMR0IF       ; TIMER 0 interrupt
              GOTO    LABEL_TIMER1_INTERRUPT
;             BCF     INTCON, TMR0IF
;      .line  281; "main.c" timer0_overflows++;
              INCF    _timer0_overflows, F
              BTFSC   STATUS, Z
              INCF   (_timer0_overflows + 1),F
              GOTO    EXIT
LABEL_TIMER1_INTERRUPT:
;      .line  283; "main.c" else if ( TMR1IF )
              BTFSS   PIR1, TMR1IF
              GOTO    EXIT
;             .line  285; "main.c" TRISA3 = 0;
              BANKSEL TRISA               ; SELECT Bank 1
              BCF     TRISA, RA3          ; switch RA3 to output -> disable count
;      .line  286; "main.c" TMR1ON = 0;  // disable timer
              BANKSEL T1CON               ; SELECT Bank 0
              BCF     T1CON, TMR1ON
;      .line  287; "main.c" TMR1IF = 0;
              BCF     PIR1, TMR1IF
;      .line  288; "main.c" T0IE = 0;
              BCF     INTCON, T0IE
;      .line  289; "main.c" TIMER1_INTERRUPT = 1;
              BSF    _b, TIMER1_INTERRUPT
;_00239_DS_:
EXIT:
              MOVF   FSAVE,W
              MOVWF  FSR
              MOVF   PSAVE,W
              MOVWF  PCLATH
              CLRF   STATUS
              SWAPF  SSAVE,W
              MOVWF  STATUS
              SWAPF  WSAVE,F
              SWAPF  WSAVE,W
              RETFIE
              END