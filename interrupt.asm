list P=16F628A
#include <P16F628A.inc>      ; processor specific definitions


UD_INTERRUPT_0   udata  0x56
packet           res 1

UD_INTERRUPT_1   udata  0x57
uart_address     res 1

UD_INTERRUPT_2   udata  0x58
uart_byte        res 1


sharebank udata_ovr 0x0070
PCLATH_SAVE	res 1
STATUS_SAVE	res 1
W_SAVE	    res 1
FSR_SAVE    res 1

;--------------------------------------------------------
; interrupt and initialization code
;--------------------------------------------------------
              code    0x0004
              MOVWF   W_SAVE
              SWAPF   STATUS,W
              CLRF    STATUS
              MOVWF   STATUS_SAVE
              MOVF    PCLATH,W
              CLRF    PCLATH
              MOVWF   PCLATH_SAVE
              MOVF    FSR,W
              MOVWF   FSR_SAVE
              BANKSEL PIR1
              BTFSS   PIR1, RCIF
              GOTO    SKIP_UART_RX_EVENT
;==========================================================
; UART RX EVENT              
;==========================================================
              CALL    _ReceiveByteSerially		; read UART
              BTFSC   _b, RX_FULL
              GOTO    EXIT
              MOVWF   uart_byte
              ;;unsigned compare: left < lit(0x4=4), size=1
;      .line  275; "main.c" if ( ( 4 <= uart_id ) && ( uart[ uart_id - 1 ] == 0xFF ) && ( uart[ uart_id - 2 ] == 0xFF ) ) RX_FULL = 1;
              MOVLW   0x03
              SUBWF   _uart_id,W         ; test if uart packets size is bigger or equeal to 3
;             BTFSS   STATUS, C
              RLF     packet, F
              MOVWF   uart_byte
              XORLW   0xff
;             BCF     STATUS, C 
              BTFSS   STATUS, Z
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



;;genSkipc:3307: created from rifx:00000000047A5780
              
       MOVF   _uart_id,W
       BANKSEL       r0x102F
       MOVWF  r0x102F
       DECF   r0x102F,W
       MOVWF  r0x1030
       CLRF   r0x1031
       MOVF   r0x1030,W
       ADDLW  (_uart + 0)
       MOVWF  r0x1030
       MOVLW  0x00
       BTFSC  STATUS,0
       INCFSZ r0x1031,W
       ADDLW  high (_uart + 0)
       MOVWF  r0x1031
       MOVF   r0x1030,W
       BANKSEL       FSR
       MOVWF  FSR
       BCF    STATUS,7
       BANKSEL       r0x1031
       BTFSC  r0x1031,0
       BSF    STATUS,7
       BANKSEL       INDF
       MOVF   INDF,W
;;1    MOVWF  r0x1032
       XORLW  0xff
       BTFSS  STATUS,2
       GOTO   _00239_DS_
       MOVLW  0xfe
       BANKSEL       r0x102F
       ADDWF  r0x102F,F
       CLRF   r0x1030
       MOVF   r0x102F,W
       ADDLW  (_uart + 0)
       MOVWF  r0x102F
       MOVLW  0x00
       BTFSC  STATUS,0
       INCFSZ r0x1030,W
       ADDLW  high (_uart + 0)
       MOVWF  r0x1030
       MOVF   r0x102F,W
       BANKSEL       FSR
       MOVWF  FSR
       BCF    STATUS,7
       BANKSEL       r0x1030
       BTFSC  r0x1030,0
       BSF    STATUS,7
       BANKSEL       INDF
       MOVF   INDF,W
       BANKSEL       r0x1031
       MOVWF  r0x1031
       XORLW  0xff
       BTFSS  STATUS,2
       GOTO   _00239_DS_
       BANKSEL       _b
       BSF    _b,0
       GOTO   _00239_DS_
_00237_DS_:
;      .line  278; "main.c" else if ( TMR0IF )
       BANKSEL       _INTCONbits
       BTFSS  _INTCONbits,2
       GOTO   _00234_DS_
;      .line  280; "main.c" TMR0IF = 0;
       BCF    _INTCONbits,2
;      .line  281; "main.c" timer0_overflows++;
       BANKSEL       _timer0_overflows
       INCF   _timer0_overflows,F
       BTFSC  STATUS,2
       INCF   (_timer0_overflows + 1),F
       GOTO   _00239_DS_
_00234_DS_:
;      .line  283; "main.c" else if ( TMR1IF )
       BANKSEL       _PIR1bits
       BTFSS  _PIR1bits,0
       GOTO   _00239_DS_
;      .line  285; "main.c" TRISA3 = 0;
       BANKSEL       _TRISAbits
       BCF    _TRISAbits,3
;      .line  286; "main.c" TMR1ON = 0;  // disable timer
       BANKSEL       _T1CONbits
       BCF    _T1CONbits,0
;      .line  287; "main.c" TMR1IF = 0;
       BCF    _PIR1bits,0
;      .line  288; "main.c" T0IE = 0;
       BCF    _INTCONbits,5
;      .line  289; "main.c" TIMER1_INTERRUPT = 1;
       BANKSEL       _b
       BSF    _b,1
;_00239_DS_:
EXIT:
       MOVF    FSR_SAVE,W
       MOVWF   FSR
       MOVF    PCLATH_SAVE,W
       MOVWF   PCLATH
       CLRF    STATUS
       SWAPF   STATUS_SAVE,W
       MOVWF   STATUS
       SWAPF   W_SAVE,F
       SWAPF   W_SAVE,W
       RETFIE          