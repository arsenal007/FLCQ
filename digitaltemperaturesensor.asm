;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;::;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                                                                                                  ;
;                        Program: DS18B20 Digital Celsius & Fahrenheit Thermometer                 ;
;                        Author:  Jake Sutherland                                                  ;
;                        Start Date: Friday 1st July 2011                                          ;
;                        Finish Date:                                                              ;
;                        Comments:                                                                 ;
;                                                                                                  ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;::;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

        LIST            P=PIC16F628A
        INCLUDE         P16F628A.INC
        __config        _CP_OFF & DATA_CP_OFF & _LVP_OFF & _BODEN_OFF & _MCLRE_OFF & _PWRTE_ON & _WDT_OFF & _INTRC_OSC_NOCLKOUT
        errorlevel      -302    ;Eliminate bank warning

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;::;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                                                                                                  ;
;                                   PIC16F628A Microcontroller                                     ;
;                                            ____ ____                                             ;
; LED 7 Segment 10's CC       VREF/AN2/RA2 -| 1  - 18 |- RA1/AN1              LED 7 Segment 1's CC ;
; LED 7 Segment 100's CC      CPM1/AN3/RA3 -| 2    17 |- RA0/AN0             LED 7 Segment .1's CC ;
; DS18B20 I/O               CMP2/T0CKI/RA4 -| 3    16 |- RA7/OSC1/CLKIN                            ;
; Fahrenheit/Celsius Select   VPP/MCLR/RA5 -| 4    15 |- RA6/OSC2/CLKOUT  Temp Format (C/F) Select ;
;                                      VSS -| 5    14 |- VDD                                       ;
; Segment DP                       INT/RB0 -| 6    13 |- RB7/T1OSC1/ICSPDAT              Segment B ;
; Segment C                      DT/RX/RB1 -| 7    12 |- RB6/T1OSCO/T1CLKI/ICSPCLK       Segment A ;
; Segment D                      CK/TX/RB2 -| 8    11 |- RB5                             Segment F ;
; Segment E                       CCP1/RB3 -|_9____10_|- RB4/PGM                         Segment G ;
;                                                                                                  ;
;                                                                                                  ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;::;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  
CBLOCK  H'20'
HUNS                                    ; Jump pointer for Hundreds digit
TENS                                    ; Jump pointer for Tens digit
ONES                                    ; Jump pointer for Ones digit
DECI                                    ; Jump pointer for Decimal digit
THOU_COL                                ; 7 Segment Display Thousands Column, used within ISR
HUNS_COL                                ; 7 Segment Display Hundreds Column, used within ISR
TENS_COL                                ; 7 Segment Display Tens Column, used within ISR
ONES_COL                                ; 7 Segment Display Ones Column, used within ISR
COLSEL                                  ; Stores COLUMN SELECT information 
DIGIT                                   ; Used for Indirect Addressing in Interrupt Service Routine
TEMPLO_C                                ; BS18B20 Celsius Temperature Result Low Byte
TEMPHI_C                                ; BS18B20 Celsius Temperature Result High Byte
TEMPLO_F                                ; Fahrenheit temperature lower byte
TEMPHI_F                                ; Fahrenheit temperature upper byte
COUNT                                   ; For counting execution times in various routines
SHIFT                                   ; For storing data to shift into and out of the DS18B20
DELAYGPR1                               ; Used in delay subroutines
FLAGS                                   ;   -------0  Temperature Format Select (0 = Celsius 1 = Fahrenheit)
                                        ;   ------0-  Celsius/Fahrenheit Temperature Negative Flag (0 = False 1 = True)
                                        ;   -----X--  Unused
                                        ;   ----0---  Positive/Negative Flag after 'Add 4' routine
                                        ;   ---X----  Unused
                                        ;   --X-----  Unused
                                        ;   -X------  Unused
                                        ;   X-------  Unused
ENDC
  
W_ISR   EQU     H'70'                   ; For context saving. Ensures GPR is accessible from any Bank. See Memory Map of 628A
S_ISR   EQU     H'71'                   ; For context saving. Ensures GPR is accessible from any Bank. See Memory Map of 628A
P_ISR   EQU     H'72'                   ; For context saving. Ensures GPR is accessible from any Bank. See Memory Map of 628A
F_ISR   EQU     H'73'                   ; For context saving. Ensures GPR is accessible from any Bank. See Memory Map of 628A
  
  
        ORG     H'000'                  ; Processor reset vector location. On power up, the program jumps here
        GOTO    SETUP                   ; 
  
        ORG     H'004'                  ; Interrupt vector location. When an Interrupt occurs, the program jumps here
  ; (Thanks Mike, K8LH for help with this ISR routine. Contact via Electro-Tech-Online.com)
  ; Save
        MOVWF   W_ISR                   ; Save W to W_ISR
        SWAPF   STATUS, W               ; Use SWAPF instruction so status bits don't change
        MOVWF   S_ISR                   ; Save Status to S_ISR
        CLRF    STATUS                  ; Switch to Bank 0
        MOVF    PCLATH, W               ; Move PCLATH to W register
        MOVWF   P_ISR                   ; Save PCLATH to P_ISR
        CLRF    PCLATH                  ; Force page 0
        MOVF    FSR, W                  ; Move FSR to W register
        MOVWF   F_ISR                   ; Save FSR to F_ISR
  
  ; Refresh display
        CLRF    PORTB                   ; Blank the display
        MOVF    PORTA, W                ; Move PORTA's current state to W
        ANDLW   B'11100000'             ; Clear ONLY Column Select Bits & leave upper nibble unchanged
        IORWF   COLSEL, W               ; Inclusive OR above with COLSEL register
        MOVWF   PORTA                   ; Leave upper nibble of PORTA unchanged & select new column
  
        MOVF    DIGIT, W                ; Column number (DIGIT is initialised to 0. Used for Indirect Addressing)
        ADDLW   THOU_COL                ; Add 'DIGIT' to 'THOU_COL' GPR address
        MOVWF   FSR                     ; FSR = THOU_COL + (0forTHOUS, 1forHUNS, 2forTENS & 3forONES) as they are in sequential order in RAM
        MOVF    INDF, W                 ; W register now holds the jump pointer for the corresponding digit to be displayed
        CALL    SEGMENT_TABLE           ; Get 7 segment arrangement from SEGMENT_TABLE for corresponding digit
  
        BTFSC   COLSEL, 1               ; Is TENS Column active
        IORLW   B'00000001'             ; Yes, so activate TEN's column decimal point
        MOVWF   PORTB                   ; NO, hide decimal point and display new column
  
  ; Prepare for next column interrupt
        INCF    DIGIT, F                ; Increment DIGIT jump pointer (used when indirect addressing above)
        BCF     STATUS, C               ; Clear C bit of STATUS register
        RRF     COLSEL, F               ; Advance column select bit       
        BTFSS   STATUS,C                ; Was that the last column? (or, Is Carry Flag Set?)  
        GOTO    $+D'3'                  ; NO, do not reset DIGIT and COLumnSELect
        CLRF    DIGIT                   ; Reset DIGIT jump pointer
        BSF     COLSEL, 3               ; Reset column select to THOU_COL (1XXX) (00001000)
  
  ; RESTORE
        BCF     PIR1, TMR2IF            ; Clear TMR2 interrupt flag while still in Bank 0        
        MOVF    F_ISR, W                ; MOVE F_ISR to W
        MOVWF   FSR                     ; Restore FSR
        MOVF    P_ISR, W                ; MOVE P_ISR to W
        MOVWF   PCLATH                  ; Restore PCLATH
        SWAPF   S_ISR, W                ; Undo previous SWAPF, place result in W
        MOVWF   STATUS                  ; Restore STATUS
        SWAPF   W_ISR, F                ; Use SWAPF instruction so status bits don't change
        SWAPF   W_ISR, W                ; Undo previous SWAPF and restore W register
        RETFIE                          ; Return From Interrupt
  
  ; 7 Segment Display Arrangement Lookup
SEGMENT_TABLE  ; This routine assigns on/off arrangement to the 7 Segment Display
        ADDWF   PCL, F
             ;    BAFGEDCp    JUMP   B|A|F|G|E|D|C|p   DISPLAY
        RETLW   B'11101110' ;   0    B|A|F|-|E|D|C|-      0
        RETLW   B'10000010' ;   1    B|-|-|-|-|-|C|-      1
        RETLW   B'11011100' ;   2    B|A|-|G|E|D|-|-      2
        RETLW   B'11010110' ;   3    B|A|-|G|-|D|C|-      3
        RETLW   B'10110010' ;   4    B|-|F|G|-|-|C|-      4
        RETLW   B'01110110' ;   5    -|A|F|G|-|D|C|-      5
        RETLW   B'01111110' ;   6    -|A|F|G|E|D|C|-      6
        RETLW   B'11000010' ;   7    B|A|-|-|-|-|C|-      7
        RETLW   B'11111110' ;   8    B|A|F|G|E|D|C|-      8
        RETLW   B'11110110' ;   9    B|A|F|G|-|D|C|-      9
        RETLW   B'00000000' ;  10    -|-|-|-|-|-|-|-    BLANK
        RETLW   B'00010000' ;  11    -|-|-|G|-|-|-|-      -
        RETURN
  
  ; Program Originates Here
SETUP  ; This routine sets up the Microcontroller's Inputs and Outputs
        MOVLW   H'07'                   ; Turn Comparators off and enable pins for I/O functions
        MOVWF   CMCON                   ;
        BSF     STATUS, RP0             ; Bank 1
        MOVLW   B'01110000'             ; RA<7><3:0> Output, RA<5:4> Input
        MOVWF   TRISA                   ;
        CLRF    TRISB                   ; RB<7:0> Outputs
        BCF     STATUS, RP0             ; Bank 0
  
        CLRF    PORTA
        CLRF    PORTB
  
ONE_OFF_INITIALISE
  ; Clear ALL Bank 0 RAM from 0X20 - 0X7F (96 bytes)
        BCF     STATUS, IRP             ; Indirect addressing Bank 0/1
        MOVLW   H'20'                   ; Initialise pointer to RAM
        MOVWF   FSR                     ; (File Select Register)
        CLRF    INDF                    ; Clear Register indirectly
        INCF    FSR, F                  ; Increment pointer
        BTFSS   FSR, 7                  ; All done?
        GOTO    $-D'3'                  ; No, Clear next byte
  
  ; Initialise Column Select
        BSF     COLSEL, 3               ; Set column select to THOU_COL (1XXX) (00001000)
  
  ; Setup TMR2 for 0.0005s or 500uS Interrupts
        CLRF    TMR2                    ; Clear TMR2 register
        BSF     STATUS, RP0             ; Bank 1
        MOVLW   B'00000010'             ;   00000010
        MOVWF   PIE1                    ;   -------0  Disable TMR1IE - TMR1 Overflow Interrupt Enable bit
                                        ;   ------1-  Enable  TMR2IE - TMR2 to PR2 Match Interrupt Enable bit
                                        ;   -----0--  Disable CCP1IE - CCP1 Interrupt Enable bit
                                        ;   ----X---  Unused
                                        ;   ---0----  Disable TXIE - USART Transmit Interrupt Enable bit
                                        ;   --0-----  Disable RCIE - USART Receive Interrupt Enable bit
                                        ;   -0------  Disable CMIE - Comparator Interrupt Enable bit
                                        ;   0-------  Disable EEIE - EE Write Complete Interrupt Enable bit
        BCF     STATUS, RP0             ; Bank 0
        CLRF    PIR1                    ; Clear Peripheral Interrupt Flags
        MOVLW   B'00000001'             ;   00000001
        MOVWF   T2CON                   ;   ------01  Pre-scale 1:4
                                        ;   -----0--  TMR2 OFF
                                        ;   -0000---  Post-scale 1:1
                                        ;   0-------  Unused
        BSF     STATUS, RP0             ; Bank 1
        MOVLW   D'250'                  ; 
        MOVWF   PR2                     ; Interrupt every 0.001s or 1mS
        BCF     STATUS, RP0             ; Bank 0
        BSF     INTCON, GIE             ; Enable Global Interrupts
        BSF     INTCON, PEIE            ; Enable Peripheral Interrupts
        BSF     T2CON, TMR2ON           ; Start TMR2
  
        GOTO    GET_TEMP
  
  ; Dallas DS18B20 Temperature Sensor 'One Wire' Communication Routines
DS18B20_RESET  ; This routine Resets the DS18B20
        CALL    DQ_LL                   ; Force the DQ Line to Logic Low
        MOVLW   (D'480'-D'5')/D'5'      ; Reset pulse must be held a minimum of 480uS. 
        CALL    DELAY                   ; For delays from 10uS to 1285uS. Example, MOVLW D'10' for 10uS, MOVLW D'500' for 500uS. (D'n'-D'5')/D'5'; n must be divisible by 5
        CALL    DQ_HIZ                  ; Release DQ Line
        MOVLW   (D'60'-D'5')/D'5'       ; Wait for recovery
        CALL    DELAY                   ; 
        BTFSC   PORTA, 4                ; Test for 'Presence Pulse'
        GOTO    DS18B20_RESET           ; If not present, Reset
        MOVLW   (D'420'-D'5')/D'5'      ; Must wait a minimum of 480uS from when DQ line is released before moving on
        CALL    DELAY                   ; 
        RETURN                          ;
  
DS18B20_READ_BYTE  ; This routine reads a byte of data from the DS18B20
        MOVLW   H'08'                   ; Amount of bits to shift out
        MOVWF   COUNT                   ; Store in COUNT GPR
        CALL    DS18B20_READ_BIT        ; Read bit
        RRF     SHIFT, F                ; Rotate bit out of carry into SHIFT GPR
        DECFSZ  COUNT, F                ; Decrement COUNT
        GOTO    $-D'3'                  ; If not zero, read next bit
        MOVF    SHIFT, W                ; If zero, move SHIFT GPR to W
        RETURN                          ;
  
DS18B20_READ_BIT  ; This routine reads one bit of data from the DS18B20
        BCF     INTCON, GIE             ; Disable Global Interrupts
        CALL    DQ_LL                   ; Force the DQ Line to Logic Low
        CALL    DQ_HIZ                  ; Release DQ Line
        BSF     STATUS, C               ; Pre-set Carry Flag
        BTFSS   PORTA, 4                ; Test DQ Line
        BCF     STATUS, C               ; If zero, Clear Carry Flag
        BSF     INTCON, GIE             ; Enable Global Interrupts
        MOVLW   (D'60'-D'5')/D'5'       ; Wait for Time Slot to end
        CALL    DELAY                   ; 
        RETURN                          ;
  
DS18B20_WRITE_BYTE  ; This routine writes a byte of data to the DS18B20
        MOVWF   SHIFT                   ; Move data to shift into DS18B20 to SHIFT GPR
        MOVLW   D'08'                   ; Amount of bits to shift in
        MOVWF   COUNT                   ; Store in COUNT GPR
        RRF     SHIFT, F                ; Rotate valid data into Carry Flag
        CALL    DS18B20_WRITE_BIT       ; Write bit
        DECFSZ  COUNT, F                ; Decrement COUNT
        GOTO    $-D'3'                  ; If not zero, read next bit
        RETURN                          ; If zero, RETURN
  
DS18B20_WRITE_BIT  ; This routine writes one bit of data to the DS18B20
        BCF     INTCON, GIE             ; Disable Global Interrupts
        CALL    DQ_LL                   ; Force the DQ Line to Logic Low
        BTFSS   STATUS, C               ; Test Carry Flag
        GOTO    $+D'2'                  ; If zero, leave DQ Line Logic Low
        CALL    DQ_HIZ                  ; If not zero, release DQ Line to Logic High
        MOVLW   (D'60'-D'5')/D'5'       ; Hold Logic Low, write 0
        CALL    DELAY                   ; 
        CALL    DQ_HIZ                  ; If not zero, release DQ Line to Logic High, write 1
        BSF     INTCON, GIE             ; Enable Global Interrupts
        RETURN                          ; 
  
DQ_HIZ  ; This routine forces the DQ Line to an Input / High Impedance state
        BSF     STATUS, RP0             ; Bank 1
        BSF     TRISA, 4                ; Make Pin 3 an input, Pull-up resistor forces line to logic 1, unless DS18B20 pulls it low
        BCF     STATUS, RP0             ; Bank 0
        RETURN
  
DQ_LL  ; This routine forces the DQ Line to Logic Low
        BCF     PORTA, 4                ; Clear output latch
        BSF     STATUS, RP0             ; Bank 1
        BCF     TRISA, 4                ; Make Pin 3 an output
        BCF     STATUS, RP0             ; Bank 0
        RETURN
  
DELAY  ; This routine can provide delays from 10uS to 1285uS depending on the number moved to the Working register prior to calling the delay
        MOVWF   DELAYGPR1               ; Move integer to GPR
        NOP                             ; No Operation
        NOP                             ; No Operation
        DECFSZ  DELAYGPR1, F            ; Decrement GPR and place back in itself
        GOTO    $-D'3'                  ; Not finished, GOTO here - 3 instructions
        RETURN                          ; Finished, Return
  
GET_TEMP
        CALL    DS18B20_RESET           ; Reset
        MOVLW   H'CC'                   ; Skiprom
        CALL    DS18B20_WRITE_BYTE      ; Skiprom
        MOVLW   H'44'                   ; Convert T
        CALL    DS18B20_WRITE_BYTE      ; Convert T
  
        CALL    DS18B20_READ_BIT        ; Initiate Read Time Slots
        BTFSS   STATUS, C               ; DS18B20 transmits a 0 while the temperature conversion is in progress and a 1 when the conversion is done
        GOTO    $-D'2'                  ; Transmitting 0 therefore NOT done
  
        CALL    DS18B20_RESET           ; Reset
        MOVLW   H'CC'                   ; Skiprom
        CALL    DS18B20_WRITE_BYTE      ; Skiprom
        MOVLW   H'BE'                   ; Read Scratchpad
        CALL    DS18B20_WRITE_BYTE      ; Read Scratchpad
  
        CALL    DS18B20_READ_BYTE       ; Move Scrachpad Byte 0 to W
        MOVWF   TEMPLO_C                ; Store in Celsius TEMPLO GPR
        CALL    DS18B20_READ_BYTE       ; Move Scrachpad Byte 1 to W
        MOVWF   TEMPHI_C                ; Store in Celsius TEMPHI GPR
  
  ; Test Temperature Format Select pin and display corresponding temperature type (C or F)
        BTFSS   PORTA, 5                ; Test Select pin for high or low
        GOTO    DISPLAY_CELSIUS         ; Clear, display Celsius
                                        ; Set, display Fahrenheit
  
  ; Begin converting 16 bit Celsius data into Fahrenheit
DISPLAY_FAHRENHEIT  ; This routine Clear all Flags and copies Celsius data into Fahrenheit GPRs for conversion
        CLRF    FLAGS                   ; Clear Flags GPR
        MOVF    TEMPLO_C, W             ; Copy Celsius Temperature Data lower byte
        MOVWF   TEMPLO_F                ; Move to Fahrenheit GPR lower byte
        MOVF    TEMPHI_C, W             ; Copy Celsius Temperature Data upper byte
        MOVWF   TEMPHI_F                ; Move to Fahrenheit GPR upper byte
  
 ; The following group of routines convert the Celsius Temperature Data into Fahrenheit x 10.
 ; This is acheived by dividing by 8 and adding the original number, which gives 18 times the value. 
 ; If you now add 320 you will have the temperature in Fahrenheit x 10.
 ; After the conversion it is simply a matter of displaying the data with the decimal shifted one place to the right.
 ; Example 
 ; 25 Degrees C  =  B'00000001 10010000'  =  D'400' 
 ; 400 / 8   =  50
 ;  50 + 400 = 450
 ; 450 + 320 = 770
 ; 770 /  10 = 77 Degrees Fahrenheit
  
  ; Add 4, this ensures number is correctly rounded when using the shift method below to divide
        MOVLW   B'00000100'             ; Move D'4' to W
        ADDWF   TEMPLO_F, F             ; Add W to TEMPLO_F and place result back in File Register TEMPLO_F
        BTFSC   STATUS, C               ; Check if TEMPLO_F overflowed
        INCF    TEMPHI_F, F             ; Overflow occurred, increment upper register 
  
  ; Flag if result positive or negative after add 4 (needed for mask routine)
        BTFSC   TEMPHI_F, 7             ; Test bit 7 of TEMPHI_F
        BSF     FLAGS, 3                ; Set flag if negative, skip if clear
  
  ; Divide by 8 using shift method
        RRF     TEMPHI_F, F             ; Rotate Right File Register 'F' (TEMPHI_F) into carry
        RRF     TEMPLO_F, F             ; Rotate Right File Register 'F' (TEMPLO_F) out of carry;  / 2
        RRF     TEMPHI_F, F             ; Repeat...
        RRF     TEMPLO_F, F             ;                                                          / 4
        RRF     TEMPHI_F, F             ; 
        RRF     TEMPLO_F, F             ;                                                          / 8
  
  ; Mask routine. Adjust upper 3 bits of TEMPHI_F due to shift. Set if negative, clear if positive
        BTFSS   FLAGS, 3                ; Test if result after 'Add 4' is positive or negative
        GOTO    $+D'4'                  ; Positive, GOTO here + D'4'
        MOVLW   B'11100000'             ; Negative,
        IORWF   TEMPHI_F, F             ; Set upper 3 bits
        GOTO    $+D'3'                  ; Done, Skip
        MOVLW   B'00011111'             ; Positive,
        ANDWF   TEMPHI_F, F             ; Clear upper 3 bits
  
  ; Add original Celsius Temperature to obtain C x 18 
        MOVF    TEMPLO_C, W             ; Move TEMPLO_C to W
        ADDWF   TEMPLO_F, F             ; Add TEMPLO_C and TEMPLO_F and place result back in File Register TEMPLO_F
        MOVF    TEMPHI_C, W             ; Pre-load TEMPHI_C to W (doesn't effect carry)
        BTFSC   STATUS, C               ; Did Add cause overflow?
        INCF    TEMPHI_C, W             ; Yes, increment TEMPHI_C store in W
        ADDWF   TEMPHI_F, F             ; Add TEMPHI_C and TEMPHI_F. Result in now C x 18
  
  ; Add 320 to obtain F x 10
        MOVLW   B'01000000'             ; 320 lower byte
        ADDWF   TEMPLO_F, F             ; Add 320 lower byte and TEMPLO_F and place result back in File Register TEMPLO_F
        BTFSC   STATUS, C               ; Did Add cause overflow?
        INCF    TEMPHI_F, F             ; Yes, increment TEMPHI_F
        MOVLW   B'00000001'             ; 320 upper byte
        ADDWF   TEMPHI_F, F             ; Add 320 upper byte and TEMPHI_F. Result is now C x 18 + 320 (= F x 10)
  
        BTFSS   TEMPHI_F, 7             ; Test Sign Bits for Fahrenheit Temperature negative (2's complemented)
        GOTO    BIN_TO_DEC_F            ; No, skip undo 2's complement
  
        COMF    TEMPLO_F, F             ; Invert TEMPLO 
        COMF    TEMPHI_F, F             ; Invert TEMPHI
        INCFSZ  TEMPLO_F, F             ; Increment TEMPLO once; After inverting the data, add one to achieve equivalent positive number (see 2's complement)
        GOTO    $+D'2'                  ; No overflow skip next instruction
        INCF    TEMPHI_F, F             ; Overflow occurred, increment upper byte
        BSF     FLAGS, 1                ; Set Negative Flag for Fahrenheit Temperature
  
BIN_TO_DEC_F  ; This routine converts a 16-bit number held in TEMPHI_F:TEMPLO_F to Decimal and stores it in 4 GPR's; HUNS, TENS, ONES & DECI 
 ; !!!!! Tweaked for this program due to Fahrenheit x 10. GPR Labels adjusted for this program and would normally be THOUS, HUNS, TENS & ONES instead of HUNS, TENS, ONES & DECI !!!!!
 ; Example 1225;
 ; THOUS holds amount of thousands (i.e. 0000 0001 = 1x1000)
 ; HUNS holds amount of hundreds (i.e. 0000 0010 = 2x100)
 ; TENS holds amount of tens (i.e. 0000 0010 = 2x10)
 ; ONES holds amount of ones (i.e. 0000 0101 = 5x1)
 ; These registers are then used as jump pointers when calling digits to display
  
        INCF    TEMPLO_F                ; Pre-load TEMPHI + 1
        INCF    TEMPHI_F                ; Pre-load TEMPHI + 1
        CLRF    HUNS                    ; THOUS = 0000 0000                                                                         GPR changed from THOUS for this program
  
        MOVLW   D'246'                  ; Move decimal '246' to W
        MOVWF   TENS                    ; HUNS GPR = 1111 0101                                                                      GPR changed from HUNS for this program
        MOVWF   ONES                    ; TENS GPR = 1111 0101                                                                      GPR changed from TENS for this program
        MOVWF   DECI                    ; ONES GPR = 1111 0101                                                                      GPR changed from ONES for this program
        DECFSZ  TEMPLO_F, F             ; Decrement TEMPLO_F register
        GOTO    $+D'4'                  ; Not 0, GOTO here + 4 instructions
        DECFSZ  TEMPHI_F, F             ; Decrement TEMPHI_F register
        GOTO    $+D'2'                  ; Not 0, skip next instruction
        GOTO    $+D'9'                  ; Is 0, done, GOTO here + 9 instructions
        INCFSZ  DECI, F                 ; Increment ONES register, skip if 0                                                        GPR changed from ONES for this program
        GOTO    $-D'6'                  ; Not 0, GOTO here - 6 instructions
        INCFSZ  ONES, F                 ; Is 0, Increment TENS register skip if 0                                                   GPR changed from TENS for this program
        GOTO    $-D'9'                  ; Not 0, GOTO here - 9 instructions & reset the ONES register to D'246'
        INCFSZ  TENS, F                 ; TENS overflowed, Increment HUNS skip if 0                                                 GPR changed from HUNS for this program
        GOTO    $-D'12'                 ; Not 0, GOTO here - 12 instructions & reset the ONES and TENS registers to D'246'
        INCF    HUNS, F                 ; HUNS overflowed, Increment THOUS                                                          GPR changed from THOUS for this program
        GOTO    $-D'15'                 ; GOTO here - 15 instructions & reset the ONES, TENS & HUNS registers to D'246'
  
        SUBWF   TENS, F                 ; W still holds D'246 so subtract it from HUNS register to determine how many 'HUNS' (0-9)   GPR changed from HUNS for this program
        SUBWF   ONES, F                 ; W still holds D'246 so subtract it from TENS register to determine how many 'TENS' (0-9)   GPR changed from TENS for this program
        SUBWF   DECI, F                 ; W still holds D'246 so subtract it from ONES register to determine how many 'ONES' (0-9)   GPR changed from ONES for this program
  
        GOTO    ZERO_SUPPRESS_AND_NEG_SIGN
  
  
DISPLAY_CELSIUS
        BCF     FLAGS, 1                ; Clear Negative Flag
        BTFSS   TEMPHI_C, 7             ; Test if Temperature is negative (2's complement format)
        GOTO    REARRANGE_DATA          ; NOT Negative, skip 'Undo 2's Complement'
        BSF     FLAGS, 1                ; Set Negative Flag; used later
  
 ;  Undo 2's Complement. When the Temperature is Negative, the DS18B20 outputs the 2's Complement. This routine converts the negative data into its positive equivalent
                                        ; Example Negative Temperature            TEMPHI = 1111 1110  TEMPLO = 0110 1110 = -25.1250 (see datasheet)
        COMF    TEMPLO_C, F             ; Invert TEMPLO 
        COMF    TEMPHI_C, F             ; Invert TEMPHI
        INCFSZ  TEMPLO_C, F             ; Increment TEMPLO once; After inverting the data, add one to achieve equivalent positive number (see 2's complement)
        GOTO    $+D'2'                  ; No overflow skip next instruction
        INCF    TEMPHI_C, F             ; Overflow occurred, increment upper byte
                                        ; Example Negative Temperature AFTER undo TEMPHI = 0000 0001  TEMPLO = 1001 0010 = +25.1250
  
REARRANGE_DATA ; This routine rearranges the 2 Temperature result bytes into usable data.
 ; EXPLANATION:
 ; * After the DS18B20 finishes a Temperature Conversion, the 12-bit result is stored in two 8-bit 
 ;   registers as a 16-bit sign-extended two?s complement number.
 ; * The 5 MSb's of the MSB (TEMPHI_C) indicate a negative or positive temperature.
 ; * The output data follows this format (12-bit resolution; 0.0625 degrees Celsius increments);
 ;
 ;         SSSS SNNN : NNNN FFFF      Where S = Sign (0 = Positive 1 = Negative)
 ;                                          N = Number, &
 ;                                          F = Fraction (16 X 0.0625 increments = 1)
 ;
 ;   After communication with the DS18B20 the output data is held in two 8 bit registers called 
 ;   TEMPHI_C & TEMPLO_C in the following format TEMPHI_C = SSSS SNNN : TEMPLO_C = NNNN FFFF
 ;         
 ;             Example;  0000 0011  1001 0001  =  (D'913' X 0.0625)  =  57.0625 Degrees Celsius
 ;
 ; * This 6 instruction routine below rearranges the 4 nibbles of TEMPHI_C & TEMPLO_C so the data 
 ;   is easy to work with. 
 ; * The MSB (TEMPHI_C) will be used for the 'whole' number i.e. 1's, 10's & 100's digits (57);
 ;   and the LSB (TEMPLO_C) will be used for the decimal digit (.1's) with the upper nibble of the 
 ;   LSB masked. (These are in fact the sign bits, but we have already handled negative data and 
 ;   created a Flag (FLAGS, 1) for negative temperatures.
 ;
 ;     Example above, after rearrangement 0011 1001  0000 0001  (see below)
 ;
                                        ; Example before; TEMPHI_C = 0000 0011  TEMPLO_C = 1001 0001   = 57.0625 Degrees Celsius
  
        MOVLW   B'11110000'             ; W = 1111 0000
        ANDWF   TEMPLO_C, W             ; F = 1001 0001  W = 1001 0000
        IORWF   TEMPHI_C, F             ;                F = 0000 0011   F = 1001 0011
        SWAPF   TEMPHI_C, F             ;                                F = 1001 0011   F = 0011 1001 (New TEMPHI_C)
        MOVLW   B'00001111'             ; W = 0000 1111
        ANDWF   TEMPLO_C, F             ; F = 1001 0001  F = 0000 0001 (New TEMPLO_C)
  
                                        ; Example after;  TEMPHI_C = 0011 1001  TEMPLO_C = 0000 0001
                                        ;                 = D'57' (X 1.0) = 57  = D'1' (X 0.0625) = 0.0625  = 57.0625 Degrees Celsius
  
BIN_TO_DEC_C  ; This routine converts the 8-bit number held in TEMPHI_C to Decimal and stores it in 3 GPR's; HUNS, TENS & ONES
 ; Example 225;
 ; HUNS holds amount of hundreds (i.e. 0000 0010 = 2x100)
 ; TENS holds amount of tens (i.e. 0000 0010 = 2x10)
 ; ONES holds amount of ones (i.e. 0000 0101 = 5x1)
 ; These registers are then used as jump pointers when calling digits to display
  
        INCF    TEMPHI_C                ; Pre-load TEMPHI + 1
        CLRF    HUNS                    ; HUNS = 0000 0000
  
        MOVLW   D'246'                  ; MOVE Decimal'246' to W
        MOVWF   TENS                    ; TENS GPR = 1111 0101
        MOVWF   ONES                    ; ONES GPR = 1111 0101
        DECFSZ  TEMPHI_C, F             ; Decrement TEMPHI register
        GOTO    $+D'2'                  ; NOT 0, skip next instruction
        GOTO    $+D'7'                  ; IS 0, TEMPHI = 0000 0000, skip next 6 instructions and calculate how many tens and hundreds
        INCFSZ  ONES, F                 ; Increment ONES register, skip if 0
        GOTO    $-D'4'                  ; NOT 0, GOTO here - 4 instructions
        INCFSZ  TENS, F                 ; IS 0, Increment TENS register skip if 0
        GOTO    $-D'7'                  ; GOTO here - 7 instructions & reset the ONES register to D'246'
        INCF    HUNS, F                 ; TENS overflowed, Increment HUNS
        GOTO    $-D'10'                 ; GOTO here - 10 instructions & reset the ONES and TENS registers to D'246'
  
        SUBWF   TENS, F                 ; W still holds D'246 so subtract it from TENS register to determine how many 'TENS'
        SUBWF   ONES, F                 ; W still holds D'246 so subtract it from ONES register to determine how many 'ONES'
  
FRACTION_ROUND  ; This routine rounds the fraction portion of the Temperature data to the nearest .1
; Provided by & used with his permission from 'Pommy'. Contact via Electro-Tech-Online.com
; Examples
; TENTHS = Fraction                           * 10 / 16
; TENTHS = 00000100 ( D'4' * 0.0625 = 0.2500 Degrees C) * 10 / 16 = 3
; TENTHS = 00001001 ( D'9' * 0.0625 = 0.5625 Degrees C) * 10 / 16 = 6
; TENTHS = 00001101 (D'13' * 0.0625 = 0.8125 Degrees C) * 10 / 16 = 8
        MOVLW   B'00001111'             ;
        ANDWF   TEMPLO_C, F             ;
        MOVF    TEMPLO_C, W             ;
        ADDWF   TEMPLO_C, F             ; *2, C=0
        RLF     TEMPLO_C, F             ; *4, C=0
        ADDWF   TEMPLO_C, F             ; *5, C=0
        RLF     TEMPLO_C, F             ; *10
        MOVLW   B'00001000'             ; 
        ADDWF   TEMPLO_C, F             ; Rounding
        SWAPF   TEMPLO_C, W             ; Pseudo divide by 16
        ANDLW   B'00001111'             ; 
        MOVWF   DECI                    ; 
  
  
ZERO_SUPPRESS_AND_NEG_SIGN        
  ; Leading zero suppression & minus (-) sign if Temperature Negative
        MOVF    HUNS, W                 ; MOVF instruction affects Z bit of STATUS register 
        BTFSC   STATUS, Z               ; Does HUNS = 0? 
        MOVLW   B'00001010'             ; Yes, jump Pointer for blank arrangement
  
        BTFSC   FLAGS, 1                ; Is Temperature Negative?
        MOVLW   B'00001011'             ; Yes, jump Pointer for '-' arrangement
        MOVWF   HUNS                    ; 
  
        BTFSS   STATUS, Z               ; Does HUNS = not 0?
        GOTO    $+D'5'                  ; Yes, do not blank TENS
        MOVF    TENS, W                 ; MOVF instruction affects Z bit of STATUS register 
        BTFSC   STATUS, Z               ; Does TENS = 0? 
        MOVLW   B'00001010'             ; Yes, jump Pointer for blank arrangement
        MOVWF   TENS                    ; 
  
  ; This routine would not be needed if interrupts were disabled for the whole BIN_TO_DEC routine. So this is added to minimise the amount of time interrupts are disabled to minimise flicker. 
        BCF     INTCON, GIE             ; Disable Interrupts
        MOVF    HUNS, W
        MOVWF   THOU_COL
        MOVF    TENS, W
        MOVWF   HUNS_COL
        MOVF    ONES, W
        MOVWF   TENS_COL
        MOVF    DECI, W
        MOVWF   ONES_COL
        BSF     INTCON, GIE             ; Enable Interrupts
  
        GOTO    GET_TEMP
END