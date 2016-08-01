;*******************************************************************
; Property of El Muffin Stuffer (Bethany Kon, Aaron Pan, David Wang)
; Version: 1.0
; Written By : Bethany Kon
;*******************************************************************

;Configuration
    list p=16f877                 ; list directive to define processor
      #include <p16f877.inc>        ; processor specific variable definitions
      __CONFIG _CP_OFF & _WDT_OFF & _BODEN_ON & _PWRTE_ON & _HS_OSC & _WRT_ENABLE_ON & _CPD_OFF & _LVP_OFF
      
      #include <rtc_macros.inc>
      #include <lcd.inc>	 
      #include <sensors.inc>
 
	udata_shr
        ADDR	    res 1	;Stores address in EEPROM where value is to be stored
	VALUE	    res 1	;Stores value to be stored in EEPROM
      
;Unbanked Variables 
	cblock  0x30
		Bin_Counter	    ;Hold address of Bin#
		Bin1_Dist	    ;0x31
		Bin2_Dist	    ;0x32
		Bin3_Dist	    ;0x33
		Bin4_Dist
		Bin5_Dist
		Bin6_Dist
		Bin7_Dist
		w_temp		    ;Saves W Reg on interrupt
		status_temp	    ;Saves STATUS Reg on interrupt
		Table_Counter	    ;Used to Display Messages in Display macro
		Bin1_State	    ;0x3B	Records the colour of the sticker on the bin, facing the robot
		Bin2_State
		Bin3_State
		Bin4_State
		Bin5_State
		Bin6_State	    ;0x40
		Bin7_State
		COUNTH		    ;Counts upper 8 bits for delays
		COUNTM		    ;Counts middle 8 bits or delays
		COUNTL		    ;Counts lower 8 bits for delays
		NumH		    ;Stores upper 8 bits of binary number to convert to decimal
		NumL		    ;Stores lower 8 bits of binary number to convert to decimal
		TenK		    ;Stores the TenK digit of Decimal number from BCD 
		Thou		    ;Stores the Thou digit of Decimal number from BCD
		Hund		    ;Stores the Hund digit of Decimal number from BCD
		Tens		    ;Stores the Tens digit of Decimal number from BCD
		Ones		    ;Stores the Ones digit of Decimal number from BCD
		Init_SecH
		Init_SecL
		Fin_SecH	    ;0x4E
		Fin_SecL
		EncNotch	    ;0x50   Records number of notches passed through the breakbeam sensor
		Pole_Position
		state_temp	    ;Temporarily stores the colour of the sticker on the bin
		key_val		    ;Stores the value of the key pressed
		front_temp
		back_temp
		DistanceH	    ;Record Total Distance in inches
		DistanceL		    
		shift_value	    ;Temporarily stores values from EEPROM 
		eeprom_counter
		eeprom_bin
		eeprom_clicks
		ISTHEREAPOLE
		Remote_Det
	endc
	
;Constants for Pin Assignments
	    #define	RS		PORTD,2
	    #define	E		PORTD,3
	    #define	KEYONE		b'00000000'	;Values of PORTB when certain keys are pressed
	    #define	KEYTWO		b'00000001'
	    #define	KEYTHREE	b'00000010'
	    #define	KEYA		b'00000011'
	    #define	KEYFOUR		b'00000100'
	    #define	KEYFIVE		b'00000101'
	    #define	KEYSIX		b'00000110'
	    #define	KEYB		b'00000111'
	    #define	KEYSEVEN    	b'00001000'	
	    #define	KEYC		b'00001011'

         ORG       0x0000     ;RESET vector must always be at 0x00
         goto      init       ;Just jump to the main code section.
	 
	 ORG	   0X0004     ;INTERRUPT vector
	 ;Disable GIE
	 bcf	   INTCON, 7
	 ;Save W and STATUS reg
	 movwf	   w_temp
	 movf	   STATUS, W
	 movwf	   status_temp
	 ;Interrupt Identification
	 clrf		STATUS
	 btfsc		INTCON, 1
	 goto		enc_int
	 goto		end_isr
	
	 ;Encoder Interrupt Service Routine
enc_int	 incf	EncNotch, W		;Increase Notches for each Interrupt
	 sublw	d'255'
	 btfsc	STATUS, Z
	 goto	end_op
	 incf	EncNotch, F
;Check Remote Emergency Stop	 
	 bsf	    ADCON0, 3
	 bsf	    ADCON0, 4
	 call	    adc
	 movf	    TenK, W
	 call	    Decimal
	 sublw	    0x30
	 btfsc	    STATUS, Z
	 goto	    chck_remote
	 bcf	INTCON, 1
	 goto	end_isr
chck_remote
	 incf	Remote_Det, F
	 movf	Remote_Det, W
	 sublw	0x01
	 btfsc	STATUS, Z
	 goto	remote_stop
	
end_isr	 ;Restore W and STATUS reg
	 movf		status_temp, W
	 movwf		STATUS
	 swapf		w_temp, F
	 swapf		w_temp, W
	 ;Enable GIE
	 bsf		INTCON, 7
	 retfie
		
;***************************************
; Look up table
;***************************************
Welcome_Msg1
	 addwf	    PCL,F
	 dt	    "1| Run Operation", 0
Welcome_Msg2
	 addwf	    PCL,F
	 dt	    "2| EEPROM", 0
Bin_Detect		
	 addwf	    PCL,F
	 dt	    "Bin Detected", 0
Decimal		
	 addwf	    PCL,F
	 dt	    "0123456789",0
Done
	 addwf	    PCL,F
	 dt	    "Terminated", 0
Prompt1
	 addwf	    PCL,F
	 dt	    "A|Time   B|Dist", 0
Prompt2
	 addwf	    PCL,F
	 dt	    "|Bin Info.",0 
ExBuck1
	 addwf	    PCL,F
	 dt	    "Position:", 0
ExBuck2
	 addwf	    PCL,F
	 dt	    "State:", 0	 
Op_Time
	 addwf	    PCL,F
	 dt	    "Time ", 0 
Op_Dist
	 addwf	    PCL,F
	 dt	    "Distance ", 0
Pole_Dist
	 addwf	    PCL,F
	 dt	    "Pole ", 0
Trial_Msg
	 addwf	    PCL, F
	 dt	    "A-C| Trials", 0
rs_Trial
	 addwf	    PCL, F
	 dt	    "TRIAL", 0
rs_Time
	 addwf	    PCL, F
	 dt	    "Date/Time of Run: ", 0
rs_Count
	 addwf	    PCL, F
	 dt	    "Number of Bins: ", 0
rs_Info
	 addwf	    PCL, F
	 dt	    "Bin Information: ", 0
	 
;*******************************************************************	
;   Macros
;*******************************************************************
	
;Display on LCD
Display macro	Message
		local	loop_
		local 	end_
		clrf	Table_Counter
		clrw		
loop_		
		movf	Table_Counter,W
		call 	Message
		xorlw	B'00000000' ;check WORK reg to see if 0 is returned
		btfsc	STATUS,Z
			goto	end_
		call	WR_DATA
		incf	Table_Counter,F
		goto	loop_
end_		
		clrf	Table_Counter
		endm
		
DELAY macro
	local	delay_loop
delay_loop
	decfsz COUNTH, f
	goto   $+2
	decfsz COUNTM, f
	goto   $+2
	decfsz COUNTL, f
	goto   delay_loop

	goto $+1
	nop
	nop
	endm
		
RECORD_TIME macro   SecH, SecL
		    clrf    SecH
		    clrf    SecL
		    ;Get seconds
		    rtc_read	0x00		;Read Address 0x00 from DS1307---seconds
		    movfw	0x77
		    movwf	SecH
		    movfw	0x78
		    movwf	SecL
		    endm
		 
BIN_INFO macro	    BinDist, BinState
	Display	    ExBuck1
	;Distance
	movf	    BinDist, W
	movwf	    Ones
	call	    calc_distance
	call	    display_distance
	;States
	call	    Second_Line
	Display	    ExBuck2
	STATES_INFO BinState
	endm
	
STATES_INFO macro   Stickers
	local	    show_Fblack
	local	    show_Bblack
	local	    next_state
	local	    fin_state
	btfss	    Stickers, 0
	goto	    show_Fblack
	call	    show_white
	goto	    next_state
show_Fblack
	call	    show_black
next_state
	btfss	    Stickers, 1
	goto	    show_Bblack
	call	    show_white
	goto	    fin_state
show_Bblack
	call	    show_black
fin_state
	call	    First_Line
	endm	

show_white
	movlw	    0x57
	call	    WR_DATA
	return
show_black
	movlw	    0x42
	call	    WR_DATA
	return
	
;*******************************************************************
;Initialize PIC     
;*******************************************************************
init
	 ;Bank 1
	 bcf	    STATUS, RP1	 
         bsf	    STATUS,RP0    
	 movlw	    b'011011'
	 movwf	    TRISA
	 movlw	    b'11111011'
	 movwf	    TRISB
	 movlw	    b'00011000'
	 movwf	    TRISC
	 movlw	    b'00000010'
	 movwf	    TRISD
	 movlw	    b'000'
	 movwf	    TRISE   
	 movlw	    b'111'	    ;Set Prescaler for TMR0 to 1:256
	 movwf	    OPTION_REG
	 ;Enable ADC Conversion for RA0, RA1, RA3
	 movlw	    b'00000100' ;b'00001110' ->enables ADC for just RA0
	 movwf	    ADCON1
	 
	 ;Bank 0
         bcf       STATUS,RP0     
         clrf      PORTA
         clrf      PORTB
         clrf      PORTC
         clrf      PORTD
	 clrf	   PORTE
	 ;Enable Interrupts
	 clrf	   INTCON
	 clrf	   PIR1
	 clrf	   PIE1
	 movlw	   b'10010000'
	 movwf	   INTCON
	 ;Enable ADC Conversion for RA0
	 movlw	    b'11000101'
	 movwf	    ADCON0
	 
	 ;Configure TMR1
	 movlw	    b'00110000'	;0x10
	 movwf	    T1CON
	 
	 movlw	   0x00		;Clear W reg
	 call	   i2c_common_setup	;Initialize Real-Time Clock communication
	 call      InitLCD		;Initialize the LCD
	 
	 
;*******************************************************************
;Main Program
;*******************************************************************
main	 ;Initialize Unbanked Variables
	 clrf	   EncNotch
	 clrf	   DistanceH
	 clrf	   DistanceL
	 clrf	   COUNTH
	 clrf	   Pole_Position
	 clrf	   Remote_Det
	 bcf	    PORTC, 0
	 movlw	   0x30
	 movwf	   Bin_Counter	

	 ;Initial Prompt to Begin Operation 
	 call	    Clear_Display
	 Display    Welcome_Msg1
	 call	    Second_Line
	 Display    Welcome_Msg2
	 call	    First_Line
	 
remote	 bsf	    ADCON0, 3
	 bsf	    ADCON0, 4
	 call	    adc
	 movf	    TenK, W
	 call	    Decimal
	 sublw	    0x30
	 btfsc	    STATUS, Z
	 goto	    remote_release
	 
;Wait for Key Press to Begin Operation	 
	 bcf	    PORTB, 1
begin	 btfss	    PORTB, 1	  
	 goto	    remote  ;$-1
	 swapf	    PORTB, w	    ;read PORTB<7:4> into W<3:0>
	 andlw	    0x0F 
	 movwf	    key_val
press1	 movlw	    KEYONE	    ;If key 1 is pressed, begin operation
	 xorwf	    key_val, W
	 btfss	    STATUS, Z
	 goto	    press2
	 goto	    run_stuffer
press2	 movlw	    KEYTWO	    ;If key 2 is pressed, display previous trials info
	 xorwf	    key_val, W
	 btfss	    STATUS, Z
	 goto	    press3
	 goto	    eeprom_display
press3	 movlw	    KEYTHREE
	 xorwf	    key_val, W
	 btfss	    STATUS, Z
	 goto	    loop
	 bsf	    PCLATH, 3
	 call	    rs232_init
	 bcf	    PCLATH, 3
loop	 btfsc	    PORTB, 1	    ;wait until key is released
	 goto	    $-1
	 goto	    main

;Wait for Remote Key to be Released
remote_release
	 bsf	    ADCON0, 3
	 bsf	    ADCON0, 4
	 call	    adc
	 movf	    TenK, W
	 call	    Decimal
	 sublw	    0x30
	 btfsc	    STATUS, Z 
	 goto	    remote_low
	 incf	    COUNTH, F
	 movf	    COUNTH, W
	 sublw	    0xFF
	 btfsc	    STATUS, Z
	 goto	    run_stuffer
	 goto	    remote_release
remote_low
	 clrf	    COUNTH
	 goto	    remote_release
	
;Begin Operation
run_stuffer	 
	 call	    Clear_Display
	 
	 call	    ultra_mid
	 call	    ultra_front
	 
;Re-enable Interrupts for Restart
	 movlw	   b'10010000'
	 movwf	   INTCON
	 
;Initialize Time
	 RECORD_TIME	Init_SecH, Init_SecL
;	 movf	    Init_SecH, W
;	 call	    WR_DATA
;	 movf	    Init_SecL, W
;	 call	    WR_DATA
 
;Set Driving Motors (motor 1 and motor 2)
	 ;Initialize PWM
	 bsf	    STATUS, RP0	    ;Bank 1
	 movlw	    b'00111111'	    ;set PWM period
	 movwf	    PR2
	 bcf	    STATUS, RP0	    ;Bank 0
	 movlw	    b'00111111'
	 movwf	    CCP1CON
	 movwf	    CCP2CON
	 movlw	    b'00000100'
	 movwf	    T2CON
	 ;Set Duty Cycle (Speed) to 100%
	 movlw	    0x64    ;0x3C  ;b'01100100'		LEFT MOTOR
	 movwf	    CCPR1L
	 movlw	    0x64    ;0x64   ;b'01100100'		RIGHT MOTOR
	 movwf	    CCPR2L
	 ;Move Robot
	 call	    move
	 
;Wait for Bin to Trigger Front Sensor OR Column to Trigger Arm Sensor
object_det
	 ;Subroutine only returns if bin is detected
	 call	    col_bin_det
;Bin is Detected!
close	 Display    Bin_Detect
	 incf	    Bin_Counter, F
	 ;Indicate a Bin Detection
	 bsf	    PORTC, 0
;Wait for Second Ultrasonic Sensor
	 call	    stick_det
;Get Sticker Colours from IR Sensor
	 call	    ir_det
;Store Bin Position and Sticker Colours
store	 call	    First_Line
	 call	    store_info
;Wait for Bin to Pass
;	 call	    HalfS
;	 call	    QuartS
	 movf	    EncNotch, W
	 addlw	    d'12'
	 movwf	    COUNTH
bin_not_passed	
	 movf	    COUNTH, W
	 subwf	    EncNotch, W
	 btfss	    STATUS, Z
	 goto	    bin_not_passed	 
 ;Turn off Bin Indication
bin_pass call	    Clear_Display
	 bcf	    PORTC, 0
;Loop to Wait for an Object Detection again
	 goto	    object_det

;END OF OPERATION - accessed if 7 Bins Detected or Total_Distance is greater than 400
end_op	 call	    Clear_Display
	 ;Prepare for Retreat - Stop robot and retract arm
	 call	    stop
	 call	    retract_arm
	 ;Power Full Speed
	 movlw	    d'0'		;Slow not encoder wheel
	 movwf	    CCPR2L
	 bsf	    PORTD, 0		    ;Reverse Wheel Rotation
	 call	    AvoidS1
	 movlw	    0x00		
	 movwf	    CCPR1L
	 movlw	    0x64		
	 movwf	    CCPR2L
	 call	    AvoidS2
	 movlw	    0x64		
	 movwf	    CCPR1L
	 
	 ;Travel Back to Start Line
	 movlw	    d'230'		;b'11111001'
	 movwf	    COUNTH
chck_remote_stop
	 bsf	    ADCON0, 3
	 bsf	    ADCON0, 4
	 call	    adc
	 movf	    TenK, W
	 call	    Decimal
	 sublw	    0x30
	 btfsc	    STATUS, Z
	 goto	    remote_stop
encon	 btfsc	    PORTB, 0
	 goto	    encon	 
ignore	 decf	    COUNTH, W
	 btfsc	    STATUS, Z
	 goto	    finish
	 decf	    COUNTH, F
enoff	 btfss	    PORTB, 0
	 goto	    enoff
	 goto	    chck_remote_stop	 
	 
finish	 bcf	    PORTD, 0		    ;Stop Robot
	 ;Extend Arm
	 ;call	    extend_arm

finish_test
;Store Total Time
	 RECORD_TIME	Fin_SecH, Fin_SecL
;	 movf	    Fin_SecH, W
;	 call	    WR_DATA
;	 movf	    Fin_SecL, W
;	 call	    WR_DATA
;	 call	    Second_Line
	 call	    calc_time
	 ;goto	    not
;Store Trial Information in EEPROM
	 call	    eeprom_store
	 call	    Clear_Display
;Display Prompt and Wait for Operator Interaction 
	 Display    Done
	 call	    HalfS
	 goto	    menu1
	 

;**********************************
;	Subroutines
;**********************************
;Move Robot Forward
move	;Output voltage to Rotate Wheels in the Forwards Direction
	 bsf	    PORTC, 5		;ENCODER MOTOR
	 bsf	    PORTE, 2
	 bcf	    PORTD, 0
	 return

;Stop Robot 
stop	;Output 0 Voltage to Stop Wheel Rotation
	 bcf	    PORTC, 5
	 bcf	    PORTE, 2
	 bcf	    PORTD, 0
	 return
	 
slow_stop
	 decf	    CCPR1L, F
	 decf	    CCPR2L, F
	 movf	    CCPR1L, W
	 sublw	    0x00
	 btfss	    STATUS, Z
	 goto	    slow_stop	 
	 call	    stop
	 movlw	    0x64  
	 movwf	    CCPR1L
	 movlw	    0x64  
	 movwf	    CCPR2L
	 return
	 
slow_start
	 call	    move
	 movlw	    0x00  
	 movwf	    CCPR1L
	 movlw	    0x00  
	 movwf	    CCPR2L
incr_speed
	 movf	    CCPR1L, W
	 sublw	    0x64
	 btfsc	    STATUS, Z
	 return
	 incf	    CCPR1L, F
	 incf	    CCPR2L, F
	 goto	    incr_speed
	 
remote_stop
	 call	    stop
	; bsf	    PORTC, 0
	 call	    HalfS
	 call	    HalfS
	 goto	    finish_test
	 
;Check for a Detection of a Bin or Column using Ultrasonic Sensors
col_bin_det
	 clrf	    ISTHEREAPOLE
;Check if Arm Sensor Triggered 
arm_det	 ;goto	    bin_det
	; call	    Clear_Display
	 ;movf	    Pole_Position, W
	 ;sublw	    0x00
	 ;btfss	    STATUS, Z
	 ;goto	    bin_det
chck_pole
	 call	    ultra_arm
	 movf	    TMR0, W
	 ;BCD Conversion
	 movwf	    NumL	
	 call	    bin8_BCD
	; call	    tenk_ir
	; goto	    arm_det
	 ;Check TenK = 0
	 movf	    Hund, W
	 call	    Decimal
	 sublw	    0x30
	 btfss	    STATUS, Z
	 goto	    bin_det	    ;Column not detected
arm_thou_chck
	 movf	    Tens, W
	 call	    Decimal
	 sublw	    0x30
	 btfss	    STATUS, Z
	 goto	    bin_det	    ;Column not detected	 
	 btfss	    ISTHEREAPOLE, 0
	 goto	    delay_arm
	 call	    avoid_column
	 goto	    bin_det
delay_arm   
	 bsf	    ISTHEREAPOLE, 0
	 call	    delay5ms
	 goto	    chck_pole
;Check if Front Sensor Triggered
bin_det	 call	    ultra_front
	 movf	    TMR0, W
	 ;BCD Conversion
	 movwf	    NumL
	 call	    bin8_BCD
	 ;Check TenK = 0
	 movf	    Hund, W
	 call	    Decimal
	 sublw	    0x30
	 btfsc	    STATUS, Z
	 goto	    thou_chck
	 goto	    arm_det	    ;Bin not detected
	 ;Check Thou = 0
thou_chck
	 movf	    Tens, W
	 call	    Decimal
	 sublw	    0x30
	 btfsc	    STATUS, Z
	 return			    ;Bin Detected!
	 goto	    arm_det	    ;Bin not detected

;Check for a Detection of Middle Ultrasonic Sensors
stick_det
	 call	    ultra_mid
	 movf	    TMR0, W
	 ;BCD Conversion
	 movwf	    NumL
	 call	    bin8_BCD
	 ;Check TenK = 0
	 movf	    Hund, W
	 call	    Decimal
	 sublw	    0x30
	 btfsc	    STATUS, Z
	 goto	    thou_chck_back
	 goto	    stick_det	    ;Stickers not detected
	 ;Check Thou = 0
thou_chck_back
	 movf	    Tens, W
	 call	    Decimal
	 sublw	    0x30
	 btfsc	    STATUS, Z
	 return			    ;Stickers Detected!
	 goto	    stick_det	    ;Stickers not detected


;Retract and Extend the Robot Arm to Avoid the Column
avoid_column  
	 clrf	    COUNTH
	 call	    slow_stop	    ;Stop Robot
	 ;Retract Arm
	 call	    retract_arm
	 ;Move Forward to Avoid Column
;	 movlw	    d'80' 
;	 movwf	    CCPR1L
	 call	    slow_start	    ;Move Robot
	 movf	    EncNotch, W
	 movwf	    Pole_Position
	 movf	    EncNotch, W
	 addlw	    d'11'
	 movwf	    COUNTH
past_column
	 movf	    COUNTH, W
	 subwf	    EncNotch, W
	 btfss	    STATUS, Z
	 goto	    past_column
	 call	    slow_stop	    ;Stop Robot
	 ;Extend Arm
	 call	    extend_arm
	 ;Column Avoided (y). Continue Operation
;	 movlw	    0x64  
;	 movwf	    CCPR1L
	 call	    slow_start
	 return

retract_arm
	 bsf	    PORTE, 0
	 call	    HalfS
	 call	    HalfS
	 call	    HalfS
	 call	    HalfS
	 call	    HalfS
	 call	    HalfS
	 call	    HalfS
	 bcf	    PORTE, 0
	 return
	 
extend_arm
	 bsf	    PORTE, 1
	 call	    HalfS
	 call	    HalfS
	 call	    HalfS
	 call	    HalfS
	 call	    HalfS
	 call	    HalfS
	 call	    HalfS
	 bcf	    PORTE, 1
	 return
	 
;Check IR Sensors for Colour Detection
ir_det	 bcf	    ADCON0, 4
	 bcf	    ADCON0, 3
	 call	    adc
	 call	    adc
	 movf	    TenK, W
	 sublw	    0x00
	 btfsc	    STATUS, Z
	 goto	    front_black
	 bsf	    state_temp, 0
	 goto	    next_ir
front_black
	 bcf	    state_temp, 0
next_ir	 bcf	    ADCON0, 4
	 bsf	    ADCON0, 3
	 call	    adc
	 call	    adc
	 movf	    TenK, W
	 sublw	    0x00
	 btfsc	    STATUS, Z
	 goto	    back_black
	 bsf	    state_temp, 1
	 return
back_black
	 bcf	    state_temp, 1
	 return
	 
;Analog to Digital Conversion
adc	 clrf	    ADRESH
	 clrf	    ADRESL
	 bsf	    ADCON0, 2
waitadc	 btfsc	    ADCON0, 2
	 goto	    waitadc
	 movf	    ADRESH, W
	 movwf	    NumH
	 movf	    ADRESL, W
	 movwf	    NumL
	 call	    bin16_BCD
	 return
	 
;TEMPORARY IR ADC DISPLAY
tenk_ir	movf	    TenK, W
	call	    Decimal
	call	    WR_DATA
thou_ir	movf	    Thou, W
	call	    Decimal
	call	    WR_DATA
hund_ir	movf	    Hund, W
	call	    Decimal
	call	    WR_DATA
ten_ir	movf	    Tens, W
	call	    Decimal
	call	    WR_DATA
one_ir	movf	    Ones, W
	call	    Decimal    
	call	    WR_DATA
	movlw	    " "
	call	    WR_DATA
	return
	 
;;Pre-Drive Correction 
;first_drivecorrect
;	 return 
	 
;Store Bin Information (Position, Front State, Back State)
store_info
	movf	    Bin_Counter, W
	movwf	    FSR
	;Load Distance
	movlw	    d'5'
	subwf	    EncNotch, W
	movwf	    INDF
	;Load IRSensor Readings
	movf	    Bin_Counter, W
	addlw	    d'10'
	movwf	    FSR
	movf	    state_temp, W
	movwf	    INDF
	return 
	
;Distance Calculation (cm) from Recorded Notches
calc_distance			    ;Every 5, Add 1
	 clrf	    DistanceH	
	 clrf	    DistanceL
	 movlw	    d'6'	    ;Divide Notches by 6
	 movwf	    COUNTH
div	 movf	    Ones, W	    ;Ones <- EncNotch
	 subwf	    COUNTH, W
	 btfsc	    STATUS, C
	 goto	    div_fin
	 movf	    COUNTH, W
	 addlw	    d'6'
	 movwf	    COUNTH
	 incf	    DistanceL, F
	 goto	    div
div_fin	 movf	    DistanceL, W	;Distance = (Ones + Tens) + DistanceL
	 addwf	    DistanceL, W
	 addwf	    DistanceL, W
	 addwf	    DistanceL, W
	 addwf	    DistanceL, W
	 movwf	    DistanceL
twobyte	 movf	    DistanceL, W
	 sublw	    0xFF	    
	 btfsc	    STATUS, Z		;DistanceL = FF -> overflow?
	 goto	    overbyte
	 incf	    DistanceL, F	;Increment DistanceL
	 movf	    Ones, W		;Check if Ones is Done
	 sublw	    0x00	 
	 btfsc	    STATUS, Z
	 return
	 decf	    Ones, F		;Decrement Ones
	 goto	    twobyte
overbyte    
	 incf	    DistanceH, F	;Increment Tens Digit
	 clrf	    DistanceL		;Clear Ones Digit
	 goto	    twobyte
	 
display_distance
	movf	    DistanceH, W
	movwf	    NumH
	movf	    DistanceL, W
	movwf	    NumL
	call	    bin16_BCD
	movf	    Hund, W
	call	    Decimal
	call	    WR_DATA
	movf	    Tens, W
	call	    Decimal
	call	    WR_DATA
	movf	    Ones, W
	call	    Decimal
	call	    WR_DATA
	return

;Calculate Time of Trial Run
calc_time
	movf	Fin_SecH, W
	subwf	Init_SecH, W
	btfsc	STATUS, C
	goto	less
greater
	movf	Init_SecH, W
	subwf	Fin_SecH, W
	movwf	NumH
	movf	Fin_SecL, W
	subwf	Init_SecL, W
	btfsc	STATUS, Z
	goto	g_nocarry
	movf	Fin_SecL, W
	subwf	Init_SecL, W
	btfsc	STATUS, C
	goto	g_carry
g_nocarry
	movf	Init_SecL, W
	subwf	Fin_SecL, W
	movwf	NumL
	goto	fin_time
g_carry
	decf	NumH, F
	movf	Fin_SecL, W
	subwf	Init_SecL, W
	sublw	d'10'
	movwf	NumL
	goto	fin_time	
less
	movf	Fin_SecH, W
	subwf	Init_SecH, W
	movwf	NumH
	movf	Fin_SecL, W
	subwf	Init_SecL, W
	btfsc	STATUS, Z
	goto	l_nocarry
	movf	Fin_SecL, W
	subwf	Init_SecL, W
	btfsc	STATUS, C
	goto	l_carry
l_nocarry	
	movf	Init_SecL, W
	subwf	Fin_SecL, W
	movwf	NumL
	movf	NumH, W
	sublw	d'6'
	movwf	NumH
	goto	fin_time
l_carry
	movf	Fin_SecL, W
	subwf	Init_SecL, W
	sublw	d'10'
	movwf	NumL
	movf	NumH, W
	sublw	d'5'
	movwf	NumH
fin_time
	movf	NumH, W
	addlw	0x30
	;call	WR_DATA
	movwf	Fin_SecH
	movf	NumL, W
	addlw	0x30
	;call	WR_DATA
	movwf	Fin_SecL
	;goto	not
	return
		 
	 
;**************************************************
; Operator Interaction - Information Transmission
;**************************************************
menu1	 call	    Clear_Display
	 Display    Prompt1
	 call	    Second_Line
	 movlw	    "1"
	 call	    WR_DATA
	 movlw	    "-"
	 call	    WR_DATA
	 movf	    Bin_Counter, W
	 call	    WR_DATA
	 Display    Prompt2
	 
menu2	 btfss	    PORTB, 1	    ;wait until key pressed
	 goto	    $-1
	 swapf	    PORTB, W	    ;read PORTB<7:4> into W<3:0>
	 andlw	    0x0F
	 movwf	    key_val
	 
	call	    Clear_Display
	
ChckA	movlw	    KEYA
	xorwf	    key_val, W
	btfss	    STATUS, Z
	goto	    ChckB
	Display	    Op_Time
	movf	    Fin_SecH, W
	call	    WR_DATA
	movf	    Fin_SecL, W
	call	    WR_DATA
	movlw	    "s"
	call	    WR_DATA
	goto	    cback
	
ChckB	movlw	    KEYB
	xorwf	    key_val, W
	btfss	    STATUS, Z
	goto	    ChckC
	Display	    Op_Dist
	movf	    EncNotch, W
	movwf	    Ones
	call	    calc_distance
	call	    display_distance
	movlw	    "c"
	call	    WR_DATA
	movlw	    "m"
	call	    WR_DATA
	goto	    cback
	
ChckC	movlw	    KEYC
	xorwf	    key_val, W
	btfss	    STATUS, Z
	goto	    ChckS
	Display	    Pole_Dist
	movf	    Pole_Position, W
	movwf	    Ones
	call	    calc_distance
	call	    display_distance
	movlw	    "c"
	call	    WR_DATA
	movlw	    "m"
	call	    WR_DATA
	goto	    cback
	
ChckS	movlw	    b'00001100'
	xorwf	    key_val, W
	btfss	    STATUS, Z
	goto	    Chck#
	goto	    menu1  
	
Chck#	movlw	    0xE
	xorwf	    key_val, W
	btfss	    STATUS, Z
	goto	    Chck1
	goto	    main
	
Chck1	movlw	    KEYONE
	xorwf	    key_val, W
	btfss	    STATUS, Z
	goto	    Chck2
	BIN_INFO    Bin1_Dist, Bin1_State
	goto	    cback
	
Chck2	movf	    Bin_Counter, W
	sublw	    0x31
	btfsc	    STATUS, Z
	goto	    cback
	movlw	    KEYTWO
	xorwf	    key_val, W
	btfss	    STATUS, Z
	goto	    Chck3
	BIN_INFO    Bin2_Dist, Bin2_State
	goto	    cback
	
Chck3	movf	    Bin_Counter, W
	sublw	    0x32
	btfsc	    STATUS, Z
	goto	    cback
	movlw	    KEYTHREE
	xorwf	    key_val, W
	btfss	    STATUS, Z
	goto	    Chck4
	BIN_INFO    Bin3_Dist, Bin3_State
	goto	    cback
	
Chck4	movf	    Bin_Counter, W
	sublw	    0x33
	btfsc	    STATUS, Z
	goto	    cback
	movlw	    KEYFOUR
	xorwf	    key_val, W
	btfss	    STATUS, Z
	goto	    Chck5
	BIN_INFO    Bin4_Dist, Bin4_State
	goto	    cback
	
Chck5	movf	    Bin_Counter, W
	sublw	    0x34
	btfsc	    STATUS, Z
	goto	    cback
	movlw	    KEYFIVE
	xorwf	    key_val, W
	btfss	    STATUS, Z
	goto	    Chck6
	BIN_INFO    Bin5_Dist, Bin5_State
	goto	    cback
	
Chck6	movf	    Bin_Counter, W
	sublw	    0x35
	btfsc	    STATUS, Z
	goto	    cback
	movlw	    KEYSIX
	xorwf	    key_val, W
	btfss	    STATUS, Z
	goto	    Chck7
	BIN_INFO    Bin6_Dist, Bin6_State
	goto	    cback
	
Chck7	movf	    Bin_Counter, W
	sublw	    0x36
	btfsc	    STATUS, Z
	goto	    cback
	movlw	    KEYSEVEN
	xorwf	    key_val, W
	btfss	    STATUS, Z
	goto	    cback
	BIN_INFO    Bin7_Dist, Bin7_State
	goto	    cback

cback	 btfsc	    PORTB, 1	    ;wait until key is released
	 goto	    $-1
	 
	 goto	    menu2
	 
not 	goto	    not
	 
;***************************************
; LCD control
;***************************************	 
First_Line
	 movlw	    b'10000000'
	 call	    WR_INS
	 return 
	 
Second_Line
	 movlw	    b'11000000'
	 call	    WR_INS
	 return
	 
Clear_Display
	 movlw	    b'00000001'
	 call	    WR_INS
	 return
	 
;***************************************
; Delays
;***************************************
;Delay: 0.5s
HalfS
      movlw 0x10
      movwf COUNTH
      movlw 0x7A
      movwf COUNTM
      movlw 0x06
      movwf COUNTL
      DELAY
		return

;Delay: 0.25s		
QuartS	
      movlw 0x88
      movwf COUNTH
      movlw 0xBD
      movwf COUNTM
      movlw 0x03
      movwf COUNTL
      DELAY
		return
		
TestS
	movlw	0x88
	movwf	COUNTH
	movlw	0xBD
	movwf	COUNTM
	movlw	0x01
	movwf	COUNTL
	DELAY
		return
AvoidS1
	movlw	 0x88
	movwf	COUNTH
	movlw	0xBD
	movwf	COUNTM
	movlw	0x30
	movwf	COUNTL
	DELAY
		return		
AvoidS2
	movlw	 0x88
	movwf	COUNTH
	movlw	0xBD
	movwf	COUNTM
	movlw	0x0F
	movwf	COUNTL
	DELAY
		return
    
    ;CITE - taken from website found by Sam Harrison
    ;---------------- Binary (16-bit) to BCD -----------------------
;
;bin8_BCD:	; --- Takes Binary.number in      NumL
;bin16_BCD:	; --- Takes Binary.number in NumH:NumL 
		; --> Returns decimal.form  in TenK:Thou:Hund:Tens:Ones
;
; Uses variables (for which YOU MUST have laid out space in a CBlock:
; NumH, NumL
; TenK, Thou, Hund, Tens, Ones

bin8_BCD
	clrF	NumH

bin16_BCD
        swapf   NumH, W
        andlw   0x0F
        addlw   0xF0
        movwf   Thou 
        addwf   Thou, F 
        addlw   0xE2 
        movwf   Hund 
        addlw   0x32 
        movwf   Ones 

        movf    NumH, W 
        andlw   0x0F 
        addwf   Hund, F 
        addwf   Hund, F 
        addwf   Ones, F 
        addlw   0xE9 
        movwf   Tens 
        addwf   Tens, F 
        addwf   Tens, F 

        swapf   NumL, W 
        andlw   0x0F 
        addwf   Tens, F 
        addwf   Ones, F 

        rlf    Tens, F 
        rlf    Ones, F 
        comf	Ones, F 
        rlf    Ones, F 

        movf    NumL, W 
        andlw   0x0F 
        addwf   Ones, F 
        rlf    Thou,F 

        movlw   0x07 
        movwf   TenK 

        movlw   0x0A
Lb1 
        decf    Tens, F 
        addwf   Ones, F 
        btfss   STATUS, C 
        goto	Lb1 
Lb2 
        decf    Hund, F 
        addwf   Tens, F 
        btfss   STATUS,C 
        goto	Lb2 
Lb3 
        decf    Thou, F 
        addwf   Hund, F 
        btfss   STATUS,C
        goto	Lb3 
Lb4 
        decf    TenK, F 
        addwf   Thou, F 
        btfss   STATUS,C 
        goto	Lb4 

        retlw   0
    
;;***************************************
;;   EEPROM 
;;***************************************
eeprom_display
	 btfsc	    PORTB, 1	    ;wait until key is released
	 goto	    $-1
	 call	Clear_Display
	 clrf	    COUNTH	;Trial A
	 clrf	    COUNTM	;Trial B
	 clrf	    COUNTL	;Trial C
	
eemenu	 Display    Trial_Msg
	 
eewait	 btfss	    PORTB, 1	    ;wait until key pressed
	 goto	    $-1
	 swapf	    PORTB, W	    ;read PORTB<7:4> into W<3:0>
	 andlw	    0x0F
	 movwf	    key_val
	 call	    Clear_Display
	
eeback	movlw	    b'00001100'
	xorwf	    key_val, W
	btfss	    STATUS, Z
	goto	    eeA
	btfsc	    PORTB, 1	    ;wait until key is released
	goto	    $-1
	goto	    main 
	
eeA	movlw	    KEYA
	xorwf	    key_val, W
	btfss	    STATUS, Z
	goto	    eeB
	clrf	    COUNTM
	clrf	    COUNTL
   ;Which Information to Display?
	btfss	    COUNTH, 0
	goto	    firstA
	call	    press_again
	bsf	    COUNTH, 0
	 movf	    eeprom_counter, W
	 subwf	    eeprom_clicks, W
	 btfsc	    STATUS, Z
	 bcf	    COUNTH, 0
	 goto	    eekey
 firstA  ;Most Recent Trial Information	
	movlw	    0x00
	movwf	    ADDR
	call	    First_Line
	call	    display_basic
	clrf	    eeprom_counter
	bsf	    COUNTH, 0
	goto	    eekey
	
eeB	movlw	    KEYB
	xorwf	    key_val, W
	btfss	    STATUS, Z
	goto	    eeC
	clrf	    COUNTH
	clrf	    COUNTL
   ;Which information to display?
	btfss	    COUNTM, 0
	goto	    firstB
	call	    press_again
	bsf	    COUNTM, 0
	 movf	    eeprom_counter, W
	 subwf	    eeprom_clicks, W
	 btfsc	    STATUS, Z
	 bcf	    COUNTM, 0
	 goto	    eekey
firstB   ;Most Recent Trial Information
	 movlw	    0x32
	movwf	    ADDR
	call	    read
	sublw	    0xFF
	btfsc	    STATUS, Z
	goto	    eeC
	movlw	    0x32
	movwf	    ADDR
	call	    First_Line
	call	    display_basic
	clrf	    eeprom_counter
	bsf	    COUNTM, 0
	goto	    eekey
	
eeC	movlw	    KEYC
	xorwf	    key_val, W
	btfss	    STATUS, Z
	goto	    eekey
	clrf	    COUNTH
	clrf	    COUNTM
   ;Which information to display?
	btfss	    COUNTL, 0
	goto	    firstC
	call	    press_again
	bsf	    COUNTL, 0
	 movf	    eeprom_counter, W
	 subwf	    eeprom_clicks, W
	 btfsc	    STATUS, Z
	 bcf	    COUNTL, 0
	 goto	    eekey
firstC   ;Most Recent Trial Information
	 movlw	    0x64
	movwf	    ADDR
	call	    read
	sublw	    0xFF
	btfsc	    STATUS, Z
	goto	    eekey
	movlw	    0x64
	movwf	    ADDR
	call	    First_Line
	call	    display_basic
	clrf	    eeprom_counter
	bsf	    COUNTL, 0
	
eekey	 btfsc	    PORTB, 1	    ;wait until key is released
	 goto	    $-1
	 goto	    eewait	
	 
	 
	 read	
	;Bank 2
	bsf	STATUS, RP1
	bcf	STATUS, RP0
	movf	ADDR, W			;Write address to read from
	movwf	EEADR
	;Bank 3
	bsf	STATUS, RP0
	bcf	EECON1, EEPGD		;Point to Data memory
	bsf	EECON1, RD		;Start read operation
	btfsc	EECON1, RD		;Wait for Read to Finish
	goto	$-1
	;Bank 2
	bcf	STATUS, RP0
	movf	EEDATA, W
	bcf	STATUS, RP0
	bcf	STATUS, RP1
	return
	
press_again
	 movf	    eeprom_counter, W
	 sublw	    0x00
	 btfss	    STATUS, Z
	 goto	    bins_display
	 movlw	    0x30
	 movwf	    eeprom_counter
   ;Show Bin Information
bins_display
	 call	    display_specific
	 return
	 
display_basic
 ;Show Trial Date and Time
	movlw		"2"		;Display Year
	call		WR_DATA
	movlw		"0"
	call		WR_DATA
	call		display_date
	movlw		"/"
	call		WR_DATA
	call		display_date	;Display Month
	movlw		"/"
	call		WR_DATA
	call		display_date	;Display Day
	movlw		" "
	call		WR_DATA
	call		display_date	;Display Time (Hour)
	movlw		":"
	call		WR_DATA
	call		display_date	;Display Time (Minutes)
	;incf		ADDR, F
   ;Show Operation Time
	call		Second_Line
	movlw		"T"
	call		WR_DATA
	movlw		"|"
	call		WR_DATA
	movlw		"0"
	call		WR_DATA
	movlw		":"
	call		WR_DATA		;Display Op Time (Seconds)
	call		read
	call		WR_DATA
	incf		ADDR, F
	call		read
	call		WR_DATA
	movlw		" "
	call		WR_DATA
   ;Show Operation Distance
	movlw		"D"
	call		WR_DATA
	movlw		"|"
	call		WR_DATA
	incf		ADDR, F
	call		read
	movwf		Ones
	call		calc_distance
	call		display_distance
   ;Show Bin Count
	movlw		" "
	call		WR_DATA
	movlw		"#"
	call		WR_DATA
	movlw		"|"
	call		WR_DATA
	incf		ADDR, F
	call		read
	movwf		eeprom_clicks
	call		read
	call		WR_DATA
	return

display_date
	call		read
	call		WR_DATA
	incf		ADDR, F
	call		read
	call		WR_DATA
	incf		ADDR, F
	return

display_specific
	call		First_Line
   ;Show Bin Number
	incf		eeprom_counter, F
	movf		eeprom_counter, W
	call		WR_DATA
	call		Second_Line
   ;Show Bin Position
	movlw		"P"
	call		WR_DATA
	movlw		"|"
	call		WR_DATA
	incf		ADDR, F
	call		read
	movwf	    	Ones
	call	    	calc_distance	
	call		display_distance
   ;Show Bin States
	movlw		" "
	call		WR_DATA
	movlw		"S"
	call		WR_DATA
	movlw		"|"
	call		WR_DATA
	incf		ADDR, F
	call		read
	movwf		eeprom_bin
	STATES_INFO	eeprom_bin
	return	
	 
eeprom_store
   ;Shift Trials
	call	storage_shift
	movlw	0x00
	movwf	ADDR
   ;Store Date and Time of Run
	rtc_read	0x06			;Record Year
	call		store_date
	rtc_read	0x05			;Record Month
	call		store_date
	rtc_read	0x04			;Record Day
	call		store_date
	rtc_read	0x02			;Record Hour
	call		store_date		
	rtc_read	0x01			;Record Minutes
	call		store_date
   ;Store Operation Time, Distance, and Pole Position
	movlw		0x4D
	movwf		FSR
basic_info
	incf		FSR, F
	movf		INDF, W
	movwf		VALUE
	call		write
	incf		ADDR, F
	movlw		0x50	
	subwf		FSR, W
	btfss		STATUS, Z
	goto		basic_info
   ;Store Bin Counter
	movf		Bin_Counter, W
	movwf		VALUE
	call		write
   ;Store Bin Information
	movlw		0x30
	movwf		FSR
bin_info
	incf		FSR, F			;Record Distances
	incf		ADDR, F
	movf		INDF, W
	movwf		VALUE
	call		write
	incf		ADDR, F
	movlw		d'10'
	addwf		FSR, F			;Record States
	movf		INDF, W
	movwf		VALUE
	call		write
	movlw		0x41
	subwf		FSR, W			;Done Recording?
	btfss		STATUS, Z
	goto		continue_storage
	return	
continue_storage
	movlw		d'10'
	subwf		FSR, F
	goto		bin_info

write	
	;Bank 2
	bcf	STATUS, RP0
	bsf	STATUS, RP1
	movf	ADDR, W			;Address to
	movwf	EEADR			;Write to
	movf	VALUE, W		;Data to
	movwf	EEDATA			;Write
	;Bank 3
	bsf	STATUS, RP0
	bcf	EECON1, WRERR		;Clear indication that device hsa been reset during write op
	bcf	EECON1, EEPGD		;Point to Data Memory
	bsf	EECON1, WREN		;Enable Writes
	movlw	0x55
	movwf	EECON2			;Write 55th to EECON2
	movlw	0xAA
	movwf	EECON2			;Write AAh to EECON2
	bsf	EECON1, WR		;Start write operation
	btfsc	EECON1, WR		;Wait for Write to Finish
	bcf	EECON1, WREN		;Disable Writes
	;Bank 0
	bcf	STATUS, RP0
	bcf	STATUS, RP1
	bcf	PIR2, EEIF		;Clear Write Interrupt Flag
	bcf	STATUS, RP0
	bcf	STATUS, RP1
	call	read
	call	WR_DATA
	return
	
storage_shift
	movlw	0x63
	movwf	ADDR
loop_shift
	call	read
	movwf	VALUE
	movlw	0x32
	addwf	ADDR, F
	call	write
   ;Check if done?
	movf	ADDR, W
	sublw	0x32
	btfsc	STATUS, Z
	return
	movlw	0x33
	subwf	ADDR, F
	goto	loop_shift

store_date
	movfw		0x77
	movwf		VALUE
	call		write
	incf		ADDR, F
	movfw		0x78
	movwf		VALUE
	call		write
	incf		ADDR, F
	return

	
;*****************************************
;	RS232
;*****************************************
;MACRO
rs232_display macro	rs232message
			local	loop_rs232
			local	end_rs232
			;Bank 0
			bcf	STATUS, RP0
			clrf	Table_Counter
			clrw
loop_rs232	
			movf	Table_Counter, W
			call	rs232message
			xorlw	b'00000000' ;Check W to see if 0 is returned
			btfsc	STATUS, Z
			goto	end_rs232
		
			movwf	TXREG
			;Bank 1
			bsf	STATUS, RP0
			btfss	TXSTA, 1	;Check TRMT bit in TXSTA (FSR) until TRMT=1
			goto	$-1
			;Bank 0
			bcf	STATUS, RP0
			incf	Table_Counter, F
			goto	loop_rs232
end_rs232
			clrf	Table_Counter
			endm
		
send_rs232
		movwf	TXREG
		;Bank 1
		bsf	STATUS, RP0
		btfss	TXSTA, 1	;Check TRMT bit in TXSTA (FSR) until TRMT=1
		goto	$-1
		;Bank 0
		bcf	STATUS, RP0
		return
	
rs232_newline
		movlw	d'13'
		call	send_rs232
		return
		
rs232_print
	call	    read
	call	    send_rs232
	incf	    ADDR, F
	return
	
	;PAGE 1
	ORG 0X0800	

rs232_init
;Set up USART for RS232
	;Bank 1
	bsf	    STATUS, RP0	    
	movlw	    d'31'	    ;BAUD rate 9600, 20MHz oscillator
	movwf	    SPBRG
	clrf	    TXSTA	    ;8 bits data, no, 1 stop
	;Bank 0
	bcf	    STATUS, RP0
	bsf	    RCSTA, SPEN	    ;Asynchronous serial port enable
	;Bank 1
	bsf	    STATUS, RP0
	bsf	    TXSTA, TXEN	    ;Transmit enable
		
rs232_main
	;Bank 0
	bcf	    STATUS, RP0
	bcf	    PCLATH, 3
	movlw	    0x00
	movwf	    ADDR
rs232_show
	rs232_display	rs_Trial
	call		rs232_newline
    ;Date
	movlw	    "2"		    ;Year
	call	    send_rs232
	movlw	    "0"
	call	    send_rs232
	call	    rs232_print
	call	    rs232_print
	movlw	    "/"
	call	    send_rs232
	call	    rs232_print	    ;Month
	call	    rs232_print
	movlw	    "/"
	call	    send_rs232
	call	    rs232_print	    ;Day
	call	    rs232_print
	movlw	    " "
	call	    send_rs232
	call	    rs232_print	    ;Hour
	call	    rs232_print
	movlw	    ":"
	call	    send_rs232
	call	    rs232_print	    ;Minutes
	call	    rs232_print
	call	    rs232_newline
    ;Operation Time
	rs232_display	Op_Time
	call	    rs232_print	 
	call	    rs232_print
	movlw	    "s"
	call	    send_rs232
	call	    rs232_newline
   ;Operation Distance
	rs232_display	Op_Dist
	call	    read
	movwf	    Ones
	call	    calc_distance
	movf	    Hund, W
	call	    Decimal
	call	    send_rs232
	movf	    Tens, W
	call	    Decimal
	call	    send_rs232
	movf	    Ones, W
	call	    Decimal
	call	    send_rs232
	incf	    ADDR, F
	call	    rs232_newline
   ;Bin Count
	rs232_display	rs_Count
	call	    rs232_print
	call	    rs232_newline
	clrf	    COUNTL
   ;Bin Information
rs232_bins
	incf	    COUNTL, F
	rs232_display	rs_Info
	call	    rs232_newline
	call	    read
	movwf	    Ones
	call	    calc_distance
	movf	    Hund, W
	call	    Decimal
	call	    send_rs232
	movf	    Tens, W
	call	    Decimal
	call	    send_rs232
	movf	    Ones, W
	call	    Decimal
	call	    send_rs232
	incf	    ADDR, F
	movlw	    " "
	call	    send_rs232
	call	    read
	movwf	    COUNTH
	btfsc	    COUNTH, 0
	goto	    rs_front
	movlw	    "B"
	call	    send_rs232
	goto	    rs_next
rs_front
	movlw	    "W"
	call	    send_rs232
rs_next
	btfsc	    COUNTH, 1
	goto	    rs_back
	movlw	    "B"
	call	    send_rs232
	goto	    rs_fin
rs_back
	movlw	    "W"
	call	    send_rs232
rs_fin
	call	    rs232_newline
	movf	    COUNTL, W
	sublw	    0x07
	btfss	    STATUS, Z
	goto	    rs232_bins
	return
    
	END
