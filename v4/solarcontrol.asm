; *** Manuel Ramos, Jr.,  Tue Apr  6 16:26:09 PHT 2010 ***
;
; *** check maximum controller output limit using IBAT + ILOAD ***
;
; *** zener diode is 2.8V -> 0x09000 for IBAT=0
;
; *** based on solarcontrol3.asm ***
;
; *** serial comm, standard asuart pins ***
; *** A/D converter ***
; *** hex 2 ascii ***
; *** basic string table ***
;
; port A[0:4] and port B[7] configured for analog input AN[0:4,6]
; port A[5,7] configured output. port A[7] for LDC
; port A[6] should be an input (by default) for 12 V battery
; port B[0] configure as digital output for PWM
; port B[1:6] configured as inputs for TEMP, serial rx/tx function
;
; verified 8-bit subtraction gives proper results
; see test_subtract.asm
; 
; max powerpoint (max current charge to battery)
; use 10-bit resolution in comparison
;
; fixed bug in chk_bat for LDC, Mon Oct 12 15:39:06 PHT 2009
;
; communicate with ds1820 temp sensor, Mon Oct 12 23:42:17 PHT 2009
;
; adjusted wait time for get_ad routine, Thu Nov  5 13:46:53 PHT 2009
; added charge limit, Thu Nov  5 13:46:53 PHT 2009
;
;
; ***
; 
; fixed bug in sum_add routine
;
; faster readings/updates on voltage and current sense
; incorporates PWM code on PB[0], moved LED to PB[6] 
; readings sequence is ra0, ra1, ra2, ra3, ra4, ra7
;  which should give output sequence [vl, vb, vs, il, ib, is] on solar_eth_pwm_v4.sch/brd
;

	list      p=16f88           ; list directive to define processor
	#include <p16f88.inc>        ; processor specific variable definitions

	errorlevel  -302              ; suppress message 302 from list file

	__CONFIG    _CONFIG1, _CP_OFF & _CCP1_RB0 & _DEBUG_OFF & _WRT_PROTECT_OFF & _CPD_OFF & _LVP_OFF & _BODEN_OFF & _MCLR_OFF & _PWRTE_ON & _WDT_OFF & _INTRC_IO
	__CONFIG    _CONFIG2, _IESO_OFF & _FCMEN_OFF

; '__CONFIG' directive is used to embed configuration word within .asm file.
; The labels following the directive are located in the respective .inc file.
; See data sheet for additional information on configuration word settings.



; **** VARIABLE DEFINITIONS ***
; w_temp        EQU     0x71        ; variable used for context saving 
; status_temp   EQU     0x72        ; variable used for context saving
pclath_temp   EQU     0x73	  ; variable used for context saving


; *** Special Function Registers ***
; see /usr/local/share/gputils/header/p16f88.inc

; *** EECON1 bits ***
; see /usr/local/share/gputils/header/p16f88.inc

; *** User Registers ***

CNT1    equ     0x20
CNT2    equ     0x21
CNT3    equ     0x22
TEMP	equ	0x23

ADCHAN	equ	0x24
TXDATA	equ	0x25
RXDATA	equ	0x26

HEXNUM	equ	0x27
AHN	equ	0x28
ALN	equ	0x29

CNT4	equ	0x2A
CNT5	equ	0x2B

STRADD	equ 	0x30

MP_PWM	equ	0x37
MAXPWRH	equ	0x38
MAXPWRL	equ	0x39
MP_CNT	equ	0x3A

TRICKLE	equ	0x3B

SUMADDC	equ	0x3C

TMP	equ	0x40

BUFPTR	equ	0x44
STATFND	equ	0x45

TMPH	equ	0x49
TMPL	equ	0x4A

VPTR	equ	0x4B

ISUNH	equ	0x4C		; make sure this starts on an even byte
ISUNL	equ	0x4D		; scale factor for ISUN and ILOAD is 0.165
ILOADH	equ	0x4E		; scale factor for ILOAD is 0.08 with 1.8V offset
ILOADL	equ	0x4F
IBATH	equ	0x50
IBATL	equ	0x51
VSUNH	equ	0x52		; storage for voltage readings
VSUNL	equ	0x53		; scale factor is 10/(10+90.9)
VLOADH	equ	0x54
VLOADL	equ	0x55
VBATH	equ	0x56
VBATL	equ	0x57

EXTRA0	equ	0x58
EXTRA1	equ	0x59
EXTRA2	equ	0x5A
EXTRA3	equ	0x5B
EXTRA4	equ	0x5C
EXTRA5	equ	0x5D

SUM3	equ	0x5E		; following registers used for multiplication
SUM2	equ	0x5F		; good idea to have this follow sense readings
SUM1	equ	0x60		; for easy access
SUM0	equ	0x61

FACTOR1	equ	0x62
FACTOR0	equ	0x63

ADDEND3	equ	0x64
ADDEND2	equ	0x65
ADDEND1	equ	0x66
ADDEND0	equ	0x67

XHB	equ	0x68
XLB	equ	0x69	
YHB	equ	0x6A
YLB	equ	0x6B

ARRAY	equ	0x70		; need 8 bytes for ARRAY
ADDRL	equ	0x78		; ADDRL, ADDRH need to be here for flash access
ADDRH	equ	0x79		; since 0x70-0x7F appears in all banks



; *** Bit Definitions ***

#define		IRP_bit	STATUS,7  ; register bank select (indirect addressing)
#define		RP1_bit	STATUS,6  ; register bank select (direct addressing)
#define		RP0_bit	STATUS,5  ; register bank select (direct addressing)

#define 	LED     PORTB,6   ; LED on the RB6 pin (not installed)
#define		PWMOUT	PORTB,0   ; pin for PWM output
#define		LDC	PORTA,7   ; pin for load disconnect (connect when low)
#define		BMODE	PORTA,6   ; 12/24 battery mode (set as input for 12 V)
#define 	DSTEMP  PORTB,1   ; DS1820 on the RB1 pin



; *** constants for default register settings ***

AD0SET	equ	B'01000001'	; configure chan 0, ADON = 1, FOSC/8 to match 4 MHz clock

ISUN	equ	0		; A/D channels for current and voltage senses
ILOAD	equ	1
IBAT	equ	2
VSUN	equ	3
VLOAD	equ	4
VBAT	equ	6

; *** battery voltage settings ***
; *** dec2hex(floor(V*10/(90.9+10)/5*255)) ***

V11P3	equ	0x39		; equivalent MSByte for 11.3V
V12P0	equ	0x3C		; for 12.0V
V12P7	equ	0x40
V13P2	equ	0x42
V13P9	equ	0x46
V14P2	equ	0x47
V15P0	equ	0x4B
V15P5	equ	0x4E

VTHRL	equ	V11P3		; low voltage threshold
VFLOAT	equ 	V13P2		; float voltage
VTHRH	equ	V15P5		; high voltage threshold
VFULL	equ	V12P7		; voltage at full charge
VEQUAL	equ	V13P9		; voltage for equalization

; IBAT0A	equ	0x90		; equivalent to IBAT = 0A v3-01:10
; IBAT0A	equ	0x95		; equivalent to IBAT = 0A for v3-11:12
; IBAT0A	equ	0x98		; equivalent to IBAT = 0A for v3-13
; IBAT0A	equ	0x99		; equivalent to IBAT = 0A for v3-14
; IBAT0A	equ	0x95		; equivalent to IBAT = 0A for v3-15
IBAT0A	equ	0x98		; equivalent to IBAT = 0A for v3-16

IBAT1A	equ	IBAT0A+0x05	; equivalent MSByte for IBAT = 1A
IBAT4A	equ	IBAT0A+0x10	; equivalent MSByte for IBAT = 4A
IBAT4P2	equ	IBAT0A+0x12	; equivalent MSByte for IBAT = 4.2A

; IBATMAX	equ	IBAT4P2
; IBATMAX	equ	0xB0		; equivalent to 4.2A for v3-01
; IBATMAX	equ	0xA6		; equivalent to 4.2A for v3-02
; IBATMAX	equ	0xA8		; equivalent to 4.2A for v3-03
; IBATMAX	equ	0xAF		; equivalent to 4.2A for v3-04
; IBATMAX	equ	0xAE		; equivalent to 4.2A for v3-05
; IBATMAX	equ	0xB1		; equivalent to 4.2A for v3-06
; IBATMAX	equ	0xA8		; equivalent to 4.2A for v3-07
; IBATMAX	equ	0xAE		; equivalent to 4.2A for v3-08
; IBATMAX	equ	0xA8		; equivalent to 4.8A for v3-09
; IBATMAX	equ	0xAE		; equivalent to 4.2A for v3-10
; IBATMAX	equ	0xA9		; equivalent to 4.2A for v3-11

; IBATMAX	equ	0xA7		; equivalent to 4.2A for v3-12

; IBATMAX	equ	0xB5		; equivalent to 4.2A for v3-13
; IBATMAX	equ	0xB5		; equivalent to 4.7A for v3-14
; IBATMAX	equ	0xB3		; equivalent to 4.8A for v3-15
IBATMAX	equ	0xB3		; equivalent to 4.8A for v3-16




; *** reset vectors ***

	ORG     0x000             ; processor reset vector
	goto    normal_entry	  ; go to beginning of program

	ORG	0x004
	goto	interrupt_handler
	
	ORG	0x020

; *** entry point for normal program from flash loader ***
; *** do not remove/move/alter "goto main" command ***
; *** this needs to be here for loader to transfer ***
; *** control back to main program ***

normal_entry
	goto	main


; *** subroutines ****

; *** delay for 25 msec at 4 MHz ***

ms25  	movlw   D'25'
        movwf   CNT2            ; load 2nd loop register
l_a     movlw   D'250'
        movwf   CNT1            ; load 1st loop register
l_b     nop
	decfsz  CNT1,f          ; decrement 1st loop register
        goto    l_b          	; jump if register=0
        decfsz  CNT2,f          ; decrement 2nd loop register
        goto    l_a          	; jump if register=0
        return                  ; return to the main program

; *** delay for 100 msec at 4 MHz ***

ms100  	movlw   D'100'
        movwf   CNT2            ; load 2nd loop register
loop_a  movlw   D'250'
        movwf   CNT1            ; load 1st loop register
loop_b  nop
	decfsz  CNT1,f          ; decrement 1st loop register
        goto    loop_b          ; jump if register=0
        decfsz  CNT2,f          ; decrement 2nd loop register
        goto    loop_a          ; jump if register=0
        return                  ; return to the main program

; *** delay for 200 msec ***

ms200	call 	ms100
	call	ms100
	return

; *** delay for 1/2 sec ***

halfsec call	ms200
	call	ms200
	call	ms100
        return                  ; return to the main program

; *** delay for 1 second ***

onesec	call 	halfsec
	call	halfsec
	return

; *** delay for 60 usec at 4 MHz ***

us60  	movlw   D'15'
        movwf   CNT1            ; load 1st loop register
	goto	loop4us		; use same 4 us loop as 480 us delay

; *** delay for 480 usec at 4 MHz ***

us480  	movlw   D'120'
        movwf   CNT1            ; load 1st loop register
loop4us nop			; 4 us loop at 4 MHz
	decfsz  CNT1,f          ; decrement 1st loop register
        goto    loop4us         ; jump if register=0
        return                  ; return to the main program



; *** acquire analog input ***

get_ad	movf	ADCHAN,W	; channel selected is in ADCHAN
	movwf	TEMP
	
	rlf	TEMP,f		; position bits to add to
	rlf	TEMP,f		; ADCON0 default setting
	rlf	TEMP,f

	movlw	AD0SET		; compose ADCON0 setting
	addwf	TEMP,W	        ; using AD0SET and selected channel from ADCHAN
        movwf   ADCON0         	; write to ADCON0 (switches the channel)

	movlw	D'100'		; wait acquisition time
	movwf	CNT1		; about x us
acquire	decfsz	CNT1,f		
	goto 	acquire

	bsf	ADCON0,GO_DONE	; start conversion
convert	btfsc	ADCON0,GO_DONE	; wait until conversion done
	goto 	convert
	
	return			; result in ADRESH and ADRESL


; *** hex to ascii convert, convert byte in HEXNUM ***

n2s	clrf	AHN
	swapf	HEXNUM,W	; start with high nibble

nibble	andlw	0x0F		; work on the nibble
	movwf	ALN		; store it temporarily to ascii low nibble
	sublw	D'9'		; check if [0..9] or [A..F]
	btfss	STATUS,C	; skip if no borrow (ALN <= 9)
	goto	hexa2f
	movlw	0x30		; offset for ascii numbers
	goto	ascii
hexa2f	movlw	0x37		; offset for ascii alphas
ascii	addwf	ALN,f

	movf	AHN,f		; test if AHN is zero
	btfss	STATUS,Z	; if zero, store ALN in AHN
	goto	aln_ok
	movf	ALN,W		; get ALN
	movwf	AHN		; put it into AHN
	movf	HEXNUM,W	; reload W and process lower nibble
	goto	nibble

aln_ok	return			; result in AHN and ALN


; *** hex to ascii convert (byte in HEXNUM) and then transmit with ser tx ***

n2s_tx 	call	n2s		; convert from hex to two-ascii char

	movf	AHN,W
	movwf	TXDATA	
	call	ser_tx		; send high nibble
	movf	ALN,W
	movwf	TXDATA	
	call	ser_tx		; send low nibble

	return

; *** same as n2s_tx except with additonal space character as byte is printed ***

n2s_txs call	n2s_tx		; do exactly as n2s_tx

	movlw	0x20		; put a space
	movwf	TXDATA
	call	ser_tx

	return

; *** print string, string location in [PCLATH STRADD] ***

print 	movf	STRADD,W
	call	string		; W contains offset into string/table
	addlw	0x00		; test if end of string (char = 0x00)
	btfsc	STATUS,Z
	goto	end_pr
	movwf	TXDATA
	call	ser_tx
	incf	STRADD,f
	call	ms200
	goto	print

end_pr	return


; *** talk to the wrap board ***

snd_ad	movlw	0x03		; send control-C twice
	movwf	TXDATA
	call	ser_tx		
	call 	halfsec
	call	ser_tx

	call	onesec

	movlw	0x0D		; send CR twice
	movwf	TXDATA
	call	ser_tx
	call 	halfsec
	call	ser_tx

	call	onesec

	movlw	0x04		; send control-D twice
	movwf	TXDATA
	call	ser_tx		
	call 	halfsec
	call	ser_tx

	call	onesec

	movlw	0x0D		; send CR twice
	movwf	TXDATA
	call	ser_tx
	call 	halfsec
	call	ser_tx

	call 	onesec

	movlw	high(string2)	; preload PCLATH with high byte of
	movwf	PCLATH		; string/table address

	movlw	low(string2)	; send login
	movwf	STRADD		
	call	print

	call	onesec
	
	movlw	low(string3)	; send password
	movwf	STRADD		
	call	print

	call	onesec
	
	movlw	low(string4)	; send first half of command
	movwf	STRADD
	call	print

	movlw	0x00
	movwf	ADCHAN		; start at channel 0

sample	call	readvi		; read voltages and currents, and transmit through serial

	movlw	low(string5)	; send second half of command
	movwf	STRADD
	call	print

	call 	onesec
	call	onesec

	return


; *** read and transmit voltages and currents ***

readvi	call 	getvolt
;	call	calcpwr

	movlw	0x0F		; get duty low nibble of CCPR1L (PWM duty cycle setting)
	andwf	CCPR1L,W
	addwf	VLOADL,f	; add PWM setting value to VLOADL (assumes low nibble is unused)

	movlw	0x0F		; get low nibble of TRICKLE status
	andwf	TRICKLE,W
	addwf	VBATL,f		; add TRICKLE status to VBATL (assumes low nibble is unused)

	call 	ser_vi		; send out voltage and current readings to serial interface

	return


; **** read and store voltages (and current readings) ***
; *** Wed Sep  2 02:05:55 PHT 2009 ***
; *** updated for new charge controller circuit ***

getvolt	movlw	ISUNH		; initialize VPTR pointer to ISUNH to store voltages
	movwf	VPTR

	movlw	0x00
	movwf	ADCHAN		; start at channel 0

voltage	call	get_ad		; acquire analog input ADCHAN, result in ADRES[H:L]

	movf	ADRESH,W	; convert and send out high byte of result
	movwf	HEXNUM		; put result in HEXNUM for conversion
;	call	n2s_txs	

	movf	VPTR,W		; do indirect addressing
	movwf	FSR		; to store voltages
	movf	HEXNUM,W
	movwf	INDF
	incf	VPTR,f		; move pointer

	movlw   ADRESL		; convert and send out low byte of result
        movwf   FSR             ; indirect address ADRESL
        movf    INDF,W         	; get ADRESL
	movwf	HEXNUM		; put result in HEXNUM for conversion
;	call	n2s_txs	

	movf	VPTR,W		; do indirect addressing
	movwf	FSR		; to store voltages
	movf	HEXNUM,W
	movwf	INDF
	incf	VPTR,f		; move pointer
	
	incf	ADCHAN,f	; point to the next channel
	movf	ADCHAN,W
	sublw	0x05
	btfsc	STATUS,Z	; skip channel ADCHAN=5
	incf	ADCHAN,f	; VBAT sense is at ADCHAN=6

	movf	ADCHAN,W
	sublw	0x07
	btfss	STATUS,Z	; check if done with all channels
	goto	voltage		; sample next channel

	return



; *** compute power from v3 and i3 ***
; *** result in SUMx ***

; *** Wed Sep  2 05:01:44 PHT 2009 ***
; *** updated to computed power delivered by the panel *** 

calcpwr	movf	VSUNL,W		; do v * i
	movwf	XLB
	movf	VSUNH,W
	movwf	XHB

	movf	ISUNL,W		; solar panel current
	movwf	YLB
	movf	ISUNH,W
	movwf	YHB

	call	multxy

	return


; *** transmit stored readings (voltages and curents) through serial ***

ser_vi  movlw	ISUNH
        movwf   FSR             ; indirect address starting at ISUNH

next_pr movf    INDF,W         	; get the byte
	movwf	HEXNUM		; put result in HEXNUM for conversion
	call	n2s_tx

	movlw	0x20		; load TXDATA with space character
	movwf	TXDATA
	btfsc	FSR,0		; check if LS byte just printed
	call	ser_tx		; send an additional space character	

	incf	FSR,f
	movf	FSR,W
	sublw	ISUNH+D'22'	; print 22 bytes include extras and multiplication result
	btfss	STATUS,Z	; check if done with all 22 consecutive bytes
	goto	next_pr		; print next byte

	movlw	0x0D		; put a return
	movwf	TXDATA
	call	ser_tx

	return


; *** add unsigned 32 bit numbers, sum = sum + addend ***
; *** add ADDEND to SUM ***
; *** make sure ADDENDx and SUMx are in the directly addressable space ***

sum_add	clrf	SUMADDC

addnxt0	movf	ADDEND0,W
	addwf	SUM0,f
	btfsc	STATUS,C
	bsf	SUMADDC,0

	btfss	SUMADDC,0
	goto	addnxt1
	incf	SUM1,f
	btfsc	STATUS,Z
	bsf	SUMADDC,1
	
addnxt1 movf	ADDEND1,W
	addwf	SUM1,f
	btfsc	STATUS,C
	bsf	SUMADDC,1

	btfss	SUMADDC,1
	goto	addnxt2
	incf	SUM2,f
	btfsc	STATUS,Z
	bsf	SUMADDC,2

addnxt2	movf	ADDEND2,W
	addwf	SUM2,f
	btfsc	STATUS,C
	bsf	SUMADDC,2

	btfss	SUMADDC,2
	goto	addnxt3
	incf	SUM3,f
	btfsc	STATUS,Z
	bsf	SUMADDC,3

addnxt3	movf	ADDEND3,W
	addwf	SUM3,f
	btfsc	STATUS,C
	bsf	SUMADDC,3
	
	return


; *** multiply two unsigned 16-bit number, [xhb	xlb] * [yhb ylb] ***
; *** result in SUMx ***


multxy	clrf	SUM0		; sum = 0
	clrf	SUM1
	clrf	SUM2
	clrf	SUM3

	movf	XLB,W		; addend = [0 0 xhb xlb]
	movwf	ADDEND0
	movf	XHB,W
	movwf	ADDEND1
	clrf	ADDEND2
	clrf	ADDEND3

	movf	YLB,W
	movwf	FACTOR0
	movf	YHB,W
	movwf	FACTOR1

	movlw	D'16'		; do the following rotate (check), add, rotate 16 times
	movwf	TEMP

rr_add	rrf	FACTOR1,f	; rotate right 16-bit factor through carry bit
	rrf	FACTOR0,f

	btfsc	STATUS,C
	call	sum_add		; factor bit is set, increment sum by addend

	rlf	ADDEND0,f
	rlf	ADDEND1,f
	rlf	ADDEND2,f
	rlf	ADDEND3,f

	decfsz	TEMP,f
	goto	rr_add

	return


; *** check battery status ***
; *** connect/disconnect load depending on battery voltage ***	

chk_bat
	call	charge_limit	; check if output current is absolute max
	
;	call	getvolt
	call 	readvi
	movf	VBATH,w		; check if VBAT > minimum voltage
	sublw	VTHRL		; VTHRL - VBATH		 

	btfss	STATUS,C	; skip if VTHRL > VBATH
	goto	load_on		; VTHRL < VBATH, connect load

	sublw	0x02		; put some hysterisis, 0x02 - (VTHRL - VBATH)
	btfss	STATUS,C	; skip if VTHRL - VBATH < 0x02
	bcf	LDC		; VTHRL - VBTH > 0x02, disconnect load
	goto	end_bat

load_on	bsf	LDC		; connect load, drive npn transistor

	movf	VBATH,w		; check if VBAT > maximum voltage
	sublw	VTHRH		; VTHRH - VBATH
	btfss	STATUS,C	; skip if VTHRH > VBATH
	goto	chk_nocharge	; battery voltage > maximum, check no charge 
	
	clrf	TRICKLE		; VBATH < VTHRH, clear TRICKLE status
	goto	end_bat		; battery safe, exit

chk_nocharge			; check no charge voltage

	movf	CCPR1L,w	; store PWM setting
	movwf	MP_PWM		

chk_charge
	call	halfsec
;	call    getvolt
	call	readvi
        movf    IBATH,w
        sublw   IBAT0A          ; IBAT0A - IBATH

        btfss   STATUS,C        ; skip if 0A > IBAT
        goto    dec_charge      ; IBAT > 0A, decrease charge current
        goto    inc_charge   	; IBAT < 0A, increase charge a little

dec_charge
        incf    CCPR1L,w        ; check if converter output at min
        andlw   0x0F
        btfsc   STATUS,Z
        goto    end_bat   	; converter already at min output

        incf    CCPR1L,w        ; IBAT > 0 A, decrease charge current
        andlw   0x0F            ; by decreasing converter VOUT
        movwf   CCPR1L

	goto	chk_charge

inc_charge			; increase charge a little bit
        movf    CCPR1L,w        ; check if converter output at max
        andlw   0x0F
        btfsc   STATUS,Z
        goto    chk_vfull   	; converter already at max output

        decf    CCPR1L,w        ; increase charge current
        andlw   0x0F            ; by increasing converter VOUT
        movwf   CCPR1L

chk_vfull
	call	halfsec
;	call    getvolt
	call 	readvi
	
	movf	VBATH,w		; check if VBAT near full voltage
	sublw	VFULL		; VFULL - VBATH		 

	btfss	STATUS,C	; skip if VFULL > VBATH
	goto	trickle		; VFULL < VBATH, trickle_charge

	movf	MP_PWM,w	; battery not yet fully charge
        andlw   0x0F            ; restore PWM setting
        movwf   CCPR1L
	goto 	end_bat

trickle
	clrf	TRICKLE
	incf	TRICKLE,f	; set TRICKLE status

find_trkle
	call	halfsec
;	call    getvolt
	call	readvi

	movf	VBATH,w		; check if VBAT near float voltage
	sublw	VFLOAT		; VFLOAT - VBATH		 

	btfss	STATUS,C	; skip if VFLOAT > VBATH
	goto	end_bat		; VFLOAT < VBATH, found trickle setting

        movf    CCPR1L,w        ; check if converter output at max
        andlw   0x0F
        btfsc   STATUS,Z
        goto    end_bat   	; converter already at max output

        decf    CCPR1L,w        ; increase charge current
        andlw   0x0F            ; by increasing converter VOUT
        movwf   CCPR1L

	goto	find_trkle

end_bat return


; *** charge 1A ***

charge_1a

	call	getvolt

        movf    IBATH,w         
        sublw   IBAT1A   	; IBAT1A - IBATH

	btfss	STATUS,C	; skip if 1A > IBAT
	goto	ibathi		; IBAT > 1A

ibatlo	
	movf	CCPR1L,w	; check if converter output at max
	andlw	0x0F	
	btfsc	STATUS,Z
	goto	end_charge_1a	; converter already at max output
	
	decf	CCPR1L,w	; IBAT < 1A, increase charge current
	andlw	0x0F		; by increasing converter VOUT
	movwf	CCPR1L

	call	getvolt
        movf    IBATH,w         
        sublw   IBAT1A   	; IBAT1A - IBATH

	btfss	STATUS,C	; skip if 1A > IBAT
	goto	end_charge_1a	; IBAT > 1A, done
	goto	ibatlo		; IBAT still < 1A, increase some more

ibathi
	incf	CCPR1L,w	; check if converter output at min
	andlw	0x0F	
	btfsc	STATUS,Z
	goto	end_charge_1a	; converter already at min output

	incf	CCPR1L,w	; IBAT > 1A, decrease charge current
	andlw	0x0F		; by decreasing converter VOUT
	movwf	CCPR1L

	call	getvolt
        movf    IBATH,w         
        sublw   IBAT1A   	; IBAT1A - IBATH

	btfss	STATUS,C	; skip if 1A > IBAT
	goto	ibathi		; IBAT still > 1A, decrease some more
	goto	end_charge_1a	; IBAT < 1A, done

end_charge_1a

	return


; *** find maximum power point setting ***
; *** find maximum charge  ***

charge_max

	call	getvolt

        movf    IBATH,w        
	movwf	EXTRA0		; store read current
	movf	IBATL,w
	movwf	EXTRA1

voltup	
	bcf     STATUS,C        ; clear carry flag
	rrf	ILOADH,w	; scale ILOADH to match IBATH
	sublw	IBATMAX  	; decrease IBATMAX by ILOADH (rescaled)
	subwf	EXTRA0,w	; IBATH - IBATMAX

	btfsc	STATUS,C	; check if IBATH < IBATMAX
	goto	end_charge_max	; IBATH >= IBATMAX

        movf    CCPR1L,w        ; check if highest voltage
        andlw   0x0F
        btfsc   STATUS,Z
        goto    voltdown   	; converter already at max voltage

        decf    CCPR1L,w        ; check if trying to increase
        andlw   0x0F            ; converter VOUT increases current
        movwf   CCPR1L

	call	halfsec
;        call    getvolt
	call	readvi
        movf    IBATH,w        
        subwf   EXTRA0,w	   	

	btfss	STATUS,Z	
	goto	nequal1		; EXTRA0 != IBATH
	movf	IBATL,w		; EXTRA0 = IBATH, 
	subwf	EXTRA1,w	; check EXTRA1 IBATL

	btfsc	STATUS,Z
	goto	voltup		; EXTRA = IBAT

nequal1
        btfss   STATUS,C        ; skip if EXTRA > IBAT
	goto	store_max1	; current increased, store this value
        goto    restore_pwm1   	; current decreased, go other way

store_max1        
        movf    IBATH,w        
	movwf	EXTRA0	
	movf	IBATL,w
	movwf	EXTRA1
	goto	voltup		; then try to increase some more	

restore_pwm1
        incf    CCPR1L,w        ; restore pwm
        andlw   0x0F            ; then try decreasing voltage
        movwf   CCPR1L		; to see if that increases current
	
voltdown
	bcf     STATUS,C        ; clear carry flag
        rrf     ILOADH,w        ; scale ILOADH to match IBATH
        sublw   IBATMAX         ; decrease IBATMAX by ILOADH (rescaled)
	subwf	EXTRA0,w	; IBATH - IBATMAX

	btfsc	STATUS,C	; check if IBATH < IBATMAX
	goto	end_charge_max	; IBATH >= IBATMAX

        incf    CCPR1L,w        ; check if converter output at min
        andlw   0x0F
        btfsc   STATUS,Z
        goto    end_charge_max  ; converter already at min voltage

        incf    CCPR1L,w        ; check if trying to decrease
        andlw   0x0F            ; converter VOUT increases current
        movwf   CCPR1L

	call	halfsec
;        call    getvolt
	call	readvi
        movf    IBATH,w        
        subwf   EXTRA0,w   

	btfss	STATUS,Z	
	goto	nequal2		; EXTRA0 != IBATH, compare them
	movf	IBATL,w		; EXTRA0 = IBATH, 
	subwf	EXTRA1,w	; check EXTRA1 IBATL

	btfsc	STATUS,Z	
	goto	voltdown	; EXTRA = IBAT

nequal2
        btfss   STATUS,C        ; skip if EXTRA0 >= IBAT
	goto	store_max2	; current increased, store this value
        goto    restore_pwm2   	; current decreased, restore pwm

store_max2        
        movf    IBATH,w        
	movwf	EXTRA0	
	movf	IBATL,w
	movwf	EXTRA1
	goto	voltdown	; then try to increase some more	

restore_pwm2
        decf    CCPR1L,w        ; restore pwm
        andlw   0x0F            ; then try decreasing voltage
        movwf   CCPR1L		; to see if that increases current

end_charge_max

	return


; *** check if current at absolute limit ***
; *** decrease output voltage to maintain current at the limit ***
 
charge_limit

;        call    getvolt
	call	readvi

	bcf     STATUS,C        ; clear carry flag
        rrf     ILOADH,w        ; scale ILOADH to match IBATH
        sublw   IBATMAX        ; decrease IBATMAX by ILOADH (rescaled)
        subwf   IBATH,w        ; IBATH - IBATMAX

        btfss   STATUS,C        ; check if IBATH < IBATMAX
        goto    end_charge_limit  ; IBATH < IBATMAX

        incf    CCPR1L,w        ; check if converter output at min
        andlw   0x0F
        btfsc   STATUS,Z
        goto    end_charge_limit  ; converter already at min voltage

        incf    CCPR1L,w        ; decrease converter VOUT
        andlw   0x0F            ; to decrease output current
        movwf   CCPR1L

        call    halfsec

	goto 	charge_limit

end_charge_limit
	
	return



; *** display received byte ***

disp_rx

	movf	RXDATA,w
	movwf	HEXNUM

	call	n2s_tx		; display received data

	movlw	0x0D		; carriage return
	movwf	TXDATA
	call	ser_tx

	clrf	RXDATA

	return


; *** interrupt handler ***
; *** for serial receive ***

interrupt_handler

	btfss	PIR1,RCIF
	goto	next_int1

	call	ser_rx

next_int1

	bcf	LDC

	retfie


; *** ds1820 temperature read ***
; *** result is in EXTRA4 EXTRA5 ***

get_ds1820

	call 	presence
	movf	CNT5,f	
	btfsc	STATUS,Z	; check if CNT5 is zero	
	goto	get_ds		; CNT5 is zero, no presence pulse found

	movlw	0xCC		; issue skip rom command
	movwf	EXTRA2
	call	write_slot
	movlw	0x44		; issue convert command
	movwf	EXTRA2
	call	write_slot

;wconv	call	read_slot	; read status of conversion	
;	incfsz	EXTRA2,f	; EXTRA2 was 0xFF, done converting
;	goto	wconv		; keep reading until conversion is done

	call	onesec		; delay to make sure conversion is done

        call    presence
        movf    CNT5,f
        btfsc   STATUS,Z        ; check if CNT5 is zero
        goto    get_ds          ; CNT5 is zero, no presence pulse found

        movlw   0xCC            ; issue skip rom command
        movwf   EXTRA2
        call    write_slot
        movlw   0xBE            ; issue read scratchpad command
        movwf   EXTRA2
        call    write_slot

	call	read_slot	; read byte 0
	movf	EXTRA2,w
	movwf	EXTRA4		; store byte 0 in EXTRA4

	call	read_slot	; read byte 1
	movf	EXTRA2,w	
	movwf	EXTRA5		; store byte 0 in EXTRA5

get_ds	return


; *** find presence pulse ***

presence

        movlw   TRISB           ; indirect address TRISB
        movwf   FSR             
        bcf   	INDF,1          ; config DSTEMP pin as output
	bcf	DSTEMP		; send initialize pulse
	
	call	us480		; delay for at least 480 us

	bsf	INDF,1		; FSR=TRISB, config DSTEMP pin as input

	call	us60		; delay for 60 us

	movlw	D'40'		; wait for presence pulse 
	movwf	CNT5		; for 240 us max

pwait	btfss	DSTEMP
	goto	waithi1		; ds1820 pulled DSTEMP pin low, wait until high
	decfsz	CNT5,f
	goto	pwait
	goto	end_presence

waithi1	btfsc	DSTEMP		; wait until ds1820 releases DSTEMP pin
	goto	end_presence
	decfsz	CNT5,f
	goto	waithi1

end_presence
	call	us480		; delay for 480 us to meet ds1820 timing

	return			; CNT5==0 indicates no presence pulse	


; *** write slots ***
; *** byte to write is in EXTRA2 ***

write_slot

 	movlw	0x08		; set up bit counter
	movwf	CNT4
	
wrtloop movlw   TRISB           ; indirect address TRISB
        movwf   FSR             
        bcf   	INDF,1          ; config DSTEMP pin as output
	bcf	DSTEMP		; pull DSTEMP pin low

	rrf	EXTRA2,f	; send bits, ls bit first
	btfsc	STATUS,C	; keep pulling DSTEMP low if bit is 0
	bsf	INDF,1		; bit is 1, config DSTEMP pin as input

	call	us60		; delay for 60 us
	call	us60		; delay for another 60 us

	bsf	INDF,1		; config DSTEMP pin as input

	decfsz	CNT4,f		; check if 8 bits have been sent
	goto	wrtloop		; transmit next bit

	return			; end write slot


; *** read slots ***
; *** byte read is in EXTRA2 ***

read_slot

        movlw   0x08            ; set up bit counter
        movwf   CNT4

rdloop 	movlw   TRISB           ; indirect address TRISB
        movwf   FSR
        bcf     INDF,1          ; config DSTEMP pin as output
        bcf     DSTEMP          ; pull DSTEMP pin low
	nop			; for 1 us
        bsf     INDF,1          ; config DSTEMP pin as input

	nop			; adjust wait time
	nop			; as needed
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop

	bsf	STATUS,C	; set carry flag
        btfss   DSTEMP          ; read DSTEMP pin
	bcf	STATUS,C	; DSTEMP=0, clear carry flag
	
        rrf     EXTRA2,f        ; rotate bits into EXTRA2, ls bit first

	movlw	D'10'		; wait for DSTEMP to go high
	movwf	CNT5		; wait for about 60 us

waithi2	btfsc	DSTEMP		; wait until ds1820 releases DSTEMP pin
	goto	chk_bits
	decfsz	CNT5,f
	goto	waithi2
	goto	end_read_slot	; read timeout, DSTEMP did not go high

chk_bits
        decfsz  CNT4,f          ; check if 8 bits have been read
        goto    rdloop          ; read next bit

end_read_slot
        return                  ; end read slot


; *** main code ***

main	bcf	IRP_bit		; select bank 0,1

	bcf	RP0_bit		; select bank 0
	bcf	RP1_bit		

	movlw	OSCCON
	movwf	FSR		; indirect address OSCCON
	movlw	B'01100000'	; configure internal osc = 4 MHz
	iorwf	INDF,f		; set OSCCON bits : IRCF = 110

	movlw	ANSEL
	movwf	FSR		; indirect address ANSEL
	movlw	B'01111111'	; configure port A[0:4]/AN[0:4], port B[7]/AN[6] as analog
	movwf	INDF		; write to ANSEL

	movlw   TRISA		; indirect address TRISA
        movwf   FSR             ; port A[0:4,6] as inputs, port A[5,7] as outputs
        movlw   B'01111111'	; configure port A[0:5] as inputs, needed for AN[0:4]
        movwf   INDF           	; configure port A[6] as input for 12V battery

	movlw   TRISB		; indirect address TRISB
        movwf   FSR             ; port B0 as output, port B[1:5,7] as inputs
        movlw   B'10111110'	; needed for serial tx/rx and AN6
        movwf   INDF           	; write to TRISB

	movlw   ADCON1
        movwf   FSR             ; indirect address ADCON1
        movlw   B'01000000'	; configure left justified, enable clock divide, internal Vref
        movwf   INDF           	; write to ADCON1

        movlw   AD0SET		; configure chan 0, ADON = 1, FOSC/8 to match 4 MHz clock
        movwf   ADCON0         	; write to ADCON0


        movlw   TRISB
        movwf   FSR             ; indirect address TRISB
        movlw   B'00100100'     ; configure port B[5,2] = 1, need to set these bits for serial
        iorwf   INDF,f          ; mask/set bits in TRISB

        movlw   TXSTA
        movwf   FSR             ; indirect address TXSTA
        movlw   B'00100100'     ; configure as async, 8-bit tx, tx enable. high baud rate
        movwf   INDF            ; write to TXSTA

        movlw   B'10010000'     ; configure enable serial, 8-bit rx, cont. rx,
        movwf   RCSTA           ; disable address detection, no frame and no overrun errors

        movlw   SPBRG
        movwf   FSR             ; indirect address SPBRG
        movlw   D'12'           ; configure for 19.2 kbps
        movwf   INDF            ; write to SPBRG

	movlw	B'00000100'	; set TMR2 prescale to 1 and turn on TMR2
	movwf	T2CON		; for PWM output

	movlw	PR2
	movwf	FSR		; indirect address PR2
	movlw	0x0F		; set PWM period (approximately 62.5 kHz)
	movwf	INDF		; write to PR2

	movlw	0x0F		; set MSB of duty cycle, start approx 100% duty -> low VBAT
	movwf	CCPR1L		; write to CCPR1L to change duty cycle (16 levels only = 4 bits)

	movlw	0x3F		; set LSB of duty cycle
	movwf	CCP1CON		; and PWM mode (on RB0 using _CCP1_RB0)

        movlw   PIE1
        movwf   FSR             ; indirect address PIE1
        movlw   B'00100000'     ; enable receive interrupt
        movwf   INDF            ; write to PIE1

	bsf	LED		; turn on LED
	bcf	LDC		; disconnect load

	clrf	EXTRA0		; clear locations EXTRA0 .. EXTRA5
        clrf    EXTRA1
        clrf    EXTRA2
        clrf    EXTRA3
        clrf    EXTRA4
        clrf    EXTRA5

	clrf	TRICKLE
	clrf	RXDATA

	bsf	INTCON, PEIE	; enable peripheral interrupt
	bsf	INTCON, GIE	; enable general interrupt

	call	onesec		; let things stabilize

	movlw	0x0D		; two carriage returns to delimit output data
	movwf	TXDATA
	call	ser_tx
	call	ser_tx

	movlw	IBATMAX		; display IBATMAX setting
        movwf   HEXNUM
        call    n2s_tx          
        movlw   0x0D 
        movwf   TXDATA
        call    ser_tx

	movlw	0x01		; do maximum tracking immediately
	movwf	MP_CNT

loopm
	movf	RXDATA,w
	btfss	STATUS,Z
	call	disp_rx		; non zero value in RXDATA, display value

	call 	get_ds1820	; get temperature, takes one second

	call	onesec
	call	readvi		; read voltage and current values

	movlw	0x0D
	movwf	TXDATA
	call	ser_tx

blink 	bsf     LED             ; LED on
        call    onesec          ; call delay subroutine
        bcf     LED             ; LED off
        call    onesec          ; call delay subroutine

	decfsz	MP_CNT,f	; check if need to find maximum power point
	goto	no_mppt

	movf	TRICKLE,w
	btfss	STATUS,Z
	goto 	no_mppt		; trickle charge set, do not do mppt

	call	charge_max	; get maximum power point setting
	movlw	D'10'		; get maximum power point every x iterations
	movwf	MP_CNT

no_mppt	nop

	call	chk_bat
	
	goto	loopm


; *** string tables ***

; *** W  must contain offset into the string/table ***
; *** make sure PCLATH is properly loaded with high byte of string/table location ***
; *** string should be terminated with 0x00 ***

	ORG	0x4A0

string	movwf	PCL
string1	retlw	'H'
	retlw	'e'
	retlw	'l'
	retlw	'l'
	retlw	'o'
	retlw	0x20
	retlw	0x00
string2	retlw	'r'
	retlw	'o'
	retlw	'o'
	retlw	't'
	retlw	0x0D
	retlw	0x00
string3	retlw	's'
	retlw	'o'
	retlw	'l'
	retlw	'a'
	retlw	'r'
	retlw	0x0D
	retlw	0x00
string4	retlw	'e'
	retlw	'c'
	retlw	'h'
	retlw	'o'
	retlw	0x20
	retlw	0x60	; char "`"
	retlw	'd'
	retlw	'a'
	retlw	't'
	retlw	'e'
	retlw	0x60	; char "`"
	retlw	0x20
	retlw	0x00
string5	retlw	0x20
	retlw	'>'
	retlw	'>'
	retlw	0x2F	; char "/"
	retlw	't'
	retlw	'm'
	retlw	'p'
	retlw	0x2F	; char "/"
	retlw	'v'
	retlw	'.'
	retlw	't'
	retlw	'x'
	retlw	't'
	retlw	0x0D
	retlw	0x00
okaystr	retlw	'o'
	retlw	'k'
	retlw	'a'
	retlw	'y'
	retlw	0x0D
	retlw	0x00
failstr	retlw	'f'
	retlw	'a'
	retlw	'i'
	retlw	'l'
	retlw	0x0D
	retlw	0x00

	
	ORG	0x700

; *** loader routines ***

; *** initialize the pic ***

initpic_ld 

; code here should be identical 
; to initialization sequence of main code

	bcf	IRP_bit		; select bank 0,1

	bcf	RP0_bit		; select bank 0
	bcf	RP1_bit		

	movlw	OSCCON
	movwf	FSR		; indirect address OSCCON
	movlw	B'01100000'	; configure internal osc = 4 MHz
	iorwf	INDF,f		; set OSCCON bits : IRCF = 110

	movlw	ANSEL
	movwf	FSR		; indirect address ANSEL
	movlw	B'01111111'	; configure port A[0:4]/AN[0:4], port B[7]/AN[6] as analog
	movwf	INDF		; write to ANSEL

	movlw   TRISA		; indirect address TRISA
        movwf   FSR             ; port A[0:4,6] as inputs, port A[5,7] as outputs
        movlw   B'01111111'	; configure port A[0:5] as inputs, needed for AN[0:4]
        movwf   INDF           	; configure port A[6] as input for 12V battery

	movlw   TRISB		; indirect address TRISB
        movwf   FSR             ; port B0 as output, port B[1:5,7] as inputs
        movlw   B'10111110'	; needed for serial tx/rx and AN6
        movwf   INDF           	; write to TRISB

	movlw   ADCON1
        movwf   FSR             ; indirect address ADCON1
        movlw   B'01000000'	; configure left justified, enable clock divide, internal Vref
        movwf   INDF           	; write to ADCON1

        movlw   AD0SET		; configure chan 0, ADON = 1, FOSC/8 to match 4 MHz clock
        movwf   ADCON0         	; write to ADCON0


        movlw   TRISB
        movwf   FSR             ; indirect address TRISB
        movlw   B'00100100'     ; configure port B[5,2] = 1, need to set these bits for serial
        iorwf   INDF,f          ; mask/set bits in TRISB

        movlw   TXSTA
        movwf   FSR             ; indirect address TXSTA
        movlw   B'00100100'     ; configure as async, 8-bit tx, tx enable. high baud rate
        movwf   INDF            ; write to TXSTA

        movlw   B'10010000'     ; configure enable serial, 8-bit rx, cont. rx,
        movwf   RCSTA           ; disable address detection, no frame and no overrun errors

        movlw   SPBRG
        movwf   FSR             ; indirect address SPBRG
        movlw   D'12'           ; configure for 19.2 kbps
        movwf   INDF            ; write to SPBRG

	movlw	B'00000100'	; set TMR2 prescale to 1 and turn on TMR2
	movwf	T2CON		; for PWM output

	movlw	PR2
	movwf	FSR		; indirect address PR2
	movlw	0x0F		; set PWM period (approximately 62.5 kHz)
	movwf	INDF		; write to PR2

	movlw	0x0F		; set MSB of duty cycle, start approx 100% duty -> low VBAT
	movwf	CCPR1L		; write to CCPR1L to change duty cycle (16 levels only = 4 bits)

	movlw	0x3F		; set LSB of duty cycle
	movwf	CCP1CON		; and PWM mode (on RB0 using _CCP1_RB0)


	bsf	LED		; turn on LED
	bcf	LDC		; disconnect load

	clrf	EXTRA0		; clear locations EXTRA0 .. EXTRA5
        clrf    EXTRA1
        clrf    EXTRA2
        clrf    EXTRA3
        clrf    EXTRA4
        clrf    EXTRA5

	call	onesec		; let things stabilize

	return


; *** delay for 100 msec at 4 MHz ***

ms100_ld 
	movlw   D'100'
        movwf   CNT2            ; load 2nd loop register

loop_a_ld
	movlw   D'250'
        movwf   CNT1            ; load 1st loop register

loop_b_ld 
	nop
        decfsz  CNT1,f          ; decrement 1st loop register
        goto    loop_b_ld       ; jump if register=0
        decfsz  CNT2,f          ; decrement 2nd loop register
        goto    loop_a_ld       ; jump if register=0
        return                  ; return to the main program


; *** blink PWMOUT LED ***

blink_ld
	bsf 	PWMOUT
	call	ms100_ld
	bcf	PWMOUT
	call	ms100_ld
	return

; *** toggle PWMOUT LED state ***

toggle_led_ld
	btfss	PWMOUT
	goto	set_led_ld
	bcf	PWMOUT
	goto	end_toggle_ld

set_led_ld
	bsf	PWMOUT

end_toggle_ld
	return


; *** serial port read ***

ser_rx  btfss   PIR1,RCIF       ; wait until RCREG is full
        goto    ser_rx

        movf    RCREG,w         ; get data from RCREG
        movwf   RXDATA          ; return data in RXDATA

	btfsc	RCSTA,OERR	; clear overrun bit if it is set
	bcf	RCSTA,CREN	; by clearing CREN bit	
	bsf	RCSTA,CREN

        return


; *** serial port transmit ***

ser_tx  btfss   PIR1,TXIF       ; wait until TXREG is empty
        goto    ser_tx

        movf    TXDATA,w        ; send data contained in TXDATA
        movwf   TXREG           ; put data to send in TXREG

        return


; *** erase 32 bytes of flash at [ADDRH:ADDRL] ***
; *** assumes 5 LSB of ADDRL is 0 *** 

erase_flash_ld
	BANKSEL EEADRH 		; Select Bank of EEADRH
	MOVF 	ADDRH, W 	;
	MOVWF 	EEADRH 		; MS Byte of Program Address to Erase
	MOVF 	ADDRL, W 	;
	MOVWF 	EEADR 		; LS Byte of Program Address to Erase
ERASE_ROW
	BANKSEL EECON1 		; Select Bank of EECON1
	BSF 	EECON1, EEPGD 	; Point to PROGRAM memory
	BSF 	EECON1, WREN 	; Enable Write to memory
	BSF 	EECON1, FREE 	; Enable Row Erase operation
;
;	BCF 	INTCON, GIE 	; Disable interrupts (if using)
	MOVLW 	0x55 		;
	MOVWF 	EECON2 		; Write 55h
	MOVLW 	0xAA 		;
	MOVWF 	EECON2 		; Write AAh
	BSF 	EECON1, WR 	; Start Erase (CPU stall)
	NOP 			; Any instructions here are ignored as processor
				; halts to begin Erase sequence
	NOP 			; processor will stop here and wait for Erase complete
				; after Erase processor continues with 3rd instruction
	BCF 	EECON1, FREE 	; Disable Row Erase operation
	BCF 	EECON1, WREN 	; Disable writes
;	BSF 	INTCON, GIE 	; Enable interrupts (if using)

	banksel	0x00
	return

; *** write 32 bytes of flash at [ADDRH:ADDRL] ***
; *** 2 LSB of ADDRL must be 0 ***

write_flash_ld
	BANKSEL EECON1 		;prepare for WRITE procedure
	BSF 	EECON1, EEPGD 	;point to program memory
	BSF 	EECON1, WREN 	;allow write cycles
	BCF 	EECON1, FREE 	;perform write only

	BANKSEL CNT1
	MOVLW 	0x04
	MOVWF 	CNT1	 	;prepare for 4 words to be written

	BANKSEL EEADRH 		;Start writing at 0x100
	MOVF 	ADDRH, W
	MOVWF 	EEADRH 		;load HIGH address
	MOVF 	ADDRL, W
	MOVWF 	EEADR 		;load LOW address
	BANKSEL ARRAY
	MOVLW 	ARRAY 		;initialize FSR to start of data
	MOVWF 	FSR

write_loop_ld
	BANKSEL EEDATA
	MOVF 	INDF, W 	;indirectly load EEDATA
	MOVWF 	EEDATA
	INCF 	FSR, F 		;increment data pointer
	MOVF 	INDF, W 	;indirectly load EEDATH
	MOVWF 	EEDATH
	INCF 	FSR, F 		;increment data pointer

	BANKSEL EECON1
	MOVLW 	0x55 		;required sequence
	MOVWF 	EECON2
	MOVLW 	0xAA
	MOVWF 	EECON2
	BSF 	EECON1, WR 	;set WR bit to begin write
	NOP 			;instructions here are ignored as processor
	NOP

	BANKSEL EEADR
	INCF 	EEADR, F 	;load next word address
	BANKSEL CNT1
	DECFSZ 	CNT1, F 	;have 4 words been written?
	GOTO 	write_loop_ld 	;NO, continue with writing

	BANKSEL EECON1
	BCF 	EECON1, WREN 	;YES, 4 words complete, disable writes
;	BSF 	INTCON,GIE 	;enable interrupts

	banksel	0x00
	return

; *** get 4 words into ARRAY ***

get_4words_ld
        movlw   ARRAY           ;initialize FSR to start of data
        movwf   FSR
        movlw   0x08
        movwf   CNT4

rx_loop_ld			; get 4 words from RS232RX
        call    ser_rx
        movf    CNT3,f          ; test if CNT3 is zero
        btfsc   STATUS,Z        ; wait until there is a valid char
        goto    rx_loop_ld      ; CNT3 is zero -> wait again

        movf    RXDATA,W
        movwf   INDF
;        movwf   TXDATA
;        call    ser_tx
        incf    FSR,f
        decfsz  CNT4,f
        goto    rx_loop_ld

        movlw   ARRAY           ;initialize FSR to start of data
        movwf   FSR
        movlw   0x08
        movwf   CNT4

tx_loop_ld			; send out receive words
        movf    INDF,W
        movwf   TXDATA
        call    ser_tx
        incf    FSR,f
        decfsz  CNT4,f
        goto    tx_loop_ld

        call    blink_ld	; blink LED when done

	return


check_flash_ld
	movlw	0x0D		; test serial
	movwf	TXDATA		; send two carriage returns
	call	ser_tx
	call	ser_tx

	clrf	TEMP
	clrf	CNT5		; not needed, can be remove to save space
	movlw	0x3F
	movwf	CNT4

wait_apos_ld			; see if "a+" is sent
        call    ser_rx
        movf    CNT3,f          ; test if CNT3 is zero
        btfss   STATUS,Z        ; wait until there is a valid char
	goto	check_apos_ld	; got a character
	decfsz	CNT5,f		; check if waiting too long
        goto    wait_apos_ld   	; wait some more
	call	toggle_led_ld
	decfsz	CNT4,f
	goto	wait_apos_ld	; wait some more
	goto	end_flash_ld	; waiting too long, return with CNT4 = 0

check_apos_ld
	movf	RXDATA,W	
	movwf	TXDATA
	call	ser_tx

	movf	TEMP,f
        btfss   STATUS,Z        ; if TEMP = 0, check for "a",
	goto	check_plus_ld	; otherwise check for "+"

	movf	RXDATA,W	
	sublw	"a"		; check if "a"
        btfsc   STATUS,Z        
	incf	TEMP,f		; found "a", look for next character
	goto	wait_apos_ld	; continue looking for "a"

check_plus_ld
	movf	RXDATA,W	
	sublw	"+"		; check if "+"
        btfss   STATUS,Z        ; if "+", continue 
	clrf	CNT4		; not character "+", return with CNT 4 = 0

end_flash_ld
	return			; check CNT4 on return 
		

loader 
	call	initpic_ld

;here	call	get_4words_ld
;	goto	here

	call	check_flash_ld	; wait and see if we are going to flash
        movf    CNT4,f          ; test if CNT4 is zero
        btfsc   STATUS,Z        ; check if  "a+", program flash if found
	goto	normal_entry	; if not "a+", goto to normal entry

	movlw	0x0D		 
	movwf	TXDATA		; send carriage return
	call	ser_tx		; to indicate ready to receive flash data

	clrf	ADDRH		; start writing only at 0x0020
	movlw	0x20
	movwf	ADDRL

write_prog_ld
	movf	ADDRL,W
	andlw	0x1F
	btfsc	STATUS,Z	; is ADDRL[4:0] = 0x00? 
	call	erase_flash_ld  ; -> erase 32 word block

;	movlw 	ARRAY 		;initialize FSR to start of data
;	movwf 	FSR
;	movlw	0x08
;	movwf	CNT1

;fill_array_ld
;	movf	CNT1,W
;	movwf	INDF	
;	incf	FSR,f
;	decfsz	CNT1,f
;	goto	fill_array_ld

	call	get_4words_ld	; get 4 words into ARRAY
	call	write_flash_ld	; write ARRAY to flash

	movlw	0x7C		; send "|" character to indicate
	movwf	TXDATA		; done writing 4 words
	call	ser_tx

	movlw	0x04
	addwf	ADDRL,f
	btfss	STATUS,Z	; is ADDRL 0x00? -> need to increment ADDRH
	goto	write_prog_ld

	incf	ADDRH,f
	movf	ADDRH,W
	sublw	0x06
	btfss	STATUS,Z	; is ADDRH 0x06? -> done writing
	goto	write_prog_ld

done_prog_ld
;	call	get_4words_ld
;	goto	done_prog_ld

	goto	normal_entry	; goto main code	


; initialize eeprom locations

;	ORG	0x2100
;	DE	0x00, 0x01, 0x02, 0x03


	END                       ; directive 'end of program'

