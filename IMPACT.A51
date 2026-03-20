 
;
; Copyright (c) 1999 R-Quest Technologies, LLC.  All rights reserved.
;
;

FALSE		equ	0
TRUE		equ	\FALSE


version1	equ	1	; v1.04
version2	equ	0
version3	equ	4

BETA_VERSION	equ	'P'

PC_IMPACT	equ	1	; Mediatechnics IMPACT product code number

	CHIP	8051


DBG_SEND_STAT	equ	FALSE
DBG_SEND_STAT2	equ	FALSE

;
;       revision table
;
; v104
;	(A) LWR	Changed to 2 part code. Main code starts at 4000h.
;		Program loader starts at 0.  Moved flash update to 
;		non diag code.
;	(B) LWR Changed Get command to got to the top agter getting disc
;	(C) LWR Cleaned up sources
;	(D) LWR Added Signature printer suport
;	(E) LWR Bounds check Diag next and prev command
;		Added Ediags commands for controller (non terminal mode) commanded diags
;	(F) LWR Added Accel out of drive tray
;	(G) LWR Added cyclic loader command
;	(H) LWR Changed cyclic mode reject to end at writers.
;		Changed auto calibrate to happen only if last command was accept or reject
;	(I) LWR Lengthened vacuume release time in insert command to 2 sec
;	(J) LWR Shortened vac release delay to 1 sec
;	(K) LWR Changes abandoned (Slow pick up for buis cards). 
;	(L) LWR Restored version J
;		Release vacuume on insert from normal release to top of travel
;	(M) LWR Source lost
;	(N) LWR Add delay to insert command if a disc is already on the gripper
;	(O) LWR increse WaitTowerDown timeout
;	(P) LWR Add delay at top of hopper when getting a disc

; v103
;	(A) LWR Added J command to help allign drive
;		Added gripper bounce delay to i command on pick retry
;
;	(B) ?????
;	(C) LWR used pwm to slow down the last column move.
;	(D) LWR fixed bug on L command if hopper empty
;	(E) LWR changed jz to cjne sequence in WaitTowerUp to fix time prob (wish I knew why!)
;	(F) LWR started looking for the disk present sensor to go away on drop disc
;	(G) LWR Reset PWM regs to power on values before programing flash
;
; v102
;	(A) LWR Added L comand - 1/st half of an insert command so we could open tray
;	    	getting a disc from the input hopper
;	        Added support for 2nd input and output hopper - shifts to second
;		hopper when 1st is empty
;

        external        ser_hi_byte,ser_lo_byte


; Instructions the assembler does not know about
;
AMOVI		equ	075h
AMOVR		equ	0f5h

; registers unique to the 89C51RC chip
;
AUXR1		equ	0a2h	; aux 1 register
CCAPM0		equ	0dah
CCAPM1		equ	0dbh
CCAPM2		equ	0dch
CCAPM3		equ	0ddh
ENABLE_PWM	equ	042h	; used with CCAPMX registers
DISABLE_PWM	equ	000h	; used with CCAPMX registers
CCAP0H		equ	0fah
CCAP1H		equ	0fbh
CCAP2H		equ	0fch
CCAP3H		equ	0fdh
CCON		equ	0d8h
CMOD		equ	0d9h

;
;
ROT_FAST_SPEED	equ	0ffh
ROT_SLOW_SPEED	equ	090h

PICK_FAST_SPEED	equ	0f8h
PICK_START_SPEED equ	000h	; start speed when accelerating up

VAC_ON_DELAY	equ	20	; wait for vaccume to pick up the disc

HOME_TIMEOUT	equ	5*10	; 10 set timeout to move gripper home

LOAD_PAUSE_AT_SPINDLE equ 40	; 40/100 sec delay to wait for possible 2nd
				; disc to fall when getting a disc from input


; Station positions - Drives is home pos
;
ROT_HOME         equ    0
MAX_ACCEPT_HOPPER equ	4	; max hopper pos to use as accept in cyclic

;loader states - used to determine if command is in sequence
;
STATE_CALIBRATED	equ	0		
STATE_ERROR		equ	1	

;error states
;
ERROR_NONE		equ     0
ERROR_SEQUENCE		equ	1
ERROR_EMPTY		equ	2 
ERROR_JAMMED		equ     3
ERROR_CALIBRATION	equ	4
ERROR_FULL		equ     5
ERROR_DISC_STUCK	equ	6
ERROR_PRINTER		equ	7

;
;  Misc Equates
;
MAX_ROT_POS	equ	5	; last rotational position
MIN_ROT_POS	equ	0	; first rotational position
AESC		equ	1bh	; ascii esc char


       PAGE
;**********************************************************************
;
;	Port Usage
;
;**********************************************************************

SIG_PORT		reg	p0	; port w/signature printer bits

; Port Outputs
;
SIG_STROBE_BIT		reg	p0.2

VAC_RELEASE_BIT		reg	P1.2
PICK_UP_BIT		reg	P1.3
PICK_DN_BIT		reg	P1.4
;ROT_ON_BIT1		reg     P1.5
;ROT_ON_BIT0		reg     P1.6
VAC_PUMP_ON_BIT		reg	P1.7

; Port Inputs
;
PICK_ARM_POS		reg	P0.0
PICK_ARM_HOME		reg	P0.1
SIG_DISC_BIT		reg	p0.3
SIG_BUSY_BIT		reg	p0.4
SIG_OUT_BIT		reg	p0.6
SIG_IN_BIT		reg	p0.5

DISC_PRES_FLAG		reg	P1.0
ROT_HOME_FLAG		reg     P1.1

MANIFOLD_FLAG		reg	P3.2
ROT_POS_FLAG		reg     P3.5



	PAGE
;**********************************************************************
;
;	internal RAM equates
;
;**********************************************************************
;
Timer           reg     10h
Timer2          reg     11h
TCount1		reg     12h
TCount2		reg     13h
TCount3		reg     14h
TCount4		reg     15h
SerBuffer       reg     16h
Scratch         reg     17h
StrOffset      reg     18h
State           reg     19h
Error           reg     1ah
RotPosition     reg     1bh
ACCount		reg	1ch		; current position in autocycle table
Timer5		reg	1dh
unused1		reg	1eh
unused2		reg	1fh

; bit addressable regs - starts at 20h
;
Flags           reg     20h
TxBusy          reg     Flags.0
DiagnosticsBit	reg	Flags.1
VacumeStatusBit	reg	Flags.2
LastMoveCCWBit	reg	Flags.3
LastMoveCWBit	reg	Flags.4
AuxHopperBit	reg	Flags.5			; true is using second hopper set
TowerTry0	reg	Flags.6
TempDiscPres	reg	Flags.7

HaveSigPrinterBit reg	21h.0		; true if printer is present
NeedToHomeLoader reg	21h.1		; true if loader needs command timeout home
CyclicHoppersBit reg	21h.2		; true if in cyclic hopper mode
AutoCalibrateOKBit reg	21h.3		; true if auto calibrate enabled

; start of non bit addressable ram = 30h
;
PickSpeed	reg	30h
HomeTimeout	reg	31h
CurrentAcceptHopper reg	32h

; check Stack location if any regs are added !!!!!!!!!!!!!!

	; stack location - stack grows UP
Stack			equ	36h

	; end of scratch ram = 7fh

	PAGE
;**********************************************************************
;
; jump table & interupt vectors. Vectored to here by boot rom
;
;**********************************************************************

CODE_START	equ	4000h
CODE_END	equ	8000h

CODE_ENTRY	equ	CODE_START+000h
CHECKSUM_SIZE	equ	CODE_START+004h
CHECKSUM	equ	CODE_START+006h
PRODUCT_CODE	equ	CODE_START+007h
X0_INT		equ	CODE_START+008h
T0_INT		equ	CODE_START+00ch
X1_INT		equ	CODE_START+010h
T1_INT		equ	CODE_START+014h
SER_INT		equ	CODE_START+018h
T2_INT		equ	CODE_START+01ch
PCA_INT		equ	CODE_START+020h
RQ_COPYRIGHT	equ	CODE_START+024h

	ORG	CODE_ENTRY
	jmp	StartOfCode

	ORG	CHECKSUM_SIZE
	dw	CODE_END - CODE_START

	ORG	CHECKSUM
	db	0ffh

	ORG	PRODUCT_CODE
	db	PC_IMPACT

	ORG	X0_INT
	jmp	X0IntEntry

	ORG	T0_INT
	jmp	T0IntEntry

	ORG	X1_INT
	jmp	X1IntEntry

	ORG	T1_INT
	jmp	T1IntEntry

	ORG	SER_INT
	jmp	SerIntEntry

	ORG	T2_INT
	jmp	T2IntEntry

	ORG	PCA_INT
	jmp	PCAIntEntry

	ORG	RQ_COPYRIGHT
	; R-Quest Copyright must be here for the boot loader to run us
	db	'Copyright (c) 2000 R-Quest Technologies, LLC. '
	db	'All rights reserved.',0



; null interrupt entry
;
X0IntEntry:
X1IntEntry:
T1IntEntry:
T2IntEntry:
PCAIntEntry:
	reti


	PAGE
;**********************************************************************
;
;       start of code
;
;**********************************************************************

StartOfCode:
	mov     ie,#0                   ;turn off the interupts
	mov     p0,#0ffh                ; clear the ports
	mov     p2,#0ffh
	mov     p1,#0ffh                ;turn off all outputs
	clr	VAC_PUMP_ON_BIT
	clr	VAC_RELEASE_BIT

;
;       Checksum code to make sure all ok
;
	mov     dptr,#CODE_START
	mov     r0,#0
CheckLoop:
	mov     a,#0
	movc    a,@a+dptr       ; get byte
	xch     a,r0            ; back to sum
	add     a,r0
	xch     a,r0
	inc     dptr            ; next byte
	mov     r1,dph          ; see if high done
	cjne    r1,#CODE_END/256,CheckLoop
	mov     r1,dpl          ; see if low done
	cjne    r1,#(CODE_END & 0ffh),CheckLoop
	xch     a,r0            ; get the sum
        jz	BootOk     	; good checksum

BootError:
	mov     sp,#Stack               ;init stack pointer
	call	DoFlashProgram


BootOk:        
	mov     sp,#Stack               ;init stack pointer

; default serial port settings
	mov     th1,#0fdh               ;timer constant 9600 baud
	mov     tl1,#0fdh
	mov     tmod,#22h               ;mode 2 for both timer 0 and 1
	mov     scon,#50h               ;8 bit no parity, rec int enabl
;        clr     dtr


; set up timer 0 interupt

	clr	NeedToHomeLoader	; don't want loader to home before ready
	mov     TCount1,#1             ; force an interupt quickly
	mov     TCount2,#1             ; force an interupt quickly
	mov     TCount3,#1             ; force an interupt quickly
	mov     tl0,#20h                ;interupt tick time
	mov     th0,#20h
        setb    ie.1                    ; enable t0 interrupt
        setb    ie.7                    ; enable 
        setb    pt0                     ; make t0 int hi priority
        setb    tr0                     ;start the timer


; get the serial port settings from the switches
;
        call   SetSerial              ;set up the serial port
	clr     ps                      ; set serial int to low priority
	setb    ie.4                    ; enable serial interupts
	mov     SerBuffer,#0              ; clear serial buffer

	call   LdrInit                ; do any ldr unique initialization


;**********************************************************************
;
;  Mountain autoloader compatible command set.
;       Impliments the following commands:
;               A - accept -> if have a disc put it in the accept else get one from a drive first
;               C - calibrate the carriage
;		D - Terminal diags
;		E - non terminal diags
;		F - Flash firmware
;		G - get -> get a disc from a drive and hold it
;               I - insert -> if have a disc put it in a drive else get one first
;		J - Insert w/o releasing disc
;		L - load -> get a disc from input hopper and hold it
;               R - reject 
;		N - Close sig printer
;		O - Open sig printer
;		P - sig Printer status
;               S - status
;		V - Version command
;
; hopper commands
;		M - Reset hoppers
;		H - Extended hoppers (cyclic)
;		K - Standard hoppers (2 in / 2 out)


;       respond with a '?' to any other commands
;
NextCommand:
	call   SerIn                   ;get a serial character
	cjne    a,#0dh,NotCR		; ignore CR and LF
	jmp    NextCommand
NotCR: 
	cjne    a,#0ah,NotLF
	jmp    NextCommand
NotLF:

;
; Extended command for diagnostics.
;       a 'D' character enters disgnostics and an ESC exits
;  
	cjne    a,#'D',NotD            ; diagnostic mode
	call	DiagCommand
	jmp    s_next_command          
NotD:  

;
; non terminal mode extended command for diagnostics.
;       a 'E' character enters disgnostics and an ESC exits
;  
	cjne    a,#'E',NotE            ; Ediagnostic mode
	call	EDiagCommand
	jmp    s_next_command          
NotE:  

        cjne    a,#'S',NotS		; status
	call   StatusCommand
	sjmp    NextCommand
NotS:
	cjne    a,#'P',NotP		; Printer status
	call	SigPrinterStatus
	jmp	NextCommand		; no status to return
NotP:
        cjne    a,#'V',NotV		; version command
	mov     dptr,#vversion		; give some help
	call   	PutString
	jmp    NextCommand
NotV:

	clr	AutoCalibrateOKBit


        cjne    a,#'F',NotF            	; program flash
	call	DoFlashProgram
	jmp    	NextCommand
NotF:
	cjne    a,#'C',NotC		; calibrate
	call   CalibrateCommand
	jmp    CheckCommandStatus
NotC:

	jmp	ArndErrors
CmdErrorSequence2:
	call	ErrorSequence                  ; say sequence error
	jmp	CommandError
ArndErrors:


	cjne    a,#'A',NotA		; accept disc   
	mov     a,State
	cjne    a,#STATE_CALIBRATED,CmdErrorSequence2 ;if not calibrated then bad command
	call   AcceptCommand
	setb	AutoCalibrateOKBit
	jmp    CheckCommandStatus
NotA:

	cjne    a,#'R',NotR		; reject disc 
	mov     a,State
	cjne    a,#STATE_CALIBRATED,CmdErrorSequence2 ;if not calibrated then bad command
	call   RejectCommand
	setb	AutoCalibrateOKBit
	jmp    CheckCommandStatus
NotR:  

	cjne    a,#'G',NotG		; get disc
	mov     a,State
	cjne    a,#STATE_CALIBRATED,CmdErrorSequence2 ;if not calibrated then bad command
	call   GetCommand
	jmp    CheckCommandStatus
NotG:  

	cjne    a,#'L',NotL            ; pick disc from input hopper & hold
	mov     a,State
	cjne    a,#STATE_CALIBRATED,CmdErrorSequence2 ;if not calibrated then bad command
	call   LoadCommand
	jmp    CheckCommandStatus
NotL:  

	cjne    a,#'I',NotI            ; insert disc   
	mov     a,State
	cjne    a,#STATE_CALIBRATED,CmdErrorSequence ;if not calibrated then bad command
	call	InsertCommand
	jmp	CheckCommandStatus
NotI:

	cjne    a,#'J',NotJ            ; allign disc   
	mov     a,State
	cjne    a,#STATE_CALIBRATED,CmdErrorSequence ;if not calibrated then bad command
	call   AllignCommand
	jmp	NextCommand		; no status to return
NotJ:

	cjne    a,#'N',NotN            ; Close printer
	mov     a,State
	cjne    a,#STATE_CALIBRATED,CmdErrorSequence ;if not calibrated then bad command
	call	SigCloseTray
	jmp	CheckCommandStatus
NotN:

	cjne    a,#'O',NotO	       ; Open printer
	mov     a,State
	cjne    a,#STATE_CALIBRATED,CmdErrorSequence ;if not calibrated then bad command
	call	SigOpenTray
	jmp	CheckCommandStatus
NotO:


; hopper commands
;
        cjne    a,#'M',NotM           	; reset hoppers
	call	ResetHopperCommand
	clr	c
	jmp	CheckCommandStatus	
	jmp    	NextCommand
NotM:

        cjne    a,#'H',NotH            	; Extended hoppers (cyclic)
	call	ExtendedHoppersCommand
	clr	c
	jmp	CheckCommandStatus	
	jmp    	NextCommand
NotH:

        cjne    a,#'K',NotK            	; Standard hoppers (2 in/ 2 out)
	call	StandardHoppersCommand
	clr	c
	jmp	CheckCommandStatus	
	jmp    	NextCommand
NotK:


; unknown command 
	mov     a,#'?'
	call   SerOut                    ;say 'what?'
;back for more
s_next_command:
	jmp    NextCommand


CmdErrorSequence:
	call	ErrorSequence                  ; say sequence error
	jmp	CommandError


;
; end of mountain command decode
;
;**********************************************************************

; vector to the error or no error
;
CheckCommandStatus:
	jnc     OK_Command
	; fall into CommandError
; command with status return errored out
;
CommandError:
	mov	a,#'E'                  ;say error
	call	SerOut
	jmp	NextCommand

; command with status return is all ok
;
OK_Command:
	mov     a,#'X'                  ;say error
	call	SerOut
	jmp	NextCommand


; error entry points
;

;disk_stuck:
;	mov     Error,#ERROR_DISC_STUCK
;	jmp    SayError
;err_full:
;	mov     Error,#ERROR_FULL
;	jmp    SayError			; flag bin error
;er_calib:
;	mov     Error,#ERROR_CALIBRATION ; flag sequence error
;	jmp    SayError
ErrorSequence:
	mov     Error,#ERROR_SEQUENCE   ;flag sequence error
	jmp    SayError
;er_jamed:
;	mov     Error,#ERROR_JAMMED	;flag jamed error
;	jmp    SayError
;er_hopper_empty:
;	mov     Error,#ERROR_EMPTY	;flag empty hopper
;	jmp    SayError

ErrorPrinter:
	mov	Error,#ERROR_PRINTER	; printer failed to move
	jmp	SayError

; say error
;
SayError:
	mov     State,#STATE_ERROR      ;flag error state
	setb    c                       ;set error flag
	ret


	PAGE
;**********************************************************************
;
; send status command - For Mountain command set
;
;**********************************************************************
StatusCommand:
	mov     a,State
	cjne    a,#STATE_CALIBRATED,StatNotCal      ;not in the calibrated state
	mov     a,#'0'                  ;no high order error bits
	sjmp    SendExit

StatNotCal:

; in the error state since that is only state left
s_error:
	mov     a,Error
	cjne    a,#ERROR_NONE,s_nok  ;not ok status
	mov     a,#'0'                  ;no high order error bits
	sjmp    SendExit

s_nok:
	cjne    a,#ERROR_SEQUENCE,s_nseq        ;not sequence error
	mov     a,#'4'                  ;set command error bit
	sjmp    SendExit

s_nseq:
	cjne    a,#ERROR_EMPTY,s_nhopper      ;not hopper empty
	mov     a,#'2'                  ;set hopper empty bit
	sjmp    SendExit             ; ok.

s_nhopper:
	cjne    a,#ERROR_JAMMED,s_njam        ;not a jam
	mov     a,#'1'                  ;set computed jam bit
	sjmp    SendExit

s_njam:
	cjne    a,#ERROR_CALIBRATION,s_ncal ;not calibration
	mov     a,#'8'                  ;set computed calibration error
	sjmp    SendExit

s_ncal:
	cjne    a,#ERROR_FULL,s_nfull      ; see if full error
	mov     a,#'A'                  ; set full error
	sjmp    SendExit

s_nfull:
	cjne    a,#ERROR_DISC_STUCK,s_nstuck
	mov     a,#'9'                  ; send disk stuck
	sjmp    SendExit
s_nstuck:

	cjne	a,#ERROR_PRINTER,s_nprinter
	mov	a,#'C'			; send printer error
	jmp	SendExit
s_nprinter:

; dont know what the error is so set everything !
	mov     a,#'F'                  ;set all error bits since we dont know
	sjmp    SendExit                     ;send low nibble
;       
; send the status bytes in a & b adding disk present & go to s_exit
SendExit:
	call   SerOut                    ; send first byte of status
	; get rest of status
	mov	a,#0			; start w/o
;	mov	c,ROT_POS_FLAG
;	mov	acc.3,c
;	mov	c,DISC_PRES_FLAG
;	mov	acc.0,c
	
; send the status byte created above and return for next command
s_exit:
	call   Acc2Ascii		; change from hex to ascii
	call   SerOut                   ; send it
	clr    c			; flag no errors
	ret

        page    
;********************************************************************
;
;       execute actual commands
;
;********************************************************************
;
LdrInit:
	mov	HomeTimeout,#0		; disable auto cal timeout
	mov	State,#STATE_ERROR
	mov	Error,#ERROR_CALIBRATION
	mov	ACCount,#0		; current autocycle position
	clr	DiagnosticsBit
	call	VacumeOff
	call	VacumeRelOff
	clr	AuxHopperBit		; start with primary hopper set
	clr	CyclicHoppersBit
	clr	AutoCalibrateOKBit
	clr	NeedToHomeLoader

	call	TowerOff

	mov	PickSpeed,#PICK_FAST_SPEED	; default pick speed
	db	AMOVI,CMOD,02h		; select int osc/4
;	mov	CMOD,#02h
    	db	AMOVI,CCAP0H,PICK_FAST_SPEED ; up speed
;	mov	CCAP0H,#PICK_FAST_SPEED
    	db	AMOVI,CCAP1H,PICK_FAST_SPEED ; down speed
;	mov	CCAP1H,#PICK_FAST_SPEED
	db	AMOVI,CCAP2H,ROT_FAST_SPEED ; ccw speed
;	mov	CCAP2H,#ROT_FAST_SPEED
	db	AMOVI,CCAP3H,ROT_FAST_SPEED ; cw speed
;	mov	CCAP3H,#ROT_FAST_SPEED
	db	AMOVI,CCON,040h
;	mov	CCON,#040h

	clr	HaveSigPrinterBit	; assume no printer
	call	SigCheckForPrinter	; ret carry if have printer
	jnc	LdrInitDonewPrinter
	setb	HaveSigPrinterBit
LdrInitDonewPrinter:

        ret

; version string for the 'V'command
;
;vversion	db	34,'Mediatechnics'
vversion	db	34,'TurnKey'
		db	34,' '
;		db	34,'IMPACT'
		db	34,'TCA-9006'	; use this R-Quest loader to be compatible with
		db	34,' '
  		db	34,'V',version1+'0','.',version2+'0',version3+'0'
		db	BETA_VERSION
		db	34+80h

;******************************************************************************
; 
; diagnostics command
;
;	D - Enters Terminal Mode Diags
;	E - Enters Controller mode Diags - No prompts and short responses
;


; EDiag comands:
;		S = return 2 hex bytes of sensor status
;			Bit 6 & 7 = 0
;			Bit 5 = Column Home
;			Bit 4 = Column Pos
;			Bit 3 = Pick Home
;			Bit 2 = Pick Pos
;			Bit 1 = Disc
;			Bit 0 = Manifold
;
;		P = Move to previous column pos
;		N = Move to next column pos
;		V = Toggle vacuume on/off
;		R = Release Vacuume
;		U = Move pick arm up
;		D = Move pick arm down
;		M = move arm to spindle (middle pos)
;		ESC = exit and calibrate the loader


EDiagCommand:
	setb	DiagnosticsBit	; let everyone know we are in diags mode
	call	WaitTowerUp
	call	RotateToHome
	call	EDiagAck	; ack diag entrance

	; wait for a command to execute
EDiagLoop:
	call	SerIn		; get a command

	clr	AutoCalibrateOKBit
	cjne	a,#'R',EDNotR	; release vacume
	call	VacumeRelease
	call	EDiagAck
	jmp	EDiagLoop
EDNotR:

	cjne	a,#'M',EDNotM	; move pick to spindle
	call	WaitTowerUp
	call	PickToAccept
	call	AcceptToSpindle
	call	EDiagAck
	jmp	EDiagLoop
EDNotM:
		
	cjne	a,#'S',EDNotS	; show sensors
	call	EDiagShowSensors
	jmp	EDiagLoop
EDNotS:

	cjne	a,#'D',EDNotD	; Pick arm down
	call	WaitTowerDown
	call	EDiagAck
	jmp	EDiagLoop
EDNotD:

	cjne	a,#'U',EDNotU	; Pick arm up
	call	WaitTowerUp
	call	EDiagAck
	jmp	EDiagLoop
EDNotU:

	cjne	a,#'V',EDNotV	; toggle vacume
	jnb	VacumeStatusBit,EDVTurnOn
	call	VacumeOff
	call	EDiagAck
	jmp	EDiagLoop
EDVTurnOn:
	call	VacumeOn
	call	EDiagAck
	jmp	EDiagLoop
EDNotV:

	cjne	a,#'N',EDNotN		; rotate to next
	mov	a,RotPosition
	cjne	a,#MAX_ROT_POS,EDNextOK
	jmp	EDNextDone
EDNextOK:
	inc	a
	call	RotateToA
EDNextDone:	
	call	EDiagAck
	jmp	EDiagLoop
EDNotN:

	cjne	a,#'P',EDNotP		; rotate to prev
	mov	a,RotPosition
	cjne	a,#MIN_ROT_POS,EDPrevOK
	jmp	EDPrevDone
EDPrevOK:
	dec	a
	call	RotateToA
EDPrevDone:	
	call	EDiagAck
	jmp	EDiagLoop
EDNotP:

	cjne	a,#AESC,EDNotESC	; see if done
	jmp	EDiagExit	
EDNotESC:

	; unknown command
	mov	a,#'?'
	call	SerOut
	jmp	EDiagLoop

EDiagExit:
	clr	DiagnosticsBit
	call	CalibrateCommand	; put the loader back into a known state
	ret

EDiagAck:
	mov	a,#'X'
	call	SerOut
	ret

; send 2 hex chars of loader sensors
;
;			Bit 6 & 7 = 0
;			Bit 5 = Column Home
;			Bit 4 = Column Pos
;			Bit 3 = Pick Home
;			Bit 2 = Pick Pos
;			Bit 1 = Disc
;			Bit 0 = Manifold
EDiagShowSensors:
	mov	a,#0
	mov	c,MANIFOLD_FLAG
	mov	a.0,c
	mov	c,DISC_PRES_FLAG
	mov	a.1,c
	mov	c,PICK_ARM_POS
	mov	a.2,c
	mov	c,PICK_ARM_HOME
	mov	a.3,c
	mov	c,ROT_POS_FLAG
	mov	a.4,c
	mov	c,ROT_HOME_FLAG
	mov	a.5,c
	call	SendAscii
	ret

;
; Terminal diags code
;
DiagSignon 	db	0dh,0ah,0ah
		db	'Mediatechnics IMPACT v'
  		db	version1+'0','.',version2+'0',version3+'0'
		db	BETA_VERSION
		db	' Diagnostics',0dh,0ah,0ah
		db	'H - Column to Home',0dh,0ah
		db	'N - Column to Next',0dh,0ah
		db	'P - Column to Prev',0dh,0ah
		db	'U - Pick Arm Up',0dh,0ah
		db	'D - Pick Arm Down',0dh,0ah
		db	'M - Pick Arm to Spindle',0dh,0ah
		db	'V - Toggle Vacume',0dh,0ah
		db	'R - Release Vacume',0dh,0ah
		db	'S - Display Sensors',0dh,0ah+80h
DiagSignon2:
		db	'T - Excersise Column',0dh,0ah
;		db	'A - Autocycle Loader',0dh,0ah
		db	'ESC - Exit',0dh,0ah+80h

DiagPrompt	db	0dh,0ah,'H,N,P,U,D,M,V,R,S,T or ESC','>'+80h

DiagMRotate2Home	db	'Column to Home..','.'+80h
DiagMRotate2Next	db	'Column to Next..','.'+80h
DiagMRotate2Prev	db	'Column to Prev..','.'+80h
DiagMPusherUp	db	'Pick Arm Up..','.'+80h
DiagMPusherDown	db	'Pick Arm Down..','.'+80h
DiagMTowerToMiddle db	'Pick Arm to Spindle..','.'+80h
DiagMVacOff	db	'Vacume OFF..','.'+80h
DiagMVacOn	db	'Vacume ON..','.'+80h
DiagMRelease	db	'Vacume Release..','.'+80h
DiagMEXTurntable db	'Excersise Column... Press ESC to stop','.'+80h
DiagMAutocycle	db	'Autocycle..','.'+80h
DiagMExit	db	'Exiting Diagnostics',0dh,0ah+80h
DiagMOK		db	'O','K'+80h
DiagMFailed	db	'Faile','d'+80h
DiagMSensors	db	0dh,0ah,'Displaying Sensors - Press ESC to stop',0dh,0ah
		db	'|CHom|CPos|PHom|PPos|Disc|Man |',0dh,0ah
		db	'|----|----|----|----|----|----|',0dh,0ah+80h
DiagMHi		db	'| HI',' '+80h
DiagMLow	db	'|low',' '+80h
DiagMEnd	db	'|',0dh+80h

DiagCommand:
	setb	DiagnosticsBit
	mov	dptr,#DiagSignon	; paint first half
	call	PutString 	
	mov	dptr,#DiagSignon2	; paint second half
	call	PutString 	
DiagLoop:
	mov	dptr,#DiagPrompt
	call	PutString 	
	call	SerIn

	cjne	a,#'H',DiagNotH
	mov	dptr,#DiagMRotate2Home
	call	PutString
	call	RotateToHome
	call	DiagSayStatus
	jmp	DiagLoop
DiagNotH:
	cjne	a,#'N',DiagNotN
	mov	a,RotPosition
	cjne	a,#MAX_ROT_POS,DNextOK
	jmp	DiagLoop
DNextOK:
	mov	dptr,#DiagMRotate2Next
	call	PutString
	mov	a,RotPosition
	inc	a
	call	RotateToA
	call	DiagSayStatus
	jmp	DiagLoop
DiagNotN:
	cjne	a,#'P',DiagNotP
	mov	a,RotPosition
	cjne	a,#MIN_ROT_POS,DPrevOK
	jmp	DiagLoop
DPrevOK:
	mov	dptr,#DiagMRotate2Prev
	call	PutString
	mov	a,RotPosition
	dec	a
	call	RotateToA
	call	DiagSayStatus
	jmp	DiagLoop
DiagNotP:
	cjne	a,#'M',DiagNotM
	mov	dptr,#DiagMTowerToMiddle
	call	PutString
	call	WaitTowerUp
	call	PickToAccept
	call	AcceptToSpindle
	call	DiagSayStatus
	jmp	DiagLoop
DiagNotM:
	cjne	a,#'U',DiagNotPshrUp
	mov	dptr,#DiagMPusherUp
	call	PutString
	call	WaitTowerUp
	call	DiagSayStatus
	jmp	DiagLoop
DiagNotPshrUp:
	cjne	a,#'D',DiagNotPshrDn
	mov	dptr,#DiagMPusherDown
	call	PutString
	call	WaitTowerDown
	call	DiagSayStatus
	jmp	DiagLoop
DiagNotPshrDn:
	cjne	a,#'R',DiagNotR
	mov	dptr,#DiagMRelease
	call	PutString
	call	VacumeRelease
	jmp	DiagLoop
DiagNotR:
	cjne	a,#'V',DiagNotV
	jnb	VacumeStatusBit,VTurnOn
	mov	dptr,#DiagMVacOff
	call	PutString
	call	VacumeOff
	jmp	DiagLoop
VTurnOn:
	mov	dptr,#DiagMVacOn
	call	PutString
	call	VacumeOn
	jmp	DiagLoop
DiagNotV:

	cjne	a,#'T',DiagNotT
	mov	dptr,#DiagMEXTurntable
	call	PutString
	call	ExcersiseTurntable
	jmp	DiagLoop
DiagNotT:

	cjne	a,#'A',DiagNotA
	mov	dptr,#DiagMAutocycle
	call	PutString
	call	DiagAutocycle
	jmp	DiagLoop
DiagNotA:

	cjne	a,#'?',DiagNotHelp
	mov	dptr,#DiagSignon	; paint first half
	call	PutString 	
	mov	dptr,#DiagSignon2	; paint second half
	call	PutString 	
	jmp	DiagLoop
DiagNotHelp:

	cjne	a,#01bh,DiagNotESC
	jmp	DiagExit
DiagNotESC:

	cjne	a,#'S',DiagNotSensors
	mov	dptr,#DiagMSensors
	call	PutString
DiagShowSenLoop:
	mov	dptr,#DiagMLow
	jnb	ROT_HOME_FLAG,DRotateHLow
	mov	dptr,#DiagMHi
DRotateHLow: call	PutString

	mov	dptr,#DiagMLow
	jnb	ROT_POS_FLAG,DRotatePLow
	mov	dptr,#DiagMHi
DRotatePLow: call	PutString

	mov	dptr,#DiagMLow
	jnb	PICK_ARM_HOME,DRotatePSLow
	mov	dptr,#DiagMHi
DRotatePSLow: call	PutString

	mov	dptr,#DiagMLow
	jnb	PICK_ARM_POS,DRotateDLow
	mov	dptr,#DiagMHi
DRotateDLow: call	PutString

	mov	dptr,#DiagMLow
	jnb	DISC_PRES_FLAG,DRotateHPLow
	mov	dptr,#DiagMHi
DRotateHPLow: call	PutString

	mov	dptr,#DiagMLow
	jnb	MANIFOLD_FLAG,DRotateCLow
	mov	dptr,#DiagMHi
DRotateCLow: call	PutString

	mov	dptr,#DiagMEnd
	call	PutString
	; see if done
	mov	a,SerBuffer
	cjne	a,#01bh,DiagSenCont
	call	SerIn		; get the character
	jmp	DiagLoop
DiagSenCont:
	mov	Timer,#5
	call	Delay
	jmp	DiagShowSenLoop		
DiagNotSensors:


	; unknown command
	mov	a,#'?'
	call	SerOut
	jmp	DiagLoop

DiagSayStatus:
	mov	dptr,#DiagMOK
	jnc	DiagSayGotStatus
	mov	dptr,#DiagMFailed
DiagSayGotStatus:
	call	PutString
	ret

DiagExit:	
	clr	DiagnosticsBit
	mov	dptr,#DiagMExit
	call	PutString
	call	CalibrateCommand	; get the loader to a known state
        ret


ExcersiseTurntable:
	call	RotateToHome
EXRotateLoop:
	mov	a,#0
	call	RotateToA
	jc	EXRotateDone

	call	WaitTowerDown
	jc	EXRotateDone

	mov	a,#5
       	call	RotateToA
	jc	EXRotateDone

	call	WaitTowerUp
	jc	EXRotateDone

       	; see if done
	mov	a,SerBuffer
	cjne	a,#01bh,EXRotateLoop
	call	SerIn	

EXRotateDone:
	ret	




DiagAutocycle:
	call	CalibrateCommand
DiagACLoop:
	call	InsertCommand
	jc	DiagACError
	call	AcceptCommand
	jc	DiagACError
	mov	a,SerBuffer
	cjne	a,#01bh,DiagACLoop
	call	SerIn	
DiagACError:
	ret		




;
;	Actual loader cycle commands
;


; calibrate command
;
CalibrateCommand:
	clr	DiagnosticsBit	
	call	CalibSigCloseTray
	jnc	CalibPrinterClosed
	call	CalibSigCloseTray	; if printer was partly open it will finish opening and error
	jc	CmdCalibError		; so we need to try to close it again
CalibPrinterClosed:
	call	WaitTowerUp
	jc	CmdCalibError	
	clr	AuxHopperBit	; calibrate restarts hopper sequence

	call	RotateToHome
	jc	CmdCalibError	
	mov	State,#STATE_CALIBRATED
c_cmd_ok:
	mov	Error,#ERROR_NONE
	mov	State,#STATE_CALIBRATED
	clr	c		; return no carry error

	jb	DISC_PRES_FLAG,CNoDisc
	call	RejectCommand	; reject the disc on the pick arm
CNoDisc:
	call	VacumeOff
	call	VacumeRelOff
c_cmd_exit:
       	ret

CmdCalibError:
	mov	Error,#ERROR_CALIBRATION
	mov	State,#STATE_ERROR
	setb	c
	ret


cmd_jamed2:
	jmp	cmd_jamed



; get a disc from current rot position
;
GetDisc:
	call	VacumeOn
	call	WaitTowerDown
	jc	GetDiscError
	mov	Timer,#VAC_ON_DELAY
	call	Delay
	call	AcceptWaitTowerUp
GetDiscError:
	ret




; Accept command 
;
AcceptCommand:
	; see if already have a disc (debounced)
	mov	c,DISC_PRES_FLAG
	mov	TempDiscPres,c

	mov	Timer,#10
	call	Delay

	jb	DISC_PRES_FLAG,AIsHi
	
	; DSIC_PRES_FLAG must be low
	jnb	TempDiscPres,APossibleGotDisc
	jmp	AcceptCommand	; try again	
AIsHi:
	jb	TempDiscPres,ANoDisc
	jmp	AcceptCommand	; try again	

APossibleGotDisc:
	mov	Timer,#20
	call	Delay
	jnb	DISC_PRES_FLAG,AGotDisc
	jmp	AcceptCommand



ANoDisc:
	call	WaitTowerUp
	jc	cmd_jamed2
	call	RotateToDrives
	jc	cmd_jamed2

	call	GetDisc
	jc	cmd_jamed2
	jnb	DISC_PRES_FLAG,AGotDisc

	call	TowerDelay
	call	GetDisc
	jc	cmd_jamed2
	jnb	DISC_PRES_FLAG,AGotDisc

	; try again for a disc
	call	TowerDelay
	call	GetDisc
	jc	cmd_jamed2
	jb	DISC_PRES_FLAG,cmd_seq_error

AGotDisc:
	call	RotateToAccept
	jc	cmd_jamed2	; return the carry error
	call	PickToAccept	; down over spindle
	jc	cmd_jamed	; return the carry error
	call	VacumeOff
	call	VacumeRelease
	call	AcceptToSpindle
	jc	cmd_jamed	; return the carry error

	; make sure disc sensor is correct
	jb	DISC_PRES_FLAG,ADroppedDisc
	call	TowerDelay
	jb	DISC_PRES_FLAG,ADroppedDisc
	call	TowerDelay
	jb	DISC_PRES_FLAG,ADroppedDisc
	jmp	cmd_jamed4
ADroppedDisc:
aa_cmd_ok:
	clr	c		; return no error
	ret


; Get command 
;
GetCommand:
	call	WaitTowerUp
	jc	cmd_jamed
	call	RotateToDrives
	jc	cmd_jamed
	call	GetDisc
	jc	cmd_jamed
	jnb	DISC_PRES_FLAG,GGotDisc

	; 2nd try
	call	TowerDelay
	call	GetDisc
	jc	cmd_jamed
	jnb	DISC_PRES_FLAG,GGotDisc

	; try again for a disc
	call	TowerDelay
	call	GetDisc
	jc	cmd_jamed
	jb	DISC_PRES_FLAG,cmd_seq_error

GGotDisc:
	jc	cmd_jamed
	call	WaitTowerUp
	clr	c		; return no error
	ret

cmd_seq_error:
	mov	Error,#ERROR_SEQUENCE
	jmp	cmd_err_exit

cmd_jamed:
	mov	Error,#ERROR_JAMMED
cmd_err_exit:
	mov	State,#STATE_ERROR
	call	TowerOff
	call	RotateOff
	call	VacumeOff
	setb	c
	ret

	
; Reject command 
RejectCommand:
	; see if already have a disc
	jnb	DISC_PRES_FLAG,RGotDisc

	call	WaitTowerUp
	jc	cmd_jamed
	call	RotateToDrives
	jc	cmd_jamed
	call	GetDisc
	jc	cmd_jamed
	jnb	DISC_PRES_FLAG,RGotDisc

	; try 2
	call	TowerDelay
	call	GetDisc
	jc	cmd_jamed
	jnb	DISC_PRES_FLAG,RGotDisc

	; try again for a disc
	call	TowerDelay
	call	GetDisc
	jc	cmd_jamed
	jb	DISC_PRES_FLAG,cmd_seq_error

RGotDisc:
	call	RotateToReject
	jc	cmd_jamed	; return the carry error
	jnb	CyclicHoppersBit,RGotNotCyclic
	call	WaitTowerDown
	jc	cmd_jamed	; return the carry error
	call	VacumeOff
	call	VacumeRelease
	call	WaitTowerUp
	jc	cmd_jamed
	jmp	RDroppedDisc

RGotNotCyclic:
	call	PickToAccept	; down over spindle
	jc	cmd_jamed	; return the carry error
	call	VacumeOff
	call	VacumeRelease
	call	AcceptToSpindle
	jc	cmd_jamed	; return the carry error

	; make sure disc sensor is correct
	jb	DISC_PRES_FLAG,RDroppedDisc
	call	TowerDelay
	jb	DISC_PRES_FLAG,RDroppedDisc
	call	TowerDelay
	jb	DISC_PRES_FLAG,RDroppedDisc
	jmp	cmd_jamed3
RDroppedDisc:

	; rotate to insert if was normal 2in/2out mode
	jb	CyclicHoppersBit,RDoneRotating
	call	RotateToInsert
	jc	cmd_jamed3
RDoneRotating:

rr_cmd_ok:
	clr	c		; return no error
	ret


cmd_jamed3:
	jmp	cmd_jamed




LoadCommand:
	call	RotateToInsert
	jc	cmd_jamed3
	call	WaitTowerDown
	jc	cmd_jamed3
	call	VacumeOn
	mov	Timer,#VAC_ON_DELAY
	call	Delay
	call	PickToSpindle
	jc	cmd_jamed3
	jnb	DISC_PRES_FLAG,LGotDisc

	; 2nd try
	call	TowerDelay
	call	WaitTowerDown
	jc	cmd_jamed3
	call	VacumeOn
	mov	Timer,#VAC_ON_DELAY
	call	Delay
	call	PickToSpindle
	jc	cmd_jamed3
	jnb	DISC_PRES_FLAG,LGotDisc

	; try again for a disc
	call	TowerDelay
	call	WaitTowerDown
	jc	cmd_jamed3
	call	VacumeOn
	mov	Timer,#VAC_ON_DELAY
	call	Delay
	call	PickToSpindle
	jc	cmd_jamed3
	jb	DISC_PRES_FLAG,lcmd_hopper_empty

LGotDisc:
	mov	Timer,#LOAD_PAUSE_AT_SPINDLE
	call	Delay		; short delay to allow possible second disc to fall off
	call	WaitTowerUp
	jc	cmd_jamed3
	call	RotateToDrives
	jc	cmd_jamed3
l_cmd_ok:
	clr	c		; return no error	
l_cmd_exit:
        ret



lcmd_hopper_empty:
	jnb	CyclicHoppersBit,LCmdNotCyclic
	mov	a,CurrentAcceptHopper
	cjne	a,#MAX_ACCEPT_HOPPER,LCmdOK
	jmp	lReallyEmpty
LCmdOK:
	inc	CurrentAcceptHopper
	jmp	LoadCommand	; back for another hopper

LCmdNotCyclic:
	jb	AuxHopperBit,lReallyEmpty  ; if already aut then really empty
	setb	AuxHopperBit	; now we are on aux hoppers
	jmp	LoadCommand	; back for another try

lReallyEmpty:
	call	VacumeOff
	mov	State,#STATE_ERROR
	mov	Error,#ERROR_EMPTY
	setb	c
	ret



cmd_jamed5:
	jmp	cmd_jamed	
	

InsertCommand:

	jb	DISC_PRES_FLAG,INoDisc
	jb	DISC_PRES_FLAG,INoDisc

	mov	Timer,#50		; 1/2 sec delay to make sure sensot did not bounce
	call	Delay
	jb	DISC_PRES_FLAG,INoDisc
	jnb	DISC_PRES_FLAG,IGotDisc	; already have a disc in gripper

INoDisc:
	call	RotateToInsert
	jc	cmd_jamed5
	call	WaitTowerDown
	jc	cmd_jamed5

	call	VacumeOn
	mov	Timer,#VAC_ON_DELAY
	call	Delay
	call	PickToSpindle
	jc	cmd_jamed5

	jnb	DISC_PRES_FLAG,IGotDisc

	; 2nd try
	call	TowerDelay
	call	WaitTowerDown
	jc	cmd_jamed5
	call	VacumeOn
	mov	Timer,#VAC_ON_DELAY
	call	Delay
	call	PickToSpindle
	jc	cmd_jamed4
	jnb	DISC_PRES_FLAG,IGotDisc

	; try again for a disc
	call	TowerDelay
	call	WaitTowerDown
	jc	cmd_jamed4
	call	VacumeOn
	mov	Timer,#VAC_ON_DELAY
	call	Delay
	call	PickToSpindle
	jc	cmd_jamed4
	jb	DISC_PRES_FLAG,cmd_hopper_empty

IGotDisc:
	mov	Timer,#LOAD_PAUSE_AT_SPINDLE
	call	Delay		; short delay to allow possible second disc to fall off
	call	WaitTowerUp
	jc	cmd_jamed4
	call	RotateToDrives
	jc	cmd_jamed4
	call	WaitTowerDown
	jc	cmd_jamed4
	call	VacumeOff
	call	VacumeReleaseNWait	; leave vac released in this call
;	call	PickToSpindle
	call	AccelWaitTowerUp
	push	psw
	call	VacumeRelOff		; now unrelease vacume - (we are at top of travel)
	pop	psw
	jc	cmd_jamed4

	; make sure disc sensor is correct
	jb	DISC_PRES_FLAG,IDroppedDisc
	call	TowerDelay
	jb	DISC_PRES_FLAG,IDroppedDisc
	call	TowerDelay
	jb	DISC_PRES_FLAG,IDroppedDisc
	jmp	cmd_jamed4
IDroppedDisc:


;	call	RotateToInsert
;	jc	cmd_jamed4
i_cmd_ok:
	clr	c		; return no error	
i_cmd_exit:
        ret

cmd_hopper_empty:
	jnb	CyclicHoppersBit,ICmdNotCyclic
	mov	a,CurrentAcceptHopper
	cjne	a,#MAX_ACCEPT_HOPPER,ICmdOK
	jmp	ReallyEmpty
ICmdOK:
	inc	CurrentAcceptHopper
	jmp	InsertCommand	; back for another hopper

ICmdNotCyclic:
	jb	AuxHopperBit,ReallyEmpty  ; if already aut then really empty
	setb	AuxHopperBit	; now we are on aux hoppers
	jmp	InsertCommand	; back for another try

ReallyEmpty:
	call	VacumeOff
	mov	State,#STATE_ERROR
	mov	Error,#ERROR_EMPTY
	setb	c
	ret

cmd_jamed4:
	jmp	cmd_jamed


 
; same as insert except don't release the disc
;	

AllignCommand:
	setb	DiagnosticsBit		; don't want the loader to home while aligning
	call	WaitTowerUp
	jc	cmd_jamed4
	jnb	DISC_PRES_FLAG,JGotDisc	; already have a disc in gripper

	call	RotateToInsert
	jc	cmd_jamed4
	call	WaitTowerDown
	jc	cmd_jamed4
	call	VacumeOn
	mov	Timer,#VAC_ON_DELAY
	call	Delay
	call	PickToSpindle
	jc	cmd_jamed4
	jnb	DISC_PRES_FLAG,JGotDisc

	; 2nd try
	call	TowerDelay
	call	WaitTowerDown
	jc	cmd_jamed4
	call	VacumeOn
	mov	Timer,#VAC_ON_DELAY
	call	Delay
	call	PickToSpindle
	jc	cmd_jamed4
	jnb	DISC_PRES_FLAG,JGotDisc

	; try again for a disc
	call	TowerDelay
	call	WaitTowerDown
	jc	cmd_jamed4
	call	VacumeOn
	mov	Timer,#VAC_ON_DELAY
	call	Delay
	call	PickToSpindle
	jc	cmd_jamed4
	jb	DISC_PRES_FLAG,jcmd_hopper_empty

JGotDisc:
	call	WaitTowerUp
	jc	cmd_jamed4
	call	RotateToDrives
	jc	cmd_jamed4
	call	WaitTowerDown
	jc	cmd_jamed4

	; lift the disc off the tray
	call	TowerUp
	mov	Timer,#8
	call	Delay
	call	TowerOff

	; wait for key hit to put the pick arm back up
	call	SerIn		; wait for a char
	call	WaitTowerUp
	jc	cmd_jamed4

	clr	c		; return no error	
        ret

jcmd_hopper_empty:
	jb	AuxHopperBit,JReallyEmpty  ; if already aut then really empty
	setb	AuxHopperBit	; now we are on aux hoppers
	jmp	AllignCommand	; back for another try

JReallyEmpty:
	call	VacumeOff
	mov	State,#STATE_ERROR
	mov	Error,#ERROR_EMPTY
	setb	c
	ret



;----------- Vacume code ----------------------
;
VacumeOn:
	setb	VAC_PUMP_ON_BIT
	setb	VacumeStatusBit
	ret

VacumeOff:
	clr	VAC_PUMP_ON_BIT
	clr	VacumeStatusBit
	ret

;---------- End of Vacume Code ----------


;---------- Vac release code -------------
;

VacumeRelOn:
	setb	VAC_RELEASE_BIT
	ret

VacumeRelOff:
	clr	VAC_RELEASE_BIT
	ret


VacumeRelease:
	call	VacumeRelOn
	mov	Timer,#100/2
	call	Delay
	call	VacumeRelOff
	ret	

VacumeReleaseNWait:
	call	VacumeRelOn
;	mov	Timer,#100/4
;	call	Delay
	mov	Timer,#100	; 1 sec test
	call	Delay
;	call	VacumeRelOff
	ret	

;---------- End of Vac release code --------------



;----------- Rotate Arm code ---------------------------------------
;
; rotate to insert (assumes that tt has been calibrated)
;
RotateToDrives:
	mov	a,#0		; drives always last hopper
	call	RotateToA
	ret


RotateToInsert:
	jnb	CyclicHoppersBit,NotCyclicInsert
	mov	a,CurrentAcceptHopper
	inc	a
	jmp	Rotate2I2
NotCyclicInsert:
        mov     a,#1		; 1 unless AuxHopperBit then is 2
	jnb	AuxHopperBit,Rotate2I2
	inc 	a
Rotate2I2:	
	call	RotateToA
        ret


RotateToAccept:
	jnb	CyclicHoppersBit,NotCyclicAccept
	mov	a,CurrentAcceptHopper
	jmp	Rotate2A2
NotCyclicAccept:
	mov	a,#3		; 3 unless AuxHopperBit then is 4 
	jnb	AuxHopperBit,Rotate2A2
	inc	a
Rotate2A2:
	call	RotateToA
	ret


RotateToReject:
	jnb	CyclicHoppersBit,NotCyclicReject
	mov	a,#0
	jmp	Rotate2R2
NotCyclicReject:

	mov	a,#5		; reject always last hopper
Rotate2R2:
	call	RotateToA
	ret

; rotate to home
;
RotateToHome:
	
	call	SetRotateSlow
	call	RotateOnCCW

	; find a hole ccw first
	mov	Timer5,#5*2
Rotate2HWaitHole1:
	mov	a,Timer5
	jz	Rotate2HError
	jb	ROT_POS_FLAG,Rotate2HWaitHole1
	jb	ROT_POS_FLAG,Rotate2HWaitHole1

	call	RotateOff
	mov	Timer,#100/2
	call	Delay

	; find an edge cw
	call	RotateOnCW
Rotate2HNextHole:
	mov	Timer5,#5*2
Rotate2HWaitHole:
	mov	a,Timer5
	jz	Rotate2HError
	jb	ROT_POS_FLAG,Rotate2HWaitHole
	jb	ROT_POS_FLAG,Rotate2HWaitHole
Rotate2HWaitEdge:
	mov	a,Timer5
	jz	Rotate2HError
	jnb	ROT_POS_FLAG,Rotate2HWaitEdge
	jnb	ROT_POS_FLAG,Rotate2HWaitEdge
	jb	ROT_HOME_FLAG,Rotate2HNextHole

	call	RotateOff
	mov	RotPosition,#ROT_HOME
	clr	LastMoveCCWBit
	clr	LastMoveCWBit
	clr	c
	ret
	

Rotate2HError:
	call	RotateOff
	setb	c
	ret
	


; rotate arm to station number in a
;
RotateToA:
	call	SetRotateFast
	cjne	a,RotPosition,NeedToMove
	call	RotateOff

	jb	LastMoveCCWBit,FinalCCWMove
	jb	LastMoveCWBit,FinalCWMove

	clr	LastMoveCCWBit
	clr	LastMoveCWBit
RotateArmToADone:
	mov	Timer,#100/4	; pause for mech to settle
	call	Delay
	clr	c
	ret			; nothing to do


FinalCCWMove:
	call	SetRotateSlow
	mov	Timer,#100/4	; pause for mech to settle
	call	Delay

	call	RotateOnCW
	mov	Timer5,#5*1	; 1 sec 
FinalCCWWaitEdge:
	mov	a,Timer5
	jz	Rotate2HError	
	jnb	ROT_POS_FLAG,FinalCCWWaitEdge
	jnb	ROT_POS_FLAG,FinalCCWWaitEdge
	call	RotateOff
	jmp	RotateArmToADone


FinalCWMove:
	call	RotateOff
	jmp	RotateArmToADone



; need to move turn table to correct station
;
NeedToMove:
	mov	b,a		; save target pos
	clr	c
	subb	a,RotPosition
	jnc	NTMoveCCW

	; must need to move CW
	; wait for edge (in case in the hole) then
	; wait for hole to clear

	; if last move go slow
	cjne	a,#0ffh,NTMoveCW	      
	call	SetRotateSlow	; only one pos to go
NTMoveCW:
	setb	LastMoveCWBit
	call	RotateOnCW
	mov	Timer5,#5*3	; 3 sec wait

	; wait for hole to clear
WaitEdgeCW:
	mov	a,Timer5
	jz	Rotate2HError2
	jnb	ROT_POS_FLAG,WaitEdgeCW
	jnb	ROT_POS_FLAG,WaitEdgeCW

	; wait for a hole
WaitHoleCW:
	mov	a,Timer5
	jz	Rotate2HError2
	jb	ROT_POS_FLAG,WaitHoleCW
	jb	ROT_POS_FLAG,WaitHoleCW

	; wait for hole to clear
WaitEdgeCW2:
	mov	a,Timer5
	jz	Rotate2HError2
	jnb	ROT_POS_FLAG,WaitEdgeCW2
	jnb	ROT_POS_FLAG,WaitEdgeCW2

	mov	a,b	; get back target pos
	dec	RotPosition
	jmp	RotateToA	; do more if needed



; wait for hole to clear, then wait for next edge
NTMoveCCW:
	cjne	a,#01,NTMoveCCWFast
	call	SetRotateSlow
NTMoveCCWFast:
	setb	LastMoveCCWBit
	call	RotateOnCCW
	mov	Timer5,#5*3
NTMoveCCWWaitHole:
	mov	a,Timer5
	jz	Rotate2HError2	
	jb	ROT_POS_FLAG,NTMoveCCWWaitHole
	jb	ROT_POS_FLAG,NTMoveCCWWaitHole

NTMoveCCWWaitEdge:
	mov	a,Timer5
	jz	Rotate2HError2	
	jnb	ROT_POS_FLAG,NTMoveCCWWaitEdge
	jnb	ROT_POS_FLAG,NTMoveCCWWaitEdge

NTMoveCCWWaitHole2:
	mov	a,Timer5
	jz	Rotate2HError2	
	jb	ROT_POS_FLAG,NTMoveCCWWaitHole2
	jb	ROT_POS_FLAG,NTMoveCCWWaitHole2

	mov	a,b	; get back target pos
	inc	RotPosition
	jmp	RotateToA	; do more if needed


Rotate2HError2:
	jmp	Rotate2HError	




RotateOff:
	db	AMOVI,CCAPM2,DISABLE_PWM
;	mov	CCAPM2,#DISABLE_PWM
	db	AMOVI,CCAPM3,DISABLE_PWM
;	mov	CCAPM3,#DISABLE_PWM
	ret


RotateOnCCW:
	mov	HomeTimeout,#HOME_TIMEOUT
	setb	LastMoveCCWBit
	db	AMOVI,CCAPM3,ENABLE_PWM
;	mov	CCAPM3,#ENABLE_PWM
	ret

RotateOnCW:
	mov	HomeTimeout,#HOME_TIMEOUT
	clr	LastMoveCCWBit
	db	AMOVI,CCAPM2,ENABLE_PWM
;	mov	CCAPM2,#ENABLE_PWM
	ret


SetRotateSlow:
	db	AMOVI,CCAP2H,ROT_SLOW_SPEED
;	mov	CCAP2H,#ROT_SLOW_SPEED
	db	AMOVI,CCAP3H,ROT_SLOW_SPEED
;	mov	CCAP3H,#ROT_SLOW_SPEED
	ret


SetRotateFast:
	db	AMOVI,CCAP2H,ROT_FAST_SPEED
;	mov	CCAP2H,#ROT_FAST_SPEED
	db	AMOVI,CCAP3H,ROT_FAST_SPEED
;	mov	CCAP3H,#ROT_FAST_SPEED
	ret


;----------- End of Rotate Arm code ---------------------------------------


;----------- Tower code ---------------------
PickToSpindle:
	mov	Timer5,#5*3
	call	TowerUp

WaitP2SEdge:
	mov	a,Timer5
	jz	WaitTowerError
	jb	PICK_ARM_HOME,WaitP2SEdge
	jb	PICK_ARM_HOME,WaitP2SEdge

WaitP2S:
	mov	a,Timer5
	jz	WaitTowerError
	jnb	PICK_ARM_HOME,WaitP2S
	jnb	PICK_ARM_HOME,WaitP2S


WaitP2sHole:
	mov	a,Timer5
	jz	WaitTowerError
	jb	PICK_ARM_HOME,WaitP2sHole
	jb	PICK_ARM_HOME,WaitP2sHole

	call	TowerOff
	clr	c
	ret


WaitTowerError:
	jmp	WaitTowerError2


;
; move pick from over spindle to onto spindle
;
PickToAccept:
	mov	Timer5,#5*2
	call	TowerDown

WaitP2AWtHole:
	mov	a,Timer5
	jz	WaitTowerError
	jnb	PICK_ARM_HOME,WaitP2AWtHole
	jnb	PICK_ARM_HOME,WaitP2AWtHole

WaitP2AWtEdge:
	mov	a,Timer5
	jz	WaitTowerError
	jb	PICK_ARM_HOME,WaitP2AWtEdge
	jb	PICK_ARM_HOME,WaitP2AWtEdge

	call	TowerOff
    	clr	c
	ret

;
; move pick from on spindle to over spindle
;
AcceptToSpindle:
	call	TowerUp
	mov	Timer5,#5*2

WaitA2SWtHole:
	mov	a,Timer5
	jnz	WaitA2SWtHole2
	mov	a,Timer5
	jz	WaitTowerErrorX
WaitA2SWtHole2:
	jnb	PICK_ARM_HOME,WaitA2SWtHole
	jnb	PICK_ARM_HOME,WaitA2SWtHole

WaitA2SWtEdge:
	mov	a,Timer5
	jnz	WaitA2SWtEdge2
	mov	a,Timer5
	jz	WaitTowerErrorX
WaitA2SWtEdge2:
	jb	PICK_ARM_HOME,WaitA2SWtEdge
	jb	PICK_ARM_HOME,WaitA2SWtEdge

	call	TowerOff
    	clr	c
	ret



WaitTowerErrorX:
	call	TowerOff
	setb	c
	jmp	WaitTUpExit	


; move tower to up position accelerating out of drive
;
AccelWaitTowerUp:
	setb	TowerTry0
	call	AccelTowerUp		; start gripper up slowly so disc does not bounce

	mov	Timer5,#5*3
	jmp	WaitTUp	


; move the tower to top position
;
WaitTowerUp:
	setb	TowerTry0
	call	TowerUp

	mov	Timer5,#5*3
WaitTUp:
;	mov	a,Timer5
;;	jz	WaitTowerError2
;	cjne	a,#0,WTUNoTO
;	jmp	WaitTowerError2
;WTUNoTO:
;	jnb	PICK_ARM_POS,WaitTUp
;	jb	PICK_ARM_HOME,WaitTUp
;	call	TowerOff
;
;	mov	a,#255
;	jnb	TowerTry0,WaitTowerError2 ; no 3rd try
;
;	clr	TowerTry0
;	mov	Timer,#10
;	call	Delay
;
;	; if not where we should be then try again
;	jnb	PICK_ARM_POS,WTUWrong
;	jb	PICK_ARM_HOME,WTUWrong
;	jmp	WaitTUpOK
;
;; we weren't up!
;; so try again
;;
;WTUWrong:
;	call	TowerUp
WaitTUp2:
	mov	a,Timer5
;	jz	WaitTowerError2
	cjne	a,#0,WTUNoTO2
	jmp	WaitTowerError2
WTUNoTO2:
	jnb	PICK_ARM_POS,WaitTUp2
	jb	PICK_ARM_HOME,WaitTUp2
	call	TowerOff

WaitTUpOK:
	clr	c
WaitTUpExit:
	ret

WaitTowerError2:
	call	TowerOff
	setb	c
	jmp	WaitTUpExit	


; move gripper up to top (if see spindle pos hole then stop there)
;
AcceptWaitTowerUp:
	; wait for both sensors to be over metal
	mov	Timer5,#5*3
	call	TowerUp
AWaitNoHoles:
	mov	a,Timer5
	jz	WaitTowerError2
	jb	PICK_ARM_HOME,AWaitNoHoles
	jb	PICK_ARM_HOME,AWaitNoHoles
	jb	PICK_ARM_POS,AWaitNoHoles
	jb	PICK_ARM_POS,AWaitNoHoles
	
	; if hole on POS sensor we are done
	; if hole on HOME sensor wiat for metal again then run for 1/4 sec more
AWaitHole:
	mov	a,Timer5
	jz	WaitTowerError2
	jb	PICK_ARM_POS, AWaitGotPosHole
	jnb	PICK_ARM_HOME,AWaitHole
	jnb	PICK_ARM_HOME,AWaitHole

AwaitHomeHole:
	mov	a,Timer5
	jz	WaitTowerError2
	jb	PICK_ARM_HOME,AwaitHomeHole
	jb	PICK_ARM_HOME,AwaitHomeHole


	; have Home sensor
	; so run for 1/10 sec more

	mov	Timer,#10
	call	Delay
	jmp	Done


AWaitGotPosHole:
	jnb	PICK_ARM_POS,AWaitHole	; make sure still have hole

Done:
	call	TowerOff
	clr	c
AWaitTUpExit:
	ret


; delay to settle manufold after going up before starting down
;
TowerDelay:
	mov	Timer,#75
	call	Delay
	ret


; move the tower to bottom position
;
WaitTowerDown:
	mov	Timer5,#5*4
	call	TowerDown
WaitTDn:
	mov	a,Timer5
	jnz	WaitTNoTO
	mov	a,Timer5
	jz	WaitTowerError2
WaitTNoTO:
	jb	MANIFOLD_FLAG,WaitTDone
	jnb	PICK_ARM_POS,WaitTDn
	jnb	PICK_ARM_HOME,WaitTDn
WaitTDone:
	call	TowerOff
	clr	c

WaitTDnExit:
	ret

TowerDown:
;	mov	TargetCCAPM1,#ENABLE_PWM
;	mov	a,TargetCCAPM1
;	cjne	a,#ENABLE_PWM,TowerDownDone
;	mov	TargetCCAPM1,#ENABLE_PWM
;	mov	a,#ENABLE_PWM
	db	AMOVI,CCAPM1,ENABLE_PWM
;	mov	CCAPM1,#ENABLE_PWM
;TowerDownDone:
        ret

TowerUp:
;	mov	TargetCCAPM0,#ENABLE_PWM
;	mov	a,TargetCCAPM0
;	cjne	a,#ENABLE_PWM,TowerUPDone
;	mov	TargetCCAPM0,#ENABLE_PWM
;	mov	a,TargetCCAPM0
;	mov	a,#ENABLE_PWM
	db	AMOVI,CCAPM0,ENABLE_PWM
;	mov	CCAPM0,#ENABLE_PWM
;TowerUPDone:
        ret

AccelTowerUp:
	mov	PickSpeed,#PICK_START_SPEED
;	mov	a,#ENABLE_PWM
	db	AMOVI,CCAPM0,ENABLE_PWM
;	mov	CCAPM0,#ENABLE_PWM
;TowerUPDone:
        ret


TowerOff:
	db	AMOVI,CCAPM0,DISABLE_PWM
;	mov	CCAPM0,#DISABLE_PWM
	db	AMOVI,CCAPM1,DISABLE_PWM
;	mov	CCAPM1,#DISABLE_PWM
        ret


;----------- End of Tower code ---------------------


	PAGE
;********************************************************************
;
; general purpose functions
;
;********************************************************************

; copy a string pointed to by the dptr to the serial port untill 
;       a char with hi bit is found
;
PutString:
	mov     StrOffset,#0           ;point to first byte of string
put_loop:
	mov     a,StrOffset            ;get offset to next char
	movc    a,@a+dptr               ;get the character
	jb      acc.7,put_done          ;see if last character
	call   SerOut                    ;send it
	inc     StrOffset              ;point to next char
	sjmp    put_loop
put_done:                               ;send last character
	clr     acc.7                   
	call   SerOut
	ret

;       turn the lower nibble into ascii

	public  Acc2Ascii
Acc2Ascii:
	anl     a,#0fh                  ; strip it
	cjne    a,#0ah,$+3              ; see if greater than 9
	jc      acc_less_9              ; no
	add     a,#7                    ; kick from number to ascii letter
acc_less_9:
	add     a,#30h                  ; up to ascii
	ret
; delay routine - delay for count in reg a
;
	public  Delay
Delay:
	mov     a,Timer                 ; see if done yet
	orl	a,#0
	jnz     Delay                   ; keep waiting
	ret

	

	PAGE
;********************************************************************
;
; RS-232 serial routines
;
;********************************************************************
SerIntEntry:
	push    a
	push    psw
	push    Scratch
	jb      ri,not_xmit             ; branch if recieve interrupt
	jnb     ti,s_int_exit           ; bad interupt
	clr     ti                      ; reset interupt
	clr     TxBusy                 ; must have been a ti int

s_int_exit:
	pop     Scratch
	pop     psw                     ; return flags
	pop     a                       ; return acc
	reti                            ; thats all

not_xmit:
	clr     ri                      ; clear the recieve interrupt
	mov     a,sbuf                  ; read the charecter
	cjne    a,#'@',$+3              ; see if needs upper casing
	jc      no_need_upit            ; its a number ( less than 'A')
	clr     acc.5                   ; force to upper case
no_need_upit:
	mov     SerBuffer,a               ; save charecter to pass on.
	ljmp    s_int_exit

; recieve a character in a - ignore parity and any framing errors
;
	public	SerIn
SerIn:   
	jnb	NeedToHomeLoader,DontHomeLoader
	; loader needs to be homed - has been 10 sec since last command
	call	WaitTowerUp		; gripper to top
	call	RotateToHome		; rotate to drives
	clr	NeedToHomeLoader
	mov	HomeTimeout,#0		; dont wan't timeout restarted
DontHomeLoader:

	mov     a,SerBuffer               ; see if char avail
	jnz     sgetc                   ; yes. go read it
	sjmp    SerIn

sgetc:  
	mov     SerBuffer,#0              ; clear it out of buffer
	ret


; transmit character in a - setting correct parity etc
;
SerOut:
	jb      TxBusy,SerOut          ;wait for last char
	mov     sbuf,a                  ;send the char
	setb    TxBusy                 ;flag char is transmiting
	ret                             ;done

;
; set the serial port to 9600 baud 8 bit no parity
;
;
SetSerial:
        mov     a,#0fdh                 ;default is 9600
	mov     th1,a
	mov     tl1,a
; tmod and tcon already set
;        mov     scon,#0d0h              ; 8 bit no parity
        mov     scon,#050h              ; 8 bit no parity
sgo:
	setb    tr1             ;start the serial port timer
	ret                     ;done

;
SendAscii:
	push    a
	swap	a		; send hi nibble first
	lcall   Acc2Ascii
	lcall   SerOut
	pop     a
	lcall   Acc2Ascii
	lcall   SerOut
	ret





	PAGE
;********************************************************************
;
; flash reprogram code
;
;********************************************************************


DoFlashProgram:
	mov	a,#PC_IMPACT	; ack the F command w/binary product code
	call	SerOut
	call	SerIn		; get correct product code back
	cjne	a,#PC_IMPACT,DoFlashAbort
	mov	a,#'X'		; ack the product code
	call	SerOut
DFWaitSer:
	jb	TxBusy,DFWaitSer
	
	mov	IE,#0		; shut down interrupts
	mov	SCON,#0		; shut down serial port
	mov	TCON,#0		; shut down counter
	mov	TMOD,#0
	mov	SP,#07h		; stack pointer to reset
	mov	PSW,#0
	db	AMOVI,CCON,0
;	mov	CCON,#0
	db	AMOVI,CMOD,0
;	mov	CMOD,#0
	; enable internal ROM
	db	AMOVI,AUXR1,020h ; do manually cause assembler doens't know about AUXR1
;	mov	AUXR1,#020h
	jmp	Program		; off to internal flash programing


DoFlashAbort:
	mov	a,#'E'		; nak the product code
	call	SerOut
	ret



	PAGE
;********************************************************************
;
; Signature Printer Drivers
;
;********************************************************************

;SIG_PORT	reg	p0
;SIG_IN_BIT	reg	p0.5
;SIG_OUT_BIT	reg	p0.6
;SIG_BUSY_BIT	reg	p0.4
;SIG_DISC_BIT	reg	p0.3
;SIG_STROBE_BIT	reg	p0.2

; Timer5 = decs 5 counts per sec 

; Do any printer initialization
; return carry if no printer or can't init
;
;SigInit:
;	setb	SIG_STROBE_BIT	; hopefully this was already done
;	mov	c,SIG_IN_BIT	; if both in & out set then no printer
;	anl	c,SIG_OUT_BIT
;	jc	SigInitExit	; return carry for error
;	call	SigClose
;SigInitExit:
;	ret

; return carry if have a sig printer
;
SigCheckForPrinter:
	setb	SIG_STROBE_BIT	; hopefully this was already done
	mov	c,SIG_IN_BIT	; if both in & out set then no printer
	anl	c,SIG_OUT_BIT
	cpl	c
	ret			; return carry if printer

	
; return printer status bits.
; if no printer will get 0fh returned
;	
SigPrinterStatus:
	mov	a,SIG_PORT	; get current printer bits
	rr	a		; shift sig status bits into low nibble
	rr	a
	rr	a
	anl	a,#0fh		; dump the top nibble
	lcall   Acc2Ascii
	lcall   SerOut
	ret


; open printer tray 
; return carry if error
;
SigOpenTray:
	jnb	HaveSigPrinterBit,SigTrayError	; see if printer installed
	call	SigWaitBusy	; make sure not already moving
	jc	SigTrayError	; timed out waiting
	
	clr	c		; assume no problems
	jb	SIG_OUT_BIT,SigOpenExit	; already open
	
	; need to open the printer tray
	call	SigStrobeMoveBit
	call	SigWaitBusy
	jc	SigTrayError	; timed out waiting for busy
	jnb	SIG_OUT_BIT,SigTrayError ; got good open
SigOpenExit:
	ret


; close printer tray
; return carry if error
;
SigCloseTray:
	jnb	HaveSigPrinterBit,SigTrayError	; see if printer installed
	call	SigWaitBusy		; make sure not already moving
	jc	SigTrayError		; timed out waiting
	
	clr	c			; assume no problems
	jb	SIG_IN_BIT,SigCloseExit	; already closed
	
	; need to close the printer tray
	call	SigStrobeMoveBit
	call	SigWaitBusy
	jc	SigTrayError		; timed out waiting for busy
	jnb	SIG_IN_BIT,SigTrayError ; still not closed
SigCloseExit:
	ret

SigTrayError:
	jmp	ErrorPrinter


; close tray used by calibrate
; so that it will return to the calib routine
;
CalibSigCloseTray:
	jnb	HaveSigPrinterBit,CSigCloseOK	; nothing to do if no printer
	call	SigWaitBusy		; make sure not already moving
	jc	CSigTrayError		; timed out waiting
	jb	SIG_IN_BIT,CSigCloseOK	; already closed
	
	; need to close the printer tray
	call	SigStrobeMoveBit
	call	SigWaitBusy
	jc	CSigTrayError		; timed out waiting for busy
	jnb	SIG_IN_BIT,CSigTrayError ; still not closed
CSigCloseOK:
	clr	c
CSigCloseExit:
	ret

CSigTrayError:
	setb	c
	jmp	CSigCloseExit



; strobe the start move bit
;
SigStrobeMoveBit:
	clr	SIG_STROBE_BIT
	mov	Timer,#100/2
	call	Delay		; 1/2 sec delay (to allow time for busy bit fron printer)
	setb	SIG_STROBE_BIT
	ret


; wait 15 sec for the busy bit to go away
; return carry if timed out
;
SigWaitBusy:
	mov	Timer5,#5*15	; 15 sec timeout
SigWaitBusyLoop:
	mov	a,Timer5
	jz	SigWaitBusyTimeout
	jb	SIG_BUSY_BIT,SigWaitBusyLoop
	clr	c		; clear error flag
	ret

SigWaitBusyTimeout:
	setb	c		; set timeout flag
	ret


	PAGE
;********************************************************************
;
; Cyclic hopper commands
;
;********************************************************************

ResetHopperCommand:
	mov	CurrentAcceptHopper,#1
	clr	AuxHopperBit	; restart hopper sequence for 2 in / 2 out 
	ret

ExtendedHoppersCommand:
	setb	CyclicHoppersBit
	mov	CurrentAcceptHopper,#1
	ret
	
StandardHoppersCommand:
	clr	CyclicHoppersBit
	ret

	PAGE
;********************************************************************
;
; Timer 0 interupt routine
;
;********************************************************************
T0IntEntry:
	push    psw
	push    a
	push    dpl
	push    dph
;	call   int_2                   ; do every time
	djnz    TCount3,int_ret
	mov     TCount3,#4
;	call    do_int_1000             ; ms stuff
	djnz    TCount1,int_ret
	mov     TCount1,#5             ;set up for 200 counts/sec
	call    do_int_200              ; do the 200 times a sec stuff
	djnz    TCount4,int_ret
	mov     TCount4,#2
	call    do_int_100              ; do the 100 times a second stuff
	djnz    TCount2,int_ret
	mov     TCount2,#20            ;set up for ~5 counts/sec
	call    do_int_5

; done back to main
;
int_ret:
	pop     dph
	pop     dpl
	pop     a
	pop     psw
	reti


; every interupt stuff (.2 ms)
;int_2:
;        ret


; 1000 times a second stuff
;do_int_1000:
;	mov     a,Timer2                ; see if other timer running
;	jz      n_timer2
;	dec     Timer2
;n_timer2:
;	ret

; 200 times a second stuff
do_int_200:
;	mov	a,TargetCCAPM0
;	db	AMOVR,CCAPM0
;	mov	a,TargetCCAPM1
;	db	AMOVR,CCAPM1
;	mov	a,TargetCCAPM2
;	db	AMOVR,CCAPM2
;	mov	a,TargetCCAPM3
;	db	AMOVR,CCAPM3



; code to accelerate pick from PickSpeed power to PIC_FAST_SPEED power
;
	mov	a,PickSpeed
	cjne	a,#PICK_FAST_SPEED,Int200AreAccelerating
	jmp	Int200Done
Int200AreAccelerating:
	inc	PickSpeed
	inc	PickSpeed
	
	mov	a,PickSpeed
;	mov	CCAP0H,a
	db	AMOVR,CCAP0H
	
;	mov	CCAP1H,a
	db	AMOVR,CCAP1H
	
Int200Done:
	ret



; 100 times a second stuff
;
do_int_100:
	mov     a,Timer
	jz      d_with_timer
	dec     Timer
d_with_timer:
	ret


; enter this code 5 times per second
;
do_int_5:
	mov	a,HomeTimeout
	jz	DoInt5InDiags		; already timed out
	djnz	HomeTimeout,DoInt5InDiags
	jnb	AutoCalibrateOKBit,DoInt5InDiags ; don't home if last command was not accept or reject
	jb	DiagnosticsBit,DoInt5InDiags	; don't actually home if in diags
	setb	NeedToHomeLoader
DoInt5InDiags:
	mov     a,Timer5
	jz      d_with_timer
	dec     Timer5
d5_with_timer:

;
;       check stack
;
	mov     a,sp                    ; get pointer
	cjne    a,#7fh,$+3              ; see if greater than ram
	jc      stack_ok                ; no
	ljmp    BootError               ; bad news              

stack_ok:

; done back to the main
;
	ret


;	org	CODE_END - 1
;	db	0ffh			; force length to be 16k

; Flash program entry points
;
 	org	0fc00h
Program:


	org	0fff0h
FlashISP:
	end


