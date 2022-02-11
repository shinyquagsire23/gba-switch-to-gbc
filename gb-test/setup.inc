SECTION "WTF", ROM0[$0]
NOP
JP $100

SECTION "Start", ROM0[$100]
    NOP
    JR $150

    DS $104 - @, $00

    DB $CE, $ED, $66, $66, $CC, $0D, $00, $0B, $03, $73, $00, $83, $00, $0C, $00, $0D, $00, $08, $11, $1F, $88, $89, $00, $0E, $DC, $CC, $6E, $E6, $DD, $DD, $D9, $99, $BB, $BB, $67, $63, $6E, $0E, $EC, $CC, $DD, $DC, $99, $9F, $BB, $B9, $33, $3E
    
    DS $143 - @, $00
    
    DB CGB_TYPE
    
    DS $150 - @, $00

SECTION "Main", ROM0[$150]

    LD A, $00
    LD rLCDC, A

    LD A, $80
    LDH [$FF68], A
    LD HL, $FF69
    XOR A
    LD [HL], A
    LD [HL], A
    ADD A, 85
    LD [HL], A
    LD [HL], A
    ADD A, 85
    LD [HL], A
    LD [HL], A
    ADD A, 85
    LD [HL], A
    LD [HL], A

;SECTION "Main", ROM0[$00]
;LOAD "Main Load", VRAM[$8000]
_start::
    DI
    
    LD HL, $FFFE
    LD SP, HL
    
    ;LD A, $C0
    ;LD [$FF00+$4C], A
    ;LD A, $11
    ;LD [$FF00+$50], A
    
    ; LCDC is guaranteed to be off, so is IE
    
    LD A, $1
    LD rVBANK, A
    CALL initVRAM
    
    XOR A
    LD rVBANK, A
    CALL initVRAM
    
    LD A, %11100100
    LD rPAL, A
    LD rPALO0, A
    LD rPALO1, A
    
    CALL initPaletteBG
    
    LD BC, font
    LD DE, ($60 * $8)
    LD HL, $8000 + ($20 * $10)
    CALL fontUploadSubWhatever
    
    LD BC, font + (($30 - $20) * $8)
    LD DE, (10 * $8)
    LD HL, $8000 + ($F0 * $10)
    CALL fontUploadSubWhatever
    
    LD BC, font + (($41 - $20) * $8)
    LD DE, (6 * $8)
    LD HL, $8000 + ($FA * $10)
    CALL fontUploadSubWhatever
    
    LD DE, $9800
    LD HL, szHello
    CALL fontPrintSz
    
    LD A, $91
    LD rLCDC, A
    
    XOR A
    LD rIE, A
    LD rIF, A
    
    /*
    XOR A
    LD [$FF00+$26], A
    LD A, $80
    LD [$FF00+$26], A
    LD A, $FF
    LD [$FF00+$25], A
    LD A, $77
    LD [$FF00+$24], A
    
    LD A, $FF
    LD [$FF00+$16], A
    LD A, $F0
    LD [$FF00+$17], A
    LD A, $FF
    LD [$FF00+$18], A
    LD A, $82
    LD [$FF00+$19], A
    */

    JP payload
:
    HALT
    NOP
    JR :-

szHello: DB "Hello GBC Micro :)", 0
font:
    INCBIN "data/font_tmp.bin"

; DE - dst
; HL - src
fontPrintSz::
:
    LD A, [HL+]
    OR A
    RET z
    
    LD [DE], A
    INC DE
    
    JR :-
    
    
; BC - font ptr
; HL - dst in VRAM
; DE - row count
fontUploadSubWhatever:
    LD A, [BC]
    INC BC
    LD [HL+], A
    LD [HL+], A ; Two bytes per bitplane line
    
    DEC DE ; Decrement remaining row count
    LD A, E
    OR D ; A = (D | E), aka. DE == 0
    JR nz, fontUploadSubWhatever
    
    ;JR .fontUploadSubShadowDone
    RET

initVRAM::
    LD A, $0
    LD HL, $8000
    LD BC, $2000
    
:
    LD [HL+], A
    DEC C
    JR nz, :-
    
    DEC B
    JR nz, :-
    
    RET
    
initPaletteBG::
    LD A, $80
    LD rPALI, A
    
    LD B, 8
:
    LD HL, colorwheel
    LD C, 8
:
    LD A, [HL+]
    LD rPALD, A
    DEC C
    JR nz, :-
    DEC B
    JR nz, :--
    
    RET

colorwheel:
    DW $7FFF                ; DATA XREF: sub_7A51+19?o
    DW $56B5
    DW $2D6B
    DW 0
    
    /*
    DW 31 * %000010000100001
    DW 20 * %000010000100001
    DW 10 * %000010000100001
    DW 0  * %000010000100001
    */

    

;ENDL
