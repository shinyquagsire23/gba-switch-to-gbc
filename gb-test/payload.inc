payload::
    xor  a
    ldh  [$FF40],a      ; Disable LCD as early as possible
    LDH [$FFFE], A
    LD A, $F0
    LD [$FF42], A ; scroll down +16
    ld   hl,.hram_payload_start
    ld   b,$7E
    ld   c,$80          ; Bottom of HRAM
.copyloop
    ldi  a,[hl]
    ld   [$ff00+c],a
    inc  c
    dec  b
    jr   nz,.copyloop
    jp   $FF80

.hram_payload_start
    ld   a,$91       
    ldh  [$FF40],a      ; Enable LCD
.restart
    ;LD A, (1 << 0)
    ;LD A, $FF
    ;LD [$FF00+$63], A
    
    ;LD DE, $C000
    LD DE, $FF00
    LD HL, $9800
    LD B, $10
.wtf_e
    LD C, $8
.wtf
    LD A, rSTAT
    AND A, $03
    JR nz, .wtf

    LD A, [DE]
    SWAP A
    OR A, $F0
    LD [HL+], A
    
    LD A, [DE]
    OR A, $F0
    LD [HL+], A
    
    INC DE
    
    DEC C
    JR nz, .wtf
    
    LD A, L
    ADD A, $10
    LD L, A
    JR nc, :+
    INC H
:
    DEC B
    JR nz, .wtf_e
    
    ;XOR A
    ;LD [$FF00+$63], A
    
    ld   b,20
.nextframe
    /*
    ld   a,$FF          ; Enable sound and route everywhere
    LD HL, $FF24
    ld  [HL+],a
    ld  [HL+],a
    ld  [HL+],a
    ld   a,$82          ; Use $82 as envelope, lo freq and hi freq
    LD HL, $FF12
    ld  [HL+],a
    ld  [HL+],a
    ld  [HL+],a
    */
    
    /*
    LD HL, $FFFE
    INC [HL]
    LD A, [HL]
    LD HL, $FF63
    LD [HL], A
    */
    
    xor  a
    ldh  [$FF0F],a      ; Clear interrupts
    inc  a
    ldh  [$FFFF],a      ; A=1, enable VBlank only
.waitint
    ldh  a,[$FF0F]
    and  a,$01
    jr   z,.waitint
    ;dec  b
    ;jr   nz,.nextframe
    jr   .restart
.hram_payload_end:
    ASSERT (.hram_payload_end - .hram_payload_start) < $7E
