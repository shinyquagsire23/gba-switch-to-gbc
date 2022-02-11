
; Interrupt Flags (offset)
DEF hIF EQU $0F
; Interrupt Flags
DEF rIF EQUS "[$FF00+$0F]"
; VBlank interrupt
DEF nIF_VBL EQU (1 << 0)
; VBlank interrupt (bit)
DEF bIF_VBL EQU 0
; STAT interrupt
DEF nIF_STAT EQU (1 << 1)
; STAT interrupt (bit)
DEF bIF_STAT EQU 1
; Timer interrupt
DEF nIF_TIMER EQU (1 << 2)
; Timer interrupt (bit)
DEF bIF_TIMER EQU 2
; Serial interrupt
DEF nIF_SERIAL EQU (1 << 3)
; Serial interrupt (bit)
DEF bIF_SERIAL EQU 3
; Button interrupt
DEF nIF_PAD EQU (1 << 4)
; Button interrupt (bit)
DEF bIF_PAD EQU 4
; Software interrupt 5
DEF nIF_SWI5 EQU (1 << 5)
; Software interrupt 5 (bit)
DEF bIF_SWI5 EQU 5
; Software interrupt 6
DEF nIF_SWI6 EQU (1 << 6)
; Software interrupt 6 (bit)
DEF bIF_SWI6 EQU 6
; Software interrupt 7
DEF nIF_SWI7 EQU (1 << 7)
; Software interrupt 7 (bit)
DEF bIF_SWI7 EQU 7

; LCD controller (offset)
DEF hLCDC EQU $40
; LCD controller
DEF rLCDC EQUS "[$FF00+$40]"
; Enable bakcground tiles (window included)
DEF nLCDC_BG_EN EQU (1 << 0)
; Enable bakcground tiles (window included) (bit)
DEF bLCDC_BG_EN EQU 0
; Enable objects
DEF nLCDC_OBJ_EN EQU (1 << 1)
; Enable objects (bit)
DEF bLCDC_OBJ_EN EQU 1
; Object size (0 = 8x8, 1 = 8x16)
DEF nLCDC_OBJ_SIZE EQU (1 << 2)
; Object size (0 = 8x8, 1 = 8x16) (bit)
DEF bLCDC_OBJ_SIZE EQU 2
; Tile map selection (0 = 9800-9C00, 1 = 9C00 - A000)
DEF nLCDC_BG_MAP_ADDR EQU (1 << 3)
; Tile map selection (0 = 9800-9C00, 1 = 9C00 - A000) (bit)
DEF bLCDC_BG_MAP_ADDR EQU 3
; Tile data selection (0 = 8800 - 9800, 1 = 8000 - 9000)
DEF nLCDC_BG_DATA_ADDR EQU (1 << 4)
; Tile data selection (0 = 8800 - 9800, 1 = 8000 - 9000) (bit)
DEF bLCDC_BG_DATA_ADDR EQU 4
; Window enable
DEF nLCDC_WIN_EN EQU (1 << 5)
; Window enable (bit)
DEF bLCDC_WIN_EN EQU 5
; Window tile map selection (0 = 9800 - 9C00, 1 = 9C00 - A000)
DEF nLCDC_WIN_MAP EQU (1 << 6)
; Window tile map selection (0 = 9800 - 9C00, 1 = 9C00 - A000) (bit)
DEF bLCDC_WIN_MAP EQU 6
; LCD enable (only disable during VBlank!)
DEF nLCDC_LCD_EN EQU (1 << 7)
; LCD enable (only disable during VBlank!) (bit)
DEF bLCDC_LCD_EN EQU 7

; LCD flags (offset)
DEF hSTAT EQU $41
; LCD flags
DEF rSTAT EQUS "[$FF00+$41]"
; Current display mode bit 0
DEF nSTAT_MODEBIT0 EQU (1 << 0)
; Current display mode bit 0 (bit)
DEF bSTAT_MODEBIT0 EQU 0
; Current display mode bit 1
DEF nSTAT_MODEBIT1 EQU (1 << 1)
; Current display mode bit 1 (bit)
DEF bSTAT_MODEBIT1 EQU 1
; LYC Compare (set if LY == LYC)
DEF nSTAT_LYCC EQU (1 << 2)
; LYC Compare (set if LY == LYC) (bit)
DEF bSTAT_LYCC EQU 2
; STAT IRQ for HBlank
DEF nSTAT_IRQ_HBLANK EQU (1 << 3)
; STAT IRQ for HBlank (bit)
DEF bSTAT_IRQ_HBLANK EQU 3
; STAT IRQ for VBlank
DEF nSTAT_IRQ_VBLANK EQU (1 << 4)
; STAT IRQ for VBlank (bit)
DEF bSTAT_IRQ_VBLANK EQU 4
; STAT OAM search interrupt
DEF nSTAT_IRQ_OAM EQU (1 << 5)
; STAT OAM search interrupt (bit)
DEF bSTAT_IRQ_OAM EQU 5
; STAT LYC Compare interrupt
DEF nSTAT_IRQ_LYCC EQU (1 << 6)
; STAT LYC Compare interrupt (bit)
DEF bSTAT_IRQ_LYCC EQU 6
; Current display mode
DEF uSTAT_MODE_SEL EQU ($03)
; Current display mode (shifted)
DEF uSTAT_MODE_SEL_SH EQU ($03 << 0)
; HBlank
DEF uSTAT_MODE_HBLANK EQU ($00)
; HBlank (shifted)
DEF uSTAT_MODE_HBLANK_SH EQU ($00 << 0)
; VBlank
DEF uSTAT_MODE_VBLANK EQU ($01)
; VBlank (shifted)
DEF uSTAT_MODE_VBLANK_SH EQU ($01 << 0)
; OAM sprite scan
DEF uSTAT_MODE_OAMSCAN EQU ($02)
; OAM sprite scan (shifted)
DEF uSTAT_MODE_OAMSCAN_SH EQU ($02 << 0)
; Displaying an image (in active area)
DEF uSTAT_MODE_DISPLAYING EQU ($03)
; Displaying an image (in active area) (shifted)
DEF uSTAT_MODE_DISPLAYING_SH EQU ($03 << 0)

; BG Scroll Y (offset)
DEF hSCY EQU $42
; BG Scroll Y
DEF rSCY EQUS "[$FF00+$42]"

; BG Scroll X (offset)
DEF hSCX EQU $43
; BG Scroll X
DEF rSCX EQUS "[$FF00+$43]"

; LCD Y (offset)
DEF hLY EQU $44
; LCD Y
DEF rLY EQUS "[$FF00+$44]"

; LCD Y Compare (offset)
DEF hLYC EQU $45
; LCD Y Compare
DEF rLYC EQUS "[$FF00+$45]"

; OAM DMA (high byte) (offset)
DEF hOAMDMA EQU $46
; OAM DMA (high byte)
DEF rOAMDMA EQUS "[$FF00+$46]"

; BG Palette (offset)
DEF hPAL EQU $47
; BG Palette
DEF rPAL EQUS "[$FF00+$47]"

; OBJ Palette 0 (offset)
DEF hPALO0 EQU $48
; OBJ Palette 0
DEF rPALO0 EQUS "[$FF00+$48]"

; OBJ Palette 1 (offset)
DEF hPALO1 EQU $49
; OBJ Palette 1
DEF rPALO1 EQUS "[$FF00+$49]"

; Window Y (offset)
DEF hWINY EQU $4A
; Window Y
DEF rWINY EQUS "[$FF00+$4A]"

; Window X (offset)
DEF hWINX EQU $4B
; Window X
DEF rWINX EQUS "[$FF00+$4B]"

; Unknown (offset)
DEF hUNK4C EQU $4C
; Unknown
DEF rUNK4C EQUS "[$FF00+$4C]"

; Overclock register (offset)
DEF hOC EQU $4D
; Overclock register
DEF rOC EQUS "[$FF00+$4D]"
; Enable mode switch on STOP
DEF nOC_DO EQU (1 << 0)
; Enable mode switch on STOP (bit)
DEF bOC_DO EQU 0
; Overclock mode (0 = default, 1 = overclocked)
DEF nOC_STAT EQU (1 << 7)
; Overclock mode (0 = default, 1 = overclocked) (bit)
DEF bOC_STAT EQU 7

; VRAM bank select (0 = data, 1 = CGB attr) (offset)
DEF hVBANK EQU $4F
; VRAM bank select (0 = data, 1 = CGB attr)
DEF rVBANK EQUS "[$FF00+$4F]"

; HDMA Source High (offset)
DEF hHDMASH EQU $51
; HDMA Source High
DEF rHDMASH EQUS "[$FF00+$51]"

; HDMA Source Low (offset)
DEF hHDMASL EQU $52
; HDMA Source Low
DEF rHDMASL EQUS "[$FF00+$52]"

; HDMA Destination High (offset)
DEF hHDMADH EQU $53
; HDMA Destination High
DEF rHDMADH EQUS "[$FF00+$53]"

; HDMA Destination Low (offset)
DEF hHDMADL EQU $54
; HDMA Destination Low
DEF rHDMADL EQUS "[$FF00+$54]"

; HDMA Control (offset)
DEF hHDMAC EQU $55
; HDMA Control
DEF rHDMAC EQUS "[$FF00+$55]"
; Copy amount counter (0 = $10, 1 = $20, ...) bit 0
DEF nHDMAC_LENBIT0 EQU (1 << 0)
; Copy amount counter (0 = $10, 1 = $20, ...) bit 0 (bit)
DEF bHDMAC_LENBIT0 EQU 0
; Copy amount counter (0 = $10, 1 = $20, ...) bit 1
DEF nHDMAC_LENBIT1 EQU (1 << 1)
; Copy amount counter (0 = $10, 1 = $20, ...) bit 1 (bit)
DEF bHDMAC_LENBIT1 EQU 1
; Copy amount counter (0 = $10, 1 = $20, ...) bit 2
DEF nHDMAC_LENBIT2 EQU (1 << 2)
; Copy amount counter (0 = $10, 1 = $20, ...) bit 2 (bit)
DEF bHDMAC_LENBIT2 EQU 2
; Copy amount counter (0 = $10, 1 = $20, ...) bit 3
DEF nHDMAC_LENBIT3 EQU (1 << 3)
; Copy amount counter (0 = $10, 1 = $20, ...) bit 3 (bit)
DEF bHDMAC_LENBIT3 EQU 3
; Copy amount counter (0 = $10, 1 = $20, ...) bit 4
DEF nHDMAC_LENBIT4 EQU (1 << 4)
; Copy amount counter (0 = $10, 1 = $20, ...) bit 4 (bit)
DEF bHDMAC_LENBIT4 EQU 4
; Copy amount counter (0 = $10, 1 = $20, ...) bit 5
DEF nHDMAC_LENBIT5 EQU (1 << 5)
; Copy amount counter (0 = $10, 1 = $20, ...) bit 5 (bit)
DEF bHDMAC_LENBIT5 EQU 5
; Copy amount counter (0 = $10, 1 = $20, ...) bit 6
DEF nHDMAC_LENBIT6 EQU (1 << 6)
; Copy amount counter (0 = $10, 1 = $20, ...) bit 6 (bit)
DEF bHDMAC_LENBIT6 EQU 6
; Copy mode (0 = all at once, 1 = HDMA)
DEF nHDMAC_MODE EQU (1 << 7)
; Copy mode (0 = all at once, 1 = HDMA) (bit)
DEF bHDMAC_MODE EQU 7
; Copy amount counter (0 = $10, 1 = $20, ...)
DEF uHDMAC_LEN_SEL EQU ($7F)
; Copy amount counter (0 = $10, 1 = $20, ...) (shifted)
DEF uHDMAC_LEN_SEL_SH EQU ($7F << 0)

; BG Palette Index (offset)
DEF hPALI EQU $68
; BG Palette Index
DEF rPALI EQUS "[$FF00+$68]"

; BG Palette Data (offset)
DEF hPALD EQU $69
; BG Palette Data
DEF rPALD EQUS "[$FF00+$69]"

; OBJ Palette Index (offset)
DEF hPALOI EQU $6A
; OBJ Palette Index
DEF rPALOI EQUS "[$FF00+$6A]"

; OBJ Palette Data (offset)
DEF hPALOD EQU $6B
; OBJ Palette Data
DEF rPALOD EQUS "[$FF00+$6B]"

; WRAM bank (0 - 7) (offset)
DEF hWBANK EQU $70
; WRAM bank (0 - 7)
DEF rWBANK EQUS "[$FF00+$70]"

; Palette Selector Mode (offset)
DEF hPSM EQU $71
; Palette Selector Mode
DEF rPSM EQUS "[$FF00+$71]"
; PSM functionality enable
DEF nPSM_EN EQU (1 << 0)
; PSM functionality enable (bit)
DEF bPSM_EN EQU 0
; PSM Window enable
DEF nPSM_WIN_EN EQU (1 << 7)
; PSM Window enable (bit)
DEF bPSM_WIN_EN EQU 7

; Palette selector Window X (offset)
DEF hPSX EQU $72
; Palette selector Window X
DEF rPSX EQUS "[$FF00+$72]"

; Palette selector Window Y (offset)
DEF hPSY EQU $73
; Palette selector Window Y
DEF rPSY EQUS "[$FF00+$73]"

; Palette selector key watch (offset)
DEF hPSW EQU $74
; Palette selector key watch
DEF rPSW EQUS "[$FF00+$74]"

; Interrupt Enable (offset)
DEF hIE EQU $FF
; Interrupt Enable
DEF rIE EQUS "[$FF00+$FF]"
