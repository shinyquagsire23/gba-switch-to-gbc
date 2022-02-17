// SPDX-License-Identifier: MIT
//
// Copyright (c) 2014, 2020 Antonio Niño Díaz (AntonioND)

#include <stdio.h>

#include <gba.h>
#include <string.h>

#define ALWAYS_INLINE __attribute__((always_inline)) static inline
#define REG_POSTFLG (*(vu32*)0x04000804)
#define REG_HALTCNT (*(vu8*)0x04000301)

//const uint8_t gbc_payload[0x3B] = {0xAF, 0xE0, 0x40, 0x21, 0x10, 0x80, 0x0E, 0x80, 0x2A, 0xE2, 0x0C, 0x20, 0xFB, 0xC3, 0x80, 0xFF, 0x3E, 0x80, 0xE0, 0x40, 0x06, 0x0C, 0xAF, 0xE0, 0x0F, 0x3C, 0xE0, 0xFF, 0x18, 0x15, 0x05, 0x20, 0xF5, 0x3E, 0xFF, 0xE0, 0x24, 0xE0, 0x25, 0xE0, 0x26, 0x3E, 0x82, 0xE0, 0x12, 0xE0, 0x13, 0xE0, 0x14, 0x18, 0xE1, 0xF0, 0x0F, 0xE6, 0x01, 0x28, 0xFA, 0x18, 0xE3};

extern const uint8_t gbc_payload[];
extern const int gbc_payload_length;

extern void RAM_stub(void);

ALWAYS_INLINE void SWI_Halt(void)
{
    asm volatile(
        "swi 0x02" :::
        "r0", "r1", "r2", "r3", "memory"
    );
}

ALWAYS_INLINE void SWI_CpuSet(const void *src, void *dst, uint32_t len_mode)
{
    register uint32_t src_ asm("r0") = (uint32_t)src;
    register uint32_t dst_ asm("r1") = (uint32_t)dst;
    register uint32_t len_mode_ asm("r2") = len_mode;

    asm volatile(
        "swi 0x0B" ::
        "r"(src_), "r"(dst_), "r"(len_mode_) :
        "r3", "memory"
    );
}

void prepare_registers(void)
{
    // Reset all I/O to default values

    *((u16*)0x04000002) = 0; // GREENSWAP

    REG_BG0CNT = 0; REG_BG1CNT = 0; REG_BG3CNT = 0;
    REG_BG2PA = 0x0100; REG_BG2PB = 0x0000; REG_BG2PC = 0x0000; REG_BG2PD = 0x0100;
    REG_BG3PA = 0x0100; REG_BG3PB = 0x0000; REG_BG3PC = 0x0000; REG_BG3PD = 0x0100;
    REG_BG3X = 0; REG_BG3Y = 0;

    REG_WIN0H = 0; REG_WIN0V = 0; REG_WIN1H = 0; REG_WIN1V = 0;
    REG_WININ = 0; REG_WINOUT = 0;

    REG_MOSAIC = 0; REG_BLDCNT = 0; REG_BLDALPHA = 0; REG_BLDY = 0;

    REG_VCOUNT = 0;

    REG_BG0HOFS = 0; REG_BG0VOFS = 0; REG_BG1HOFS = 0; REG_BG1VOFS = 0;
    REG_BG2HOFS = 0; REG_BG2VOFS = 0; REG_BG3HOFS = 0; REG_BG3VOFS = 0;

    REG_SOUND1CNT_L = 0; REG_SOUND1CNT_H = 0; REG_SOUND1CNT_X = 0;
    REG_SOUND2CNT_L = 0; REG_SOUND2CNT_H = 0;
    REG_SOUND3CNT_L = 0; REG_SOUND3CNT_H = 0; REG_SOUND3CNT_X = 0;
    REG_SOUND4CNT_L = 0; REG_SOUND4CNT_H = 0;

    REG_SOUNDCNT_L = 0; REG_SOUNDCNT_X = 0;

    REG_DMA0SAD = 0; REG_DMA0DAD = 0; REG_DMA0CNT = 0;
    REG_DMA1SAD = 0; REG_DMA1DAD = 0; REG_DMA1CNT = 0;
    REG_DMA2SAD = 0; REG_DMA2DAD = 0; REG_DMA2CNT = 0;
    REG_DMA3SAD = 0; REG_DMA3DAD = 0; REG_DMA3CNT = 0;

    REG_TM0CNT = 0; REG_TM1CNT = 0; REG_TM2CNT = 0; REG_TM3CNT = 0;

    REG_KEYCNT = 0;

    //REG_WAITCNT = ???

    // Do BIOS configuration...

    //int i;
    //for (i = 0; i < 0x18000 / 4; i ++) // Fill VRAM with 0xFF
    //    ((u32*)VRAM)[i] = 0xCFCFCFCF;

    BG_PALETTE[0] = 0x0000;
    BG_PALETTE[1] = 0x7FFF;

    REG_BG2CNT = 0x4180;
    REG_BG2X = 0xFFFFD800; // -40.0
    REG_BG2Y = 0xFFFFF800; // -8.0

    REG_SOUNDCNT_H = 0x88C2;
    REG_SOUNDBIAS = 0xC200; // 6 bit, 262.144kHz
    
    //((u32*)VRAM)[0] = 0xCFCFCFCF;
    
    //*(vu32*)0x4000800 = 0xFEFFFFF8;
    
    //*(vu32*)0x4000800 = 0x0E0000F8;
}

/*
Never enters GBC mode
.iwram:03004000 __iwram_start                           ; DATA XREF: LOAD:0300007C↑o
.iwram:03004000                                         ; switch2gbc+14↓o ...
.iwram:03004000                 MOVS    R3, #0x80       ; Alternative name is '__iwram_start'
.iwram:03004000                                         ; do_halt
.iwram:03004002                 MOVS    R2, #0x81
.iwram:03004004                 LSLS    R3, R3, #0x13
.iwram:03004006                 LSLS    R2, R2, #3
.iwram:03004008                 STRH    R2, [R3]
.iwram:0300400A                 NOP
.iwram:0300400C                 NOP
.iwram:0300400E                 MOVS    R2, #1
.iwram:03004010                 LDR     R3, =0x4000300
.iwram:03004012                 STRB    R2, [R3]
.iwram:03004014                 MOVS    R2, #0
.iwram:03004016                 LDR     R3, =0x4000301
.iwram:03004018                 STRB    R2, [R3]
.iwram:0300401A
.iwram:0300401A loc_300401A                             ; CODE XREF: __iwram_start:loc_300401A↓j
.iwram:0300401A                 B       loc_300401A
*/

/*
Enters GBC mode
.iwram:03004000 __iwram_start                           ; DATA XREF: LOAD:0300007C↑o
.iwram:03004000                                         ; switch2gbc+14↓o ...
.iwram:03004000                 MOVS    R3, #0x80       ; Alternative name is '__iwram_start'
.iwram:03004000                                         ; do_halt
.iwram:03004002                 MOVS    R2, #0x81
.iwram:03004004                 LSLS    R3, R3, #0x13
.iwram:03004006                 LSLS    R2, R2, #3
.iwram:03004008                 STRH    R2, [R3]
.iwram:0300400A                 NOP
.iwram:0300400C                 NOP
.iwram:0300400E                 NOP
.iwram:03004010                 MOVS    R2, #1
.iwram:03004012                 LDR     R3, =0x4000300
.iwram:03004014                 STRB    R2, [R3]
.iwram:03004016                 MOVS    R2, #0
.iwram:03004018                 LDR     R3, =0x4000301
.iwram:0300401A                 STRB    R2, [R3]
.iwram:0300401C
.iwram:0300401C loc_300401C                             ; CODE XREF: __iwram_start:loc_300401C↓j
.iwram:0300401C                 B       loc_300401C
*/

#if 0
IWRAM_CODE void do_halt(void)
{
    REG_DISPCNT = 0x0408; // stalls forever here here (no nops)
    __asm__("nop"); // never enters CGB mode
    __asm__("nop"); // never enters CGB mode
    //__asm__("nop"); // boots to CGB
    
    REG_POSTFLG = 1;
    REG_HALTCNT = 0;
    while (1)
    {
        //REG_DISPCNT |= 0x08;
        //REG_DISPCNT &= ~0x08;
        //__asm__("nop");
    }

    while (1);
}
#endif

#if 1

IWRAM_CODE void do_halt(void)
{
    REG_DISPCNT = 0x0408; // stalls forever here here (no nops)
    /*__asm__("nop"); // boots to CGB
    __asm__("nop"); // boots to CGB
    __asm__("nop"); // boots to CGB
    __asm__("nop"); // boots to CGB
    __asm__("nop"); // boots to CGB
    __asm__("nop"); // boots to CGB
    __asm__("nop"); // stalls forever here
    __asm__("nop"); // stalls?
    __asm__("nop"); // boots to CGB
    __asm__("nop"); // boots to CGB
    __asm__("nop"); // boots to CGB
    __asm__("nop"); // boots to CGB
    __asm__("nop"); // boots to CGB
    __asm__("nop"); // boots to CGB
    __asm__("nop"); // stalls forever here*/
    int i = 0;
    REG_POSTFLG = 0;
    REG_HALTCNT = 0;
    do
    {
        REG_DISPCNT |= 0x8;
        i++;
        REG_DISPCNT &= ~0x8;
    }
    while (i < 0x8);

    REG_DISPCNT |= 0x8;

    //REG_HALTCNT = 0x80;
    __asm__("nop");
    __asm__("nop"); // boots to CGB if uncommented

    //while (1);
}
#endif

u32 xfer32(u32 tosend) 
{
    //Wait for exchange to start
    REG_SIODATA32 = tosend;
    REG_SIOCNT &= ~((1 << 7) | (1<<3));
    REG_SIOCNT |= (1 << 7) | (1<<3);
    while(REG_SIOCNT & 0x80){} // wait for send

    //Master sends data, we read
    //while((REG_RCNT & 0x1) == 0){}
    u32 ret = REG_SIODATA32; //Get data

    u32 timeout = 1000;
    while (!(REG_RCNT & 1))
    {
        if (!--timeout) break;
    }
    
    return ret;
}

#ifdef BIOS_TESTS
IWRAM_CODE void bios_tests(void)
{
    //REG_POSTFLG = 0xFFFFFFFF;
    while (1)
    {
        for (int i = 0x300; i < 0x1000; i += 4)
        {
            xfer32(*(u32*)(0x04000000+i));
            xfer32(0x04000000+i);
            xfer32(0x1234ABCD);
        }
    }
}
#endif // BIOS_TESTS

void switch2gbc(void)
{
    REG_IME = 0;

    //bios_idk();
    
    // BIOS swapped boot, white screen  no jingle
    *(vu32*)0x4000800 = 0x0D000000 | 1 | 8 | 0x20;
    //do_halt();

#ifdef BIOS_TESTS
    void (*jump_fn2)(void) = (void*)((intptr_t)bios_idk - 0x02000000);
    jump_fn2();
#endif // BIOS_TESTS

    //void (*jump_fn)(void) = (void*)0x06010000;
    void (*jump_fn)(void) = (void*)((intptr_t)do_halt - 0x02000000);
    jump_fn();

    REG_DISPCNT |= 0x80;

    // Never reached in hardware. Trap emulators.
    while (1)
    {
        //REG_BG2HOFS += 1;
        REG_DISPCNT |= 0x80;
        REG_DISPCNT &= ~0x80;
        
        // Uncommenting these in sequence seems to change whether the loop plays and CGB doesn't enter,
        // or if CGB is entered
        __asm__("nop"); // CGB
        __asm__("nop"); // CGB
        __asm__("nop"); // jittery screen
        __asm__("nop"); // jittery screen
        __asm__("nop"); // CGB
        __asm__("nop"); // CGB
        __asm__("nop"); // CGB
        __asm__("nop"); // CGB
        __asm__("nop"); // CGB
        __asm__("nop"); // CGB
        __asm__("nop"); // blank screen
        __asm__("nop"); // stable screen
        __asm__("nop"); // CGB
        __asm__("nop"); // CGB
        __asm__("nop"); // CGB
        __asm__("nop"); // CGB
        __asm__("nop"); // CGB
        __asm__("nop"); // CGB
        __asm__("nop"); // stable screen
        __asm__("nop"); // stable screen
        __asm__("nop"); // CGB
        __asm__("nop"); // CGB
        __asm__("nop"); // CGB
        __asm__("nop"); // CGB
        __asm__("nop"); // CGB
        __asm__("nop"); // CGB
        __asm__("nop"); // blank screen
        __asm__("nop"); // jitters
    }
}

void delayed_switch2gbc(void)
{
    REG_IME = 0;
    REG_RCNT = 0;
    REG_SIOCNT = 0x1008;

    // Write payload to IWRAM
    uint8_t* iwram_8 = (uint32_t*)0x03000000;
    //memset(iwram_8, 0, 0x3B*4);
    for (int i = 0; i < gbc_payload_length; i++)
    {
        iwram_8[i * 4] = gbc_payload[i];
    }

    memset(VRAM, 0xCF, 0xC000);

    BG_PALETTE[0] = 0x0000;
    BG_PALETTE[1] = 0x7FFF;
    
    
    REG_IME = 0;
    REG_IE = 0;
    REG_IF = 0xFFFF;

    REG_KEYCNT = (1 << 14) | (1 << 9) | (1 << 8);
    //irqEnable(IRQ_KEYPAD);
    REG_IE |= IRQ_KEYPAD;
    REG_IE |= IRQ_DMA3;
    
    
    //\x04\x00\xa0\xe3
    //\x00\x0c\xa0\xe1
    //\x03\x0c\x80\xe2
    //\x01\x00\xc0\xe5
    
    //memset(0x02000000, 0xCF, 0x10);
    //memset(0x03000000, 0xCF, 0xA);
    
    //uint16_t ticks_per_second = 16 * 1024;

    //REG_TM0CNT_L = UINT16_MAX - ticks_per_second;
    //REG_TM0CNT_H = TIMER_START | TIMER_IRQ | 3;

    //*(uint32_t*)0x03007FFC = 0x06010000 + (14*4);

    switch2gbc();
}

#if 0
__attribute__((naked)) void RAM_stub(void)
{
    asm volatile(
        ".arm\n"
        "nop\n"
        "nop\n"
        "nop\n"
        "nop\n"
        "MSR CPSR_c, #0xDF\n"
        "ldr sp, =0x06018000\n"
        "ldr r8, =0x04000300\n"
        "ldr r0, =0x4000800\n"
        "ldr r1, =(0xFFFF0000 | 1 | 2 | 4 | 0x10)\n"
        "str r1, [r0]\n"
        "ldr  r0, =0x6000000\n"
        "ldr  r1, =0x4000\n"
        "ldr r4, =0xFFFF\n"
        "ldr r5, =0x10000\n"
        "mov lr, pc\n"
        "mov r6, #0x20\n"
        "loop:\n"
        "str  r6, [r0, r1]\n"
        "add  r1, r1, #4\n"
        "cmp  r1, r5\n"
        "bne  skip\n"
        "mov  r1, #0\n"
        "skip:\n"
        "add r6, #0x1\n"
        "ldr  r3, =0x4000000\n"
        "lsl r2, r6, #0xA\n"
        "strh r2, [r3, #0x28]\n"
        "add  r3, r3, #0x80\n"
        "add  r3, r3, #0x80\n"
        "add  r3, r3, #0x80\n"
        "add  r3, r3, #0x80\n"
        "strh r4, [r3, #0x2]\n"
        "ldr  r3, =0x4000000\n"
        "ldr  r0, =0x06001000\n"
        "str r0, [r3, #0xD8]\n"
        "ldr r2, =0x08000000\n"
        "str r2, [r3, #0xD4]\n"
        "ldr r2, =(0x1000 | (((3 << 5) | (2 << 7) | (0 << 9) | (1 << 10) | (0 << 12) | (1 << 14) | (1 << 15)) << 16))\n"
        "str r2, [r3, #0xDC]\n"
        "ldr r2, =0x04000208\n"
        "mov r3, #0x1\n"
        "strb r3, [r2]\n"
        "mov r2, #0x100\n"
        "bx r2\n"
        //"swi #0x2\n"
        //"strb r8, [r8, #1]\n"
        
        "bx lr\n"
        "b loop\n"
    );
}
#endif
