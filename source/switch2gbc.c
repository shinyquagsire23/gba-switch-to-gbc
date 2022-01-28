// SPDX-License-Identifier: MIT
//
// Copyright (c) 2014, 2020 Antonio Niño Díaz (AntonioND)

#include <stdio.h>

#include <gba.h>

#define ALWAYS_INLINE __attribute__((always_inline)) static inline

const uint8_t gbc_payload[0x3B] = {0xAF, 0xE0, 0x40, 0x21, 0x10, 0x80, 0x0E, 0x80, 0x2A, 0xE2, 0x0C, 0x20, 0xFB, 0xC3, 0x80, 0xFF, 0x3E, 0x80, 0xE0, 0x40, 0x06, 0x0C, 0xAF, 0xE0, 0x0F, 0x3C, 0xE0, 0xFF, 0x18, 0x15, 0x05, 0x20, 0xF5, 0x3E, 0xFF, 0xE0, 0x24, 0xE0, 0x25, 0xE0, 0x26, 0x3E, 0x82, 0xE0, 0x12, 0xE0, 0x13, 0xE0, 0x14, 0x18, 0xE1, 0xF0, 0x0F, 0xE6, 0x01, 0x28, 0xFA, 0x18, 0xE3};

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

// BSS is by default in IWRAM
uint16_t GBC_DISPCNT_VALUE;

IWRAM_CODE void prepare_registers(void)
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

    int i;
    for (i = 0; i < 0x18000 / 4; i ++) // Fill VRAM with 0xFF
        ((u32*)VRAM)[i] = 0xCFCFCFCF;

    BG_PALETTE[0] = 0x0000;
    BG_PALETTE[1] = 0x7FFF;

    REG_BG2CNT = 0x4180;
    REG_BG2X = 0xFFFFD800; // -40.0
    REG_BG2Y = 0xFFFFF800; // -8.0

    REG_SOUNDCNT_H = 0x88C2;
    REG_SOUNDBIAS = 0xC200; // 6 bit, 262.144kHz
    
    ((u32*)VRAM)[0] = 0xCFCFCFCF;
    
    //*(vu32*)0x4000800 = 0xFEFFFFF8;
    
    //*(vu32*)0x4000800 = 0x0E0000F8;
}

IWRAM_CODE void switch2gbc(void)
{
    REG_IME = 0;
    
    //*(vu32*)0x4000800 = 0x0D000028 | 2;

    // Write 0x0408 to DISPCNT = 0x0408: Mode 0, GBC mode enabled, BG2 enabled
    GBC_DISPCNT_VALUE = 0x0408;

    // GBC mode bit can only be modified from BIOS, like from inside CpuSet()
    // Copy 1 halfword, 16 bit mode
    SWI_CpuSet(&GBC_DISPCNT_VALUE, (void *)(REG_BASE + 0), 1);
    
    //memset(0x02000000, 0xff, 0x100);
    
    //*(vu32*)0x4000800 = 0x0D000020 | 1; // jump to bootrom overlay thing
    
    uint32_t hival = (3 << 5) | (2 << 7) | (1 << 9) | (0 << 10) | (0 << 12) | (1 << 15);
    
    REG_DMA3SAD = 0x06010000;
    REG_DMA3DAD = 0x06000000;
    //REG_DMA3CNT = 0x1000 | (hival << 16);
    
    // Normal boot, black screen with jingle
    //*(vu32*)0x4000800 = 0x0D000000 | 0 | 2 | 4 | 0 | 0x10 | 0x20;
    //SWI_Halt();

    // BIOS swapped boot, white screen  no jingle
    //*(vu32*)0x4000800 = 0x0D000000 | 0 | 2 | 4 | 8 | 0x10 | 0x20;
    //SWI_Halt();

    //void (*jump_fn)(void) = (void*)0x06010000;
    //void (*jump_fn)(void) = (void*)0x03000000;
    //jump_fn();

    //REG_IME = 1;

    // VRAM sets up registers, then jumps to 0x0 (actually 0x03000000) to trigger GBC mode
    asm volatile(
        
        "ldr r0, =0x06010000\n"
        "mov lr, r0\n"
        //"ldr r0, =0x03000010\n"
        "bx r0"
    );

    // It seems that the GBC mode begins when HALTCNT is written.
    
    *(uint8_t*)0x4000301 = 0x00;

    // Never reached in hardware. Trap emulators.
    while (1)
    {
        *(uint8_t*)0x4000301 = 0x00;
        //SWI_Halt();
    }
}

IWRAM_CODE void simpleirq(void)
{
    REG_IME = 0;
    REG_IF = 0xFFFF;
    REG_IME = 1;
}

IWRAM_CODE void delayed_switch2gbc(void)
{
    REG_IME = 0;
    /*consoleDemoInit();
    iprintf("Swap cartridges now!\n");
    iprintf("\n");
    iprintf("Waiting 10 seconds...\n");*/
    
    //memcpy((void*)0x02000000, cgb_agb_boot_bin, cgb_agb_boot_bin_size);

//*(uint32_t*)0x02000000 = 0xEAFFFFFE;
    //*(uint32_t*)0x02000004 = 0xEAFFFFFE;
    //*(uint32_t*)0x02000008 = 0xEAFFFFFE;
    //*(uint32_t*)0x0200000C = 0xEAFFFFFE;

#if 0 
    *(uint32_t*)0x02000000 = 0x0;
    *(uint32_t*)0x02000004 = 0x0;
    *(uint32_t*)0x02000008 = 0x0;
    *(uint32_t*)0x0200000C = 0xE3A00004;
    *(uint32_t*)0x02000010 = 0xE1A00C00;
    *(uint32_t*)0x02000014 = 0xE2800C03;
    
    *(uint32_t*)0x02000018 = 0xE3A01000;
    *(uint32_t*)0x0200001C = 0xE5C01001;
    
    *(uint32_t*)0x02000020 = 0xE3A00006;
    *(uint32_t*)0x02000024 = 0xE1A00C00;
    
    *(uint32_t*)0x02000028 = 0xe3a01000;
    *(uint32_t*)0x0200002C = 0xe7801001;
    *(uint32_t*)0x02000030 = 0xe2811004;
    *(uint32_t*)0x02000034 = 0xe3510801;
    *(uint32_t*)0x02000038 = 0xeafffffb;
    
    *(uint32_t*)0x0200003C = 0xEAFFFFFE;
#endif

    uint32_t* out = (uint32_t*)0x02000000;

    *out = 0; out++;
    *out = 0; out++;
    *out = 0x0; out++;
    *out = 0x0; out++;
    *out = 0x0; out++;
    *out = 0x0; out++;
    *out = 0x0; out++;
    *out = 0x0; out++;
    //*out = 0xEAFFFFFE; out++;
    //*out = 0xe12fff1e; out++;
    //*out = 0xe25ef004; out++;
    *out = 0xe59F0000; out++;
    *out = 0xe12fff10; out++;
    *out = 0x06010000; out++;

    *out = 0xe5c88001; out++;
    *out = 0xe12fff1e; out++;
    
    
#if 0
    *(uint32_t*)0x03000000 = 0x0;
    *(uint32_t*)0x03000004 = 0x0;
    *(uint32_t*)0x03000008 = 0x0;
    *(uint32_t*)0x0300000C = 0xE3A00004;
    *(uint32_t*)0x03000010 = 0xE1A00C00;
    *(uint32_t*)0x03000014 = 0xE2800C03;
    
    *(uint32_t*)0x03000018 = 0xE3A01000;
    *(uint32_t*)0x0300001C = 0xE5C01001;
    
    //*(uint32_t*)0x03000020 = 0xEAFFFFF9;
    //*(uint32_t*)0x03000020 = 0xE3A00006;
    *(uint32_t*)0x03000024 = 0xE1A00C00;
    
    *(uint32_t*)0x03000028 = 0xe3a01000;
    *(uint32_t*)0x0300002C = 0xe7801001;
    *(uint32_t*)0x03000030 = 0xe2811004;
    *(uint32_t*)0x03000034 = 0xe3510801;
    *(uint32_t*)0x03000038 = 0xeafffffb;
    
    *(uint32_t*)0x0300003C = 0xEAFFFFFE;
#endif

    out = (uint32_t*)0x03000000;

    

    //memcpy(out, (void*)((intptr_t)&RAM_stub & ~1), 0x80);

#if 0
    *out = 0x0; out++;
    *out = 0x0; out++;
    *out = 0x0; out++;
    *out = 0x0; out++;
    *out = 0xe59F0000; out++;
    *out = 0xe12fff10; out++;
    *out = 0x06010000; out++;
#endif

    *out = 0; out++;
    *out = 0; out++;
    *out = 0x0; out++;
    *out = 0x0; out++;
    *out = 0x0; out++;
    *out = 0x0; out++;
    *out = 0x0; out++;
    *out = 0x0; out++;
    //*out = 0xEAFFFFFE; out++;
    //*out = 0xe12fff1e; out++;
    //*out = 0xe25ef004; out++;
    *out = 0xe59F0000; out++;
    *out = 0xe12fff10; out++;
    *out = 0x06010000; out++;


    out = (uint32_t*)0x03000100;
    *out = 0xe59f200c; out++;
    *out = 0xe59f300c; out++;
    *out = 0xe5823000; out++;
    *out = 0xe5c88001; out++;
    *out = 0xe12fff1e; out++;
    *out = 0x04000800; out++;
    *out = 0xffffffdf; out++;
    
#if 0
    *out = 0xFF00FF00; out++;
    *out = 0x0; out++;
    *out = 0x0; out++;
    *out = 0x0; out++;

    *out = 0xe3a00406; out++;
    *out = 0xe3a01000; out++;
    *out = 0xe3a03301; out++;
    *out = 0xe2833c03; out++;
    *out = 0xe7801001; out++;
    //*out = 0x0; out++;
    *out = 0xe2811004; out++;
    *out = 0xe3510801; out++;
    *out = 0x1a000000; out++;
    *out = 0xe3a01000; out++;
    *out = 0xe5c33001; out++;
    *out = 0xeafffff8; out++;
#endif

    out = (uint32_t*)0x02020000;

    

    //memcpy(out, (void*)((intptr_t)&RAM_stub & ~1), 0x80);

#if 1
    *out = 0x0; out++;
    *out = 0x0; out++;
    *out = 0x0; out++;
    *out = 0x0; out++;
    *out = 0xe59F0000; out++;
    *out = 0xe12fff10; out++;
    *out = 0x06010000; out++;
#endif

    out = (uint32_t*)0x06010000;

    //*out = 0xFF00FF00; out++;
    //*out = 0x0; out++;
    //*out = 0x0; out++;
    //*out = 0x0; out++;
    
    /**out = 0xe3a00406; out++;
    *out = 0xe3a01000; out++;
    *out = 0xe3a03301; out++;
    *out = 0xe2833c03; out++;
    //*out = 0xe7801001; out++;
    *out = 0x0; out++;
    *out = 0xe2811004; out++;
    *out = 0xe3510801; out++;
    *out = 0x1a000000; out++;
    *out = 0xe3a01000; out++;
    *out = 0xe5c33001; out++;
    *out = 0xeafffff8; out++;*/

    memcpy(out, (void*)((intptr_t)&RAM_stub & ~1), 0x100);

    // Write payload to IWRAM
    uint8_t* iwram_8 = (uint32_t*)0x03000000;
    memset(iwram_8, 0, 0x3B*4);
    for (int i = 0; i < 0x3B; i++)
    {
        iwram_8[i * 4] = gbc_payload[i];
    }

    //irqEnable(IRQ_TIMER0);

    // Clocks per second = 16777216 = 16 * 1024 * 1024
    // With 1024 prescaler = 16 * 1024 for one second

    uint16_t ticks_per_second = 16 * 1024 * 10;

    //REG_TM0CNT_L = UINT16_MAX - ticks_per_second;
    //REG_TM0CNT_H = TIMER_START | TIMER_IRQ | 3;

    //for (int i = 0; i < 10 * 1024; i++)
    //    SWI_Halt();
    while ((REG_KEYINPUT & (1 << 8)));
    while (!(REG_KEYINPUT & (1 << 8)));

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

    *(uint32_t*)0x03007FFC = 0x06010000 + (14*4);

    switch2gbc();
}

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
