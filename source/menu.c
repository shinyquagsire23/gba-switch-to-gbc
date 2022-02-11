// SPDX-License-Identifier: MIT
//
// Copyright (c) 2020 Antonio Niño Díaz (AntonioND)

#include <stdio.h>
#include <string.h>

#include <gba.h>

#include "switch2gbc.h"

//#include "cgb_bootstrap.h"

// --------------------------------------------------------------------

#define PACKED __attribute__((packed))

#if 0
// Struct that holds the input to SWI_BgAffineSet()
typedef struct PACKED {
    int32_t bgx;        // 24.8 fixed point
    int32_t bgy;        // 24.8 fixed point
    int16_t scrx;
    int16_t scry;
    int16_t scalex;     // 8.8 fixed point
    int16_t scaley;     // 8.8 fixed point
    uint16_t angle;     // 8.8 fixed point. Range: 0 - 0xFFFF
    uint16_t padding;
} bg_affine_src;

// Struct that holds the state of a background affine transformation. It is used
// as container of the output of SWI_BgAffineSet()
typedef struct PACKED {
    int16_t pa;
    int16_t pb;
    int16_t pc;
    int16_t pd;
    int32_t xoff;
    int32_t yoff;
} bg_affine_dst;

bg_affine_src aff_src;
bg_affine_dst aff_dst;

// This function gets a list of background transformations and outputs the
// correct affine matrices for the GBA hardware.
void SWI_BgAffineSet(const bg_affine_src *src, bg_affine_dst *dst,
                     uint32_t count)
{
    register uint32_t src_ asm("r0") = (uint32_t)src;
    register uint32_t dst_ asm("r1") = (uint32_t)dst;
    register uint32_t count_ asm("r2") = count;

    asm volatile(
        "swi 0x0E" ::
        "r"(src_), "r"(dst_), "r"(count_) :
        "r3", "memory"
    );
}

void reset_affine_registers(void)
{
    aff_src.bgx = 160 / 2 << 8;
    aff_src.bgy = 144 / 2 << 8;

    aff_src.scrx = 40 + 160 / 2;
    aff_src.scry = 8 + 144 / 2;

    aff_src.scalex = 1 << 8;
    aff_src.scaley = 1 << 8;

    aff_src.angle = 0 << 8;
}

void refresh_affine_registers(void)
{
    REG_BG2PA = aff_dst.pa;
    REG_BG2PB = aff_dst.pb;
    REG_BG2PC = aff_dst.pc;
    REG_BG2PD = aff_dst.pd;
    REG_BG2X = aff_dst.xoff;
    REG_BG2Y = aff_dst.yoff;
}

void update_affine_registers(void)
{
    SWI_BgAffineSet(&aff_src, &aff_dst, 1);

    refresh_affine_registers();
}

int mosx, mosy;

void update_mosaic(void)
{
    REG_MOSAIC = mosx | mosy << 4;
}

void reset_mosaic(void)
{
    mosx = 0;
    mosy = 0;

    update_mosaic();
}
#endif

// --------------------------------------------------------------------

int main(void)
{
    //irqInit();
    INT_VECTOR = IntrMain;

    //consoleDemoInit();

    //reset_affine_registers();
    //reset_mosaic();

    // User configuration menu

    //enter_menu();

    //update_affine_registers();
    //update_mosaic();
    //load_menu();

    // Reset all registers to the values expected when switching to GBC mode

    prepare_registers();

    // Do funny stuff here with the IO registers

    // It is strange, when pressing L to stretch the screen, it seems that BG2
    // affine transformation is ignored.  When pressing R again, the affine
    // transformation is applied again.

    //refresh_affine_registers();
    //update_mosaic();
    //*((u16*)0x04000002) = greenswap;

    //REG_BG2CNT |= BG_MOSAIC;

    // Actually switch

    delayed_switch2gbc();

    return 0;
}
