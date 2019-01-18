/*
	Copyright (c) 2019 Shutong Feng
 
	All rights reserved.

	Redistribution and use in source and binary forms, with or without 
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written 
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
	POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "sf.h"
#include "instr-riscv.h"
#include "mextern.h"

typedef union 
{
    float fvalue;
    uint32_t bits;
    struct
    {
        unsigned    significant:23;
        unsigned    exponent:8;
        unsigned    sign:1;
    } fields;
} FLOAT;

static uint32_t sign_extend(uint32_t data, uint8_t n)
{
    int32_t tmp = data;
    tmp <<= 32 - n;
    tmp >>= 32 - n;
    return tmp;
}

enum 
{
    NEG_INF     = 0b0000000001,
    NEG_NORM    = 0b0000000010,
    NEG_SUBNORM = 0b0000000100,
    NEG_ZERO    = 0b0000001000,
    POS_ZERO    = 0b0000010000,
    POS_SUBNORM = 0b0000100000,
    POS_NORM    = 0b0001000000,
    POS_INF     = 0b0010000000,
    SIGNAL_NAN  = 0b0100000000,
    QUIET_NAN   = 0b1000000000,
};

void riscv_flw(Engine *E, State *S, uint8_t rs1, uint8_t rd, uint16_t imm0)
{
	uint32_t addr = reg_read_riscv(E, S, rs1) + sign_extend(imm0, 12);
    uint32_t bits = superHreadlong(E, S, addr);
    float data;
    memcpy(&data, &bits, sizeof(uint32_t));
    freg_set_riscv(E, S, rd, data);
	return;
}

void riscv_fsw(Engine *E, State *S, uint8_t rs1, uint8_t rs2, uint16_t imm0, uint16_t imm5)
{
    uint32_t addr = reg_read_riscv(E, S, rs1) + sign_extend(imm0 + (imm5 << 5), 12);
    float data = freg_read_riscv(E, S, rs2);
    uint32_t bits;
    memcpy(&bits, &data, sizeof(uint32_t));
	superHwritelong(E, S, addr, bits);
}

void riscv_fmadd_s(Engine *E, State *S, uint8_t rs1, uint8_t rs2, uint8_t rs3, uint8_t rd) 
{
    float data = fmaf(freg_read_riscv(E, S, rs1), freg_read_riscv(E, S, rs2), freg_read_riscv(E, S, rs3));
    freg_set_riscv(E, S, rd, data);
}

void riscv_fmsub_s(Engine *E, State *S, uint8_t rs1, uint8_t rs2, uint8_t rs3, uint8_t rd) 
{
    float data = fmaf(freg_read_riscv(E, S, rs1), freg_read_riscv(E, S, rs2), -freg_read_riscv(E, S, rs3));
    freg_set_riscv(E, S, rd, data);
}

void riscv_fnmadd_s(Engine *E, State *S, uint8_t rs1, uint8_t rs2, uint8_t rs3, uint8_t rd) 
{
    float data = -fmaf(freg_read_riscv(E, S, rs1), freg_read_riscv(E, S, rs2), freg_read_riscv(E, S, rs3));
    freg_set_riscv(E, S, rd, data);
}

void riscv_fnmsub_s(Engine *E, State *S, uint8_t rs1, uint8_t rs2, uint8_t rs3, uint8_t rd) 
{
    float data = -fmaf(freg_read_riscv(E, S, rs1), freg_read_riscv(E, S, rs2), -freg_read_riscv(E, S, rs3));
    freg_set_riscv(E, S, rd, data);
}

void riscv_fadd_s(Engine *E, State *S, uint8_t rs1, uint8_t rs2, uint8_t rd) 
{
    float data = freg_read_riscv(E, S, rs1) + freg_read_riscv(E, S, rs2);
    freg_set_riscv(E, S, rd, data);
}

void riscv_fsub_s(Engine *E, State *S, uint8_t rs1, uint8_t rs2, uint8_t rd) 
{
    float data = freg_read_riscv(E, S, rs1) - freg_read_riscv(E, S, rs2);
    freg_set_riscv(E, S, rd, data);
}

void riscv_fmul_s(Engine *E, State *S, uint8_t rs1, uint8_t rs2, uint8_t rd) 
{
    float data = freg_read_riscv(E, S, rs1) * freg_read_riscv(E, S, rs2);
    freg_set_riscv(E, S, rd, data);
}

void riscv_fdiv_s(Engine *E, State *S, uint8_t rs1, uint8_t rs2, uint8_t rd) 
{
    float data = freg_read_riscv(E, S, rs1) / freg_read_riscv(E, S, rs2);
    freg_set_riscv(E, S, rd, data);
}

void riscv_fsqrt_s(Engine *E, State *S, uint8_t rs1, uint8_t rs2, uint8_t rd) 
{
    float data = sqrtf(freg_read_riscv(E, S, rs1));
    freg_set_riscv(E, S, rd, data);
}

void riscv_fsgnj_s(Engine *E, State *S, uint8_t rs1, uint8_t rs2, uint8_t rd) 
{
    // copy sign bit from rs2 to rs1
    FLOAT fp1, fp2;
    float data;

    fp1.fvalue = freg_read_riscv(E, S, rs1);
    fp2.fvalue = freg_read_riscv(E, S, rs2);
    fp1.fields.sign = fp2.fields.sign;

    data = fp1.fvalue;
    freg_set_riscv(E, S, rd, data);
}

void riscv_fsgnjn_s(Engine *E, State *S, uint8_t rs1, uint8_t rs2, uint8_t rd) 
{
    // copy sign bit from -rs2 to rs1
    FLOAT fp1, fp2;
    float data;

    fp1.fvalue = freg_read_riscv(E, S, rs1);
    fp2.fvalue = -freg_read_riscv(E, S, rs2);
    fp1.fields.sign = fp2.fields.sign;

    data = fp1.fvalue;
    freg_set_riscv(E, S, rd, data);
}

void riscv_fsgnjx_s(Engine *E, State *S, uint8_t rs1, uint8_t rs2, uint8_t rd) 
{
    // copy sign bit rs2 XOR rs1 to rs1
    FLOAT fp1, fp2;
    float data;

    fp1.fvalue = freg_read_riscv(E, S, rs1);
    fp2.fvalue = freg_read_riscv(E, S, rs2);
    fp1.fields.sign = fp1.fields.sign ^ fp2.fields.sign;

    data = fp1.fvalue;
    freg_set_riscv(E, S, rd, data);
}

void riscv_fmin_s(Engine *E, State *S, uint8_t rs1, uint8_t rs2, uint8_t rd) 
{
    float data = fminf(freg_read_riscv(E, S, rs1), freg_read_riscv(E, S, rs2));
    freg_set_riscv(E, S, rd, data);
}

void riscv_fmax_s(Engine *E, State *S, uint8_t rs1, uint8_t rs2, uint8_t rd) 
{
    float data = fmaxf(freg_read_riscv(E, S, rs1), freg_read_riscv(E, S, rs2));
    freg_set_riscv(E, S, rd, data);
}

void riscv_fcvt_w_s(Engine *E, State *S, uint8_t rs1, uint8_t rs2, uint8_t rd) 
{
    // cvt float in rs1 to signed int in rd
    float fdata = freg_read_riscv(E, S, rs1);
    int32_t data = rintf(fdata);
    reg_set_riscv(E, S, rd, data);
}

void riscv_fcvt_wu_s(Engine *E, State *S, uint8_t rs1, uint8_t rs2, uint8_t rd) 
{
    // cvt float in rs1 to unsigned int in rd
    float fdata = freg_read_riscv(E, S, rs1);
    uint32_t data = rintf(fdata);
    reg_set_riscv(E, S, rd, data);
}

void riscv_fle_s(Engine *E, State *S, uint8_t rs1, uint8_t rs2, uint8_t rd) 
{
    // store 1 in rd if rs1 <= rs2
    if (freg_read_riscv(E, S, rs1) <= freg_read_riscv(E, S, rs2))
        reg_set_riscv(E, S, rd, 1);
    else 
        reg_set_riscv(E, S, rd, 0);
}

void riscv_flt_s(Engine *E, State *S, uint8_t rs1, uint8_t rs2, uint8_t rd) 
{
    // store 1 if rd if rs1 < rs2
    if (freg_read_riscv(E, S, rs1) < freg_read_riscv(E, S, rs2))
        reg_set_riscv(E, S, rd, 1);
    else 
        reg_set_riscv(E, S, rd, 0);
}

void riscv_feq_s(Engine *E, State *S, uint8_t rs1, uint8_t rs2, uint8_t rd) 
{
    // store 1 if rs1 == rs2
    if (freg_read_riscv(E, S, rs1) == freg_read_riscv(E, S, rs2))
        reg_set_riscv(E, S, rd, 1);
    else 
        reg_set_riscv(E, S, rd, 0);
}

void riscv_fcvt_s_w(Engine *E, State *S, uint8_t rs1, uint8_t rs2, uint8_t rd) 
{
    // cvt signed int in rs1 to float in rd
    int32_t xdata = (int32_t) reg_read_riscv(E, S, rs1);
    float data = (float) xdata;
    freg_set_riscv(E, S, rd, data);
}

void riscv_fcvt_s_wu(Engine *E, State *S, uint8_t rs1, uint8_t rs2, uint8_t rd) 
{
    // cvt unsigned int in rs1 to float in rd
    uint32_t xdata = reg_read_riscv(E, S, rs1);
    float data = (float) xdata;
    freg_set_riscv(E, S, rd, data);
}

void riscv_fmv_w_x(Engine *E, State *S, uint8_t rs1, uint8_t rs2, uint8_t rd) 
{
    FLOAT fp;
    fp.bits = reg_read_riscv(E, S, rs1);
    freg_set_riscv(E, S, rd, fp.fvalue);
}

void riscv_fmv_x_w(Engine *E, State *S, uint8_t rs1, uint8_t rs2, uint8_t rd) 
{
    FLOAT fp;
    fp.fvalue = freg_read_riscv(E, S, rs1);
    reg_set_riscv(E, S, rd, fp.bits);
}

void riscv_fclass_s(Engine *E, State *S, uint8_t rs1, uint8_t rs2, uint8_t rd) 
{
    FLOAT fp;
    uint32_t data;
    fp.fvalue = freg_read_riscv(E, S, rs1);
    
    if (fp.fields.exponent == 0b00000000)
    {
        if (fp.fields.sign)
        {
            if (fp.fields.significant)
                data = NEG_SUBNORM;
            else
                data = NEG_ZERO;
        }
        else
        {
            if (fp.fields.significant)
                data = POS_SUBNORM;
            else
                data = POS_ZERO;
        }
    }
    else if (fp.fields.exponent == 0b11111111)
    {
        if (!fp.fields.significant)
        {
            if (fp.fields.sign)
                data = NEG_INF;
            else
                data = POS_INF;
        }
        else
        {
            if (fp.fields.significant >> 22)
                data = QUIET_NAN;
            else 
                data = SIGNAL_NAN;
        }
    }
    else
    {
        if (fp.fields.sign)
            data = NEG_NORM;
        else
            data = POS_NORM;
    }

    reg_set_riscv(E, S, rd, data);
}
