/*
	Copyright (c) 1999-2017, Zhengyang Gu (author)
 
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
#include <string.h>
#include <fenv.h>
#include "sf.h"
#include "instr-riscv.h"
#include "opstr-riscv.h"
#include "mextern.h"

int
riscvstep(Engine *E, State *S, int drain_pipe)
{
	int			i;
	uint32_t	tmpinstr;
	uint32_t	tmpPC;
	Picosec		saved_globaltime;


	USED(drain_pipeline);

	saved_globaltime = E->globaltimepsec;
	for (i = 0; (i < E->quantum) && E->on && S->runnable; i++)
	{
		if (!eventready(E->globaltimepsec, S->TIME, S->CYCLETIME))
		{
			E->globaltimepsec = max(E->globaltimepsec, S->TIME) + S->CYCLETIME;
			continue;
		}

		if (S->sleep)
		{
			update_energy(OP_SLEEP, 0, 0);
			S->ICLK++;
			S->TIME += S->CYCLETIME;
			E->globaltimepsec = max(E->globaltimepsec, S->TIME) + S->CYCLETIME;

			continue;
		}

		tmpPC = S->PC;
		tmpinstr = superHreadlong(E, S, S->PC);

		riscvdecode(E, tmpinstr, &(S->riscv->P.EX));

		S->riscv->P.EX.fetchedpc = S->PC;
		S->PC += 4;
		S->CLK++;
		S->ICLK++;
		S->dyncnt++;
		S->TIME += S->CYCLETIME;

		switch (S->riscv->P.EX.format)
		{
			case INSTR_R:
			{
				instr_r *tmp;

				tmp = (instr_r *)&S->riscv->P.EX.instr;
				(*(S->riscv->P.EX.fptr))(E, S, tmp->rs1, tmp->rs2, tmp->rd);
				break;
			}

			case INSTR_R_F:
			{
				instr_r *tmp;

				int saved_rm = fegetround();
				int rm = tmp->funct3 == 0b111 ? (S->riscv->FCSR & 0b11100000) >> 5 : tmp->funct3;
				switch(rm)
				{
					case 0b000: fesetround(FE_TONEAREST); break;
					case 0b001: fesetround(FE_TOWARDZERO); break;
					case 0b010: fesetround(FE_DOWNWARD); break;
					case 0b011: fesetround(FE_UPWARD); break;
					// This mode is not supported by C/x86.
					case 0b100: fesetround(FE_TONEAREST); break;
					default:
					{

					}
				}
				feclearexcept(FE_ALL_EXCEPT);

				tmp = (instr_r *)&S->riscv->P.EX.instr;
				(*(S->riscv->P.EX.fptr))(E, S, tmp->rs1, tmp->rs2, tmp->rd);

				fesetround(saved_rm);
				if(fetestexcept(FE_INEXACT)) S->riscv->FCSR |= 0b00001;
				if(fetestexcept(FE_UNDERFLOW)) S->riscv->FCSR |= 0b00010;
				if(fetestexcept(FE_OVERFLOW)) S->riscv->FCSR |= 0b00100;
				if(fetestexcept(FE_DIVBYZERO)) S->riscv->FCSR |= 0b01000;
				if(fetestexcept(FE_INVALID)) S->riscv->FCSR |= 0b10000;

				break;
			}

			case INSTR_R4:
			{
				instr_r4 *tmp;

				int saved_rm = fegetround();
				int rm = tmp->funct3 == 0b111 ? (S->riscv->FCSR & 0b11100000) >> 5 : tmp->funct3;
				switch(rm)
				{
					case 0b000: fesetround(FE_TONEAREST); break;
					case 0b001: fesetround(FE_TOWARDZERO); break;
					case 0b010: fesetround(FE_DOWNWARD); break;
					case 0b011: fesetround(FE_UPWARD); break;
					// This mode is not supported by C/x86.
					case 0b100: fesetround(FE_TONEAREST); break;
					default:
					{

					}
				}
				feclearexcept(FE_ALL_EXCEPT);

				tmp = (instr_r4 *)&S->riscv->P.EX.instr;
				(*(S->riscv->P.EX.fptr))(E, S, tmp->rs1, tmp->rs2, tmp->rs3, tmp->rd);

				fesetround(saved_rm);
				if(fetestexcept(FE_INEXACT)) S->riscv->FCSR |= 0b00001;
				if(fetestexcept(FE_UNDERFLOW)) S->riscv->FCSR |= 0b00010;
				if(fetestexcept(FE_OVERFLOW)) S->riscv->FCSR |= 0b00100;
				if(fetestexcept(FE_DIVBYZERO)) S->riscv->FCSR |= 0b01000;
				if(fetestexcept(FE_INVALID)) S->riscv->FCSR |= 0b10000;

				break;
			}

			case INSTR_I:
			{
				instr_i *tmp;

				tmp = (instr_i *)&S->riscv->P.EX.instr;
				(*(S->riscv->P.EX.fptr))(E, S, tmp->rs1, tmp->rd, tmp->imm0);
				break;
			}

			case INSTR_S:
			{
				instr_s *tmp;

				tmp = (instr_s *)&S->riscv->P.EX.instr;
				(*(S->riscv->P.EX.fptr))(E, S, tmp->rs1, tmp->rs2, tmp->imm0, tmp->imm5);
				break;
			}

			case INSTR_B:
			{
				instr_b *tmp;

				tmp = (instr_b *)&S->riscv->P.EX.instr;
				(*(S->riscv->P.EX.fptr))(E, S, tmp->rs1, tmp->rs2, tmp->imm1, tmp->imm5, tmp->imm11, tmp->imm12);
				break;
			}

			case INSTR_U:
			{
				instr_u *tmp;

				tmp = (instr_u *)&S->riscv->P.EX.instr;
				(*(S->riscv->P.EX.fptr))(E, S, tmp->rd, tmp->imm0);
				break;
			}

			case INSTR_J:
			{
				instr_j *tmp;

				tmp = (instr_j *)&S->riscv->P.EX.instr;
				(*(S->riscv->P.EX.fptr))(E, S, tmp->rd, tmp->imm1, tmp->imm11, tmp->imm12, tmp->imm20);
				break;
			}

			case INSTR_N:
			{
				(*(S->riscv->P.EX.fptr))(E, S);
				break;
			}

			default:
			{
				sfatal(E, S, "Unknown Instruction Type !!");
				break;
			}
		}

		if (SF_BITFLIP_ANALYSIS)
		{
			S->Cycletrans += bit_flips_32(tmpPC, S->PC);
			S->Cycletrans = 0;
		}

		if (S->pipeshow)
		{
			riscvdumppipe(E, S);
		}

		E->globaltimepsec = max(E->globaltimepsec, S->TIME) + S->CYCLETIME;
	}
	E->globaltimepsec = saved_globaltime;
	S->last_stepclks = i;

	return i;
}


void
riscvdumppipe(Engine *E, State *S)
{
	mprint(E, S, nodeinfo, "\nnode ID=%d, PC=0x" UHLONGFMT ", ICLK=" UVLONGFMT ", sleep?=%d\n",
		S->NODE_ID, S->PC, S->ICLK, S->sleep);

	mprint(E, S, nodeinfo, "EX: [%s]\n", riscv_opstrs[S->riscv->P.EX.op]);

	return;
}
