#include <string.h>
#include "sf.h"
#include "mextern.h"

void
riscvdumpregs(Engine *E, State *S)
{
	int i;
	uint32_t bits;

	for (i = 0; i < 32; i++)
	{
		mprint(E, S, nodeinfo, "R%-2d\t\t", i);
		mbitprint(E, S, 32, S->riscv->R[i]);
		mprint(E, S, nodeinfo, "  [0x%08lx]\n", S->riscv->R[i]);
	}

	for (i = 0; i < 32; i++)
	{
		mprint(E, S, nodeinfo, "F%-2d\t\t", i);
		memcpy(&bits, &S->riscv->F[i], 4);
		mbitprint(E, S, 32, bits);
		mprint(E, S, nodeinfo, "  [%10g]\n", S->riscv->F[i]);
	}
	mprint(E, S, nodeinfo, "FCSR\t\t");
	memcpy(&bits, &S->riscv->FCSR, 4);
	mbitprint(E, S, 32, bits);
	mprint(E, S, nodeinfo, "  [0x%08lx]\n", S->riscv->FCSR);

	return;
}

State *
riscvnewstate(Engine *E, double xloc, double yloc, double zloc, char *trajfilename)
{
    State   *S = superHnewstate(E, xloc, yloc, zloc, trajfilename);

    S->riscv = (RiscvState *) mcalloc(E, 1, sizeof(RiscvState), "S->riscv");
    S->dumpregs = riscvdumpregs;
	S->dumppipe = riscvdumppipe;
	S->endian = Little;
	S->machinetype = MACHINE_RISCV;

    if (S->riscv == NULL)
	{
		mexit(E, "Failed to allocate memory for S->riscv.", -1);
	}
    
    S->step = riscvstep;

    return S;
}