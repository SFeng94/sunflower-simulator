.globl _start
.align	4

_start:

init:
	j			main

.globl my_sqrt

my_sqrt:
	fmv.s.x fa5, a0
	fsqrt.s fa5, fa5
	fmv.x.s a0, fa5
	ret
