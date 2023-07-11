/*
e3a00000
e3a01001
e3a030e4
e3a040ec
e3a050e8
e3a06064
e5830000
e5840000
e5c31001
ea00000a
e5948000
e1580006
15846000
1a000002
e5c31000
e5c30000
eaffffff
e5d58000
e3580001
01a0f00e
eafffffb
ebfffff3
e28aa001
eafffffc
*/

.global _start
_start:
start:
	mov  r0, #0
	mov  r1, #1
	
	mov  r3, #0xe4 // timer controls i/o register
	mov  r4, #0xec // time input
	mov  r5, #0xe8 // alarm warn output
	mov  r6, #100  // 100 ns
	
	str	 r0, [r3] // set default values
	str  r0, [r4]
	strb r1, [r3, #1] // enable timer
	b loop
	
wait100:
	ldr   r8, [r4]
	cmp   r8, r6   // if (time input != 100):
	strne r6, [r4] // initate the alarm
	bne sleep
	strb  r1, [r3] // else:
	strb  r0, [r3] // pulse in timer reset
	b sleep
	
sleep:
	ldrb r8, [r5]
	cmp	 r8, #1
	moveq pc, lr   // return when time reach
	b sleep

loop:
	bl wait100
	add r10, r10, #1
	b loop