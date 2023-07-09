/*
THIS CODE WRITE "ddccbbaa" in RAM[2]
and invert to "aabbccdd" in RAM[3] using LDRB and STRB

HEX:
E3A00008
E3A010AA
E3A020BB
E3A030CC
E3A040DD

E5801000
E5C02001
E5C03002
E5C04003

E5905000
E5D06000
E5D07001
E5D08002
E5D09003

E5809004
E5C08005
E5C07006
E5C06007
*/

.global _start
_start:
	mov r0, #0x8
	mov r1, #0xaa
	mov r2, #0xbb
	mov r3, #0xcc
	mov r4, #0xdd
	
	str  r1, [r0]
	strb r2, [r0, #1]
	strb r3, [r0, #2]
	strb r4, [r0, #3]
    
    ldr  r5, [r0]
	ldrb r6, [r1]
	ldrb r7, [r1, #1]
	ldrb r8, [r1, #2]
	ldrb r9, [r1, #3]
	
	strb r6, [r1, #3]
	strb r7, [r1, #2]
	strb r8, [r1, #1]
	strb r9, [r1]