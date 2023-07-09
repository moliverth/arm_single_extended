/*
THIS CODE RETURN 1 IN r0 WHEN r1 AND r2 EQUAL ZERO
INITIATE r1 AND r2 WITH VALUE 3, SUBTRACT GREATER OR r1

e3a00000
e3a01003
e3a02003
e3a03000
eaffffff

e0213002
e3530000
1a000002
01110002
0a000008
eaffffff

e1510002
ab000001
bb000002
eafffff5

e2411001
e1a0f00e

e2422001
e1a0f00e

e3a00001
*/

.global _start
_start:
    mov r0, #0
    mov r1, #3
    mov r2, #3
    mov r3, #0
    b main

main:
    eor r3, r1, r2
    cmp r3, #0    	// test if (r1 == r2)
	bne subgreater	// if not equal branch
	tsteq r1, r2
    beq end			// if both are zero branch
	b subgreater

subgreater:
	cmp r1, r2
    blge sub1		// subtract r1 if r1 >= r2
    bllt sub2
	b main

sub1:
    sub r1, r1, #1
    mov pc, lr

sub2:
    sub r2, r2, #1
    mov pc, lr

end:
    mov r0, #1
