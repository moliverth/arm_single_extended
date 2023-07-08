/*
THIS CODE COUNT TO 3 IN R1 AND RETURN 1 IN R0
* USING "bl" TO SAVE POSITION OF  "b end"

HEX:
e3a00000
e3a01000
e3510003
01a0f00e
e2811001
ebfffffb
ea000000
e3a00001
*/

start:
    mov r0, #0
    mov r1, #0

count:
    cmp r1, #3
    moveq pc, lr
    add r1, r1, #1
    bl count
    b end

end:
    mov r0, #1

