# arm_single_extended

## 🚀 Util Links:

* https://iitd-plos.github.io/col718/ref/arm-instructionset.pdf
* https://developer.arm.com/documentation/dui0489/i?lang=en
* https://fpgatutorial.com/systemverilog-operators/
* https://www.hdlworks.com/hdl_corner/verilog_ref/items/SystemRealConversionFuncs.htm
* https://cpulator.01xz.net/?sys=arm
* https://armconverter.com/

### ✒️ Thanks
```
base code from:
  David_Harris@hmc.edu and Sarah_Harris@hmc.edu 25 June 2013
  Single-cycle implementation of a subset of ARMv4

original instructions set: ADD, SUB, AND, ORR, LDR, STR e B.
registers: 16 32-bit

extended instructions: MOV, CMP, TST, EOR, LDRB, STRB e BL.
```
## Data-processing instructions
```
  ADD, SUB, AND, ORR, MOV, CMP, TST, EOR.
  INSTR<cond><S> rd, rn, #immediate
  INSTR<cond><S> rd, rn, rm
    rd <- rn INSTR rm	      if (S) Update Status Flags
    rd <- rn INSTR immediate	if (S) Update Status Flags
  Instr[31:28] = cond
  Instr[27:26] = op = 00
  Instr[25:20] = funct
      [25]:    1 for immediate, 0 for register
      [24:21]: 0100 (ADD) / 0010 (SUB) / 0000 (AND) / 1100 (ORR)
      [20]:    S (1 = update CPSR status Flags)
  Instr[19:16] = rn
  Instr[15:12] = rd
  Instr[11:8]  = 0000
  Instr[7:0]   = imm8   (for #immediate type) / {0000,rm} (for register type)
```
## Load/Store instructions
```
  LDR, LDRB, STR, STRB
  INSTR rd, [rn, #offset]

  LDR: rd <- Mem[rn+offset]
  STR: Mem[rn+offset] <- rd

  Instr[31:28] = cond
  Instr[27:26] = op = 01 
  Instr[25:20] = funct
    [25]:    0 (A)
    [24:21]: 1100 (P/U/B/W)
    [20]:    L (1 for LDR, 0 for STR)
  Instr[19:16] = rn
  Instr[15:12] = rd
  Instr[11:0]  = imm12 (zero extended)  
```
## Branch instruction 
```
  (PC <= PC + offset, PC holds 8 bytes past Branch Instr)
  B, BL.
  B  target: PC <- PC + 8 + imm24 << 2
  BL (link): also LR <- PC+4

  Instr[31:28] = cond
  Instr[27:25] = op = 10
  Instr[25:24] = funct
    [25]: 1 (Branch)
    [24]: 0 (link)
  Instr[23:0]  = imm24 (sign extend, shift left 2)
  Note: no Branch delay slot on ARM
```
## Other:
```
  R15 reads as PC+8
  
  Conditional Encoding:
    cond  Meaning                       Flag
    0000  Equal                         Z = 1
    0001  Not Equal                     Z = 0
    0010  Carry Set                     C = 1
    0011  Carry Clear                   C = 0
    0100  Minus                         N = 1
    0101  Plus                          N = 0
    0110  Overflow                      V = 1
    0111  No Overflow                   V = 0
    1000  Unsigned Higher               C = 1 & Z = 0
    1001  Unsigned Lower/Same           C = 0 | Z = 1
    1010  Signed greater/equal          N = V
    1011  Signed less                   N != V
    1100  Signed greater                N = V & Z = 0
    1101  Signed less/equal             N != V | Z = 1
    1110  Always                        any
```
## Timer I/O Device
```
  TIMESTAMP   (in micro seconds) -> RAM[60] => "0xf0" (LDR)
  TIMER INPUT (in micro seconds) -> RAM[59] => "0xec" (STR)
  TIMER OUTPUT FLAGS -------------> RAM[58] => "0xe8" (LDRB)
    #0 RAM[58][0]   warn (return 1 if a set timer has been reached)
    
  TIMER CONTROL FLAGS ------------> RAM[57] => "0xe4" (STRB)
    #0 RAM[57][7:0]   reset_timer
    #1 RAM[57][15:8]  enable_clock 
    #2 RAM[57][23:16] reset_timestamp
    #3 RAM[57][31:24] unimplemented
```
