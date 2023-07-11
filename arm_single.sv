/* 
base code from:
  David_Harris@hmc.edu and Sarah_Harris@hmc.edu 25 June 2013
  Single-cycle implementation of a subset of ARMv4

original instructions set: ADD, SUB, AND, ORR, LDR, STR e B.
registers: 16 32-bit

extended instructions: MOV, CMP, TST, EOR, LDRB, STRB e BL.

Data-processing instructions
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

Load/Store instructions
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

Branch instruction (PC <= PC + offset, PC holds 8 bytes past Branch Instr)
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

Other:
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
*/


module testbench();
  parameter real clk_frequency = 1e6; // 1 Mhz.
  parameter real clk_period = (1e12/clk_frequency); // 1e12 correspound to 1 sec in real datatype scale.
  
  logic clk = 0;
  logic reset;

  // instantiate arm device to be tested
  logic [31:0] WriteData, DataAdr;
  logic        MemWrite;
  top dut(clk, reset, WriteData, DataAdr, MemWrite, twarn);

  // initialize test
  initial
    begin
      reset <= 1; #(clk_period/2); reset <= 0;
    end

  // stop simulation when warn from time io device 
  always @ (twarn)
    if (twarn) begin
      $stop;
    end

  // generate clock to sequence tests
  always begin
    #(clk_period/2) clk = ~clk;
  end
endmodule

module top(input  logic        clk, reset, 
           output logic [31:0] WriteData, DataAdr, 
           output logic        MemWrite, twarn);
  /*
    TIMER i/o DEVICE
    TIMESTAMP   (in micro seconds) -> RAM[60] => "0xf0" (LDR)
    TIMER INPUT (in micro seconds) -> RAM[59] => "0xec" (STR)
    TIMER OUTPUT FLAGS -------------> RAM[58] => "0xe8" (LDRB)
      #0 RAM[58][0]   warn (return 1 if a set timer has been reached)
      
    TIMER CONTROL FLAGS ------------> RAM[57] => "0xe4" (STRB)
      #0 RAM[57][7:0]   reset_timer
      #1 RAM[57][15:8]  enable_clock 
      #2 RAM[57][23:16] reset_timestamp
      #3 RAM[57][31:24] unimplemented
  */
  logic [63:0] timestamp;
  logic [63:0] timer_input;
  logic reset_timer, enable_clock, reset_timestamp;
  logic [2:0]  tctrl;
  assign {reset_timer, enable_clock, reset_timestamp} = tctrl;
  timer timer(clk, enable_clock, reset_timestamp, reset_timer, 
              timer_input, twarn, timestamp);

  // instantiate processor and memories
  logic [31:0] PC, Instr, ReadData;
  logic        ByteFlag;
  
  arm arm(clk, reset, PC, Instr, MemWrite, ByteFlag,
          DataAdr, WriteData, ReadData);
  imem imem(PC, Instr);
  dmem dmem(clk, MemWrite, ByteFlag, DataAdr, WriteData, ReadData, 
            tctrl, timer_input, timestamp, twarn);
endmodule

module dmem(input  logic        clk, we, ByteFlag,
            input  logic [31:0] a, wd,
            output logic [31:0] rd, 
            output logic [2:0]  tctrl,
            output logic [63:0] timer_input,
            input  logic [63:0] timestamp,
            input  logic        twarn);
  
  logic [31:0] RAM[63:0];

  // tctrl[2:0]
  logic reset_timer, enable_clock, reset_timestamp;

  // READ
  always_comb begin
    if (ByteFlag)
      case(a % 3'b100) 
        2'b00:  begin // already aligned
                  assign rd = {24'b0, RAM[a[31:2]][7:0]}; 
                end
        2'b01:  begin // 1-byte shift
                  assign rd = {24'b0, RAM[a[31:2]][15:8]}; 
                end
        2'b10:  begin // 2-byte shift
                  assign rd = {24'b0, RAM[a[31:2]][23:16]}; 
                end 
        2'b11:  begin // 3-byte shift
                  assign rd = {24'b0, RAM[a[31:2]][31:24]}; 
                end
      endcase
    else assign rd = RAM[a[31:2]];

    // Timer Driver
    timer_input     = {32'b0, RAM[59]};
    reset_timer     = RAM[57][7:0]   | 8'b0;
    enable_clock    = RAM[57][15:8]  | 8'b0;
    reset_timestamp = RAM[57][23:16] | 8'b0;
    tctrl = {reset_timer, enable_clock, reset_timestamp};
  end

// WRITE
always_ff @(posedge clk) begin
  if (we)
    if (ByteFlag)
      case(a % 3'b100) 
        2'b00:  begin // already aligned
                  RAM[a[31:2]] <= {RAM[a[31:2]][31:8], wd[7:0]};
                end
        2'b01:  begin // 1-byte shift
                  RAM[a[31:2]] <= {RAM[a[31:2]][31:16], wd[7:0], RAM[a[31:2]][7:0]};
                end
        2'b10:  begin // 2-byte shift
                  RAM[a[31:2]] <= {RAM[a[31:2]][31:24], wd[7:0], RAM[a[31:2]][15:0]};
                end 
        2'b11:  begin // 3-byte shift
                  RAM[a[31:2]] <= {wd[7:0], RAM[a[31:2]][23:0]};
                end
      endcase
    else RAM[a[31:2]] <= wd;

  // Timer Driver
  RAM[60] <= timestamp[31:0];
  RAM[58] <= {24'b0, {7'b0, twarn}};
end
endmodule

module imem(input  logic [31:0] a,
            output logic [31:0] rd);

  logic [31:0] RAM[63:0];

  initial
      $readmemh("memfile.dat",RAM);

  assign rd = RAM[a[31:2]]; // word aligned
endmodule

module arm(input  logic        clk, reset,
           output logic [31:0] PC,
           input  logic [31:0] Instr,
           output logic        MemWrite, ByteFlag,
           output logic [31:0] ALUResult, WriteData,
           input  logic [31:0] ReadData);

  logic [3:0] ALUFlags;
  logic       RegWrite, MovFlag, LinkFlag,
              ALUSrc, MemtoReg, PCSrc;
  logic [1:0] RegSrc, ImmSrc;
  logic [2:0] ALUControl; // added 1 bit for expand ALU OPs

  controller c(clk, reset, Instr[31:12], ALUFlags, 
               RegSrc, RegWrite, ImmSrc, 
               ALUSrc, ALUControl,
               MemWrite, ByteFlag, MemtoReg, MovFlag, LinkFlag, PCSrc);
  datapath dp(clk, reset, 
              RegSrc, RegWrite, ImmSrc,
              ALUSrc, ALUControl,
              MemtoReg, PCSrc, MovFlag, LinkFlag,
              ALUFlags, PC, Instr,
              ALUResult, WriteData, ReadData);
endmodule

module controller(input  logic         clk, reset,
                  input  logic [31:12] Instr,
                  input  logic [3:0]   ALUFlags,
                  output logic [1:0]   RegSrc,
                  output logic         RegWrite,
                  output logic [1:0]   ImmSrc,
                  output logic         ALUSrc, 
                  output logic [2:0]   ALUControl,
                  output logic         MemWrite, ByteFlag, MemtoReg,
                  output logic         MovFlag, LinkFlag,
                  output logic         PCSrc);

  logic [1:0] FlagW;
  logic       PCS, RegW, MemW, MovF, LinkF;
  
  decoder dec(Instr[27:26], Instr[25:20], Instr[15:12],
              FlagW, PCS, RegW, MemW, ByteFlag, 
              MemtoReg, ALUSrc, MovF, LinkF, ImmSrc, RegSrc, ALUControl);
  condlogic cl(clk, reset, Instr[31:28], ALUFlags,
               FlagW, PCS, RegW, MemW, MovF, LinkF,
               PCSrc, RegWrite, MemWrite, MovFlag, LinkFlag);
endmodule

module decoder(input  logic [1:0] Op,
               input  logic [5:0] Funct,
               input  logic [3:0] Rd,
               output logic [1:0] FlagW,
               output logic       PCS, RegW, MemW, ByteFlag,
               output logic       MemtoReg, ALUSrc, MovF, 
               output logic       LinkF,
               output logic [1:0] ImmSrc, RegSrc, 
               output logic [2:0] ALUControl);

  logic [9:0] controls;
  logic       Branch, ALUOp;

  assign {RegSrc, ImmSrc, ALUSrc, MemtoReg, 
      RegW, MemW, Branch, ALUOp} = controls; 
    
  // PC Logic
  assign PCS  = ((Rd == 4'b1111) & RegW) | Branch; 

  // Main Decoder
  always_comb
  	case(Op)
  	  2'b00:  begin
                if (Funct[5])  
                  controls = 10'b0000101001;  // Data processing immediate
                else  
                  controls = 10'b0000001001;  // Data processing register

                if (Funct[4:1] == 4'b1010 | Funct[4:1] == 4'b1000)  // CMP and TST
                  controls[3] = 1'b0; // disable RegW

                // "dont care" for other OPs Flags
                ByteFlag = 1'bx;
                LinkF = 1'bx;
              end

  	  2'b01:  begin
                if (Funct[0])  
                  controls = 10'b0001111000; // LDR
                else           
                  controls = 10'b1001110100; // STR
                if (Funct[2]) // LDRB and STRB
                  ByteFlag = 1'b1;
                else
                  ByteFlag = 1'b0;

                LinkF = 1'bx;
              end

  	  2'b10:  begin
                controls = 10'b0110100010;
                if (Funct[4])  // BL
                  LinkF = 1'b1;
                else  // B
                  LinkF = 1'b0;

                ByteFlag = 1'bx;
              end   

  	  default: begin  
                  controls = 10'bx; // Unimplemented 
                  ByteFlag = 1'bx;  
                  LinkF = 1'bx; 
               end
  	endcase
 
  // ALU Decoder             
  always_comb
    if (ALUOp) begin  // which DP Instr?
      if (Funct[4:1] ==  4'b1101) begin  // MOV
            ALUControl = 3'bx;
            MovF = 1'b1;  
          end
      else begin
        MovF = 1'b0;
        case(Funct[4:1]) 
            4'b0100: ALUControl = 3'b000;  // ADD
            4'b0010: ALUControl = 3'b001;  // SUB
            4'b0000: ALUControl = 3'b010;  // AND
            4'b1100: ALUControl = 3'b011;  // ORR
            4'b0001: ALUControl = 3'b111;  // EOR 
            4'b1010: ALUControl = 3'b001;  // CMP
            4'b1000: ALUControl = 3'b010;  // TST
            default:
              ALUControl = 3'bx;
        endcase
      end

      // update flags if S bit is set 
	    // (C & V only updated for arith instructions)
      FlagW[1]      = Funct[0]; // FlagW[1] = S-bit
	    // FlagW[0] = S-bit & (ADD | TST | SUB | COMP)
      FlagW[0]      = Funct[0] & (ALUControl == 3'b000 | ALUControl == 3'b001);
      
    end else begin
      ALUControl = 3'b000; // add for non-DP instructions
      FlagW      = 3'b000; // don't update Flags
    end
endmodule

module condlogic(input  logic       clk, reset,
                 input  logic [3:0] Cond,
                 input  logic [3:0] ALUFlags,
                 input  logic [1:0] FlagW,
                 input  logic       PCS, RegW, MemW, MovF, LinkF,
                 output logic       PCSrc, RegWrite, MemWrite, 
                 output logic       MovFlag, LinkFlag);
                 
  logic [1:0] FlagWrite;
  logic [3:0] Flags;
  logic       CondEx;

  flopenr #(2)flagreg1(clk, reset, FlagWrite[1], 
                       ALUFlags[3:2], Flags[3:2]);
  flopenr #(2)flagreg0(clk, reset, FlagWrite[0], 
                       ALUFlags[1:0], Flags[1:0]);

  // write controls are conditional
  condcheck cc(Cond, Flags, CondEx);
  assign FlagWrite = FlagW & {2{CondEx}};
  assign RegWrite  = RegW  & CondEx;
  assign MemWrite  = MemW  & CondEx;
  assign PCSrc     = PCS   & CondEx;
  assign MovFlag   = MovF  & CondEx;
  assign LinkFlag  = LinkF & CondEx;
endmodule    

module condcheck(input  logic [3:0] Cond,
                 input  logic [3:0] Flags,
                 output logic       CondEx);
  
  logic neg, zero, carry, overflow, ge;
  
  assign {neg, zero, carry, overflow} = Flags;
  assign ge = (neg == overflow);
  
  always_comb
    case(Cond)
      4'b0000: CondEx = zero;             // EQ
      4'b0001: CondEx = ~zero;            // NE
      4'b0010: CondEx = carry;            // CS
      4'b0011: CondEx = ~carry;           // CC
      4'b0100: CondEx = neg;              // MI
      4'b0101: CondEx = ~neg;             // PL
      4'b0110: CondEx = overflow;         // VS
      4'b0111: CondEx = ~overflow;        // VC
      4'b1000: CondEx = carry & ~zero;    // HI
      4'b1001: CondEx = ~(carry & ~zero); // LS
      4'b1010: CondEx = ge;               // GE
      4'b1011: CondEx = ~ge;              // LT
      4'b1100: CondEx = ~zero & ge;       // GT
      4'b1101: CondEx = ~(~zero & ge);    // LE
      4'b1110: CondEx = 1'b1;             // Always
      default: CondEx = 1'bx;             // undefined
    endcase
endmodule

module datapath(input  logic        clk, reset,
                input  logic [1:0]  RegSrc,
                input  logic        RegWrite,
                input  logic [1:0]  ImmSrc,
                input  logic        ALUSrc,
                input  logic [2:0]  ALUControl,
                input  logic        MemtoReg,
                input  logic        PCSrc,
                input  logic        MovFlag,
                input  logic        LinkFlag,
                output logic [3:0]  ALUFlags,
                output logic [31:0] PC,
                input  logic [31:0] Instr,
                output logic [31:0] ALUResult, WriteData,
                input  logic [31:0] ReadData);

  logic [31:0] PCNext, PCPlus4, PCPlus8;
  logic [31:0] ExtImm, SrcA, SrcB, Result, MovORAluResult;
  logic [3:0]  RA1, RA2;

  // next PC logic
  mux2 #(32)  pcmux(PCPlus4, Result, PCSrc, PCNext);
  flopr #(32) pcreg(clk, reset, PCNext, PC);
  adder #(32) pcadd1(PC, 32'b100, PCPlus4);
  adder #(32) pcadd2(PCPlus4, 32'b100, PCPlus8);

  // register file logic
  mux2 #(4)   ra1mux(Instr[19:16], 4'b1111, RegSrc[0], RA1);
  mux2 #(4)   ra2mux(Instr[3:0], Instr[15:12], RegSrc[1], RA2);
  regfile     rf(clk, RegWrite, LinkFlag, RA1, RA2,
                 Instr[15:12], Result, PCPlus8, 
                 SrcA, WriteData); 
  mux2 #(32)  movmux(ALUResult, SrcB, MovFlag, MovORAluResult);
  mux2 #(32)  resmux(MovORAluResult, ReadData, MemtoReg, Result);
  extend      ext(Instr[23:0], ImmSrc, ExtImm);

  // ALU logic
  mux2 #(32)  srcbmux(WriteData, ExtImm, ALUSrc, SrcB);
  alu         alu(SrcA, SrcB, ALUControl, ALUResult, ALUFlags);
endmodule

module regfile(input  logic        clk, 
               input  logic        we3, lflag,
               input  logic [3:0]  ra1, ra2, wa3, 
               input  logic [31:0] wd3, r15,
               output logic [31:0] rd1, rd2);

  logic [31:0] rf[14:0];
  /*
    three ported register file
    read two ports combinationally
    write third port on rising edge of clock
    register 15 reads PC+8 instead
    lflag (Branch Linked) -> register 14 reads PC+4
  */

  always_ff @(posedge clk)
    if (lflag) rf[14] <= r15 - 3'b100;

  always_ff @(posedge clk)
    if (we3) rf[wa3] <= wd3;	

  assign rd1 = (ra1 == 4'b1111) ? r15 : rf[ra1];
  assign rd2 = (ra2 == 4'b1111) ? r15 : rf[ra2];
endmodule

module extend(input  logic [23:0] Instr,
              input  logic [1:0]  ImmSrc,
              output logic [31:0] ExtImm);
 
  always_comb
    case(ImmSrc) 
      2'b00:   ExtImm = {24'b0, Instr[7:0]};  // 8-bit unsigned immediate
      2'b01:   ExtImm = {20'b0, Instr[11:0]}; // 12-bit unsigned immediate (LDR/STR)
      2'b10:   ExtImm = {{6{Instr[23]}}, Instr[23:0], 2'b00}; // 24-bit two's complement shifted branch 
      default: ExtImm = 32'bx; // undefined
    endcase             
endmodule

module adder #(parameter WIDTH=8)
              (input  logic [WIDTH-1:0] a, b,
               output logic [WIDTH-1:0] y);
             
  assign y = a + b;
endmodule

module flopenr #(parameter WIDTH = 8)
                (input  logic             clk, reset, en,
                 input  logic [WIDTH-1:0] d, 
                 output logic [WIDTH-1:0] q);

  always_ff @(posedge clk, posedge reset)
    if (reset)   q <= 0;
    else if (en) q <= d;
endmodule

module flopr #(parameter WIDTH = 8)
              (input  logic             clk, reset,
               input  logic [WIDTH-1:0] d, 
               output logic [WIDTH-1:0] q);

  always_ff @(posedge clk, posedge reset)
    if (reset) q <= 0;
    else       q <= d;
endmodule

module mux2 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, 
              input  logic             s, 
              output logic [WIDTH-1:0] y);

  assign y = s ? d1 : d0; 
endmodule


module alu(input  logic [31:0] a, b,
           input  logic [2:0]  ALUControl,
           output logic [31:0] Result,
           output logic [3:0]  ALUFlags);

  logic        neg, zero, carry, overflow;
  logic [31:0] condinvb;
  logic [32:0] sum;

  assign condinvb = ALUControl[0] ? ~b : b;
  assign sum = a + condinvb + ALUControl[0];

  always_comb
    casex (ALUControl[2:0]) // added 1 bit for expand ALU OPs
      3'b00?: Result = sum;
      3'b010: Result = a & b; // AND
      3'b011: Result = a | b; // OR
      3'b111: Result = a ^ b; // EOR
    endcase

  assign neg      = Result[31];
  assign zero     = (Result == 32'b0);
  assign carry    = (ALUControl[1] == 1'b0) & sum[32];
  assign overflow = (ALUControl[1] == 1'b0) & 
                    ~(a[31] ^ b[31] ^ ALUControl[0]) & 
                    (a[31] ^ sum[31]); 
  assign ALUFlags    = {neg, zero, carry, overflow};
endmodule
