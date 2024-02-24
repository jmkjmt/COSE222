module signextmux(
	 input [23:0] in,
	 input [1:0] select,
	 output reg[31:0] extVal); //signextmux module 선언 및 input, output 설정
 
	integer i;
	always @ (*) begin //코드 내 신호가 바뀔 때마다 실행
	case(select)
	0 : begin
		extVal[7:0]=in[7:0];
		for(i=8;i<32;i=i+1) extVal[i]=in[7];
		end //select == 0이면 in[7:0]을 signed - extend을 하여 extVal에 저장
	1 : begin
		extVal[11:0]=in[11:0];
		for(i=12;i<32;i=i+1) extVal[i]='b0;
		end //select == 1이면 in[11:0]을 unsigned - extend을 하여 extVal에 저장
	2 : begin
		extVal[1:0]=2'b00;
		extVal[25:2]=in[23:0];
		for(i=26;i<32;i=i+1) extVal[i]=in[23];
		end //select == 0이면 in의 값을 shifting two lefts를 하고 signed - extend을 하여 extVal에 저장
	default : extVal='h00000000; //위 case가 아니라면, extVal은 00000000으로 초기화
	endcase
	end
endmodule // signextmux module 종료 
module register_1bit(
	input regin,
	input clk,
	input write,
	input reset,
	output reg regout); //register_1bit module 선언 및 input, output 설정
	
	always @ (posedge clk,posedge reset)//clk이 positive edge, reset이 positive edge일 때때 다음 코드 실행
	if(reset) begin
		regout<=0; //reset이 positive edge라면, regout 신호에 0을 할당
	end // 
	else if(write) begin
		regout<=regin;
	end //reset이 not positive edge이면서 write 신호가 1이라면, regout 신호에 regin 할당
	
endmodule // register_1bit module 종료
module register( 
	input[31:0] regin,
	input clk,
	input write,
	input reset,
	output reg[31:0] regout); //register module 선언 및 input, output 설정
	
	always @ (posedge clk) //clk이 positive edge일 때 다음 코드 실행
	if(reset) 
		regout<='h0000; // reset 신호가 1이라면, regout에 000
	else if(write)
		regout<=regin; //reset 신호가 0이고 write 신호가 1이라면, regout 신호에 regin 할당
endmodule //register module 종료

module decoder_4to16(
	input[3:0] in,
	output reg[15:0] out); //decoder_4to16 module 선언 및 input, output 설정
	
	always @ (*) //코드 내 신호가 바뀔 때마다 실행
	case(in)
	0 : out='h0001;
	1 : out='h0002;
	2 : out='h0004;
	3 : out='h0008;
	4 : out='h0010;
	5 : out='h0020;
	6 : out='h0040;
	7 : out='h0080;
	8 : out='h0100;
	9 : out='h0200;
   10 : out='h0400;
   11 : out='h0800;
   12 : out='h1000;
   13 : out='h2000;
   14 : out='h4000;
   15 : out='h8000;
    endcase //in의 값에 따라 out에 16bit 값을 할당
endmodule // decoder_4to16 module 종료

module registerfile(
	input[3:0] reg1,
	input[3:0] reg2,
	input[3:0] regdst,
	input[31:0] regsrc,
	input clk,
	input reset,
	input we, // write 할지 말지
	output[31:0] out1,
	output[31:0] out2,
	output[31:0] pc); //registerfile module 선언 및 input, output 설정
	
	wire[15:0] write; //몇번 레지스터를 사용할지
	wire[31:0] registers [15:0]; //32비트 레지스터 16개

	decoder_4to16 selecter(.in (regdst), .out (write)); //base2-4bit regdst 를 code로 변환
	//regsrc로 regdst에 해당되는 register들을 업데이트
	register register0 (.regin (regsrc), .write (write[0]&we), .clk (clk), .reset (reset), .regout (registers[0]));
	//register0의 regin 신호에 regsrc 연결, write 신호에 write[0]과 we의 AND연산 결과 연결, clk 신호에 clk 연결, reset 신호에 reset 연결, regout 신호에 registers[0] 연결
	register register1 (.regin (regsrc), .write (write[1]&we), .clk (clk), .reset (reset), .regout (registers[1]));
	//register1의 regin 신호에 regsrc 연결, write 신호에 write[1]과 we의 AND연산 결과 연결, clk 신호에 clk 연결, reset 신호에 reset 연결, regout 신호에 registers[1] 연결
	register register2 (.regin (regsrc), .write (write[2]&we), .clk (clk), .reset (reset), .regout (registers[2]));
	//register2의 regin 신호에 regsrc 연결, write 신호에 write[2]과 we의 AND연산 결과 연결, clk 신호에 clk 연결, reset 신호에 reset 연결, regout 신호에 registers[2] 연결
	register register3 (.regin (regsrc), .write (write[3]&we), .clk (clk), .reset (reset), .regout (registers[3]));
	//register3의 regin 신호에 regsrc 연결, write 신호에 write[3]과 we의 AND연산 결과 연결, clk 신호에 clk 연결, reset 신호에 reset 연결, regout 신호에 registers[3] 연결
	register register4 (.regin (regsrc), .write (write[4]&we), .clk (clk), .reset (reset), .regout (registers[4]));
	//register4의 regin 신호에 regsrc 연결, write 신호에 write[4]과 we의 AND연산 결과 연결, clk 신호에 clk 연결, reset 신호에 reset 연결, regout 신호에 registers[4] 연결
	register register5 (.regin (regsrc), .write (write[5]&we), .clk (clk), .reset (reset), .regout (registers[5]));
	//register5의 regin 신호에 regsrc 연결, write 신호에 write[5]과 we의 AND연산 결과 연결, clk 신호에 clk 연결, reset 신호에 reset 연결, regout 신호에 registers[5] 연결
	register register6 (.regin (regsrc), .write (write[6]&we), .clk (clk), .reset (reset), .regout (registers[6]));
	//register6의 regin 신호에 regsrc 연결, write 신호에 write[6]과 we의 AND연산 결과 연결, clk 신호에 clk 연결, reset 신호에 reset 연결, regout 신호에 registers[6] 연결
	register register7 (.regin (regsrc), .write (write[7]&we), .clk (clk), .reset (reset), .regout (registers[7]));
	//register7의 regin 신호에 regsrc 연결, write 신호에 write[7]과 we의 AND연산 결과 연결, clk 신호에 clk 연결, reset 신호에 reset 연결, regout 신호에 registers[7] 연결
	register register8 (.regin (regsrc), .write (write[8]&we), .clk (clk), .reset (reset), .regout (registers[8]));
	//register8의 regin 신호에 regsrc 연결, write 신호에 write[8]과 we의 AND연산 결과 연결, clk 신호에 clk 연결, reset 신호에 reset 연결, regout 신호에 registers[8] 연결
	register register9 (.regin (regsrc), .write (write[9]&we), .clk (clk), .reset (reset), .regout (registers[9]));
	//register9의 regin 신호에 regsrc 연결, write 신호에 write[9]과 we의 AND연산 결과 연결, clk 신호에 clk 연결, reset 신호에 reset 연결, regout 신호에 registers[9] 연결
	register register10 (.regin (regsrc), .write (write[10]&we), .clk (clk), .reset (reset), .regout (registers[10]));
	//register10의 regin 신호에 regsrc 연결, write 신호에 write[10]과 we의 AND연산 결과 연결, clk 신호에 clk 연결, reset 신호에 reset 연결, regout 신호에 registers[10] 연결
	register register11 (.regin (regsrc), .write (write[11]&we), .clk (clk), .reset (reset), .regout (registers[11]));
	//register11의 regin 신호에 regsrc 연결, write 신호에 write[11]]과 we의 AND연산 결과 연결, clk 신호에 clk 연결, reset 신호에 reset 연결, regout 신호에 registers[11] 연결
	register register12 (.regin (regsrc), .write (write[12]&we), .clk (clk), .reset (reset), .regout (registers[12]));
	//register12의 regin 신호에 regsrc 연결, write 신호에 write[12]과 we의 AND연산 결과 연결, clk 신호에 clk 연결, reset 신호에 reset 연결, regout 신호에 registers[12] 연결
	register register13 (.regin (regsrc), .write (write[13]&we), .clk (clk), .reset (reset), .regout (registers[13]));
	//register13의 regin 신호에 regsrc 연결, write 신호에 write[13]과 we의 AND연산 결과 연결, clk 신호에 clk 연결, reset 신호에 reset 연결, regout 신호에 registers[13] 연결
	register register14 (.regin (regsrc), .write (write[14]&we), .clk (clk), .reset (reset), .regout (registers[14]));
	//register14의 regin 신호에 regsrc 연결, write 신호에 write[14]과 we의 AND연산 결과 연결, clk 신호에 clk 연결, reset 신호에 reset 연결, regout 신호에 registers[14] 연결
	register register15 (.regin (regsrc), .write (write[15]&we), .clk (clk), .reset (reset), .regout (registers[15]));
	//register15의 regin 신호에 regsrc 연결, write 신호에 write[15]과 we의 AND연산 결과 연결, clk 신호에 clk 연결, reset 신호에 reset 연결, regout 신호에 registers[15] 연결

	assign out1=registers[reg1]; //out1에 registers[reg1] 신호 할당
	assign out2=registers[reg2]; //out2에 registers[reg2] 신호 할당
	assign pc=registers[15]; //pc에 registers[15] 할당

endmodule //registerfile module 종료
  
module armreduced(
	input clk,
	input reset,
	output reg[31:0] pc,
	input[31:0] inst,
	input nIRQ,
	output [3:0] be,
	output[31:0] memaddr,
	output memwrite,
	output memread,
	output[31:0] writedata,
	input[31:0] readdata); //armreduced module 선언 및 input, output 설정
	
	//signals
  wire IRwrite,regwrite,NZCVwrite; //IRwrite, regwrite, NZCVwrite 신호 선언
  wire [1:0] regdst,regsrc,ALUsrcA,immsrc,ALUsrcB; //2bit의 regdst, regsrc, ALUsrcA, ALUsrcB 신호 선언 
  wire [2:0] ALUop; //3bit의 ALUop 신호 선언
  wire [3:0] instop; //4bit의 instop 신호 선언
  wire regBdst; //regBdst신호 선언

  
  //wires_out
  wire[31:0] imm; //32bit의 imm 신호 선언
  wire[31:0] pc_wire;
  
  //wires_in
  wire [3:0] ALUflags; //4bit의 ALUflags 신호 선언
  wire [31:0] ALUresult,readA,readB; //32bit의 ALUresult, readA, readB 신호 선언
  
  wire [3:0] RFdst [2:0]; //4bit의 RFdst 신호 3개의 배열 선언
  wire [31:0] RFsrc [4:0]; //32bit의 RFsrc 신호 5개의 배열 선언
  wire [31:0] ALUnum1 [1:0]; //32bit의 ALUnum1 신호 2개의 배열 선언
  wire [31:0] ALUnum2 [1:0]; //32bit의 ALUnum2 신호 2개의 배열 선언
  
  wire [3:0] regBread [1:0]; //4bit의 regBread 신호 2개의 배열 선언
  
  //registers
  assign be = 4'b1111; //be에 2진수 1111할당
  wire[31:0] A,B,instructions,mdr,ALUout; //32bit의 A, B, instructions, mdr, ALUout 신호 선언
  reg n,v; //n, v의 register 신호 선언
  wire z,c; //z, c, 신호 선언
  register InstructionRegister(.regin (inst), .write ('b1), .clk (clk), .reset (reset), .regout (instructions));
  register MDR(.regin (readdata), .write ('b1), .clk (clk), .reset (reset), .regout (mdr));
//register MDR의 regin 신호에 readdata 연결, write 신호에 신호 '1' 연결, clk 신호에 clk 연결, reset 신호에 reset 연결, regout 신호에 mdr 연결

  wire stall; //data hazard가 있는지 판단
  assign stall = ((IFID_Inst[19:16] == MEMWB_inst1512) | (regBread[regBdst] == MEMWB_inst1512))&MEMWB_regwrite
   | ((IFID_Inst[19:16] == EXMEM_inst1512) | (regBread[regBdst] == EXMEM_inst1512))&EXMEM_regwrite
   |((IFID_Inst[19:16] == IDEX_Inst[15:12]) | (regBread[regBdst] == IDEX_Inst[15:12]))&IDEX_regwrite &~reset;
  //IF step
  wire [31:0]PCtarget; //32bit의 PCtarget 신호 선언: pc 가 branch할 주소
  wire PCsrc_wire; //PCsrc_wire 신호 선언 : pc 가 branch하는지

  always @(posedge clk)begin //clk이 positive edge이면 실행
    if(PCsrc_wire == 1'b1) begin
        pc = PCtarget; //PCsrc_wire 신호가 '1'일 때 pc에 PCtarget 값 저장: branch
    end
	if(stall == 1'b1) begin
		pc=pc; //data hazard 때문에 현상유지
	end
	else begin
        pc = pc + 4; //PCsrc_wire 신호가 '0'이면(no branch) pc에 pc + 4 저장
    end
  end
  wire [31:0]IFID_PC; //IF-ID 단계에서 pc
  wire [31:0]IFID_Inst; //IF-ID 단계에서 instruction
  wire IFID_stall;
  wire stall_in;
  assign stall_in = stall | PCsrc_wire; //branch한다면 해당 instruction을 무효화
  IFID cache1(.stall_in(stall), .clk(clk), .reset(reset), .PC_in(pc_wire), .stall_out(IFID_stall), .Inst_in(inst),.PC_out(IFID_PC), .Inst_out(IFID_Inst));
  //IFID module의 cache1을 선언하고, cache1의 clk 신호에 clk 연결, PC_in 신호에 pc 연결, Inst_in 신호에 inst 연결, PC_out 신호에 IFID_PC 연결, Inst_out 신호에 IFID_Inst 연결

  //ID step
  wire ID_memwrite; //ID_memwrite 신호 선언
  wire ID_memread; //ID_memread 신호 선언
  wire ID_branch; //ID_branch 신호 선언
  signalunit SignalControl(
		.clk (clk),
		.reset (reset),
		.flags (IFID_Inst[31:20]),
		.zero (z),
		.branch(ID_branch),
		.Mwrite (ID_memwrite),
		.Mread (ID_memread),
		.regwrite (regwrite),
		.regdst (regdst),
		.regsrc (regsrc),
		.ALUsrcA (ALUsrcA),
		.ALUsrcB (ALUsrcB),
		.ALUop (instop),
		.NZCVwrite (NZCVwrite),
		.immsrc (immsrc),
		.regbdst (regBdst)); 
//signalunit module인 SignalControl 선언하고, SignalControl의 clk 신호에 clk 연결, reset 신호에 reset 연결, flags에 IFID_Inst[31:20] 연결, zero 신호에 z 연결, branch 신호에 ID_branch 연결
//Mwrite 신호에 ID_memwrite 연결, Mread 신호에 ID_memread 연결, regwrite 신호에 regwrite 연결, regdst 신호에 regdst 연결, regsrc 신호에 regsrc 연결, ALUsrcA 신호에 ALUsrcA연결, ALUsrcB 신호에 ALUsrcB 연결
//ALUop 신호에 instop 연결, NZCVwrite 신호에 NZCVwrite 연결, immsrc 신호에 immsrc 연결, regbdst 신호에 regBdst 연결	
  signextmux Immidiate(IFID_Inst[23:0],immsrc,imm); 
//signextmux module인 Immidiate를 선언하고, 24bit in을 IFID_Inst[23:0]으로, 2bit select을 immsrc으로 input으로 하고, 32bit의 imm의 값을 output함
  wire [31:0]IDEX_PC, IDEX_A, IDEX_B, IDEX_imm, IDEX_Inst; //32bit의 IDEX_PC 신호 선언, IDEX_A 신호 선언, IDEX_B 신호 선언, IDEX_imm 신호 선언, IDEX_Inst 신호 선언
  wire IDEX_memwrite, IDEX_branch; //IDEX_memwrite 신호 선언, IDEX_branch 신호 선언
  wire IDEX_regwrite,IDEX_NZCVwrite, IDEX_memread; //IDEX_regwrite 신호 선언, IDEX_NZCVwrite 신호 선언, IDEX_memread 신호 선언
  wire [1:0] IDEX_regdst,IDEX_regsrc,IDEX_ALUsrcA,IDEX_ALUsrcB; //2bit의 IDEX_regdst 신호 선언, IDEX_regsrc 신호 선언, IDEX_ALUsrcA 신호 선언, IDEX_ALUsrcB 신호 선언
  wire [3:0] IDEX_instop; //4bit의 IDEX_instop 신호 선언
  wire IDEX_stall; 
  wire IFID_stall_in;
  assign IFID_stall_in = IFID_stall | PCsrc_wire;//branch한다면 해당 instruction을 무효화
  IDEX cache2(.stall_in(IFID_stall_in), .clk(clk), .reset(reset), .inst_in(IFID_Inst), .PC_in(IFID_PC), .readA_in(readA), .readB_in(readB), .regdst_in(regdst), .regsrc_in(regsrc), .branch_in(ID_branch), .memwrite_in(ID_memwrite),
	.memread_in(ID_memread), .regwrite_in(regwrite), .ALUsrcA_in(ALUsrcA), .ALUsrcB_in(ALUsrcB),
	.instop_in(instop), .NZCVwrite_in(NZCVwrite), .imm_in(imm), .stall_out(IDEX_stall), .inst_out(IDEX_Inst), .PC_out(IDEX_PC), .readA_out(IDEX_A), .readB_out(IDEX_B),
	.regdst_out(IDEX_regdst), .regsrc_out(IDEX_regsrc), .branch_out(IDEX_branch), .memwrite_out(IDEX_memwrite),
	.memread_out(IDEX_memread), .regwrite_out(IDEX_regwrite), .ALUsrcA_out(IDEX_ALUsrcA), .ALUsrcB_out(IDEX_ALUsrcB),
	.instop_out(IDEX_instop), .NZCVwrite_out(IDEX_NZCVwrite), .imm_out(IDEX_imm));
//IDEX module인 cache2를 선언하고, cache2의 clk 신호에 clk연결, inst_in신호에 IFUD_INST연결, PC_in신호에 IFID_PC 연결, readA_in 신호에 readA 연결, readB_in 신호에 readB 연결, regdst_in 신호에 regdst 연결, regsrc_in
//regsrc 연결, branch_in 신호에 ID_branch 연결, memwrite_in 신호에 ID_memwrite 연결, memread_in 신호에 ID_memread 연결, regwrite_in 신호에 regwrite 연결, ALUsrcA_in 신호에 ALUsrcA 연결, ALUsrcB_in 신호에 ALUsrcB 연결
//instop_in 신호에 instop 연결, NZCVwrite_in 신호에 NZCVwrite 연결, imm_in 신호에 imm 연결, inst_out 신호에 IDEX_Inst 연결, PC_out 신호에 IDEX_PC 연결, readA_out 신호에 IDEX_A 연결, readB_out 신호에 IDEX_B 연결, 
//regdst_out 신호에 IDEX_regdst 연결, regsrc_out 신호에 IDEX_regsrc 연결, branch_out 신호에 IDEX_branch 연결, memwrite_out 신호에 IDEX_memwrite 연결, memread_out 신호에 IDEX_memread 연결, regwrite_out 신호에 IDEX_regwrite 연결
//ALUsrcA_out 신호에 IDEX_ALUsrcA 연결, ALUsrcB_out 신호에 IDEX_ALUsrcB 연결, instop_out 신호에 IDEX_instop 연결, NZCVwrite_out 신호에 IDEX_NZCVwrite 연결, imm_out 신호에 IDEX_imm 연결

  //EX step
  ALUopdecoder ALUopDecoder(
	.instop (IDEX_instop),
	.aluop (ALUop));
	//ALUopdecoder인 ALUopDecoder를 선언하고, instop 신호에 IDEX_instop 연결, aluop 신호에 ALUop 연결
  ALU32bit ALU(
	.inpa (ALUnum1[IDEX_ALUsrcA]),
	.inpb (ALUnum2[IDEX_ALUsrcB]),
	.cin (c),
	.aluop (ALUop),
	.result (ALUresult),
	.negative (ALUflags[3]),
	.zero (ALUflags[2]),
	.cout (ALUflags[1]),
	.overflow (ALUflags[0]));
	//ALU32bit module인 ALU를 선언하고, inpa 신호에 ALUnum1[IDEX_ALUsrcA] 연결, inpb 신호에 ALUnum2[IDEX_ALUsrcB] 연결, cin에 c 연결, aluop 신호에 ALUop 연결, result 신호에 ALUresult 연결
	//negative 신호에 ALUflags[3] 연결, zero 신호에 ALUflags[2] 연결, cout 신호에 ALUflags[1] 연결, overflow 신호에 ALUflags[0] 연결
  register ALUoutRegister(.regin (ALUresult), .write ('b1), .clk (clk), .reset (reset), .regout (ALUout));
//register인 ALUoutRegister를 선언하고 regin 신호에 ALUresult 연결, write 신호를 '1'로 설정, clk 신호에 clk 연결, reset 신호에 reset 연결, regout 신호에 ALUout 연결
  wire [1:0]EXMEM_regdst, EXMEM_regsrc; //2bit인 EXMEM_regdst 선언, EXMEM_regsrc 신호 선언
  wire EXMEM_branch, EXMEM_regwrite, EXMEM_c_out, EXMEM_z_out; //EXMEM_branch 신호 선언, EXMEM_regwrite 신호 선언, EXMEM_c_out 신호 선언, EXMEM_z_out 신호 선언
  wire [31:0]EXMEM_B; //32bit인 EXMEM_B 신호 선언
  wire [31:0]PCbranch, EXMEM_PC; //32bit인 PCbranch 신호 선언, EXMEM_PC 신호 선언
  assign PCbranch = IDEX_PC + IDEX_imm; //branch 할 주소 할당
  wire EXMEM_stall;
  wire [3:0]EXMEM_inst1512; //data hazard 를 판단하기 위한 rd register
  wire EXMEM_memwrite, EXMEM_memread;
  wire IDEX_stall_in;
  assign IDEX_stall_in = IDEX_stall |PCsrc_wire;//branch한다면 해당 instruction을 무효화
  EXMEM cache3(.stall_in(IDEX_stall_in), .clk(clk), .reset(reset), .inst1512_in(IDEX_Inst[15:12]), .PC_in(IDEX_PC), .PCtarget_in(PCbranch), .regdst_in(IDEX_regdst), .regsrc_in(IDEX_regdst), .B_in(IDEX_B), .branch_in(IDEX_branch), .memwrite_in(IDEX_memwrite),
  .memread_in(IDEX_memread), .regwrite_in(IDEX_regwrite), .NZCVwrite_in(IDEX_NZCVwrite), .c_in(ALUflags[1]), .z_in(ALUflags[2]),
  .inst1512_out(EXMEM_inst1512), .stall_out(EXMEM_stall), .PC_out(EXMEM_PC), .PCtarget_out(EXMEM_PCbranch), .regdst_out(EXMEM_regdst), .regsrc_out(EXMEM_regsrc), .B_out(EXMEM_B), .branch_out(EXMEM_branch),  .memwrite_out(EXMEM_memwrite),
  .memread_out(EXMEM_memread), .regwrite_out(EXMEM_regwrite), .c_out(EXMEM_c_out), .z_out(EXMEM_z_out));
  //EXMEM module인 cache3을 선언하고 clk신호에 clk 연결, PC_in 신호에 IDEX_PC 신호 연결, PCtarget_in 신호에 PCbranch 연결, regdst_in 신호에 IDEX_regdst 연결, regsrc_in 신호에 IDEX_regdst 연결
  //B_in 신호에 IDEX_B 연결, branch_in 신호에 IDEX_branch 연결, memwrite_in 신호에 IDEX_memwrite 연결, memread_in 신호에 IDEX_memread 연결, regwrite_in 신호에 IDEX_regwrite 연결, 
  //NZCVwrite_in 신호에 IDEX_NZCVwrite 연결, c_in 신호에 ALUflags[1] 연결, z_in 신호에 ALUflags[2] 연결, PC_out 신호에 EXMEM_PC 연결, PCtarget_out 신호에 EXMEM_PCbranch 연결,regdst_out 신호에 EXMEM_regdst 신호 연결
  //regsrc_out 신호에 EXMEM_regsrc 연결, B_out 신호에 EXMEM_B 연결, branch_out 신호에 EXMEM_branch 연결, memwrite_out 신호에 memwrite 연결, memread_out 신호에 memread 연결, regwrite_out 신호에 EXMEM_regwrite 연결
  //c_out 신호에 EXMEM_c_out 연결, z_out 신호에 EXMEM_z_out 연결

  //MEM step
  assign memwrite = EXMEM_memwrite & ~EXMEM_stall; // stall시 memory write 하지 않음
  assign memread = EXMEM_memread & ~EXMEM_stall; // stall 시 memroy read 하지 않음
  wire [31:0]MEMWB_ALUout, MEMWB_PC; //32bit의 MEMWB_ALUout 신호 선언, MEMWB_PC 신호 선언
  wire [1:0]MEMWB_regdst; //2bit의 MEMWB_regdst 신호 선언
  wire [2:0]MEMWB_regsrc; //3bit의 MEMWB_regsrc 신호 선언
  wire MEMWB_temp_regwrite;
  wire MEMWB_regwrite; //MEMWB_regwrite 신호 선언
  assign MEMWB_regwrite = MEMWB_temp_regwrite & ~MEMWB_stall; //instruction 은 regwrite하라고 하지만 data hazard가 있을 때
  assign PCsrc_wire =  EXMEM_branch & EXMEM_z_out; //PCsrc_wire 신호에 EXMEM_branch와 EXMEM_z_out의 and 연산 결과 할당
  assign PCtarget = EXMEM_PCbranch; //PCtarget 신호에 EXMEM_PCbranch 할당
  wire MEMWB_stall;
  wire [3:0]MEMWB_inst1512;
  MEMWB cache4(.stall_in(EXMEM_stall), .clk(clk), .reset(reset), .inst1512_in(EXMEM_inst1512), .PC_in(EXMEM_PC), .regdst_in(EXMEM_regdst), .regsrc_in(EXMEM_regsrc), .regwrite_in(EXMEM_regwrite), .ALUout_in(ALUout),
	.inst1512_out(MEMWB_inst1512), .stall_out(MEMWB_stall), .PC_out(MEMWB_PC), .regdst_out(MEMWB_regdst), .regsrc_out(MEMWB_regsrc), .regwrite_out(MEMWB_temp_regwrite), .ALUout_out(MEMWB_ALUout));
//MEMWB module인 cache4를 선언하고 cache4의 clk 신호에 clk 연결, PC_in 신호에 EXMEM_PC 연결, regdst_in 신호에 EXMEM_regdst 연결, regsrc_in 신호에 EXMEM_regsrc 연결, regwrite_in 신호에 EXMEM_regwrite 연결,
//ALUout_in 신호에 ALUout 연결, PC_out 신호에 MEMWB_PC 연결, regdst_out 신호에 MEMWB_regdst 연결, regsrc_out 신호에 MEMWB_regsrc 연결, regwrite_out 신호에 MEMWB_regwrite 연결, ALUout_out에 MEMWB_ALUout 연결
  
//mux
		//register file destination(Rd field)
		assign RFdst[0]=MEMWB_inst1512; //if regdst=0 then rd=instructions[15:12]
		assign RFdst[1]=4'b1110; //if regdst=2 then rd=14(Link register)
		//register file source
		assign RFsrc[0]=mdr;// write register = MDR
		assign RFsrc[1]=MEMWB_ALUout; //write register = ALUout (in aluout register)
		assign RFsrc[2]=ALUresult; // write register = ALUresult (in immediate aluout)
		assign RFsrc[3]=B; //write register = B
		assign RFsrc[4]=MEMWB_PC;

		assign regBread[0]=IFID_Inst[3:0]; //regBread[0]에 IFID_Inst[3:0]의 값 할당
		assign regBread[1]=IFID_Inst[15:12]; //regBread[1]에 IFID_Inst[15:12]의 값 할당

		assign ALUnum1[0]=IDEX_A; //ALUnum1[0]에 IDEX_A 할당
		assign ALUnum1[1]='h0; //ALUnum1[1]에 16진수 '0' 할당

		assign ALUnum2[0]=IDEX_imm; //ALUnum2[0]에 IDEX_imm 할당
		assign ALUnum2[1]=IDEX_B<<IDEX_Inst[11:7]; //ALUnum2[1]에 IDEX_B를 Inst[11:7]만큼 shifting left한 결과 할당

		assign writedata=EXMEM_B; //writedata에 EXMEM_B 할당
		assign memaddr=ALUout; //memaddr에 ALUout 할당

		
		
		registerfile RegisterFile(
			.reg1 (IFID_Inst[19:16]),
			.reg2 (regBread[regBdst]),
			.regdst (RFdst[MEMWB_regdst]),
			.regsrc (RFsrc[MEMWB_regsrc]),
			.clk (clk),
			.reset (reset),
			.we (MEMWB_regwrite),
			.out1 (readA),
			.out2 (readB),
			.pc (pc_wire));
		//registerfile module인 RegisterFile을 선언하고, reg1 신호에 IFID_Inst[19:16] 연결, reg2 신호에 regBread[regBdst] 연결, regdst 신호에 RFdst[MEMWB_regdst] 연결, regsrc 신호에 RFsrc[MEMWB_regsrc] 연결
		//clk 신호에 clk 연결, reset 신호에 reset 연결, we 신호에 MEMWB_regwrite 연결, out1 신호에 readA 연결, out2 신호에 readB 연결, pc 신호에 IFID_PC 연결
	

		

endmodule //armreduced module 선언 종료

module IFID(
	input stall_in,
	input clk,
	input reset,
	input [31:0]Inst_in,
	input [31:0]PC_in,
	output reg stall_out,
	output reg[31:0]Inst_out,
	output reg[31:0]PC_out); //IFID module 선언 및 input, output 설정
  
  always @(posedge clk)begin 
	if(reset == 1'b1) begin
      stall_out <= 1'b0;
	  Inst_out <= 'h000000000;
	  PC_out <= 'h00000000;
	end
	stall_out <= stall_in;
	Inst_out <= Inst_in;
	PC_out <= PC_in;
  end //clk이 positive edge일 때 Inst_out에 Inst_in 저장, PC_out에 PC_in 저장
endmodule //IFID module 종료

module IDEX(
	input stall_in,
	input clk,
	input reset,
	input [31:0]inst_in,
	input [31:0]PC_in,
	input [31:0]readA_in,
	input [31:0]readB_in,
	input [1:0]regdst_in,
	input [2:0]regsrc_in,
	input branch_in,
	input memwrite_in,
	input memread_in,
	input regwrite_in,
	input ALUsrcA_in,
	input ALUsrcB_in,
	input [3:0]instop_in,
	input NZCVwrite_in,
	input [31:0]imm_in,
	output reg stall_out,
	output reg[31:0]inst_out,
	output reg branch_out,
	output reg[31:0]PC_out,
	output reg readA_out,
	output reg[31:0]readB_out,
	output reg[1:0]regdst_out,
	output reg[2:0]regsrc_out,
	output reg memwrite_out,
	output reg memread_out,
	output reg regwrite_out,
	output reg ALUsrcA_out,
	output reg ALUsrcB_out,
	output reg [3:0]instop_out,
	output reg NZCVwrite_out,
	output reg [31:0]imm_out
); //IDEX module 선언 및 input, output 설정
  always @(posedge clk)begin
	if(reset == 1'b1) begin
	    inst_out <= 'h00000000;
		PC_out <= 'h00000000;
		branch_out <= 1'b0;
		memwrite_out <= 1'b0;
		memread_out <= 1'b0;
		regwrite_out <= 1'b0;
		NZCVwrite_out <= 1'b0;
		stall_out <= 1'b0;
	end else begin
		inst_out <= inst_in;
		PC_out <= PC_in;
		readA_out <= readA_in;
		readB_out <= readB_in;
		regdst_out <= regdst_in;
		regsrc_out <= regsrc_in;
		branch_out <= branch_in;
		memwrite_out <= memwrite_in;
		memread_out <= memread_in;
		regwrite_out <= regwrite_in;
		ALUsrcA_out <= ALUsrcA_in;
		ALUsrcB_out <= ALUsrcB_in;
		instop_out <= instop_in;
		NZCVwrite_out <= NZCVwrite_in;
		imm_out <= imm_in;
		stall_out <= stall_in;
	end
  end
  //clk이 positive edge일 때 Inst_out에 Inst_in 저장, PC_out에 PC_in 저장, readA_out에 readA_in 저장, readB_out에 readB_in 저장, regdst_out에 regdst_in 저장, regsrc_out에 regsrc_in 저장, branch_out에 branch_in 저장,
  //memwrite_out에 memwrite_in 저장, memread_out에 memread_in 저장, regwrite_out에 regwrite_in 저장, ALUsrcA_out에 ALUsrcA_in 저장, ALUsrcB_out에 ALUsrcB_in 저장, instop_out에 instop_in 저장,
  //NZCVwrite_out에 NZCVwrite_in 저장, imm_out에 imm_in 저장

endmodule //IDEX module 종료

module EXMEM(
  input stall_in,
  input clk,
  input reset,
  input [3:0]inst1512_in,
  input [31:0]PC_in,
  input [31:0]PCtarget_in,
  input [1:0]regdst_in,
  input [2:0]regsrc_in,
  input [31:0]B_in,
  input branch_in,
  input memwrite_in,
  input memread_in,
  input regwrite_in,
  input c_in, z_in,
  input NZCVwrite_in,
  output reg[3:0]inst1512_out,
  output reg stall_out,
  output reg[31:0]PC_out,
  output reg branch_out,
  output reg[31:0]PCtarget_out,
  output reg[1:0]regdst_out,
  output reg[2:0]regsrc_out,
  output reg[31:0]B_out,
  output reg memwrite_out,
  output reg memread_out,
  output reg regwrite_out,
  output reg c_out, z_out
); //EXMEM module 선언 및 input, output 설정
always @(posedge clk)begin
	if(reset == 1'b1) begin
	  PC_out <= 'h000000000;
	  branch_out <= 1'b0;
	  PCtarget_out <= 1'b0;
	  memwrite_out <= 1'b0;
	  memread_out <= 1'b0;
	  regwrite_out <= 1'b0;
	  stall_out <= 1'b0;
	end else begin
		inst1512_out <= inst1512_in;
		PC_out <= PC_in;
		branch_out <= branch_in;
		PCtarget_out <= PCtarget_in;
		regdst_out <= regdst_in;
		regsrc_out <= regsrc_in;
		memwrite_out <= memwrite_in;
		memread_out <= memread_in;
		regwrite_out <= regwrite_in;
		B_out <= B_in;
		stall_out <= stall_in;
		if(NZCVwrite_in == 1'b1)begin
		c_out <=c_in;
		z_out <=z_in;
		end
	end
end
//clk가 positive edge일때, PC_out에 PC_in 저장, branch_out에 branch_in 저장, PCtarget_out에 PC_target_in 저장, regdst_out에 regsrc_in 저장, regsrc_out에 regsrc_in 저장, memwrite_out에 memwrite_in 저장, 
//memread_out에 memread_in 저장, regwrite_out에 regwrite_in 저장, B_out에 B_in 저장, NZCVwrite_in이 '1'이라면, c_out에 c_in을 저장, z_out에 z_in 저장
endmodule //EXMEM module 저장

module MEMWB( 
	input stall_in,
	input clk,
	input reset,
	input [3:0]inst1512_in,
	input [31:0]PC_in,
	input [1:0]regdst_in,
	input [1:0]regsrc_in,
	input regwrite_in,
	input [31:0]readdata_in,
	input [31:0]ALUout_in,
	output reg[31:0]inst1512_out,
	output reg stall_out,
	output reg[31:0]PC_out,
	output reg[1:0]regdst_out,
	output reg[1:0]regsrc_out,
	output reg regwrite_out,
	output reg[31:0]readdata_out,
	output reg[31:0]ALUout_out
); //MEMWB module 선언 및 input, output 설정
  always @(posedge clk)begin
	if(reset == 1'b1)begin
	  stall_out <= 1'b0;
	  regwrite_out <= 1'b0;
	  readdata_out <= 1'b0;
	  PC_out <= 'h00000000;
	end
	inst1512_out <= inst1512_in;
	stall_out <= stall_in;
	regdst_out <= regdst_in;
	regsrc_out <= regsrc_in;
	regwrite_out <= regwrite_in;
	readdata_out <= readdata_in;
	ALUout_out <= ALUout_in;
	PC_out <= PC_in;
	
  end
  //clk가 positive edge일 때 regdst_out에 regdst_in 저장, regsrc_out에 regsrc_in 저장, regwrite_out에 regwrite_in 저장, readdata_out에 readdata_in 저장, ALUout_out에 ALUout_in 저장, PC_out에 PC_in 저장
endmodule //MEMWB module 선언 종료
