module signalcontrol(
	input[11:0] flags,
	input zero,
	output reg [2:0] total, //number of step need to complete a instruction
	output reg [19:0] s2,
	output reg [19:0] s3,
	output reg [19:0] s4);
	
 always @ (*) begin
  if((flags[11]&flags[10]&flags[9])||(flags[8]^zero)) begin
   if(flags[7]) begin //B, BL
    if(~flags[4]) begin //B
    s2=20'b00010110001001000100;
    s3=20'bxxxxxxxxxxxxxxxxxxxx;
    s4=20'bxxxxxxxxxxxxxxxxxxxx;
    total=2;
    end
   else begin //BL
    s2=20'b00011001001001000100;
    s3=20'b00010101xxxxxxxx0xxx;
    s4=20'bxxxxxxxxxxxxxxxxxxxx;
    total=3;
   end
  end
  else if(flags[6]) begin //LDR, STR
   s2={10'b0001010101,(flags[5]==1 ? 2'b11 : 2'b10),(flags[3]==1 ? 4'b0100 : 4'b0010), 3'b001, (flags[0]==1 ? 1'b0 : 1'b1)};
  
   if(~flags[0]) begin //STR
    s3=20'b1000xxxxxxxxxxxx0xxx;
    s4=20'bxxxxxxxxxxxxxxxxxxxx;
    total=3;
   end
   else begin //LDR
    s3=20'b0010xxxxxxxxxxxx0xxx;
    s4=20'b00010000xxxxxxxx0xxx;
    total=4;
   end 
  end
  
  else //Else
   case(flags[4:1])/*
      0 : //AND
      1 : //EOR
      2 : //SUB
      4 : //ADD
      5 : //ADC
      6 : //SBC
      12 : //ORR*/
    10 : begin //CMP
    s2={10'b0001010101, (flags[5]==1 ? 2'b10 : 2'b11), 8'b00101000};
    s3=20'bxxxxxxxxxxxxxxxxxxxx;
    s4=20'bxxxxxxxxxxxxxxxxxxxx;
    total=2;
    end
    13 : begin //MOV
    s2={10'b0001010110, (flags[5]==1 ? 2'b10 : 2'b11), 4'b0100 ,flags[0], 3'b000};
    s3=20'b00010001xxxxxxxx0xxx;
    s4=20'bxxxxxxxxxxxxxxxxxxxx;
    total=3; 
    end
    default : begin //ALU
     s2={10'b0001010101, (flags[5]==1 ? 2'b10 : 2'b11), flags[4:0], 3'b000};
     s3=20'b00010001xxxxxxxx0xxx;
     s4=20'bxxxxxxxxxxxxxxxxxxxx;
     total=3;
    end
   endcase
  end
  else begin //Recovery
   s2=20'b00010101xxxxxxxx0xxx;
   s3=20'bxxxxxxxxxxxxxxxxxxxx;
   s4=20'bxxxxxxxxxxxxxxxxxxxx;
   total=2;
  end
 end
endmodule 

module oneAdder(
	input clk, //clock
	input reset, //reset
	input [2:0] current, 
	output reg[2:0] regout);
	
	wire last;
	
	assign last=current==regout;
	
	always @ (posedge clk,posedge reset)
	if(reset)
		regout<='b000;
	else if(last)
		regout<='b000;
	else
		regout<=regout+'b001;	
endmodule

module signalunit
	(input clk,//clock
	 input reset, //reset
	 input[11:0] flags, //어떤 명령어인지 결정할 수 있는 flag
	 input zero,
	 output Mwrite, //memory write signal
	 output IRwrite,//instruction register write signal
	 output Mread, //memory read signal
	 output regwrite, //register write signal
	 output[1:0] regdst, //register destination signal
	 output[1:0] regsrc, //register source signal
	 output[1:0] ALUsrcA, //ALU source A signal
	 output[1:0] ALUsrcB, //ALU source B signal
	 output[3:0] ALUop, //ALU opcode signal
	 output NZCVwrite, //NZCV flag update signal
	 output [1:0] immsrc, //immediate value source signal
	 output regbdst); //register B destination signal
  
  wire [19:0] s [4:0]; // array of 5 20-bit vector(20비트 시그널, load 명령어의 길이가 5이기 때문에 크기가 5인 array를 선언)
  
  assign s[0] = 20'b01110110000101000xxx; //0번 step의 20비트 시그널(모든 instruction이 동일)
  assign s[1] = 20'b0000xxxx000000100xxx; //1번 step의 20비트 시그널(모든 instruction이 동일)
  
  wire [2:0] total; //instruction의 길이(cycle)
  wire [2:0] step; //현재 실행중인 instruction이 위치한 단계
  
  oneAdder Step(.clk (clk), .reset (reset), .current (total), .regout (step)); //clock edge마다 total과 같아질 때까지step을 1증가시킴
  
  signalcontrol bringSignal( //각 setp에 따라 20비트로 signal을 지정
	.flags (flags),
	.zero (zero),
	.total (total),
	.s2 (s[2]), //2번 step의 20비트 시그널
	.s3 (s[3]), //3번 step의 20비트 시그널
	.s4 (s[4]));   //4번 step의 20비트 시그널
 
 //아래는 instruction의 step별로 20비트의 signal을 활성화
  assign Mwrite=s[step][19]; //  1: memory write 
  assign IRwrite=s[step][18]; // 1: instruction register write(instruction fetch)
  assign Mread=s[step][17]; // 1:memory read
  assign regwrite=s[step][16]; // 1: register write
  assign regdst=s[step][15:14]; // select destination register(0:rd/1:pc/2:lr)
  assign regsrc=s[step][13:12]; // select source register
  assign ALUsrcA=s[step][11:10]; //select ALUsourceA (0:PC, 1: A, 2: 0)
  assign ALUsrcB=s[step][9:8]; //select ALUsourceB(0:4 1:8 2:imm 3: b<< inst[12:7])
  assign ALUop=s[step][7:4]; //select ALU opcode
  assign NZCVwrite=s[step][3]; //update NZCV flags
  assign immsrc=s[step][2:1]; //select how extend immediate value 
  assign regbdst=s[step][0]; // select second source register(0: inst[3:0], 1: inst[15:12])
  
endmodule