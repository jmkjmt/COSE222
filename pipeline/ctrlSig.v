module signalcontrol(
	input[11:0] flags,
	input zero, //number of step need to complete a instruction
	output reg [17:0] s); //signalcontrol module 선언 및 input, output 설정
	
 always @ (*) begin //코드 내 신호가 바뀔 때마다 실행
  if((flags[11]&flags[10]&flags[9])||(flags[8]^zero)) begin //flags[11], flags[10], flags[9]의 값이 모두 1이거나 flags[8]와 zero의 값이 다를 때 실행
   if(flags[7]) begin //B, BL //flags[7] == 1일 때 실행
    if(~flags[4]) begin //B //flags[4] == 0일 때 실행
    s=18'b1000xxxxxxxxxx010x;
    end
   else begin //BL //flags[4] == 1일 때 실행
    s=18'b10011100xxxxxx010x;
   end
  end
  else if(flags[6]) begin //LDR, STR //flag[6] == 1일 때 실행
   if(~flags[0]) begin //STR  //flags[0] == 0일 때 실행
    s=18'b01000xxx000100001x;
   end
   else begin //LDR //flags[1] == 1일 때 실행
    s=18'b00100xxx000100001x;
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
    s=18'b00000xxx001010101x;
    end
    13 : begin //MOV
    s=18'b00010001100100001x;
    end
    default : begin //ALU
     s={9'b000100010, (flags[5]==1 ? 1'b0 : 1'b1), flags[4:0], 3'b000};
    end
   endcase
  end
  else begin //Recovery
    s=18'b1000xxxxxxxxxx0xxx;
  end
 end
endmodule //signalcontrol module 선언 종료

module signalunit
	(input clk,//clock
	 input reset, //reset
	 input[11:0] flags, //어떤 명령어인지 결정할 수 있는 flag
	 input zero,
   output branch, //branch signal
	 output Mwrite, //memory write signal
	 output Mread, //memory read signal
	 output regwrite, //register write signal
	 output regdst, //register destination signal
	 output[2:0] regsrc, //ALU source A signal
	 output ALUsrcA, //ALU source A signal
   output ALUsrcB, //ALU source B signal
	 output[3:0] ALUop, //ALU opcode signal
	 output NZCVwrite, //NZCV flag update signal
	 output [1:0] immsrc, //immediate value source signal
	 output regbdst); //register B destination signal
//signalunit module 선언 및 input, output 설정정
  
  wire [17:0] s; // 18-bit signal

  signalcontrol bringSignal( //18-bit signal을 할당
	.flags (flags),
	.zero (zero),
	.s (s));
 
 //아래는 각 instruction의 signal을 활성화
  assign branch=s[17]; // 1: branch
  assign Mwrite=s[16]; //  1: memory write 
  assign Mread=s[15]; // 1:memory read
  assign regwrite=s[14]; // 1: register write
  assign regdst=s[13]; // select destination register(0:rd/1:lr)
  assign regsrc=s[12:10]; // select source register(0: mdr/ 1:MEMWB_ALUout/ 2:ALUresult/3:B/ 4:MEMWB_PC)
  assign ALUsrcA=s[9]; //(0:A/ 1:0)
  assign ALUsrcB=s[8]; //select ALUsourceB(0:imm 1: b<< inst[12:7])
  assign ALUop=s[7:4]; //select ALU opcode
  assign NZCVwrite=s[3]; //update NZCV flags
  assign immsrc=s[2:1]; //select how extend immediate value 
  assign regbdst=s[0]; // select second source register(0: inst[3:0], 1: inst[15:12])
  
endmodule //signalunit module 선언 종료