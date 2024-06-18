
module DIG_Register_BUS #(
    parameter Bits = 1
)
(
    input C,
    input en,
    input [(Bits - 1):0]D,
    output [(Bits - 1):0]Q
);

    reg [(Bits - 1):0] state = 'h0;

    assign Q = state;

    always @ (posedge C) begin
        if (en)
            state <= D;
   end
endmodule
module DIG_ROM_256X16_InstructionMemory (
    input [7:0] A,
    input sel,
    output reg [15:0] D
);
    reg [15:0] my_rom [0:20];
	 integer i;
	 
    always @(sel,A,D, my_rom[0],my_rom[1],my_rom[2],my_rom[3],my_rom[4],my_rom[5],my_rom[6],my_rom[7],my_rom[8],my_rom[9],my_rom[10],my_rom[11],my_rom[12],my_rom[13],my_rom[14],my_rom[15],my_rom[16],my_rom[17],my_rom[18],my_rom[19],my_rom[20]) begin
        if (~sel)
            D = 16'hz;
        else if (A > 8'h5)
            D = 16'h0;
        else
            D = my_rom[A];
    end

    initial begin
		  for (i = 0; i < 21; i = i + 1) begin
            my_rom[i] = 16'h0000;
        end
      my_rom[0] = 16'h7054;
      my_rom[1] = 16'h708a;
      my_rom[2] = 16'h229a;
      my_rom[3] = 16'h50c1;
      my_rom[4] = 16'h7045;
    end
endmodule

module DIG_Add
#(
    parameter Bits = 1
)
(
    input [(Bits-1):0] a,
    input [(Bits-1):0] b,
    input c_i,
    output [(Bits - 1):0] s,
    output c_o
);
   wire [Bits:0] temp;

   assign temp = a + b + c_i;
   assign s = temp [(Bits-1):0];
   assign c_o = temp[Bits];
endmodule


module DIG_RegisterFile
#(
    parameter Bits = 8,
    parameter AddrBits = 4
)
(
    input [(Bits-1):0] Din,
    input we,
    input [(AddrBits-1):0] Rw,
    input C,
    input [(AddrBits-1):0] Ra,
    input [(AddrBits-1):0] Rb,
    output [(Bits-1):0] Da,
    output [(Bits-1):0] Db,
    output [(Bits-1):0] RH0,
    output [(Bits-1):0] RH1,
    output [(Bits-1):0] RH2,
    output [(Bits-1):0] RH3,
    output [(Bits-1):0] RH4,
    output [(Bits-1):0] RH5,
    output [(Bits-1):0] RH6,
    output [(Bits-1):0] RH7
);
    reg [(Bits-1):0] memory[0:((1 << AddrBits)-1)];
    reg [7:0] RH0_t;
    reg [7:0] RH1_t;
    reg [7:0] RH2_t;
    reg [7:0] RH3_t;
    reg [7:0] RH4_t;
    reg [7:0] RH5_t;
    reg [7:0] RH6_t;
    reg [7:0] RH7_t;
    assign Da = memory[Ra];
    assign Db = memory[Rb];
    integer y;
    initial begin
        for (y = 0; y <(1 << AddrBits); y = y + 1) begin
            memory[y] = 0;
        end
    end
    always @ (posedge C) begin
        if (we) 
            memory[Rw] <= Din;
        // regouts
        RH0_t = memory[0];
        RH1_t = memory[1];
        RH2_t = memory[2];
        RH3_t = memory[3];
        RH4_t = memory[4];
        RH5_t = memory[5];
        RH6_t = memory[6];
        RH7_t = memory[7];
    end
    assign RH0 = RH0_t;
    assign RH1 = RH1_t;
    assign RH2 = RH2_t;
    assign RH3 = RH3_t;
    assign RH4 = RH4_t;
    assign RH5 = RH5_t;
    assign RH6 = RH6_t;
    assign RH7 = RH7_t;
endmodule


module Mux_2x1_NBits #(
    parameter Bits = 2
)
(
    input [0:0] sel,
    input [(Bits - 1):0] in_0,
    input [(Bits - 1):0] in_1,
    output reg [(Bits - 1):0] out
);
    always @ (sel, in_0, in_1, out) begin
        case (sel)
            1'h0: out = in_0;
            1'h1: out = in_1;
            default:
                out = 'h0;
        endcase
    end
endmodule

module DIG_BitExtender #(
    parameter inputBits = 2,
    parameter outputBits = 4
)
(
    input [(inputBits-1):0] in,
    output [(outputBits - 1):0] out
);
    assign out = {{(outputBits - inputBits){in[inputBits - 1]}}, in};
endmodule



module DIG_RAMDualPort
#(
    parameter Bits = 8,
    parameter AddrBits = 4
)
(
  input [(AddrBits-1):0] A,
  input [(Bits-1):0] Din,
  input str,
  input C,
  input ld,
  output [(Bits-1):0] D
);
  reg [(Bits-1):0] memory[0:((1 << AddrBits) - 1)];

  assign D = ld? memory[A] : 'hz;

  always @ (posedge C) begin
    if (str)
      memory[A] <= Din;
  end
endmodule

module LUT_CONTROLLER (
    input \0 ,
    input \1 ,
    input \2 ,
    input \3 ,
    input \4 ,
    input \5 ,
    input \6 ,
    output reg  [17:0]  out
);
    reg [17:0] my_lut [0:127];
    wire [6:0] temp;
    assign temp = {\6 , \5 , \4 , \3 , \2 , \1 , \0 };

    always @ (temp,out,my_lut[0],my_lut[1],my_lut[2],my_lut[3],my_lut[4],my_lut[5],my_lut[6],my_lut[7],my_lut[8],my_lut[9],my_lut[10],my_lut[11],my_lut[12],my_lut[13],my_lut[14],my_lut[15],my_lut[16],my_lut[17],my_lut[18],my_lut[19],my_lut[20],my_lut[21],my_lut[22],my_lut[23],my_lut[24],my_lut[25],my_lut[26],my_lut[27],my_lut[28],my_lut[29],my_lut[30],my_lut[31],my_lut[32],my_lut[33],my_lut[34],my_lut[35],my_lut[36],my_lut[37],my_lut[38],my_lut[39],my_lut[40],my_lut[41],my_lut[42],my_lut[43],my_lut[44],my_lut[45],my_lut[46],my_lut[47],my_lut[48],my_lut[49],my_lut[50],my_lut[51],my_lut[52],my_lut[53],my_lut[54],my_lut[55],my_lut[56],my_lut[57],my_lut[58],my_lut[59],my_lut[60],my_lut[61],my_lut[62],my_lut[63],my_lut[64],my_lut[65],my_lut[66],my_lut[67],my_lut[68],my_lut[69],my_lut[70],my_lut[71],my_lut[72],my_lut[73],my_lut[74],my_lut[75],my_lut[76],my_lut[77],my_lut[78],my_lut[79],my_lut[80],my_lut[81],my_lut[82],my_lut[83],my_lut[84],my_lut[85],my_lut[86],my_lut[87],my_lut[88],my_lut[89],my_lut[90],my_lut[91],my_lut[92],my_lut[93],my_lut[94],my_lut[95],my_lut[96],my_lut[97],my_lut[98],my_lut[99],my_lut[100],my_lut[101],my_lut[102],my_lut[103],my_lut[104],my_lut[105],my_lut[106],my_lut[107],my_lut[108],my_lut[109],my_lut[110],my_lut[111],my_lut[112],my_lut[113],my_lut[114],my_lut[115],my_lut[116],my_lut[117],my_lut[118],my_lut[119],my_lut[120],my_lut[121],my_lut[122],my_lut[123],my_lut[124],my_lut[125],my_lut[126],my_lut[127]) begin
       out = my_lut[temp];
    end

    initial begin
        my_lut[0] = 18'h0;
        my_lut[1] = 18'h588;
        my_lut[2] = 18'h608;
        my_lut[3] = 18'h688;
        my_lut[4] = 18'h4000;
        my_lut[5] = 18'h8008;
        my_lut[6] = 18'h8008;
        my_lut[7] = 18'h8;
        my_lut[8] = 18'h8;
        my_lut[9] = 18'h88;
        my_lut[10] = 18'h88;
        my_lut[11] = 18'h388;
        my_lut[12] = 18'h408;
        my_lut[13] = 18'h1200;
        my_lut[14] = 18'h1200;
        my_lut[15] = 18'h1280;
        my_lut[16] = 18'h1300;
        my_lut[17] = 18'h488;
        my_lut[18] = 18'h708;
        my_lut[19] = 18'h208;
        my_lut[20] = 18'h508;
        my_lut[21] = 18'h0;
        my_lut[22] = 18'h0;
        my_lut[23] = 18'h0;
        my_lut[24] = 18'h40;
        my_lut[25] = 18'h40;
        my_lut[26] = 18'h40;
        my_lut[27] = 18'h40;
        my_lut[28] = 18'h40;
        my_lut[29] = 18'h40;
        my_lut[30] = 18'h40;
        my_lut[31] = 18'h40;
        my_lut[32] = 18'h20048;
        my_lut[33] = 18'h20048;
        my_lut[34] = 18'h20048;
        my_lut[35] = 18'h20048;
        my_lut[36] = 18'h20048;
        my_lut[37] = 18'h20048;
        my_lut[38] = 18'h20048;
        my_lut[39] = 18'h20048;
        my_lut[40] = 18'ha0;
        my_lut[41] = 18'ha0;
        my_lut[42] = 18'ha0;
        my_lut[43] = 18'ha0;
        my_lut[44] = 18'ha0;
        my_lut[45] = 18'ha0;
        my_lut[46] = 18'ha0;
        my_lut[47] = 18'ha0;
        my_lut[48] = 18'h20a0;
        my_lut[49] = 18'h20a0;
        my_lut[50] = 18'h20a0;
        my_lut[51] = 18'h20a0;
        my_lut[52] = 18'h20a0;
        my_lut[53] = 18'h20a0;
        my_lut[54] = 18'h20a0;
        my_lut[55] = 18'h20a0;
        my_lut[56] = 18'hb;
        my_lut[57] = 18'hb;
        my_lut[58] = 18'hb;
        my_lut[59] = 18'hb;
        my_lut[60] = 18'hb;
        my_lut[61] = 18'hb;
        my_lut[62] = 18'hb;
        my_lut[63] = 18'hb;
        my_lut[64] = 18'hb;
        my_lut[65] = 18'hb;
        my_lut[66] = 18'hb;
        my_lut[67] = 18'hb;
        my_lut[68] = 18'hb;
        my_lut[69] = 18'hb;
        my_lut[70] = 18'hb;
        my_lut[71] = 18'hb;
        my_lut[72] = 18'h70b;
        my_lut[73] = 18'h70b;
        my_lut[74] = 18'h70b;
        my_lut[75] = 18'h70b;
        my_lut[76] = 18'h70b;
        my_lut[77] = 18'h70b;
        my_lut[78] = 18'h70b;
        my_lut[79] = 18'h70b;
        my_lut[80] = 18'h70b;
        my_lut[81] = 18'h70b;
        my_lut[82] = 18'h70b;
        my_lut[83] = 18'h70b;
        my_lut[84] = 18'h70b;
        my_lut[85] = 18'h70b;
        my_lut[86] = 18'h70b;
        my_lut[87] = 18'h70b;
        my_lut[88] = 18'h38b;
        my_lut[89] = 18'h38b;
        my_lut[90] = 18'h38b;
        my_lut[91] = 18'h38b;
        my_lut[92] = 18'h38b;
        my_lut[93] = 18'h38b;
        my_lut[94] = 18'h38b;
        my_lut[95] = 18'h38b;
        my_lut[96] = 18'h40b;
        my_lut[97] = 18'h40b;
        my_lut[98] = 18'h40b;
        my_lut[99] = 18'h40b;
        my_lut[100] = 18'h40b;
        my_lut[101] = 18'h40b;
        my_lut[102] = 18'h40b;
        my_lut[103] = 18'h40b;
        my_lut[104] = 18'h1040b;
        my_lut[105] = 18'h1040b;
        my_lut[106] = 18'h1040b;
        my_lut[107] = 18'h1040b;
        my_lut[108] = 18'h1040b;
        my_lut[109] = 18'h1040b;
        my_lut[110] = 18'h1040b;
        my_lut[111] = 18'h1040b;
        my_lut[112] = 18'h80f;
        my_lut[113] = 18'h80f;
        my_lut[114] = 18'h80f;
        my_lut[115] = 18'h80f;
        my_lut[116] = 18'h80f;
        my_lut[117] = 18'h80f;
        my_lut[118] = 18'h80f;
        my_lut[119] = 18'h80f;
        my_lut[120] = 18'h12;
        my_lut[121] = 18'h12;
        my_lut[122] = 18'h12;
        my_lut[123] = 18'h12;
        my_lut[124] = 18'h12;
        my_lut[125] = 18'h12;
        my_lut[126] = 18'h12;
        my_lut[127] = 18'h12;
    end
endmodule


module Mux_4x1_NBits #(
    parameter Bits = 2
)
(
    input [1:0] sel,
    input [(Bits - 1):0] in_0,
    input [(Bits - 1):0] in_1,
    input [(Bits - 1):0] in_2,
    input [(Bits - 1):0] in_3,
    output reg [(Bits - 1):0] out
);
    always @ (sel, out, in_0, in_1, in_2, in_3 ) begin
        case (sel)
            2'h0: out = in_0;
            2'h1: out = in_1;
            2'h2: out = in_2;
            2'h3: out = in_3;
            default:
                out = 'h0;
        endcase
    end
endmodule

module ALU (
    input [3:0] ALUop,        // ALU Selection
    input [7:0] A, B,         // ALU 8-bit Inputs
    input [2:0] SHAMT,         // SHAMT
    output reg [7:0] S,         // ALU Output
    output reg [7:0] HIGH,      // HIGH Output
    output Zero                 // Zero Flag
);
    reg [15:0] ALU_Result;      // 16-bit result for operations
    assign Zero = (S == 0);

    always @(*) begin
        case (ALUop)
            4'b0000: begin
                ALU_Result = A + B;             // ADD
                HIGH = 8'b0;
            end
            4'b0001: begin
                ALU_Result = A - B;             // SUB
                HIGH = 8'b0;
            end
            4'b0100: begin
                ALU_Result = A * B;             // MUL
                HIGH = ALU_Result[15:8];         // Last 8 bits
            end
            4'b0101: begin
                //ALU_Result = A / B;             // DIV
               // HIGH = A % B;              // Remainder
            end
            4'b0110: begin
                //ALU_Result = A / B;             // DIVU
                //HIGH = A % B;              // Remainder
            end
            4'b0111: begin
                ALU_Result = A & B;             // AND
                HIGH = 8'b0;
            end
            4'b1000: begin
                ALU_Result = A | B;             // OR
                HIGH = 8'b0;
            end
            4'b1010: begin
                ALU_Result = A ^ B;             // XOR
                HIGH = 8'b0;
            end
            4'b1001: begin
                ALU_Result = ~(A | B);          // NOR
                HIGH = 8'b0;
            end
            4'b1011: begin
                ALU_Result = B << SHAMT;            // SLL 
                HIGH = 8'b0;
            end
            4'b1100: begin
                ALU_Result = B >> SHAMT;            // SRL 
                HIGH = 8'b0;
            end
            4'b1101: begin
                ALU_Result = B >>> SHAMT;           // SRA 
                HIGH = 8'b0;
            end
            4'b1110: begin
                ALU_Result = (A < B) ? 8'b1 : 8'b0; // LESSTHAN
                HIGH = 8'b0;
            end
            default: begin
                ALU_Result = 8'b0;              // Default case
                HIGH = 8'b0;
            end
        endcase
    end

    always @(*) begin
        S = ALU_Result[7:0];             // First 8 bits of result
    end
endmodule

module CPU (
  input CLK,
  output [7:0] HI,
  output [7:0] LO,
  output [7:0] RH0,
  output [7:0] RH1,
  output [7:0] RH2,
  output [7:0] RH3,
  output [7:0] RH4,
  output [7:0] RH5,
  output [7:0] RH6,
  output [7:0] RH7
);
  wire [7:0] s0;
  wire [7:0] PC;
  wire [15:0] s1;
  wire [7:0] s2;
  wire [7:0] s3;
  wire RegWrite;
  wire [2:0] s4;
  wire [2:0] s5;
  wire [2:0] s6;
  wire [7:0] DA;
  wire [7:0] s7;
  wire [11:0] s8;
  wire [3:0] OPCODE;
  wire [5:0] s9;
  wire [2:0] FUNCT;
  wire [2:0] s10;
  wire RegDst;
  wire [2:0] s11;
  wire [7:0] s12;
  wire AluSrc;
  wire [7:0] s13;
  wire [7:0] s14;
  wire [7:0] s15;
  wire MemWrite;
  wire MemRead;
  wire [7:0] s16;
  wire MemtoReg;
  wire [7:0] s17;
  wire [7:0] s18;
  wire [7:0] s19;
  wire s20;
  wire [7:0] s21;
  wire Jump;
  wire [7:0] s22;
  wire [7:0] s23;
  wire s24;
  wire [7:0] LO_temp;
  wire [7:0] s25;
  wire [7:0] HI_temp;
  wire LUI;
  wire [7:0] s26;
  wire JAL;
  wire [7:0] s27;
  wire [7:0] s28;
  wire s29;
  wire s30;
  wire s31;
  wire s32;
  wire s33;
  wire s34;
  wire s35;
  wire [17:0] s36;
  wire [3:0] AluOp;
  wire JR;
  wire MFEN;
  wire [1:0] SLIHO;
  wire [1:0] s37;
  wire s38;
  wire [7:0] RH0_t;
  wire [7:0] RH1_t;
  wire [7:0] RH2_t;
  wire [7:0] RH3_t;
  wire [7:0] RH4_t;
  wire [7:0] RH5_t;
  wire [7:0] RH6_t;
  wire [7:0] RH7_t;
  // PC
  DIG_Register_BUS #(
    .Bits(8)
  )
  DIG_Register_BUS_i0 (
    .D( s0 ),
    .C( CLK ),
    .en( CLK ),
    .Q( PC )
  );
  // Instruction Memory
  DIG_ROM_256X16_InstructionMemory DIG_ROM_256X16_InstructionMemory_i1 (
    .A( PC ),
    .sel( 1'b1 ),
    .D( s1 )
  );
  DIG_Add #(
    .Bits(8)
  )
  DIG_Add_i2 (
    .a( 8'b1 ),
    .b( PC ),
    .c_i( 1'b0 ),
    .s( s2 )
  );
  // Registers
  DIG_RegisterFile #(
    .Bits(8),
    .AddrBits(3)
  )
  DIG_RegisterFile_i3 (
    .Din( s3 ),
    .we( RegWrite ),
    .Rw( s4 ),
    .C( CLK ),
    .Ra( s5 ),
    .Rb( s6 ),
    .Da( DA ),
    .Db( s7 ),
    .RH0( RH0_t ),
    .RH1( RH1_t ),
    .RH2( RH2_t ),
    .RH3( RH3_t ),
    .RH4( RH4_t ),
    .RH5( RH5_t ),
    .RH6( RH6_t ),
    .RH7( RH7_t )
  );
  Mux_2x1_NBits #(
    .Bits(3)
  )
  Mux_2x1_NBits_i4 (
    .sel( RegDst ),
    .in_0( s10 ),
    .in_1( s6 ),
    .out( s11 )
  );
  DIG_BitExtender #(
    .inputBits(6),
    .outputBits(8)
  )
  DIG_BitExtender_i5 (
    .in( s9 ),
    .out( s12 )
  );
  Mux_2x1_NBits #(
    .Bits(8)
  )
  Mux_2x1_NBits_i6 (
    .sel( AluSrc ),
    .in_0( s7 ),
    .in_1( s13 ),
    .out( s14 )
  );
  // Data Memory
  DIG_RAMDualPort #(
    .Bits(8),
    .AddrBits(8)
  )
  DIG_RAMDualPort_i7 (
    .A( s15 ),
    .Din( s7 ),
    .str( MemWrite ),
    .C( CLK ),
    .ld( MemRead ),
    .D( s16 )
  );
  Mux_2x1_NBits #(
    .Bits(8)
  )
  Mux_2x1_NBits_i8 (
    .sel( MemtoReg ),
    .in_0( s15 ),
    .in_1( s16 ),
    .out( s17 )
  );
  DIG_Add #(
    .Bits(8)
  )
  DIG_Add_i9 (
    .a( s2 ),
    .b( s18 ),
    .c_i( 1'b0 ),
    .s( s19 )
  );
  Mux_2x1_NBits #(
    .Bits(8)
  )
  Mux_2x1_NBits_i10 (
    .sel( s20 ),
    .in_0( s2 ),
    .in_1( s19 ),
    .out( s21 )
  );
  assign s20 = (~ (s36[13] & s38) & s36[5]);
  Mux_2x1_NBits #(
    .Bits(8)
  )
  Mux_2x1_NBits_i11 (
    .sel( Jump ),
    .in_0( s21 ),
    .in_1( s22 ),
    .out( s23 )
  );
  // LO
  DIG_Register_BUS #(
    .Bits(8)
  )
  DIG_Register_BUS_i12 (
    .D( s15 ),
    .C( CLK ),
    .en( s24 ),
    .Q( LO_temp )
  );
  // HI
  DIG_Register_BUS #(
    .Bits(8)
  )
  DIG_Register_BUS_i13 (
    .D( s25 ),
    .C( CLK ),
    .en( s24 ),
    .Q( HI_temp )
  );
  Mux_2x1_NBits #(
    .Bits(8)
  )
  Mux_2x1_NBits_i14 (
    .sel( LUI ),
    .in_0( s12 ),
    .in_1( s26 ),
    .out( s13 )
  );
  Mux_2x1_NBits #(
    .Bits(3)
  )
  Mux_2x1_NBits_i15 (
    .sel( JAL ),
    .in_0( s11 ),
    .in_1( 3'b111 ),
    .out( s4 )
  );
  Mux_2x1_NBits #(
    .Bits(8)
  )
  Mux_2x1_NBits_i16 (
    .sel( JAL ),
    .in_0( s17 ),
    .in_1( s27 ),
    .out( s28 )
  );
  DIG_Add #(
    .Bits(8)
  )
  DIG_Add_i17 (
    .a( PC ),
    .b( 8'b1 ),
    .c_i( 1'b0 ),
    .s( s27 )
  );
  // CONTROLLER
  LUT_CONTROLLER LUT_CONTROLLER_i18 (
    .\0 ( s29 ),
    .\1 ( s30 ),
    .\2 ( s31 ),
    .\3 ( s32 ),
    .\4 ( s33 ),
    .\5 ( s34 ),
    .\6 ( s35 ),
    .out( s36 )
  );
  Mux_4x1_NBits #(
    .Bits(8)
  )
  Mux_4x1_NBits_i19 (
    .sel( SLIHO ),
    .in_0( s28 ),
    .in_1( s28 ),
    .in_2( HI_temp ),
    .in_3( LO_temp ),
    .out( s3 )
  );
  Mux_2x1_NBits #(
    .Bits(2)
  )
  Mux_2x1_NBits_i20 (
    .sel( MFEN ),
    .in_0( 2'b0 ),
    .in_1( s37 ),
    .out( SLIHO )
  );
  assign s24 = (1'b1 & s36[12]);
  Mux_2x1_NBits #(
    .Bits(8)
  )
  Mux_2x1_NBits_i21 (
    .sel( JR ),
    .in_0( s23 ),
    .in_1( DA ),
    .out( s0 )
  );
  assign s26[3:0] = 4'b0;
  assign s26[7:4] = s12[3:0];
  assign s18 = s12;
  // ALU
  ALU ALU_i22 (
    .ALUop( AluOp ),
    .A( DA ),
    .B( s14 ),
    .SHAMT( s5 ),
    .S( s15 ),
    .HIGH( s25 ),
    .Zero( s38 )
  );
  assign s8 = s1[11:0];
  assign OPCODE = s1[15:12];
  assign s9 = s8[5:0];
  assign s6 = s8[8:6];
  assign s5 = s8[11:9];
  assign FUNCT = s9[2:0];
  assign s10 = s9[5:3];
  assign s22 = s8[7:0];
  assign s29 = FUNCT[0];
  assign s30 = FUNCT[1];
  assign s31 = FUNCT[2];
  assign s32 = OPCODE[0];
  assign s33 = OPCODE[1];
  assign s34 = OPCODE[2];
  assign s35 = OPCODE[3];
  assign RegDst = s36[0];
  assign AluSrc = s36[1];
  assign MemtoReg = s36[2];
  assign RegWrite = s36[3];
  assign MemWrite = s36[4];
  assign Jump = s36[6];
  assign AluOp = s36[10:7];
  assign MemRead = s36[11];
  assign JR = s36[14];
  assign MFEN = s36[15];
  assign LUI = s36[16];
  assign JAL = s36[17];
  assign s37 = FUNCT[2:1];
  assign HI = HI_temp;
  assign LO = LO_temp;
  assign RH0 = RH0_t;
  assign RH1 = RH1_t;
  assign RH2 = RH2_t;
  assign RH3 = RH3_t;
  assign RH4 = RH4_t;
  assign RH5 = RH5_t;
  assign RH6 = RH6_t;
  assign RH7 = RH7_t;
endmodule

module main(
    input start,
    input CLK,
    input [3:0] CODE,
    output [7:0] RVIS
);
    wire [7:0] HI;
    wire [7:0] LO;
    wire [7:0] RH0;
    wire [7:0] RH1;
    wire [7:0] RH2;
    wire [7:0] RH3;
    wire [7:0] RH4;
    wire [7:0] RH5;
    wire [7:0] RH6;
    wire [7:0] RH7;
    reg [7:0] RFIN;

    
    reg clk_enable;
    reg start_reg;
    
    always @(posedge CLK) begin
        start_reg <= start;
        if (start_reg)
            clk_enable <= 1'b1;
        else
            clk_enable <= 1'b0;
    end

    CPU cpu_instance (
        .CLK(start_reg),
        .HI(HI),
        .LO(LO),
        .RH0(RH0),
        .RH1(RH1),
        .RH2(RH2),
        .RH3(RH3),
        .RH4(RH4),
        .RH5(RH5),
        .RH6(RH6),
        .RH7(RH7)
    );

    always @(CODE, RFIN, RH0,RH1,RH2,RH3,RH4,RH5,RH6,RH7) begin
        case (CODE)
            4'd0: RFIN = RH0;
            4'd1: RFIN = RH1;
            4'd2: RFIN = RH2;
            4'd3: RFIN = RH3;
            4'd4: RFIN = RH4;
            4'd5: RFIN = RH5;
            4'd6: RFIN = RH6;
            4'd7: RFIN = RH7;
            default: RFIN = 8'd0; // Default case to handle unexpected CODE values
        endcase
    end

    assign RVIS = RFIN;

endmodule

