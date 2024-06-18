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
                ALU_Result = A / B;             // DIV
                HIGH = A % B;              // Remainder
            end
            4'b0110: begin
                ALU_Result = A / B;             // DIVU
                HIGH = A % B;              // Remainder
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