// // ================== ALU =======================
// module ALU (
//     input  [31:0] A, B,
//     input        Ci,
//     input  [3:0] OP,
//     output reg [31:0] Out,
//     output reg Z, N, C, V
// );
    

//     always @(*) begin
//             Z = 0; N = 0; C = 0; V = 0;
//         case (OP)
            
//             4'b0000: {C, Out} = {1'b0, A} + {1'b0, B};           // A + B
//             4'b0001: {C, Out} = {1'b0, A} + {1'b0, B} + Ci;      // A + B + Ci
//             4'b0010: {C, Out} = {1'b0, A} - {1'b0, B};           // A - B
//             4'b0011: {C, Out} = {1'b0, A} - {1'b0, B} - Ci;      // A - B - Ci
//             4'b0100: Out = A & B;                                // AND
//             4'b0101: Out = A | B;                                // OR
//             4'b0110: Out = A ^ B;                                // XOR
//             4'b0111: Out = ~(A ^ B);                             // XNOR
//             4'b1000: Out = A & ~B;                              // A AND (NOT B)
//             4'b1001: Out = A | ~B;                              // A OR (NOT B)
//             4'b1010: Out = A << B[4:0];                         // Shift l√≥gico izq.
//             4'b1011: Out = A >> B[4:0];                         // shift l√≥gico der
//             4'b1100: Out = $signed(A) >>> B[4:0];               // shift aritm√©tico der
//             4'b1101: Out = A;                                    // pasa A
//             4'b1110: Out = B;                                   // pasa B
//             4'b1111: Out = ~B;                                   // NOT B
            
//             default: Out = 32'b0;
//         endcase

//         // ############# FLAGS ################
        
//         if (OP <= 4'd3) begin                                   //flags para OP 0 - 3 Z N V C
//             Z = (Out == 0);
//             N = Out[31];
//             if (OP[1] == 0 )                                        // suma
//                 V = (~(A[31] ^ B[31]) & (A[31] ^ Out[31]));         
//             else                                                     // resta
//                 V = ( (A[31] ^ B[31]) & (A[31] ^ Out[31]) );        //
                
//         end else if (OP >= 4'd4 && OP <= 4'd9) begin      // flags para OP 4 - 9 Z N
//             Z = (Out == 0);
//             N = Out[31];
//         end
//                                                             // OP 10 - 15 no afectan ning√∫n flag
//     end
// endmodule


// /// ============ Second Operand Handler (SOH) ==================================
// module SOH (
//     input  [31:0] R,
//     input  [21:0] Imm,
//     input  [3:0]  IS,
//     output reg [31:0] N
// );
//     always @(*) begin
//         case (IS)
//             // IS 0000‚Äì0011: Inmediato Imm concatenado con 10 ceros
//             4'b0000,
//             4'b0001,
//             4'b0010,
//             4'b0011: N = {Imm, 10'b0};

//             // IS 0100‚Äì0111: Inmediato (22‚Üí32) con extensi√≥n de signo
//             4'b0100,
//             4'b0101,
//             4'b0110,
//             4'b0111: N = {{10{Imm[21]}}, Imm};

//             // IS 1000: pasar R
//             4'b1000: N = R;

//             // IS 1001: Inmediato corto (13‚Üí32) con extensi√≥n de signo
//             4'b1001: N = {{19{Imm[12]}}, Imm[12:0]};

//             // IS 1010‚Äì1011: 5 bits para shift
//             4'b1010: N = {27'b0, R[4:0]};
//             4'b1011: N = {27'b0, Imm[4:0]};

//             // IS 1100‚Äì1111: repeticiones de R o Imm corto
//             4'b1100: N = R;
//             4'b1101: N = {{19{Imm[12]}}, Imm[12:0]};
//             4'b1110: N = R;
//             4'b1111: N = {{19{Imm[12]}}, Imm[12:0]};

//             default: N = 32'b0;
//         endcase
//     end
// endmodule

module Registro_EX_MEM(
    input        clk,
    input        R,             
    input        load_ex,
    input        rf_le_ex,
    input        E_ex,
    input  [1:0] size_ex,
    input        rw_dm_ex,

    output reg        load_mem,
    output reg        rf_le_mem,
    output reg        E_mem,
    output reg [1:0]  size_mem,
    output reg        rw_dm_mem
);
    always @(posedge clk) begin
        if (R) begin
            load_mem   <= 0;
            rf_le_mem  <= 0;
            E_mem      <= 0;
            size_mem   <= 2'b00;
            rw_dm_mem  <= 0;
        end else begin
            load_mem   <= load_ex;
            rf_le_mem  <= rf_le_ex;
            E_mem      <= E_ex;
            size_mem   <= size_ex;
            rw_dm_mem  <= rw_dm_ex;
        end
    end
endmodule