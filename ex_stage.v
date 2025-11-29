module Control_Handler(
    input ID_JUMPL,
    input ID_BRANCH,
    input ID_CALL,
    input a,
    input [3:0] ID_COND,

    // Condition codes seleccionados por el MUX PSR/ALU
    input MUX_N, MUX_Z, MUX_V, MUX_C,

    output reg [1:0] PC_SEL,
    output reg       clr_IF
);

always @(*) begin
    // Default
    PC_SEL = 2'b00;  // NPC
    clr_IF = 1'b0;

    // ===========================
    // JMPL
    // ===========================
    if (ID_JUMPL) begin
        PC_SEL = 2'b10;   // ALU_OUT
        clr_IF = 1'b1;
    end

    // ===========================
    // CALL
    // ===========================
    else if (ID_CALL) begin
        PC_SEL = 2'b01;   // TA
        clr_IF = 1'b1;
    end

    // ===========================
    // BRANCH
    // ===========================
    else if (ID_BRANCH) begin
        // Evaluación SPARC real:
        // N, Z, V, C
        case (ID_COND)
            4'b0000: begin end                      // BN → nunca brincar
            4'b0001: if (MUX_Z) begin PC_SEL=2'b01; clr_IF=1; end     // BE
            4'b0010: if (MUX_Z || (MUX_N ^ MUX_V)) begin PC_SEL=2'b01; clr_IF=1; end  // BLE
            4'b0011: if (MUX_N ^ MUX_V) begin PC_SEL=2'b01; clr_IF=1; end // BL
            4'b0100: if (MUX_C || MUX_Z) begin PC_SEL=2'b01; clr_IF=1; end // BLEU
            4'b0101: if (MUX_C) begin PC_SEL=2'b01; clr_IF=1; end          // BCS
            4'b0110: if (MUX_N) begin PC_SEL=2'b01; clr_IF=1; end          // BNEG
            4'b0111: if (MUX_V) begin PC_SEL=2'b01; clr_IF=1; end          // BVS
            4'b1000: begin PC_SEL=2'b01; clr_IF=1; end                     // BA (always)
            4'b1001: if (!MUX_Z) begin PC_SEL=2'b01; clr_IF=1; end         // BNE
            4'b1010: if (!MUX_Z && !(MUX_N ^ MUX_V)) begin PC_SEL=2'b01; clr_IF=1; end // BG
            4'b1011: if (!(MUX_N ^ MUX_V)) begin PC_SEL=2'b01; clr_IF=1; end // BGE
            4'b1100: if (!MUX_Z && !MUX_C) begin PC_SEL=2'b01; clr_IF=1; end // BGU
            4'b1101: if (!MUX_C) begin PC_SEL=2'b01; clr_IF=1; end          // BCC
            4'b1110: if (!MUX_N) begin PC_SEL=2'b01; clr_IF=1; end          // BPOS
            4'b1111: if (!MUX_V) begin PC_SEL=2'b01; clr_IF=1; end          // BVC
        endcase
    end

end

endmodule


// module MUX_ICC(
//     input ALU_Z, ALU_N, ALU_V, ALU_C,
//     input PSR_Z, PSR_N, PSR_V, PSR_C,
//     input ID_WE_PSR, 
//     output reg MUX_Z, MUX_N, MUX_V, MUX_C
// );
//     always @(*) begin
//         if (ID_WE_PSR) begin
//             MUX_Z = PSR_Z;
//             MUX_N = PSR_N;
//             MUX_V = PSR_V;
//             MUX_C = PSR_C;
//         end else begin
//             MUX_Z = ALU_Z;
//             MUX_N = ALU_N;
//             MUX_V = ALU_V;
//             MUX_C = ALU_C;
//         end
//     end
// endmodule

module MUX_EX_ICC(
    input ALU_Z, ALU_N, ALU_V, ALU_C,
    input PSR_Z, PSR_N, PSR_V, PSR_C,
    input EX_WE_PSR,
    output reg CH_Z, CH_N, CH_V, CH_C
);
    always @(*) begin
        if (EX_WE_PSR) begin
            CH_Z = PSR_Z;
            CH_N = PSR_N;
            CH_V = PSR_V;
            CH_C = PSR_C;
        end else begin
            CH_Z = ALU_Z;
            CH_N = ALU_N;
            CH_V = ALU_V;
            CH_C = ALU_C;
        end
    end
endmodule

module Program_Status_Register(
    input WE_PSR, clk,
    input ALU_Z, ALU_N, ALU_V, ALU_C,
    output reg PSR_Z, PSR_N, PSR_V, PSR_C   
);
    always @(posedge clk) begin
        if (WE_PSR) begin
            PSR_Z = ALU_Z;
            PSR_N = ALU_N;
            PSR_V = ALU_V;
            PSR_C = ALU_C;
        end
    end
endmodule

module MUX_ALU_CALL(
    input [31:0] ALU_OUT,
    input [31:0] PC_D,
    input EX_CALL,
    output reg [31:0] MUX_OUT
);
    always @(*) begin
        if (EX_CALL) begin
            MUX_OUT = PC_D;
        end else begin
            MUX_OUT = ALU_OUT;
        end
    end
endmodule

module Data_Hazard_Detection_Unit(
    input  [4:0] RA, RB, RC,
    input  [4:0] EX_RD, MEM_RD, WB_RD,
    input        EX_RF_LE, MEM_RF_LE, WB_RF_LE,
    input        EX_LOAD,

    output reg   LE_IF,
    output reg   NOP_STALL,
    output reg [1:0] SEL_A,
    output reg [1:0] SEL_B,
    output reg [1:0] SEL_C
);

// ==========================
// FORWARDING FOR RA
// ==========================
always @(*) begin
    // Default
    SEL_A = 2'b00;

    if (EX_RF_LE && (EX_RD == RA) && (EX_RD != 0))
        SEL_A = 2'b01;  // EX forwarding
    else if (MEM_RF_LE && (MEM_RD == RA) && (MEM_RD != 0))
        SEL_A = 2'b10;  // MEM forwarding
    else if (WB_RF_LE && (WB_RD == RA) && (WB_RD != 0))
        SEL_A = 2'b11;  // WB forwarding
end

// ==========================
// FORWARDING FOR RB
// ==========================
always @(*) begin
    SEL_B = 2'b00;

    if (EX_RF_LE && (EX_RD == RB) && (EX_RD != 0))
        SEL_B = 2'b01;
    else if (MEM_RF_LE && (MEM_RD == RB) && (MEM_RD != 0))
        SEL_B = 2'b10;
    else if (WB_RF_LE && (WB_RD == RB) && (WB_RD != 0))
        SEL_B = 2'b11;
end

// ==========================
// FORWARDING FOR RC
// ==========================
always @(*) begin
    SEL_C = 2'b00;

    if (EX_RF_LE && (EX_RD == RC) && (EX_RD != 0))
        SEL_C = 2'b01;
    else if (MEM_RF_LE && (MEM_RD == RC) && (MEM_RD != 0))
        SEL_C = 2'b10;
    else if (WB_RF_LE && (WB_RD == RC) && (WB_RD != 0))
        SEL_C = 2'b11;
end

// ==========================
// LOAD-USE HAZARD STALL
// ==========================
always @(*) begin
    if (EX_LOAD &&
        ((EX_RD == RA) || (EX_RD == RB) || (EX_RD == RC)) &&
        (EX_RD != 0))
    begin
        LE_IF     = 1'b0;   // stop IF
        NOP_STALL = 1'b1;   // insert NOP in ID/EX
    end
    else begin
        LE_IF     = 1'b1;
        NOP_STALL = 1'b0;
    end
end

endmodule
 

module Arithmetic_Logic_Unit (
    input  [31:0] A, B,
    input        Ci,
    input  [3:0] OP,
    output reg [31:0] Out,
    output reg Z, N, C, V
);
    

    always @(*) begin
            Z = 0; N = 0; C = 0; V = 0;
        case (OP)
            
            4'b0000: {C, Out} = {1'b0, A} + {1'b0, B};           // A + B
            4'b0001: {C, Out} = {1'b0, A} + {1'b0, B} + Ci;      // A + B + Ci
            4'b0010: {C, Out} = {1'b0, A} - {1'b0, B};           // A - B
            4'b0011: {C, Out} = {1'b0, A} - {1'b0, B} - Ci;      // A - B - Ci
            4'b0100: Out = A & B;                                // AND
            4'b0101: Out = A | B;                                // OR
            4'b0110: Out = A ^ B;                                // XOR
            4'b0111: Out = ~(A ^ B);                             // XNOR
            4'b1000: Out = A & ~B;                              // A AND (NOT B)
            4'b1001: Out = A | ~B;                              // A OR (NOT B)
            4'b1010: Out = A << B[4:0];                         // Shift l√≥gico izq.
            4'b1011: Out = A >> B[4:0];                         // shift l√≥gico der
            4'b1100: Out = $signed(A) >>> B[4:0];               // shift aritm√©tico der
            4'b1101: Out = A;                                    // pasa A
            4'b1110: Out = B;                                   // pasa B
            4'b1111: Out = ~B;                                   // NOT B
            
            default: Out = 32'b0;
        endcase

        // ############# FLAGS ################
        
        if (OP <= 4'd3) begin                                   //flags para OP 0 - 3 Z N V C
            Z = (Out == 0);
            N = Out[31];
            if (OP[1] == 0 )                                        // suma
                V = (~(A[31] ^ B[31]) & (A[31] ^ Out[31]));         
            else                                                     // resta
                V = ( (A[31] ^ B[31]) & (A[31] ^ Out[31]) );        //
                
        end else if (OP >= 4'd4 && OP <= 4'd9) begin      // flags para OP 4 - 9 Z N
            Z = (Out == 0);
            N = Out[31];
        end
                                                            // OP 10 - 15 no afectan ning√∫n flag
    end
endmodule


/// ============ Second Operand Handler (SOH) ==================================
module Second_Operand_Handler (
    input  [31:0] R,
    input  [21:0] Imm,
    input  [3:0]  IS,
    output reg [31:0] N
);
    always @(*) begin
        case (IS)
            // IS 0000‚Äì0011: Inmediato Imm concatenado con 10 ceros
            4'b0000,
            4'b0001,
            4'b0010,
            4'b0011: N = {Imm, 10'b0};

            // IS 0100‚Äì0111: Inmediato (22‚Üí32) con extensi√≥n de signo
            4'b0100,
            4'b0101,
            4'b0110,
            4'b0111: N = {{10{Imm[21]}}, Imm};

            // IS 1000: pasar R
            4'b1000: N = R;

            // IS 1001: Inmediato corto (13‚Üí32) con extensi√≥n de signo
            4'b1001: N = {{19{Imm[12]}}, Imm[12:0]};

            // IS 1010‚Äì1011: 5 bits para shift
            4'b1010: N = {27'b0, R[4:0]};
            4'b1011: N = {27'b0, Imm[4:0]};

            // IS 1100‚Äì1111: repeticiones de R o Imm corto
            4'b1100: N = R;
            4'b1101: N = {{19{Imm[12]}}, Imm[12:0]};
            4'b1110: N = R;
            4'b1111: N = {{19{Imm[12]}}, Imm[12:0]};

            default: N = 32'b0;
        endcase
    end
endmodule

module Registro_EX_MEM(
    input        clk,
    input        R,      
    input [4:0]  ex_rd,   
    input        load_ex,
    input        rf_le_ex,
    input        E_ex,
    input  [1:0] size_ex,
    input        rw_dm_ex,
    input[31:0] alu_out_ex,
    input [31:0] PC_D_ex,

    output reg [31:0] df_a_mem,
    output reg        load_mem,
    output reg        rf_le_mem,
    output reg [4:0]  mem_rd,
    output reg        E_mem,
    output reg [1:0]  size_mem,
    output reg        rw_dm_mem,
    output reg [31:0] alu_out_mem,
    output reg [31:0] PC_D_mem
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