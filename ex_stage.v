module Control_Handler(
    input ID_JUMPL,
    input ID_BRANCH,
    input ID_CALL,
    input a,
    input [3:0] ID_COND,

    // Condition codes seleccionados por el MUX PSR/ALU
    input MUX_N, MUX_Z, MUX_V, MUX_C,

    output reg [1:0] PC_SEL,
    output reg       clr_IF,
    output reg       kill_ID 
);

always @(*) begin
    // Default
    PC_SEL = 2'b00;  // NPC
    clr_IF = 1'b0;
    kill_ID = 1'b0;

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
        kill_ID = 1'b0;
    end

    // ===========================
    // BRANCH
    // ===========================
    else if (ID_BRANCH) begin
        reg taken;
        taken = 1'b0;

    // Evaluación SPARC:
        case (ID_COND)
            4'b0000: taken = 1'b0;                          // BN
            4'b0001: taken = (MUX_Z);                       // BE
            4'b0010: taken = (MUX_Z || (MUX_N ^ MUX_V));     // BLE
            4'b0011: taken = (MUX_N ^ MUX_V);                // BL
            4'b0100: taken = (MUX_C || MUX_Z);               // BLEU
            4'b0101: taken = (MUX_C);                        // BCS
            4'b0110: taken = (MUX_N);                        // BNEG
            4'b0111: taken = (MUX_V);                        // BVS
            4'b1000: taken = 1'b1;                           // BA
            4'b1001: taken = (!MUX_Z);                       // BNE
            4'b1010: taken = (!MUX_Z && !(MUX_N ^ MUX_V));    // BG
            4'b1011: taken = (!(MUX_N ^ MUX_V));              // BGE
            4'b1100: taken = (!MUX_Z && !MUX_C);              // BGU
            4'b1101: taken = (!MUX_C);                        // BCC
            4'b1110: taken = (!MUX_N);                        // BPOS
            4'b1111: taken = (!MUX_V);                        // BVC
            default: taken = 1'b0;
        endcase

        if (taken) begin
            // Branch taken: redirige a TA y flush la instrucción después del delay-slot
            PC_SEL = 2'b01;
            clr_IF = 1'b1;
            kill_ID = 1'b0;   // OJO: el delay-slot SÍ se ejecuta cuando taken
        end else begin
            // Branch NOT taken:
            // Si a=1 (annul), entonces el delay-slot debe convertirse en NOP (matar ID->EX)
            PC_SEL = 2'b00;
            clr_IF = 1'b0;
            kill_ID = a;      // <<< ANNUL: mata el delay-slot
        end
    end
end

endmodule

module MUX_EX_ICC(
    input ALU_Z, ALU_N, ALU_V, ALU_C,
    input PSR_Z, PSR_N, PSR_V, PSR_C,
    input EX_WE_PSR,
    output reg CH_Z, CH_N, CH_V, CH_C
);
   always @(*) begin
    if (EX_WE_PSR) begin
        // flags recién calculados (subcc, addcc, etc.)
        CH_Z = ALU_Z;
        CH_N = ALU_N;
        CH_V = ALU_V;
        CH_C = ALU_C;
    end else begin
        // flags ya guardados en el PSR
        CH_Z = PSR_Z;
        CH_N = PSR_N;
        CH_V = PSR_V;
        CH_C = PSR_C;
    end
end
endmodule

module Program_Status_Register(
    input WE_PSR, clk,
    input ALU_Z, ALU_N, ALU_V, ALU_C,
    output reg PSR_Z, PSR_N, PSR_V, PSR_C   
);
    initial begin 
        PSR_Z = 1'b0;
        PSR_N = 1'b0;
        PSR_V = 1'b0;
        PSR_C = 1'b0;
    end


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

    input  [1:0] EX_LOAD,          // <-- NUEVO: tipo de “writeback source” en EX

    output reg   LE_IF,
    output reg   NOP_STALL,
    output reg [1:0] SEL_A,
    output reg [1:0] SEL_B,
    output reg [1:0] SEL_C
);

    // =====================================
    // STALL: SOLO para LOAD-USE (1 ciclo)
    // EX_LOAD==2'b01 significa que el valor viene de MEM (load)
    // Si el RD de ese load se usa inmediatamente en RA/RB/RC => stall + burbuja
    // =====================================
    always @(*) begin
        if (EX_RF_LE && (EX_LOAD == 2'b01) && (EX_RD != 0) &&
            ((EX_RD == RA) || (EX_RD == RB) || (EX_RD == RC))) begin
            LE_IF     = 1'b0;   // congela PC, NPC, IF/ID
            NOP_STALL = 1'b1;   // mete burbuja en ID/EX (vía MUX_ID_STALL)
        end else begin
            LE_IF     = 1'b1;
            NOP_STALL = 1'b0;
        end
    end

    // =====================================
    // FORWARDING FOR RA
    // =====================================
    always @(*) begin
        SEL_A = 2'b00;
        if (EX_RF_LE && (EX_RD == RA) && (EX_RD != 0))
            SEL_A = 2'b01;
        else if (MEM_RF_LE && (MEM_RD == RA) && (MEM_RD != 0))
            SEL_A = 2'b10;
        else if (WB_RF_LE && (WB_RD == RA) && (WB_RD != 0))
            SEL_A = 2'b11;
    end

    // =====================================
    // FORWARDING FOR RB
    // =====================================
    always @(*) begin
        SEL_B = 2'b00;
        if (EX_RF_LE && (EX_RD == RB) && (EX_RD != 0))
            SEL_B = 2'b01;
        else if (MEM_RF_LE && (MEM_RD == RB) && (MEM_RD != 0))
            SEL_B = 2'b10;
        else if (WB_RF_LE && (WB_RD == RB) && (WB_RD != 0))
            SEL_B = 2'b11;
    end

    // =====================================
    // FORWARDING FOR RC
    // =====================================
    always @(*) begin
        SEL_C = 2'b00;
        if (EX_RF_LE && (EX_RD == RC) && (EX_RD != 0))
            SEL_C = 2'b01;
        else if (MEM_RF_LE && (MEM_RD == RC) && (MEM_RD != 0))
            SEL_C = 2'b10;
        else if (WB_RF_LE && (WB_RD == RC) && (WB_RD != 0))
            SEL_C = 2'b11;
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
    input [1:0]   load_ex,
    input        rf_le_ex,
    input        E_ex,
    input  [1:0] size_ex,
    input        rw_dm_ex,
    input        se_ex,
    input[31:0] alu_out_ex,
    input [31:0] PC_D_ex,
    input [31:0] ex_sethi_imm22,
    output reg [31:0] mem_sethi_imm22,

    output reg [31:0] df_a_mem,
    output reg [1:0]  load_mem,
    output reg        rf_le_mem,
    output reg [4:0]  mem_rd,
    output reg        E_mem,
    output reg [1:0]  size_mem,
    output reg        rw_dm_mem,
    output reg        se_mem,
    output reg [31:0] alu_out_mem,
    output reg [31:0] PC_D_mem
);
    always @(posedge clk) begin
        if (R) begin
            mem_sethi_imm22 <= 32'b0;
            load_mem   <= 2'b00;
            rf_le_mem  <= 0;
            E_mem      <= 0;
            size_mem   <= 2'b00;
            rw_dm_mem  <= 0;
            se_mem     <= 1'b0;
            mem_rd     <= 5'b0;
            alu_out_mem<= 32'b0;
            PC_D_mem   <= 32'b0;
        end else begin
            mem_sethi_imm22 <= ex_sethi_imm22;
            load_mem   <= load_ex;
            rf_le_mem  <= rf_le_ex;
            E_mem      <= E_ex;
            size_mem   <= size_ex;
            rw_dm_mem  <= rw_dm_ex;
            se_mem     <= se_ex;
            mem_rd     <= ex_rd;
            alu_out_mem<= alu_out_ex;
            PC_D_mem   <= PC_D_ex;
        end
    end
endmodule