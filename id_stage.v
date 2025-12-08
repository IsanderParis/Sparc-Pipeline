
module Target_Address_Generator (
    input  [31:0] PC,
    input  [29:0] OFFSET,   // offset ya procesado en ID
    input         isCALL,
    input         isBRANCH,
    output reg [31:0] TA
);

    always @(*) begin
        if (isCALL)
            TA = PC + OFFSET;      // OFFSET = disp30 << 2
        else if (isBRANCH)
            TA = PC + OFFSET;      // OFFSET = disp22 << 2
        else
            TA = 32'b0;
    end

endmodule


// registro de 32 bits
module reg32 (
    input  wire        clk,
    input  wire        ld,     // load enable
    input  wire [31:0] D,
    output reg  [31:0] Q
);
    always @(posedge clk) begin
        if (ld)
            Q <= D;
    end
endmodule

// decoder de 5 a 32
module dec5to32 (
    input  wire [4:0]  in,
    input  wire        en,
    output reg  [31:0] out
);
    always @(*) begin
        out = 32'b0;
        if (en) begin
            out[in] = 1'b1;
        end
    end
endmodule

// mux de 32 a 1
module mux32_1_32bit (
    input  wire [31:0] in0,
    input  wire [31:0] in1,
    input  wire [31:0] in2,
    input  wire [31:0] in3,
    input  wire [31:0] in4,
    input  wire [31:0] in5,
    input  wire [31:0] in6,
    input  wire [31:0] in7,
    input  wire [31:0] in8,
    input  wire [31:0] in9,
    input  wire [31:0] in10,
    input  wire [31:0] in11,
    input  wire [31:0] in12,
    input  wire [31:0] in13,
    input  wire [31:0] in14,
    input  wire [31:0] in15,
    input  wire [31:0] in16,
    input  wire [31:0] in17,
    input  wire [31:0] in18,
    input  wire [31:0] in19,
    input  wire [31:0] in20,
    input  wire [31:0] in21,
    input  wire [31:0] in22,
    input  wire [31:0] in23,
    input  wire [31:0] in24,
    input  wire [31:0] in25,
    input  wire [31:0] in26,
    input  wire [31:0] in27,
    input  wire [31:0] in28,
    input  wire [31:0] in29,
    input  wire [31:0] in30,
    input  wire [31:0] in31,
    input  wire [4:0]  sel,
    output reg  [31:0] out
);
    always @(*) begin
        case (sel)
            5'd0:  out = in0;
            5'd1:  out = in1;
            5'd2:  out = in2;
            5'd3:  out = in3;
            5'd4:  out = in4;
            5'd5:  out = in5;
            5'd6:  out = in6;
            5'd7:  out = in7;
            5'd8:  out = in8;
            5'd9:  out = in9;
            5'd10: out = in10;
            5'd11: out = in11;
            5'd12: out = in12;
            5'd13: out = in13;
            5'd14: out = in14;
            5'd15: out = in15;
            5'd16: out = in16;
            5'd17: out = in17;
            5'd18: out = in18;
            5'd19: out = in19;
            5'd20: out = in20;
            5'd21: out = in21;
            5'd22: out = in22;
            5'd23: out = in23;
            5'd24: out = in24;
            5'd25: out = in25;
            5'd26: out = in26;
            5'd27: out = in27;
            5'd28: out = in28;
            5'd29: out = in29;
            5'd30: out = in30;
            5'd31: out = in31;
            default: out = 32'b0;
        endcase
    end
endmodule

// register file 32x32, 3 puertos
module Register_File (
    input  wire        clock,
    input  wire        LE,        // load enable global
    input  wire [4:0]  RW,        // direcci√≥n de escritura
    input  wire [4:0]  RA,        // direcci√≥n puerto A
    input  wire [4:0]  RB,        // direcci√≥n puerto B
    input  wire [4:0]  RD,        // direcci√≥n puerto D
    input  wire [31:0] PW,        // datos de escritura

    output wire [31:0] PA,        // puerto de lectura A
    output wire [31:0] PB,        // puerto de lectura B
    output wire [31:0] PD         // puerto de lectura D
);

    // Se√±ales de carga a los registros (salida del decoder)
    wire [31:0] ld_raw;
    wire [31:0] ld;

    // Salidas de los 32 registros
    wire [31:0] q0,  q1,  q2,  q3,  q4,  q5,  q6,  q7;
    wire [31:0] q8,  q9,  q10, q11, q12, q13, q14, q15;
    wire [31:0] q16, q17, q18, q19, q20, q21, q22, q23;
    wire [31:0] q24, q25, q26, q27, q28, q29, q30, q31;

    // Decoder 5->32
    dec5to32 DEC (
        .in (RW),
        .en (LE),
        .out(ld_raw)
    );

    // Asegurar que el registro 0 no se pueda escribir NUNCA
    assign ld = { ld_raw[31:1], 1'b0 };

    // Registro 0: siempre 0
    assign q0 = 32'b0;

    // Registros 1 a 31
    reg32 R1  (.clk(clock), .ld(ld[1]),  .D(PW), .Q(q1));
    reg32 R2  (.clk(clock), .ld(ld[2]),  .D(PW), .Q(q2));
    reg32 R3  (.clk(clock), .ld(ld[3]),  .D(PW), .Q(q3));
    reg32 R4  (.clk(clock), .ld(ld[4]),  .D(PW), .Q(q4));
    reg32 R5  (.clk(clock), .ld(ld[5]),  .D(PW), .Q(q5));
    reg32 R6  (.clk(clock), .ld(ld[6]),  .D(PW), .Q(q6));
    reg32 R7  (.clk(clock), .ld(ld[7]),  .D(PW), .Q(q7));
    reg32 R8  (.clk(clock), .ld(ld[8]),  .D(PW), .Q(q8));
    reg32 R9  (.clk(clock), .ld(ld[9]),  .D(PW), .Q(q9));
    reg32 R10 (.clk(clock), .ld(ld[10]), .D(PW), .Q(q10));
    reg32 R11 (.clk(clock), .ld(ld[11]), .D(PW), .Q(q11));
    reg32 R12 (.clk(clock), .ld(ld[12]), .D(PW), .Q(q12));
    reg32 R13 (.clk(clock), .ld(ld[13]), .D(PW), .Q(q13));
    reg32 R14 (.clk(clock), .ld(ld[14]), .D(PW), .Q(q14));
    reg32 R15 (.clk(clock), .ld(ld[15]), .D(PW), .Q(q15));
    reg32 R16 (.clk(clock), .ld(ld[16]), .D(PW), .Q(q16));
    reg32 R17 (.clk(clock), .ld(ld[17]), .D(PW), .Q(q17));
    reg32 R18 (.clk(clock), .ld(ld[18]), .D(PW), .Q(q18));
    reg32 R19 (.clk(clock), .ld(ld[19]), .D(PW), .Q(q19));
    reg32 R20 (.clk(clock), .ld(ld[20]), .D(PW), .Q(q20));
    reg32 R21 (.clk(clock), .ld(ld[21]), .D(PW), .Q(q21));
    reg32 R22 (.clk(clock), .ld(ld[22]), .D(PW), .Q(q22));
    reg32 R23 (.clk(clock), .ld(ld[23]), .D(PW), .Q(q23));
    reg32 R24 (.clk(clock), .ld(ld[24]), .D(PW), .Q(q24));
    reg32 R25 (.clk(clock), .ld(ld[25]), .D(PW), .Q(q25));
    reg32 R26 (.clk(clock), .ld(ld[26]), .D(PW), .Q(q26));
    reg32 R27 (.clk(clock), .ld(ld[27]), .D(PW), .Q(q27));
    reg32 R28 (.clk(clock), .ld(ld[28]), .D(PW), .Q(q28));
    reg32 R29 (.clk(clock), .ld(ld[29]), .D(PW), .Q(q29));
    reg32 R30 (.clk(clock), .ld(ld[30]), .D(PW), .Q(q30));
    reg32 R31 (.clk(clock), .ld(ld[31]), .D(PW), .Q(q31));

    // MUX para puerto A (PA)
    mux32_1_32bit MUX_A (
        .in0(q0),   .in1(q1),   .in2(q2),   .in3(q3),
        .in4(q4),   .in5(q5),   .in6(q6),   .in7(q7),
        .in8(q8),   .in9(q9),   .in10(q10), .in11(q11),
        .in12(q12), .in13(q13), .in14(q14), .in15(q15),
        .in16(q16), .in17(q17), .in18(q18), .in19(q19),
        .in20(q20), .in21(q21), .in22(q22), .in23(q23),
        .in24(q24), .in25(q25), .in26(q26), .in27(q27),
        .in28(q28), .in29(q29), .in30(q30), .in31(q31),
        .sel(RA),
        .out(PA)
    );

    // MUX para puerto B (PB)
    mux32_1_32bit MUX_B (
        .in0(q0),   .in1(q1),   .in2(q2),   .in3(q3),
        .in4(q4),   .in5(q5),   .in6(q6),   .in7(q7),
        .in8(q8),   .in9(q9),   .in10(q10), .in11(q11),
        .in12(q12), .in13(q13), .in14(q14), .in15(q15),
        .in16(q16), .in17(q17), .in18(q18), .in19(q19),
        .in20(q20), .in21(q21), .in22(q22), .in23(q23),
        .in24(q24), .in25(q25), .in26(q26), .in27(q27),
        .in28(q28), .in29(q29), .in30(q30), .in31(q31),
        .sel(RB),
        .out(PB)
    );

    // MUX para puerto D (PD)
    mux32_1_32bit MUX_D (
        .in0(q0),   .in1(q1),   .in2(q2),   .in3(q3),
        .in4(q4),   .in5(q5),   .in6(q6),   .in7(q7),
        .in8(q8),   .in9(q9),   .in10(q10), .in11(q11),
        .in12(q12), .in13(q13), .in14(q14), .in15(q15),
        .in16(q16), .in17(q17), .in18(q18), .in19(q19),
        .in20(q20), .in21(q21), .in22(q22), .in23(q23),
        .in24(q24), .in25(q25), .in26(q26), .in27(q27),
        .in28(q28), .in29(q29), .in30(q30), .in31(q31),
        .sel(RD),
        .out(PD)
    );

endmodule

module MUX_DF_PA(
    input [31:0] DF_PA,
    input [31:0] DF_A_ALU,
    input [31:0] DF_A_MEM,
    input [31:0] DF_A_WB,
    input [1:0]  DF_Sel_A,
    output reg [31:0] MUX_A_OUT

);
always @(*) begin
    case (DF_Sel_A)
        2'b00: MUX_A_OUT = DF_PA;
        2'b01: MUX_A_OUT = DF_A_ALU;
        2'b10: MUX_A_OUT = DF_A_MEM;
        2'b11: MUX_A_OUT = DF_A_WB;
        default: MUX_A_OUT = 32'b0;
    endcase
end

endmodule

module MUX_DF_PB(
    input [31:0] DF_PB,
    input [31:0] DF_B_ALU,
    input [31:0] DF_B_MEM,
    input [31:0] DF_B_WB,
    input [1:0]  DF_Sel_B,
    output reg [31:0] MUX_B_OUT

);
always @(*) begin
    case (DF_Sel_B)
        2'b00: MUX_B_OUT = DF_PB;
        2'b01: MUX_B_OUT = DF_B_ALU;
        2'b10: MUX_B_OUT = DF_B_MEM;
        2'b11: MUX_B_OUT = DF_B_WB;
        default: MUX_B_OUT = 32'b0;
    endcase
end
endmodule

module MUX_DF_PC_D(
    input [31:0] DF_PC_D,
    input [31:0] DF_C_ALU,
    input [31:0] DF_C_MEM,
    input [31:0] DF_C_WB,
    input [1:0]  DF_Sel_C,
    output reg[31:0] MUX_C_OUT

);
always @(*) begin
    case (DF_Sel_C)
        2'b00: MUX_C_OUT = DF_PC_D;
        2'b01: MUX_C_OUT = DF_C_ALU;
        2'b10: MUX_C_OUT = DF_C_MEM;
        2'b11: MUX_C_OUT = DF_C_WB;
        default: MUX_C_OUT = 32'b0;
    endcase
end
endmodule

module MUX_CALL(
    input [4:0] rd,
    input       isCALL,
    output reg [4:0] MUX_RD_OUT
);
always @(*) begin
    if (isCALL)
        MUX_RD_OUT = 5'd15; // R15 para CALL
    else
        MUX_RD_OUT = rd;
end

endmodule

module CU_ID(
    input  [31:0] instruction,

    output reg [3:0] ID_ALU_OP_out,
    output reg [3:0] ID_SOH_OP_out,
    output reg       ID_LOAD_out,
    output reg       ID_a_out,
    output reg       ID_RF_LE_out,
    output reg       ID_CALL_out,
    output reg       ID_WE_PSR_out,
    output reg       ID_E_out,
    output reg [1:0] ID_SIZE_out,
    output reg       ID_RW_DM_out,
    output reg [63:0] keyword,
    output reg       ID_JUMPL_out,
    output reg       ID_BRANCH_out
    output reg       ID_SE_out

);

    // ==========
    // Campos ISA
    // ==========
    wire [1:0] op    = instruction[31:30];
    wire       bit_a = instruction[29];
    wire [3:0] cond  = instruction[28:25];
    wire [2:0] op2   = instruction[24:22];   // formato 2 (SETHI / Bicc)
    wire [5:0] op3   = instruction[24:19];   // formato 3 (ALU / MEM / JMPL)
    wire       bit_i = instruction[13];

    always @(*) begin
        // ==========================
        // Defaults (NOP / UNKNOWN)
        // ==========================
        ID_ALU_OP_out   = 4'b0000;
        ID_SOH_OP_out   = 4'b0000;
        ID_LOAD_out     = 1'b0;
        ID_a_out        = 1'b0;
        ID_RF_LE_out    = 1'b0;
        ID_CALL_out     = 1'b0;
        ID_WE_PSR_out   = 1'b0;
        ID_E_out        = 1'b0;
        ID_SIZE_out     = 2'b00;           // WORD por defecto
        ID_RW_DM_out    = 1'b0;            // 0 = LOAD, 1 = STORE
        ID_JUMPL_out    = 1'b0;
        ID_BRANCH_out   = 1'b0;
        keyword         = "UNKNOWN ";
        ID_SE_out       = 1'b0;            // unsigned por defecto

        // ==========================
        // NOP = instrucciòn 0
        // ==========================
        if (instruction == 32'b0) begin
            keyword = "NOP     ";
        end
        else begin
            // Decodificaci√≥n por OP
            case (op)

                // OP = 01 ‚Üí CALL (formato 1)
                2'b01: begin
                    ID_CALL_out   = 1'b1;        // es un CALL
                    ID_RF_LE_out  = 1'b1;        // escribe en R15
                    ID_SOH_OP_out = 4'b0000;     
                    keyword       = "CALL    ";
                end

                // OP = 00 ‚Üí SETHI / Branch condicional
                2'b00: begin
                    case (op2)

                        // -------- SETHI -------- (op2 = 100)
                        3'b100: begin
                            ID_RF_LE_out  = 1'b1;
                            ID_SOH_OP_out = 4'b0000;   
                            keyword       = "SETHI   ";
                        end

                        // -------- Bicc (branch entero) -------- (op2 = 010)
                        3'b010: begin
                            ID_BRANCH_out = 1'b1;
                            ID_a_out      = bit_a;     
                            ID_SOH_OP_out = 4'b0000;   
                            // decodificar condici√≥n
                            case (cond)
                                4'b0000: keyword = "BN      "; // never
                                4'b0001: keyword = "BE      "; // Z
                                4'b0010: keyword = "BLE     "; // Z OR (N XOR V)
                                4'b0011: keyword = "BL      "; // N XOR V
                                4'b0100: keyword = "BLEU    "; // C OR Z
                                4'b0101: keyword = "BCS     "; // C
                                4'b0110: keyword = "BNEG    "; // N
                                4'b0111: keyword = "BVS     "; // V
                                4'b1000: keyword = "BA      "; // always
                                4'b1001: keyword = "BNE     "; // NOT Z
                                4'b1010: keyword = "BG      "; // NOT(Z OR (N XOR V))
                                4'b1011: keyword = "BGE     "; // NOT(N XOR V)
                                4'b1100: keyword = "BGU     "; // NOT (C OR Z)
                                4'b1101: keyword = "BCC     "; // NOT C
                                4'b1110: keyword = "BPOS    "; // NOT N
                                4'b1111: keyword = "BVC     "; // NOT V
                                default: keyword = "BRANCH  ";
                            endcase
                        end

                        default: begin
                            // otros op2 de op=00 no implementados
                            keyword = "OP00UNK ";
                        end
                    endcase
                end

                // OP = 10 ‚Üí ALU / SHIFT / JMPL (formato 3)
                2'b10: begin
                
                    ID_SOH_OP_out = bit_i ? 4'b1101 : 4'b1000;
                    ID_RF_LE_out  = 1'b1;        // casi todas escriben en rd

                    case (op3)

                        // ========== ADD ==========
                        6'b000000: begin
                            ID_ALU_OP_out = 4'b0000;
                            keyword       = "ADD     ";
                        end

                        // ========== ADDcc ==========
                        6'b010000: begin
                            ID_ALU_OP_out = 4'b0000;
                            ID_WE_PSR_out = 1'b1;
                            keyword       = "ADDCC   ";
                        end

                        // ========== SUB ==========
                        6'b000100: begin
                            ID_ALU_OP_out = 4'b0010;
                            keyword       = "SUB     ";
                        end

                        // ========== SUBcc ==========
                        6'b010100: begin
                            ID_ALU_OP_out = 4'b0010;
                            ID_WE_PSR_out = 1'b1;
                            keyword       = "SUBCC   ";
                        end

                        // ========== ADDX ==========
                        6'b001000: begin
                            ID_ALU_OP_out = 4'b0001;  // A + B + Ci
                            keyword       = "ADDX    ";
                        end

                        // ========== ADDXcc ==========
                        6'b011000: begin
                            ID_ALU_OP_out = 4'b0001;  // A + B + Ci
                            ID_WE_PSR_out = 1'b1;
                            keyword       = "ADDXCC  ";
                        end

                        // ========== SUBX ==========
                        6'b001001: begin
                            ID_ALU_OP_out = 4'b0011;  // A - B - Ci
                            keyword       = "SUBX    ";
                        end

                        // ========== SUBXcc ==========
                        6'b011001: begin
                            ID_ALU_OP_out = 4'b0011;
                            ID_WE_PSR_out = 1'b1;
                            keyword       = "SUBXCC  ";
                        end

                        // ========== TADDcc ==========
                        6'b100000: begin
                            ID_ALU_OP_out = 4'b0000;  // suma normal
                            ID_WE_PSR_out = 1'b1;     // actualiza cc (trap se maneja fuera)
                            keyword       = "TADDCC  ";
                        end

                        // ========== TADDccTV ==========
                        6'b100010: begin
                            ID_ALU_OP_out = 4'b0000;
                            ID_WE_PSR_out = 1'b1;
                            keyword       = "TADDCCTV";
                        end

                        // ========== TSUBcc ==========
                        6'b100001: begin
                            ID_ALU_OP_out = 4'b0010;
                            ID_WE_PSR_out = 1'b1;
                            keyword       = "TSUBCC  ";
                        end

                        // ========== TSUBccTV ==========
                        6'b100011: begin
                            ID_ALU_OP_out = 4'b0010;
                            ID_WE_PSR_out = 1'b1;
                            keyword       = "TSUBCCTV";
                        end

                        // ========== AND ==========
                        6'b000001: begin
                            ID_ALU_OP_out = 4'b0100;
                            keyword       = "AND     ";
                        end

                        // ========== ANDcc ==========
                        6'b010001: begin
                            ID_ALU_OP_out = 4'b0100;
                            ID_WE_PSR_out = 1'b1;
                            keyword       = "ANDCC   ";
                        end

                        // ========== ANDN ==========
                        6'b000101: begin
                            ID_ALU_OP_out = 4'b1000;  
                            keyword       = "ANDN    ";
                        end

                        // ========== ANDNcc ==========
                        6'b010101: begin
                            ID_ALU_OP_out = 4'b1000;  
                            ID_WE_PSR_out = 1'b1;
                            keyword       = "ANDNCC  ";
                        end

                        // ========== OR ==========
                        6'b000010: begin
                            ID_ALU_OP_out = 4'b0101;
                            keyword       = "OR      ";
                        end

                        // ========== ORcc ==========
                        6'b010010: begin
                            ID_ALU_OP_out = 4'b0101;
                            ID_WE_PSR_out = 1'b1;
                            keyword       = "ORCC    ";
                        end

                        // ========== XOR ==========
                        6'b000011: begin
                            ID_ALU_OP_out = 4'b0110;
                            keyword       = "XOR     ";
                        end

                        // ========== XORcc ==========
                        6'b010011: begin
                            ID_ALU_OP_out = 4'b0110;
                            ID_WE_PSR_out = 1'b1;
                            keyword       = "XORCC   ";
                        end

                        // ========== XNOR ==========
                        6'b000111: begin
                            ID_ALU_OP_out = 4'b0111;
                            keyword       = "XNOR    ";
                        end

                        // ========== XNORcc ==========
                        6'b010111: begin
                            ID_ALU_OP_out = 4'b0111;
                            ID_WE_PSR_out = 1'b1;
                            keyword       = "XNORCC  ";
                        end

                        // ========== SLL ==========
                        6'b100101: begin
                            ID_ALU_OP_out = 4'b1010;
                            keyword       = "SLL     ";
                        end

                        // ========== SRL ==========
                        6'b100110: begin
                            ID_ALU_OP_out = 4'b1011;
                            keyword       = "SRL     ";
                        end

                        // ========== SRA ==========
                        6'b100111: begin
                            ID_ALU_OP_out = 4'b1100;
                            keyword       = "SRA     ";
                        end

                        // ========== JMPL ==========
                        6'b111000: begin
                            ID_ALU_OP_out = 4'b0000;  // rs1 + op2
                            ID_JUMPL_out  = 1'b1;     // salto indirecto
                            keyword       = "JMPL    ";
                        end

                        default: begin
                            keyword = "ALU_OP  ";
                        end
                    endcase
                end

                // OP = 11 ‚ LOAD / STORE (formato 3)
                2'b11: begin
                    ID_E_out      = 1'b1;                      // habilitar memoria
                    ID_SOH_OP_out = bit_i ? 4'b0111 : 4'b1000; // imm122 o rs2

                    case (op3)

                        // ------ LD (word) ------
                        6'b000000: begin
                            ID_LOAD_out  = 1'b1;
                            ID_RF_LE_out = 1'b1;
                            ID_SIZE_out  = 2'b10;
                            ID_SE_out    = 1'b0;        // word: da igual, lo dejamos unsigned
                            keyword      = "LD      ";
                        end

                        // ------ LDUB ------
                        6'b000001: begin
                            ID_LOAD_out  = 1'b1;
                            ID_RF_LE_out = 1'b1;
                            ID_SIZE_out  = 2'b00;
                            ID_SE_out    = 1'b0;        // unsigned byte
                            keyword      = "LDUB    ";
                        end

                        // ------ LDUH ------
                        6'b000010: begin
                            ID_LOAD_out  = 1'b1;
                            ID_RF_LE_out = 1'b1;
                            ID_SIZE_out  = 2'b01;
                            ID_SE_out    = 1'b0;        // unsigned halfword
                            keyword      = "LDUH    ";
                        end

                        // ------ LDD (double) ------
                        6'b000011: begin
                            ID_LOAD_out  = 1'b1;
                            ID_RF_LE_out = 1'b1;
                            ID_SIZE_out  = 2'b10; 
                            ID_SE_out    = 1'b0;        // sin sign-extend especial
                            keyword      = "LDD     ";
                        end

                        // ------ LDSB (signed byte) ------
                        6'b001001: begin
                            ID_LOAD_out  = 1'b1;
                            ID_RF_LE_out = 1'b1;
                            ID_SIZE_out  = 2'b00;
                            ID_SE_out    = 1'b1;        // <<< SIGNED BYTE
                            keyword      = "LDSB    ";
                        end

                        // ------ LDSH (signed halfword) ------
                        6'b001010: begin
                            ID_LOAD_out  = 1'b1;
                            ID_RF_LE_out = 1'b1;
                            ID_SIZE_out  = 2'b01;
                            ID_SE_out    = 1'b1;        // <<< SIGNED HALFWORD
                            keyword      = "LDSH    ";
                        end

                        // ------ ST (word) ------
                        6'b000100: begin
                            ID_RW_DM_out = 1'b1;
                            ID_SIZE_out  = 2'b10;
                            ID_SE_out    = 1'b0;        // SE no importa en store
                            keyword      = "ST      ";
                        end

                        // ------ STB ------
                        6'b000101: begin
                            ID_RW_DM_out = 1'b1;
                            ID_SIZE_out  = 2'b00;
                            ID_SE_out    = 1'b0;
                            keyword      = "STB     ";
                            ID_LOAD_out = 0;
                            ID_RF_LE_out = 0; 

                        end

                        // ------ STH ------
                        6'b000110: begin
                            ID_RW_DM_out = 1'b1;
                            ID_SIZE_out  = 2'b01;
                            ID_SE_out    = 1'b0;
                            keyword      = "STH";
                        end

                        // ------ STD (store double) ------
                        6'b000111: begin
                            ID_RW_DM_out = 1'b1;
                            ID_SIZE_out  = 2'b10;       // mismo control b√°sico
                            ID_SE_out    = 1'b0;
                            keyword      = "STD";
                            ID_LOAD_out = 0;
                            ID_RF_LE_out = 0; 
                        end

                        // ------ LDSTUB (atomic, aqu√≠ como LDUB b√°sica) ------
                        6'b001101: begin
                            ID_LOAD_out  = 1'b1;
                            ID_RF_LE_out = 1'b1;
                            ID_SIZE_out  = 2'b00;
                            ID_SE_out    = 1'b0;        // unsigned byte
                            keyword      = "LDSTUB";
                        end

                        // ------ SWAP (aqu√≠ la tratamos como LD word) ------
                        6'b001111: begin
                            ID_LOAD_out  = 1'b1;
                            ID_RF_LE_out = 1'b1;
                            ID_SIZE_out  = 2'b10;
                            ID_SE_out    = 1'b0;        // word
                            keyword      = "SWAP";
                        end

                        default: begin
                            keyword = "LDST_OP ";
                        end
                    endcase
                end

                default: begin
                    keyword = "UNKNOWN";
                    ID_LOAD_out = 1'b0;
                end
            endcase
        end
    end

endmodule

module MUX_ID_STALL(
    input        ID_MUX_sel,

    // ======= Inputs desde CU =======
    input  [3:0] ID_MUX_ALU_OP_in,
    input  [3:0] ID_MUX_SOH_OP_in,
    input        ID_MUX_LOAD_in,
    input        ID_MUX_a_in,
    input        ID_MUX_RF_LE_in,
    input        ID_MUX_CALL_in,
    input        ID_MUX_WE_PSR_in,
    input        ID_MUX_E_in,
    input  [1:0] ID_MUX_SIZE_in,
    input        ID_MUX_RW_DM_in,
    input        ID_MUX_BRANCH_in,
    input        ID_MUX_JUMPL_in,
    input        ID_MUX_SE_in,

    // ======= Outputs hacia ID/EX =======
    output [3:0] ID_MUX_ALU_OP_out,
    output [3:0] ID_MUX_SOH_OP_out,
    output        ID_MUX_LOAD_out,
    output        ID_MUX_a_out,
    output        ID_MUX_RF_LE_out,
    output        ID_MUX_CALL_out,
    output        ID_MUX_WE_PSR_out,
    output        ID_MUX_E_out,
    output [1:0] ID_MUX_SIZE_out,
    output        ID_MUX_RW_DM_out,
    output        ID_MUX_BRANCH_out,
    output        ID_MUX_JUMPL_out,
    output        ID_MUX_SE_out
);

    // ===============================
    // Burbuja cuando ID_MUX_sel = 1
    // ===============================
    assign ID_MUX_ALU_OP_out  = (ID_MUX_sel) ? 4'b0000 : ID_MUX_ALU_OP_in;
    assign ID_MUX_SOH_OP_out  = (ID_MUX_sel) ? 4'b0000 : ID_MUX_SOH_OP_in;
    assign ID_MUX_LOAD_out    = (ID_MUX_sel) ? 1'b0     : ID_MUX_LOAD_in;
    assign ID_MUX_a_out       = (ID_MUX_sel) ? 1'b0     : ID_MUX_a_in;
    assign ID_MUX_RF_LE_out   = (ID_MUX_sel) ? 1'b0     : ID_MUX_RF_LE_in;
    assign ID_MUX_CALL_out    = (ID_MUX_sel) ? 1'b0     : ID_MUX_CALL_in;
    assign ID_MUX_WE_PSR_out  = (ID_MUX_sel) ? 1'b0     : ID_MUX_WE_PSR_in;
    assign ID_MUX_E_out       = (ID_MUX_sel) ? 1'b0     : ID_MUX_E_in;
    assign ID_MUX_SIZE_out    = (ID_MUX_sel) ? 2'b00    : ID_MUX_SIZE_in;
    assign ID_MUX_RW_DM_out   = (ID_MUX_sel) ? 1'b0     : ID_MUX_RW_DM_in;
    assign ID_MUX_BRANCH_out  = (ID_MUX_sel) ? 1'b0     : ID_MUX_BRANCH_in;
    assign ID_MUX_JUMPL_out   = (ID_MUX_sel) ? 1'b0     : ID_MUX_JUMPL_in;
    assign ID_MUX_SE_out      = (ID_MUX_sel) ? 1'b0     : ID_MUX_SE_in;


endmodule


// =========================
//  Rgesitro
// ========================
module Registro_ID_EX(
    input        clk, R,

    // ======== Inputs desde ID ========
    input  [3:0] ID_ALU_OP_in,
    input  [3:0] ID_SOH_OP_in,
    input        ID_LOAD_in,
    input        ID_a_in,
    input        ID_RF_LE_in,
    input        ID_CALL_in,
    input        ID_WE_PSR_in,
    input        ID_E_in,
    input  [1:0] ID_SIZE_in,
    input        ID_RW_DM_in,
    input        ID_SE_in,
    input [31:0] DF_A, DF_B, DF_C,  
    input [1:0]  EX_PC_SEL_in,
    input [4:0]  rd_in,
    input [21:0] imm22_in,
         

    // ======== Outputs hacia EX ========
    output reg [3:0] EX_ALU_OP_out,
    output reg [3:0] EX_SOH_OP_out,
    output reg       EX_LOAD_out,
    output reg       EX_a_out,
    output reg       EX_RF_LE_out,
    output reg       EX_CALL_out,
    output reg       EX_WE_PSR_out,
    output reg       EX_E_out,
    output reg [1:0] EX_SIZE_out,
    output reg       EX_RW_DM_out,
    output reg       EX_SE_out,
    output reg [31:0] A_out, B_out, C_out,
    output reg [4:0] rd_out,
    output reg [1:0] EX_PC_SEL_out,
    output reg [21:0] imm22_out
        
);

    always @(posedge clk) begin
        if (R) begin
            // ===== Reset =====
            EX_ALU_OP_out  <= 4'b0000;
            EX_SOH_OP_out  <= 4'b0000;
            EX_LOAD_out    <= 1'b0;
            EX_a_out       <= 1'b0;
            EX_RF_LE_out   <= 1'b0;
            EX_CALL_out    <= 1'b0;
            EX_WE_PSR_out  <= 1'b0;
            EX_E_out       <= 1'b0;
            EX_SIZE_out    <= 2'b00;
            EX_RW_DM_out   <= 1'b0;
            EX_SE_out      <= 1'b0;
            A_out          <= 32'b0;
            B_out          <= 32'b0;
            C_out          <= 32'b0;
            rd_out         <= 5'b0;
            EX_PC_SEL_out  <= 2'b00;
            imm22_out      <= 22'b0;
              
        end
        
        else begin
            EX_ALU_OP_out  <= ID_ALU_OP_in;
            EX_SOH_OP_out  <= ID_SOH_OP_in;
            EX_LOAD_out    <= ID_LOAD_in;
            EX_a_out       <= ID_a_in;
            EX_RF_LE_out   <= ID_RF_LE_in;
            EX_CALL_out    <= ID_CALL_in;
            EX_WE_PSR_out  <= ID_WE_PSR_in;
            EX_E_out       <= ID_E_in;
            EX_SIZE_out    <= ID_SIZE_in;
            EX_RW_DM_out   <= ID_RW_DM_in;
            EX_SE_out      <= ID_SE_in;
            A_out          <= DF_A;
            B_out          <= DF_B;
            C_out          <= DF_C;
            rd_out         <= rd_in;
            EX_PC_SEL_out  <= EX_PC_SEL_in;
            imm22_out      <= imm22_in;
               
        end
    end

endmodule