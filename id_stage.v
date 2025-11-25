
module Target_Address()
endmodule

module Register_File()
endmodule

module MUX_DF_PA()
endmodule

module MUX_DF_PB()
endmodule

module MUX_DF_PC()
endmodule

module MUX_CALL()
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

        // ==========================
        // NOP = instrucci√≥n 0
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
                
                    ID_SOH_OP_out = bit_i ? 4'b1001 : 4'b1000;
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

                // OP = 11 ‚Üí LOAD / STORE (formato 3)
                2'b11: begin
                    ID_E_out      = 1'b1;                      // habilitar memoria
                    ID_SOH_OP_out = bit_i ? 4'b1001 : 4'b1000; // simm13 o rs2

                    case (op3)

                        // ------ LD (word) ------
                        6'b000000: begin
                            ID_LOAD_out  = 1'b1;
                            ID_RF_LE_out = 1'b1;
                            ID_SIZE_out  = 2'b10;
                            keyword      = "LD      ";
                        end

                        // ------ LDUB ------
                        6'b000001: begin
                            ID_LOAD_out  = 1'b1;
                            ID_RF_LE_out = 1'b1;
                            ID_SIZE_out  = 2'b00;
                            keyword      = "LDUB    ";
                        end

                        // ------ LDUH ------
                        6'b000010: begin
                            ID_LOAD_out  = 1'b1;
                            ID_RF_LE_out = 1'b1;
                            ID_SIZE_out  = 2'b01;
                            keyword      = "LDUH    ";
                        end

                        // ------ LDD (double) ------
                        6'b000011: begin
                            ID_LOAD_out  = 1'b1;
                            ID_RF_LE_out = 1'b1;
                            ID_SIZE_out  = 2'b10; 
                            keyword      = "LDD     ";
                        end

                        // ------ LDSB (signed byte) ------
                        6'b001001: begin
                            ID_LOAD_out  = 1'b1;
                            ID_RF_LE_out = 1'b1;
                            ID_SIZE_out  = 2'b00;
                            keyword      = "LDSB    ";
                        end

                        // ------ LDSH (signed halfword) ------
                        6'b001010: begin
                            ID_LOAD_out  = 1'b1;
                            ID_RF_LE_out = 1'b1;
                            ID_SIZE_out  = 2'b01;
                            keyword      = "LDSH    ";
                        end

                        // ------ ST (word) ------
                        6'b000100: begin
                            ID_RW_DM_out = 1'b1;
                            ID_SIZE_out  = 2'b10;
                            keyword      = "ST      ";
                        end

                        // ------ STB ------
                        6'b000101: begin
                            ID_RW_DM_out = 1'b1;
                            ID_SIZE_out  = 2'b00;
                            keyword      = "STB     ";
                        end

                        // ------ STH ------
                        6'b000110: begin
                            ID_RW_DM_out = 1'b1;
                            ID_SIZE_out  = 2'b01;
                            keyword      = "STH";
                        end

                        // ------ STD (store double) ------
                        6'b000111: begin
                            ID_RW_DM_out = 1'b1;
                            ID_SIZE_out  = 2'b10; // mismo control b√°sico
                            keyword      = "STD";
                        end

                        // ------ LDSTUB (atomic, aqu√≠ como LDUB b√°sica) ------
                        6'b001101: begin
                            ID_LOAD_out  = 1'b1;
                            ID_RF_LE_out = 1'b1;
                            ID_SIZE_out  = 2'b00;
                            keyword      = "LDSTUB";
                        end

                        // ------ SWAP (aqu√≠ la tratamos como LD word) ------
                        6'b001111: begin
                            ID_LOAD_out  = 1'b1;
                            ID_RF_LE_out = 1'b1;
                            ID_SIZE_out  = 2'b10;
                            keyword      = "SWAP";
                        end

                        default: begin
                            keyword = "LDST_OP ";
                        end
                    endcase
                end

                default: begin
                    keyword = "UNKNOWN";
                end
            endcase
        end
    end

endmodule

module MUX_ID(
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
    output        ID_MUX_JUMPL_out
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
    output reg       EX_RW_DM_out
        
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
               
        end
    end

endmodule