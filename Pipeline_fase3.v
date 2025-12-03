`timescale 1ns/1ps
// README:
//descripcion de circuito: Fase 3

// Registros generales del pipeline:
// Registro_IF_ID
// Registro_ID_EX
// Registro_EX_MEM
// Registro_MEM_WB

// Componentest por estapas:
// IF: Instruction Fetch
// PC_IF
// NPC_IF
// Instruction_Memory
// ID: Instruction Decode
// CU_ID
// MUX_ID
// EX: Execute
// MEM: Memory
// WB: Write Back
//  Para compilar el archivo: iverilog -o Pipeline.out Pipeline.v
//  Para simular el archivo: vvp Pipeline.out
// 

// =========================

module MUX_IF(
    input  [31:0] npc_in,       // Valor actual del nPC
    output [31:0] mux_out       // Salida = npc_in + 4
);
    assign mux_out = npc_in + 4;
endmodule

// =========================

module NPC_IF(
    input        clk,           // reloj
    input        LE,            // Load Enable
    input        R,             // Reset
    input  [31:0] mux_out,      // entrada desde MUX_IF
    output reg [31:0] npc       // salida = siguiente PC
);
    always @(posedge clk) begin
        if (R)
            npc <= 4;
        else if (LE)
            npc <= mux_out;
    end
endmodule

// =========================

module PC_IF(
    input        clk,           // reloj
    input        LE,            // Load Enable
    input        R,             // Reset
    input  [31:0] nPC,          // entrada desde NPC_IF
    output reg [31:0] pc_out    // salida: PC actual
);
    always @(posedge clk) begin
        if (R)
            pc_out <= 0;
        else if (LE)
            pc_out <= nPC;
    end
endmodule

// =========================

module Instruction_Memory(
  input  [31:0] pc_out,
  output [31:0] instruction
);
  reg [31:0] Mem [0:127];   // 128 instrucciones

  assign instruction = Mem[pc_out >> 2];
endmodule

// =========================

module Registro_IF_ID(
    input        clk,                 // reloj
    input        R,                   // reset
    input  [31:0] instruction,        // entrada desde Instruction Memory
    output reg [31:0] instruction_out // salida hacia la etapa ID
);
    always @(posedge clk) begin
        if (R)
            instruction_out <= 32'b0;   // limpia en reset
        else
            instruction_out <= instruction; // guarda la nueva instrucci√≥n
    end
endmodule

// =========================================================
//  Instruction Decode (ID)
// =========================================================


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



// =====================
//  Execute (EX)
// =====================

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

// ====
//  Memory (MEM)
// ====
module Registro_MEM_WB(
    input  clk,
    input  R,             // Reset
    input  rf_le_mem,
    output reg rf_le_wb
);
    always @(posedge clk or posedge R) begin
        if (R)
            rf_le_wb <= 0;
        else
            rf_le_wb <= rf_le_mem;
    end
endmodule


module Pipeline (
    input        clk,
    input        reset,
    input        LE
);
    // todas las conexiones internas y m√≥dulos se definen aquui
    // aqui es donde hago las instanciass?
endmodule

module

module TEST_BENCH3;

  // Se√±ales globales 
  reg clk, reset, LE;

  // IF stage
  wire [31:0] pc_out, npc_out, mux_out;
  wire [31:0] instruction_IF, instruction_ID;

  // ID stage (CU outputs)
  wire [3:0]  ID_ALU_OP, ID_SOH_OP;
  wire        ID_LOAD, ID_a, ID_RF_LE, ID_CALL;
  wire        ID_WE_PSR, ID_E, ID_RW_DM;
  wire [1:0]  ID_SIZE;
  wire        ID_BRANCH, ID_JUMPL;  
  wire [63:0] keyword;

  // EX stage
  wire [3:0] EX_ALU_OP, EX_SOH_OP;
  wire       EX_LOAD, EX_a, EX_RF_LE, EX_CALL;
  wire       EX_WE_PSR, EX_E, EX_RW_DM;
  wire [1:0] EX_SIZE;

  // MEM stage
  wire MEM_LOAD, MEM_RF_LE, MEM_E, MEM_RW_DM;
  wire [1:0] MEM_SIZE;

  // WB stage
  wire WB_RF_LE;

  // ==========================================================================
  //                                 IF
  // ==========================================================================
  MUX_IF mux_if (
    .npc_in(npc_out),
    .mux_out(mux_out)
  );

  NPC_IF npc_if (
    .clk(clk),
    .LE(LE),
    .R(reset),
    .mux_out(mux_out),
    .npc(npc_out)
  );

  PC_IF pc_if (
    .clk(clk),
    .LE(LE),
    .R(reset),
    .nPC(npc_out),
    .pc_out(pc_out)
  );

  Instruction_Memory im (
    .pc_out(pc_out),
    .instruction(instruction_IF)
  );

  Registro_IF_ID if_id (
    .clk(clk),
    .R(reset),
    .instruction(instruction_IF),
    .instruction_out(instruction_ID)
  );

  // ==========================================================================
  //                                 ID
  // ==========================================================================
  CU_ID cu_id (
    .instruction(instruction_ID),
    .ID_ALU_OP_out(ID_ALU_OP),
    .ID_SOH_OP_out(ID_SOH_OP),
    .ID_LOAD_out(ID_LOAD),
    .ID_a_out(ID_a),
    .ID_RF_LE_out(ID_RF_LE),
    .ID_CALL_out(ID_CALL),
    .ID_WE_PSR_out(ID_WE_PSR),
    .ID_E_out(ID_E),
    .ID_SIZE_out(ID_SIZE),
    .ID_RW_DM_out(ID_RW_DM),
    .ID_BRANCH_out(ID_BRANCH),   
    .ID_JUMPL_out(ID_JUMPL),     
    .keyword(keyword)
  );

 

  Registro_ID_EX id_ex (
    .clk(clk),
    .R(reset),
    .ID_ALU_OP_in(ID_ALU_OP),
    .ID_SOH_OP_in(ID_SOH_OP),
    .ID_LOAD_in(ID_LOAD),
    .ID_a_in(ID_a),
    .ID_RF_LE_in(ID_RF_LE),
    .ID_CALL_in(ID_CALL),
    .ID_WE_PSR_in(ID_WE_PSR),
    .ID_E_in(ID_E),
    .ID_SIZE_in(ID_SIZE),
    .ID_RW_DM_in(ID_RW_DM),
    .EX_ALU_OP_out(EX_ALU_OP),
    .EX_SOH_OP_out(EX_SOH_OP),
    .EX_LOAD_out(EX_LOAD),
    .EX_a_out(EX_a),
    .EX_RF_LE_out(EX_RF_LE),
    .EX_CALL_out(EX_CALL),
    .EX_WE_PSR_out(EX_WE_PSR),
    .EX_E_out(EX_E),
    .EX_SIZE_out(EX_SIZE),
    .EX_RW_DM_out(EX_RW_DM)
  );

  // ==========================================================================
  //                                 EX
  // ==========================================================================
  Registro_EX_MEM ex_mem (
    .clk(clk),
    .R(reset),
    .load_ex(EX_LOAD),
    .rf_le_ex(EX_RF_LE),
    .E_ex(EX_E),
    .size_ex(EX_SIZE),
    .rw_dm_ex(EX_RW_DM),
    .load_mem(MEM_LOAD),
    .rf_le_mem(MEM_RF_LE),
    .E_mem(MEM_E),
    .size_mem(MEM_SIZE),
    .rw_dm_mem(MEM_RW_DM)
  );

  // ==========================================================================
  //                                 MEM
  // ==========================================================================
  Registro_MEM_WB mem_wb (
    .clk(clk),
    .R(reset),
    .rf_le_mem(MEM_RF_LE),
    .rf_le_wb(WB_RF_LE)
  );

  //                      Cargar memoria desde archivo
  reg [31:0] program [0:127];
  initial begin
    $readmemb("demo.txt", program);
    $display("\nArchivo demo.txt cargado.\n");

    for (integer i = 0; i < 128; i = i + 1)
      im.Mem[i] = program[i];
  end

  //                              Clock & Reset
  initial begin
    clk = 0;
    forever #2 clk = ~clk;
  end

  initial begin
    reset = 1;
    LE = 1;
    #3 reset = 0;
  end

  //                           Print del Pipeline
  always @(posedge clk) begin
    if (!reset) begin

      $display("\n[%0t] IF: PC=%0d  nPC=%0d | Instr=%b | %s",
        $time, pc_out, npc_out, instruction_IF, keyword);

      $display("  ID  | BR=%b JMPL=%b RF_LE=%b LOAD=%b CALL=%b RW_DM=%b WE_PSR=%b a=%b E=%b SIZE=%02b SOH_OP=%b ALU_OP=%b",
        ID_BRANCH, ID_JUMPL, ID_RF_LE, ID_LOAD, ID_CALL,
        ID_RW_DM, ID_WE_PSR, ID_a, ID_E, ID_SIZE, ID_SOH_OP, ID_ALU_OP);

      $display("  EX  | RF_LE_EX=%b LOAD_EX=%b RW_DM_EX=%b E_EX=%b SIZE_EX=%02b",
        EX_RF_LE, EX_LOAD, EX_RW_DM, EX_E, EX_SIZE);

      $display("  MEM | RF_LE_MEM=%b LOAD_MEM=%b RW_DM_MEM=%b E_MEM=%b SIZE_MEM=%02b",
        MEM_RF_LE, MEM_LOAD, MEM_RW_DM, MEM_E, MEM_SIZE);

      $display("  WB  | RF_LE_WB=%b", WB_RF_LE);

    end
  end

  initial #48 $finish;

endmodule

