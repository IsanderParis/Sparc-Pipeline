// =========================

module MUX_IF(
    input  [31:0] npc_in,       // Valor actual del nPC
    input  [31:0] alu_out,      // Salida ALU
    input  [31:0] ta,           // Target Address (instrucción de
    input   [1:0] sel,          // Seleccion de la fuente
    output reg [31:0] mux_out       // Salida = npc_in + 4
);
    // assign mux_out = npc_in + 4;
    //implementar condicion con sel (alu_out, TA, nPC)
    always @(*) begin
        case (sel)
            2'b00: mux_out = npc_in;
            2'b01: mux_out = ta;
            2'b10: mux_out = alu_out;
            default: mux_out = 32'b0;
        endcase
    end

endmodule

module Adder(
    input [31:0] mux_out,
    output [31:0] adder_out
);

assign adder_out = mux_out + 4;
endmodule
// =========================

module NPC_IF(
    input        clk,           // reloj
    input        LE,            // Load Enable
    input        R,             // Reset
    input  [31:0] adder_out,      // entrada desde MUX_IF
    output reg [31:0] npc       // salida = siguiente PC
);
    always @(posedge clk) begin
        if (R)
            npc <= 4;
        else if (LE)
            npc <= adder_out;
    end
endmodule

// =========================

module PC_IF(
    input        clk,           // reloj
    input        LE,            // Load Enable
    input        R,             // Reset
    input  [31:0] mux_out,          // entrada desde NPC_IF
    output reg [31:0] pc_out    // salida: PC actual
);
    always @(posedge clk) begin
        if (R)
            pc_out <= 0;
        else if (LE)
            pc_out <= mux_out;
    end
endmodule

// =========================
// no es correcta
// module Instruction_Memory(
//   input  [31:0] pc_out,
//   output [31:0] instruction
// );
//   reg [31:0] Mem [0:511];   

//   assign instruction = Mem[pc_out >> 2];
// endmodule

// este es el que se uso en fase 1
module Instruction_Memory (
  input  [8:0]  A,     // 0..511
  output [31:0] I
);
  reg [7:0] imem [0:511];
  integer k;

  initial begin
    // Limpia toda la ROM a 0
    for (k = 0; k < 512; k = k + 1) imem[k] = 8'h00;
  end

//   Lectura big-endian (A = byte más significativo del word)
  assign I = { imem[A], imem[A+1], imem[A+2], imem[A+3] };
endmodule
// =========================

module Registro_IF_ID(
    input        clk,                 // reloj
    input        R,                   // reset
    input        LE,
    input CH_clear,
    input  [31:0] pc_in,              // entrada desde PC_IF
    input  [31:0] instruction_in,        // entrada desde Instruction Memory
    output reg [31:0] instruction_out, // salida hacia la etapa ID
    output reg [31:0] pc_out         // salida hacia la etapa ID
);
    always @(posedge clk) begin
        if (R || CH_clear)
            instruction_out <= 32'b0;   // limpia en reset
        else if (LE)
            instruction_out <= instruction_in; // guarda la nueva instrucción
            pc_out <= pc_in;
    end
endmodule