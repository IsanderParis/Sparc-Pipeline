// =========================

module MUX_IF(
    input  [31:0] npc_in,       // Valor actual del nPC
    input   [1:0] sel,          // Seleccion de la fuente
    output [31:0] mux_out       // Salida = npc_in + 4
);
    assign mux_out = npc_in + 4;
    //implementar condicion con sel (alu_out, TA, nPC)
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

// este es el que se uso en fase 1
module instruction_memory (
  input  [8:0]  A,     // 0..511
  output [31:0] I
);
  reg [7:0] imem [0:511];
  integer k;

  initial begin
    // Limpia toda la ROM a 0
    for (k = 0; k < 512; k = k + 1) imem[k] = 8'h00;
    // Carga solo los primeros 16 bytes para evitar warnings
    $readmemb("precharge_code.txt", imem, 0, 15);
  end

  // Lectura big-endian (A = byte más significativo del word)
  assign I = { imem[A], imem[A+1], imem[A+2], imem[A+3] };
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