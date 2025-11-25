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