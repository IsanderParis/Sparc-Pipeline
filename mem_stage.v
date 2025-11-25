
module MUX_MEM_OUT()
endmodule

module Data_Memory (
  input         clk,
  input  [8:0]  A_in,     // usamos los 9 bits (0..511)
  input  [31:0] DI,       // dato de entrada
  input  [1:0]  Size,     // 00=byte, 01=half, 10=word
  input         RW,       // 0=read, 1=write
  input         E,        // enable de escritura (válido si RW=1)
  output [31:0] DO
);
  reg [7:0]  mem [0:511];
  reg [31:0] dout;
  assign DO = dout;

  wire [8:0] A = A_in; // fuerza rango 0..511

  integer i;
  initial begin
    for (i = 0; i < 512; i = i + 1) mem[i] = 8'h00;
    // Carga solo los primeros 16 bytes para la demostracion
    $readmemb("precharge_code.txt", mem, 0, 15);
  end

  // Lectura combinacional (big-endian)
  always @(*) begin
    case (Size)
      2'b00: dout = {24'b0, mem[A]};
      2'b01: dout = {16'b0, mem[A],    mem[A+1]};
      2'b10: dout = {       mem[A],    mem[A+1], mem[A+2], mem[A+3]};
      default: dout = 32'b0;
    endcase
  end

  // Escritura SINCRÓNICA
  always @(posedge clk) begin
    if (RW && E) begin
      case (Size)
        2'b00: begin
          mem[A] <= DI[7:0];
        end
        2'b01: begin
          mem[A]   <= DI[15:8];
          mem[A+1] <= DI[7:0];
        end
        2'b10: begin
          mem[A]   <= DI[31:24];
          mem[A+1] <= DI[23:16];
          mem[A+2] <= DI[15:8];
          mem[A+3] <= DI[7:0];
        end
      endcase
    end
  end
endmodule


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
