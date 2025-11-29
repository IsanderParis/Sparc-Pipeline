module Registro_MEM_WB(
    input  clk,
    input  R,             // Reset

    input  rf_le_mem,
    input [31:0] MEM_MUX_OUT,
    input [4:0] mem_rd,

    output reg rf_le_wb,
    output reg [31:0] wb_mux_out,
    output reg [4:0] wb_rd
);
    always @(posedge clk) begin
        if (R) begin
            rf_le_wb <= 0;
            wb_mux_out <= 0;
            wb_rd <= 0;
        end else begin
            rf_le_wb <= rf_le_mem;
            wb_mux_out <= MEM_MUX_OUT;
            wb_rd <= mem_rd;
        end
    end
endmodule
