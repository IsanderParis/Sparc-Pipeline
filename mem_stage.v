
module MUX_MEM_OUT(
    input [31:0] ALU_OUT,
    input [31:0] MEM_OUT,
    input MEM_LOAD,
    output reg [31:0] MUX_OUT
);
    always @(*) begin
        if (MEM_LOAD) begin
            MUX_OUT = MEM_OUT;
        end else begin
            MUX_OUT = ALU_OUT;
        end
    end

endmodule

module Data_Memory (
    input         clk,
    input  [8:0]  A_in,     // address (0..511)
    input  [31:0] DI,       // data in
    input  [1:0]  Size,     // 00=byte, 01=halfword, 10=word
    input         RW,       // 1=write, 0=read
    input         E,        // write enable
    output [31:0] DO
);

    reg [7:0]  mem [0:511];
    reg [31:0] dout;

    assign DO = dout;

    wire [8:0] A = A_in;

    integer i;

    initial begin
        for (i = 0; i < 512; i = i + 1)
            mem[i] = 8'h00;   // inicializa TODA la memoria a 0
    end

    // ============================
    //  READ: combinational
    // ============================
    always @(*) begin
        case (Size)
            2'b00: dout = {24'b0, mem[A]};

            2'b01: dout = {
                16'b0,
                mem[A],
                mem[A+1]
            };

            2'b10: dout = {
                mem[A],
                mem[A+1],
                mem[A+2],
                mem[A+3]
            };

            default: dout = 32'b0;
        endcase
    end

    // ============================
    //  WRITE: synchronous
    // ============================
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
