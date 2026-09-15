`include "../include/i2c.vh"

module i2c_glitchfilter #(
    parameter SYS_CLK_FREQ_MHZ=100, 
    parameter MAX_GLITCH_DURATION_NS=50
)(
    input clk, rst_n, d,
    output reg q
);
    localparam MAX_GLITCH_COUNT = `NCYCLES(SYS_CLK_FREQ_MHZ, MAX_GLITCH_DURATION_NS);
    localparam COUNT_WIDTH = $clog2(MAX_GLITCH_COUNT);

    reg d_reg;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)
            d_reg <= 1'b1;
        else
            d_reg <= d;
    end

    reg [COUNT_WIDTH-1:0] count;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)
            count <= {COUNT_WIDTH{1'b0}};
        else if(d_reg == q)
            count <= {COUNT_WIDTH{1'b0}};
        else if(count == MAX_GLITCH_COUNT-1)
            count <= {COUNT_WIDTH{1'b0}};
        else
            count <= count + 1'b1;
    end

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            q <= 1'b1;
        end else if(d_reg != q && count == MAX_GLITCH_COUNT-1) begin
            q <= d_reg;
        end
    end
endmodule