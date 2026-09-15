module fifo #(
    parameter DATA_WIDTH = 8,
    parameter DEPTH = 4096
)(
    input clk, rst_n, r_en, w_en,
    input [DATA_WIDTH-1:0] wr_data,

    output [DATA_WIDTH-1:0] rd_data,
    output full, empty
);
    
    localparam ADDR_WIDTH = $clog2(DEPTH);

    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];
    reg [ADDR_WIDTH-1:0] rd_ptr, wr_ptr;
    
    reg wr_oflow, rd_oflow;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            wr_ptr <= {ADDR_WIDTH{1'b0}};
            wr_oflow <= 1'b0;
        end else begin
            if(w_en && !full) begin
                wr_ptr <= (wr_ptr==DEPTH-1) ? {ADDR_WIDTH{1'b0}} : wr_ptr+1;
                wr_oflow <= (wr_ptr==DEPTH-1) ? ~wr_oflow : wr_oflow;                          
            end
        end
    end

    always @(posedge clk ) begin
        if(w_en && !full) begin
            mem[wr_ptr] <= wr_data;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            rd_ptr <= {ADDR_WIDTH{1'b0}};
            rd_oflow <= 1'b0;
        end else begin
            if(r_en && !empty) begin
                rd_ptr <= (rd_ptr==DEPTH-1) ? {ADDR_WIDTH{1'b0}} : rd_ptr+1;
                rd_oflow <= (rd_ptr==DEPTH-1) ? ~rd_oflow : rd_oflow;
            end 
        end
    end

    assign rd_data = mem[rd_ptr];

    assign full = (wr_ptr == rd_ptr) && (wr_oflow != rd_oflow);
    assign empty = (wr_ptr == rd_ptr) && (wr_oflow == rd_oflow);
endmodule