module i2c_sync2(
    input clk, rst_n, d,
    output q
);   
    reg [1:0] sync;
    
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)
            sync <= 2'b11;
        else
            sync <= {d, sync[1]};
    end
    
    assign q = sync[0];
endmodule