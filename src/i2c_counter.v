module i2c_counter #(
    parameter COUNT_WIDTH=16
)(
    input clk, rst_n, en, reset_on_disable,
    input [COUNT_WIDTH-1:0] max_count,
    output reg done
);
    reg [COUNT_WIDTH-1:0] count;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            count <= {COUNT_WIDTH{1'b0}};
            done <= 1'b0;
        end else if(en) begin
            count <= (count >= max_count) ? {COUNT_WIDTH{1'b0}} : count + 1;
            done <= (count >= max_count);
        end else begin
            done <= 1'b0;
            count <= reset_on_disable ? {COUNT_WIDTH{1'b0}} : count;
        end
    end
endmodule

            
            