module i2c_counter #(
    parameter COUNT_WIDTH=16
)(
    input clk, rst_n, en, reset_on_disable,
    input [COUNT_WIDTH-1:0] max_count,
    output reg done
);
    reg [COUNT_WIDTH-1:0] count;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            count <= 0;
            done  <= 1'b0;
        end else if (!en) begin
            done  <= 1'b0;
            count <= reset_on_disable ? 0 : count;
        end else begin
            if (count >= max_count) begin
                count <= 0;
                done  <= 1'b1;
            end else begin
                count <= count+1'b1;
                done  <= 1'b0;
            end
        end
    end
endmodule

            
            