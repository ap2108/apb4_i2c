module i2c_start_stop_detector(
    input clk, rst_n,

    input scl_in, sda_in,
    output start_detect, stop_detect
);
    reg sda_d;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)
            sda_d <= 1'b1;
        else
            sda_d <= sda_in;
    end

    assign start_detect = sda_d && !sda_in && scl_in;
    assign stop_detect = !sda_d && sda_in && scl_in;
endmodule