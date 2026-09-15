`include "../include/i2c.vh"

module i2c_slv_driver (
    input clk, rst_n, 
    input [7:0] tx_data,

    input [9:0] self_slv_addr,
    input self_slv_addr_10bit,

    input tx_en, rx_en,

    input start_detect, stop_detect,

    input scl_in, sda_in, 
    output reg sda_oe, scl_oe,

    output reg [7:0] rx_data,
    output reg xfr_ready
);  

    // Slave Bit level FSM states --------------------------------------------------------------------------
    localparam [3:0] IDLE=0, START=1, ADDR_LOW=2, ADDR_HIGH=3, ADDR_ACK_LOW=4, ADDR_ACK_HIGH=5, 
        RX_LOW=6, RX_HIGH=7, RX_ACK_LOW=8, RX_ACK_HIGH=9, TX_LOW=10, TX_HIGH=11, TX_ACK_LOW=12, TX_ACK_HIGH=13;

    reg [3:0] state, next_state;

    // Slave Bit level FSM logic ----------------------------------------------------------------------------
    reg addr10_second_byte, next_addr10_second_byte;
    reg rw, next_rw, next_xfr_ready;

    reg [2:0] bit_idx, next_bit_idx;
    
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)
            rx_data <= 8'b0;
        else if((state==ADDR_HIGH || state==RX_HIGH) && scl_in)
            rx_data[bit_idx] <= sda_in;
    end

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            bit_idx <= 3'd7;
            addr10_second_byte <= 1'b0;
            rw <= 1'b1;
            state <= IDLE;  
            xfr_ready <= 1'b0;
        end else begin
            bit_idx <= next_bit_idx;
            addr10_second_byte <= next_addr10_second_byte;
            rw <= next_rw;
            state <= next_state;
            xfr_ready <= next_xfr_ready;
        end
    end
        
    always @(*) begin
        next_state = state;
        next_rw = rw;
        next_bit_idx = bit_idx;
        next_addr10_second_byte = addr10_second_byte;

        next_xfr_ready = 1'b0;
        {scl_oe, sda_oe} = 2'b00;

        if(start_detect) begin
            next_state = START;
        end else if(stop_detect) begin
            next_state = IDLE;
        end else begin
            case(state)
                IDLE: begin
                    next_addr10_second_byte = 1'b0;
                    next_bit_idx = 3'd7;
                end
                START: begin
                    next_state = !scl_in ? ADDR_LOW : START;
                end
                ADDR_LOW: begin
                    next_state = scl_in ? ADDR_HIGH : ADDR_LOW;
                end
                ADDR_HIGH: begin
                    if(!scl_in) begin
                        next_bit_idx = bit_idx-1;

                        if(bit_idx==3'd0) begin
                            if(self_slv_addr_10bit) begin
                                if(addr10_second_byte) begin
                                    next_state = (rx_data==self_slv_addr[7:0]) ? ADDR_ACK_LOW : IDLE;
                                end else begin
                                    next_state = (rx_data[7:1]=={5'b11110, self_slv_addr[9:8]}) ? ADDR_ACK_LOW : IDLE;
                                    next_rw = rx_data[0];
                                    next_addr10_second_byte = 1'b1;
                                end
                            end else begin 
                                next_state = (rx_data[7:1]==self_slv_addr[6:0]) ? ADDR_ACK_LOW : IDLE;
                                next_rw = rx_data[0];
                            end
                        end else begin
                            next_state = ADDR_LOW;
                        end
                    end
                end
                ADDR_ACK_LOW: begin
                    if((tx_en&&rw) || (rx_en&&!rw)) begin
                        next_state = scl_in ? ADDR_ACK_HIGH : ADDR_ACK_LOW;
                        sda_oe = 1'b1;
                    end else begin
                        next_state = IDLE;
                    end
                end
                ADDR_ACK_HIGH: begin
                    sda_oe = 1'b1;
                    if(!scl_in) begin
                        if(self_slv_addr_10bit && !addr10_second_byte) begin
                            next_state = ADDR_LOW;
                        end else begin
                            next_state = rw ? TX_LOW : RX_LOW;
                        end
                    end
                end
                RX_LOW: begin
                    next_state = scl_in ? RX_HIGH : RX_LOW;
                end
                RX_HIGH: begin
                    if(!scl_in) begin
                        next_bit_idx = bit_idx-1;
                        next_state = (bit_idx==3'd0) ? RX_ACK_LOW : RX_LOW;
                        next_xfr_ready = (bit_idx==3'd0) ? 1'b1 : 1'b0;
                    end
                end
                RX_ACK_LOW: begin
                    sda_oe = 1'b1; 
                    next_state = scl_in ? RX_ACK_HIGH : RX_ACK_LOW;
                end
                RX_ACK_HIGH: begin
                    sda_oe = 1'b1; 
                    if(!scl_in)
                        next_state = rx_en ? RX_LOW : IDLE;
                end
                TX_LOW: begin
                    if (tx_en) begin
                        {scl_oe, sda_oe} = {1'b0, !tx_data[bit_idx]};
                        next_state = scl_in ? TX_HIGH : TX_LOW;
                    end else begin
                        {scl_oe, sda_oe} = 2'b10; // clock stretching
                        next_state = TX_LOW; 
                    end
                end
                TX_HIGH: begin
                    if(!scl_in) begin
                        next_bit_idx = bit_idx-1;
                        next_state = (bit_idx==3'd0) ? TX_ACK_LOW : TX_LOW;
                        next_xfr_ready = (bit_idx==3'd0) ? 1'b1 : 1'b0;
                    end else begin
                        sda_oe = !tx_data[bit_idx];
                    end
                end
                TX_ACK_LOW: begin
                    next_state = scl_in ? TX_ACK_HIGH : TX_ACK_LOW;
                end
                TX_ACK_HIGH: begin
                    if(!scl_in)
                        next_state = tx_en ? TX_LOW : IDLE;
                    else
                        next_state = sda_in ? IDLE : TX_ACK_HIGH;
                end
                default: begin
                    next_state = IDLE;
                end
            endcase
        end
    end
endmodule