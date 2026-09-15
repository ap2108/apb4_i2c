`include "../include/i2c.vh"

module i2c_master_driver #(
    parameter SYS_CLK_FREQ_MHZ=100,
    parameter TIMEOUT_WIDTH=24
)(
    input clk, rst_n, 
    input [2:0] trig,
    input [1:0] speed,

    input [TIMEOUT_WIDTH-1:0] stretch_timeout, sda_stuck_timeout,

    input [7:0] tx_data,

    input start_detect, stop_detect,

    input scl_in, sda_in, 
    output reg sda_oe, scl_oe,

    output reg [7:0] rx_data,
    output reg [2:0] resp
);  
    
    reg scl_oe_next, sda_oe_next;

    // Bit level FSM states -------------------------------------------------------------------
    localparam [3:0] IDLE=0, START_SU=1, START_HD=2, TX_LOW=3, TX_HIGH=4, RX_LOW=5, RX_HIGH=6, 
        STOP_LOW=7, STOP_SU=8, STOP_HD=9, BUS_BUSY=10, BUS_BUFFER_WAIT=11, SDA_STUCK_ERROR=12;
        
    reg [3:0] state, next_state;

    // Timing setup ----------------------------------------------------------------------------- 
    localparam COUNT_WIDTH = $clog2(`NCYCLES(SYS_CLK_FREQ_MHZ, 6000));      

    reg [COUNT_WIDTH-1:0] count_value;
    reg count_EN;
    wire count_done;

    reg counter_reset_on_disable;

    wire scl_err, sda_err;

    i2c_counter #(.COUNT_WIDTH(COUNT_WIDTH)) u_ctr_main (
        .clk(clk),
        .rst_n(rst_n),
        .en(count_EN),
        .reset_on_disable(counter_reset_on_disable),
        .max_count(count_value),
        .done(count_done)
    );  
    
    i2c_counter #(.COUNT_WIDTH(TIMEOUT_WIDTH)) u_ctr_sda_err (
        .clk(clk),
        .rst_n(rst_n),
        .en(!sda_in && (state==IDLE)),
        .reset_on_disable(1'b1),
        .max_count(sda_stuck_timeout),
        .done(sda_err)
    );

    i2c_counter #(.COUNT_WIDTH(TIMEOUT_WIDTH)) u_ctr_scl_err (
        .clk(clk),
        .rst_n(rst_n),
        .en(!scl_in),
        .reset_on_disable(1'b1),
        .max_count(stretch_timeout),
        .done(scl_err)
    );

    // Timing values select logic for different speeds --------------------------------------------
    reg [COUNT_WIDTH-1:0] t_sta_su, t_sta_hd, t_low, t_high, t_sto_su, t_buf;
    
    always @(*) begin
        case(speed)
            2'b00: begin
                t_sta_su = `NCYCLES(SYS_CLK_FREQ_MHZ, `T_STA_SU_STD);
                t_sta_hd = `NCYCLES(SYS_CLK_FREQ_MHZ, `T_STA_HD_STD);
                t_low = `NCYCLES(SYS_CLK_FREQ_MHZ, `T_LOW_STD);
                t_high = `NCYCLES(SYS_CLK_FREQ_MHZ, `T_HIGH_STD);
                t_sto_su = `NCYCLES(SYS_CLK_FREQ_MHZ, `T_STO_SU_STD);
                t_buf = `NCYCLES(SYS_CLK_FREQ_MHZ, `T_BUF_STD);
            end

            2'b01: begin
                t_sta_su = `NCYCLES(SYS_CLK_FREQ_MHZ, `T_STA_SU_FAST);
                t_sta_hd = `NCYCLES(SYS_CLK_FREQ_MHZ, `T_STA_HD_FAST);
                t_low = `NCYCLES(SYS_CLK_FREQ_MHZ, `T_LOW_FAST);
                t_high = `NCYCLES(SYS_CLK_FREQ_MHZ, `T_HIGH_FAST);
                t_sto_su = `NCYCLES(SYS_CLK_FREQ_MHZ, `T_STO_SU_FAST);
                t_buf = `NCYCLES(SYS_CLK_FREQ_MHZ, `T_BUF_FAST);
            end

            default: begin
                t_sta_su = `NCYCLES(SYS_CLK_FREQ_MHZ, `T_STA_SU_FASTP);
                t_sta_hd = `NCYCLES(SYS_CLK_FREQ_MHZ, `T_STA_HD_FASTP);
                t_low = `NCYCLES(SYS_CLK_FREQ_MHZ, `T_LOW_FASTP);
                t_high = `NCYCLES(SYS_CLK_FREQ_MHZ, `T_HIGH_FASTP);
                t_sto_su = `NCYCLES(SYS_CLK_FREQ_MHZ, `T_STO_SU_FASTP);
                t_buf = `NCYCLES(SYS_CLK_FREQ_MHZ, `T_BUF_FASTP);
            end
        endcase
    end
    
    // Error handling FSM ---------------------------------------------------------------------------
    reg running;
    reg recovery_running, next_recovery_running;
    
    localparam GF_DELAY = `NCYCLES(SYS_CLK_FREQ_MHZ, 50) + 3;
    reg [GF_DELAY-1:0] sda_intent;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)
            sda_intent <= {GF_DELAY{1'b1}};
        else
            sda_intent <= {!sda_oe_next, sda_intent[GF_DELAY-1:1]};
    end

    reg [2:0] resp_reg;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) 
            resp_reg <= `RESP_IDLE;
        else 
            resp_reg <= resp;
    end

    always @(*) begin
        case(resp_reg)
            `RESP_IDLE, `RESP_INPROGRESS, `RESP_BUS_BUSY: begin
                if(scl_err) 
                    resp = `RESP_SCL_ERR;
                else if(state==SDA_STUCK_ERROR && !sda_in)
                    resp = `RESP_SDA_ERR;
                else begin
                    if(sda_intent[0] && !sda_in && 
                        (state==START_SU || state==STOP_HD || state==TX_HIGH))
                        resp = `RESP_ARB_LOST;
                    else
                        resp = running ? `RESP_INPROGRESS : ((state==BUS_BUSY) ? `RESP_BUS_BUSY : `RESP_IDLE);
                end
            end
            `RESP_ARB_LOST: begin
                resp = (state==BUS_BUSY) ? `RESP_ARB_LOST : `RESP_IDLE;
            end
            `RESP_SDA_ERR: begin
                resp = `RESP_SDA_ERR;
            end
            `RESP_SCL_ERR: begin
                resp = `RESP_SCL_ERR;
            end
            default: begin
                resp = `RESP_IDLE;
            end
        endcase
    end
    
    // Bit level FSM logic ----------------------------------------------------------------------------
    reg ack, next_ack;

    reg [2:0] bit_idx, next_bit_idx;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)
            rx_data <= 8'b0;
        else if(state==RX_HIGH)
            rx_data[bit_idx] <= count_done ? sda_in : rx_data[bit_idx];
    end

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            bit_idx <= 3'd0;
            ack <= 1'b0;
            recovery_running <= 1'b0;
            state <= IDLE;

            {scl_oe, sda_oe} <= 2'b00;
        end else begin
            bit_idx <= next_bit_idx;
            ack <= next_ack;
            state <= next_state;
            recovery_running <= next_recovery_running;

            {scl_oe, sda_oe} <= {scl_oe_next, sda_oe_next};
        end
    end

    always @(*) begin
        count_value = {COUNT_WIDTH{1'b0}}; 
        count_EN = 1'b0;
        counter_reset_on_disable = 1'b0;
        running = 1'b0;

        next_bit_idx = bit_idx;
        next_ack = ack;
        next_recovery_running = recovery_running;

        {scl_oe_next, sda_oe_next} = {scl_oe, sda_oe};

        if(resp_reg==`RESP_ARB_LOST) begin
            next_state = BUS_BUSY;
        end else if(resp_reg==`RESP_SCL_ERR || resp_reg==`RESP_SDA_ERR) begin
            next_state = IDLE;
        end else begin 
            next_state = state;
            case(state)
                IDLE: begin
                    running = 1'b0;
                    count_EN = 1'b0;
                    count_value = {COUNT_WIDTH{1'b0}};
                    counter_reset_on_disable = 1'b1;
                    next_bit_idx = 3'd7;
                    next_ack = 1'b0;
                    
                    if(sda_err) begin
                        next_state = TX_LOW;
                        next_recovery_running = 1'b1;
                    end else begin
                        next_recovery_running = 1'b0;
                        if(start_detect) begin
                            next_state = BUS_BUSY;
                        end else begin
                            case(trig)
                                `TRIG_START: next_state = START_SU;
                                `TRIG_TX: next_state = TX_LOW;
                                `TRIG_RX: next_state = RX_LOW;
                                `TRIG_STOP: next_state = STOP_LOW;
                                default: next_state = IDLE;
                            endcase
                        end
                    end
                end
                BUS_BUSY: begin
                    running = 1'b0;
                    next_state = stop_detect ? BUS_BUFFER_WAIT : BUS_BUSY;
                    {scl_oe_next, sda_oe_next} = 2'b00;
                end
                BUS_BUFFER_WAIT: begin
                    running = 1'b1;
                    count_EN = count_done ? 1'b0 : 1'b1;
                    count_value = t_buf;
                    next_state = count_done ? IDLE : BUS_BUFFER_WAIT;
                    {scl_oe_next, sda_oe_next} = 2'b00;
                end
                START_SU: begin
                    running = 1'b1;
                    count_EN = count_done ? 1'b0 : scl_in;
                    count_value = t_sta_su;
                    next_state = count_done ? START_HD : START_SU;
                    {scl_oe_next, sda_oe_next} = 2'b00;
                end
                START_HD: begin
                    running = 1'b1;
                    count_EN = count_done ? 1'b0 : scl_in;
                    count_value = t_sta_hd;
                    next_state = count_done ? IDLE : START_HD;
                    {scl_oe_next, sda_oe_next} = 2'b01;
                end
                TX_LOW: begin
                    running = 1'b1;
                    count_EN = count_done ? 1'b0 : 1'b1;
                    count_value = t_low;
                    next_state = count_done ? TX_HIGH : TX_LOW;
                    {scl_oe_next, sda_oe_next} = {1'b1, (recovery_running ? 1'b0 : !tx_data[bit_idx])};
                end
                TX_HIGH: begin
                    running = 1'b1;
                    count_EN = count_done ? 1'b0 : scl_in;
                    count_value = t_high;

                    if(count_done) begin
                        if(ack) begin
                            next_state = IDLE;
                        end else begin
                            next_bit_idx = (bit_idx==0) ? 3'd7 : bit_idx-1;
                            next_state = (bit_idx==3'd0) ? RX_LOW : TX_LOW;
                            next_ack = (bit_idx==3'd0);
                        end
                    end 
                    {scl_oe_next, sda_oe_next} = {1'b0, (recovery_running ? 1'b0 : !tx_data[bit_idx])};
                end
                RX_LOW: begin
                    running = 1'b1;
                    count_EN = count_done ? 1'b0 : 1'b1;
                    count_value = t_low;
                    next_state = count_done ? RX_HIGH : RX_LOW;
                    {scl_oe_next, sda_oe_next} = 2'b10;
                end
                RX_HIGH: begin
                    running = 1'b1;
                    count_EN = count_done ? 1'b0 : scl_in;
                    count_value = t_high;

                    if(count_done) begin
                        if(recovery_running)
                            next_state = SDA_STUCK_ERROR;
                        else begin
                            if(ack) begin
                                next_state = IDLE;
                            end else begin
                                next_bit_idx = bit_idx-1;
                                next_state = (bit_idx==3'd0) ? TX_LOW : RX_LOW;
                                next_ack = (bit_idx==3'd0);
                            end
                        end
                    end
                    {scl_oe_next, sda_oe_next} = 2'b00;
                end
                STOP_LOW: begin
                    running = 1'b1;
                    count_EN = count_done ? 1'b0 : 1'b1;
                    count_value = t_low;
                    next_state = count_done ? STOP_SU : STOP_LOW;
                    {scl_oe_next, sda_oe_next} = 2'b11;
                end
                STOP_SU: begin
                    running = 1'b1;
                    count_EN = count_done ? 1'b0 : scl_in;
                    count_value = t_sto_su;
                    next_state = count_done ? STOP_HD : STOP_SU; 
                    {scl_oe_next, sda_oe_next} = 2'b01; 
                end
                STOP_HD: begin
                    running = 1'b1;
                    count_EN = count_done ? 1'b0 : scl_in;
                    count_value = t_buf;
                    next_state = count_done ? IDLE : STOP_HD; 
                    {scl_oe_next, sda_oe_next} = 2'b00;
                end
                SDA_STUCK_ERROR: begin
                    next_state = (resp_reg==`RESP_SDA_ERR) ? SDA_STUCK_ERROR : IDLE;
                    {scl_oe_next, sda_oe_next} = 2'b00;
                end
                default: begin
                    next_state = IDLE;
                    {scl_oe_next, sda_oe_next} = 2'b00;
                end 
            endcase
        end
    end
endmodule