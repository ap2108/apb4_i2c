`include "../include/i2c.vh"

module i2c_core #(
    parameter SYS_CLK_FREQ_MHZ=100,
    parameter CMD_HEADER_WIDTH=4,
    parameter DATA_WIDTH=8
)(
    input sys_clk, sys_rst_n,
    input cmd_fifo_empty, rx_fifo_full, 
    input [CMD_HEADER_WIDTH+DATA_WIDTH-1:0] cmd_in, cmd_next, 

    input [1:0] irq_clear,
    output reg cmd_fifo_r_en,
    output reg rx_fifo_w_en,
    output reg [DATA_WIDTH-1:0] rx_out, 
    output reg [4:0] irq_status_out, 
    
    output reg [2*DATA_WIDTH-1:0] tx_count, rx_count,
    
    input scl_in, sda_in,
    output scl_oe, sda_oe
);  
   
    // next_* registers for outputs ---------------------------------------------------------------------------------------------
    reg [4:0] next_irq_status_out; 
    reg [2*DATA_WIDTH-1:0] next_tx_count, next_rx_count;
    reg [DATA_WIDTH-1:0] next_rx_out;
    reg next_rx_fifo_w_en;

    // Config registers ---------------------------------------------------------------------------------------------------------
    reg [3*DATA_WIDTH-1:0] stretch_timeout, sda_stuck_timeout;
    
    reg [3:0] mode;  
    // mode[3] --> role (0-slave, 1-master), mode[2:1] --> speed bits (00-std, 01-fast, 1x-fast+), mode[0] --> address type (0-7bit, 1-10bit)  
    
    reg [1:0] target_addrh, self_addrh;
    reg [7:0] target_addrl, self_addrl;

    // Output select logic ------------------------------------------------------------------------------------------------------
    wire scl_oe_m, sda_oe_m, scl_oe_s, sda_oe_s;
    wire [7:0] rx_buf_m, rx_buf_s;
    wire [7:0] rx_buf; 
    
    assign scl_oe = mode[3] ? scl_oe_m : scl_oe_s;
    assign sda_oe = mode[3] ? sda_oe_m : sda_oe_s;

    assign rx_buf = mode[3] ? rx_buf_m : rx_buf_s;

    // S/P Detector ----------------------------------------------------------------------------------------------------------------
    wire start_detect, stop_detect;

    reg sda_d;

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if(!sys_rst_n)
            sda_d <= 1'b1;
        else
            sda_d <= sda_in;
    end

    assign start_detect = sda_d && !sda_in && scl_in;
    assign stop_detect = !sda_d && sda_in && scl_in;

    // Master bit engine Instantiation ----------------------------------------------------------------------------------------------
    reg [2:0] trig;
    reg [7:0] tx_buf, next_tx_buf;
        
    wire [2:0] resp;

    i2c_master_driver #(   
        .SYS_CLK_FREQ_MHZ(SYS_CLK_FREQ_MHZ),
        .TIMEOUT_WIDTH(3*DATA_WIDTH)
    ) u_master_driver (
        .clk(sys_clk),
        .rst_n(sys_rst_n),
        .trig(trig),
        .start_detect(start_detect),
        .stop_detect(stop_detect),
        .speed(mode[2:1]),
        .stretch_timeout(stretch_timeout),
        .sda_stuck_timeout(sda_stuck_timeout),
        .tx_data(tx_buf),
        .scl_in(scl_in),
        .sda_in(sda_in),
        .scl_oe(scl_oe_m),
        .sda_oe(sda_oe_m),
        .rx_data(rx_buf_m),
        .resp(resp)
    );
    
    // Slave bit engine Instantiation ----------------------------------------------------------------------------------------------
    reg slv_tx_en, slv_rx_en;
    wire slv_xfr_ready;

    i2c_slv_driver u_slv_driver (
        .clk(sys_clk),
        .rst_n(sys_rst_n),
        .tx_data(tx_buf),   
        .self_slv_addr({self_addrh, self_addrl}),
        .self_slv_addr_10bit(mode[0]),
        .start_detect(start_detect),
        .stop_detect(stop_detect),
        .tx_en(slv_tx_en),
        .rx_en(slv_rx_en),
        .scl_in(scl_in),
        .sda_in(sda_in),
        .scl_oe(scl_oe_s),
        .sda_oe(sda_oe_s),
        .rx_data(rx_buf_s),
        .xfr_ready(slv_xfr_ready)
    );

    // Command handling ------------------------------------------------------------------------------------------------------------

    wire [CMD_HEADER_WIDTH-1:0] cmd_in_header, cmd_next_header;
    wire [DATA_WIDTH-1:0] cmd_in_data, cmd_next_data;

    assign {cmd_in_header, cmd_in_data} = cmd_in;
    assign {cmd_next_header, cmd_next_data} = cmd_next;

    // Byte level FSM states -------------------------------------------------------------------------------------------------------
    reg [3:0] state, next_state;

    localparam [3:0] IDLE=4'd0, LOAD=4'd1, START=4'd2, ADDR1=4'd3, ADDR2=4'd4, TX_DATA=4'd5, 
        RX_DATA=4'd6, STOP=4'd7, ERROR_STICKY=4'd8, SLV_TX=4'd9, SLV_TX_WAIT=4'd10, SLV_RX=4'd11, SLV_RX_WAIT=4'd12, NACK_DRAIN=4'd13;
    
    // Byte level FSM logic -------------------------------------------------------------------------------------------------------
    reg addr10_addressed, next_addr10_addressed;
    
    localparam DATA_IDX_WIDTH = (DATA_WIDTH <= 1) ? 1 : $clog2(DATA_WIDTH);

    reg [2:0] byte_idx, next_byte_idx;

    wire [DATA_IDX_WIDTH-1:0] data_idx;
    assign data_idx = (DATA_WIDTH-1) - (8*byte_idx);

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            state <= IDLE;
            byte_idx <= 3'b0;
            addr10_addressed <= 1'b0;
            tx_buf <= 8'h00;
            rx_out <= {DATA_WIDTH{1'b0}};
            rx_fifo_w_en <= 1'b0;

            irq_status_out <= 5'b0;
            tx_count <= {(2*DATA_WIDTH){1'b0}};
            rx_count <= {(2*DATA_WIDTH){1'b0}};

            target_addrh <= 2'b0;
            target_addrl <= 8'b0;

            self_addrh <= 2'b0;
            self_addrl <= 8'b0;

            mode <= 4'b1000;
            stretch_timeout <= {(3*DATA_WIDTH){1'b1}};
            sda_stuck_timeout <= {(3*DATA_WIDTH){1'b1}};
        end else begin
            state <= next_state;                                     
            byte_idx <= next_byte_idx;
            addr10_addressed <= next_addr10_addressed;
            tx_buf <= next_tx_buf;
            rx_out <= next_rx_out;
            rx_fifo_w_en <= next_rx_fifo_w_en;

            irq_status_out <= next_irq_status_out;
            tx_count <= next_tx_count;
            rx_count <= next_rx_count;

            if(state==IDLE) case(cmd_in_header) 
                `CMD_TARGET_ADDRH: target_addrh <= cmd_in_data[1:0];
                `CMD_TARGET_ADDRL: target_addrl <= cmd_in_data[7:0];
                `CMD_SELF_ADDRH: self_addrh <= cmd_in_data[1:0];
                `CMD_SELF_ADDRL: self_addrl <= cmd_in_data[7:0];

                `CMD_MODE: mode <= cmd_in_data[3:0];
                `CMD_SCL_TIMEOUT_2: stretch_timeout <= {cmd_in_data, stretch_timeout[2*DATA_WIDTH-1:0]};
                `CMD_SCL_TIMEOUT_1: stretch_timeout <= {stretch_timeout[3*DATA_WIDTH-1:2*DATA_WIDTH], cmd_in_data, stretch_timeout[DATA_WIDTH-1:0]};
                `CMD_SCL_TIMEOUT_0: stretch_timeout <= {stretch_timeout[3*DATA_WIDTH-1:DATA_WIDTH], cmd_in_data};

                `CMD_SDA_TIMEOUT_2: sda_stuck_timeout <= {cmd_in_data, sda_stuck_timeout[2*DATA_WIDTH-1:0]};
                `CMD_SDA_TIMEOUT_1: sda_stuck_timeout <= {sda_stuck_timeout[3*DATA_WIDTH-1:2*DATA_WIDTH], cmd_in_data, sda_stuck_timeout[DATA_WIDTH-1:0]};
                `CMD_SDA_TIMEOUT_0: sda_stuck_timeout <= {sda_stuck_timeout[3*DATA_WIDTH-1:DATA_WIDTH], cmd_in_data};
            endcase 
        end
    end

    always @(*) begin
        next_rx_fifo_w_en = 1'b0;

        next_state = state;
        trig = `TRIG_IDLE;

        next_tx_buf = tx_buf;
        next_addr10_addressed = addr10_addressed;

        next_irq_status_out = irq_status_out;
        cmd_fifo_r_en = 1'b0;

        next_rx_out = rx_out;
        next_byte_idx = byte_idx;

        next_rx_count = rx_count;
        next_tx_count = tx_count;
        
        slv_tx_en = 1'b0;
        slv_rx_en = 1'b0;

        case (mode[3] ? resp : `RESP_IDLE)
            `RESP_SDA_ERR: begin    
                next_irq_status_out[0] = 1'b1;
                next_state = IDLE;
            end
            `RESP_BUS_BUSY: begin
                next_irq_status_out[1] = 1'b1;
                next_state = IDLE;
            end
            `RESP_SCL_ERR: begin
                next_irq_status_out[2] = 1'b1;
                next_state = IDLE;
            end 
            `RESP_ARB_LOST: begin
                next_irq_status_out[3] = 1'b1;
                next_state = ERROR_STICKY;
            end
            default: begin
                case(state)
                    IDLE: begin
                        next_irq_status_out[2:0] = 3'b0;
                        next_addr10_addressed = 1'b0;
                        next_byte_idx = 3'b0;

                        if(resp==`RESP_IDLE) begin
                            if(cmd_in_header==`CMD_TX) begin
                                next_state = mode[3] ? START : SLV_TX;
                                trig = mode[3] ? `TRIG_START : `TRIG_IDLE;
                            end else if(cmd_in_header==`CMD_RX && !rx_fifo_full) begin
                                next_state = mode[3] ? START : SLV_RX;
                                trig = mode[3] ? `TRIG_START : `TRIG_IDLE;
                            end else begin
                                cmd_fifo_r_en = 1'b1;
                            end
                        end
                    end
                    START: begin
                        if(resp==`RESP_IDLE) begin
                            trig = `TRIG_TX;    
                            next_state = ADDR1;
                            
                            if(cmd_in_header==`CMD_TX) begin
                                if(mode[0]) 
                                    next_tx_buf = {5'b11110, target_addrh, 1'b0};
                                else
                                    next_tx_buf = {target_addrl[6:0], 1'b0};
                            end else begin
                                if(mode[0])     
                                    next_tx_buf = addr10_addressed ? {5'b11110, target_addrh, 1'b1} : {5'b11110, target_addrh, 1'b0};
                                else
                                    next_tx_buf = {target_addrl[6:0], 1'b1};
                            end
                        end
                    end
                    ADDR1: begin
                        if(resp==`RESP_IDLE) begin
                            if(rx_buf[7]) begin
                                next_state = STOP;
                                trig = `TRIG_STOP;
                                next_irq_status_out[4] = 1'b1;
                            end else begin
                                if(mode[0] && !addr10_addressed) begin
                                    next_state = ADDR2;
                                    trig = `TRIG_TX;
                                    next_tx_buf = target_addrl;
                                end else begin
                                    next_state = (cmd_in_header==`CMD_TX) ? TX_DATA : RX_DATA;
                                    if(cmd_in_header==`CMD_TX) begin
                                        next_state = TX_DATA;
                                        trig = `TRIG_TX;
                                        next_tx_buf = cmd_in_data[DATA_WIDTH-1 -: 8];
                                        next_byte_idx = 3'b1;
                                    end else begin
                                        next_state = RX_DATA;
                                        trig = `TRIG_RX;
                                    end
                                end
                            end
                        end 
                    end
                    ADDR2: begin
                        if(resp==`RESP_IDLE) begin
                            if(rx_buf[7]) begin
                                next_state = STOP;
                                trig = `TRIG_STOP;
                                next_irq_status_out[4] = 1'b1;
                            end else begin
                                next_addr10_addressed = 1'b1;
                                if(cmd_in_header==`CMD_TX) begin
                                    next_state = TX_DATA;
                                    trig = `TRIG_TX;
                                    next_tx_buf = cmd_in_data[DATA_WIDTH-1 -: 8];
                                    next_byte_idx = 3'b1;
                                end else begin
                                    next_state = START;
                                    trig = `TRIG_START;
                                end
                            end
                        end 
                    end
                    TX_DATA: begin
                        if(resp==`RESP_IDLE) begin
                            if(rx_buf[7]) begin
                                next_state = STOP;
                                trig = `TRIG_STOP;
                                next_irq_status_out[4] = 1'b1;
                            end else begin
                                if(byte_idx < (DATA_WIDTH/8)) begin
                                    next_state = TX_DATA;
                                    trig = `TRIG_TX;
                                    next_tx_buf = cmd_in_data[data_idx -: 8];
                                    next_byte_idx = byte_idx+1;
                                end else begin
                                    next_tx_count = tx_count+1;
                                    cmd_fifo_r_en = 1'b1;

                                    if(cmd_next_header==`CMD_TX) begin
                                        next_state = TX_DATA;
                                        next_tx_buf = cmd_next_data[DATA_WIDTH-1 -: 8];
                                        next_byte_idx = 3'b1;
                                        trig = `TRIG_TX;
                                    end else if(cmd_next_header==`CMD_RX && !rx_fifo_full) begin
                                        next_byte_idx = 3'b0;
                                        next_state = START;
                                        trig = `TRIG_START;
                                    end else begin
                                        next_byte_idx = 3'b0;
                                        next_state = STOP;
                                        trig = `TRIG_STOP;
                                    end
                                end
                            end
                        end 
                    end
                    RX_DATA: begin
                        next_tx_buf = (cmd_next_header!=`CMD_RX && byte_idx==(DATA_WIDTH/8)-1) ? 8'b1000_0000 : 8'b0;
                        
                        if(resp==`RESP_IDLE) begin
                            next_rx_out[data_idx -: 8] = rx_buf;

                            if(byte_idx+1 < (DATA_WIDTH/8)) begin
                                next_state = RX_DATA;
                                trig = `TRIG_RX;
                                next_byte_idx = byte_idx+1;
                            end else begin
                                next_byte_idx = 3'b0;
                                next_rx_count = rx_count+1;
                                cmd_fifo_r_en = 1'b1;
                                next_rx_fifo_w_en = 1'b1;        

                                if(cmd_next_header==`CMD_RX && !rx_fifo_full) begin
                                    next_state = RX_DATA;
                                    trig = `TRIG_RX;
                                end else if(cmd_next_header==`CMD_TX) begin
                                    next_state = START;
                                    trig = `TRIG_START;                            
                                end else begin
                                    next_state = STOP;
                                    trig = `TRIG_STOP;
                                end
                            end
                        end
                    end
                    STOP: begin
                        if (resp == `RESP_IDLE) begin
                            if (irq_status_out[4])
                                next_state = NACK_DRAIN;
                            else
                                next_state = irq_status_out[3] ? ERROR_STICKY : IDLE;
                        end     
                    end
                    NACK_DRAIN: begin
                        if(cmd_fifo_empty) 
                            next_state = ERROR_STICKY;
                        else if ((cmd_in_header==`CMD_TX) || (cmd_in_header==`CMD_RX))
                            cmd_fifo_r_en = 1'b1;
                        else 
                            next_state = ERROR_STICKY;
                    end
                    ERROR_STICKY: begin
                        next_irq_status_out[4] = irq_clear[0] ? 1'b0 : irq_status_out[4];   // NACK 
                        next_irq_status_out[3] = irq_clear[1] ? 1'b0 : irq_status_out[3];   // ARB_LOST

                        next_state = (next_irq_status_out[4] || next_irq_status_out[3]) ? ERROR_STICKY : IDLE;
                    end
                    SLV_TX: begin
                        slv_tx_en = 1'b1;
                        next_tx_buf = cmd_in_data[data_idx -: 8];
                        next_byte_idx = byte_idx+1;
                        next_state = SLV_TX_WAIT;
                    end
                    SLV_TX_WAIT: begin
                        slv_tx_en = 1'b1;

                        if(slv_xfr_ready) begin
                            if(byte_idx < (DATA_WIDTH/8)) begin
                                next_state = SLV_TX;
                            end else begin
                                next_tx_count = tx_count+1;
                                cmd_fifo_r_en = 1'b1;
                                next_byte_idx = 3'b0;
                                
                                if(cmd_next_header==`CMD_TX) begin
                                    next_state = SLV_TX;
                                end else begin
                                    next_state = IDLE;
                                    slv_tx_en = 1'b0;
                                end
                            end
                        end 
                    end
                    SLV_RX: begin
                        slv_rx_en = 1'b1;
                        next_state = SLV_RX_WAIT;
                    end
                    SLV_RX_WAIT: begin
                        slv_rx_en = 1'b1;

                        if(slv_xfr_ready) begin
                            if(byte_idx+1 < (DATA_WIDTH/8)) begin
                                next_state = SLV_RX;
                                next_rx_out[data_idx -: 8] = rx_buf;
                                next_byte_idx = byte_idx+1;
                            end else begin
                                next_rx_count = rx_count+1;
                                next_byte_idx = 3'b0;
                                cmd_fifo_r_en = 1'b1;
                                next_rx_fifo_w_en = 1'b1;        
                                next_rx_out[data_idx -: 8] = rx_buf;
                                
                                if(cmd_next_header==`CMD_RX && !rx_fifo_full) begin
                                    next_state = SLV_RX;
                                end else begin
                                    next_state = IDLE;
                                    slv_rx_en = 1'b0;
                                end
                            end
                        end 
                    end
                endcase
            end
        endcase
    end
endmodule
