`include "../include/i2c.vh"

module apb4_i2c_top #(
    parameter PCLK_FREQ_MHZ=100,
    parameter ADDR_WIDTH=5,
    parameter CMD_HEADER_WIDTH=4,
    parameter DATA_WIDTH=8,
    parameter FIFO_DEPTH=32
)(
    input PCLK, PRESETn, 
    input [ADDR_WIDTH-1:0] PADDR,
    input PSEL, PENABLE, PWRITE,
    input [DATA_WIDTH-1:0] PWDATA,
    
    output PREADY, PSLVERR,
    output reg [DATA_WIDTH-1:0] PRDATA,

    inout SCL, SDA
);  

    // 2FF Synchronizer ----------------------------------------------------------------------
    wire scl_in_sync, sda_in_sync;

    i2c_sync2 u_sync_scl (
        .clk(PCLK),
        .rst_n(PRESETn),
        .d(SCL),
        .q(scl_in_sync)
    );

    i2c_sync2 u_sync_sda (
        .clk(PCLK),
        .rst_n(PRESETn),
        .d(SDA),
        .q(sda_in_sync)
    );

    // Glitch Filter --------------------------------------------------------------------------
    wire scl_in_filtered, sda_in_filtered;

    i2c_glitchfilter #(
        .SYS_CLK_FREQ_MHZ(PCLK_FREQ_MHZ),
        .MAX_GLITCH_DURATION_NS(50) 
    ) u_gf_scl (
        .clk(PCLK),
        .rst_n(PRESETn),
        .d(scl_in_sync),
        .q(scl_in_filtered)
    );

    i2c_glitchfilter #(
        .SYS_CLK_FREQ_MHZ(PCLK_FREQ_MHZ),
        .MAX_GLITCH_DURATION_NS(50)
    ) u_gf_sda (
        .clk(PCLK),
        .rst_n(PRESETn),
        .d(sda_in_sync),
        .q(sda_in_filtered)
    );

    // Read only APB registers ------------------------------------------------------------------------------------------------
    wire [2*DATA_WIDTH-1:0] tx_count, rx_count;

    wire [7:0] irq_status;

    // CMD and RX Fifo --------------------------------------------------------------------------------------------------------

    // Interconnect nets
    wire cmd_fifo_empty, rx_fifo_full;
    wire [CMD_HEADER_WIDTH+DATA_WIDTH-1:0] i2c_cmd;
    wire [DATA_WIDTH-1:0] i2c_rx_data;

    wire cmd_fifo_r_en, rx_fifo_w_en;
    
    // Other nets
    wire cmd_fifo_full, rx_fifo_empty;

    reg cmd_fifo_w_en;

    reg [CMD_HEADER_WIDTH-1:0] cmd;

    wire [DATA_WIDTH-1:0] rx_fifo_rdata;

    wire [2:0] irq_clear;

    // Command fifo 
    fifo #(
        .DATA_WIDTH(CMD_HEADER_WIDTH+DATA_WIDTH),
        .DEPTH(FIFO_DEPTH)
    ) u_cmd_fifo(
        .clk(PCLK),
        .rst_n(PRESETn),
        .r_en(cmd_fifo_r_en),
        .w_en(cmd_fifo_w_en),
        .wr_data({cmd, PWDATA}),
        .rd_data(i2c_cmd),
        .full(cmd_fifo_full),
        .empty(cmd_fifo_empty)
    );

    // RX Fifo 
    fifo #(
        .DATA_WIDTH(DATA_WIDTH),
        .DEPTH(FIFO_DEPTH)
    ) u_rx_fifo(
        .clk(PCLK),
        .rst_n(PRESETn),
        .r_en(PSEL && PENABLE && !PWRITE && (PADDR==`ADDR_RX_FIFO)),
        .w_en(rx_fifo_w_en),
        .wr_data(i2c_rx_data),
        .rd_data(rx_fifo_rdata),
        .full(rx_fifo_full),
        .empty(rx_fifo_empty)
    );

    // I2C Core ---------------------------------------------------------------------------------------------------------------
    i2c_core #(
        .SYS_CLK_FREQ_MHZ (PCLK_FREQ_MHZ),
        .CMD_HEADER_WIDTH (CMD_HEADER_WIDTH),  
        .DATA_WIDTH (DATA_WIDTH)      
    ) u_i2c_core (
        .sys_clk(PCLK),
        .sys_rst_n(PRESETn),
        .cmd_fifo_empty(cmd_fifo_empty),
        .rx_fifo_full(rx_fifo_full),
        .cmd_in(i2c_cmd),     
        .cmd_fifo_r_en(cmd_fifo_r_en),     
        .rx_fifo_w_en(rx_fifo_w_en),      
        .rx_out(i2c_rx_data),      
        .irq_status_out(irq_status[4:0]),
        .irq_clear(irq_clear[2:1]),
        .tx_count(tx_count),
        .rx_count(rx_count),
        .scl_in(scl_in_filtered),
        .sda_in(sda_in_filtered),
        .scl_oe(scl_oe),
        .sda_oe(sda_oe)
    );

    // I2C Bus --------------------------------------------------------------------------------------------------------------

    assign SCL = scl_oe ? 1'b0 : 1'bz;
    assign SDA = sda_oe ? 1'b0 : 1'bz;
    
    // Interrupts and error handling -----------------------------------------------------------------------------------------

    // irq_status: | RX_FIFO_EMPTY | CMD_FIFO_FULL | ILLEGAL_ACCESS |  NACK  | ARB_LOST |  SCL_HUNG | BUS_BUSY |  SDA_HUNG |
    // bits (8)  : |       7       |       6       |        5       |    4   |     3    |     2     |     1    |     0     |
    // Type      : |     Live      |     Live      |     Sticky     | Sticky |  Sticky  |   Fatal   |   Live   |   Fatal   |

    assign irq_status[6] = cmd_fifo_full;
    assign irq_status[7] = rx_fifo_empty;

    reg illegal_access;
    assign irq_status[5] = illegal_access;                                                                                                                       

    always @(posedge PCLK or negedge PRESETn) begin
        if(!PRESETn) 
            illegal_access <= 1'b0;
        else begin
            if((PSEL && PENABLE) && ((PWRITE && PADDR[4]) || (!PWRITE && !PADDR[4])))
                illegal_access <= 1'b1;
            else if(irq_clear[0])
                illegal_access <= 1'b0;
        end
    end

    // irq_clear: | ARB_LOST | NACK | ILLEGAL_ACCESS |
    // bits (3) : |     2    |   1  |       0        |

    assign irq_clear = ((PSEL && PENABLE && PWRITE) && (PADDR==`ADDR_IRQ_CLEAR)) ? PWDATA[2:0] : 3'b0;

    assign PSLVERR = (PSEL && PENABLE) && (|irq_status);

    // APB address decoder -----------------------------------------------------------------------------------------------------
    always @(*) begin
        cmd_fifo_w_en = 1'b0;
        cmd = `CMD_IGNORE;
        PRDATA = {DATA_WIDTH{1'b0}};
        
        if(PSEL && PENABLE) begin
            if(PWRITE) begin
                if(!cmd_fifo_full) begin
                    cmd_fifo_w_en = 1'b1;
                    case(PADDR)
                        `ADDR_I2C_SLV_TARGET_ADDRH: cmd = `CMD_TARGET_ADDRH;
                        `ADDR_I2C_SLV_TARGET_ADDRL: cmd = `CMD_TARGET_ADDRL;
                        `ADDR_MODE: cmd = `CMD_MODE;
                        `ADDR_SCL_TIMEOUT2: cmd = `CMD_SCL_TIMEOUT_2;
                        `ADDR_SCL_TIMEOUT1: cmd = `CMD_SCL_TIMEOUT_1;
                        `ADDR_SCL_TIMEOUT0: cmd = `CMD_SCL_TIMEOUT_0;
                        `ADDR_SDA_TIMEOUT2: cmd = `CMD_SDA_TIMEOUT_2;
                        `ADDR_SDA_TIMEOUT1: cmd = `CMD_SDA_TIMEOUT_1;
                        `ADDR_SDA_TIMEOUT0: cmd = `CMD_SDA_TIMEOUT_0;
                        `ADDR_TX_DATA: cmd = `CMD_TX;
                        `ADDR_RX_DATA: cmd = `CMD_RX;
                        `ADDR_I2C_SLV_SELF_ADDRH: cmd = `CMD_SELF_ADDRH;
                        `ADDR_I2C_SLV_SELF_ADDRL: cmd = `CMD_SELF_ADDRL;
                        default: cmd_fifo_w_en = 1'b0;
                    endcase
                end else begin
                    cmd_fifo_w_en = 1'b0;
                end
            end else begin
                case(PADDR)
                    `ADDR_RX_FIFO: begin
                        PRDATA = rx_fifo_empty ? {DATA_WIDTH{1'b0}} : rx_fifo_rdata;
                    end
                    `ADDR_TX_COUNT_HIGH:
                        PRDATA = tx_count[(2*DATA_WIDTH)-1:DATA_WIDTH];

                    `ADDR_TX_COUNT_LOW:
                        PRDATA = tx_count[DATA_WIDTH-1:0];

                    `ADDR_RX_COUNT_HIGH:
                        PRDATA = rx_count[(2*DATA_WIDTH)-1:DATA_WIDTH];

                    `ADDR_RX_COUNT_LOW:
                        PRDATA = rx_count[DATA_WIDTH-1:0];

                    `ADDR_IRQ_STATUS:
                        PRDATA = (DATA_WIDTH>8) ? {{(DATA_WIDTH-8){1'b0}}, irq_status} : irq_status;

                    default:
                        PRDATA = {DATA_WIDTH{1'b0}};
                endcase
            end
        end
    end
    
    assign PREADY = 1'b1;

    // -------------------------------------------------------------------------------------------------------------------------
    
endmodule