`timescale 1ns/1ps
`include "../include/i2c.vh"

module tb_apb4_i2c_top_2dut;

localparam PCLK_FREQ_MHZ       = 100;
localparam ADDR_WIDTH          = 5;
localparam CMD_HEADER_WIDTH    = 4;
localparam DATA_WIDTH          = 8;
localparam FIFO_DEPTH          = 16;

// CLOCK & RESET
reg PCLK = 1'b0;
always #5 PCLK = ~PCLK;   // 100 MHz

reg PRESETn;

// APB SIGNALS
reg  [ADDR_WIDTH-1:0]  m_PADDR;
reg                    m_PSEL;
reg                    m_PENABLE;
reg                    m_PWRITE;
reg  [DATA_WIDTH-1:0]  m_PWDATA;
wire                   m_PREADY;
wire                   m_PSLVERR;
wire [DATA_WIDTH-1:0]  m_PRDATA;

reg  [ADDR_WIDTH-1:0]  s_PADDR;
reg                    s_PSEL;
reg                    s_PENABLE;
reg                    s_PWRITE;
reg  [DATA_WIDTH-1:0]  s_PWDATA;
wire                   s_PREADY;
wire                   s_PSLVERR;
wire [DATA_WIDTH-1:0]  s_PRDATA;

// SHARED BUS WITH EXPLICIT PULLUPS
wire SCL;
wire SDA;

pullup p_scl(SCL);
pullup p_sda(SDA);

// DUT INSTANTIATIONS
apb4_i2c_top #(
    .PCLK_FREQ_MHZ(PCLK_FREQ_MHZ), .ADDR_WIDTH(ADDR_WIDTH),
    .CMD_HEADER_WIDTH(CMD_HEADER_WIDTH), .DATA_WIDTH(DATA_WIDTH), .FIFO_DEPTH(FIFO_DEPTH)
) dut_master (
    .PCLK(PCLK), .PRESETn(PRESETn),
    .PADDR(m_PADDR), .PSEL(m_PSEL), .PENABLE(m_PENABLE), .PWRITE(m_PWRITE), .PWDATA(m_PWDATA),
    .PREADY(m_PREADY), .PSLVERR(m_PSLVERR), .PRDATA(m_PRDATA),
    .SCL(SCL), .SDA(SDA)
);

apb4_i2c_top #(
    .PCLK_FREQ_MHZ(PCLK_FREQ_MHZ), .ADDR_WIDTH(ADDR_WIDTH),
    .CMD_HEADER_WIDTH(CMD_HEADER_WIDTH), .DATA_WIDTH(DATA_WIDTH), .FIFO_DEPTH(FIFO_DEPTH)
) dut_slave (
    .PCLK(PCLK), .PRESETn(PRESETn),
    .PADDR(s_PADDR), .PSEL(s_PSEL), .PENABLE(s_PENABLE), .PWRITE(s_PWRITE), .PWDATA(s_PWDATA),
    .PREADY(s_PREADY), .PSLVERR(s_PSLVERR), .PRDATA(s_PRDATA),
    .SCL(SCL), .SDA(SDA)
);

// APB NON-BLOCKING DRIVER TASKS
task automatic apb_write_master(input [ADDR_WIDTH-1:0] addr, input [DATA_WIDTH-1:0] data);
begin
    @(posedge PCLK);
    m_PADDR   <= addr;
    m_PWDATA  <= data;
    m_PWRITE  <= 1'b1;
    m_PSEL    <= 1'b1;
    m_PENABLE <= 1'b0;

    @(posedge PCLK);
    m_PENABLE <= 1'b1;

    while (!m_PREADY) @(posedge PCLK);

    @(posedge PCLK);
    m_PSEL    <= 1'b0;
    m_PENABLE <= 1'b0;
    m_PWRITE  <= 1'b0;
end
endtask

task automatic apb_write_slave(input [ADDR_WIDTH-1:0] addr, input [DATA_WIDTH-1:0] data);
begin
    @(posedge PCLK);
    s_PADDR   <= addr;
    s_PWDATA  <= data;
    s_PWRITE  <= 1'b1;
    s_PSEL    <= 1'b1;
    s_PENABLE <= 1'b0;

    @(posedge PCLK);
    s_PENABLE <= 1'b1;

    while (!s_PREADY) @(posedge PCLK);

    @(posedge PCLK);
    s_PSEL    <= 1'b0;
    s_PENABLE <= 1'b0;
    s_PWRITE  <= 1'b0;
end
endtask

task automatic apb_read_master(input [ADDR_WIDTH-1:0] addr, output [DATA_WIDTH-1:0] data);
begin
    @(posedge PCLK);
    m_PADDR   <= addr;
    m_PWRITE  <= 1'b0;
    m_PSEL    <= 1'b1;
    m_PENABLE <= 1'b0;

    @(posedge PCLK);
    m_PENABLE <= 1'b1;

    @(posedge PCLK);
    data = m_PRDATA;

    m_PSEL    <= 1'b0;
    m_PENABLE <= 1'b0;
end
endtask

reg rrr;

task automatic apb_read_slave(input [ADDR_WIDTH-1:0] addr, output [DATA_WIDTH-1:0] data);
begin
    @(posedge PCLK);
    s_PADDR   <= addr;
    s_PWRITE  <= 1'b0;
    s_PSEL    <= 1'b1;
    s_PENABLE <= 1'b0;

    @(posedge PCLK);
    s_PENABLE <= 1'b1;

    @(posedge PCLK);
    data = s_PRDATA;

    s_PSEL    <= 1'b0;
    s_PENABLE <= 1'b0;
end
endtask


integer errors;

task automatic check_master_rx(input [7:0] expected, input integer index);
    reg [7:0] actual;
begin
    apb_read_master(`ADDR_RX_FIFO, actual);
    $display("[%0t] TEST2 MASTER RX[%0d] = %02h  expected = %02h", $time, index, actual, expected);
    if (actual !== expected) begin
        $display("ERROR: TEST2 MASTER RX[%0d] mismatch", index);
        errors = errors + 1;
    end
end
endtask

task automatic check_slave_rx(input [7:0] expected, input integer index);
    reg [7:0] actual;
begin
    apb_read_slave(`ADDR_RX_FIFO, actual);
    $display("[%0t] TEST1 SLAVE RX[%0d] = %02h  expected = %02h", $time, index, actual, expected);
    if (actual !== expected) begin
        $display("ERROR: TEST1 SLAVE RX[%0d] mismatch", index);
        errors = errors + 1;
    end
end
endtask

initial begin
    errors = 0;
    PRESETn = 1'b0;
    m_PADDR = 0; m_PSEL = 0; m_PENABLE = 0; m_PWRITE = 0; m_PWDATA = 0;
    s_PADDR = 0; s_PSEL = 0; s_PENABLE = 0; s_PWRITE = 0; s_PWDATA = 0;

    repeat (5) @(posedge PCLK);
    PRESETn = 1'b1;
    repeat (5) @(posedge PCLK);

    // TEST 1
    $display("\n============================================================");
    $display("TEST 1: MASTER TX -> SLAVE RX");
    $display("============================================================");
    apb_write_master(`ADDR_MODE, 8'h08);
    apb_write_master(`ADDR_I2C_SLV_TARGET_ADDRH, 8'h00);
    apb_write_master(`ADDR_I2C_SLV_TARGET_ADDRL, 8'h00);
    apb_write_master(`ADDR_SCL_TIMEOUT2, 8'hFF);
    apb_write_master(`ADDR_SCL_TIMEOUT1, 8'hFF);
    apb_write_master(`ADDR_SCL_TIMEOUT0, 8'hFF);
    apb_write_master(`ADDR_SDA_TIMEOUT2, 8'hFF);
    apb_write_master(`ADDR_SDA_TIMEOUT1, 8'hFF);
    apb_write_master(`ADDR_SDA_TIMEOUT0, 8'hFF);

    apb_write_slave(`ADDR_MODE, 8'h00);
    apb_write_slave(`ADDR_I2C_SLV_SELF_ADDRH, 8'h00);
    apb_write_slave(`ADDR_I2C_SLV_SELF_ADDRL, 8'h00);
    apb_write_slave(`ADDR_SCL_TIMEOUT2, 8'hFF);
    apb_write_slave(`ADDR_SCL_TIMEOUT1, 8'hFF);
    apb_write_slave(`ADDR_SCL_TIMEOUT0, 8'hFF);
    apb_write_slave(`ADDR_SDA_TIMEOUT2, 8'hFF);
    apb_write_slave(`ADDR_SDA_TIMEOUT1, 8'hFF);
    apb_write_slave(`ADDR_SDA_TIMEOUT0, 8'hFF);

    apb_write_slave(`ADDR_RX_DATA, 8'h00);
    apb_write_slave(`ADDR_RX_DATA, 8'h00);
    apb_write_slave(`ADDR_RX_DATA, 8'h00);
    apb_write_slave(`ADDR_RX_DATA, 8'h00);
    apb_write_slave(`ADDR_RX_DATA, 8'h00);


    #1000;
    apb_write_master(`ADDR_TX_DATA, 8'h11);
    apb_write_master(`ADDR_TX_DATA, 8'h22);
    apb_write_master(`ADDR_TX_DATA, 8'h33);
    apb_write_master(`ADDR_TX_DATA, 8'h44);
    apb_write_master(`ADDR_TX_DATA, 8'h55);
    
    #1000000; // Allow full transaction runtime

    check_slave_rx(8'h11, 0);
    check_slave_rx(8'h22, 1);
    check_slave_rx(8'h33, 2);
    check_slave_rx(8'h44, 3);
    check_slave_rx(8'h55, 4);

    // RESET FOR TEST 2
    #1000000; 

    PRESETn = 1'b0;
    repeat (5) @(posedge PCLK);
    PRESETn = 1'b1;
    repeat (5) @(posedge PCLK);

    // TEST 2
    $display("\n============================================================");
    $display("TEST 2: MASTER RX <- SLAVE TX");
    $display("============================================================");
    apb_write_master(`ADDR_MODE, 8'h08);
    apb_write_master(`ADDR_I2C_SLV_TARGET_ADDRH, 8'h00);
    apb_write_master(`ADDR_I2C_SLV_TARGET_ADDRL, 8'h00);

    apb_write_slave(`ADDR_MODE, 8'h00);
    apb_write_slave(`ADDR_I2C_SLV_SELF_ADDRH, 8'h00);
    apb_write_slave(`ADDR_I2C_SLV_SELF_ADDRL, 8'h00);

    repeat(100) @(posedge PCLK);

    apb_write_slave(`ADDR_TX_DATA, 8'hA1);
    apb_write_slave(`ADDR_TX_DATA, 8'hB2);
    apb_write_slave(`ADDR_TX_DATA, 8'hC3);
    apb_write_slave(`ADDR_TX_DATA, 8'hD4);
    apb_write_slave(`ADDR_TX_DATA, 8'hE5);

    apb_write_master(`ADDR_RX_DATA, 8'h00);
    apb_write_master(`ADDR_RX_DATA, 8'h00);
    apb_write_master(`ADDR_RX_DATA, 8'h00);
    apb_write_master(`ADDR_RX_DATA, 8'h00);
    apb_write_master(`ADDR_RX_DATA, 8'h00);

    #1000000; // Allow full transaction runtime

    check_master_rx(8'hA1, 0);
    check_master_rx(8'hB2, 1);
    check_master_rx(8'hC3, 2);
    check_master_rx(8'hD4, 3);
    check_master_rx(8'hE5, 4);

    $display("\n============================================================");
    if (errors == 0) $display("ALL TESTS PASS");
    else $display("TEST FAILED : TOTAL ERRORS = %0d", errors);
    $display("============================================================");

    #1000000;
    $finish;
end

initial begin
    #10000000;
    $display("\nTIMEOUT - SIMULATION DID NOT FINISH WITHIN EXPECTED DURATION");
    $finish;
end

initial begin
    $dumpfile("sim/wave.vcd");
    $dumpvars(0, tb_apb4_i2c_top_2dut);
end
endmodule