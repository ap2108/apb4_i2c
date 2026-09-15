`ifndef I2C_VH
`define I2C_VH

// I2C driver triggers
`define TRIG_START 3'b000
`define TRIG_TX 3'b001
`define TRIG_RX 3'b010
`define TRIG_STOP 3'b011
`define TRIG_IDLE 3'b100

// I2C driver responses 
`define RESP_IDLE 3'd0
`define RESP_INPROGRESS 3'd1
`define RESP_ARB_LOST 3'd2
`define RESP_SDA_ERR 3'd3
`define RESP_SCL_ERR 3'd4
`define RESP_BUS_BUSY 3'd5

// Commands 
`define CMD_TARGET_ADDRH 4'b0000
`define CMD_TARGET_ADDRL 4'b0001
`define CMD_MODE 4'b0010
`define CMD_SCL_TIMEOUT_2 4'b0011
`define CMD_SCL_TIMEOUT_1 4'b0100
`define CMD_SCL_TIMEOUT_0 4'b0101
`define CMD_SDA_TIMEOUT_2 4'b0110
`define CMD_SDA_TIMEOUT_1 4'b0111
`define CMD_SDA_TIMEOUT_0 4'b1000
`define CMD_TX 4'b1001
`define CMD_RX 4'b1010
`define CMD_IGNORE 4'b1011
`define CMD_SELF_ADDRH 4'b1100
`define CMD_SELF_ADDRL 4'b1101

// APB Addresses
// Write only
`define ADDR_I2C_SLV_TARGET_ADDRH 0
`define ADDR_I2C_SLV_TARGET_ADDRL 1
`define ADDR_MODE 2
`define ADDR_SCL_TIMEOUT2 3
`define ADDR_SCL_TIMEOUT1 4
`define ADDR_SCL_TIMEOUT0 5

`define ADDR_SDA_TIMEOUT2 6
`define ADDR_SDA_TIMEOUT1 7
`define ADDR_SDA_TIMEOUT0 8

`define ADDR_TX_DATA 9
`define ADDR_RX_DATA 10
`define ADDR_IRQ_CLEAR 11

`define ADDR_I2C_SLV_SELF_ADDRH 12
`define ADDR_I2C_SLV_SELF_ADDRL 13

// Read only registers
`define ADDR_RX_FIFO 16
`define ADDR_TX_COUNT_HIGH 17
`define ADDR_TX_COUNT_LOW 18

`define ADDR_RX_COUNT_HIGH 19
`define ADDR_RX_COUNT_LOW 20

`define ADDR_IRQ_STATUS 21

// Timing 
// Standard (100 kHz)
`define T_STA_SU_STD 4700
`define T_STA_HD_STD 4000
`define T_LOW_STD 6000
`define T_HIGH_STD 4000
`define T_STO_SU_STD 4000
`define T_BUF_STD 4700

// Fast (400 kHz)
`define T_STA_SU_FAST 600
`define T_STA_HD_FAST 600
`define T_LOW_FAST 1500
`define T_HIGH_FAST 1000
`define T_STO_SU_FAST 600
`define T_BUF_FAST 1300     

// Fast+ (1 MHz)
`define T_STA_SU_FASTP 260
`define T_STA_HD_FASTP 260
`define T_LOW_FASTP 600
`define T_HIGH_FASTP 400
`define T_STO_SU_FASTP 260
`define T_BUF_FASTP 500

`define NCYCLES(FREQ_MHZ, NS) (((FREQ_MHZ * NS)/1000) + 1)

`endif