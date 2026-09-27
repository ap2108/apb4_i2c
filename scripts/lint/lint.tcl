read_file -type verilog {../include/i2c.vh ../src/apb4_i2c_top.v ../src/fifo.v ../src/i2c_core.v ../src/i2c_counter.v ../src/i2c_glitchfilter.v ../src/i2c_master_driver.v ../src/i2c_slv_driver.v ../src/i2c_start_stop_detector.v ../src/i2c_sync2.v}

#read_file -type sgdc waiver.sgdc

current_goal lint/lint_rtl -alltop
run_goal

current_goal lint/lint_rtl_enhanced -alltop
run_goal
