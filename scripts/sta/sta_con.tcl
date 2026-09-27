create_clock -period 10 -name pclk [get_ports PCLK]
create_clock -period 10 -name async_clk

set_clock_groups -asynchronous -group [get_clocks pclk] -group [get_clocks async_clk]

set_clock_uncertainty 0.1 [get_clocks pclk]

set_false_path -from [get_ports PRESETn]

set_input_delay 2.5 -clock pclk [remove_from_collection [all_inputs] [get_ports {PCLK SDA SCL}]]
set_output_delay 1.5 -clock pclk [remove_from_collection [all_outputs] [get_ports {SDA SCL}]]

set_input_delay 2.5 -clock async_clk [get_ports {SDA SCL}]
set_output_delay 1.5 -clock async_clk [get_ports {SDA SCL}]

