set search_path [list . /fetools/work_area/frontend/SAED32_EDK/lib/stdcell_lvt/db_nldm]
 
set target_library [list saed32lvt_ss0p75v25c.db]
set link_library [list * $target_library]
 
set designer "adithya"
 
analyze -format verilog [glob ../src/*.v]
elaborate apb4_i2c_top

current_design apb4_i2c_top
link

source synth_con.sdc

write -format ddc -hier -output unmapped/apb4_i2c_top.ddc

compile_ultra -incremental -only_design_rule
 
write -format verilog -hierarchy -output netlist/apb4_i2c_top_netlist.v
write -format ddc -hierarchy -output mapped/apb4_i2c_top.ddc
 

