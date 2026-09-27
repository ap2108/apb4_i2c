set search_path [list . /fetools/work_area/frontend/SAED32_EDK/lib/stdcell_lvt/db_nldm]
 
set target_library [list saed32lvt_ss0p75v25c.db]
set link_library [list * $target_library]
 
set designer "adithya"
 
read_verilog [list apb4_i2c_top_netlist.v]
current_design apb4_i2c_top

link_design

source sta_con.sdc
update_timing
