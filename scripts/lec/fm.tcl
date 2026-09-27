# reference design (rtl)
read_verilog -container r -libname WORK -05 [glob ../src/*.v]
read_db {/fetools/work_area/frontend/SAED32_EDK/lib/stdcell_lvt/db_nldm/saed32lvt_ss0p75v25c.db}

set_top r:/WORK/apb4_i2c_top

# synthesized design (netlist)
read_verilog -container i -libname WORK -05 [glob netlist/*.v]
read_db {/fetools/work_area/frontend/SAED32_EDK/lib/stdcell_lvt/db_nldm/saed32lvt_ss0p75v25c.db}

set_top i:/WORK/apb4_i2c_top

match
verify
exit
