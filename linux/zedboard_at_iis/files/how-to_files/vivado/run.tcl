# This is a simple example script to generate a basic Zynq design in
# Xilinx Vivado 2015.1
# 
# To create the design, open Vivado 2015.1, and source the script by
# executing source run.tcl

### create the project
create_project zedboard_example . -part xc7z020clg484-1
set_property board_part em.avnet.com:zed:part0:1.1 [current_project]

### create the block design
create_bd_design zedboard_example_bd

### add IPs
create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7:5.5 processing_system7_0

apply_bd_automation -rule xilinx.com:bd_rule:processing_system7 -config {make_external "FIXED_IO, DDR" apply_board_preset "1" }  [get_bd_cells processing_system7_0]

### disable M_AXI_GP0 port - remove this if you connect own hardware
set_property -dict [list CONFIG.PCW_USE_M_AXI_GP0 {0}] [get_bd_cells processing_system7_0]

### add more IPs in block design flow here if required

### finish block design
regenerate_bd_layout
save_bd_design
validate_bd_design

### generate a wrapper file for the block design and add this to the design sources
make_wrapper -files [get_files ./zedboard_example.srcs/sources_1/bd/zedboard_example_bd/zedboard_example_bd.bd] -top
add_files -norecurse -force ./zedboard_example.srcs/sources_1/bd/zedboard_example_bd/hdl/zedboard_example_bd_wrapper.v
update_compile_order -fileset sources_1
update_compile_order -fileset sim_1

reset_target all [get_files ./zedboard_example.srcs/sources_1/bd/zedboard_example_bd/zedboard_example_bd.bd]

generate_target all [get_files ./zedboard_example.srcs/sources_1/bd/zedboard_example_bd/zedboard_example_bd.bd]

set_property STEPS.SYNTH_DESIGN.ARGS.FLATTEN_HIERARCHY full [get_runs synth_1]

### hdl code generation, synthesis, implementation and bit-stream generation. 
launch_runs impl_1 -to_step write_bitstream
