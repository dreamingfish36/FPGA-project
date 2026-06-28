// Copyright 1986-2017 Xilinx, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2017.4 (win64) Build 2086221 Fri Dec 15 20:55:39 MST 2017
// Date        : Thu Jun  4 12:24:20 2026
// Host        : LAPTOP-OBFF2D66 running 64-bit major release  (build 9200)
// Command     : write_verilog -force -mode synth_stub
//               d:/Github/FPGA-course/sobel/sobel_03_uart_hdmi/sobel_03_uart_hdmi.srcs/sources_1/bd/ps_uart_bram_hdmi/ip/ps_uart_bram_hdmi_blk_mem_gen_0_0/ps_uart_bram_hdmi_blk_mem_gen_0_0_stub.v
// Design      : ps_uart_bram_hdmi_blk_mem_gen_0_0
// Purpose     : Stub declaration of top-level module interface
// Device      : xc7z020clg400-2
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
(* x_core_info = "blk_mem_gen_v8_4_1,Vivado 2017.4" *)
module ps_uart_bram_hdmi_blk_mem_gen_0_0(clka, ena, wea, addra, dina, douta, clkb, rstb, enb, web, 
  addrb, dinb, doutb)
/* synthesis syn_black_box black_box_pad_pin="clka,ena,wea[3:0],addra[31:0],dina[31:0],douta[31:0],clkb,rstb,enb,web[3:0],addrb[31:0],dinb[31:0],doutb[31:0]" */;
  input clka;
  input ena;
  input [3:0]wea;
  input [31:0]addra;
  input [31:0]dina;
  output [31:0]douta;
  input clkb;
  input rstb;
  input enb;
  input [3:0]web;
  input [31:0]addrb;
  input [31:0]dinb;
  output [31:0]doutb;
endmodule
