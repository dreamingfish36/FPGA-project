-makelib ies_lib/xil_defaultlib -sv \
  "C:/Xilinx/Vivado/2017.4/data/ip/xpm/xpm_cdc/hdl/xpm_cdc.sv" \
-endlib
-makelib ies_lib/xpm \
  "C:/Xilinx/Vivado/2017.4/data/ip/xpm/xpm_VCOMP.vhd" \
-endlib
-makelib ies_lib/xil_defaultlib \
  "../../../../hdmi_out_test.srcs/sources_1/ip/video_clock/video_clock_clk_wiz.v" \
  "../../../../hdmi_out_test.srcs/sources_1/ip/video_clock/video_clock.v" \
-endlib
-makelib ies_lib/xil_defaultlib \
  glbl.v
-endlib

