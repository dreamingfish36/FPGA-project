`timescale 1ns / 1ps

// ============================================================
// camera_gui_source_select_core_640.v
// S05-640-CORE
//
// 放置位置：OV5640/RAM 读出 640x480 摄像头视频流之后，HDMI TMDS 编码之前。
// 功能：
// - source_select=0：输出 GUI UART image 路径，128x72 放大 5 倍到 640x360，并居中显示；
// - source_select=1：输出 FPGA OV5640 camera 路径，640x480 实时处理；
// - 两路统一进入 video_stream_alg_640 做实时算法处理；
// - mode / threshold / overlay / source_select 都来自 PS 端 BRAM 控制字。
// ============================================================

module camera_gui_source_select_core_640 #(
    parameter H_ACTIVE = 640,
    parameter V_ACTIVE = 480
)(
    input  wire        video_clk,
    input  wire        rst,

    // 来自 OV5640/RAM 读出的 640x480 摄像头视频流
    input  wire        cam_hs,
    input  wire        cam_vs,
    input  wire        cam_de,
    input  wire [7:0]  cam_r,
    input  wire [7:0]  cam_g,
    input  wire [7:0]  cam_b,

    // HDMI 当前有效区计数，用于把 GUI 128x72 图像放大成 640x360
    input  wire [11:0] h_cnt,
    input  wire [11:0] v_cnt,

    // PS/BRAM 控制读口，连接 ps_uart_bram_hdmi_wrapper 的 BRAM_PORTB
    output wire        bram_clk,
    output wire        bram_rst,
    output wire        bram_en,
    output wire [3:0]  bram_we,
    output wire [31:0] bram_addr,
    output wire [31:0] bram_din,
    input  wire [31:0] bram_dout,

    // 处理后送 HDMI 的 640x480 视频流
    output wire        out_hs,
    output wire        out_vs,
    output wire        out_de,
    output wire [7:0]  out_r,
    output wire [7:0]  out_g,
    output wire [7:0]  out_b,

    // 调试观察
    output wire [3:0]  dbg_display_mode,
    output wire [7:0]  dbg_threshold,
    output wire        dbg_overlay_enable,
    output wire        dbg_source_select
);

    wire [3:0] display_mode;
    wire [7:0] threshold;
    wire       overlay_enable;
    wire       source_select;

    wire [7:0] gui_r;
    wire [7:0] gui_g;
    wire [7:0] gui_b;
    wire       gui_de;

    bram_control_gui_reader_640 #(
        .H_ACTIVE(H_ACTIVE),
        .V_ACTIVE(V_ACTIVE),
        .GUI_W(128),
        .GUI_H(72),
        .SCALE_X(5),
        .SCALE_Y(5),
        .GUI_X0(0),
        .GUI_Y0(60)
    ) u_bram_control_gui_reader_640 (
        .clk            (video_clk),
        .rst            (rst),
        .h_cnt          (h_cnt),
        .v_cnt          (v_cnt),
        .video_de       (cam_de),
        .bram_clk       (bram_clk),
        .bram_rst       (bram_rst),
        .bram_en        (bram_en),
        .bram_we        (bram_we),
        .bram_addr      (bram_addr),
        .bram_din       (bram_din),
        .bram_dout      (bram_dout),
        .display_mode   (display_mode),
        .threshold      (threshold),
        .overlay_enable (overlay_enable),
        .source_select  (source_select),
        .gui_r          (gui_r),
        .gui_g          (gui_g),
        .gui_b          (gui_b),
        .gui_de         (gui_de)
    );

    // ========================================================
    // S05-640-CORE-A：图像来源选择 MUX
    // 选择放在算法前：后面的 16 个算法不用分别判断图像来源。
    // ========================================================
    wire sel_hs = cam_hs;
    wire sel_vs = cam_vs;
    wire sel_de = source_select ? cam_de : gui_de;
    wire [7:0] sel_r = source_select ? cam_r : gui_r;
    wire [7:0] sel_g = source_select ? cam_g : gui_g;
    wire [7:0] sel_b = source_select ? cam_b : gui_b;

    video_stream_alg_640 #(
        .H_ACTIVE(H_ACTIVE),
        .V_ACTIVE(V_ACTIVE)
    ) u_video_stream_alg_640 (
        .clk            (video_clk),
        .rst            (rst),
        .in_hs          (sel_hs),
        .in_vs          (sel_vs),
        .in_de          (sel_de),
        .in_r           (sel_r),
        .in_g           (sel_g),
        .in_b           (sel_b),
        .display_mode   (display_mode),
        .threshold      (threshold),
        .overlay_enable (overlay_enable),
        .out_hs         (out_hs),
        .out_vs         (out_vs),
        .out_de         (out_de),
        .out_r          (out_r),
        .out_g          (out_g),
        .out_b          (out_b)
    );

    assign dbg_display_mode   = display_mode;
    assign dbg_threshold      = threshold;
    assign dbg_overlay_enable = overlay_enable;
    assign dbg_source_select  = source_select;

endmodule
