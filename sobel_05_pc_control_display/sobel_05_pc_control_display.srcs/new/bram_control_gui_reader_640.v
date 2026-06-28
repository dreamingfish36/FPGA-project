`timescale 1ns / 1ps

// ============================================================
// bram_control_gui_reader_640.v
// S05-640-BRAM
//
// 作用：
// 1) 从 PS/BRAM 控制区读取 display_mode / threshold / overlay / source_select；
// 2) 在 source_select=0 时，把 GUI 发来的 128x72 framebuffer 放大 5 倍，
//    显示成 640x360，并在 640x480 HDMI 画面中垂直居中；
// 3) 在 source_select=1 时，只读取控制字，不读取 GUI 图像。
//
// BRAM 地址约定：
// 0x0000~0x8fff  : GUI 128x72 RGB888 framebuffer，每个像素 32-bit
// 0x9000         : display_mode
// 0x9004         : threshold
// 0x9008         : overlay_enable
// 0x900c         : source_select，0=GUI UART image，1=FPGA camera
// ============================================================

module bram_control_gui_reader_640 #(
    parameter H_ACTIVE = 640,
    parameter V_ACTIVE = 480,
    parameter GUI_W    = 128,
    parameter GUI_H    = 72,
    parameter SCALE_X  = 5,
    parameter SCALE_Y  = 5,
    parameter GUI_X0   = 0,
    parameter GUI_Y0   = 60
)(
    input  wire        clk,
    input  wire        rst,

    input  wire [11:0] h_cnt,
    input  wire [11:0] v_cnt,
    input  wire        video_de,

    // 连接 PS 端 AXI BRAM Controller 的 PL 读口
    output wire        bram_clk,
    output wire        bram_rst,
    output reg         bram_en,
    output wire [3:0]  bram_we,
    output reg  [31:0] bram_addr,
    output wire [31:0] bram_din,
    input  wire [31:0] bram_dout,

    output reg  [3:0]  display_mode,
    output reg  [7:0]  threshold,
    output reg         overlay_enable,
    output reg         source_select,

    output reg  [7:0]  gui_r,
    output reg  [7:0]  gui_g,
    output reg  [7:0]  gui_b,
    output reg         gui_de
);

    localparam CTRL_MODE_ADDR      = 32'h0000_9000;
    localparam CTRL_THRESHOLD_ADDR = 32'h0000_9004;
    localparam CTRL_OVERLAY_ADDR   = 32'h0000_9008;
    localparam CTRL_SOURCE_ADDR    = 32'h0000_900c;

    assign bram_clk = clk;
    assign bram_rst = rst;
    assign bram_we  = 4'b0000;
    assign bram_din = 32'd0;

    reg [2:0] ctrl_state;
    reg       read_gui_d;

    // GUI 128x72 是 16:9；640x480 是 4:3。
    // 这里不强行拉伸，而是 5 倍放大成 640x360，上下各留 60 行黑边。
    wire gui_window = video_de &&
                      (h_cnt >= GUI_X0) && (h_cnt < GUI_X0 + GUI_W * SCALE_X) &&
                      (v_cnt >= GUI_Y0) && (v_cnt < GUI_Y0 + GUI_H * SCALE_Y);

    wire [11:0] rel_h = h_cnt - GUI_X0;
    wire [11:0] rel_v = v_cnt - GUI_Y0;
    wire [6:0]  gui_x = rel_h / SCALE_X;
    wire [6:0]  gui_y = rel_v / SCALE_Y;
    wire [13:0] gui_word_addr = gui_y * GUI_W + gui_x;
    wire [31:0] gui_byte_addr = {16'd0, gui_word_addr, 2'b00};

    always @(posedge clk) begin
        if (rst) begin
            bram_en        <= 1'b1;
            bram_addr      <= CTRL_MODE_ADDR;
            ctrl_state     <= 3'd0;
            display_mode   <= 4'd10; // 默认 soble
            threshold      <= 8'd80;
            overlay_enable <= 1'b0;
            source_select  <= 1'b0;  // 默认保留旧功能：GUI UART image
            gui_r          <= 8'd0;
            gui_g          <= 8'd0;
            gui_b          <= 8'd0;
            gui_de         <= 1'b0;
            read_gui_d     <= 1'b0;
        end else begin
            bram_en <= 1'b1;

            // GUI 图像路径只在 source_select=0 且处于 GUI 显示区域时占用 BRAM 读口。
            if ((source_select == 1'b0) && gui_window) begin
                bram_addr  <= gui_byte_addr;
                read_gui_d <= 1'b1;
            end else begin
                read_gui_d <= 1'b0;
                // 非 GUI 图像读窗口时轮询控制字。
                case (ctrl_state)
                    3'd0: begin bram_addr <= CTRL_MODE_ADDR;      ctrl_state <= 3'd1; end
                    3'd1: begin display_mode <= bram_dout[3:0];   bram_addr <= CTRL_THRESHOLD_ADDR; ctrl_state <= 3'd2; end
                    3'd2: begin threshold <= bram_dout[7:0];      bram_addr <= CTRL_OVERLAY_ADDR;   ctrl_state <= 3'd3; end
                    3'd3: begin overlay_enable <= bram_dout[0];   bram_addr <= CTRL_SOURCE_ADDR;    ctrl_state <= 3'd4; end
                    3'd4: begin source_select <= bram_dout[0];    bram_addr <= CTRL_MODE_ADDR;      ctrl_state <= 3'd1; end
                    default: ctrl_state <= 3'd0;
                endcase
            end

            // BRAM 读数据延迟一拍使用。
            gui_de <= read_gui_d;
            if (read_gui_d) begin
                gui_r <= bram_dout[23:16];
                gui_g <= bram_dout[15:8];
                gui_b <= bram_dout[7:0];
            end else begin
                gui_r <= 8'd0;
                gui_g <= 8'd0;
                gui_b <= 8'd0;
            end
        end
    end

endmodule
