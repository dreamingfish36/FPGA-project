module hdmi_bram_sobel_display(
    input clk,
    input rst,
    output hs,
    output vs,
    output de,
    output [7:0] rgb_r,
    output [7:0] rgb_g,
    output [7:0] rgb_b,
    output reg bram_en,
    output [3:0] bram_we,
    output [31:0] bram_addr,
    output [31:0] bram_din,
    input [31:0] bram_dout
);

parameter H_ACTIVE = 16'd1280;
parameter H_FP     = 16'd110;
parameter H_SYNC   = 16'd40;
parameter H_BP     = 16'd220;
parameter V_ACTIVE = 16'd720;
parameter V_FP     = 16'd5;
parameter V_SYNC   = 16'd5;
parameter V_BP     = 16'd20;

localparam H_TOTAL = H_ACTIVE + H_FP + H_SYNC + H_BP;
localparam V_TOTAL = V_ACTIVE + V_FP + V_SYNC + V_BP;
localparam H_START = H_FP + H_SYNC + H_BP;
localparam V_START = V_FP + V_SYNC + V_BP;
localparam IMG_WIDTH  = 128;
localparam IMG_HEIGHT = 72;
localparam SCALE_X = H_ACTIVE / IMG_WIDTH;
localparam SCALE_Y = V_ACTIVE / IMG_HEIGHT;

localparam CTRL_MODE_ADDR      = 32'h0000_9000;
localparam CTRL_THRESHOLD_ADDR = 32'h0000_9004;
localparam CTRL_OVERLAY_ADDR   = 32'h0000_9008;

// -----------------------------------------------------------------------------
// Display mode definition
// -----------------------------------------------------------------------------
// 修改点 A：display_mode 现在扩展为 4 bit，可支持 0~15 共 16 个模式。
// 注意：PS 端 main.c 写入 CTRL_MODE_ADDR 的 value 也必须允许 0~15。
// 注意：GUI / camera_uart_sender.py 的 mode 映射也要同步增加对应选项。
localparam [3:0] MODE_ORIGINAL      = 4'd0;   // 原图
localparam [3:0] MODE_HUE_ADJUST    = 4'd1;   // 色度调节 chrominance
localparam [3:0] MODE_GAMMA         = 4'd2;   // gamma 矫正
localparam [3:0] MODE_BRIGHTNESS    = 4'd3;   // 亮度调节 brightness
localparam [3:0] MODE_GRAY          = 4'd4;   // 灰度处理 RGB2YCbCr
localparam [3:0] MODE_SHARPEN       = 4'd5;   // 锐化处理 sobel_sharpen_proc
localparam [3:0] MODE_AVER_FILTER   = 4'd6;   // aver 均值滤波 aver_filter
localparam [3:0] MODE_CONTRAST      = 4'd7;   // 对比度增强 Curve_Contrast_Array
localparam [3:0] MODE_GAUSS_FILTER  = 4'd8;   // guass/gauss 高斯滤波 gauss_filter
localparam [3:0] MODE_MID_FILTER    = 4'd9;   // midfilter 中值滤波 median_filter_3x3
localparam [3:0] MODE_SOBEL         = 4'd10;  // soble/Sobel 边缘检测
localparam [3:0] MODE_RED_OVERLAY   = 4'd11;  // 红色边缘叠加
localparam [3:0] MODE_BIN           = 4'd12;  // bin 二值化 binarization
localparam [3:0] MODE_AREA_BIN      = 4'd13;  // area_bin 区域二值化
localparam [3:0] MODE_EROSION       = 4'd14;  // erosion 腐蚀
localparam [3:0] MODE_DILATE        = 4'd15;  // dilate 膨胀

localparam SCAN_IDLE              = 4'd0;
localparam SCAN_CTRL_MODE_REQ     = 4'd1;
localparam SCAN_CTRL_MODE_WAIT1   = 4'd2;
localparam SCAN_CTRL_MODE_WAIT2   = 4'd3;
localparam SCAN_CTRL_THR_REQ      = 4'd4;
localparam SCAN_CTRL_THR_WAIT1    = 4'd5;
localparam SCAN_CTRL_THR_WAIT2    = 4'd6;
localparam SCAN_CTRL_OVL_REQ      = 4'd7;
localparam SCAN_CTRL_OVL_WAIT1    = 4'd8;
localparam SCAN_CTRL_OVL_WAIT2    = 4'd9;
localparam SCAN_RUN               = 4'd10;
localparam SCAN_WAIT              = 4'd11;

reg [11:0] h_cnt;
reg [11:0] v_cnt;
reg hs_reg;
reg vs_reg;
reg de_reg;
reg hs_reg_d0;
reg vs_reg_d0;
reg de_reg_d0;
reg [13:0] display_rd_addr;
reg [7:0] edge_pixel;
reg [23:0] rgb_pixel;

// S05-ALG-PIXEL：显示阶段从各算法结果缓存中读出的当前像素
reg [23:0] hue_pixel;
reg [23:0] gamma_pixel;
reg [23:0] brightness_pixel;
reg [7:0] gray_pixel;
reg [7:0] sharpen_pixel;
reg [7:0] aver_pixel;
reg [7:0] contrast_pixel;
reg [7:0] gauss_pixel;
reg [7:0] median_pixel;
reg [7:0] alg_sobel_pixel;
reg bin_pixel;
reg area_bin_pixel;
reg erosion_pixel;
reg dilate_pixel;

reg [3:0] scan_state;
reg [6:0] scan_x;
reg [6:0] scan_y;
reg [31:0] bram_addr_reg;
reg scan_valid_d1;
reg scan_valid_d2;
reg [6:0] scan_x_d1;
reg [6:0] scan_y_d1;
reg [6:0] scan_x_d2;
reg [6:0] scan_y_d2;
reg scan_frame_start;
reg sobel_done;

// 修改点 B：display_mode 从 2 bit 改成 4 bit，才能表示 16 个模式。
reg [3:0] display_mode;
reg [7:0] threshold;
reg overlay_enable;

(* ram_style = "block" *) reg [23:0] rgb_mem [0:9215];
(* ram_style = "block" *) reg [7:0] edge_mem [0:9215];

// -----------------------------------------------------------------------------
// S05-ALG-MEM：外部工程算法结果缓存
// -----------------------------------------------------------------------------
// 说明：另一个工程的算法模块大多是"视频流输入 -> 视频流输出"。
// 当前工程是"PS 写入 BRAM -> PL 扫描 BRAM -> HDMI 显示"，所以这里用
// 结果缓存 RAM 把各算法模块的流式输出保存下来，显示阶段再按 display_mode 读取。
// 注意：这些缓存只改变显示通路，不修改任何外部算法模块的端口。
(* ram_style = "block" *) reg [23:0] hue_mem        [0:9215];
(* ram_style = "block" *) reg [23:0] gamma_mem      [0:9215];
(* ram_style = "block" *) reg [23:0] brightness_mem [0:9215];
(* ram_style = "block" *) reg [7:0]  gray_mem       [0:9215];
(* ram_style = "block" *) reg [7:0]  sharpen_mem    [0:9215];
(* ram_style = "block" *) reg [7:0]  aver_mem       [0:9215];
(* ram_style = "block" *) reg [7:0]  contrast_mem   [0:9215];
(* ram_style = "block" *) reg [7:0]  gauss_mem      [0:9215];
(* ram_style = "block" *) reg [7:0]  median_mem     [0:9215];
(* ram_style = "block" *) reg [7:0]  alg_sobel_mem  [0:9215];
(* ram_style = "block" *) reg        bin_mem        [0:9215];
(* ram_style = "block" *) reg        area_bin_mem   [0:9215];
(* ram_style = "block" *) reg        erosion_mem    [0:9215];
(* ram_style = "block" *) reg        dilate_mem     [0:9215];

wire h_active;
wire v_active;
wire video_active;
wire hsync_now;
wire vsync_now;
wire video_frame_start;
wire [11:0] active_x;
wire [11:0] active_y;
wire [6:0] disp_x;
wire [6:0] disp_y;
wire [13:0] disp_addr;
wire [13:0] scan_word_addr;
wire [13:0] scan_store_addr;
wire scan_issue;
wire scan_last;
wire ctrl_read_active;

wire gray_valid;
wire [7:0] gray;
wire [15:0] gray_x;
wire [15:0] gray_y;
wire edge_valid;
wire [7:0] edge_data;
wire [15:0] edge_x;
wire [15:0] edge_y;
wire edge_frame_done;
wire [13:0] edge_wr_addr;

// -----------------------------------------------------------------------------
// S05-ALG-WIRE：从另一个顶层工程提取的算法模块连接线
// -----------------------------------------------------------------------------
// 输入流：当前模块从 BRAM 扫描出的 128x72 RGB888 像素流。
// 不匹配提醒：原工程算法大多按 1920x1080 视频流设计；这里改为 128x72 参数/输入流。
// 如果对应算法模块内部写死 1920/1080，仍需进入该算法模块内部检查，但这里不改端口。
wire alg_in_de;
wire alg_in_vs;
wire [7:0] alg_in_r;
wire [7:0] alg_in_g;
wire [7:0] alg_in_b;
assign alg_in_de = scan_valid_d2;
assign alg_in_vs = scan_frame_start;
assign alg_in_r  = bram_dout[23:16];
assign alg_in_g  = bram_dout[15:8];
assign alg_in_b  = bram_dout[7:0];

// 色度调节 chrominance
wire chrominance_vsync;
wire chrominance_de;
wire chrominance_href;
wire [7:0] chrominance_gray;

// 亮度调节 brightness
wire brightness_vsync;
wire brightness_de;
wire brightness_href;
wire [7:0] brightness_out_r;
wire [7:0] brightness_out_g;
wire [7:0] brightness_out_b;

// gamma 矫正
wire [7:0] gamma_data_r;
wire [7:0] gamma_data_g;
wire [7:0] gamma_data_b;
wire gamma_de;

// 灰度 RGB2YCbCr
wire y_vs;
wire y_de;
wire [7:0] y_data;
wire [7:0] cb_data;
wire [7:0] cr_data;

// 3x3 矩阵窗口，共用给均值/高斯/中值/Sobel/区域二值化
wire matrix_de;
wire [7:0] matrix11;
wire [7:0] matrix12;
wire [7:0] matrix13;
wire [7:0] matrix21;
wire [7:0] matrix22;
wire [7:0] matrix23;
wire [7:0] matrix31;
wire [7:0] matrix32;
wire [7:0] matrix33;

// 锐化
wire sharpen_vsync;
wire sharpen_de;
wire [7:0] sharpen_gray;

// 均值滤波
wire aver_filter_vs;
wire aver_filter_de;
wire [7:0] aver_filter_data;

// 对比度增强
wire [7:0] curve_gray;

// 高斯滤波
wire gauss_filter_vs;
wire gauss_filter_de;
wire [7:0] gauss_filter_data;

// 中值滤波
wire median_vs;
wire median_hs;
wire median_de;
wire [7:0] median_data;

// Sobel
wire alg_sobel_vs;
wire alg_sobel_de;
wire [7:0] alg_sobel_data;

// 二值化 / 区域二值化
wire bin_vs;
wire bin_hs;
wire bin_de;
wire bin_data;
wire area_bin_vs;
wire area_bin_de;
wire area_bin_data;

// 1-bit 矩阵窗口，用于腐蚀、膨胀
wire matrix_de_1bit;
wire matrix11_1bit;
wire matrix12_1bit;
wire matrix13_1bit;
wire matrix21_1bit;
wire matrix22_1bit;
wire matrix23_1bit;
wire matrix31_1bit;
wire matrix32_1bit;
wire matrix33_1bit;
wire matrix_de_erosion;
wire matrix11_erosion;
wire matrix12_erosion;
wire matrix13_erosion;
wire matrix21_erosion;
wire matrix22_erosion;
wire matrix23_erosion;
wire matrix31_erosion;
wire matrix32_erosion;
wire matrix33_erosion;

// 腐蚀 / 膨胀
wire erosion_vs;
wire erosion_de;
wire erosion_data;
wire dilate_vs;
wire dilate_de;
wire dilate_data;

// 各算法输出缓存写地址。按各模块输出 de 有效顺序递增。
reg [13:0] hue_wr_addr;
reg [13:0] gamma_wr_addr;
reg [13:0] brightness_wr_addr;
reg [13:0] gray_wr_addr;
reg [13:0] sharpen_wr_addr;
reg [13:0] aver_wr_addr;
reg [13:0] contrast_wr_addr;
reg [13:0] gauss_wr_addr;
reg [13:0] median_wr_addr;
reg [13:0] alg_sobel_wr_addr;
reg [13:0] bin_wr_addr;
reg [13:0] area_bin_wr_addr;
reg [13:0] erosion_wr_addr;
reg [13:0] dilate_wr_addr;

wire [15:0] display_gray_sum;
wire [7:0] display_gray;
wire edge_on;
wire overlay_active;
wire [7:0] gray_binary_pixel;
wire [7:0] rgb_binary_pixel;
reg [7:0] out_r;
reg [7:0] out_g;
reg [7:0] out_b;

assign h_active = (h_cnt >= H_START[11:0]) && (h_cnt < (H_START + H_ACTIVE));
assign v_active = (v_cnt >= V_START[11:0]) && (v_cnt < (V_START + V_ACTIVE));
assign video_active = h_active && v_active;

assign hsync_now = (h_cnt >= H_FP[11:0]) && (h_cnt < (H_FP + H_SYNC));
assign vsync_now = (v_cnt >= V_FP[11:0]) && (v_cnt < (V_FP + V_SYNC));
assign video_frame_start = (h_cnt == 12'd0) && (v_cnt == 12'd0);

assign active_x = h_cnt - H_START[11:0];
assign active_y = v_cnt - V_START[11:0];
assign disp_x = active_x / SCALE_X;
assign disp_y = active_y / SCALE_Y;
assign disp_addr = {disp_y, 7'b0} + {7'd0, disp_x};

assign scan_word_addr = {scan_y, 7'b0} + {7'd0, scan_x};
assign scan_store_addr = {scan_y_d2, 7'b0} + {7'd0, scan_x_d2};
assign scan_issue = (scan_state == SCAN_RUN);
assign scan_last = (scan_x == 7'd127) && (scan_y == 7'd71);
assign ctrl_read_active = (scan_state >= SCAN_CTRL_MODE_REQ) && (scan_state <= SCAN_CTRL_OVL_WAIT2);
assign edge_wr_addr = {edge_y[6:0], 7'b0} + {7'd0, edge_x[6:0]};

assign hs = hs_reg_d0;
assign vs = vs_reg_d0;
assign de = de_reg_d0;
assign rgb_r = (de_reg_d0 && sobel_done) ? out_r : 8'h00;
assign rgb_g = (de_reg_d0 && sobel_done) ? out_g : 8'h00;
assign rgb_b = (de_reg_d0 && sobel_done) ? out_b : 8'h00;

assign bram_we = 4'b0000;
assign bram_din = 32'd0;
assign bram_addr = bram_addr_reg;

assign display_gray_sum = ({8'd0, rgb_pixel[23:16]} * 8'd77) +
                          ({8'd0, rgb_pixel[15:8]}  * 8'd150) +
                          ({8'd0, rgb_pixel[7:0]}   * 8'd29);
assign display_gray = display_gray_sum[15:8];
assign edge_on = (alg_sobel_pixel >= threshold);
assign gray_binary_pixel = (display_gray >= threshold) ? 8'hff : 8'h00;
assign rgb_binary_pixel  = (display_gray >= threshold) ? 8'hff : 8'h00;
assign overlay_active = overlay_enable || (display_mode == MODE_RED_OVERLAY);

// -----------------------------------------------------------------------------
// Display mode selection
// -----------------------------------------------------------------------------
// 修改点 C：16 个显示模式的核心选择逻辑在这里。
// 如果后面还要继续扩展显示效果，优先在下面 case(display_mode) 里增加/修改。
//
// 数据来源说明：
//   rgb_pixel      ：从 BRAM 图像区读取并缓存的原始 RGB 像素。
//   display_gray   ：由当前 rgb_pixel 计算出的灰度值，用于显示灰度/二值图。
//   edge_pixel     ：sobel_core 输出后写入 edge_mem 的边缘强度。
//   edge_on        ：edge_pixel >= threshold 的二值化边缘结果。
//   threshold      ：PS 端写入 CTRL_THRESHOLD_ADDR，PL 每帧读取。
//   overlay_enable ：PS 端写入 CTRL_OVERLAY_ADDR，PL 每帧读取。
//
// 自定义扩展建议：
//   - 想增加新模式：修改 MODE_USER_CUSTOM 或替换某个 MODE_* 分支。
//   - 想接入新模块：在模块实例化区域增加模块，然后在 case 中使用新模块输出。
always @(*) begin
    // 默认输出原图，避免未覆盖分支产生不确定输出。
    out_r = rgb_pixel[23:16];
    out_g = rgb_pixel[15:8];
    out_b = rgb_pixel[7:0];

    case (display_mode)
        MODE_ORIGINAL: begin
            out_r = rgb_pixel[23:16];
            out_g = rgb_pixel[15:8];
            out_b = rgb_pixel[7:0];
        end

        MODE_HUE_ADJUST: begin
            // 色度调节模块 chrominance 的输出是 8-bit 灰度量，因此这里复制到 RGB 三通道显示。
            out_r = hue_pixel[23:16];
            out_g = hue_pixel[15:8];
            out_b = hue_pixel[7:0];
        end

        MODE_GAMMA: begin
            out_r = gamma_pixel[23:16];
            out_g = gamma_pixel[15:8];
            out_b = gamma_pixel[7:0];
        end

        MODE_BRIGHTNESS: begin
            out_r = brightness_pixel[23:16];
            out_g = brightness_pixel[15:8];
            out_b = brightness_pixel[7:0];
        end

        MODE_GRAY: begin
            out_r = gray_pixel;
            out_g = gray_pixel;
            out_b = gray_pixel;
        end

        MODE_SHARPEN: begin
            out_r = sharpen_pixel;
            out_g = sharpen_pixel;
            out_b = sharpen_pixel;
        end

        MODE_AVER_FILTER: begin
            out_r = aver_pixel;
            out_g = aver_pixel;
            out_b = aver_pixel;
        end

        MODE_CONTRAST: begin
            out_r = contrast_pixel;
            out_g = contrast_pixel;
            out_b = contrast_pixel;
        end

        MODE_GAUSS_FILTER: begin
            out_r = gauss_pixel;
            out_g = gauss_pixel;
            out_b = gauss_pixel;
        end

        MODE_MID_FILTER: begin
            out_r = median_pixel;
            out_g = median_pixel;
            out_b = median_pixel;
        end

        MODE_SOBEL: begin
            out_r = edge_on ? 8'hff : 8'h00;
            out_g = edge_on ? 8'hff : 8'h00;
            out_b = edge_on ? 8'hff : 8'h00;
        end

        MODE_RED_OVERLAY: begin
            if (edge_on) begin
                out_r = 8'hff;
                out_g = 8'h20;
                out_b = 8'h20;
            end else begin
                out_r = rgb_pixel[23:16];
                out_g = rgb_pixel[15:8];
                out_b = rgb_pixel[7:0];
            end
        end

        MODE_BIN: begin
            out_r = bin_pixel ? 8'hff : 8'h00;
            out_g = bin_pixel ? 8'hff : 8'h00;
            out_b = bin_pixel ? 8'hff : 8'h00;
        end

        MODE_AREA_BIN: begin
            out_r = area_bin_pixel ? 8'hff : 8'h00;
            out_g = area_bin_pixel ? 8'hff : 8'h00;
            out_b = area_bin_pixel ? 8'hff : 8'h00;
        end

        MODE_EROSION: begin
            out_r = erosion_pixel ? 8'hff : 8'h00;
            out_g = erosion_pixel ? 8'hff : 8'h00;
            out_b = erosion_pixel ? 8'hff : 8'h00;
        end

        MODE_DILATE: begin
            out_r = dilate_pixel ? 8'hff : 8'h00;
            out_g = dilate_pixel ? 8'hff : 8'h00;
            out_b = dilate_pixel ? 8'hff : 8'h00;
        end

        default: begin
            out_r = rgb_pixel[23:16];
            out_g = rgb_pixel[15:8];
            out_b = rgb_pixel[7:0];
        end
    endcase

    // overlay_enable 是独立开关：除纯 Sobel 和红色叠加模式外，其他模式也可强制叠加红边。
    if ((display_mode != MODE_SOBEL) &&
        (display_mode != MODE_RED_OVERLAY) &&
        overlay_enable && edge_on) begin
        out_r = 8'hff;
        out_g = 8'h20;
        out_b = 8'h20;
    end
end

// -----------------------------------------------------------------------------
// Module connection area
// -----------------------------------------------------------------------------
// 连接说明 1：保留原工程已有 rgb_to_gray + sobel_core，作为扫描完成握手/基础边缘检测通路。
// 这些端口不修改；额外算法模块放在后面 S05-ALG-CONNECT 区域。
rgb_to_gray u_rgb_to_gray (
    .clk(clk),
    .rst_n(~rst),
    .rgb_valid(scan_valid_d2),
    .r(bram_dout[23:16]),
    .g(bram_dout[15:8]),
    .b(bram_dout[7:0]),
    .x({9'd0, scan_x_d2}),
    .y({9'd0, scan_y_d2}),
    .gray_valid(gray_valid),
    .gray(gray),
    .gray_x(gray_x),
    .gray_y(gray_y)
);

sobel_core #(
    .WIDTH(IMG_WIDTH),
    .HEIGHT(IMG_HEIGHT)
) u_sobel_core (
    .clk(clk),
    .rst_n(~rst),
    .frame_start(scan_frame_start),
    .gray_valid(gray_valid),
    .gray(gray),
    .gray_x(gray_x),
    .gray_y(gray_y),
    .edge_valid(edge_valid),
    .edge_data(edge_data),
    .edge_x(edge_x),
    .edge_y(edge_y),
    .edge_frame_done(edge_frame_done)
);

// -----------------------------------------------------------------------------
// S05-ALG-CONNECT：从另一个工程顶层提取的算法模块实例化
// -----------------------------------------------------------------------------
// 输入统一来自 BRAM 扫描流：alg_in_r/g/b + alg_in_de + alg_in_vs。
// 重要：没有修改任何算法模块端口；这里只做端口连接。
// 如果 Vivado 报 "module not found"，需要把对应算法模块 .v 文件加入 Design Sources。
// 如果 Vivado 报 "port not found"，说明你当前算法模块版本和另一个工程顶层不一致，按报错端口名核对。

chrominance #(
    .R(3)
) u_alg_chrominance (
    .clk              (clk),
    .rst_n            (~rst),
    .per_frame_vsync  (alg_in_vs),
    .per_frame_de     (alg_in_de),
    .per_frame_hs     (alg_in_de),
    .per_frame_data   ({alg_in_r, alg_in_g, alg_in_b}),
    .post_frame_vsync (chrominance_vsync),
    .post_frame_de    (chrominance_de),
    .post_frame_hs    (chrominance_href),
    .post_frame_data  (chrominance_gray)
);

brightness u_alg_brightness (
    .clk     (clk),
    .rst_n   (~rst),
    .data_in ({alg_in_r, alg_in_g, alg_in_b}),
    .im_gain (8'd100),
    .in_de   (alg_in_de),
    .pre_vs  (alg_in_vs),
    .pre_hs  (alg_in_de),
    .pos_de  (brightness_de),
    .pos_vs  (brightness_vsync),
    .pos_hs  (brightness_href),
    .out_r   (brightness_out_r),
    .out_g   (brightness_out_g),
    .out_b   (brightness_out_b)
);

gamma_lookuptable u_alg_gamma_r (
    .video_clk  (clk),
    .video_data (alg_in_r),
    .video_de   (alg_in_de),
    .gamma_de   (gamma_de),
    .gamma_data (gamma_data_r)
);

gamma_lookuptable u_alg_gamma_g (
    .video_clk  (clk),
    .video_data (alg_in_g),
    .video_de   (alg_in_de),
    .gamma_de   (),
    .gamma_data (gamma_data_g)
);

gamma_lookuptable u_alg_gamma_b (
    .video_clk  (clk),
    .video_data (alg_in_b),
    .video_de   (alg_in_de),
    .gamma_de   (),
    .gamma_data (gamma_data_b)
);

RGB2YCbCr u_alg_RGB2YCbCr (
    .clk       (clk),
    .rst_n     (~rst),
    .vsync_in  (alg_in_vs),
    .hsync_in  (alg_in_de),
    .de_in     (alg_in_de),
    .red       (alg_in_r[7:3]),
    .green     (alg_in_g[7:2]),
    .blue      (alg_in_b[7:3]),
    .vsync_out (y_vs),
    .hsync_out (),
    .de_out    (y_de),
    .y         (y_data),
    .cb        (cb_data),
    .cr        (cr_data)
);

sobel_sharpen_proc #(
    .IMG_HDISP(IMG_WIDTH),
    .IMG_VDISP(IMG_HEIGHT)
) u_alg_sobel_sharpen_proc (
    .clk             (clk),
    .rst_n           (~rst),
    .per_img_vsync   (y_vs),
    .per_img_href    (y_de),
    .per_img_gray    (y_data),
    .post_img_vsync  (sharpen_vsync),
    .post_img_href   (sharpen_de),
    .post_img_gray   (sharpen_gray)
);

matrix_3x3 #(
    .IMG_WIDTH  (11'd128),
    .IMG_HEIGHT (11'd72)
) u_alg_matrix_3x3 (
    .video_clk  (clk),
    .rst_n      (~rst),
    .video_vs   (y_vs),
    .video_de   (y_de),
    .video_data (y_data),
    .matrix_de  (matrix_de),
    .matrix11   (matrix11),
    .matrix12   (matrix12),
    .matrix13   (matrix13),
    .matrix21   (matrix21),
    .matrix22   (matrix22),
    .matrix23   (matrix23),
    .matrix31   (matrix31),
    .matrix32   (matrix32),
    .matrix33   (matrix33)
);

aver_filter u_alg_aver_filter (
    .video_clk        (clk),
    .rst_n            (~rst),
    .matrix_de        (matrix_de),
    .matrix_vs        (y_vs),
    .matrix11         (matrix11),
    .matrix12         (matrix12),
    .matrix13         (matrix13),
    .matrix21         (matrix21),
    .matrix22         (matrix22),
    .matrix23         (matrix23),
    .matrix31         (matrix31),
    .matrix32         (matrix32),
    .matrix33         (matrix33),
    .aver_filter_vs   (aver_filter_vs),
    .aver_filter_de   (aver_filter_de),
    .aver_filter_data (aver_filter_data)
);

Curve_Contrast_Array u_alg_Curve_Contrast_Array (
    .Pre_Data  (y_data),
    .Post_Data (curve_gray)
);

gauss_filter u_alg_gauss_filter (
    .video_clk         (clk),
    .rst_n             (~rst),
    .matrix_de         (matrix_de),
    .matrix_vs         (y_vs),
    .matrix11          (matrix11),
    .matrix12          (matrix12),
    .matrix13          (matrix13),
    .matrix21          (matrix21),
    .matrix22          (matrix22),
    .matrix23          (matrix23),
    .matrix31          (matrix31),
    .matrix32          (matrix32),
    .matrix33          (matrix33),
    .gauss_filter_vs   (gauss_filter_vs),
    .gauss_filter_de   (gauss_filter_de),
    .gauss_filter_data (gauss_filter_data)
);

median_filter_3x3 u_alg_median_filter_3x3 (
    .clk         (clk),
    .rst_n       (~rst),
    .vsync_in    (y_vs),
    .hsync_in    (matrix_de),
    .de_in       (matrix_de),
    .data11      (matrix11),
    .data12      (matrix12),
    .data13      (matrix13),
    .data21      (matrix21),
    .data22      (matrix22),
    .data23      (matrix23),
    .data31      (matrix31),
    .data32      (matrix32),
    .data33      (matrix33),
    .target_data (median_data),
    .vsync_out   (median_vs),
    .hsync_out   (median_hs),
    .de_out      (median_de)
);

sobel #(
    .SOBEL_THRESHOLD(64)
) u_alg_sobel (
    .video_clk  (clk),
    .rst_n      (~rst),
    .matrix_de  (matrix_de),
    .matrix_vs  (y_vs),
    .matrix11   (matrix11),
    .matrix12   (matrix12),
    .matrix13   (matrix13),
    .matrix21   (matrix21),
    .matrix22   (matrix22),
    .matrix23   (matrix23),
    .matrix31   (matrix31),
    .matrix32   (matrix32),
    .matrix33   (matrix33),
    .sobel_vs   (alg_sobel_vs),
    .sobel_de   (alg_sobel_de),
    .sobel_data (alg_sobel_data)
);

binarization u_alg_binarization (
    .clk       (clk),
    .rst_n     (~rst),
    .vsync_in  (y_vs),
    .hsync_in  (y_de),
    .de_in     (y_de),
    .y_in      (y_data),
    .vsync_out (bin_vs),
    .hsync_out (bin_hs),
    .de_out    (bin_de),
    .pix       (bin_data)
);

area_bin u_alg_area_bin (
    .video_clk     (clk),
    .rst_n         (~rst),
    .matrix_de     (matrix_de),
    .matrix_vs     (y_vs),
    .matrix11      (matrix11),
    .matrix12      (matrix12),
    .matrix13      (matrix13),
    .matrix21      (matrix21),
    .matrix22      (matrix22),
    .matrix23      (matrix23),
    .matrix31      (matrix31),
    .matrix32      (matrix32),
    .matrix33      (matrix33),
    .area_bin_vs   (area_bin_vs),
    .area_bin_de   (area_bin_de),
    .area_bin_data (area_bin_data)
);

matrix_3x3_1bit #(
    .IMG_WIDTH  (11'd128),
    .IMG_HEIGHT (11'd72)
) u_alg_matrix_3x3_1bit (
    .video_clk  (clk),
    .rst_n      (~rst),
    .video_vs   (bin_vs),
    .video_de   (bin_de),
    .video_data (bin_data),
    .matrix_de  (matrix_de_1bit),
    .matrix11   (matrix11_1bit),
    .matrix12   (matrix12_1bit),
    .matrix13   (matrix13_1bit),
    .matrix21   (matrix21_1bit),
    .matrix22   (matrix22_1bit),
    .matrix23   (matrix23_1bit),
    .matrix31   (matrix31_1bit),
    .matrix32   (matrix32_1bit),
    .matrix33   (matrix33_1bit)
);

erosion u_alg_erosion (
    .video_clk    (clk),
    .rst_n        (~rst),
    // 注意：原顶层这里连接的是 area_bin_vs，但 1bit 矩阵来自 bin_data。
    // 为保持数据源一致，这里连接 bin_vs；若老师工程要求完全照搬，请改回 area_bin_vs。
    .bin_vs       (bin_vs),
    .bin_de       (matrix_de_1bit),
    .bin_data_11  (matrix11_1bit),
    .bin_data_12  (matrix12_1bit),
    .bin_data_13  (matrix13_1bit),
    .bin_data_21  (matrix21_1bit),
    .bin_data_22  (matrix22_1bit),
    .bin_data_23  (matrix23_1bit),
    .bin_data_31  (matrix31_1bit),
    .bin_data_32  (matrix32_1bit),
    .bin_data_33  (matrix33_1bit),
    .erosion_vs   (erosion_vs),
    .erosion_de   (erosion_de),
    .erosion_data (erosion_data)
);

matrix_3x3_1bit #(
    .IMG_WIDTH  (11'd128),
    .IMG_HEIGHT (11'd72)
) u_alg_matrix_3x3_erosion (
    .video_clk  (clk),
    .rst_n      (~rst),
    .video_vs   (erosion_vs),
    .video_de   (erosion_de),
    .video_data (erosion_data),
    .matrix_de  (matrix_de_erosion),
    .matrix11   (matrix11_erosion),
    .matrix12   (matrix12_erosion),
    .matrix13   (matrix13_erosion),
    .matrix21   (matrix21_erosion),
    .matrix22   (matrix22_erosion),
    .matrix23   (matrix23_erosion),
    .matrix31   (matrix31_erosion),
    .matrix32   (matrix32_erosion),
    .matrix33   (matrix33_erosion)
);

dilate u_alg_dilate (
    .video_clk    (clk),
    .rst_n        (~rst),
    // 注意：原顶层 bin_vs 接 bin_vs，但矩阵来自 erosion_data。
    // 为保持数据源一致，这里连接 erosion_vs；如需完全照搬原工程，请改回 bin_vs。
    .bin_vs       (erosion_vs),
    .bin_de       (matrix_de_erosion),
    .bin_data_11  (matrix11_erosion),
    .bin_data_12  (matrix12_erosion),
    .bin_data_13  (matrix13_erosion),
    .bin_data_21  (matrix21_erosion),
    .bin_data_22  (matrix22_erosion),
    .bin_data_23  (matrix23_erosion),
    .bin_data_31  (matrix31_erosion),
    .bin_data_32  (matrix32_erosion),
    .bin_data_33  (matrix33_erosion),
    .dilate_vs    (dilate_vs),
    .dilate_de    (dilate_de),
    .dilate_data  (dilate_data)
);


always @(posedge clk) begin
    if (rst) begin
        h_cnt <= 12'd0;
    end else if (h_cnt == H_TOTAL - 1) begin
        h_cnt <= 12'd0;
    end else begin
        h_cnt <= h_cnt + 12'd1;
    end
end

always @(posedge clk) begin
    if (rst) begin
        v_cnt <= 12'd0;
    end else if (h_cnt == H_TOTAL - 1) begin
        if (v_cnt == V_TOTAL - 1) begin
            v_cnt <= 12'd0;
        end else begin
            v_cnt <= v_cnt + 12'd1;
        end
    end
end

always @(posedge clk) begin
    if (rst) begin
        hs_reg <= 1'b0;
        vs_reg <= 1'b0;
        de_reg <= 1'b0;
        hs_reg_d0 <= 1'b0;
        vs_reg_d0 <= 1'b0;
        de_reg_d0 <= 1'b0;
        display_rd_addr <= 14'd0;
        edge_pixel <= 8'd0;
        rgb_pixel <= 24'd0;
        hue_pixel <= 24'd0;
        gamma_pixel <= 24'd0;
        brightness_pixel <= 24'd0;
        gray_pixel <= 8'd0;
        sharpen_pixel <= 8'd0;
        aver_pixel <= 8'd0;
        contrast_pixel <= 8'd0;
        gauss_pixel <= 8'd0;
        median_pixel <= 8'd0;
        alg_sobel_pixel <= 8'd0;
        bin_pixel <= 1'b0;
        area_bin_pixel <= 1'b0;
        erosion_pixel <= 1'b0;
        dilate_pixel <= 1'b0;
    end else begin
        hs_reg <= hsync_now;
        vs_reg <= vsync_now;
        de_reg <= video_active;
        hs_reg_d0 <= hs_reg;
        vs_reg_d0 <= vs_reg;
        de_reg_d0 <= de_reg;
        display_rd_addr <= video_active ? disp_addr : 14'd0;
        edge_pixel <= edge_mem[display_rd_addr];
        rgb_pixel <= rgb_mem[display_rd_addr];
        hue_pixel <= hue_mem[display_rd_addr];
        gamma_pixel <= gamma_mem[display_rd_addr];
        brightness_pixel <= brightness_mem[display_rd_addr];
        gray_pixel <= gray_mem[display_rd_addr];
        sharpen_pixel <= sharpen_mem[display_rd_addr];
        aver_pixel <= aver_mem[display_rd_addr];
        contrast_pixel <= contrast_mem[display_rd_addr];
        gauss_pixel <= gauss_mem[display_rd_addr];
        median_pixel <= median_mem[display_rd_addr];
        alg_sobel_pixel <= alg_sobel_mem[display_rd_addr];
        bin_pixel <= bin_mem[display_rd_addr];
        area_bin_pixel <= area_bin_mem[display_rd_addr];
        erosion_pixel <= erosion_mem[display_rd_addr];
        dilate_pixel <= dilate_mem[display_rd_addr];
    end
end

always @(posedge clk) begin
    if (rst) begin
        scan_state <= SCAN_IDLE;
        scan_x <= 7'd0;
        scan_y <= 7'd0;
        bram_addr_reg <= 32'd0;
        bram_en <= 1'b0;
        scan_valid_d1 <= 1'b0;
        scan_valid_d2 <= 1'b0;
        scan_x_d1 <= 7'd0;
        scan_y_d1 <= 7'd0;
        scan_x_d2 <= 7'd0;
        scan_y_d2 <= 7'd0;
        scan_frame_start <= 1'b0;
        sobel_done <= 1'b0;
        display_mode <= MODE_SOBEL;
        threshold <= 8'd80;
        overlay_enable <= 1'b0;
    end else begin
        scan_frame_start <= 1'b0;

        bram_en <= scan_issue || scan_valid_d1 || scan_valid_d2 || ctrl_read_active;
        if (scan_issue) begin
            bram_addr_reg <= {16'd0, scan_word_addr, 2'b00};
        end

        scan_valid_d1 <= scan_issue;
        scan_valid_d2 <= scan_valid_d1;
        scan_x_d1 <= scan_x;
        scan_y_d1 <= scan_y;
        scan_x_d2 <= scan_x_d1;
        scan_y_d2 <= scan_y_d1;

        case (scan_state)
            SCAN_IDLE: begin
                if (video_frame_start) begin
                    scan_state <= SCAN_CTRL_MODE_REQ;
                    scan_x <= 7'd0;
                    scan_y <= 7'd0;
                    sobel_done <= 1'b0;
                end
            end

            SCAN_CTRL_MODE_REQ: begin
                bram_addr_reg <= CTRL_MODE_ADDR;
                scan_state <= SCAN_CTRL_MODE_WAIT1;
            end

            SCAN_CTRL_MODE_WAIT1: begin
                scan_state <= SCAN_CTRL_MODE_WAIT2;
            end

            SCAN_CTRL_MODE_WAIT2: begin
                // 修改点 E：从控制字读取 4 bit mode，支持 0~15。
                // 不要修改 bram_dout 端口宽度；bram_dout 仍必须保持 32 bit，
                // 因为图像像素读取仍然使用 bram_dout[23:0]。
                display_mode <= bram_dout[3:0];
                scan_state <= SCAN_CTRL_THR_REQ;
            end

            SCAN_CTRL_THR_REQ: begin
                bram_addr_reg <= CTRL_THRESHOLD_ADDR;
                scan_state <= SCAN_CTRL_THR_WAIT1;
            end

            SCAN_CTRL_THR_WAIT1: begin
                scan_state <= SCAN_CTRL_THR_WAIT2;
            end

            SCAN_CTRL_THR_WAIT2: begin
                // 控制字读取：threshold 仍然只使用低 8 bit，不受 display_mode 扩展影响。
                threshold <= bram_dout[7:0];
                scan_state <= SCAN_CTRL_OVL_REQ;
            end

            SCAN_CTRL_OVL_REQ: begin
                bram_addr_reg <= CTRL_OVERLAY_ADDR;
                scan_state <= SCAN_CTRL_OVL_WAIT1;
            end

            SCAN_CTRL_OVL_WAIT1: begin
                scan_state <= SCAN_CTRL_OVL_WAIT2;
            end

            SCAN_CTRL_OVL_WAIT2: begin
                // 控制字读取：overlay_enable 仍然只使用 bit0，不受 display_mode 扩展影响。
                overlay_enable <= bram_dout[0];
                scan_state <= SCAN_RUN;
                scan_frame_start <= 1'b1;
            end

            SCAN_RUN: begin
                if (scan_last) begin
                    scan_state <= SCAN_WAIT;
                    scan_x <= 7'd0;
                    scan_y <= 7'd0;
                end else if (scan_x == 7'd127) begin
                    scan_x <= 7'd0;
                    scan_y <= scan_y + 7'd1;
                end else begin
                    scan_x <= scan_x + 7'd1;
                end
            end

            SCAN_WAIT: begin
                if (edge_frame_done) begin
                    scan_state <= SCAN_IDLE;
                    sobel_done <= 1'b1;
                end
            end

            default: begin
                scan_state <= SCAN_IDLE;
            end
        endcase
    end
end

// -----------------------------------------------------------------------------
// BRAM image/edge cache writeback
// -----------------------------------------------------------------------------
// 连接说明 3：图像数据通路仍然保持不变。
// scan_valid_d2 有效时，bram_dout[23:0] 被写入 rgb_mem。
// 因此扩展 display_mode 时不能把 bram_dout 端口改成 [3:0]，否则会破坏图像传输。
always @(posedge clk) begin
    if (scan_valid_d2) begin
        rgb_mem[scan_store_addr] <= bram_dout[23:0];
    end

    if (edge_valid) begin
        edge_mem[edge_wr_addr] <= edge_data;
    end
end


// -----------------------------------------------------------------------------
// S05-ALG-CACHE：把各算法模块的流式输出写入缓存 RAM
// -----------------------------------------------------------------------------
// 写地址按各自 de/valid 递增。scan_frame_start 到来时统一清零，开始缓存新一帧。
// 如果某个算法模块内部延迟较大，前几帧可能显示旧数据；连续运行后会稳定刷新。
always @(posedge clk) begin
    if (rst || scan_frame_start) begin
        hue_wr_addr        <= 14'd0;
        gamma_wr_addr      <= 14'd0;
        brightness_wr_addr <= 14'd0;
        gray_wr_addr       <= 14'd0;
        sharpen_wr_addr    <= 14'd0;
        aver_wr_addr       <= 14'd0;
        contrast_wr_addr   <= 14'd0;
        gauss_wr_addr      <= 14'd0;
        median_wr_addr     <= 14'd0;
        alg_sobel_wr_addr  <= 14'd0;
        bin_wr_addr        <= 14'd0;
        area_bin_wr_addr   <= 14'd0;
        erosion_wr_addr    <= 14'd0;
        dilate_wr_addr     <= 14'd0;
    end else begin
        if (chrominance_de && (hue_wr_addr < 14'd9216)) begin
            hue_mem[hue_wr_addr] <= {chrominance_gray, chrominance_gray, chrominance_gray};
            hue_wr_addr <= hue_wr_addr + 14'd1;
        end

        if (gamma_de && (gamma_wr_addr < 14'd9216)) begin
            gamma_mem[gamma_wr_addr] <= {gamma_data_r, gamma_data_g, gamma_data_b};
            gamma_wr_addr <= gamma_wr_addr + 14'd1;
        end

        if (brightness_de && (brightness_wr_addr < 14'd9216)) begin
            brightness_mem[brightness_wr_addr] <= {brightness_out_r, brightness_out_g, brightness_out_b};
            brightness_wr_addr <= brightness_wr_addr + 14'd1;
        end

        if (y_de && (gray_wr_addr < 14'd9216)) begin
            gray_mem[gray_wr_addr] <= y_data;
            gray_wr_addr <= gray_wr_addr + 14'd1;
        end

        if (sharpen_de && (sharpen_wr_addr < 14'd9216)) begin
            sharpen_mem[sharpen_wr_addr] <= sharpen_gray;
            sharpen_wr_addr <= sharpen_wr_addr + 14'd1;
        end

        if (aver_filter_de && (aver_wr_addr < 14'd9216)) begin
            aver_mem[aver_wr_addr] <= aver_filter_data;
            aver_wr_addr <= aver_wr_addr + 14'd1;
        end

        if (y_de && (contrast_wr_addr < 14'd9216)) begin
            contrast_mem[contrast_wr_addr] <= curve_gray;
            contrast_wr_addr <= contrast_wr_addr + 14'd1;
        end

        if (gauss_filter_de && (gauss_wr_addr < 14'd9216)) begin
            gauss_mem[gauss_wr_addr] <= gauss_filter_data;
            gauss_wr_addr <= gauss_wr_addr + 14'd1;
        end

        if (median_de && (median_wr_addr < 14'd9216)) begin
            median_mem[median_wr_addr] <= median_data;
            median_wr_addr <= median_wr_addr + 14'd1;
        end

        if (alg_sobel_de && (alg_sobel_wr_addr < 14'd9216)) begin
            alg_sobel_mem[alg_sobel_wr_addr] <= alg_sobel_data;
            alg_sobel_wr_addr <= alg_sobel_wr_addr + 14'd1;
        end

        if (bin_de && (bin_wr_addr < 14'd9216)) begin
            bin_mem[bin_wr_addr] <= bin_data;
            bin_wr_addr <= bin_wr_addr + 14'd1;
        end

        if (area_bin_de && (area_bin_wr_addr < 14'd9216)) begin
            area_bin_mem[area_bin_wr_addr] <= area_bin_data;
            area_bin_wr_addr <= area_bin_wr_addr + 14'd1;
        end

        if (erosion_de && (erosion_wr_addr < 14'd9216)) begin
            erosion_mem[erosion_wr_addr] <= erosion_data;
            erosion_wr_addr <= erosion_wr_addr + 14'd1;
        end

        if (dilate_de && (dilate_wr_addr < 14'd9216)) begin
            dilate_mem[dilate_wr_addr] <= dilate_data;
            dilate_wr_addr <= dilate_wr_addr + 14'd1;
        end
    end
end

endmodule
