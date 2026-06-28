`timescale 1ns / 1ps

// ============================================================
// video_stream_alg_640.v
// S05-640P-ALG
//
// 作用：对 640x480 RGB888 实时视频流做模式选择和基础图像算法处理。
// 放置位置：DDR 读出视频流之后，HDMI 输出之前。
//
// 注意：这是“真 640x480 视频流处理”模块，不再使用 128x72 图像缓存。
// 输入/输出均为 video_clk 同步的视频流：hs/vs/de + RGB888。
// ============================================================

module video_stream_alg_640 #(
    parameter H_ACTIVE = 640,
    parameter V_ACTIVE = 480
)(
    input  wire        clk,
    input  wire        rst,

    input  wire        in_hs,
    input  wire        in_vs,
    input  wire        in_de,
    input  wire [7:0]  in_r,
    input  wire [7:0]  in_g,
    input  wire [7:0]  in_b,

    input  wire [3:0]  display_mode,
    input  wire [7:0]  threshold,
    input  wire        overlay_enable,

    output reg         out_hs,
    output reg         out_vs,
    output reg         out_de,
    output reg  [7:0]  out_r,
    output reg  [7:0]  out_g,
    output reg  [7:0]  out_b
);

    // ========================================================
    // S05-640P-ALG-A：模式编号必须和 GUI / PS / PL 保持一致
    // ========================================================
    localparam [3:0] MODE_ORIGINAL    = 4'd0;
    localparam [3:0] MODE_HUE         = 4'd1;
    localparam [3:0] MODE_GAMMA       = 4'd2;
    localparam [3:0] MODE_BRIGHTNESS  = 4'd3;
    localparam [3:0] MODE_GRAY        = 4'd4;
    localparam [3:0] MODE_SHARPEN     = 4'd5;
    localparam [3:0] MODE_AVER        = 4'd6;
    localparam [3:0] MODE_CONTRAST    = 4'd7;
    localparam [3:0] MODE_GAUSS       = 4'd8;
    localparam [3:0] MODE_MIDFILTER   = 4'd9;
    localparam [3:0] MODE_SOBLE       = 4'd10;
    localparam [3:0] MODE_RED_OVERLAY = 4'd11;
    localparam [3:0] MODE_BIN         = 4'd12;
    localparam [3:0] MODE_AREA_BIN    = 4'd13;
    localparam [3:0] MODE_EROSION     = 4'd14;
    localparam [3:0] MODE_DILATE      = 4'd15;

    // ========================================================
    // 工具函数
    // ========================================================
    function [7:0] sat_add8;
        input [7:0] a;
        input [7:0] b;
        reg [8:0] s;
        begin
            s = {1'b0, a} + {1'b0, b};
            sat_add8 = s[8] ? 8'hff : s[7:0];
        end
    endfunction

    function [7:0] sat_sub8;
        input [7:0] a;
        input [7:0] b;
        begin
            sat_sub8 = (a > b) ? (a - b) : 8'd0;
        end
    endfunction

    function [7:0] gray_calc;
        input [7:0] r;
        input [7:0] g;
        input [7:0] b;
        reg [15:0] y;
        begin
            // 0.299R + 0.587G + 0.114B 近似
            y = r * 8'd77 + g * 8'd150 + b * 8'd29;
            gray_calc = y[15:8];
        end
    endfunction

    function [7:0] gamma_approx;
        input [7:0] x;
        reg [15:0] xx;
        begin
            // 简单 gamma 近似：x^2 / 255，偏暗效果，便于硬件实现
            xx = x * x;
            gamma_approx = xx[15:8];
        end
    endfunction

    function [7:0] contrast_approx;
        input [7:0] x;
        reg signed [9:0] t;
        begin
            // 对比度增强：以 128 为中心放大 2 倍
            t = (($signed({1'b0, x}) - 10'sd128) <<< 1) + 10'sd128;
            if (t < 0)
                contrast_approx = 8'd0;
            else if (t > 255)
                contrast_approx = 8'hff;
            else
                contrast_approx = t[7:0];
        end
    endfunction

    // 说明：真正 9 点中值滤波在 25MHz 下资源和时序压力较大。
    // 这里先把 midfilter 做成“平均滤波显示”，便于工程先跑通；
    // 如果后续资源和时序允许，可以替换为流水线排序网络。

    // ========================================================
    // S05-640P-ALG-B：3x3 灰度窗口行缓存
    // 说明：WIDTH=640 时，这里会综合成行缓存资源。
    // ========================================================
    reg [11:0] x_cnt;
    reg [11:0] y_cnt;
    reg        de_d;
    reg        vs_d;

    wire [7:0] gray_in = gray_calc(in_r, in_g, in_b);

    (* ram_style = "block" *) reg [7:0] line0 [0:H_ACTIVE-1];
    (* ram_style = "block" *) reg [7:0] line1 [0:H_ACTIVE-1];

    wire [7:0] line0_val = line0[x_cnt];
    wire [7:0] line1_val = line1[x_cnt];

    reg [7:0] w11, w12, w13;
    reg [7:0] w21, w22, w23;
    reg [7:0] w31, w32, w33;

    always @(posedge clk) begin
        if (rst) begin
            x_cnt <= 12'd0;
            y_cnt <= 12'd0;
            de_d  <= 1'b0;
            vs_d  <= 1'b0;
            {w11,w12,w13,w21,w22,w23,w31,w32,w33} <= 72'd0;
        end else begin
            de_d <= in_de;
            vs_d <= in_vs;

            if (in_vs && !vs_d) begin
                x_cnt <= 12'd0;
                y_cnt <= 12'd0;
            end else if (in_de) begin
                // 行缓存：line0 保存上一行，line1 保存上上一行
                line0[x_cnt] <= gray_in;
                line1[x_cnt] <= line0_val;

                // 3x3 移位窗口
                w11 <= w12; w12 <= w13; w13 <= line1_val;
                w21 <= w22; w22 <= w23; w23 <= line0_val;
                w31 <= w32; w32 <= w33; w33 <= gray_in;

                if (x_cnt == H_ACTIVE-1) begin
                    x_cnt <= 12'd0;
                    if (y_cnt != V_ACTIVE-1)
                        y_cnt <= y_cnt + 1'b1;
                end else begin
                    x_cnt <= x_cnt + 1'b1;
                end
            end else if (de_d && !in_de) begin
                x_cnt <= 12'd0;
            end
        end
    end

    wire window_valid = in_de && (x_cnt >= 12'd2) && (y_cnt >= 12'd2);

    wire [11:0] avg_sum   = w11 + w12 + w13 + w21 + w22 + w23 + w31 + w32 + w33;
    wire [7:0]  avg_gray  = avg_sum / 9;
    wire [11:0] gauss_sum = w11 + (w12<<1) + w13 + (w21<<1) + (w22<<2) + (w23<<1) + w31 + (w32<<1) + w33;
    wire [7:0]  gauss_gray = gauss_sum[11:4];

    wire signed [10:0] sobel_gx = -$signed({3'b0,w11}) + $signed({3'b0,w13})
                                - ($signed({3'b0,w21}) <<< 1) + ($signed({3'b0,w23}) <<< 1)
                                - $signed({3'b0,w31}) + $signed({3'b0,w33});
    wire signed [10:0] sobel_gy =  $signed({3'b0,w11}) + ($signed({3'b0,w12}) <<< 1) + $signed({3'b0,w13})
                                - $signed({3'b0,w31}) - ($signed({3'b0,w32}) <<< 1) - $signed({3'b0,w33});
    wire [10:0] abs_gx = sobel_gx[10] ? (~sobel_gx + 1'b1) : sobel_gx;
    wire [10:0] abs_gy = sobel_gy[10] ? (~sobel_gy + 1'b1) : sobel_gy;
    wire [11:0] sobel_sum = abs_gx + abs_gy;
    wire [7:0]  sobel_gray = sobel_sum > 12'd255 ? 8'hff : sobel_sum[7:0];
    wire        edge_on = window_valid && (sobel_gray >= threshold);

    wire bin_center = gray_in >= threshold;
    wire bin_area   = avg_gray >= threshold;
    wire erosion_on = (w11 >= threshold) && (w12 >= threshold) && (w13 >= threshold) &&
                      (w21 >= threshold) && (w22 >= threshold) && (w23 >= threshold) &&
                      (w31 >= threshold) && (w32 >= threshold) && (w33 >= threshold);
    wire dilate_on  = (w11 >= threshold) || (w12 >= threshold) || (w13 >= threshold) ||
                      (w21 >= threshold) || (w22 >= threshold) || (w23 >= threshold) ||
                      (w31 >= threshold) || (w32 >= threshold) || (w33 >= threshold);
    wire [7:0] median_gray = avg_gray; // S05-640P-ALG-D：当前版本用平均滤波占位中值滤波，便于先通过时序

    // ========================================================
    // S05-640P-ALG-C：模式选择输出
    // ========================================================
    always @(posedge clk) begin
        if (rst) begin
            out_hs <= 1'b0;
            out_vs <= 1'b0;
            out_de <= 1'b0;
            out_r  <= 8'd0;
            out_g  <= 8'd0;
            out_b  <= 8'd0;
        end else begin
            out_hs <= in_hs;
            out_vs <= in_vs;
            out_de <= in_de;

            if (!in_de) begin
                out_r <= 8'd0;
                out_g <= 8'd0;
                out_b <= 8'd0;
            end else begin
                case (display_mode)
                    MODE_ORIGINAL: begin
                        out_r <= in_r; out_g <= in_g; out_b <= in_b;
                    end
                    MODE_HUE: begin
                        // 色度演示：通道轮换，便于肉眼确认模式切换
                        out_r <= in_g; out_g <= in_b; out_b <= in_r;
                    end
                    MODE_GAMMA: begin
                        out_r <= gamma_approx(in_r); out_g <= gamma_approx(in_g); out_b <= gamma_approx(in_b);
                    end
                    MODE_BRIGHTNESS: begin
                        out_r <= sat_add8(in_r, 8'd32); out_g <= sat_add8(in_g, 8'd32); out_b <= sat_add8(in_b, 8'd32);
                    end
                    MODE_GRAY: begin
                        out_r <= gray_in; out_g <= gray_in; out_b <= gray_in;
                    end
                    MODE_SHARPEN: begin
                        // 简化锐化：center + (center - average)
                        out_r <= sat_add8(gray_in, sat_sub8(gray_in, avg_gray));
                        out_g <= sat_add8(gray_in, sat_sub8(gray_in, avg_gray));
                        out_b <= sat_add8(gray_in, sat_sub8(gray_in, avg_gray));
                    end
                    MODE_AVER: begin
                        out_r <= avg_gray; out_g <= avg_gray; out_b <= avg_gray;
                    end
                    MODE_CONTRAST: begin
                        out_r <= contrast_approx(in_r); out_g <= contrast_approx(in_g); out_b <= contrast_approx(in_b);
                    end
                    MODE_GAUSS: begin
                        out_r <= gauss_gray; out_g <= gauss_gray; out_b <= gauss_gray;
                    end
                    MODE_MIDFILTER: begin
                        out_r <= median_gray; out_g <= median_gray; out_b <= median_gray;
                    end
                    MODE_SOBLE: begin
                        out_r <= edge_on ? 8'hff : 8'h00;
                        out_g <= edge_on ? 8'hff : 8'h00;
                        out_b <= edge_on ? 8'hff : 8'h00;
                    end
                    MODE_RED_OVERLAY: begin
                        if (edge_on || overlay_enable) begin
                            // overlay_enable=1 时也保持红色边缘叠加模式有效
                            out_r <= edge_on ? 8'hff : in_r;
                            out_g <= edge_on ? 8'h00 : in_g;
                            out_b <= edge_on ? 8'h00 : in_b;
                        end else begin
                            out_r <= in_r; out_g <= in_g; out_b <= in_b;
                        end
                    end
                    MODE_BIN: begin
                        out_r <= bin_center ? 8'hff : 8'h00;
                        out_g <= bin_center ? 8'hff : 8'h00;
                        out_b <= bin_center ? 8'hff : 8'h00;
                    end
                    MODE_AREA_BIN: begin
                        out_r <= bin_area ? 8'hff : 8'h00;
                        out_g <= bin_area ? 8'hff : 8'h00;
                        out_b <= bin_area ? 8'hff : 8'h00;
                    end
                    MODE_EROSION: begin
                        out_r <= erosion_on ? 8'hff : 8'h00;
                        out_g <= erosion_on ? 8'hff : 8'h00;
                        out_b <= erosion_on ? 8'hff : 8'h00;
                    end
                    MODE_DILATE: begin
                        out_r <= dilate_on ? 8'hff : 8'h00;
                        out_g <= dilate_on ? 8'hff : 8'h00;
                        out_b <= dilate_on ? 8'hff : 8'h00;
                    end
                    default: begin
                        out_r <= in_r; out_g <= in_g; out_b <= in_b;
                    end
                endcase
            end
        end
    end

endmodule
