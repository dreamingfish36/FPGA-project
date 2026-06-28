module hdmi_bram_display(
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

// 原来 10 倍会铺满 1280x720，这里改成 4 倍，方便把图像放到右上角并显示边框
localparam IMG_SCALE  = 4;
localparam DISP_IMG_W = IMG_WIDTH  * IMG_SCALE;
localparam DISP_IMG_H = IMG_HEIGHT * IMG_SCALE;

// 边框宽度和颜色
localparam BORDER_SIZE = 4;
localparam [7:0] BORDER_R = 8'hFF;
localparam [7:0] BORDER_G = 8'h00;
localparam [7:0] BORDER_B = 8'h00;

// 背景颜色
localparam [7:0] BG_R = 8'h00;
localparam [7:0] BG_G = 8'h00;
localparam [7:0] BG_B = 8'h00;

// 右上角显示：整个“边框 + 图像”区域靠右、靠上
localparam FRAME_X = H_ACTIVE - DISP_IMG_W - (BORDER_SIZE * 2);
localparam FRAME_Y = 0;
localparam IMG_START_X = FRAME_X + BORDER_SIZE;
localparam IMG_START_Y = FRAME_Y + BORDER_SIZE;
localparam FRAME_W = DISP_IMG_W + (BORDER_SIZE * 2);
localparam FRAME_H = DISP_IMG_H + (BORDER_SIZE * 2);

reg [11:0] h_cnt;
reg [11:0] v_cnt;
reg hs_reg;
reg vs_reg;
reg de_reg;
reg hs_reg_d0;
reg vs_reg_d0;
reg de_reg_d0;
reg hs_reg_d1;
reg vs_reg_d1;
reg de_reg_d1;
reg image_area_d0;
reg image_area_d1;
reg border_area_d0;
reg border_area_d1;
reg [31:0] bram_addr_reg;
reg [23:0] pixel_reg;

wire h_active;
wire v_active;
wire video_active;
wire hsync_now;
wire vsync_now;
wire [11:0] active_x;
wire [11:0] active_y;
wire image_area;
wire frame_area;
wire border_area;
wire [6:0] image_x;
wire [6:0] image_y;
wire [13:0] image_word_addr;

assign h_active = (h_cnt >= H_START[11:0]) && (h_cnt < (H_START + H_ACTIVE));
assign v_active = (v_cnt >= V_START[11:0]) && (v_cnt < (V_START + V_ACTIVE));
assign video_active = h_active && v_active;

assign hsync_now = (h_cnt >= H_FP[11:0]) && (h_cnt < (H_FP + H_SYNC));
assign vsync_now = (v_cnt >= V_FP[11:0]) && (v_cnt < (V_FP + V_SYNC));

assign active_x = h_cnt - H_START[11:0];
assign active_y = v_cnt - V_START[11:0];

assign image_area = video_active &&
                    (active_x >= IMG_START_X[11:0]) &&
                    (active_x <  (IMG_START_X + DISP_IMG_W)) &&
                    (active_y >= IMG_START_Y[11:0]) &&
                    (active_y <  (IMG_START_Y + DISP_IMG_H));

assign frame_area = video_active &&
                    (active_x >= FRAME_X[11:0]) &&
                    (active_x <  (FRAME_X + FRAME_W)) &&
                    (active_y >= FRAME_Y[11:0]) &&
                    (active_y <  (FRAME_Y + FRAME_H));

assign border_area = frame_area && !image_area;

// IMG_SCALE = 4，所以除以 4 可以写成右移 2 位
assign image_x = (active_x - IMG_START_X[11:0]) >> 2;
assign image_y = (active_y - IMG_START_Y[11:0]) >> 2;
assign image_word_addr = {image_y, 7'b0} + {7'd0, image_x};

assign hs = hs_reg_d1;
assign vs = vs_reg_d1;
assign de = de_reg_d1;

assign rgb_r = de_reg_d1 ? (border_area_d1 ? BORDER_R : (image_area_d1 ? pixel_reg[23:16] : BG_R)) : 8'h00;
assign rgb_g = de_reg_d1 ? (border_area_d1 ? BORDER_G : (image_area_d1 ? pixel_reg[15:8]  : BG_G)) : 8'h00;
assign rgb_b = de_reg_d1 ? (border_area_d1 ? BORDER_B : (image_area_d1 ? pixel_reg[7:0]   : BG_B)) : 8'h00;

assign bram_we = 4'b0000;
assign bram_din = 32'd0;
assign bram_addr = bram_addr_reg;

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
        hs_reg_d1 <= 1'b0;
        vs_reg_d1 <= 1'b0;
        de_reg_d1 <= 1'b0;
        image_area_d0 <= 1'b0;
        image_area_d1 <= 1'b0;
        border_area_d0 <= 1'b0;
        border_area_d1 <= 1'b0;
        bram_en <= 1'b0;
        bram_addr_reg <= 32'd0;
        pixel_reg <= 24'd0;
    end else begin
        hs_reg <= hsync_now;
        vs_reg <= vsync_now;
        de_reg <= video_active;
        hs_reg_d0 <= hs_reg;
        vs_reg_d0 <= vs_reg;
        de_reg_d0 <= de_reg;
        hs_reg_d1 <= hs_reg_d0;
        vs_reg_d1 <= vs_reg_d0;
        de_reg_d1 <= de_reg_d0;

        image_area_d0 <= image_area;
        image_area_d1 <= image_area_d0;
        border_area_d0 <= border_area;
        border_area_d1 <= border_area_d0;

        bram_en <= image_area;
        bram_addr_reg <= image_area ? {16'd0, image_word_addr, 2'b00} : 32'd0;
        pixel_reg <= bram_dout[23:0];
    end
end

endmodule
