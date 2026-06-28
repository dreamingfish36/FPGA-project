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
// Display mode definition - 10 modes, synchronized with GUI / SDK main.c
// -----------------------------------------------------------------------------
// Mode number must be exactly the same as:
//   1) PC GUI / camera_uart_sender.py DISPLAY_MODES
//   2) SDK main.c MODE_* definitions
//   3) this Verilog case(display_mode)
//
// 0 original          : original RGB image
// 1 gray              : grayscale
// 2 gauss             : Gaussian filter
// 3 mid_filter        : median filter
// 4 sobel             : Sobel edge, black/white output
// 5 edge_overlay_red  : original RGB image + red edge overlay
// 6 binary            : binary image
// 7 red               : red channel only
// 8 green             : green channel only
// 9 blue              : blue channel only
localparam [3:0] MODE_ORIGINAL         = 4'd0;  // original
localparam [3:0] MODE_GRAY             = 4'd1;  // grayscale
localparam [3:0] MODE_GAUSS_FILTER     = 4'd2;  // gauss filter
localparam [3:0] MODE_MID_FILTER       = 4'd3;  // median filter
localparam [3:0] MODE_SOBEL            = 4'd4;  // sobel edge
localparam [3:0] MODE_EDGE_OVERLAY_RED = 4'd5;  // red edge overlay
localparam [3:0] MODE_BIN              = 4'd6;  // binary
localparam [3:0] MODE_RED_ONLY         = 4'd7;  // R channel only
localparam [3:0] MODE_GREEN_ONLY       = 4'd8;  // G channel only
localparam [3:0] MODE_BLUE_ONLY        = 4'd9;  // B channel only

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
reg [23:0] rgb_pixel;

// S05-ALG-PIXEL??¦Ä??§Ø??
reg [7:0] gray_pixel;
reg [7:0] gauss_pixel;
reg [7:0] median_pixel;
reg [7:0] alg_sobel_pixel;
reg bin_pixel;

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

// display_mode is 4 bits. GUI/SDK currently use valid modes 0~9.
reg [3:0] display_mode;
reg [7:0] threshold;
reg overlay_enable;

(* ram_style = "block" *) reg [23:0] rgb_mem [0:9215];

// -----------------------------------------------------------------------------
// Algorithm result cache - only keep the 10 modes currently used by GUI / SDK.
// Removed old unused caches: hue, gamma, brightness, sharpen, average,
// contrast, area_bin, erosion and dilate.
// -----------------------------------------------------------------------------
(* ram_style = "block" *) reg [7:0]  gray_mem       [0:9215];
(* ram_style = "block" *) reg [7:0]  gauss_mem      [0:9215];
(* ram_style = "block" *) reg [7:0]  median_mem     [0:9215];
(* ram_style = "block" *) reg [7:0]  alg_sobel_mem  [0:9215];
(* ram_style = "block" *) reg        bin_mem        [0:9215];

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

// -----------------------------------------------------------------------------
// Algorithm input stream from BRAM scan path.
// Only these algorithms are kept:
// gray, gauss_filter, median_filter_3x3, sobel and binarization.
// -----------------------------------------------------------------------------
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




// RGB to Y/grayscale conversion
wire y_vs;
wire y_de;
wire [7:0] y_data;

// 3x3 matrix window shared by gauss, median, sobel
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

// Median filter

// ???

// Gaussian filter
wire gauss_filter_vs;
wire gauss_filter_de;
wire [7:0] gauss_filter_data;

// Median filter
wire median_vs;
wire median_hs;
wire median_de;
wire [7:0] median_data;

// Sobel
wire alg_sobel_vs;
wire alg_sobel_de;
wire [7:0] alg_sobel_data;

// Binary output
wire bin_vs;
wire bin_hs;
wire bin_de;
wire bin_data;


// Write addresses for the kept algorithm result caches.
reg [13:0] gray_wr_addr;
reg [13:0] gauss_wr_addr;
reg [13:0] median_wr_addr;
reg [13:0] alg_sobel_wr_addr;
reg [13:0] bin_wr_addr;

wire edge_on;
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

assign hs = hs_reg_d0;
assign vs = vs_reg_d0;
assign de = de_reg_d0;
assign rgb_r = (de_reg_d0 && sobel_done) ? out_r : 8'h00;
assign rgb_g = (de_reg_d0 && sobel_done) ? out_g : 8'h00;
assign rgb_b = (de_reg_d0 && sobel_done) ? out_b : 8'h00;

assign bram_we = 4'b0000;
assign bram_din = 32'd0;
assign bram_addr = bram_addr_reg;

assign edge_on = (alg_sobel_pixel >= threshold);

// -----------------------------------------------------------------------------
// Display mode selection - 10 modes
// -----------------------------------------------------------------------------
// The display_mode value comes from CTRL_MODE_ADDR through PS UART control.
// It must match the GUI / SDK mode table:
//   0 original, 1 gray, 2 gauss, 3 mid_filter, 4 sobel,
//   5 edge_overlay_red, 6 binary, 7 red, 8 green, 9 blue.
always @(*) begin
    // Default output: original RGB image.
    out_r = rgb_pixel[23:16];
    out_g = rgb_pixel[15:8];
    out_b = rgb_pixel[7:0];

    case (display_mode)
        MODE_ORIGINAL: begin
            out_r = rgb_pixel[23:16];
            out_g = rgb_pixel[15:8];
            out_b = rgb_pixel[7:0];
        end

        MODE_GRAY: begin
            out_r = gray_pixel;
            out_g = gray_pixel;
            out_b = gray_pixel;
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

        MODE_EDGE_OVERLAY_RED: begin
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

        MODE_RED_ONLY: begin
            out_r = rgb_pixel[23:16];
            out_g = 8'h00;
            out_b = 8'h00;
        end

        MODE_GREEN_ONLY: begin
            out_r = 8'h00;
            out_g = rgb_pixel[15:8];
            out_b = 8'h00;
        end

        MODE_BLUE_ONLY: begin
            out_r = 8'h00;
            out_g = 8'h00;
            out_b = rgb_pixel[7:0];
        end

        default: begin
            out_r = rgb_pixel[23:16];
            out_g = rgb_pixel[15:8];
            out_b = rgb_pixel[7:0];
        end
    endcase

    // Independent overlay switch: when overlay_enable=1, all non-Sobel and
    // non-overlay modes can still be forced to show red Sobel edges.
    if ((display_mode != MODE_SOBEL) &&
        (display_mode != MODE_EDGE_OVERLAY_RED) &&
        overlay_enable && edge_on) begin
        out_r = 8'hff;
        out_g = 8'h20;
        out_b = 8'h20;
    end
end

// -----------------------------------------------------------------------------
// Module connection area
// -----------------------------------------------------------------------------
// ? 1? rgb_to_gray + sobel_core??/??¡¤
// §»??????? S05-ALG-CONNECT 
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
// S05-ALG-CONNECT??????
// -----------------------------------------------------------------------------
// ?? BRAM ?alg_in_r/g/b + alg_in_de + alg_in_vs
// ???¦Ê???????
//  Vivado  "module not found"????? .v ? Design Sources
//  Vivado  "port not found"?????·Ú????????

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
    .cb        (),
    .cr        ()
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
        rgb_pixel <= 24'd0;
        gray_pixel <= 8'd0;
        gauss_pixel <= 8'd0;
        median_pixel <= 8'd0;
        alg_sobel_pixel <= 8'd0;
        bin_pixel <= 1'b0;
    end else begin
        hs_reg <= hsync_now;
        vs_reg <= vsync_now;
        de_reg <= video_active;
        hs_reg_d0 <= hs_reg;
        vs_reg_d0 <= vs_reg;
        de_reg_d0 <= de_reg;
        display_rd_addr <= video_active ? disp_addr : 14'd0;
        rgb_pixel <= rgb_mem[display_rd_addr];
        gray_pixel <= gray_mem[display_rd_addr];
        gauss_pixel <= gauss_mem[display_rd_addr];
        median_pixel <= median_mem[display_rd_addr];
        alg_sobel_pixel <= alg_sobel_mem[display_rd_addr];
        bin_pixel <= bin_mem[display_rd_addr];
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
                // Read 4-bit mode control value. Valid GUI/SDK modes are 0~9.
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
                // Read threshold control value, low 8 bits only.
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
                // Read overlay enable control value, bit0 only.
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
// BRAM image cache writeback
// The original RGB pixels from PS BRAM are cached for display and for channel-only modes.
always @(posedge clk) begin
    if (scan_valid_d2) begin
        rgb_mem[scan_store_addr] <= bram_dout[23:0];
    end
end


// -----------------------------------------------------------------------------
// Cache outputs of the kept algorithms.
// Removed cache write logic for old unused modes.
// -----------------------------------------------------------------------------
always @(posedge clk) begin
    if (rst || scan_frame_start) begin
        gray_wr_addr       <= 14'd0;
        gauss_wr_addr      <= 14'd0;
        median_wr_addr     <= 14'd0;
        alg_sobel_wr_addr  <= 14'd0;
        bin_wr_addr        <= 14'd0;
    end else begin

        if (y_de && (gray_wr_addr < 14'd9216)) begin
            gray_mem[gray_wr_addr] <= y_data;
            gray_wr_addr <= gray_wr_addr + 14'd1;
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
    end
end

endmodule
