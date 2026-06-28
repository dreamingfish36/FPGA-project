`timescale 1ns / 1ps

// ============================================================
// ov5640_ram_hdmi_640_alg_control.v
// S05-640-TOP
//
// 作用：
// 1) 保留原 OV5640 -> RAM -> HDMI 的摄像头显示链路；
// 2) 把摄像头路径整理成 640x480 RGB888 视频流；
// 3) 插入 source_select MUX + 16 模式实时算法模块；
// 4) 通过 PS/BRAM 控制字接收 GUI 发来的：
//    mode / threshold / overlay / source_select；
// 5) source_select=0：显示 GUI UART 发来的 128x72 图像，放大到 640x360 居中；
//    source_select=1：显示 OV5640 摄像头 640x480 实时算法结果。
//
// 注意：这版是 640x480 方案，不再使用 1080P 参数。
// ============================================================

module ov5640_ram_hdmi_640_alg_control
(
    input  wire       sys_clk,
    input  wire       sys_rst_n,

    // CMOS / OV5640 Interface
    input  wire [7:0] ov5640_data,
    input  wire       ov5640_vsync,
    input  wire       ov5640_href,
    input  wire       ov5640_pclk,
    output wire       ov5640_rst_n,
    output wire       ov5640_pwdn,
    output wire       ov5640_xclk,
    output wire       sccb_scl,
    inout  wire       sccb_sda,

    // PS/BRAM read port for GUI framebuffer and control words
    // 连接到 ZYNQ/AXI BRAM Controller 的 PL 读口。
    output wire        bram_clk,
    output wire        bram_rst,
    output wire        bram_en,
    output wire [3:0]  bram_we,
    output wire [31:0] bram_addr,
    output wire [31:0] bram_din,
    input  wire [31:0] bram_dout,

    // HDMI Interface
    output wire       tmds_clk_p,
    output wire       tmds_clk_n,
    output wire [2:0] tmds_data_p,
    output wire [2:0] tmds_data_n
);

    // ========================================================
    // S05-480-TOP-A：480x360 图像缓存参数
    // 为了降低 BRAM 占用，不再缓存完整 640x480 一帧。
    // 当前方案只缓存 OV5640 输出画面的前 480x360 区域，
    // HDMI 仍然保持 640x480 输出，并把 480x360 摄像头图像居中显示。
    // 资源估算：480*360*16bit ≈ 2.76Mbit，比 640*480*16bit 明显更省 BRAM。
    // ========================================================
    localparam RAM_WIDTH       = 480;
    localparam RAM_HEIGHT      = 360;
    localparam RAM_DEPTH       = 172800;  // 480 * 360
    localparam ADDR_WIDTH      = 18;      // 2^18 = 262144 > 172800
    localparam H_CENTER_START  = 80;      // (640 - 480) / 2
    localparam V_CENTER_START  = 60;      // (480 - 360) / 2

    wire [1:0] sync_sigs;
    wire [9:0] tmds_red, tmds_green, tmds_blue;
    wire tmds_shift_red0, tmds_shift_green0, tmds_shift_blue0;

    wire clk_25m_cam, clk_25m_vid, clk_250m, clk_125m;
    wire locked2;
    wire rst_n;

    wire [15:0] image_data;
    wire        image_data_en;
    wire        cfg_done;
    wire [15:0] ram_q;

    wire [31:0] h_addr_32, v_addr_32;
    wire [11:0] h_addr, v_addr;
    wire        vga_de;

    wire        alg_hs;
    wire        alg_vs;
    wire        alg_de;
    wire [7:0]  alg_r;
    wire [7:0]  alg_g;
    wire [7:0]  alg_b;
    wire [1:0]  alg_sync_sigs;

    assign ov5640_pwdn  = 1'b0;
    assign ov5640_rst_n = 1'b1;
    assign ov5640_xclk  = clk_25m_cam;
    assign rst_n        = sys_rst_n;
    assign h_addr       = h_addr_32[11:0];
    assign v_addr       = v_addr_32[11:0];
    assign alg_sync_sigs = {alg_vs, alg_hs};

    // ========================================================
    // S05-640-TOP-B：时钟
    // 640x480 HDMI 使用 25MHz pixel clock，TMDS 使用 250MHz。
    // ========================================================
    clk_wiz_0_cg clk_wiz_inst (
        .resetn  (sys_rst_n),
        .clk_in1 (sys_clk),
        .clk_25  (clk_25m_vid),
        .clk_250 (clk_250m),
        .clk_125 (clk_125m),
        .locked  (locked2)
    );

    assign clk_25m_cam = clk_25m_vid;

    // ========================================================
    // S05-640-TOP-C：把 OV5640 的 PCLK 域信号同步到 125MHz 采样域
    // ========================================================
    (* ASYNC_REG = "TRUE" *) reg [3:0] pclk_shifter;
    (* ASYNC_REG = "TRUE" *) reg [7:0] data_r1,  data_r2,  data_r3;
    (* ASYNC_REG = "TRUE" *) reg       href_r1,  href_r2,  href_r3;
    (* ASYNC_REG = "TRUE" *) reg       vsync_r1, vsync_r2, vsync_r3;

    always @(posedge clk_125m) begin
        pclk_shifter <= {pclk_shifter[2:0], ov5640_pclk};
        data_r1      <= ov5640_data;
        data_r2      <= data_r1;
        data_r3      <= data_r2;
        href_r1      <= ov5640_href;
        href_r2      <= href_r1;
        href_r3      <= href_r2;
        vsync_r1     <= ov5640_vsync;
        vsync_r2     <= vsync_r1;
        vsync_r3     <= vsync_r2;
    end

    wire pclk_edge  = (pclk_shifter[2:1] == 2'b01);
    wire sync_href  = href_r3;
    wire sync_vsync = vsync_r3;
    wire [7:0] sync_data = data_r3;

    ov5640_top ov5640_top_inst (
        .sys_clk         (clk_25m_cam),
        .sys_rst_n       (rst_n),
        .sys_init_done   (cfg_done),
        .sample_clk      (clk_125m),
        .pclk_edge       (pclk_edge),
        .ov5640_href     (sync_href),
        .ov5640_vsync    (sync_vsync),
        .ov5640_data     (sync_data),
        .cfg_done        (cfg_done),
        .sccb_scl        (sccb_scl),
        .sccb_sda        (sccb_sda),
        .ov5640_wr_en    (image_data_en),
        .ov5640_data_out (image_data)
    );

    // ========================================================
    // S05-480-TOP-D：摄像头写 RAM 地址生成，只保存 480x360，降低 BRAM
    // ========================================================
    reg [11:0] cam_h_cnt;
    reg [11:0] cam_v_cnt;
    reg        sync_href_d1;

    always @(posedge clk_125m or negedge rst_n) begin
        if (!rst_n) begin
            cam_h_cnt    <= 12'd0;
            cam_v_cnt    <= 12'd0;
            sync_href_d1 <= 1'b0;
        end else if (sync_vsync) begin
            cam_h_cnt    <= 12'd0;
            cam_v_cnt    <= 12'd0;
            sync_href_d1 <= 1'b0;
        end else if (pclk_edge) begin
            sync_href_d1 <= sync_href;
            if (sync_href && !sync_href_d1) begin
                cam_h_cnt <= 12'd0;
            end else if (!sync_href && sync_href_d1) begin
                cam_v_cnt <= cam_v_cnt + 1'b1;
            end else if (image_data_en) begin
                cam_h_cnt <= cam_h_cnt + 1'b1;
            end
        end
    end

    reg [ADDR_WIDTH-1:0] wr_addr_step1;
    reg [11:0]           wr_h_step1;
    reg [15:0]           wr_data_step1;
    reg [1:0]            wr_en_pipe;
    reg [ADDR_WIDTH-1:0] wr_addr_final;
    reg [15:0]           wr_data_final;

    always @(posedge clk_125m) begin
        if (pclk_edge) begin
            // 480 * V = V*256 + V*128 + V*64 + V*32
            wr_addr_step1 <= (cam_v_cnt << 8) + (cam_v_cnt << 7) +
                             (cam_v_cnt << 6) + (cam_v_cnt << 5);
            wr_h_step1    <= cam_h_cnt;
            wr_data_step1 <= image_data;
            wr_en_pipe[0] <= image_data_en && (cam_h_cnt < RAM_WIDTH) && (cam_v_cnt < RAM_HEIGHT);

            wr_addr_final <= wr_addr_step1 + wr_h_step1;
            wr_data_final <= wr_data_step1;
            wr_en_pipe[1] <= wr_en_pipe[0];
        end else begin
            wr_en_pipe[1] <= 1'b0;
        end
    end

    // ========================================================
    // S05-640-TOP-E：HDMI 时序与 RAM 读地址
    // ========================================================
    HDMI_SYNC_cg HDMI_SYNC1 (
        .pixclk   (clk_25m_vid),
        .rstn     (rst_n),
        .CounterX (h_addr_32),
        .CounterY (v_addr_32),
        .Sync     (sync_sigs),
        .DrawArea (vga_de)
    );

    localparam READ_LATENCY = 6;
    wire [11:0] pre_h_addr = h_addr + READ_LATENCY;

    // 480x360 摄像头图像在 640x480 HDMI 有效区中居中显示。
    // 读 RAM 时提前 READ_LATENCY 个像素取数，以补偿 BRAM 读延迟。
    wire pre_disp_valid = (pre_h_addr >= H_CENTER_START) &&
                          (pre_h_addr <  H_CENTER_START + RAM_WIDTH) &&
                          (v_addr     >= V_CENTER_START) &&
                          (v_addr     <  V_CENTER_START + RAM_HEIGHT);

    wire [11:0] rel_h = (pre_h_addr >= H_CENTER_START) ? (pre_h_addr - H_CENTER_START) : 12'd0;
    wire [11:0] rel_v = (v_addr     >= V_CENTER_START) ? (v_addr     - V_CENTER_START) : 12'd0;

    reg [ADDR_WIDTH-1:0] rd_addr_step1;
    reg [11:0]           rd_h_step1;
    reg [ADDR_WIDTH-1:0] rd_addr_final;

    always @(posedge clk_25m_vid) begin
        // 480 * V = V*256 + V*128 + V*64 + V*32
        rd_addr_step1 <= (rel_v << 8) + (rel_v << 7) +
                         (rel_v << 6) + (rel_v << 5);
        rd_h_step1    <= rel_h;
        rd_addr_final <= rd_addr_step1 + rd_h_step1;
    end

    ram #(
        .DEPTH      (RAM_DEPTH),
        .ADDR_WIDTH (ADDR_WIDTH)
    ) ram_inst (
        .clka  (clk_125m),
        .wea   (wr_en_pipe[1]),
        .addra (wr_addr_final),
        .dina  (wr_data_final),
        .clkb  (clk_25m_vid),
        .addrb (rd_addr_final),
        .doutb (ram_q)
    );

    // 摄像头 RAM 输出从 RGB565 转 RGB888。
    // 注意：HDMI 的 de 仍保持整个 640x480 有效区，
    // 只是 480x360 之外输出黑色边框，避免在有效区中断 DE 导致显示异常。
    wire cam_window_now = vga_de &&
                          (h_addr >= H_CENTER_START) &&
                          (h_addr <  H_CENTER_START + RAM_WIDTH) &&
                          (v_addr >= V_CENTER_START) &&
                          (v_addr <  V_CENTER_START + RAM_HEIGHT);

    wire [7:0] cam_r = cam_window_now ? {ram_q[15:11], 3'd0} : 8'd0;
    wire [7:0] cam_g = cam_window_now ? {ram_q[10:5],  2'd0} : 8'd0;
    wire [7:0] cam_b = cam_window_now ? {ram_q[4:0],   3'd0} : 8'd0;

    // ========================================================
    // S05-640-TOP-F：图像来源选择 + 16 模式实时算法处理
    // ========================================================
    camera_gui_source_select_core_640 #(
        .H_ACTIVE (640),
        .V_ACTIVE (480)
    ) u_camera_gui_source_select_core_640 (
        .video_clk          (clk_25m_vid),
        .rst                (~rst_n),

        .cam_hs             (sync_sigs[0]),
        .cam_vs             (sync_sigs[1]),
        .cam_de             (vga_de),
        .cam_r              (cam_r),
        .cam_g              (cam_g),
        .cam_b              (cam_b),

        .h_cnt              (h_addr),
        .v_cnt              (v_addr),

        .bram_clk           (bram_clk),
        .bram_rst           (bram_rst),
        .bram_en            (bram_en),
        .bram_we            (bram_we),
        .bram_addr          (bram_addr),
        .bram_din           (bram_din),
        .bram_dout          (bram_dout),

        .out_hs             (alg_hs),
        .out_vs             (alg_vs),
        .out_de             (alg_de),
        .out_r              (alg_r),
        .out_g              (alg_g),
        .out_b              (alg_b),

        .dbg_display_mode   (),
        .dbg_threshold      (),
        .dbg_overlay_enable (),
        .dbg_source_select  ()
    );

    // ========================================================
    // S05-640-TOP-G：HDMI TMDS 编码输出
    // ========================================================
    TMDS_encoder_cg TMDS_encoder_b (
        .clk  (clk_25m_vid),
        .VD   (alg_b),
        .VDE  (alg_de),
        .CD   (alg_sync_sigs),
        .TMDS (tmds_blue)
    );

    TMDS_encoder_cg TMDS_encoder_g (
        .clk  (clk_25m_vid),
        .VD   (alg_g),
        .VDE  (alg_de),
        .CD   (alg_sync_sigs),
        .TMDS (tmds_green)
    );

    TMDS_encoder_cg TMDS_encoder_r (
        .clk  (clk_25m_vid),
        .VD   (alg_r),
        .VDE  (alg_de),
        .CD   (alg_sync_sigs),
        .TMDS (tmds_red)
    );

    HDMI_TMDS_cg HDMI_TMDS_inst (
        .clk_TMDS         (clk_250m),
        .rstn             (rst_n),
        .TMDS_red         (tmds_red),
        .TMDS_green       (tmds_green),
        .TMDS_blue        (tmds_blue),
        .TMDS_shift_red0  (tmds_shift_red0),
        .TMDS_shift_green0(tmds_shift_green0),
        .TMDS_shift_blue0 (tmds_shift_blue0)
    );

    OBUFDS #(.IOSTANDARD("TMDS_33")) TMDS_CLK_BUF (
        .I(clk_25m_vid), .O(tmds_clk_p), .OB(tmds_clk_n)
    );

    OBUFDS #(.IOSTANDARD("TMDS_33")) TMDS_B_BUF (
        .I(tmds_shift_blue0), .O(tmds_data_p[0]), .OB(tmds_data_n[0])
    );

    OBUFDS #(.IOSTANDARD("TMDS_33")) TMDS_G_BUF (
        .I(tmds_shift_green0), .O(tmds_data_p[1]), .OB(tmds_data_n[1])
    );

    OBUFDS #(.IOSTANDARD("TMDS_33")) TMDS_R_BUF (
        .I(tmds_shift_red0), .O(tmds_data_p[2]), .OB(tmds_data_n[2])
    );

endmodule
