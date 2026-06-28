// ============================================================
// top_640_source_select_example.v
// 说明：这是把 ZYNQ PS/BRAM 控制系统和 OV5640 640x480 实时算法 HDMI 输出
// 连接在一起的参考顶层。
//
// 如果你的工程顶层端口名不同，不要直接硬替换；按这里的连接关系
// 把 bram_* 接到 ov5640_ram_hdmi_640_alg_control 即可。
// ============================================================

module top(
    inout [14:0] DDR_addr,
    inout [2:0] DDR_ba,
    inout DDR_cas_n,
    inout DDR_ck_n,
    inout DDR_ck_p,
    inout DDR_cke,
    inout DDR_cs_n,
    inout [3:0] DDR_dm,
    inout [31:0] DDR_dq,
    inout [3:0] DDR_dqs_n,
    inout [3:0] DDR_dqs_p,
    inout DDR_odt,
    inout DDR_ras_n,
    inout DDR_reset_n,
    inout DDR_we_n,
    inout FIXED_IO_ddr_vrn,
    inout FIXED_IO_ddr_vrp,
    inout [53:0] FIXED_IO_mio,
    inout FIXED_IO_ps_clk,
    inout FIXED_IO_ps_porb,
    inout FIXED_IO_ps_srstb,

    input wire sys_clk,
    input wire sys_rst_n,

    input  wire [7:0] ov5640_data,
    input  wire       ov5640_vsync,
    input  wire       ov5640_href,
    input  wire       ov5640_pclk,
    output wire       ov5640_rst_n,
    output wire       ov5640_pwdn,
    output wire       ov5640_xclk,
    output wire       sccb_scl,
    inout  wire       sccb_sda,

    output wire hdmi_oen,
    output wire TMDS_clk_n,
    output wire TMDS_clk_p,
    output wire [2:0] TMDS_data_n,
    output wire [2:0] TMDS_data_p
);

    wire bram_clk;
    wire bram_rst;
    wire bram_en;
    wire [3:0] bram_we;
    wire [31:0] bram_addr;
    wire [31:0] bram_din;
    wire [31:0] bram_dout;

    assign hdmi_oen = 1'b0;

    // PS 仍然负责：
    // 1) 接收 GUI UART 图像并写入 BRAM framebuffer；
    // 2) 接收 GUI 控制命令并写入 0x9000/0x9004/0x9008/0x900C。
    ps_uart_bram_hdmi_wrapper ps_uart_bram_hdmi_wrapper_i (
        .BRAM_PORTB_addr(bram_addr),
        .BRAM_PORTB_clk(bram_clk),
        .BRAM_PORTB_din(bram_din),
        .BRAM_PORTB_dout(bram_dout),
        .BRAM_PORTB_en(bram_en),
        .BRAM_PORTB_rst(bram_rst),
        .BRAM_PORTB_we(bram_we),
        .DDR_addr(DDR_addr),
        .DDR_ba(DDR_ba),
        .DDR_cas_n(DDR_cas_n),
        .DDR_ck_n(DDR_ck_n),
        .DDR_ck_p(DDR_ck_p),
        .DDR_cke(DDR_cke),
        .DDR_cs_n(DDR_cs_n),
        .DDR_dm(DDR_dm),
        .DDR_dq(DDR_dq),
        .DDR_dqs_n(DDR_dqs_n),
        .DDR_dqs_p(DDR_dqs_p),
        .DDR_odt(DDR_odt),
        .DDR_ras_n(DDR_ras_n),
        .DDR_reset_n(DDR_reset_n),
        .DDR_we_n(DDR_we_n),
        .FIXED_IO_ddr_vrn(FIXED_IO_ddr_vrn),
        .FIXED_IO_ddr_vrp(FIXED_IO_ddr_vrp),
        .FIXED_IO_mio(FIXED_IO_mio),
        .FIXED_IO_ps_clk(FIXED_IO_ps_clk),
        .FIXED_IO_ps_porb(FIXED_IO_ps_porb),
        .FIXED_IO_ps_srstb(FIXED_IO_ps_srstb)
    );

    // 统一 HDMI 输出：
    // source_select=0 -> GUI UART image
    // source_select=1 -> OV5640 camera 640x480
    ov5640_ram_hdmi_640_alg_control u_ov5640_ram_hdmi_640_alg_control (
        .sys_clk      (sys_clk),
        .sys_rst_n    (sys_rst_n),
        .ov5640_data  (ov5640_data),
        .ov5640_vsync (ov5640_vsync),
        .ov5640_href  (ov5640_href),
        .ov5640_pclk  (ov5640_pclk),
        .ov5640_rst_n (ov5640_rst_n),
        .ov5640_pwdn  (ov5640_pwdn),
        .ov5640_xclk  (ov5640_xclk),
        .sccb_scl     (sccb_scl),
        .sccb_sda     (sccb_sda),
        .bram_clk     (bram_clk),
        .bram_rst     (bram_rst),
        .bram_en      (bram_en),
        .bram_we      (bram_we),
        .bram_addr    (bram_addr),
        .bram_din     (bram_din),
        .bram_dout    (bram_dout),
        .tmds_clk_p   (TMDS_clk_p),
        .tmds_clk_n   (TMDS_clk_n),
        .tmds_data_p  (TMDS_data_p),
        .tmds_data_n  (TMDS_data_n)
    );

endmodule
