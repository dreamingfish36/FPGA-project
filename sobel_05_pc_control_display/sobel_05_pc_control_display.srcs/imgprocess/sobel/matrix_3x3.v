`timescale 1ns / 1ps
// ============================================================
// fifo_matrix_buf.v
// ------------------------------------------------------------
// Purpose:
//   Pure Verilog replacement for the original Pango DRM Based FIFO
//   instance named "fifo_matrix_buf".
//
// Why this file is needed:
//   The original matrix_3x3.v instantiates:
//
//       fifo_matrix_buf fifo_matrix_buf1 (...);
//       fifo_matrix_buf fifo_matrix_buf2 (...);
//
//   In Vivado, if you do not want to generate a FIFO IP core, add
//   this file to Design Sources. Do NOT change the ports in
//   matrix_3x3.v.
//
// Interface compatibility:
//   This wrapper keeps the same port names used by the Pango FIFO:
//       wr_clk, wr_rst, wr_en, wr_data, wr_full, almost_full,
//       rd_clk, rd_rst, rd_en, rd_data, rd_empty, almost_empty
//
// Default configuration:
//       DATA_WIDTH = 8
//       ADDR_WIDTH = 11
//       DEPTH      = 2048
//
// Notes:
//   1. wr_rst and rd_rst are active-high resets, matching the way
//      matrix_3x3.v connects them: .wr_rst(~rst_n), .rd_rst(~rst_n)
//   2. This is a synthesizable dual-clock asynchronous FIFO using
//      Gray-code pointers.
//   3. If wr_clk and rd_clk are both video_clk, it still works normally.
// ============================================================

module fifo_matrix_buf #(
    parameter integer DATA_WIDTH        = 8,
    parameter integer ADDR_WIDTH        = 11,
    parameter integer ALMOST_FULL_NUM   = 1020,
    parameter integer ALMOST_EMPTY_NUM  = 4
)(
    input  wire                   wr_clk,
    input  wire                   wr_rst,
    input  wire                   wr_en,
    input  wire [DATA_WIDTH-1:0]  wr_data,
    output wire                   wr_full,
    output wire                   almost_full,

    input  wire                   rd_clk,
    input  wire                   rd_rst,
    input  wire                   rd_en,
    output reg  [DATA_WIDTH-1:0]  rd_data,
    output wire                   rd_empty,
    output wire                   almost_empty
);

    localparam integer DEPTH     = (1 << ADDR_WIDTH);
    localparam integer PTR_WIDTH = ADDR_WIDTH + 1;

    // ========================================================
    // Internal memory.
    // Vivado can infer distributed RAM or block RAM from this array.
    // ========================================================
    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];

    // Binary and Gray pointers.
    reg [PTR_WIDTH-1:0] wr_bin;
    reg [PTR_WIDTH-1:0] wr_gray;
    reg [PTR_WIDTH-1:0] rd_bin;
    reg [PTR_WIDTH-1:0] rd_gray;

    // Cross-clock pointer synchronizers.
    reg [PTR_WIDTH-1:0] rd_gray_sync_w1;
    reg [PTR_WIDTH-1:0] rd_gray_sync_w2;
    reg [PTR_WIDTH-1:0] wr_gray_sync_r1;
    reg [PTR_WIDTH-1:0] wr_gray_sync_r2;

    // Registered flag outputs.
    reg wr_full_r;
    reg rd_empty_r;
    reg almost_full_r;
    reg almost_empty_r;

    assign wr_full      = wr_full_r;
    assign rd_empty     = rd_empty_r;
    assign almost_full  = almost_full_r;
    assign almost_empty = almost_empty_r;

    // ========================================================
    // Utility functions.
    // ========================================================
    function [PTR_WIDTH-1:0] bin2gray;
        input [PTR_WIDTH-1:0] bin;
        begin
            bin2gray = (bin >> 1) ^ bin;
        end
    endfunction

    function [PTR_WIDTH-1:0] gray2bin;
        input [PTR_WIDTH-1:0] gray;
        integer i;
        begin
            gray2bin[PTR_WIDTH-1] = gray[PTR_WIDTH-1];
            for (i = PTR_WIDTH-2; i >= 0; i = i - 1) begin
                gray2bin[i] = gray2bin[i+1] ^ gray[i];
            end
        end
    endfunction

    // ========================================================
    // Synchronize read pointer into write clock domain.
    // ========================================================
    always @(posedge wr_clk or posedge wr_rst) begin
        if (wr_rst) begin
            rd_gray_sync_w1 <= {PTR_WIDTH{1'b0}};
            rd_gray_sync_w2 <= {PTR_WIDTH{1'b0}};
        end else begin
            rd_gray_sync_w1 <= rd_gray;
            rd_gray_sync_w2 <= rd_gray_sync_w1;
        end
    end

    // ========================================================
    // Synchronize write pointer into read clock domain.
    // ========================================================
    always @(posedge rd_clk or posedge rd_rst) begin
        if (rd_rst) begin
            wr_gray_sync_r1 <= {PTR_WIDTH{1'b0}};
            wr_gray_sync_r2 <= {PTR_WIDTH{1'b0}};
        end else begin
            wr_gray_sync_r1 <= wr_gray;
            wr_gray_sync_r2 <= wr_gray_sync_r1;
        end
    end

    // ========================================================
    // Write-side logic.
    // ========================================================
    wire wr_do;
    assign wr_do = wr_en && !wr_full_r;

    wire [PTR_WIDTH-1:0] wr_bin_next;
    wire [PTR_WIDTH-1:0] wr_gray_next;

    assign wr_bin_next  = wr_bin + {{(PTR_WIDTH-1){1'b0}}, wr_do};
    assign wr_gray_next = bin2gray(wr_bin_next);

    wire wr_full_next;
    assign wr_full_next =
        (wr_gray_next == {~rd_gray_sync_w2[PTR_WIDTH-1:PTR_WIDTH-2],
                           rd_gray_sync_w2[PTR_WIDTH-3:0]});

    wire [PTR_WIDTH-1:0] rd_bin_sync_w;
    wire [PTR_WIDTH-1:0] wr_used_count;

    assign rd_bin_sync_w = gray2bin(rd_gray_sync_w2);
    assign wr_used_count = wr_bin_next - rd_bin_sync_w;

    always @(posedge wr_clk or posedge wr_rst) begin
        if (wr_rst) begin
            wr_bin        <= {PTR_WIDTH{1'b0}};
            wr_gray       <= {PTR_WIDTH{1'b0}};
            wr_full_r     <= 1'b0;
            almost_full_r <= 1'b0;
        end else begin
            if (wr_do) begin
                mem[wr_bin[ADDR_WIDTH-1:0]] <= wr_data;
            end

            wr_bin    <= wr_bin_next;
            wr_gray   <= wr_gray_next;
            wr_full_r <= wr_full_next;

            if (wr_used_count >= ALMOST_FULL_NUM)
                almost_full_r <= 1'b1;
            else
                almost_full_r <= 1'b0;
        end
    end

    // ========================================================
    // Read-side logic.
    // Standard FIFO style:
    //   rd_data is updated on rd_clk when rd_en && !rd_empty.
    // ========================================================
    wire rd_do;
    assign rd_do = rd_en && !rd_empty_r;

    wire [PTR_WIDTH-1:0] rd_bin_next;
    wire [PTR_WIDTH-1:0] rd_gray_next;

    assign rd_bin_next  = rd_bin + {{(PTR_WIDTH-1){1'b0}}, rd_do};
    assign rd_gray_next = bin2gray(rd_bin_next);

    wire rd_empty_next;
    assign rd_empty_next = (rd_gray_next == wr_gray_sync_r2);

    wire [PTR_WIDTH-1:0] wr_bin_sync_r;
    wire [PTR_WIDTH-1:0] rd_used_count;

    assign wr_bin_sync_r = gray2bin(wr_gray_sync_r2);
    assign rd_used_count = wr_bin_sync_r - rd_bin_next;

    always @(posedge rd_clk or posedge rd_rst) begin
        if (rd_rst) begin
            rd_bin         <= {PTR_WIDTH{1'b0}};
            rd_gray        <= {PTR_WIDTH{1'b0}};
            rd_data        <= {DATA_WIDTH{1'b0}};
            rd_empty_r     <= 1'b1;
            almost_empty_r <= 1'b1;
        end else begin
            if (rd_do) begin
                rd_data <= mem[rd_bin[ADDR_WIDTH-1:0]];
            end

            rd_bin     <= rd_bin_next;
            rd_gray    <= rd_gray_next;
            rd_empty_r <= rd_empty_next;

            if (rd_used_count <= ALMOST_EMPTY_NUM)
                almost_empty_r <= 1'b1;
            else
                almost_empty_r <= 1'b0;
        end
    end

endmodule
