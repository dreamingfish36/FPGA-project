`timescale 1ns / 1ps
// ============================================================
// fifo_matrix_buf_1bit.v
// ------------------------------------------------------------
// Pure Verilog replacement for the Pango DRM Based FIFO used by
// matrix_3x3_1bit.v.
//
// Purpose:
//   Keep the original module name and port names unchanged, so
//   matrix_3x3_1bit.v does NOT need a Vivado FIFO Generator IP.
//
// Original instance in matrix_3x3_1bit.v:
//   fifo_matrix_buf_1bit fifo_matrix_buf1 (...);
//   fifo_matrix_buf_1bit fifo_matrix_buf2 (...);
//
// Data width : 1 bit
// Depth      : 2048 words
// Addr width : 11 bits
// FIFO type  : asynchronous-style FIFO with independent wr_clk/rd_clk
// Reset      : wr_rst / rd_rst are active high
//
// Notes:
//   1. No Xilinx IP core is required.
//   2. Do not rename this module.
//   3. Do not change the port names if you want it to match the
//      original Pango FIFO instance.
// ============================================================

module fifo_matrix_buf_1bit (
    input  wire       wr_clk,
    input  wire       wr_rst,
    input  wire       wr_en,
    input  wire       wr_data,
    output wire       wr_full,
    output wire       almost_full,

    input  wire       rd_clk,
    input  wire       rd_rst,
    input  wire       rd_en,
    output reg        rd_data,
    output wire       rd_empty,
    output wire       almost_empty
);

    // ========================================================
    // FIFO parameter area
    // If your original Pango FIFO used different depth or flags,
    // only change these parameters.
    // ========================================================
    localparam integer DATA_WIDTH          = 1;
    localparam integer ADDR_WIDTH          = 11;
    localparam integer FIFO_DEPTH          = (1 << ADDR_WIDTH);  // 2048
    localparam integer ALMOST_FULL_LEVEL   = 1020;
    localparam integer ALMOST_EMPTY_LEVEL  = 4;

    // Memory array
    reg [DATA_WIDTH-1:0] mem [0:FIFO_DEPTH-1];

    // Binary and Gray pointers.
    // One extra MSB is used to distinguish full from empty.
    reg [ADDR_WIDTH:0] wr_bin;
    reg [ADDR_WIDTH:0] wr_gray;
    reg [ADDR_WIDTH:0] rd_bin;
    reg [ADDR_WIDTH:0] rd_gray;

    // Synchronized pointers across clock domains
    reg [ADDR_WIDTH:0] rd_gray_sync1;
    reg [ADDR_WIDTH:0] rd_gray_sync2;
    reg [ADDR_WIDTH:0] wr_gray_sync1;
    reg [ADDR_WIDTH:0] wr_gray_sync2;

    wire [ADDR_WIDTH:0] wr_bin_next;
    wire [ADDR_WIDTH:0] rd_bin_next;
    wire [ADDR_WIDTH:0] wr_gray_next;
    wire [ADDR_WIDTH:0] rd_gray_next;

    wire                wr_do;
    wire                rd_do;

    wire [ADDR_WIDTH:0] rd_bin_sync;
    wire [ADDR_WIDTH:0] wr_bin_sync;

    wire [ADDR_WIDTH:0] used_words_wr;
    wire [ADDR_WIDTH:0] used_words_rd;

    // ========================================================
    // Helper functions
    // ========================================================
    function [ADDR_WIDTH:0] bin2gray;
        input [ADDR_WIDTH:0] bin;
        begin
            bin2gray = (bin >> 1) ^ bin;
        end
    endfunction

    function [ADDR_WIDTH:0] gray2bin;
        input [ADDR_WIDTH:0] gray;
        integer i;
        begin
            gray2bin[ADDR_WIDTH] = gray[ADDR_WIDTH];
            for (i = ADDR_WIDTH-1; i >= 0; i = i - 1) begin
                gray2bin[i] = gray2bin[i+1] ^ gray[i];
            end
        end
    endfunction

    // ========================================================
    // Write side
    // ========================================================
    assign wr_do        = wr_en && !wr_full;
    assign wr_bin_next  = wr_bin + {{ADDR_WIDTH{1'b0}}, wr_do};
    assign wr_gray_next = bin2gray(wr_bin_next);

    // Full when next write pointer catches synchronized read pointer
    // with inverted top two bits.
    assign wr_full = (wr_gray_next == {
                        ~rd_gray_sync2[ADDR_WIDTH:ADDR_WIDTH-1],
                         rd_gray_sync2[ADDR_WIDTH-2:0]
                     });

    always @(posedge wr_clk or posedge wr_rst) begin
        if (wr_rst) begin
            wr_bin  <= {ADDR_WIDTH+1{1'b0}};
            wr_gray <= {ADDR_WIDTH+1{1'b0}};
        end else begin
            if (wr_do) begin
                mem[wr_bin[ADDR_WIDTH-1:0]] <= wr_data;
            end
            wr_bin  <= wr_bin_next;
            wr_gray <= wr_gray_next;
        end
    end

    // Synchronize read pointer into write clock domain
    always @(posedge wr_clk or posedge wr_rst) begin
        if (wr_rst) begin
            rd_gray_sync1 <= {ADDR_WIDTH+1{1'b0}};
            rd_gray_sync2 <= {ADDR_WIDTH+1{1'b0}};
        end else begin
            rd_gray_sync1 <= rd_gray;
            rd_gray_sync2 <= rd_gray_sync1;
        end
    end

    assign rd_bin_sync  = gray2bin(rd_gray_sync2);
    assign used_words_wr = wr_bin - rd_bin_sync;
    assign almost_full   = (used_words_wr >= ALMOST_FULL_LEVEL[ADDR_WIDTH:0]);

    // ========================================================
    // Read side
    // ========================================================
    assign rd_do        = rd_en && !rd_empty;
    assign rd_bin_next  = rd_bin + {{ADDR_WIDTH{1'b0}}, rd_do};
    assign rd_gray_next = bin2gray(rd_bin_next);

    assign rd_empty = (rd_gray == wr_gray_sync2);

    always @(posedge rd_clk or posedge rd_rst) begin
        if (rd_rst) begin
            rd_bin  <= {ADDR_WIDTH+1{1'b0}};
            rd_gray <= {ADDR_WIDTH+1{1'b0}};
            rd_data <= {DATA_WIDTH{1'b0}};
        end else begin
            if (rd_do) begin
                rd_data <= mem[rd_bin[ADDR_WIDTH-1:0]];
            end
            rd_bin  <= rd_bin_next;
            rd_gray <= rd_gray_next;
        end
    end

    // Synchronize write pointer into read clock domain
    always @(posedge rd_clk or posedge rd_rst) begin
        if (rd_rst) begin
            wr_gray_sync1 <= {ADDR_WIDTH+1{1'b0}};
            wr_gray_sync2 <= {ADDR_WIDTH+1{1'b0}};
        end else begin
            wr_gray_sync1 <= wr_gray;
            wr_gray_sync2 <= wr_gray_sync1;
        end
    end

    assign wr_bin_sync   = gray2bin(wr_gray_sync2);
    assign used_words_rd = wr_bin_sync - rd_bin;
    assign almost_empty  = (used_words_rd <= ALMOST_EMPTY_LEVEL[ADDR_WIDTH:0]);

endmodule
