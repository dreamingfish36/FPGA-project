module brightness(
    input   wire            clk,
    input   wire            rst_n,
    input   wire  [23:0]    data_in,//杈撳叆鏁版嵁
    input   wire  [8:0]     im_gain,//浜害鍙樻崲鍙傛暟(鍙寜閿皟鑺?)
    input   wire            in_de,//杈撳叆鏈夋晥淇″彿
    input   wire            pre_vs,
    input   wire            pre_hs,
    
    
    output  reg             pos_de,
    output  reg             pos_vs,
    output  reg             pos_hs,
    output  wire   [7:0]    out_r,
    output  wire   [7:0]    out_g,
    output  wire   [7:0]    out_b
);

wire [8:0] reg_r;
wire [8:0] reg_g;
wire [8:0] reg_b;

assign reg_r=data_in[23:16]+im_gain;
assign reg_g=data_in[15:8]+im_gain;
assign reg_b=data_in[7:0]+im_gain;

reg [7:0] reg_r_1;
reg [7:0] reg_g_1;
reg [7:0] reg_b_1;



always@(posedge clk or negedge rst_n)begin
    if(!rst_n)
        reg_r_1   <= 0;
    else if(in_de && reg_r>=255)
        reg_r_1   <= 255;
    else if(in_de)
        reg_r_1   <= reg_r;
end

always@(posedge clk or negedge rst_n)begin
    if(!rst_n)
        reg_g_1   <= 0;
    else if(in_de && reg_g>=255)
        reg_g_1   <= 255;
    else if(in_de)
        reg_g_1   <= reg_g;
end

always@(posedge clk or negedge rst_n)begin
    if(!rst_n)
        reg_b_1   <= 0;
    else if(in_de && reg_b>=255)
        reg_b_1   <= 255;
    else if(in_de)
        reg_b_1   <= reg_b;
end

assign out_r=reg_r_1;
assign out_g=reg_g_1;
assign out_b=reg_b_1;

always@(posedge clk or negedge rst_n)begin
    if(!rst_n)
        pos_vs <= 0;
    else
        pos_vs <= pre_vs;
    
end

always@(posedge clk or negedge rst_n)begin
    if(!rst_n)
        pos_de <= 0;
    else
        pos_de <= in_de;
    
end

always@(posedge clk or negedge rst_n)begin
    if(!rst_n)
        pos_hs <= 0;
    else
        pos_hs <= pre_hs;
    
end

endmodule