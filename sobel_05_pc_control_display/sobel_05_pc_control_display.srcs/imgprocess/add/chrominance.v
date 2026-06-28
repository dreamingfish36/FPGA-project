`timescale 1ns / 1ps


module chrominance#(
					parameter R=3
	)
	(
					input			clk,
					input			rst_n,
					input			per_frame_vsync,
					input			per_frame_de,
                    input			per_frame_hs,
					input	[23:0]	per_frame_data,
					output			post_frame_vsync,
					output			post_frame_de,
                    output			post_frame_hs,
					output	[23:0]	post_frame_data

    );

reg [15:0]r_d0;
reg [15:0]g_d0;
reg [15:0]b_d0;

reg [15:0]r_d1;
reg [15:0]g_d1;
reg [15:0]b_d1;

reg [15:0]r_d2;
reg [15:0]g_d2;
reg [15:0]b_d2;

reg [15:0]r_d3;
reg [15:0]g_d3;
reg [15:0]b_d3;




always @(posedge clk or negedge rst_n) begin
	if(~rst_n) begin
		r_d0<=0;
		g_d0<=0;
		b_d0<=0;
	end else begin
		 r_d0<=	per_frame_data[23:16]+per_frame_data[23:16]*R;
		 g_d0<= per_frame_data[15: 8]+per_frame_data[15: 8]*R;
		 b_d0<= per_frame_data[7 : 0]+per_frame_data[7 : 0]*R;
	end
end



always @(posedge clk or negedge rst_n) begin
	if(~rst_n) begin
		r_d1<=0;
		g_d1<=0;
		b_d1<=0;
	end else begin
		 r_d1<=(per_frame_data[15:8]+per_frame_data[7:0])>>1	;
		 g_d1<=(per_frame_data[23:16]+per_frame_data[7:0])>>1	;
		 b_d1<=(per_frame_data[23:16]+per_frame_data[15:8]) >>1;
	end
end


always @(posedge clk or negedge rst_n) begin
	if(~rst_n) begin
		r_d2<=0;
	end else if(r_d0>=r_d1)begin
		r_d2<=r_d0-r_d1;
	end else
		r_d2<=r_d1-r_d0;
end

always @(posedge clk or negedge rst_n) begin
	if(~rst_n) begin
		g_d2<=0;
	end else if(g_d0>=g_d1)begin
		g_d2<=g_d0-g_d1;
	end else
		g_d2<=g_d1-g_d0;
end


always @(posedge clk or negedge rst_n) begin
	if(~rst_n) begin
		b_d2<=0;
	end else if(b_d0>=b_d1)begin
		b_d2<=b_d0-b_d1;
	end else
		b_d2<=b_d1-b_d0;
end


always @(posedge clk or negedge rst_n) begin
	if(~rst_n) begin
		r_d3<=0;
		g_d3<=0;
		b_d3<=0;
	end else begin
		 r_d3<=	(r_d2>255)? 255:r_d2;
		 g_d3<= (g_d2>255)? 255:g_d2;
		 b_d3<= (b_d2>255)? 255:b_d2;
	end
end





reg[3:0]per_frame_vsync_r;
reg[3:0]per_frame_de_r;
reg[3:0]per_frame_hs_r;

always @(posedge clk or negedge rst_n) begin
	if(~rst_n) begin
		per_frame_vsync_r<=0;
		per_frame_de_r   <=0;
        per_frame_hs_r   <=0;
	end else begin
		per_frame_vsync_r<={per_frame_vsync_r[2:0],per_frame_vsync};
		per_frame_de_r   <={per_frame_de_r	[2:0]   ,per_frame_de   };
        per_frame_hs_r   <={per_frame_hs_r	[2:0]   ,per_frame_hs   };
	end
end


assign post_frame_vsync=per_frame_vsync_r[3];
assign post_frame_de   =per_frame_de_r   [3];
assign post_frame_hs   =per_frame_hs_r   [3];
assign post_frame_data =post_frame_de?{r_d3[7:0],g_d3[7:0],b_d3[7:0]}:0;


endmodule
