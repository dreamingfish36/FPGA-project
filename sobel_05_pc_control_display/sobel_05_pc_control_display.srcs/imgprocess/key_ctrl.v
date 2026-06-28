module key_ctrl(
    
    
    input clk,          // 150 MHz时钟信号
    input rst_n,
   input key ,
//15个画面

    input   per0_frame_vsync,
    input   per0_frame_href,
    input  [15:0]    write_data_0,

    input   per01_frame_vsync,
    input   per01_frame_href,
    input  [15:0]    write_data_01,

    input   per02_frame_vsync,
    input   per02_frame_href,
    input  [15:0]    write_data_02,

    input   per1_frame_vsync,
    input   per1_frame_href,
    input  [15:0]    write_data_1,

    input   per2_frame_vsync,
    input   per2_frame_href,
    input   [15:0]    write_data_2,

    input   per21_frame_vsync,
    input   per21_frame_href,
    input   [15:0]    write_data_21,

    input   per3_frame_vsync,
    input   per3_frame_href,
    input   [15:0]    write_data_3,

   input   per31_frame_vsync,
    input   per31_frame_href,
    input   [15:0]     write_data_31,

   input   per32_frame_vsync,
    input   per32_frame_href,
    input   [15:0]     write_data_32,

    input   per4_frame_vsync,
    input   per4_frame_href,
    input   [15:0]    write_data_4,

    input   per5_frame_vsync,
    input   per5_frame_href,
    input   [15:0]    write_data_5,

    input   per6_frame_vsync,
    input   per6_frame_href,
    input   [15:0]    write_data_6,

    input   per7_frame_vsync,
    input   per7_frame_href,
    input   [15:0]    write_data_7,

    input   per8_frame_vsync,
    input   per8_frame_href,
    input   [15:0]    write_data_8,

    input   per9_frame_vsync,
    input   per9_frame_href,
    input   [15:0]    write_data_9,
    /*
    input per10_frame_vsync,
	input per10_frame_href,
	input  per10_img_Bit,
*/    

    output  post_frame_vsync,
    output post_frame_href, 
    output [15:0]write_data

    
  

);
		wire ctrl;
	wire key_flag0, key_state0;
assign ctrl = key_flag0 && (~key_state0);

	key key_filter0(
		.clk(clk),
		.reset_n(rst_n),
		.key_in(key),
		.key_flag(key_flag0),
		.key_state(key_state0 )
	);
// ctrl ==2'd1变换

// 计数器的位宽
parameter COUNTER_WIDTH = 32;
// 计数器的最大值//150MHZ 下延时5秒
parameter MAX_COUNT = 750000000*2;

// 计数器寄存器
reg [COUNTER_WIDTH-1:0] counter;
// 计数器逻辑
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        // 异步复位
        counter <= 0;
    end else begin
        if (counter == MAX_COUNT - 1) begin
            // 达到最大计数，设置done信号
            counter <= 0;      
        end else begin
            // 计数
            counter <= counter + 1;
            
        end
    end
end

// 状态定义
localparam [3:0]  
mode0= 4'd0,
mode01=4'd1,
mode02=4'd2,
mode1= 4'd3, 
mode2= 4'd4, 
mode21=4'd5, 
mode3= 4'd6,  
mode31=4'd7,  
mode32=4'd8,  
mode4= 4'd9,  
mode5= 4'd10,  
mode6= 4'd11, 
mode7= 4'd12,
mode8= 4'd13, 
mode9= 4'd14;
//mode10=4'd15;


reg		post_frame_vsync_r;
reg	    post_frame_href_r;	
reg    [15:0]write_data_r;

// 状态寄存器
reg [3:0] current_state;
// 计数器逻辑
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        // 异步复位
        current_state <= mode0;
        post_frame_vsync_r<=per0_frame_vsync;
        post_frame_href_r<=per0_frame_href;
        write_data_r<=write_data_0;
    end else begin
        case (current_state)
        
        //原图
            mode0: begin
                if (ctrl ==1'd1)            
                    current_state <= mode01; // 进入延时状态                 
                    else begin
                        post_frame_vsync_r<=per0_frame_vsync;
                        post_frame_href_r<=per0_frame_href;
                        write_data_r <= write_data_0;
                end 
                
            end

            mode01: begin
                if (ctrl ==1'd1)            
                    current_state <= mode02; // 进入延时状态                 
                    else begin
                        post_frame_vsync_r<=per01_frame_vsync;
                        post_frame_href_r<=per01_frame_href;
                        write_data_r <= write_data_01;
                end 
                
            end
            mode02: begin
                if (ctrl ==1'd1)            
                    current_state <= mode1; // 进入延时状态                 
                    else begin
                        post_frame_vsync_r<=per02_frame_vsync;
                        post_frame_href_r<=per02_frame_href;
                        write_data_r <= write_data_02;
                end 
                
            end
            
            mode1: begin
                if (ctrl ==1'd1) begin           
                    current_state <= mode2; 
                     end
                    else begin
                        post_frame_vsync_r <=per1_frame_vsync;
                        post_frame_href_r  <=per1_frame_href;
                        write_data_r    <=write_data_1;    //24位
                end
            end

            mode2: begin
                if (ctrl ==1'd1) begin           
                    current_state <= mode21; 
                     end
                    else begin
                        post_frame_vsync_r<=per2_frame_vsync;
                        post_frame_href_r<=per2_frame_href;
                        write_data_r<=write_data_2;
                end
            end

            mode21: begin
                if (ctrl ==1'd1) begin           
                    current_state <= mode3; 
                     end
                    else begin
                        post_frame_vsync_r<=per21_frame_vsync;
                        post_frame_href_r<=per21_frame_href;
                        write_data_r<=write_data_21;
                end
            end

            mode3: begin
                if (ctrl ==1'd1)         
                    current_state <= mode31;
                    else begin
                        post_frame_vsync_r<=per3_frame_vsync;
                        post_frame_href_r<=per3_frame_href;
                        write_data_r<=write_data_3; 
                end
            end
    
            mode31: begin
                if (ctrl ==1'd1)         
                    current_state <= mode32;
                    else begin
                        post_frame_vsync_r<=per31_frame_vsync;
                        post_frame_href_r<=per31_frame_href;
                        write_data_r<=write_data_31; 
                end
            end

            mode32: begin
                if (ctrl ==1'd1)         
                    current_state <= mode4;
                    else begin
                        post_frame_vsync_r<=per32_frame_vsync;
                        post_frame_href_r<=per32_frame_href;
                        write_data_r<=write_data_32; 
                end
            end
            mode4: begin
                if (ctrl ==1'd1)     
                    current_state <= mode5; 
                    else begin
                        post_frame_vsync_r<=per4_frame_vsync;
                        post_frame_href_r<=per4_frame_href;
                        write_data_r<=write_data_4;
                
                end
            end

            mode5: begin
                if (ctrl ==1'd1)          
                    current_state <= mode6; 
                    else begin
                        post_frame_vsync_r<=per5_frame_vsync;
                        post_frame_href_r<=per5_frame_href;
                        write_data_r<=write_data_5;
                 
                end
            end

            mode6: begin
                if (ctrl ==1'd1)     
                    current_state <= mode7; 
                    else begin
                        post_frame_vsync_r<=per6_frame_vsync;
                        post_frame_href_r<=per6_frame_href;
                        write_data_r<=write_data_6;//24位
                 
                end
            end

            mode7: begin
                if (ctrl ==1'd1)      
                    current_state <= mode8;
                    else begin
                        post_frame_vsync_r<=per7_frame_vsync;
                        post_frame_href_r<=per7_frame_href;
                        write_data_r<=write_data_7;//24位
                 
                end
            end


            mode8: begin
                if (ctrl ==1'd1)         
                    current_state <= mode9; 
                    else begin
                        post_frame_vsync_r<=per8_frame_vsync;
                        post_frame_href_r<=per8_frame_href;
                        write_data_r<=write_data_8;
                
                end
            end

            mode9: begin
                if (ctrl ==1'd1)          
                    current_state <= mode0; 
                    else begin
                        post_frame_vsync_r<=per9_frame_vsync;
                        post_frame_href_r<=per9_frame_href;
                        write_data_r<=write_data_9;//24位
                 
                end
            end
            
            
            default: begin
                current_state <= mode0;
                post_frame_vsync_r<=per0_frame_vsync;
                post_frame_href_r<=per0_frame_href;
                write_data_r <= write_data_0;
            end
        endcase
    end
end



/*
always@(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		post_frame_vsync_r <= 0;
		per_frame_href_r <= 0;
	end
	else begin
		post_frame_vsync_r 	<= 	
		per_frame_href_r 	<= 	
	end
end
*/
////
reg	[2:0]	post_frame_vsync_r2;
reg	[2:0]	post_frame_href_r2;	
reg [15:0]  post_img_Gray_r2[2:0];
always@(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		post_frame_vsync_r2 <= 0;
		post_frame_href_r2 <= 0;
       post_img_Gray_r2[0]<= 0;
       post_img_Gray_r2[1]<= 0;
       post_img_Gray_r2[2]<= 0;
	end
	else begin
		post_frame_vsync_r2 	<= 	{post_frame_vsync_r2[1:0], 	post_frame_vsync_r};
		post_frame_href_r2 	<= 	{post_frame_href_r2[1:0], 	post_frame_href_r};
         	post_img_Gray_r2[0]<= 	write_data_r;
             post_img_Gray_r2[1]<= 	post_img_Gray_r2[0];
             post_img_Gray_r2[2]<= 	post_img_Gray_r2[1];
	end
end



assign	post_frame_vsync 	= 	post_frame_vsync_r2[2];
assign	post_frame_href 	= 	post_frame_href_r2[2];
assign	write_data		=	post_frame_href_r2[2] ? post_img_Gray_r2[0] : 15'd0;

endmodule








