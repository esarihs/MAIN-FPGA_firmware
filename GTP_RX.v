`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2022/05/31 17:22:00
// Design Name: 
// Module Name: GTP_RX
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module GTP_RX(
    input wire gt_rxusrclk_in,
    input wire reset_in,
    input wire [15:0] gt_rxdata,
    input wire [1:0]  gt_rxcharisk,
    input wire [3:0] COMMUNICATE_BTW_FPGA,
    output reg [7:0] WR_DATA,
    output reg wr_en
    );
/*    
    parameter PAUSE = 4'h0;
    parameter HEADER = 4'h1;
    parameter USR_DATA = 4'h2;
    parameter CONSTANT_DATA = 4'h3;
    parameter FOOTER = 4'h4;
*/
    parameter PAUSE = 4'h0;
    parameter RX1   = 4'h1;
    parameter RX2   = 4'h2;
    parameter RX3   = 4'h3;
    parameter RX4   = 4'h4;
    parameter RX5   = 4'h5;
/*        
    reg [3:0] RX_state;
    reg [63:0] RX_paralell_data_reg;
    reg [63:0] RX_paralell_data_reg_tmp;
    reg [3:0] RX_counter;
    assign RX_paralell_data = RX_paralell_data_reg;
*/
    reg [3:0] RX_state;

    always@(posedge gt_rxusrclk_in) begin
        if(reset_in) begin
            RX_state <= RX1;
        end
        else begin
            case (RX_state)
                PAUSE : begin
                    RX_state <= RX1;
                    WR_DATA <= 8'b0;
                    wr_en <= 1'b0;
                end
                RX1 : begin
                    WR_DATA <= gt_rxdata[15:8];
                    if(gt_rxcharisk == 2'b1) begin
                        RX_state <= RX2;  
                        wr_en <= 1'b1;      
                    end
                    else begin
                        RX_state <= RX1;
                        wr_en <= 1'b0;
                    end
                end
                RX2 : begin
                    RX_state <= RX3;
                    WR_DATA <= gt_rxdata[15:8];
                    wr_en <= 1'b1;                    
                end 
                RX3 : begin
                    RX_state <= RX4;
                    WR_DATA <= gt_rxdata[15:8];
                    wr_en <= 1'b1;                    
                end 
                RX4 : begin
                    RX_state <= RX5;
                    WR_DATA <= gt_rxdata[15:8];
                    wr_en <= 1'b1;                    
                end 
                RX5 : begin
                    RX_state <= RX1;
                    WR_DATA <= gt_rxdata[15:8];
                    wr_en <= 1'b1;                    
                end 
            endcase
        end
    end
                    
                    
/*        
    always@(posedge gt_rxusrclk_in) begin
        if(reset_in) begin
            RX_state <= HEADER;
            RX_paralell_data_reg <= 64'b0;
            RX_paralell_data_reg_tmp <= 64'b0;
            RX_counter <= 4'b0;
        end
        else begin
            case (RX_state)
                PAUSE : begin
                    RX_state <= HEADER;
                end
                HEADER : begin
                    if(gt_rxcharisk == 2'b01 && COMMUNICATE_BTW_FPGA == 1'b1) begin
                        RX_state <= USR_DATA;
                    end
                    else begin
                        RX_state <= HEADER;
                    end
                    RX_paralell_data_reg[63:0] <= RX_paralell_data_reg_tmp[63:0];
                    RX_paralell_data_reg_tmp[63:0] <= {RX_paralell_data_reg_tmp[47:0], gt_rxdata};
                end
                USR_DATA : begin
                    RX_state <= CONSTANT_DATA;
                    RX_paralell_data_reg_tmp[63:0] <= {RX_paralell_data_reg_tmp[47:0], gt_rxdata};
                    RX_counter <= RX_counter + 4'b1;
                end
                CONSTANT_DATA : begin
                    RX_state <= FOOTER;
                    RX_paralell_data_reg_tmp[63:0] <= {RX_paralell_data_reg_tmp[47:0], gt_rxdata};
                end
                FOOTER : begin
                    RX_state <= HEADER;
                    RX_paralell_data_reg_tmp[63:0] <= {RX_paralell_data_reg_tmp[47:0], gt_rxdata};
                end
                default : begin
                    RX_state <= PAUSE;
                end
           endcase
        end
    end
*/

/*                    
vio_1 RX_counter_check (
      .clk(gt_rxusrclk_in),              // input wire clk
      .probe_in0(RX_counter[0]),  // input wire [0 : 0] probe_in0
      .probe_in1(RX_counter[1]),  // input wire [0 : 0] probe_in1
      .probe_in2(RX_counter[2]),  // input wire [0 : 0] probe_in2
      .probe_in3(RX_counter[3])  // input wire [0 : 0] probe_in3
    );    
*/

ila_RXdata RXdata (
	.clk(gt_rxusrclk_in), // input wire clk

    .probe0(gt_rxdata), // input wire [15:0]  probe0  
    .probe1(gt_rxcharisk), // input wire [1:0]  probe1 
    .probe2(RX_state), // input wire [3:0]  probe2 
    .probe3(COMMUNICATE_BTW_FPGA), // input wire [3:0]  probe3 
    .probe4(WR_DATA), // input wire [7:0]  probe4 
    .probe5(wr_en) // input wire [0:0]  probe5
);    
endmodule
