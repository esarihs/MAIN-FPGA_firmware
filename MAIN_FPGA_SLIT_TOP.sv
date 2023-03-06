`timescale 1ps / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2020/06/04 16:39:49
// Design Name: 
// Module Name: SLIT_TOP
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


module SLIT_TOP #(
			   parameter PAR_NASIC = 1,
			   parameter PAR_NMAXASIC = 16, // fixed
			   parameter PAR_NDATA_BIT = 14'd8192,
			   parameter PAR_NDATA_NUM = 8192,
			   parameter FPGA_REV = 32'h2023_0210,
			   parameter FPGA_VER = 8'h01,
			   parameter PAR_ISFLIP_D_EXT_RD_START   =   16'b1100_1001_1100_1101,
			   parameter PAR_ISFLIP_D_EXT_WR_START   =   16'b0000_0000_0000_0001, // only one bit is valid
			   parameter PAR_ISFLIP_EXTERNAL_QUARTZCLK   =   16'b0,
			   parameter PAR_ISFLIP_D_READ_CLK   =   16'b0100_1111_1101_1110,
               parameter PAR_ISFLIP_D_SAMPLING_CLK   =   16'h0000_0000_0000_0001, // only one bit is valid.
               parameter PAR_ISFLIP_D_MSCK   =   16'b0000_0000_0000_1100, // only four bits are valid
               parameter PAR_ISFLIP_D_MSCS   =   16'b1001_1110_0100_1011,
               parameter PAR_ISFLIP_D_MSI   =   16'b0100_1111_0100_1011,
               parameter PAR_ISFLIP_D_MSO   =   16'b0100_1110_1100_0111,
               parameter PAR_ISFLIP_D_RD_RDY =   16'b0000_0000_0000_1111, // only eight bits are valid
               parameter PAR_ISFLIP_D_RD_BUSY =   16'b0000_0000_1101_1111, // only eight bits are valid
               //parameter PAR_ISFLIP_D_RD_BUSY =   16'b0000_0000_1100_1111, // only eight bits are valid
               parameter PAR_ISFLIP_D_FPGA_SER_A = 16'b0100_1111_0100_1110,
               parameter PAR_ISFLIP_D_FPGA_SER_C = 16'b0000_0000_0000_0000, 
               //parameter PAR_ISFLIP_D_FPGA_SER_D = 16'b1100_1001_1100_1101
               parameter PAR_ISFLIP_D_FPGA_SER_D = 16'b1110_0111_0100_1111
			   )(
			     // Reset and Clock
			     input          ExternalQuartzClk_P,
			     input          ExternalQuartzClk_N,
			     // SliT I/F
			     // Clock
			     output [PAR_NASIC-1:0] D_READ_CLK_P,
			     output [PAR_NASIC-1:0] D_READ_CLK_N,
			     output  [0:0] D_SAMPLING_CLK_P,
			     output  [0:0] D_SAMPLING_CLK_N,
			     // ASIC Status signal
			     input [PAR_NASIC-1:0]  D_RD_RDY_P,
			     input [PAR_NASIC-1 - ((PAR_NASIC-1)/8)*(PAR_NASIC-8):0]  D_RD_RDY_N,
			     //input [PAR_NASIC-1:0]  D_RD_RDY_N,
			     input [PAR_NASIC-1:0]  D_RD_BUSY_P,
			     input [PAR_NASIC-1 - ((PAR_NASIC-1)/8)*(PAR_NASIC-8):0]  D_RD_BUSY_N,
			     //input [PAR_NASIC-1:0]  D_RD_BUSY_N,
			     /*
			     // Reset (generated from another FPGA [U10])
			     output [PAR_NASIC-1:0] D_RST_N_P,
			     output [PAR_NASIC-1:0] D_RST_N_N,
			     */

			     // Slow Control I/F
			     output [((PAR_NASIC-1)/4):0] D_MSCK_P, 
			     output [((PAR_NASIC-1)/4):0] D_MSCK_N, 
			     output [PAR_NASIC-1:0] D_MSCS_P,
			     output [PAR_NASIC-1:0] D_MSCS_N,
			     input [PAR_NASIC-1:0]  D_MSO_P,
			     input [PAR_NASIC-1:0]  D_MSO_N,
			     output [PAR_NASIC-1:0] D_MSI_P,
			     output [PAR_NASIC-1:0] D_MSI_N,

			     // Trigger signal

			     output D_EXT_WR_START_P,
			     output D_EXT_WR_START_N,
			     output [PAR_NASIC-1:0] D_EXT_RD_START_P,
			     output [PAR_NASIC-1:0] D_EXT_RD_START_N,
			     // Data I/F
			     input [PAR_NASIC-1:0]  D_FPGA_SER_C_P,
			     input [PAR_NASIC-1:0]  D_FPGA_SER_C_N,
			     input [PAR_NASIC-1:0]  D_FPGA_SER_A_P,
			     input [PAR_NASIC-1:0]  D_FPGA_SER_A_N,
			     input [PAR_NASIC-1:0]  D_FPGA_SER_D_P,
			     input [PAR_NASIC-1:0]  D_FPGA_SER_D_N,
			     
			     output  ICDAC_CSLD,
			     output  ICDAC_SCK,
			     output  ICDAC_DIN,
			     output  [7:0]   TESTPULSE_SWITCH,

	            output  [1:0]   TEST_PIN,
                output [((PAR_NASIC-1)/8):0] ENB_1V8A,
                output [((PAR_NASIC-1)/8):0] ENB_1V8D,
                output [((PAR_NASIC-1)/8):0] ENB_1V1A,
                output  ENB_SAMPLING_CLK,
                output  ENB_WR_TRG,
                output  ENB_ASIC_RST,
                inout   [7:0]   TEMP_SDA,
                inout   [7:0]   TEMP_SCL,
                input   MUX_OUT,
                output  MUX_SPI_SYNC_B,
                output  MUX_SPI_DIN,
                output  MUX_SPI_CLK,
                output   [3:0]   COMMUNICATE_BTW_FPGA_TX,
                input    [3:0]   COMMUNICATE_BTW_FPGA_RX,
			     // EEPROM
                 output      EEPROM_CS   ,
                 output      EEPROM_SK   ,
                 output      EEPROM_DI   ,
                 input       EEPROM_DO   ,
                 // Jitter Cleaner
                 output JC_IN_CLK_P,
                 output JC_IN_CLK_N,
                 input  JC_OUT_CLK_P,
                 input  JC_OUT_CLK_N,
                 output JC_SPI_CS,
                 output JC_SPI_CLK,
                 output JC_SPI_SDI,
                 input  JC_SPI_SDO,
                 output JC_RSTB,
                 output JC_OEB,
                 input  JC_LOLB,
                 input  JC_INTRB,
                 input  JC_LOS_XAXBB,         
                 //I2C
                 output SCL,
                 inout SDA,        
                 output MOD_ABS,
                 // GTP
                 input           RXN_IN,
                 input           RXP_IN,
                 output          TXN_OUT,
                 output          TXP_OUT,
                 input           GTP_REFCLK_P        ,
                 input           GTP_REFCLK_N        ,
                 output          SFP_TX_P            ,
                 output          SFP_TX_N            ,
                 input           SFP_RX_P            ,
                 input           SFP_RX_N                             
			     );
   // ----------------------------------------------

   wire [4:0]   ASIC_IDs    [15:0]; // {isFarZ0, isOuterSensor, asic8Id[2:0]}
   assign  ASIC_IDs[0]  =   {1'b0,1'b1,3'd0}; // asic32Id = #8
   assign  ASIC_IDs[1]  =   {1'b0,1'b1,3'd1}; // asic32Id = #9
   assign  ASIC_IDs[2]  =   {1'b0,1'b1,3'd2}; // asic32Id = #10
   assign  ASIC_IDs[3]  =   {1'b0,1'b1,3'd3}; // asic32Id = #11
   assign  ASIC_IDs[4]  =   {1'b0,1'b1,3'd4}; // asic32Id = #12
   assign  ASIC_IDs[5]  =   {1'b0,1'b1,3'd5}; // asic32Id = #13
   assign  ASIC_IDs[6]  =   {1'b0,1'b1,3'd6}; // asic32Id = #14
   assign  ASIC_IDs[7]  =   {1'b0,1'b1,3'd7}; // asic32Id = #15
   assign  ASIC_IDs[8]  =   {1'b1,1'b1,3'd0}; // asic32Id = #24
   assign  ASIC_IDs[9]  =   {1'b1,1'b1,3'd1}; // asic32Id = #25
   assign  ASIC_IDs[10] =   {1'b1,1'b1,3'd2}; // asic32Id = #26
   assign  ASIC_IDs[11] =   {1'b1,1'b1,3'd3}; // asic32Id = #27
   assign  ASIC_IDs[12] =   {1'b1,1'b1,3'd4}; // asic32Id = #28
   assign  ASIC_IDs[13] =   {1'b1,1'b1,3'd5}; // asic32Id = #29
   assign  ASIC_IDs[14] =   {1'b1,1'b1,3'd6}; // asic32Id = #30
   assign  ASIC_IDs[15] =   {1'b1,1'b1,3'd7}; // asic32Id = #31

    //assign  ENB_1V8A[0] =   1'b1;
    //assign  ENB_1V8D[0] =   1'b1;
    //assign  ENB_1V1A[0] =   1'b1;
    //assign  ENB_SAMPLING_CLK    =   1'b1;
    //assign  ENB_WR_TRG       gt_rxdata   =   1'b1;
    //assign  ENB_RST             =   1'b1;
   //------------------------------------------------------------------------------
   // Clock Generator
   //------------------------------------------------------------------------------
    wire ExternalQuartzClk;
  LVDS_RECEIVER #(
         .PAR_NASIC(1),
         .PAR_ISFLIP(PAR_ISFLIP_EXTERNAL_QUARTZCLK)
 )
 LVDS_RECEIVER_ExtQuartzClk
 (
     .InputSignal_P(ExternalQuartzClk_P),
     .InputSignal_N(ExternalQuartzClk_N),
     .OutputSignal(ExternalQuartzClk)
         );

   wire 					    SystemClk;
   wire 					    SamplingClk;
   wire 					    ReadClk;
   wire 					    AsynchronousRst;

   CLOCK_GENERATOR(
		   .MmcmRst(1'b0),
		   .ExternalClk(ExternalQuartzClk),
		   .SystemClk(SystemClk),
		   .SamplingClk(SamplingClk),
		   .ReadClk(ReadClk),
           .JC_SCLK(JC_SPI_CLK),
		   .SystemRst(AsynchronousRst)
		   );

   //------------------------------------------------------------------------------
   // Synchronous Reset
   //------------------------------------------------------------------------------
   wire     TCP_OPEN_ACK;
   wire     SynchronousRst;
   assign SynchronousRst    = ~TCP_OPEN_ACK;

   //------------------------------------------------------------------------------
   // ASIC RESET
   //------------------------------------------------------------------------------
   wire AsicResetTrigger;
/*
   wire AsicResetSignal;
   GENERATE_PULSE #(
		    .PAR_WIDTH(16'd10),
		    .PAR_POLARITY(1'b0)
		    )
   GENERATE_PULSE(
		  .Rst(SynchronousRst),
		  .Clk(SystemClk),
		  .Trg(AsicResetTrigger),
		  .Dout(AsicResetSignal)
		  );

   wire [PAR_NASIC-1:0] 			    D_RST_N ;
   assign  D_RST_N =   {$bits(D_RST_N){AsicResetSignal}};
    wire    [PAR_NASIC-1:0] IsFlip_D_RST_N;
    assign  IsFlip_D_RST_N  =   1'b0; // tmpppp
   LVDS_TRANSMITTER #(
		 .PAR_NASIC(PAR_NASIC)
		 )
		 LVDS_TRANSMITTER_RESET(
		   .InputSignal(D_RST_N),
		   .IsFlip(IsFlip_D_RST_N),
		   .OutputSignal_P(D_RST_N_P),
		   .OutputSignal_N(D_RST_N_N)
		   );
*/
   //------------------------------------------------------------------------------
   //  Clock buffer
   //------------------------------------------------------------------------------
   wire 					    SamplingClk_ONOFF;
   wire 					    ReadClk_ONOFF;

   wire 					    RD_CLK_ENB;
   //assign  RD_CLK_ENB           =   AsicRdRdy[0]; // tmpppp
   assign  RD_CLK_ENB           =   1'b1; // tmpppp
   assign ReadClk_ONOFF         = ReadClk  && (RD_CLK_ENB || ReadClkForceEnable ) && ~ReadClkForceDisable;

   //assign SamplingClk_ONOFF     = SamplingClk && SamplingClkForceEnable;
   //assign SAMPING_CLK_ONOFF    = SAMPLING_CLK  && (SAMPLING_CLK_ENB || SAMPLING_CLK_FORCE_ENB);
   assign SamplingClk_ONOFF     = SamplingClk && SamplingClkForceEnable;

   LVDS_TRANSMITTER_FROM_DDR #(
			  .PAR_NASIC(PAR_NASIC),
			  .PAR_ISFLIP(PAR_ISFLIP_D_READ_CLK)
			  )
   LVDS_TRANSMITTER_FROM_DDR_READ(
			     .InputSignal(ReadClk_ONOFF),
			     .OutputSignal_P(D_READ_CLK_P),
			     .OutputSignal_N(D_READ_CLK_N)
			     );

   LVDS_TRANSMITTER_FROM_DDR #(
			  .PAR_NASIC(1),
			  .PAR_ISFLIP(PAR_ISFLIP_D_SAMPLING_CLK)
			  )
   LVDS_TRANSMITTER_FROM_DDR_SAMPLING(
				 .InputSignal(SamplingClk_ONOFF),
				 .OutputSignal_P(D_SAMPLING_CLK_P),
				 .OutputSignal_N(D_SAMPLING_CLK_N)
				 );

//------------------------------------------------------------------------------
//  Slow Control
//------------------------------------------------------------------------------
    wire    [((PAR_NASIC-1)/4):0] D_MSCK  ;
    wire    [PAR_NMAXASIC-1:0] tmp_D_MSCK;
    wire    [PAR_NASIC-1:0] D_MSCS  ;
    wire    [PAR_NASIC-1:0] D_MSO   ;
    wire    [PAR_NASIC-1:0] D_MSI   ; 

    wire    [PAR_NMAXASIC-1:0]  SlowCtrlSelect;
    genvar i;
    for(i=0; i<((PAR_NASIC-1)/4+1); i++) begin
        assign  D_MSCK[i]   =   (tmp_D_MSCK[4*i+0] & SlowCtrlSelect[4*i+0]) | 
                                (tmp_D_MSCK[4*i+1] & SlowCtrlSelect[4*i+1]) | 
                                (tmp_D_MSCK[4*i+2] & SlowCtrlSelect[4*i+2]) | 
                                (tmp_D_MSCK[4*i+3] & SlowCtrlSelect[4*i+3]);
    end
  
  LVDS_TRANSMITTER #(
        .PAR_NASIC((PAR_NASIC-1)/4+1),
        .PAR_ISFLIP(PAR_ISFLIP_D_MSCK)
    )
    LVDS_TRANSMITTER_MSCK
    (
    .InputSignal(D_MSCK),
    .OutputSignal_P(D_MSCK_P),
    .OutputSignal_N(D_MSCK_N)
        );

  LVDS_TRANSMITTER #(
        .PAR_NASIC(PAR_NASIC),
        .PAR_ISFLIP(PAR_ISFLIP_D_MSCS)
    )
        LVDS_TRANSMITTER_MSCS
    (
    .InputSignal(D_MSCS & SlowCtrlSelect[PAR_NASIC-1:0]),
    .OutputSignal_P(D_MSCS_P),
    .OutputSignal_N(D_MSCS_N)
        );

  LVDS_TRANSMITTER #(
        .PAR_NASIC(PAR_NASIC),
        .PAR_ISFLIP(PAR_ISFLIP_D_MSI)
)
  LVDS_TRANSMITTER_MSI
(
    .InputSignal(D_MSI & SlowCtrlSelect[PAR_NASIC-1:0]),
    .OutputSignal_P(D_MSI_P),
    .OutputSignal_N(D_MSI_N)
        );

  LVDS_RECEIVER #(
        .PAR_NASIC(PAR_NASIC),
        .PAR_ISFLIP(PAR_ISFLIP_D_MSO)
)
LVDS_RECEIVER_MSO
(
    .InputSignal_P(D_MSO_P),
    .InputSignal_N(D_MSO_N),
    .OutputSignal(D_MSO)
        );
   //------------------------------------------------------------------------------
   //  Register Control
   //------------------------------------------------------------------------------
   wire RBCP_ACT        ;
   wire [31:0] RBCP_ADDR       ;
   wire [7:0]  RBCP_WD         ;
   wire        RBCP_WE         ;
   wire        RBCP_RE         ;
   wire        RBCP_REG_ACK    ;
   wire [7:0]  RBCP_REG_RD     ;

   wire        SiTCP_RST       ;

   wire [15:0] TIM_PERIOD     ;
   wire         SoftwareWriteTrigger;
   wire         SoftwareReadTrigger;
   wire [1:0]  TriggerEdgeType; // [1] Write, [0] Read
   wire [15:0] DelayHardwareWriteTrigger;
   wire [15:0] DelayHardwareAutoReadTrigger;
   wire        SamplingClkForceEnable;
   wire        ReadClkForceEnable;
   wire        ReadClkForceDisable;

   wire        LemoInputPolarity; // 1(not changed), 0(flipped)

    wire    HardwareWriteTriggerEnable;
    wire    CyclicAutoWriteTriggerEnable;
    wire    [27:0]  CyclicAutoWriteTriggerInterval;
    wire    HardwareReadTriggerEnable;
    wire    AutoReadTriggerEnable;

	wire   [15:0]  IsEnableAsic;
    wire   IsEnableZeroSuppression;
    wire [1:0]  ModeSelection;
    wire [23:0] 	    NSubData;
    wire    IsEnableTimeWindow;
    wire    [12:0]  TimeWindow_Cycle;
    wire    [12:0]  TimeWindow_Start;
    wire    [12:0]  TimeWindow_End;
    wire    [7:0]   AddressOutsideFrbs;
    wire    [23:0]  IcDacSetting;
    wire    [7:0]   IcTrigger;
    wire [7:0]   TestPulseEnable;
    wire [15:0]  TestPulseDelay;
    wire [15:0]  TestPulseInterval;
    wire [1:0][7:0]  SelectTestSignal;
    wire [PAR_NMAXASIC-1:0]                AsicRdRdy16;
    wire [PAR_NMAXASIC-1:0]                AsicRdBusy16;
    wire    [7:0]   OtherSignals;
    assign  OtherSignals    =   {6'b0,TriggerReady,TCP_OPEN_ACK};
    wire ADC_mon_enb;
    wire [6:0] ADC_mon_addr;
    wire [15:0] ADC_mon_data;
    wire I2C_reset;
    wire I2C_start;
    wire [7:0] I2C_address;
    wire I2C_error;
    wire [7:0] I2C_data;
    


   SLIT_REG #(
		     .FPGA_REV   (FPGA_REV),
		     .FPGA_VER   (FPGA_VER),
             .PAR_NASIC(PAR_NASIC)
		     ) SLIT_REG(
				       .CLK                    (SystemClk          ),   // in   : Clock
				       .RST                    (SiTCP_RST          ),   // in   : System reset
				       // SiTCP RBCP I/F
				       .LOC_ADDR               (RBCP_ADDR[31:0]    ),   // in   : Address[31:0]
				       .LOC_WD                 (RBCP_WD[7:0]       ),   // in   : Data[7:0]
				       .LOC_WE                 (RBCP_WE            ),   // in   : Write enable
				       .LOC_RE                 (RBCP_RE            ),   // in   : Read enable
				       .LOC_ACK                (RBCP_REG_ACK       ),   // out  : Access acknowledge
				       .LOC_RD                 (RBCP_REG_RD[7:0]   ),    // out  : Read data[7:0]
                        // ------------------------------------------------------------------
				       .TIM_PERIOD             (TIM_PERIOD[15:0]   ),
				       .SlowCtrlSelect         (SlowCtrlSelect),
				       .SoftwareWriteTrigger(SoftwareWriteTrigger),
				       .SoftwareReadTrigger(SoftwareReadTrigger),
				       .TriggerEdgeType(TriggerEdgeType),
				       .DelayHardwareWriteTrigger(DelayHardwareWriteTrigger),
                       .DelayHardwareAutoReadTrigger(DelayHardwareAutoReadTrigger),
				       .SamplingClkForceEnable(SamplingClkForceEnable),
				       .ReadClkForceEnable(ReadClkForceEnable),
				       .ReadClkForceDisable(ReadClkForceDisable),
				       .LemoInputPolarity(LemoInputPolarity),
				       .AsicResetTrigger(AsicResetTrigger),
				       .HardwareWriteTriggerEnable(HardwareWriteTriggerEnable),
                       .CyclicAutoWriteTriggerEnable(CyclicAutoWriteTriggerEnable),
                       .CyclicAutoWriteTriggerInterval(CyclicAutoWriteTriggerInterval),
                       .HardwareReadTriggerEnable(HardwareReadTriggerEnable),
                       .AutoReadTriggerEnable(AutoReadTriggerEnable),
                       .IsEnableAsic(IsEnableAsic),
                       .IsEnableZeroSuppression(IsEnableZeroSuppression),
                       .ModeSelection(ModeSelection),
                       .NSubData(NSubData),
                       .IsEnableTimeWindow(IsEnableTimeWindow),
                       .TimeWindow_Cycle(TimeWindow_Cycle),
                       .TimeWindow_Start(TimeWindow_Start),
                       .TimeWindow_End(TimeWindow_End),
                       .AddressOutsideFrbs(AddressOutsideFrbs),
                       .IcDacSetting(IcDacSetting),
                       .IcTrigger(IcTrigger),
                       .TestPulseEnable(TestPulseEnable),
                       .TestPulseDelay(TestPulseDelay),
                       .TestPulseInterval(TestPulseInterval),
                       .SelectTestSignal0(SelectTestSignal[0]),
                       .SelectTestSignal1(SelectTestSignal[1]),
                       .AsicRdRdy16(AsicRdRdy16),
                       .AsicRdBusy16(AsicRdBusy16),
                       .OtherSignals(OtherSignals),
                       .Enb1V8A(ENB_1V8A),
                       .Enb1V8D(ENB_1V8D),
                       .Enb1V1A(ENB_1V1A),
                       .EnbSamplingClk(ENB_SAMPLING_CLK),
                       .EnbWrTrg(ENB_WR_TRG),
                       .EnbAsicRst(ENB_ASIC_RST),
                       .ADC_enable(ADC_mon_enb),
                       .ADC_address(ADC_mon_addr),
                       .ADC_data(ADC_mon_data),
                       .COMMUNICATE_BTW_FPGA({COMMUNICATE_BTW_FPGA_TX[2:0],COMMUNICATE_BTW_FPGA_RX[3:0]}),
                       .I2C_reset(I2C_reset),
                       .I2C_start(I2C_start),
                       .I2C_address(I2C_address),
                       .I2C_error(I2C_error),
                       .I2C_data(I2C_data)
                       );

   wire        SIO_ACT         ;
   assign  SIO_ACT = RBCP_ACT & (RBCP_ADDR[31:28]!=4'd8);

   //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   // copy from timer.v
   wire        TIM_1US_SC    ;
   reg 	       pulse1us   ;
   reg 	       usCry       ;
   reg [15:0]  usTim      ;

   reg [31:0]  Counter_SYS_RST_N;
   always@ (posedge SystemClk or posedge AsynchronousRst) begin
      if(AsynchronousRst)begin
         {usCry,usTim[15:0]}          <= 17'd0;
      end else begin
         {usCry,usTim[15:0]}      <= (usCry    ? {1'b0,TIM_PERIOD[15:0]}    : {usCry,usTim[15:0]} - 17'd1);
      end
   end
   
   always@ (posedge SystemClk) begin
      pulse1us    <= usCry; // original
   end
   assign	TIM_1US_SC	= pulse1us;
   //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   wire [PAR_NASIC-1:0]         RBCP_SIO_ACK    ;
   wire [PAR_NASIC-1:0][7:0]    RBCP_SIO_RD     ;

    genvar  iasic;
    generate begin
        for(iasic=0; iasic<PAR_NASIC; iasic++) begin
           SIO_MASTER          SIO_MASTER(
                       // System
                       .CLK                    (SystemClk          ), // in    : System clock
                       .RST                    (SiTCP_RST          ), // in    : System reset
                       // Timer
                       //.TIM_1US                (TIM_1US            ), // in    : 1us interrupt
                       .TIM_1US                (TIM_1US_SC            ), // in    : 1us interrupt
                       .ADDR_WIDTH             (2'd3               ), // in    : Address width[1:0]
                                       //         0: 8bit, 1:16bit, 2:24bit, 3:32bit
                       // SiTCP I/F
                       .LOC_ACT                (SIO_ACT            ), // in    : Request
                       .LOC_ADDR               (RBCP_ADDR[31:0]    ), // in    : Address[31:0]
                       .LOC_WD                 (RBCP_WD[7:0]       ), // in    : Data[7:0]
                       .LOC_WE                 (RBCP_WE            ), // in    : Write enable
                       .LOC_RE                 (RBCP_RE            ), // in    : Read enable
                       .LOC_ACK                (RBCP_SIO_ACK[iasic]       ), // out   : Access acknowledge
                       .LOC_RD                 (RBCP_SIO_RD[iasic][7:0]   ), // out   : Read data[7:0]
                       // Serial I/F
                       .SCK                    (tmp_D_MSCK[iasic]  ),  // out  : Clock
                       .SCS                    (D_MSCS[iasic]    ),  // out  : Active
                       .SI                     (D_MSO[iasic]     ),  // in   : Data input
                       .SO                     (D_MSI[iasic]     )   // out  : Data output
                       );
        end
        for(iasic=PAR_NASIC; iasic<PAR_NMAXASIC; iasic++) begin
            assign  tmp_D_MSCK[iasic]   =   1'b0;
        end
    end
    endgenerate

   //------------------------------------------------------------------------------

   wire         RBCP_ACK    ;
   wire         RBCP_JC_ACK ;
   wire         RBCP_GTP_ACK;
   wire [7:0]   RBCP_RD     ;
   wire [7:0]   RBCP_JC_RD  ;
   wire [7:0]   RBCP_GTP_RD ;
   
   wire [PAR_NASIC-1:0]RBCP_SIO_ACK_RD0;
   wire [PAR_NASIC-1:0]RBCP_SIO_ACK_RD1;
   wire [PAR_NASIC-1:0]RBCP_SIO_ACK_RD2;
   wire [PAR_NASIC-1:0]RBCP_SIO_ACK_RD3;
   wire [PAR_NASIC-1:0]RBCP_SIO_ACK_RD4;
   wire [PAR_NASIC-1:0]RBCP_SIO_ACK_RD5;
   wire [PAR_NASIC-1:0]RBCP_SIO_ACK_RD6;
   wire [PAR_NASIC-1:0]RBCP_SIO_ACK_RD7;
   

   for(i=0; i<PAR_NASIC; i++) begin
        assign  RBCP_SIO_ACK_RD0[i]  =    RBCP_SIO_ACK[i] & SlowCtrlSelect[i] & RBCP_SIO_RD[i][0] ;
        assign  RBCP_SIO_ACK_RD1[i]  =    RBCP_SIO_ACK[i] & SlowCtrlSelect[i] & RBCP_SIO_RD[i][1] ;
        assign  RBCP_SIO_ACK_RD2[i]  =    RBCP_SIO_ACK[i] & SlowCtrlSelect[i] & RBCP_SIO_RD[i][2] ;
        assign  RBCP_SIO_ACK_RD3[i]  =    RBCP_SIO_ACK[i] & SlowCtrlSelect[i] & RBCP_SIO_RD[i][3] ;
        assign  RBCP_SIO_ACK_RD4[i]  =    RBCP_SIO_ACK[i] & SlowCtrlSelect[i] & RBCP_SIO_RD[i][4] ;
        assign  RBCP_SIO_ACK_RD5[i]  =    RBCP_SIO_ACK[i] & SlowCtrlSelect[i] & RBCP_SIO_RD[i][5] ;
        assign  RBCP_SIO_ACK_RD6[i]  =    RBCP_SIO_ACK[i] & SlowCtrlSelect[i] & RBCP_SIO_RD[i][6] ;
        assign  RBCP_SIO_ACK_RD7[i]  =    RBCP_SIO_ACK[i] & SlowCtrlSelect[i] & RBCP_SIO_RD[i][7] ;
   end
        

   //assign   RBCP_ACK    = |RBCP_SIO_ACK | RBCP_REG_ACK;
   assign   RBCP_ACK    = (|(RBCP_SIO_ACK&SlowCtrlSelect[PAR_NASIC-1:0])) | RBCP_REG_ACK | RBCP_JC_ACK | RBCP_GTP_ACK;
   assign   RBCP_RD     =   (RBCP_ADDR[31:16] == 16'h2000) ? RBCP_GTP_RD[7:0] : (RBCP_ADDR[31:16] == 16'h4000) ? RBCP_JC_RD[7:0] : ({$bits(RBCP_RD){RBCP_REG_ACK   }} & RBCP_REG_RD) |
                            {|RBCP_SIO_ACK_RD7,|RBCP_SIO_ACK_RD6,|RBCP_SIO_ACK_RD5,|RBCP_SIO_ACK_RD4,
                             |RBCP_SIO_ACK_RD3,|RBCP_SIO_ACK_RD2,|RBCP_SIO_ACK_RD1,|RBCP_SIO_ACK_RD0
                            } ;

  //------------------------------------------------------------------------------
   //  IC Operation
   //------------------------------------------------------------------------------
    IC_OPERATION IC_OPERATION(
        .SystemClk(SystemClk),
        .Rst(SiTCP_RST),
        .IcDac_CSLD(ICDAC_CSLD),
        .IcDac_SCK(ICDAC_SCK),
        .IcDac_DIN(ICDAC_DIN),
        .IcTrigger(IcTrigger),
        .IcDacSetting(IcDacSetting)
    );

   //------------------------------------------------------------------------------
   //  Status Signal
   //------------------------------------------------------------------------------
   wire [PAR_NASIC-1:0] 			    AsicRdRdy;
   wire [PAR_NASIC-1:0] 			    AsicRdBusy;

    assign  AsicRdRdy16  =   {{$bits(AsicRdRdy16 )- $bits(AsicRdRdy ){1'b0}},AsicRdRdy };
    assign  AsicRdBusy16 =   {{$bits(AsicRdBusy16)- $bits(AsicRdBusy){1'b0}},AsicRdBusy};
   
   LVDS_RECEIVER_MAX8 #(
   //LVDS_RECEIVER #(
		 .PAR_NASIC(PAR_NASIC),
		 .PAR_ISFLIP(PAR_ISFLIP_D_RD_RDY)
		 )
   LVDS_RECEIVER_RD_RDY (
		       .InputSignal_P(D_RD_RDY_P),
		       .InputSignal_N(D_RD_RDY_N),
		       .OutputSignal(AsicRdRdy)
		       );


   LVDS_RECEIVER_MAX8 #(
   //LVDS_RECEIVER #(
		 .PAR_NASIC(PAR_NASIC),
		 .PAR_ISFLIP(PAR_ISFLIP_D_RD_BUSY)
		 )
   LVDS_RECEIVER_RD_BUSY (
			.InputSignal_P(D_RD_BUSY_P),
			.InputSignal_N(D_RD_BUSY_N),
			.OutputSignal(AsicRdBusy)
			);
   //------------------------------------------------------------------------------
   //  Trigger Ready
   //------------------------------------------------------------------------------
    reg     TriggerReady;
   wire AsicReadTriggerStartTiming_SYSCLK;
   wire FpgaReadDoneTiming;
   
   SLIT_TRIGGER_READY
   SLIT_TRIGGER_READY(
  			    // Reset and Clock
  			    .Rst(SynchronousRst),
  			    .Clk(SystemClk),
  			    // Input
  			    .AsicReadTriggerTiming(AsicReadTriggerStartTiming_SYSCLK),
  			    .FpgaReadDoneTiming(FpgaReadDoneTiming),
  			    // Output
  			    .TriggerReady(TriggerReady)
   );

   //------------------------------------------------------------------------------
   //    Status Signal
   //------------------------------------------------------------------------------ 
   wire AsicGlobalWrDoneTiming;
   
   SLIT_ASIC_STATUS_SIGNAL #(
    .PAR_NASIC(PAR_NASIC)
   )
   SLIT_ASIC_STATUS_SIGNAL(
				    // Reset and Clock
				    .Rst(SynchronousRst),
				    .Clk(SystemClk),
				    // Input
				    .IsEnableAsic(IsEnableAsic[PAR_NASIC-1:0]),
				    .AsicRdRdy(AsicRdRdy),
				    .AsicRdBusy(AsicRdBusy),
				    // Output
				    .AsicGlobalWrDoneTiming(AsicGlobalWrDoneTiming) // SystemClk
				    );

   //------------------------------------------------------------------------------
   //    Write Trigger
   //------------------------------------------------------------------------------
   wire TEST_LEMO_IN_SRC;
   assign   TEST_LEMO_IN_SRC =   1'b0;

   wire LemoInputPolarity_SAMPLINGCLK;
   
      CLK_CONVERSION_FIFO_initial_one
      CLK_CONVERSION_FIFO_LemoInputPolarity
        (
         .Rst(AsynchronousRst),
         .WrClk(SystemClk),
         .RdClk(SamplingClk),
         .Din(LemoInputPolarity),
         .Dout(LemoInputPolarity_SAMPLINGCLK)
         );
         
    reg [1:0]   TEST_LEMO_IN_ARRAY_WR;
    always  @(posedge SamplingClk) begin
        TEST_LEMO_IN_ARRAY_WR  <=  {TEST_LEMO_IN_ARRAY_WR[0],TEST_LEMO_IN_SRC};
    end
    
  wire  AsicWrTrg;
   assign  AsicWrTrg  =   (~LemoInputPolarity_SAMPLINGCLK & TEST_LEMO_IN_ARRAY_WR[1]) || (LemoInputPolarity_SAMPLINGCLK & ~TEST_LEMO_IN_ARRAY_WR[1]);
 
 wire   [3:0]   OverFlowCounter;
wire    WriteTriggerStartTiming_SAMPLINGCLK; // used for test pulse
wire    AlwaysActiveAsicWriteTriggerStartTiming_SYSCLK; // used for event number
    //wire    TriggerStart;
   SLIT_WRITE_TRIGGER #(
				 .PAR_NASIC(PAR_NASIC),
				 .PAR_TRIGGER_WIDTH(16'd64),
				 .PAR_ISFLIP(PAR_ISFLIP_D_EXT_WR_START)
				 )
   SLIT_WRITE_TRIGGER(
			       // Reset and Clock
			       .Rst_SYSCLK(SynchronousRst),
			       .Clk(SamplingClk),
			       .SystemClk(SystemClk),
			       // Input
			       .TriggerReady_SYSCLK(TriggerReady),
                   .IsEnableAsic(IsEnableAsic[PAR_NASIC-1:0]),
                   // Hardware Trigger
			       .HardwareTriggerEnable_SYSCLK(HardwareWriteTriggerEnable),
			       .InputHardwareTrigger(AsicWrTrg),
			       .TriggerEdgeType_SYSCLK(TriggerEdgeType[1]),
                   .DelayHardwareTrigger(DelayHardwareWriteTrigger), // timing violation
                   // Software Trigger
			       .InputSoftwareTrigger_SYSCLK(SoftwareWriteTrigger),
			       // Cyclic Auto Trigger
			       .CyclicAutoTriggerEnable_SYSCLK(CyclicAutoWriteTriggerEnable),
			       .CyclicAutoTriggerInterval(CyclicAutoWriteTriggerInterval), // timing violation
			       // Output
			       .TriggerStart_P(D_EXT_WR_START_P),
			       .TriggerStart_N(D_EXT_WR_START_N),
			       .OverFlowCounter(OverFlowCounter),
			       //.TriggerStart(TriggerStart), // tmpppp
			       .TriggerStartTiming_SAMPLINGCLK(WriteTriggerStartTiming_SAMPLINGCLK),
			       .AlwaysActiveTriggerStartTiming_SYSCLK(AlwaysActiveAsicWriteTriggerStartTiming_SYSCLK)
			       );

   //------------------------------------------------------------------------------
   //    Test Pulse
   //------------------------------------------------------------------------------
   //wire [(PAR_NASIC-1)/4:0]   tmp_TESTPULSE_SWITCH;
   //assign   TESTPULSE_SWITCH    =   2'b00; 
   wire testPulseTiming;
   TESTPULSE    TESTPULSE(
        .Rst(SynchronousRst),
        .Clk(SamplingClk),
        .TestPulseEnable(TestPulseEnable),
        .TestPulseDelay(TestPulseDelay),
        .TestPulseInterval(TestPulseInterval),
        .WriteTriggerStartTiming(WriteTriggerStartTiming_SAMPLINGCLK),
        .TestPulseOutput(TESTPULSE_SWITCH),
        .TestPulseTiming(testPulseTiming)
    );
    
   //------------------------------------------------------------------------------
   //    Read Trigger
   //------------------------------------------------------------------------------
   wire LemoInputPolarity_READCLK;
   
      CLK_CONVERSION_FIFO_initial_one
      CLK_CONVERSION_FIFO_LemoInputPolarity_RD
        (
         .Rst(AsynchronousRst),
         .WrClk(SystemClk),
         .RdClk(ReadClk),
         .Din(LemoInputPolarity),
         .Dout(LemoInputPolarity_READCLK)
         );

    reg [1:0]   TEST_LEMO_IN_ARRAY_RD;
    always  @(posedge SystemClk) begin
        TEST_LEMO_IN_ARRAY_RD  <=  {TEST_LEMO_IN_ARRAY_RD[0],TEST_LEMO_IN_SRC};
    end
    
  wire  AsicRdTrg;
  assign  AsicRdTrg  =   (~LemoInputPolarity_READCLK & TEST_LEMO_IN_ARRAY_RD[1]) || (LemoInputPolarity_READCLK & ~TEST_LEMO_IN_ARRAY_RD[1]);

   SLIT_READ_TRIGGER #(
				.PAR_NASIC(PAR_NASIC),
				.PAR_TRIGGER_WIDTH(16'd64),
				.PAR_ISFLIP(PAR_ISFLIP_D_EXT_RD_START)
				)
   SLIT_READ_TRIGGER(
			      // Reset and Clock
			      .Rst(SynchronousRst),
			      .SystemClk(SystemClk),
			      // Input
			      .TriggerReady(TriggerReady),
                  .IsEnableAsic(IsEnableAsic[PAR_NASIC-1:0]),
                  // Hardware Trigger
			      .HardwareTriggerEnable(HardwareReadTriggerEnable),
			      .InputHardwareTrigger(AsicRdTrg),
			      .TriggerEdgeType(TriggerEdgeType[0]),
			      // Auto Trigger
			      .AutoTriggerEnable(AutoReadTriggerEnable),
			      .DelayHardwareAutoTrigger(DelayHardwareAutoReadTrigger),
			      .AsicGlobalWrDoneTiming(AsicGlobalWrDoneTiming), // SystemClk
			      // Software Trigger
			      .InputSoftwareTrigger(SoftwareReadTrigger),
			      // Output
			      .TriggerStart_P(D_EXT_RD_START_P),
			      .TriggerStart_N(D_EXT_RD_START_N),
			      .TriggerStartTiming(AsicReadTriggerStartTiming_SYSCLK)
			      );

   //------------------------------------------------------------------------------
   // Event Number
   //------------------------------------------------------------------------------
   wire  [23:0]  EventNumber;
   
   SliT_EVENT_NUMBER_COUNTER
     SliT_EVENT_NUMBER_COUNTER(
					// Reset and clock
					.Rst(SynchronousRst),
					.Clk(SystemClk),
					// Input
					.WriteTriggerStartTiming(AlwaysActiveAsicWriteTriggerStartTiming_SYSCLK),
					.ReadTriggerStartTiming(AsicReadTriggerStartTiming_SYSCLK),
					// Output
					.EventNumber(EventNumber)
					);

   //------------------------------------------------------------------------------
   // SliT
   //------------------------------------------------------------------------------
   wire [PAR_NASIC-1:0] TimingHeader;
   wire [PAR_NASIC-1:0] TimingEachData;
   wire [PAR_NASIC-1:0][15:0]   RawAddress_READCLK;
   wire [PAR_NASIC-1:0][127:0]  RawData_READCLK;
   //wire [PAR_NASIC-1:0] RawAddress_READCLK;
   //wire [PAR_NASIC-1:0] RawData_READCLK;
   wire [PAR_NASIC-1:0] FpgaSerC;
   
   SLIT_CHIP #(
			.PAR_NASIC(PAR_NASIC),
			.PAR_ISFLIP_D_FPGA_SER_A(PAR_ISFLIP_D_FPGA_SER_A),
			.PAR_ISFLIP_D_FPGA_SER_C(PAR_ISFLIP_D_FPGA_SER_C),
			.PAR_ISFLIP_D_FPGA_SER_D(PAR_ISFLIP_D_FPGA_SER_D)
			)
   SLIT_CHIP(
		      // Reset
		      .Rst(SynchronousRst),
		      .tmpClk(SystemClk),
		      // Input
		      .FpgaSerC_P(D_FPGA_SER_C_P),
		      .FpgaSerC_N(D_FPGA_SER_C_N),
		      .FpgaSerA_P(D_FPGA_SER_A_P),
		      .FpgaSerA_N(D_FPGA_SER_A_N),
		      .FpgaSerD_P(D_FPGA_SER_D_P),
		      .FpgaSerD_N(D_FPGA_SER_D_N),
		      // Output
		      .TimingHeader(TimingHeader),
		      .TimingEachData(TimingEachData),
		      .RawAddress(RawAddress_READCLK),
		      .RawData(RawData_READCLK),
              .FpgaSerC(FpgaSerC)
		      );

   //------------------------------------------------------------------------------
   // SliT FPGA
   //------------------------------------------------------------------------------
 wire    TCP_TX_FULL;
   wire    TCP_TX_WR;
   wire [7:0]   TCP_TX_DATA;
   SLIT_AIF #(
        	       .PAR_NASIC(PAR_NASIC),
        	       .PAR_NDATA_BIT(PAR_NDATA_BIT),
        	       .PAR_NDATA_NUM(PAR_NDATA_NUM)
        	       )
   SLIT_AIF(
		     // Reset and Clock
		     .Rst(SynchronousRst),
		     .ReadClk(FpgaSerC),
		     .SystemClk(SystemClk),
		     // Input data from ASIC
		     .TimingHeader(TimingHeader),
		     .TimingEachData(TimingEachData),
		     .RawAddress_READCLK(RawAddress_READCLK),
		     .RawData_READCLK(RawData_READCLK),
		     // Flag, Mode, Parameters
		     .ASIC_IDs(ASIC_IDs[PAR_NASIC-1:0]),
		     .EventNumber(EventNumber),
		     .IsEnableAsic(IsEnableAsic[PAR_NASIC-1:0]),
		     .IsEnableZeroSuppression(IsEnableZeroSuppression),
		     .ModeSelection(ModeSelection),
		     .IsEnableTimeWindow(IsEnableTimeWindow),
		     .TimeWindow_Cycle(TimeWindow_Cycle),
		     .TimeWindow_Start(TimeWindow_Start),
		     .TimeWindow_End(TimeWindow_End),
		     .NSubData(NSubData),
		     .AddressOutsideFrbs(AddressOutsideFrbs),
		     // Timing
             .AsicReadTriggerStartTiming(AsicReadTriggerStartTiming_SYSCLK),
             .TimingReadDone_LargeFifo(FpgaReadDoneTiming),
		     // FIFO for SiTCP
		     .TxFull(TCP_TX_FULL),
		     .SiTcpFifoValid(TCP_TX_WR),
		     .SiTcpFifoRdData(TCP_TX_DATA)
		     );

//------------------------------------------------------------------------------
//  SiTCP Core
//------------------------------------------------------------------------------
    wire            GMII_CLK        ;
    wire            GMII_TX_EN      ;
    wire    [7:0]   GMII_TXD        ;
    wire            GMII_TX_ER      ;
    wire    [7:0]   GMII_RXD        ;
    wire            GMII_RX_DV      ;
    wire            GMII_RX_ER      ;
    wire            TCP_CLOSE_REQ   ;
    wire            TIM_1US;
    wire            FORCE_DEFAULTn  ;
    wire    [31:0]  EXT_IP_ADDR      ;
    
    assign  FORCE_DEFAULTn =    1'b0;


    wire    [31:0]  DEFAULT_IP_ADDR ;
    assign  EXT_IP_ADDR =   32'h0;
    
    
    WRAP_SiTCP_GMII_XC7A_32K #(.TIM_PERIOD(8'd150)) SiTCP(
        .CLK                    (SystemClk  ), // in	: System Clock >129MHz
        .RST                    (AsynchronousRst), // in	: System reset
        .TIM_1US                (TIM_1US    ), // out   : 1 usec interval pulse
        // Configuration parameters
        .FORCE_DEFAULTn         (FORCE_DEFAULTn     ), // in	: Load default parameters
        .EXT_IP_ADDR            (EXT_IP_ADDR[31:0]   ), // in	: IP address[31:0]
        .EXT_TCP_PORT           (16'd0      ), // in	: TCP port #[15:0]
        .EXT_RBCP_PORT          (16'h0      ), // in	: RBCP port #[15:0]
        .PHY_ADDR               (5'd0       ), // in	: PHY-device MIF address[4:0]
        // EEPROM
        .EEPROM_CS              (EEPROM_CS), // out	: Chip select
        .EEPROM_SK              (EEPROM_SK), // out	: Serial data clock
        .EEPROM_DI              (EEPROM_DI), // out	: Serial write data
        .EEPROM_DO              (EEPROM_DO), // in	: Serial read data
        // MII interface
        .GMII_RSTn              (),	// out	: PHY reset
        .GMII_1000M             (1'b1),	// in	: GMII mode (0:MII, 1:GMII)
        // TX
        .GMII_TX_CLK            (GMII_CLK       ), // in	: Tx clock
        .GMII_TX_EN             (GMII_TX_EN     ), // out	: Tx enable
        .GMII_TXD               (GMII_TXD[7:0]  ), // out	: Tx data[7:0]
        .GMII_TX_ER             (GMII_TX_ER     ), // out	: TX error
        // RX
        .GMII_RX_CLK            (GMII_CLK       ), // in	: Rx clock
        .GMII_RX_DV             (GMII_RX_DV     ), // in	: Rx data valid
        .GMII_RXD               (GMII_RXD[7:0]  ), // in	: Rx data[7:0]
        .GMII_RX_ER             (GMII_RX_ER     ), // in	: Rx error
        .GMII_CRS               (1'b0), // in	: Carrier sense
        .GMII_COL               (1'b0), // in	: Collision detected
        // Management IF
        .GMII_MDC               (), // out	: Clock for MDIO
        .GMII_MDIO_IN           (1'b0), // in	: Data
        .GMII_MDIO_OUT          (), // out	: Data
        .GMII_MDIO_OE           (), // out	: MDIO output enable
        // User I/F
        .SiTCP_RST              (SiTCP_RST      ), // out	: Reset for SiTCP and related circuits
        // TCP connection control
        .TCP_OPEN_REQ           (1'b0), // in	: Reserved input, shoud be 0
        .TCP_OPEN_ACK           (TCP_OPEN_ACK       ), // out	: Acknowledge for open (=Socket busy)
        .TCP_ERROR              (), // out	: TCP error, its active period is equal to MSL
        .TCP_CLOSE_REQ          (TCP_CLOSE_REQ), // out	: Connection close request
        .TCP_CLOSE_ACK          (TCP_CLOSE_REQ), // in	: Acknowledge for closing
        // FIFO I/F
        .TCP_RX_WC              (16'hFFFF), // in	: Rx FIFO write count[15:0] (Unused bits should be set 1)
        .TCP_RX_WR              (), // out	: Write enable
        .TCP_RX_DATA            (), // out	: Write data[7:0]
        .TCP_TX_FULL            (TCP_TX_FULL        ), // out	: Almost full flag
        .TCP_TX_WR              (TCP_TX_WR          ), // in	: Write enable
        .TCP_TX_DATA            (TCP_TX_DATA[7:0]   ), // in	: Write data[7:0]
        // RBCP
        .RBCP_ACT               (RBCP_ACT           ), // out	: RBCP active
        .RBCP_ADDR              (RBCP_ADDR[31:0]    ), // out	: Address[31:0]
        .RBCP_WD                (RBCP_WD[7:0]       ), // out	: Data[7:0]
        .RBCP_WE                (RBCP_WE            ), // out	: Write enable
        .RBCP_RE                (RBCP_RE            ), // out	: Read enable
        .RBCP_ACK               (RBCP_ACK           ), // in	: Access acknowledge
        .RBCP_RD                (RBCP_RD[7:0]       ), // in	: Read data[7:0]
        .DEFAULT_IP_ADDR        (DEFAULT_IP_ADDR[31:0])
    );

//------------------------------------------------------------------------------
//  PCS/PMA Core
//------------------------------------------------------------------------------
    wire    mdc             ;
    wire    mdio_out        ;
    wire    mii_complete    ;
    wire    gmii_isolate    ;

    mii_initializer     mii_initializer(
        // System
        .CLK        (SystemClk  ),  // in : system clock (125M)
        .RST        (SiTCP_RST  ), // in : system reset
        // PHY
        .PHYAD      (5'd0       ), // in : [4:0] PHY address
        // MII
        .MDC        (mdc        ), // out: clock (1/128 system clock)
        .MDIO_OUT   (mdio_out   ), // out: connect this to "PCS/PMA + RocketIO" module .mdio?_i()
        // status
        .COMPLETE   (mii_complete)  // out: initializing sequence has completed (active H)
    );

    wire    gmii_rx_clk;
    BUFGCE BUF_SGMII(.O(GMII_CLK), .CE(1'b1), .I(gmii_rx_clk));
    
    wire    gt_pll0outclk;
    wire    gt_pll0outrefclk;
    wire    gt_pll1outclk;
    wire    gt_pll1outrefclk;
    wire    gt_pll0lock;
    wire    gt_pll0refclklost;
    wire    gt_pll1lock;
    wire    gt_pll1refclklost;
    wire    gt_pll1reset;
    wire    gt_pll1pd;
    wire    gtrefclk;


wire    [15:0]  CFG_REG;
assign CFG_REG[15:0] = 16'b0000_0000_0010_0000; // Use only BASE-X configulation

gig_ethernet_pcs_pma_1_example_design   gig_ethernet_pcs_pma_1_example_design(
    .independent_clock_bufg     (SamplingClk),  // input wire independent_clock_bufg
    .gtrefclk_p                 (GTP_REFCLK_P), // input wire gtrefclk_p
    .gtrefclk_n                 (GTP_REFCLK_N), // input wire gtrefclk_n
    .rxuserclk2                 (),             // output wire rxuserclk2_out
    .txn                        (SFP_TX_N),     // output wire txn
    .txp                        (SFP_TX_P),     // output wire txp
    .rxn                        (SFP_RX_N),     // input wire rxn
    .rxp                        (SFP_RX_P),     // input wire rxp
    .gmii_tx_clk                (),
    .gmii_rx_clk                (gmii_rx_clk),
    .gmii_txd                   (GMII_TXD[7:0]),// input wire [7 : 0] gmii_txd
    .gmii_tx_en                 (GMII_TX_EN),   // input wire gmii_tx_en
    .gmii_tx_er                 (GMII_TX_ER),   // input wire gmii_tx_er
    .gmii_rxd                   (GMII_RXD[7:0]),// output wire [7 : 0] gmii_rxd
    .gmii_rx_dv                 (GMII_RX_DV),   // output wire gmii_rx_dv
    .gmii_rx_er                 (GMII_RX_ER),   // output wire gmii_rx_er
    .configuration_vector       (5'd0),         // input wire [4 : 0] configuration_vector
    .an_interrupt               (),             // output wire an_interrupt
    .an_adv_config_vector       (CFG_REG[15:0]),// input wire [15 : 0] an_adv_config_vector
    .an_restart_config          (1'b0),         // input wire an_restart_config
    .status_vector              (),             // output wire [15 : 0] status_vector
    .reset                      (AsynchronousRst),                                    // input wire reset
    .signal_detect              (1'b1),          // input wire signal_detect
    .gtrefclk_out               (gtrefclk),
    .gt0_pll1reset              (gt_pll1reset),  // PLL reset for GTP
    .gt0_pll1pd                  (gt_pll1pd),
    .gt0_pll0outclk_out         (gt_pll0outclk),          // output wire gt0_pll0outclk_out
    .gt0_pll0outrefclk_out      (gt_pll0outrefclk),    // output wire gt0_pll0outrefclk_out
    .gt0_pll1outclk_out         (gt_pll1outclk),          // output wire gt0_pll1outclk_out
    .gt0_pll1outrefclk_out      (gt_pll1outrefclk),    // output wire gt0_pll1outrefclk_out
    .gt0_pll0lock_out           (gt_pll0lock),              // output wire gt0_pll0lock_out
    .gt0_pll0refclklost_out     (gt_pll0refclklost),  // output wire gt0_pll0refclklost_out
    .gt0_pll1lock_out           (gt_pll1lock),              // output wire gt0_pll1lock_out
    .gt0_pll1refclklost_out     (gt_pll1refclklost)  // output wire gt0_pll1refclklost_out
);

   //------------------------------------------------------------------------------//*
/*
   ILA_TOP ILA_TOP (
       .clk(SystemClk), // input wire clk
       .probe0(TCP_OPEN_ACK),
       .probe1(TriggerReady),
       //.probe2(AlwaysActiveAsicWriteTriggerStartTiming_SYSCLK),
       .probe2(WriteTriggerStartTiming_SAMPLINGCLK),
       .probe3(AsicReadTriggerStartTiming_SYSCLK),
       .probe4(FpgaReadDoneTiming),
       //.probe6(AsicGlobalWrDoneTiming),
       .probe5(EventNumber[3:0]),
       .probe6(testPulseTiming),
       .probe7(TESTPULSE_SWITCH[(PAR_NASIC-1)/4:0]),
       .probe8(AsicRdRdy[PAR_NASIC-1:0]),
       .probe9(AsicRdBusy[PAR_NASIC-1:0]),
       .probe10(TimingHeader[PAR_NASIC-1:0]),
       .probe11(TimingEachData[PAR_NASIC-1:0])
   );
 */
   //assign   TEST_PIN    [0] =   TESTPULSE_SWITCH[0];
   //assign   TEST_PIN    [0] =   AsicRdRdy[1];
   //assign   TEST_PIN    [1] =   AsicRdBusy[0];
   //assign   TEST_PIN    [1] =   AsicRdBusy[1];
   wire [31:0]  TestSignalList;
   assign   TestSignalList = {AsicRdRdy16[7:0],AsicRdBusy16[7:0],
                              9'b0,
                              EventNumber[3:0],
                              TESTPULSE_SWITCH[0],TCP_OPEN_ACK,TriggerReady};   
   SLIT_TEST_PIN
   SLIT_TEST_PIN(
        .Clk(SystemClk),
        .SelectTestSignal(SelectTestSignal),
        .TestSignalList(TestSignalList),
        .TestPin(TEST_PIN)
   );

   //------------------------------------------------------------------------------
/*
      ILA_TOP_SC (
       .clk(SystemClk), // input wire clk
       .probe0(SiTCP_RST),
       .probe1(TIM_1US_SC),
       .probe2(SIO_ACT),
       .probe3(RBCP_ACT),
       .probe4(RBCP_ADDR[31:0]),
       .probe5(RBCP_WD[7:0]),
       .probe6(RBCP_RE),
       .probe7(RBCP_SIO_RD[0][7:0]),
       .probe8(RBCP_REG_ACK),
       .probe9(RBCP_REG_RD[7:0]),
       .probe10(RBCP_ACK),
       .probe11(RBCP_RD[7:0]),
       .probe12(D_MSCK[((PAR_NASIC-1)/4):0]),
       .probe13(tmp_D_MSCK[PAR_NMAXASIC-1:0]),
       .probe14(SlowCtrlSelect[PAR_NMAXASIC-1:0]),
       .probe15(D_MSCS[PAR_NASIC-1:0]),
       .probe16(D_MSO[PAR_NASIC-1:0]),
       .probe17(D_MSI[PAR_NASIC-1:0]),
       .probe18(RBCP_SIO_ACK[PAR_NASIC-1:0]),
       .probe19(RBCP_SIO_ACK_RD0[PAR_NASIC-1:0]),
       .probe20(RBCP_SIO_ACK_RD1[PAR_NASIC-1:0]),
       .probe21(RBCP_SIO_ACK_RD2[PAR_NASIC-1:0]),
       .probe22(RBCP_SIO_ACK_RD3[PAR_NASIC-1:0]),
       .probe23(RBCP_SIO_ACK_RD4[PAR_NASIC-1:0]),
       .probe24(RBCP_SIO_ACK_RD5[PAR_NASIC-1:0]),
       .probe25(RBCP_SIO_ACK_RD6[PAR_NASIC-1:0]),
       .probe26(RBCP_SIO_ACK_RD7[PAR_NASIC-1:0])
   );
*/


wire JC_IN_CLK;
wire JC_CLK;

    LVDS_TRANSMITTER #(
        .PAR_NASIC(1),
        .PAR_ISFLIP(16'b0)
    )
    LVDS_TRANSMITTER_TO_JC(
        //.InputSignal(JC_IN_CLK),
        .InputSignal(SamplingClk),
        //.InputSignal(MOD_CLK),
        .OutputSignal_P(JC_IN_CLK_P),
        .OutputSignal_N(JC_IN_CLK_N)
    );

    LVDS_RECEIVER #(
        .PAR_NASIC(1),
        .PAR_ISFLIP(16'b0)
    )
    LVDS_RECEIVER_FROM_JC(
        .InputSignal_P(JC_OUT_CLK_P),
        .InputSignal_N(JC_OUT_CLK_N),
        .OutputSignal(JC_CLK)
    );

    JC_COMMAND_GEN JC_COMMAND_GEN(
        .CLK_in (JC_SPI_CLK),
        .SystemClk  (SystemClk),
        
        .LOC_ADDR   (RBCP_ADDR[31:0]),
        .LOC_WD     (RBCP_WD[7:0]),
        .LOC_WE     (RBCP_WE),
        .LOC_RE     (RBCP_RE),
        .LOC_ACK    (RBCP_JC_ACK),
        .LOC_RD     (RBCP_JC_RD[7:0]),
        
        .SPI_CS     (JC_SPI_CS),
        .SPI_SDI    (JC_SPI_SDI),
        .SPI_SDO    (JC_SPI_SDO),
        .reset      (JC_RSTB),
        .OEB        (JC_OEB),
        .LOLB       (JC_LOLB),
        .INTRB      (JC_INTRB),
        .LOS_XAXBB  (JC_LOS_XAXBB)
    );


wire GTP_reset;
wire GTP_TX_reset;
wire GTP_RX_reset;
wire [15:0] gt_txdata_in;
wire [1:0] gt_txcharisk_in;
wire gt_txusrclk_out;
wire [15:0] gt_rxdata_out;
wire [1:0] gt_rxcharisk_out;
wire gt_rxusrclk_out;
wire [5:0] gt_status;
wire [15:0] error_counter;
wire [15:0] gt_tx_usr_data;  
wire [63:0] RX_paralell_data;

    GTP_2p5Gbps_exdes   GTP_2p5Gbps_exdes(
    .reset_in                   (GTP_reset),
    .gtrefclk_in                (gtrefclk),
    .gt_pll0outclk              (gt_pll0outclk),
    .gt_pll0outrefclk           (gt_pll0outrefclk),
    .gt_pll1outclk              (gt_pll1outclk),
    .gt_pll1outrefclk           (gt_pll1outrefclk),
    .gt_pll0lock                (gt_pll0lock),
    .gt_pll0refclklost          (gt_pll0refclklost),
    .gt_pll1lock                (gt_pll1lock),
    .gt_pll1refclklost          (gt_pll1refclklost),
    .DRP_CLK_IN                 (SystemClk),
    .GTTX_RESET_IN              (GTP_TX_reset),
    .GTRX_RESET_IN              (GTP_RX_reset),
    .RXN_IN                     (RXN_IN),
    .RXP_IN                     (RXP_IN),
    .TXN_OUT                    (TXN_OUT),
    .TXP_OUT                    (TXP_OUT),
    .gt_txdata_in               (gt_txdata_in),
    .gt_txcharisk_in            (gt_txcharisk_in),
    .gt_txusrclk_out            (gt_txusrclk_out),
    .gt_rxdata_out              (gt_rxdata_out),
    .gt_rxcharisk_out           (gt_rxcharisk_out),
    .gt_rxusrclk_out            (gt_rxusrclk_out),
    .gt_pll1reset_out           (gt_pll1reset),
    .gt_pll1pd_out              (gt_pll1pd),
    .gt_status_out              (gt_status),
    .error_counter              (error_counter)
    );

    GTP_TX  GTP_TX(
        .gt_txusrclk_in (gt_txusrclk_out),
        .reset_in       (GTP_TX_reset),
        .gt_tx_usr_data (gt_tx_usr_data),
        .gt_txdata      (gt_txdata_in),
        .gt_txcharisk   (gt_txcharisk_in)
    );
 
 wire wr_en;
 wire [7:0] WR_DATA;
 wire ReadStop;

 assign COMMUNICATE_BTW_FPGA_TX[3] = prog_full;

    GTP_RX  GTP_RX(
        .gt_rxusrclk_in (gt_rxusrclk_out),
        .reset_in       (GTP_RX_reset),
        .gt_rxdata      (gt_rxdata_out),
        .gt_rxcharisk   (gt_rxcharisk_out),
        .COMMUNICATE_BTW_FPGA (COMMUNICATE_BTW_FPGA_TX[3:0]),
        .wr_en (wr_en),
        .WR_DATA (WR_DATA)
    );
    
    wire prog_full;
    wire valid;
    wire rd_en;
    wire empty;
    assign  rd_en   = ~empty && ~ReadStop;
    wire    [7:0]   dout;
    wire full;
    wire wr_ack;

    fifo_w8_d256 fifo_w8_d256 (
      .clk(gt_rxusrclk_out),              // input wire clk
      .srst(GTP_RX_reset),            // input wire srst
      .din(WR_DATA),              // input wire [7 : 0] din
      .wr_en(wr_en),          // input wire wr_en
      .rd_en(rd_en),          // input wire rd_en
      .dout(dout),            // output wire [7 : 0] dout
      .full(full),            // output wire full
      .wr_ack(wr_ack),        // output wire wr_ack
      .empty(empty),          // output wire empty
      .valid(valid),          // output wire valid
      .prog_full(prog_full)  // output wire prog_full
    );

    ila_SiTCP_FIFO ila_SiTCP_FIFO(
        .clk(gt_rxusrclk_out), // input wire clk
    
    
        .probe0(WR_DATA[7:0]), // input wire [7:0]  probe0  
        .probe1(wr_en), // input wire [0:0]  probe1 
        .probe2(wr_ack), // input wire [0:0]  probe2 
        .probe3(prog_full), // input wire [0:0]  probe3 
        .probe4(COMMUNICATE_BTW_FPGA_TX[3]) // input wire [0:0]  probe4
    );
    
    vio_TOP your_instance_name (
      .clk(gt_rxusrclk_out),                // input wire clk
      .probe_out0(ReadStop)  // output wire [0 : 0] probe_out0
    );
      
    GTP_control     GTP_control(
		.CLK                      (SystemClk          ),   // in   : Clock
		.LOC_ADDR                 (RBCP_ADDR[31:0]    ),
		.LOC_WD                   (RBCP_WD[7:0]       ),
		.LOC_WE                   (RBCP_WE            ),
		.LOC_RE                   (RBCP_RE            ),
		.LOC_ACK                  (RBCP_GTP_ACK       ),
		.LOC_RD                   (RBCP_GTP_RD        ),
		.gt_status                (gt_status          ),
		.error_counter            (error_counter      ),
		.GTP_reset                (GTP_reset          ),
		.GTP_TX_reset             (GTP_TX_reset       ),
		.GTP_RX_reset             (GTP_RX_reset       ),
		.gt_tx_usr_data           (gt_tx_usr_data)
		);

    ADC_monitoring  ADC_monitoring(
        .CLK_in         (SystemClk),
        .reset_in       (AsynchronousRst),
        .enable_in      (ADC_mon_enb),
        .daddr_in       (ADC_mon_addr),
        .monitor_out    (ADC_mon_data)
    );
    
    assign MOD_ABS = 1'b0;

    I2C_protocol    I2C_protocol(
        .CLK_in     (SystemClk),
        .reset_in   (I2C_reset),
        .start      (I2C_start),
        .address    (I2C_address),
        .SCL        (SCL),
        .SDA        (SDA),
        .error_out  (I2C_error),
        .data_out   (I2C_data)
    );
    
    
endmodule
