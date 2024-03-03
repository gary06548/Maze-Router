module MRA(
	// CHIP IO
	clk            	,	
	rst_n          	,	
	in_valid       	,	
	frame_id        ,	
	net_id         	,	  
	loc_x          	,	  
        loc_y         	,
	cost		,		
	busy         	,

    // AXI4 IO
	     arid_m_inf,
	   araddr_m_inf,
	    arlen_m_inf,
	   arsize_m_inf,
	  arburst_m_inf,
	  arvalid_m_inf,
	  arready_m_inf,
	
	      rid_m_inf,
	    rdata_m_inf,
	    rresp_m_inf,
	    rlast_m_inf,
	   rvalid_m_inf,
	   rready_m_inf,
	
	     awid_m_inf,
	   awaddr_m_inf,
	   awsize_m_inf,
	  awburst_m_inf,
	    awlen_m_inf,
	  awvalid_m_inf,
	  awready_m_inf,
	
	    wdata_m_inf,
	    wlast_m_inf,
	   wvalid_m_inf,
	   wready_m_inf,
	
	      bid_m_inf,
	    bresp_m_inf,
	   bvalid_m_inf,
	   bready_m_inf
);

// ===============================================================
//  					Parameter Declaration 
// ===============================================================
parameter ID_WIDTH=4, DATA_WIDTH=128, ADDR_WIDTH=32;    // DO NOT modify AXI4 Parameter
//parameter NUM_ROW = 64, NUM_COLUMN = 64; 				
//parameter MAX_NUM_MACRO = 15;

// ===============================================================
//  					Input / Output 
// ===============================================================

// << CHIP io port with system >>
input 		clk,rst_n;
input 		in_valid;
input  [4:0] 	frame_id;
input  [3:0]    net_id;     
input  [5:0]    loc_x; 
input  [5:0]    loc_y; 
output [13:0] 	cost;
output          busy;       
  
// AXI Interface wire connecttion for pseudo DRAM read/write

// ------------------------
// <<<<< AXI READ >>>>>
// ------------------------
// (1)	axi read address channel 
output wire [ID_WIDTH-1:0]      arid_m_inf;
output wire [1:0]            arburst_m_inf;
output wire [2:0]             arsize_m_inf;
output wire [7:0]              arlen_m_inf;
output wire                  arvalid_m_inf;
input  wire                  arready_m_inf;
output wire [ADDR_WIDTH-1:0]  araddr_m_inf;
// ------------------------
// (2)	axi read data channel 
input  wire [ID_WIDTH-1:0]       rid_m_inf;
input  wire                   rvalid_m_inf;
output wire                   rready_m_inf;
input  wire [DATA_WIDTH-1:0]   rdata_m_inf;
input  wire                    rlast_m_inf;
input  wire [1:0]              rresp_m_inf;
// ------------------------
// <<<<< AXI WRITE >>>>>
// ------------------------
// (1) 	axi write address channel 
output wire [ID_WIDTH-1:0]      awid_m_inf;
output wire [1:0]            awburst_m_inf;
output wire [2:0]             awsize_m_inf;
output wire [7:0]              awlen_m_inf;
output wire                  awvalid_m_inf;
input  wire                  awready_m_inf;
output wire [ADDR_WIDTH-1:0]  awaddr_m_inf;
// -------------------------
// (2)	axi write data channel 
output wire                   wvalid_m_inf;
input  wire                   wready_m_inf;
output wire [DATA_WIDTH-1:0]   wdata_m_inf;
output reg                     wlast_m_inf;
// -------------------------
// (3)	axi write response channel 
input  wire  [ID_WIDTH-1:0]      bid_m_inf;
input  wire                   bvalid_m_inf;
output wire                   bready_m_inf;
input  wire  [1:0]             bresp_m_inf;
// -----------------------------

//----- DRAM read constant -----
assign arid_m_inf    = 4'd0; 	// id = 0
assign arburst_m_inf = 2'b01;	// incr mode
assign arsize_m_inf  = 3'b100;	// size = 16 bytes 
assign arlen_m_inf   = 8'd127;
//assign arlen_m_inf   = 8'd1;

//----- DRAM write constant -----
assign awid_m_inf    = 4'd0;    // id = 0
assign awburst_m_inf = 2'b01;   // incr mode
assign awsize_m_inf  = 3'b100;  // size = 16 bytes
assign awlen_m_inf   = 8'd127;
//assign awlen_m_inf   = 8'd1;


wire fill_done, retrace_done;
wire [3:0] net_num, cur_net;

//==============================================//
//                     FSM                      //
//==============================================//
localparam Idle    = 3'd0;
localparam DRAM_r  = 3'd1;
localparam Fill    = 3'd2;
localparam Retrace = 3'd3;
localparam DRAM_w  = 3'd4;

reg [2:0] c_state, n_state;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) c_state <= Idle;
    else        c_state <= n_state;
end

always @* begin
  	case (c_state)
        Idle: begin
            if (in_valid)    n_state = DRAM_r;
            else             n_state = Idle;
        end
        DRAM_r: begin
            if (rlast_m_inf) n_state = Fill;
            else             n_state = DRAM_r;
        end
        Fill: begin
            if (fill_done)   n_state = Retrace;
            else             n_state = Fill;
        end
        Retrace: begin
            if(retrace_done) begin
                if(cur_net == net_num)
                    n_state = DRAM_w;
                else
                    n_state = Fill;
            end
            else begin
                n_state = Retrace;
            end
        end
        DRAM_w: begin
            if (wlast_m_inf) n_state = Idle;
            else             n_state = DRAM_w;
        end
        default: begin
            n_state = Idle;
        end
  	endcase
end

//==============================================//
//                   Design                     //
//==============================================//
reg in_valid_d1;
reg read_weight_finish;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) in_valid_d1 <= 1'b0;
    else        in_valid_d1 <= in_valid;
end

assign busy = (c_state != Idle) & (~in_valid_d1);

reg [4:0] s_frame_id;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        s_frame_id <= 5'b0;
    end
    else begin
        if (in_valid) begin
            s_frame_id <= frame_id;
        end
        else begin
            s_frame_id <= s_frame_id;
        end 
    end
end

reg bridge_valid;
reg [6:0] dram_read_cnt;
wire c_state_is_Idle    = (c_state == Idle);
wire c_state_is_DRAM_r  = (c_state == DRAM_r);
wire c_state_is_Fill    = (c_state == Fill);
wire c_state_is_Retrace = (c_state == Retrace);
wire c_state_is_DRAM_w  = (c_state == DRAM_w); 
wire done = &dram_read_cnt;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        wlast_m_inf <= 1'b0;
    end
    else begin
        if (c_state_is_DRAM_w) begin
            if (dram_read_cnt == 7'd126) 
                wlast_m_inf <= 1'b1;
            else
                wlast_m_inf <= 1'b0;
        end
        else begin
            wlast_m_inf <= 1'b0;
        end 
    end
end

reg first_fill;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        first_fill <= 1'b1;
    end
    else begin
        if (c_state_is_Retrace) 
            first_fill <= 1'b0;
        else if (c_state_is_Idle) 
            first_fill <= 1'b1;
        else 
            first_fill <= first_fill;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        bridge_valid <= 1'b0;
    end
    else begin
        if (c_state_is_DRAM_r) begin
            bridge_valid <= 1'b1;
        end
        else if ( (c_state_is_Fill && first_fill) || c_state_is_DRAM_w) begin
            if (done) bridge_valid <= 1'b0;
            else      bridge_valid <= 1'b1;
        end
        else begin
            bridge_valid <= 1'b0;
        end 
    end
end

wire retrace_WEB;
wire WEB_loc = ~(rvalid_m_inf & c_state_is_DRAM_r) & retrace_WEB; 
wire WEB_wei = ~(rvalid_m_inf & ~c_state_is_DRAM_r);
wire [127:0] out_loc, out_wei;
wire [5:0] nxt_retrace_x, nxt_retrace_y;

reg  [6:0] addr_loc, addr_wei;
wire [6:0] retrace_addr = (nxt_retrace_y << 1'b1) + nxt_retrace_x[5];
reg c_state_is_DRAM_w_d1, c_state_is_DRAM_w_d2;

always @(posedge clk) c_state_is_DRAM_w_d1 <= c_state_is_DRAM_w;
always @(posedge clk) c_state_is_DRAM_w_d2 <= c_state_is_DRAM_w_d1;

always @* begin
    if (c_state_is_Retrace)        addr_loc = retrace_addr; 
    else if (c_state_is_DRAM_w_d1) addr_loc = dram_read_cnt + wready_m_inf + 1'b1; 
    else if (c_state_is_DRAM_w)    addr_loc = 7'd0;
    else                           addr_loc = dram_read_cnt;
end

always @* begin
    if (read_weight_finish) addr_wei = retrace_addr; 
    else                    addr_wei = dram_read_cnt;
end

reg [127:0] wdata, first_wdata, nxt_wdata;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        first_wdata <= 128'd0;
    end
    else begin
        if (c_state_is_DRAM_w_d2) begin
            first_wdata <= first_wdata;
        end
        else if (c_state_is_DRAM_w_d1) begin
            first_wdata <= out_loc;
        end
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) nxt_wdata <= 128'd0;
    else        nxt_wdata <= out_loc;
end

always @* begin
    if (dram_read_cnt != 0) wdata = nxt_wdata;
    else                    wdata = first_wdata;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) dram_read_cnt <= 7'd0;
    else begin
        if (rvalid_m_inf || wready_m_inf) 
            dram_read_cnt <= dram_read_cnt + 1'b1;
        else              
            dram_read_cnt <= dram_read_cnt;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) read_weight_finish <= 1'b0;
    else begin
        if (rlast_m_inf && !c_state_is_DRAM_r) read_weight_finish <= 1'b1;
        else if (c_state_is_Idle)              read_weight_finish <= 1'b0;
        else                                   read_weight_finish <= read_weight_finish;
    end
end

wire [127:0] wb_data;
wire [127:0] in_loc = (c_state_is_DRAM_r)? rdata_m_inf : wb_data;
wire [127:0] in_wei = (c_state_is_Idle)? 128'd0 : rdata_m_inf; // use for avoiding hold time violation
wire bridge_mode = ~c_state_is_DRAM_w;

// ----------------------- <<< Write updated location map to DRAM >>> -----------------------
reg [31:0] addr_dram;
always @* begin
    if (c_state_is_DRAM_r)      addr_dram = {16'h0001, s_frame_id, 11'd0};
    else if (c_state_is_DRAM_w) addr_dram = {16'h0001, s_frame_id, 11'd0};
    else                        addr_dram = {16'h0002, s_frame_id, 11'd0};
end

BRIDGE #(.ID_WIDTH(ID_WIDTH), .ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH) ) BR1(
    // Input Signals
    .clk(clk), .rst_n(rst_n), .bridge_valid(bridge_valid),
    .addr_dram(addr_dram), .mode(bridge_mode), .wdata(wdata), 
    
    // DRAM Signals
    .arvalid_m_inf(arvalid_m_inf), .araddr_m_inf(araddr_m_inf), .rready_m_inf(rready_m_inf), 
    .awvalid_m_inf(awvalid_m_inf), .awaddr_m_inf(awaddr_m_inf), .wvalid_m_inf(wvalid_m_inf), 
    .wdata_m_inf(wdata_m_inf),     .bready_m_inf(bready_m_inf), .arready_m_inf(arready_m_inf), 
    .rvalid_m_inf(rvalid_m_inf),   .rresp_m_inf(rresp_m_inf),   .rdata_m_inf(rdata_m_inf), 
    .awready_m_inf(awready_m_inf), .wready_m_inf(wready_m_inf), .bvalid_m_inf(bvalid_m_inf), 
    .bresp_m_inf(bresp_m_inf),     .rlast_m_inf(rlast_m_inf),   .wlast_m_inf(wlast_m_inf)
);

PATH_MAP PM1(.clk(clk), .rst_n(rst_n), .in_valid(in_valid), .c_state_is_DRAM_r(c_state_is_DRAM_r), 
             .c_state_is_Fill(c_state_is_Fill), .c_state_is_Retrace(c_state_is_Retrace), 
             .read_weight_finish(read_weight_finish), .net_id(net_id), .loc_x(loc_x), .loc_y(loc_y),
             .nxt_retrace_x(nxt_retrace_x), .nxt_retrace_y(nxt_retrace_y),
             .dram_read_cnt(dram_read_cnt), .rdata_m_inf(rdata_m_inf), .out_loc(out_loc), .out_wei(out_wei), .fill_done(fill_done), 
             .retrace_done(retrace_done), .retrace_WEB(retrace_WEB), .net_num(net_num), .cur_net(cur_net), .wb_data(wb_data), 
             .cost(cost));

Location_Map_128x128 LM1( .A0(addr_loc[0]), .A1(addr_loc[1]), .A2(addr_loc[2]),
                          .A3(addr_loc[3]), .A4(addr_loc[4]), .A5(addr_loc[5]), .A6(addr_loc[6]),
                          
                          .DO0(out_loc[0]),     .DO1(out_loc[1]),     .DO2(out_loc[2]),     .DO3(out_loc[3]),     .DO4(out_loc[4]),
                          .DO5(out_loc[5]),     .DO6(out_loc[6]),     .DO7(out_loc[7]),     .DO8(out_loc[8]),     .DO9(out_loc[9]),
                          .DO10(out_loc[10]),   .DO11(out_loc[11]),   .DO12(out_loc[12]),   .DO13(out_loc[13]),   .DO14(out_loc[14]),
                          .DO15(out_loc[15]),   .DO16(out_loc[16]),   .DO17(out_loc[17]),   .DO18(out_loc[18]),   .DO19(out_loc[19]),
                          .DO20(out_loc[20]),   .DO21(out_loc[21]),   .DO22(out_loc[22]),   .DO23(out_loc[23]),   .DO24(out_loc[24]),
                          .DO25(out_loc[25]),   .DO26(out_loc[26]),   .DO27(out_loc[27]),   .DO28(out_loc[28]),   .DO29(out_loc[29]),
                          .DO30(out_loc[30]),   .DO31(out_loc[31]),   .DO32(out_loc[32]),   .DO33(out_loc[33]),   .DO34(out_loc[34]),
                          .DO35(out_loc[35]),   .DO36(out_loc[36]),   .DO37(out_loc[37]),   .DO38(out_loc[38]),   .DO39(out_loc[39]),
                          .DO40(out_loc[40]),   .DO41(out_loc[41]),   .DO42(out_loc[42]),   .DO43(out_loc[43]),   .DO44(out_loc[44]),
                          .DO45(out_loc[45]),   .DO46(out_loc[46]),   .DO47(out_loc[47]),   .DO48(out_loc[48]),   .DO49(out_loc[49]),
                          .DO50(out_loc[50]),   .DO51(out_loc[51]),   .DO52(out_loc[52]),   .DO53(out_loc[53]),   .DO54(out_loc[54]),
                          .DO55(out_loc[55]),   .DO56(out_loc[56]),   .DO57(out_loc[57]),   .DO58(out_loc[58]),   .DO59(out_loc[59]),
                          .DO60(out_loc[60]),   .DO61(out_loc[61]),   .DO62(out_loc[62]),   .DO63(out_loc[63]),   .DO64(out_loc[64]),
                          .DO65(out_loc[65]),   .DO66(out_loc[66]),   .DO67(out_loc[67]),   .DO68(out_loc[68]),   .DO69(out_loc[69]),
                          .DO70(out_loc[70]),   .DO71(out_loc[71]),   .DO72(out_loc[72]),   .DO73(out_loc[73]),   .DO74(out_loc[74]),
                          .DO75(out_loc[75]),   .DO76(out_loc[76]),   .DO77(out_loc[77]),   .DO78(out_loc[78]),   .DO79(out_loc[79]),
                          .DO80(out_loc[80]),   .DO81(out_loc[81]),   .DO82(out_loc[82]),   .DO83(out_loc[83]),   .DO84(out_loc[84]),
                          .DO85(out_loc[85]),   .DO86(out_loc[86]),   .DO87(out_loc[87]),   .DO88(out_loc[88]),   .DO89(out_loc[89]),
                          .DO90(out_loc[90]),   .DO91(out_loc[91]),   .DO92(out_loc[92]),   .DO93(out_loc[93]),   .DO94(out_loc[94]),
                          .DO95(out_loc[95]),   .DO96(out_loc[96]),   .DO97(out_loc[97]),   .DO98(out_loc[98]),   .DO99(out_loc[99]),
                          .DO100(out_loc[100]), .DO101(out_loc[101]), .DO102(out_loc[102]), .DO103(out_loc[103]), .DO104(out_loc[104]),
                          .DO105(out_loc[105]), .DO106(out_loc[106]), .DO107(out_loc[107]), .DO108(out_loc[108]), .DO109(out_loc[109]),
                          .DO110(out_loc[110]), .DO111(out_loc[111]), .DO112(out_loc[112]), .DO113(out_loc[113]), .DO114(out_loc[114]),
                          .DO115(out_loc[115]), .DO116(out_loc[116]), .DO117(out_loc[117]), .DO118(out_loc[118]), .DO119(out_loc[119]),
                          .DO120(out_loc[120]), .DO121(out_loc[121]), .DO122(out_loc[122]), .DO123(out_loc[123]), .DO124(out_loc[124]),
                          .DO125(out_loc[125]), .DO126(out_loc[126]), .DO127(out_loc[127]), 
                          
                          .DI0(in_loc[0]),     .DI1(in_loc[1]),     .DI2(in_loc[2]),     .DI3(in_loc[3]),     .DI4(in_loc[4]),
                          .DI5(in_loc[5]),     .DI6(in_loc[6]),     .DI7(in_loc[7]),     .DI8(in_loc[8]),     .DI9(in_loc[9]),
                          .DI10(in_loc[10]),   .DI11(in_loc[11]),   .DI12(in_loc[12]),   .DI13(in_loc[13]),   .DI14(in_loc[14]),
                          .DI15(in_loc[15]),   .DI16(in_loc[16]),   .DI17(in_loc[17]),   .DI18(in_loc[18]),   .DI19(in_loc[19]),
                          .DI20(in_loc[20]),   .DI21(in_loc[21]),   .DI22(in_loc[22]),   .DI23(in_loc[23]),   .DI24(in_loc[24]),
                          .DI25(in_loc[25]),   .DI26(in_loc[26]),   .DI27(in_loc[27]),   .DI28(in_loc[28]),   .DI29(in_loc[29]),
                          .DI30(in_loc[30]),   .DI31(in_loc[31]),   .DI32(in_loc[32]),   .DI33(in_loc[33]),   .DI34(in_loc[34]),
                          .DI35(in_loc[35]),   .DI36(in_loc[36]),   .DI37(in_loc[37]),   .DI38(in_loc[38]),   .DI39(in_loc[39]),
                          .DI40(in_loc[40]),   .DI41(in_loc[41]),   .DI42(in_loc[42]),   .DI43(in_loc[43]),   .DI44(in_loc[44]),
                          .DI45(in_loc[45]),   .DI46(in_loc[46]),   .DI47(in_loc[47]),   .DI48(in_loc[48]),   .DI49(in_loc[49]),
                          .DI50(in_loc[50]),   .DI51(in_loc[51]),   .DI52(in_loc[52]),   .DI53(in_loc[53]),   .DI54(in_loc[54]),
                          .DI55(in_loc[55]),   .DI56(in_loc[56]),   .DI57(in_loc[57]),   .DI58(in_loc[58]),   .DI59(in_loc[59]),
                          .DI60(in_loc[60]),   .DI61(in_loc[61]),   .DI62(in_loc[62]),   .DI63(in_loc[63]),   .DI64(in_loc[64]),
                          .DI65(in_loc[65]),   .DI66(in_loc[66]),   .DI67(in_loc[67]),   .DI68(in_loc[68]),   .DI69(in_loc[69]),
                          .DI70(in_loc[70]),   .DI71(in_loc[71]),   .DI72(in_loc[72]),   .DI73(in_loc[73]),   .DI74(in_loc[74]),
                          .DI75(in_loc[75]),   .DI76(in_loc[76]),   .DI77(in_loc[77]),   .DI78(in_loc[78]),   .DI79(in_loc[79]),
                          .DI80(in_loc[80]),   .DI81(in_loc[81]),   .DI82(in_loc[82]),   .DI83(in_loc[83]),   .DI84(in_loc[84]),
                          .DI85(in_loc[85]),   .DI86(in_loc[86]),   .DI87(in_loc[87]),   .DI88(in_loc[88]),   .DI89(in_loc[89]),
                          .DI90(in_loc[90]),   .DI91(in_loc[91]),   .DI92(in_loc[92]),   .DI93(in_loc[93]),   .DI94(in_loc[94]),
                          .DI95(in_loc[95]),   .DI96(in_loc[96]),   .DI97(in_loc[97]),   .DI98(in_loc[98]),   .DI99(in_loc[99]),
                          .DI100(in_loc[100]), .DI101(in_loc[101]), .DI102(in_loc[102]), .DI103(in_loc[103]), .DI104(in_loc[104]),
                          .DI105(in_loc[105]), .DI106(in_loc[106]), .DI107(in_loc[107]), .DI108(in_loc[108]), .DI109(in_loc[109]),
                          .DI110(in_loc[110]), .DI111(in_loc[111]), .DI112(in_loc[112]), .DI113(in_loc[113]), .DI114(in_loc[114]),
                          .DI115(in_loc[115]), .DI116(in_loc[116]), .DI117(in_loc[117]), .DI118(in_loc[118]), .DI119(in_loc[119]),
                          .DI120(in_loc[120]), .DI121(in_loc[121]), .DI122(in_loc[122]), .DI123(in_loc[123]), .DI124(in_loc[124]),
                          .DI125(in_loc[125]), .DI126(in_loc[126]), .DI127(in_loc[127]), 
                          .CK(clk), .WEB(WEB_loc), .OE(1'b1), .CS(1'b1));

Weight_Map_128x128 WM1( .A0(addr_wei[0]), .A1(addr_wei[1]), .A2(addr_wei[2]),
                        .A3(addr_wei[3]), .A4(addr_wei[4]), .A5(addr_wei[5]), .A6(addr_wei[6]),
                        
                        .DO0(out_wei[0]),     .DO1(out_wei[1]),     .DO2(out_wei[2]),     .DO3(out_wei[3]),     .DO4(out_wei[4]),
                        .DO5(out_wei[5]),     .DO6(out_wei[6]),     .DO7(out_wei[7]),     .DO8(out_wei[8]),     .DO9(out_wei[9]),
                        .DO10(out_wei[10]),   .DO11(out_wei[11]),   .DO12(out_wei[12]),   .DO13(out_wei[13]),   .DO14(out_wei[14]),
                        .DO15(out_wei[15]),   .DO16(out_wei[16]),   .DO17(out_wei[17]),   .DO18(out_wei[18]),   .DO19(out_wei[19]),
                        .DO20(out_wei[20]),   .DO21(out_wei[21]),   .DO22(out_wei[22]),   .DO23(out_wei[23]),   .DO24(out_wei[24]),
                        .DO25(out_wei[25]),   .DO26(out_wei[26]),   .DO27(out_wei[27]),   .DO28(out_wei[28]),   .DO29(out_wei[29]),
                        .DO30(out_wei[30]),   .DO31(out_wei[31]),   .DO32(out_wei[32]),   .DO33(out_wei[33]),   .DO34(out_wei[34]),
                        .DO35(out_wei[35]),   .DO36(out_wei[36]),   .DO37(out_wei[37]),   .DO38(out_wei[38]),   .DO39(out_wei[39]),
                        .DO40(out_wei[40]),   .DO41(out_wei[41]),   .DO42(out_wei[42]),   .DO43(out_wei[43]),   .DO44(out_wei[44]),
                        .DO45(out_wei[45]),   .DO46(out_wei[46]),   .DO47(out_wei[47]),   .DO48(out_wei[48]),   .DO49(out_wei[49]),
                        .DO50(out_wei[50]),   .DO51(out_wei[51]),   .DO52(out_wei[52]),   .DO53(out_wei[53]),   .DO54(out_wei[54]),
                        .DO55(out_wei[55]),   .DO56(out_wei[56]),   .DO57(out_wei[57]),   .DO58(out_wei[58]),   .DO59(out_wei[59]),
                        .DO60(out_wei[60]),   .DO61(out_wei[61]),   .DO62(out_wei[62]),   .DO63(out_wei[63]),   .DO64(out_wei[64]),
                        .DO65(out_wei[65]),   .DO66(out_wei[66]),   .DO67(out_wei[67]),   .DO68(out_wei[68]),   .DO69(out_wei[69]),
                        .DO70(out_wei[70]),   .DO71(out_wei[71]),   .DO72(out_wei[72]),   .DO73(out_wei[73]),   .DO74(out_wei[74]),
                        .DO75(out_wei[75]),   .DO76(out_wei[76]),   .DO77(out_wei[77]),   .DO78(out_wei[78]),   .DO79(out_wei[79]),
                        .DO80(out_wei[80]),   .DO81(out_wei[81]),   .DO82(out_wei[82]),   .DO83(out_wei[83]),   .DO84(out_wei[84]),
                        .DO85(out_wei[85]),   .DO86(out_wei[86]),   .DO87(out_wei[87]),   .DO88(out_wei[88]),   .DO89(out_wei[89]),
                        .DO90(out_wei[90]),   .DO91(out_wei[91]),   .DO92(out_wei[92]),   .DO93(out_wei[93]),   .DO94(out_wei[94]),
                        .DO95(out_wei[95]),   .DO96(out_wei[96]),   .DO97(out_wei[97]),   .DO98(out_wei[98]),   .DO99(out_wei[99]),
                        .DO100(out_wei[100]), .DO101(out_wei[101]), .DO102(out_wei[102]), .DO103(out_wei[103]), .DO104(out_wei[104]),
                        .DO105(out_wei[105]), .DO106(out_wei[106]), .DO107(out_wei[107]), .DO108(out_wei[108]), .DO109(out_wei[109]),
                        .DO110(out_wei[110]), .DO111(out_wei[111]), .DO112(out_wei[112]), .DO113(out_wei[113]), .DO114(out_wei[114]),
                        .DO115(out_wei[115]), .DO116(out_wei[116]), .DO117(out_wei[117]), .DO118(out_wei[118]), .DO119(out_wei[119]),
                        .DO120(out_wei[120]), .DO121(out_wei[121]), .DO122(out_wei[122]), .DO123(out_wei[123]), .DO124(out_wei[124]),
                        .DO125(out_wei[125]), .DO126(out_wei[126]), .DO127(out_wei[127]), 
                        
                        .DI0(in_wei[0]),     .DI1(in_wei[1]),     .DI2(in_wei[2]),     .DI3(in_wei[3]),     .DI4(in_wei[4]),
                        .DI5(in_wei[5]),     .DI6(in_wei[6]),     .DI7(in_wei[7]),     .DI8(in_wei[8]),     .DI9(in_wei[9]),
                        .DI10(in_wei[10]),   .DI11(in_wei[11]),   .DI12(in_wei[12]),   .DI13(in_wei[13]),   .DI14(in_wei[14]),
                        .DI15(in_wei[15]),   .DI16(in_wei[16]),   .DI17(in_wei[17]),   .DI18(in_wei[18]),   .DI19(in_wei[19]),
                        .DI20(in_wei[20]),   .DI21(in_wei[21]),   .DI22(in_wei[22]),   .DI23(in_wei[23]),   .DI24(in_wei[24]),
                        .DI25(in_wei[25]),   .DI26(in_wei[26]),   .DI27(in_wei[27]),   .DI28(in_wei[28]),   .DI29(in_wei[29]),
                        .DI30(in_wei[30]),   .DI31(in_wei[31]),   .DI32(in_wei[32]),   .DI33(in_wei[33]),   .DI34(in_wei[34]),
                        .DI35(in_wei[35]),   .DI36(in_wei[36]),   .DI37(in_wei[37]),   .DI38(in_wei[38]),   .DI39(in_wei[39]),
                        .DI40(in_wei[40]),   .DI41(in_wei[41]),   .DI42(in_wei[42]),   .DI43(in_wei[43]),   .DI44(in_wei[44]),
                        .DI45(in_wei[45]),   .DI46(in_wei[46]),   .DI47(in_wei[47]),   .DI48(in_wei[48]),   .DI49(in_wei[49]),
                        .DI50(in_wei[50]),   .DI51(in_wei[51]),   .DI52(in_wei[52]),   .DI53(in_wei[53]),   .DI54(in_wei[54]),
                        .DI55(in_wei[55]),   .DI56(in_wei[56]),   .DI57(in_wei[57]),   .DI58(in_wei[58]),   .DI59(in_wei[59]),
                        .DI60(in_wei[60]),   .DI61(in_wei[61]),   .DI62(in_wei[62]),   .DI63(in_wei[63]),   .DI64(in_wei[64]),
                        .DI65(in_wei[65]),   .DI66(in_wei[66]),   .DI67(in_wei[67]),   .DI68(in_wei[68]),   .DI69(in_wei[69]),
                        .DI70(in_wei[70]),   .DI71(in_wei[71]),   .DI72(in_wei[72]),   .DI73(in_wei[73]),   .DI74(in_wei[74]),
                        .DI75(in_wei[75]),   .DI76(in_wei[76]),   .DI77(in_wei[77]),   .DI78(in_wei[78]),   .DI79(in_wei[79]),
                        .DI80(in_wei[80]),   .DI81(in_wei[81]),   .DI82(in_wei[82]),   .DI83(in_wei[83]),   .DI84(in_wei[84]),
                        .DI85(in_wei[85]),   .DI86(in_wei[86]),   .DI87(in_wei[87]),   .DI88(in_wei[88]),   .DI89(in_wei[89]),
                        .DI90(in_wei[90]),   .DI91(in_wei[91]),   .DI92(in_wei[92]),   .DI93(in_wei[93]),   .DI94(in_wei[94]),
                        .DI95(in_wei[95]),   .DI96(in_wei[96]),   .DI97(in_wei[97]),   .DI98(in_wei[98]),   .DI99(in_wei[99]),
                        .DI100(in_wei[100]), .DI101(in_wei[101]), .DI102(in_wei[102]), .DI103(in_wei[103]), .DI104(in_wei[104]),
                        .DI105(in_wei[105]), .DI106(in_wei[106]), .DI107(in_wei[107]), .DI108(in_wei[108]), .DI109(in_wei[109]),
                        .DI110(in_wei[110]), .DI111(in_wei[111]), .DI112(in_wei[112]), .DI113(in_wei[113]), .DI114(in_wei[114]),
                        .DI115(in_wei[115]), .DI116(in_wei[116]), .DI117(in_wei[117]), .DI118(in_wei[118]), .DI119(in_wei[119]),
                        .DI120(in_wei[120]), .DI121(in_wei[121]), .DI122(in_wei[122]), .DI123(in_wei[123]), .DI124(in_wei[124]),
                        .DI125(in_wei[125]), .DI126(in_wei[126]), .DI127(in_wei[127]), 
                        .CK(clk), .WEB(WEB_wei), .OE(1'b1), .CS(1'b1));
                        
endmodule


module PATH_MAP(
    clk, rst_n, in_valid, c_state_is_DRAM_r, c_state_is_Fill, c_state_is_Retrace,
    read_weight_finish, net_id, loc_x, loc_y, nxt_retrace_x, nxt_retrace_y,
    dram_read_cnt, rdata_m_inf, out_loc, out_wei, fill_done, retrace_done, retrace_WEB, 
    net_num, cur_net, wb_data, cost
); 

input clk, rst_n, in_valid;
input c_state_is_DRAM_r, c_state_is_Fill, c_state_is_Retrace;
input read_weight_finish;
input [3:0] net_id;     
input [5:0] loc_x, loc_y; 
input [6:0] dram_read_cnt;
input [127:0] rdata_m_inf, out_loc, out_wei;
output fill_done;
output reg retrace_done;
output reg retrace_WEB;
output reg [5:0] nxt_retrace_x, nxt_retrace_y;
output reg [3:0] net_num;   // total number of nets
output reg [3:0] cur_net;   // processing number, increase when retrace done
output reg [13:0] cost;
output reg [127:0] wb_data; // write back updated data to location map

reg source_flag;
reg [3:0] in_cnt;
reg [3:0] net [0:14];
reg [5:0] source_x [0:14];
reg [5:0] source_y [0:14];
reg [5:0] sink_x [0:14];
reg [5:0] sink_y [0:14];
reg [1:0] path_map [0:63][0:63];

integer i, j;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        net_num <= 4'd0;
        in_cnt  <= 4'd0;
    end
    else begin
        if (in_valid) begin
            net_num <= in_cnt;
            
            if (!source_flag) in_cnt <= in_cnt + 1'b1;
            else              in_cnt <= in_cnt;
        end
        else begin
            net_num <= net_num;
            in_cnt  <= 4'd0;
        end
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        source_flag <= 1'b1; // first element is source
        
        for (i=0; i<15; i=i+1) begin
            net[i]      <= 4'd0;
            source_x[i] <= 6'd0;
            source_y[i] <= 6'd0;
            sink_x[i]   <= 6'd0;
            sink_y[i]   <= 6'd0;
        end
    end
    else begin
        if (in_valid) begin
            source_flag <= ~source_flag;
            net[in_cnt] <= net_id;
            
            if (source_flag) begin    
                source_x[in_cnt] <= loc_x;
                source_y[in_cnt] <= loc_y;
            end
            else begin
                sink_x[in_cnt] <= loc_x;
                sink_y[in_cnt] <= loc_y;
            end
        end
    end
end

reg [5:0] retrace_x, retrace_y;
reg c_state_is_Fill_d1, c_state_is_Fill_d2;
wire Fill_1st_cycle = (~c_state_is_Fill_d1) & c_state_is_Fill;
wire Fill_2nd_cycle = (~c_state_is_Fill_d2) & c_state_is_Fill_d1;
wire can_do_retrace = (c_state_is_Retrace && read_weight_finish);
reg can_do_retrace_d1, can_do_retrace_d2;
always @(posedge clk) can_do_retrace_d1 <= can_do_retrace;
always @(posedge clk) can_do_retrace_d2 <= can_do_retrace_d1;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        c_state_is_Fill_d1 <= 1'b0;
        c_state_is_Fill_d2 <= 1'b0;
    end
    else begin
        c_state_is_Fill_d1 <= c_state_is_Fill;
        c_state_is_Fill_d2 <= c_state_is_Fill_d1;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        cur_net <= 4'd0;
    end
    else begin
        if (c_state_is_DRAM_r) begin
            cur_net <= 4'd0;
        end
        else if (c_state_is_Retrace && retrace_done) begin // if retrace done
            cur_net <= cur_net + 1'b1;
        end
        else begin
            cur_net <= cur_net;
        end
    end
end

reg [5:0] path_map_x, path_map_y;
always @* begin
    if (Fill_1st_cycle) begin
      	path_map_x = source_x[cur_net];
      	path_map_y = source_y[cur_net];
    end 
    else if (Fill_2nd_cycle) begin
      	path_map_x = sink_x[cur_net];
      	path_map_y = sink_y[cur_net];
    end 
    else if (c_state_is_Retrace) begin
      	path_map_x = retrace_x;
      	path_map_y = retrace_y;
    end
    else begin
      	path_map_x = 6'b0;
      	path_map_y = 6'b0;
    end
end

assign fill_done = (path_map[ sink_y[cur_net] ][ sink_x[cur_net] ][1] && c_state_is_Fill);

reg [1:0] fill_cnt;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) fill_cnt <= 2'd0;
    else begin
        if (c_state_is_Fill) begin
            if (fill_done) fill_cnt <= fill_cnt - 2'd2;
            else           fill_cnt <= fill_cnt + 1'b1;
        end
        else if (c_state_is_Retrace) begin
            if (can_do_retrace_d1 && retrace_WEB) fill_cnt <= fill_cnt - 1'b1;
            else                                  fill_cnt <= fill_cnt;
        end
        else    fill_cnt <= 2'd0;
    end
end

// retrace priority
//  down: y+1
//    up: y-1
// right: x+1
//  left: x-1

reg down, up, right, left;

always @(posedge clk) begin
    if (&nxt_retrace_y)  down <= 1'b0;
    else                 down <= (path_map[nxt_retrace_y+1][nxt_retrace_x][0] == fill_cnt[1] && path_map[nxt_retrace_y+1][nxt_retrace_x][1]); // can move down 
end

always @(posedge clk) begin
    if (|nxt_retrace_y)    up <= (path_map[nxt_retrace_y-1][nxt_retrace_x][0] == fill_cnt[1] && path_map[nxt_retrace_y-1][nxt_retrace_x][1]); // can move up
    else                   up <= 1'b0; 
end

always @(posedge clk) begin
    if (&nxt_retrace_x) right <= 1'b0;
    else                right <= (path_map[nxt_retrace_y][nxt_retrace_x+1][0] == fill_cnt[1] && path_map[nxt_retrace_y][nxt_retrace_x+1][1]); // can move right 
end

always @(posedge clk) begin
    if (|nxt_retrace_x)  left <= (path_map[nxt_retrace_y][nxt_retrace_x-1][0] == fill_cnt[1] && path_map[nxt_retrace_y][nxt_retrace_x-1][1]); // can move left
    else                 left <= 1'b0; 
end

always @(posedge clk) begin
    if (can_do_retrace_d2) begin
        if (retrace_WEB) begin
            nxt_retrace_x <= nxt_retrace_x;
            nxt_retrace_y <= nxt_retrace_y;
        end
        else begin
            case ({down, up, right, left})
                4'b0001: begin                            // go left
                    nxt_retrace_x <= nxt_retrace_x - 1'b1;
                    nxt_retrace_y <= nxt_retrace_y;  
                end
                4'b0010, 4'b0011: begin                   // go right
                    nxt_retrace_x <= nxt_retrace_x + 1'b1;
                    nxt_retrace_y <= nxt_retrace_y;
                end
                4'b0100, 4'b0101, 4'b0110, 4'b0111: begin // go up
                    nxt_retrace_x <= nxt_retrace_x;
                    nxt_retrace_y <= nxt_retrace_y - 1'b1;
                end
                default: begin                            // go down
                    nxt_retrace_x <= nxt_retrace_x;
                    nxt_retrace_y <= nxt_retrace_y + 1'b1;
                end
            endcase
        end
    end 
    else if (can_do_retrace_d1) begin
        case ({down, up, right, left})
            4'b0001: begin                            // go left
                nxt_retrace_x <= nxt_retrace_x - 1'b1;
                nxt_retrace_y <= nxt_retrace_y;  
            end
            4'b0010, 4'b0011: begin                   // go right
                nxt_retrace_x <= nxt_retrace_x + 1'b1;
                nxt_retrace_y <= nxt_retrace_y;
            end
            4'b0100, 4'b0101, 4'b0110, 4'b0111: begin // go up
                nxt_retrace_x <= nxt_retrace_x;
                nxt_retrace_y <= nxt_retrace_y - 1'b1;
            end
            default: begin                            // go down
                nxt_retrace_x <= nxt_retrace_x;
                nxt_retrace_y <= nxt_retrace_y + 1'b1;
            end
        endcase
    end
    else begin
        nxt_retrace_x <= sink_x[cur_net];
        nxt_retrace_y <= sink_y[cur_net];
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        retrace_x   <= 6'd0;
        retrace_y   <= 6'd0;
        retrace_WEB <= 1'b1; // read mode
    end
    else begin
        if (can_do_retrace_d2 && c_state_is_Retrace && !retrace_done) begin 
            retrace_x   <= nxt_retrace_x;
            retrace_y   <= nxt_retrace_y;
            retrace_WEB <= ~retrace_WEB;
        end 
        else begin
            retrace_x   <= sink_x[cur_net];
            retrace_y   <= sink_y[cur_net];
            retrace_WEB <= 1'b1; // read mode
        end
    end
end

// Lee's aogorithm state
// 2'b00: empty
// 2'b01: blocked
// 2'b10: state 1
// 2'b11: state 2

wire nxt_retrace_done = ( (retrace_x == source_x[cur_net]) && (retrace_y == source_y[cur_net]) );
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) retrace_done <= 1'b0;
    else        retrace_done <= nxt_retrace_done;
end

always @(posedge clk or negedge rst_n) begin 
    if (!rst_n) begin
        for (i=0; i<64; i=i+1) begin
            for (j=0; j<64; j=j+1) begin
                path_map[i][j] <= 2'd0;
            end
        end
    end
    else begin
        if (Fill_2nd_cycle) begin      //set path map to 0 at sink's location while 2nd cycle of Fill state
            path_map[ path_map_y ][ path_map_x ] <= 2'd0; // set to empty state
        end
        else if (Fill_1st_cycle) begin //set path map to 2 at source's location while 1st cycle of Fill state
            path_map[ path_map_y ][ path_map_x ] <= {1'b1, fill_cnt[1]}; // Lee's algo. state 1
        end
        else if (retrace_done) begin
            for (i=0; i<64; i=i+1) begin
                for (j=0; j<64; j=j+1) begin
                    path_map[i][j] <= {1'b0, {(~path_map[i][j][1]) & path_map[i][j][0]}};
                end
       	    end
        end
        else if (c_state_is_DRAM_r) begin
            for (i=0; i<32; i=i+1) begin
                path_map[ dram_read_cnt[6:1] ][ (dram_read_cnt[0]<<5) + i ] <= {1'b0, |rdata_m_inf[i*4 +: 4]};
            end
        end // ------------------------ <<< Lee's Algorithm >>> --------------------------
        else if (c_state_is_Fill) begin
            // center of path map
            for (i=1; i<63; i=i+1) begin
                for (j=1; j<63; j=j+1) begin 
                    if ( !(|path_map[i][j]) && (path_map[i-1][j][1] | path_map[i+1][j][1] | path_map[i][j-1][1] | path_map[i][j+1][1]) )
            					  path_map[i][j] <= {1'b1, fill_cnt[1]};
            				else
            					  path_map[i][j] <= path_map[i][j];
                end
            end
            
            // --------------------------- Edge of Path Map ------------------------------
            for (j=1; j<63; j=j+1) begin
                // left boundary
                if ( !(|path_map[0][j]) && (path_map[0][j-1][1] | path_map[0][j+1][1] | path_map[1][j][1]) )
                    path_map[0][j] <= {1'b1, fill_cnt[1]};
                else
                	  path_map[0][j] <= path_map[0][j];
                
                // right boundary	
                if ( !(|path_map[63][j]) && (path_map[63][j-1][1] | path_map[63][j+1][1] | path_map[62][j][1]) )
                	  path_map[63][j] <= {1'b1, fill_cnt[1]};
                else
                  	path_map[63][j] <= path_map[63][j];
            end
            
            for (i=1; i<63; i=i+1) begin
                // upper boundary
                if ( !(|path_map[i][0]) && (path_map[i-1][0][1] | path_map[i+1][0][1] | path_map[i][1][1]) )
                  	path_map[i][0] <= {1'b1, fill_cnt[1]};
                else
                  	path_map[i][0] <= path_map[i][0];
                
                // lower boundary	
                if ( !(|path_map[i][63]) && (path_map[i-1][63][1] | path_map[i+1][63][1] | path_map[i][62][1]) )
                  	path_map[i][63] <= {1'b1, fill_cnt[1]};
                else
                  	path_map[i][63] <= path_map[i][63];
            end
            
            // --------------------------- Corner of Path Map ------------------------------
            // upper left corner
            if ( !(|path_map[0][0]) && ( path_map[0][1][1] | path_map[1][0][1]) ) 
                path_map[0][0] <= {1'b1, fill_cnt[1]};
            else
                path_map[0][0] <= path_map[0][0];
                
            // upper right corner
            if ( !(|path_map[0][63]) && ( path_map[0][62][1] | path_map[1][63][1]) ) 
                path_map[0][63] <= {1'b1, fill_cnt[1]};
            else 
                path_map[0][63] <= path_map[0][63];
                
            // lower left corner
            if ( !(|path_map[63][0]) && ( path_map[63][1][1] | path_map[62][0][1]) ) 
                path_map[63][0] <= {1'b1, fill_cnt[1]};
            else 
                path_map[63][0] <= path_map[63][0];
                
            // lower right corner
            if ( !(|path_map[63][63]) && ( path_map[63][62][1] | path_map[62][63][1]) ) 
                path_map[63][63] <= {1'b1, fill_cnt[1]};
            else 
                path_map[63][63] <= path_map[63][63];
        end
        else if (can_do_retrace_d2) begin 
            path_map[ path_map_y ][ path_map_x ] <= 2'd1; // change state to "blocked"
        end
    end
end

// ----------------------- <<< Replace the data in location map by mask >>> ----------------------- 
always @* begin
    for (i=0; i<32; i=i+1) begin
        if (i == retrace_x[4:0]) wb_data[i*4 +: 4] = net[cur_net]; 
        else                     wb_data[i*4 +: 4] = out_loc[i*4 +: 4];
    end
end


// ----------------------- <<< Calculate total cost >>> -----------------------
wire [3:0] weight = out_wei[retrace_x[4:0]*4 +: 4];

always @(posedge clk or negedge rst_n) begin
  	if(!rst_n) begin
  		  cost <= 14'b0;
    end
  	else begin
    		if(in_valid)
            cost <= 14'b0;
    		else if(can_do_retrace_d2 && !retrace_WEB && !nxt_retrace_done) 
            cost <= cost + weight;
  	end
end

endmodule




module BRIDGE#(parameter ID_WIDTH=4, ADDR_WIDTH=32, DATA_WIDTH=128) (
    // Input Signals
    clk,
    rst_n,
    bridge_valid,
    addr_dram,
    mode,
    wdata,
    
    // DRAM Signals
    arvalid_m_inf, araddr_m_inf, rready_m_inf, awvalid_m_inf, awaddr_m_inf, wvalid_m_inf, wdata_m_inf, bready_m_inf,
	arready_m_inf, rvalid_m_inf, rresp_m_inf, rdata_m_inf, awready_m_inf, wready_m_inf, bvalid_m_inf, bresp_m_inf, rlast_m_inf, wlast_m_inf
);

// Input Signals
input clk, rst_n;
input bridge_valid;
input mode;         // mode0: write data into DRAM, mode1: read data from DRAM
input [ADDR_WIDTH-1:0] addr_dram;
input [DATA_WIDTH-1:0] wdata;

// ------------------------
// <<<<< AXI READ >>>>>
// ------------------------
// (1)	axi read address channel 
output reg                   arvalid_m_inf;
input  wire                  arready_m_inf;
output reg  [ADDR_WIDTH-1:0]  araddr_m_inf;
// ------------------------
// (2)	axi read data channel 
input  wire                   rvalid_m_inf;
output reg                    rready_m_inf;
input  wire [DATA_WIDTH-1:0]   rdata_m_inf;
input  wire                    rlast_m_inf;
input  wire [1:0]              rresp_m_inf;

// ------------------------
// <<<<< AXI WRITE >>>>>
// ------------------------
// (1) 	axi write address channel 
output reg                   awvalid_m_inf;
input  wire                  awready_m_inf;
output reg  [ADDR_WIDTH-1:0]  awaddr_m_inf;
// -------------------------
// (2)	axi write data channel 
output reg                    wvalid_m_inf;
input  wire                   wready_m_inf;
output reg  [DATA_WIDTH-1:0]   wdata_m_inf;
input  wire                    wlast_m_inf;
// -------------------------
// (3)	axi write response channel 
input  wire                   bvalid_m_inf;
output reg                    bready_m_inf;
input  wire  [1:0]             bresp_m_inf;
// -----------------------------


//==============================================//
//                     FSM                      //
//==============================================//
reg [3:0] c_state, n_state;

localparam Idle             = 4'd0;
localparam Wait_DRAM_r1     = 4'd1;
localparam Wait_DRAM_r2     = 4'd2;
localparam Wait_DRAM_w1     = 4'd3;
localparam Wait_DRAM_w2     = 4'd4; // wait wready and write dram
localparam Wait_DRAM_w3     = 4'd5; // wait bresp

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) c_state <= Idle;
    else        c_state <= n_state;
end

always @* begin
    case (c_state)
	Idle: begin
            if (bridge_valid) begin
                if (mode) n_state = Wait_DRAM_r1;
              	else      n_state = Wait_DRAM_w1;
            end
    	    else          n_state = Idle;
        end
        Wait_DRAM_r1: begin
    	    if (arready_m_inf)  n_state = Wait_DRAM_r2;
	    else                n_state = Wait_DRAM_r1;
        end
        Wait_DRAM_r2: begin
	    if (rvalid_m_inf) begin 
	        if (rlast_m_inf) n_state = Idle;
	        else             n_state = Wait_DRAM_r2;
	    end
	    else                n_state = Wait_DRAM_r2;
        end
        Wait_DRAM_w1: begin
	    if (awready_m_inf)  n_state = Wait_DRAM_w2;
	    else                n_state = Wait_DRAM_w1;
        end
        Wait_DRAM_w2: begin
	    if (wlast_m_inf)    n_state = Wait_DRAM_w3;
	    else                n_state = Wait_DRAM_w2;
        end
        Wait_DRAM_w3: begin
	    if (bvalid_m_inf)   n_state = Idle;
	    else                n_state = Wait_DRAM_w3;
        end
        default: begin
	    n_state = Idle;
        end
    endcase
end


//==============================================//
//                   Design                     //
//==============================================//

//-----------------------------------  Input ------------------------------------
reg [127:0] s_data; // static transmit data from DRAM
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        s_data  <= 128'd0;
    end
    else begin
        if (rvalid_m_inf) begin
            s_data  <= rdata_m_inf;
        end
        else begin
            s_data  <= s_data;
        end    
    end
end

//-----------------------------------  FSM  ------------------------------------

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        arvalid_m_inf <= 1'b0;
        araddr_m_inf  <= 128'd0;
    end
    else begin
        case (c_state) 
            Wait_DRAM_r1: begin
                if (arready_m_inf) begin // n_state == Wait_DRAM_back
                    arvalid_m_inf <= 1'b0;
                    araddr_m_inf  <= 128'd0;
                end
                else begin
                    arvalid_m_inf <= 1'b1;
                    araddr_m_inf  <= addr_dram;
                end
            end
            default: begin
                arvalid_m_inf <= 1'b0;
                araddr_m_inf  <= 128'd0;
            end
        endcase
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rready_m_inf <= 1'b0;
    end
    else begin
        case (c_state) 
            Wait_DRAM_r2: begin
                if (rlast_m_inf) rready_m_inf <= 1'b0;
                else             rready_m_inf <= 1'b1;
            end
            default: begin
                rready_m_inf <= rready_m_inf;
            end
        endcase
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        awvalid_m_inf <= 1'b0;
        awaddr_m_inf  <= 128'd0;
    end
   else begin
        case (c_state) 
            Wait_DRAM_w1: begin
                if (awready_m_inf) begin
                    awvalid_m_inf <= 1'b0;
                    awaddr_m_inf  <= 128'd0;
                end
                else begin
                    awvalid_m_inf <= 1'b1;
                    awaddr_m_inf  <= addr_dram;
                end
            end
            default: begin
                awvalid_m_inf <= 1'b0;
                awaddr_m_inf  <= 128'd0;
            end
        endcase
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) bready_m_inf <= 1'b0;
    else begin
        if (c_state == Wait_DRAM_w3) begin
            if (bvalid_m_inf) bready_m_inf <= 1'b0;
            else              bready_m_inf <= 1'b1;
        end
        else begin
            bready_m_inf <= 1'b0;
        end
    end  
end

always @* begin
    wdata_m_inf = wdata;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        wvalid_m_inf <= 1'b0;
    end
    else begin
        if (c_state == Wait_DRAM_w2) begin
            if (wlast_m_inf) wvalid_m_inf <= 1'b0;
            else             wvalid_m_inf <= 1'b1;
        end
        else begin
            wvalid_m_inf <= 1'b0;
        end
    end  
end

endmodule
