module AraBlackbox
    #(
      parameter TRANS_ID_WIDTH,
      parameter NLANES,
      parameter VLEN,
      parameter AXI_ID_WIDTH,
      parameter AXI_ADDR_WIDTH,
      parameter AXI_USER_WIDTH,
      parameter AXI_DATA_WIDTH
     )
(
    input			    clk_i,
    input			    rst_ni,

    input			    axi_resp_i_aw_ready,
    output			    axi_req_o_aw_valid,
    output [AXI_ID_WIDTH-1:0]	    axi_req_o_aw_bits_id,
    output [AXI_ADDR_WIDTH-1:0]	    axi_req_o_aw_bits_addr,
    output [7:0]		    axi_req_o_aw_bits_len,
    output [2:0]		    axi_req_o_aw_bits_size,
    output [1:0]		    axi_req_o_aw_bits_burst,
    output			    axi_req_o_aw_bits_lock,
    output [3:0]		    axi_req_o_aw_bits_cache,
    output [2:0]		    axi_req_o_aw_bits_prot,
    output [3:0]		    axi_req_o_aw_bits_qos,
    output [3:0]		    axi_req_o_aw_bits_region,
    output [5:0]		    axi_req_o_aw_bits_atop,
    output [AXI_USER_WIDTH-1:0]	    axi_req_o_aw_bits_user,

    input			    axi_resp_i_w_ready,
    output			    axi_req_o_w_valid,
    output [AXI_DATA_WIDTH-1:0]	    axi_req_o_w_bits_data,
    output [(AXI_DATA_WIDTH/8)-1:0] axi_req_o_w_bits_strb,
    output			    axi_req_o_w_bits_last,
    output [AXI_USER_WIDTH-1:0]	    axi_req_o_w_bits_user,

    input			    axi_resp_i_ar_ready,
    output			    axi_req_o_ar_valid,
    output [AXI_ID_WIDTH-1:0]	    axi_req_o_ar_bits_id,
    output [AXI_ADDR_WIDTH-1:0]	    axi_req_o_ar_bits_addr,
    output [7:0]		    axi_req_o_ar_bits_len,
    output [2:0]		    axi_req_o_ar_bits_size,
    output [1:0]		    axi_req_o_ar_bits_burst,
    output			    axi_req_o_ar_bits_lock,
    output [3:0]		    axi_req_o_ar_bits_cache,
    output [2:0]		    axi_req_o_ar_bits_prot,
    output [3:0]		    axi_req_o_ar_bits_qos,
    output [3:0]		    axi_req_o_ar_bits_region,
    output [AXI_USER_WIDTH-1:0]	    axi_req_o_ar_bits_user,

    output			    axi_req_o_b_ready,
    input			    axi_resp_i_b_valid,
    input [AXI_ID_WIDTH-1:0]	    axi_resp_i_b_bits_id,
    input [1:0]			    axi_resp_i_b_bits_resp,
    input [AXI_USER_WIDTH-1:0]	    axi_resp_i_b_bits_user,

    output			    axi_req_o_r_ready,
    input			    axi_resp_i_r_valid,
    input [AXI_ID_WIDTH-1:0]	    axi_resp_i_r_bits_id,
    input [AXI_DATA_WIDTH-1:0]	    axi_resp_i_r_bits_data,
    input [1:0]			    axi_resp_i_r_bits_resp,
    input			    axi_resp_i_r_bits_last,
    input [AXI_USER_WIDTH-1:0]	    axi_resp_i_r_bits_user,

    input			    req_req_valid,
    input			    req_resp_ready,
    input [31:0]		    req_insn,
    input [63:0]		    req_rs1,
    input [63:0]		    req_rs2,
    input [1:0]			    req_frm,
    input [TRANS_ID_WIDTH-1:0]	    req_trans_id,
    input			    req_store_pending,
    input			    req_acc_cons_en,
    input			    req_inval_ready,

    output			    resp_req_ready,
    output			    resp_resp_valid,
    output [63:0]		    resp_result,
    output [TRANS_ID_WIDTH-1:0]	    resp_trans_id,
    output			    resp_store_pending,
    output			    resp_store_complete,
    output			    resp_load_complete,
    output [4:0]		    resp_fflags,
    output			    resp_fflags_valid,
    output			    resp_inval_valid,
    output [63:0]		    resp_inval_addr
);

   localparam                           NrAXIMasters = 1; // Actually masters, but slaves on the crossbar
   localparam                           AxiSocIdWidth  = AXI_ID_WIDTH - $clog2(NrAXIMasters);
   localparam                           AxiCoreIdWidth = AxiSocIdWidth - 1;

`include "axi/typedef.svh"

   typedef logic [AXI_DATA_WIDTH-1:0]   axi_data_t;
   typedef logic [AXI_DATA_WIDTH/8-1:0] axi_strb_t;
   typedef logic [AXI_ADDR_WIDTH-1:0]   axi_addr_t;
   typedef logic [AXI_USER_WIDTH-1:0]   axi_user_t;
   typedef logic [AXI_ID_WIDTH-1:0]     axi_id_t;
   typedef logic [AxiCoreIdWidth-1:0]   axi_core_id_t;


  `AXI_TYPEDEF_AR_CHAN_T(ar_chan_t, axi_addr_t, axi_id_t, axi_user_t)
  `AXI_TYPEDEF_R_CHAN_T(r_chan_t, axi_data_t, axi_id_t, axi_user_t)
  `AXI_TYPEDEF_AW_CHAN_T(aw_chan_t, axi_addr_t, axi_id_t, axi_user_t)
  `AXI_TYPEDEF_W_CHAN_T(w_chan_t, axi_data_t, axi_strb_t, axi_user_t)
  `AXI_TYPEDEF_B_CHAN_T(b_chan_t, axi_id_t, axi_user_t)
  `AXI_TYPEDEF_REQ_T(axi_req_t, aw_chan_t, w_chan_t, ar_chan_t)
  `AXI_TYPEDEF_RESP_T(axi_resp_t, b_chan_t, r_chan_t)
  `AXI_TYPEDEF_ALL(ara_axi, axi_addr_t, axi_core_id_t, axi_data_t, axi_strb_t, axi_user_t)

   import acc_pkg::cva6_to_acc_t;
   import acc_pkg::acc_to_cva6_t;
   
   // Accelerator ports
   cva6_to_acc_t                     acc_req;
   acc_to_cva6_t                    acc_resp;
   ara_axi_req_t     ara_axi_req;
   ara_axi_resp_t    ara_axi_resp;

   
   assign acc_req.acc_req.req_valid = req_req_valid;
   assign acc_req.acc_req.resp_ready = req_resp_ready;
   assign acc_req.acc_req.insn = req_insn;
   assign acc_req.acc_req.rs1 = req_rs1;
   assign acc_req.acc_req.rs2 = req_rs2;
   assign acc_req.acc_req.frm = req_frm;
   assign acc_req.acc_req.trans_id = req_trans_id;
   assign acc_req.acc_req.store_pending = req_store_pending;
   assign acc_req.acc_req.acc_cons_en = req_acc_cons_en;
   assign acc_req.acc_req.inval_ready = req_inval_ready;
   assign acc_req.acc_mmu_en = 1'b0;

   assign resp_req_ready = acc_resp.acc_resp.req_ready;
   assign resp_resp_valid = acc_resp.acc_resp.resp_valid;
   assign resp_result = acc_resp.acc_resp.result;
   assign resp_trans_id = acc_resp.acc_resp.trans_id;
   assign resp_store_pending = acc_resp.acc_resp.store_pending;
   assign resp_store_complete = acc_resp.acc_resp.store_complete;
   assign resp_load_complete = acc_resp.acc_resp.load_complete;
   assign resp_fflags = acc_resp.acc_resp.fflags;
   assign resp_fflags_valid = acc_resp.acc_resp.fflags_valid;
   assign resp_inval_valid = acc_resp.acc_resp.inval_valid;
   assign resp_inval_addr = acc_resp.acc_resp.inval_addr;

   assign ara_axi_resp.aw_ready = axi_resp_i_aw_ready;
   assign axi_req_o_aw_valid = ara_axi_req.aw_valid;
   assign axi_req_o_aw_bits_id = ara_axi_req.aw.id;
   assign axi_req_o_aw_bits_addr = ara_axi_req.aw.addr;
   assign axi_req_o_aw_bits_len = ara_axi_req.aw.len;
   assign axi_req_o_aw_bits_size = ara_axi_req.aw.size;
   assign axi_req_o_aw_bits_burst = ara_axi_req.aw.burst;
   assign axi_req_o_aw_bits_lock = ara_axi_req.aw.lock;
   assign axi_req_o_aw_bits_cache = ara_axi_req.aw.cache;
   assign axi_req_o_aw_bits_prot = ara_axi_req.aw.prot;
   assign axi_req_o_aw_bits_qos = ara_axi_req.aw.qos;
   assign axi_req_o_aw_bits_region = ara_axi_req.aw.region;
   assign axi_req_o_aw_bits_atop = ara_axi_req.aw.atop;
   assign axi_req_o_aw_bits_user = ara_axi_req.aw.user;

   assign ara_axi_resp.w_ready = axi_resp_i_w_ready;
   assign axi_req_o_w_valid = ara_axi_req.w_valid;
   assign axi_req_o_w_bits_data = ara_axi_req.w.data;
   assign axi_req_o_w_bits_strb = ara_axi_req.w.strb;
   assign axi_req_o_w_bits_last = ara_axi_req.w.last;
   assign axi_req_o_w_bits_user = ara_axi_req.w.user;

   assign ara_axi_resp.ar_ready = axi_resp_i_ar_ready;
   assign axi_req_o_ar_valid = ara_axi_req.ar_valid;
   assign axi_req_o_ar_bits_id = ara_axi_req.ar.id;
   assign axi_req_o_ar_bits_addr = ara_axi_req.ar.addr;
   assign axi_req_o_ar_bits_len = ara_axi_req.ar.len;
   assign axi_req_o_ar_bits_size = ara_axi_req.ar.size;
   assign axi_req_o_ar_bits_burst = ara_axi_req.ar.burst;
   assign axi_req_o_ar_bits_lock = ara_axi_req.ar.lock;
   assign axi_req_o_ar_bits_cache = ara_axi_req.ar.cache;
   assign axi_req_o_ar_bits_prot = ara_axi_req.ar.prot;
   assign axi_req_o_ar_bits_qos = ara_axi_req.ar.qos;
   assign axi_req_o_ar_bits_region = ara_axi_req.ar.region;
   assign axi_req_o_ar_bits_user = ara_axi_req.ar.user;

   assign axi_req_o_b_ready = ara_axi_req.b_ready;
   assign ara_axi_resp.b_valid = axi_resp_i_b_valid;
   assign ara_axi_resp.b.id = axi_resp_i_b_bits_id;
   assign ara_axi_resp.b.resp = axi_resp_i_b_bits_resp;
   assign ara_axi_resp.b.user = axi_resp_i_b_bits_user;

   assign axi_req_o_r_ready = ara_axi_req.r_ready;
   assign ara_axi_resp.r_valid = axi_resp_i_r_valid;
   assign ara_axi_resp.r.id = axi_resp_i_r_bits_id;
   assign ara_axi_resp.r.resp = axi_resp_i_r_bits_resp;
   assign ara_axi_resp.r.data = axi_resp_i_r_bits_data;
   assign ara_axi_resp.r.user = axi_resp_i_r_bits_user;
   assign ara_axi_resp.r.last = axi_resp_i_r_bits_last;


   ara #(
         .VLEN(VLEN),
	 .NrLanes(NLANES),
	 .AxiDataWidth(AXI_DATA_WIDTH),
	 .AxiAddrWidth(AXI_ADDR_WIDTH),
         .axi_ar_t(ara_axi_ar_chan_t),
         .axi_r_t(ara_axi_r_chan_t),
         .axi_aw_t(ara_axi_aw_chan_t),
         .axi_w_t(ara_axi_w_chan_t),
         .axi_b_t(ara_axi_b_chan_t),
         .axi_req_t(ara_axi_req_t),
         .axi_resp_t(ara_axi_resp_t)
	 ) ara (
		.clk_i(clk_i),
		.rst_ni(rst_ni),
                .scan_enable_i(1'b0),
                .scan_data_i(1'b0),
                .scan_data_o(),
		.acc_req_i(acc_req),
		.acc_resp_o(acc_resp),
		.axi_req_o(ara_axi_req),
		.axi_resp_i(ara_axi_resp)
		);
endmodule
