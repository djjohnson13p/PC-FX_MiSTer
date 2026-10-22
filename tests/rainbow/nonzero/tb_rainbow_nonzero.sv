// SPDX-License-Identifier: GPL-2.0-only
`timescale 1ns/1ps
// Three literal software-reference controls. Actual FIFO/VLC public ports;
// continuous CE, no parser/dequant/IDCT, no hierarchy reads or forced state.
module tb_rainbow_nonzero;
  import huc6271_pkg::*;
  logic clk=0;
  always #5 clk=~clk;
  logic reset=1, ce=1, init=0, start=0, push=0;
  logic [7:0] push_d=0;
  logic producer_complete=0;
  wire rx_v, reqr_n, empty, pop;
  wire [7:0] rx_d, q;
  wire in_end=producer_complete && !push && empty;
  wire qcx_wr, qw_v, cbuf, blk_start, blk_end, cw_v, nrl_v, run_v, busy, done;
  wire [3:0] qcx_d, mb;
  wire [6:0] qw_a, run_idx;
  wire [7:0] qw_d;
  wire [2:0] blk;
  wire [5:0] cw_idx;
  wire [11:0] cw_val;
  wire [4:0] nrl_n;
  wire [8:0] run_len;

  huc6271_kr_rx rx (
    .clk, .reset, .ce, .flush(init), .kr_d(8'd0), .kr_wr_n(1'b1),
    .rx_v, .rx_d, .want(!producer_complete), .reqr_n,
    .push, .push_d, .empty, .pop, .q
  );
  huc6271_vlc dut (
    .clk, .reset, .ce, .init, .start, .kind(8'hf8), .empty, .pop, .q, .in_end,
    .qcx_wr, .qcx_d, .qw_v, .qw_a, .qw_d, .cb_full(2'b00), .cbuf,
    .blk_start, .blk_end, .mb, .blk, .cw_v, .cw_idx, .cw_val,
    .pipe_idle(1'b1), .nrl_v, .nrl_n, .run_rdy(1'b1), .rle_full(1'b0),
    .run_v, .run_idx, .run_len, .busy, .done
  );

  localparam integer MAX_CYCLES=12000;
  logic [7:0] payload[0:127];
  logic [31:0] expected[0:156];
  string vector_dir, case_name="setup";
  integer payload_len, expected_len;
  integer errors=0, cases=0, cycles=0, emitted=0, popped=0;
  integer starts=0, ends=0, coefficients=0, completions=0;
  integer max_occupancy=0, first_done=-1;
  integer active_mb=-1, active_blk=-1;
  bit count_enabled=0, block_open=0, last_done=0;
  logic [31:0] actual_event;

  task automatic fail(input string reason);
    errors=errors+1;
    $display("CHECK_FAIL case=%s cycle=%0d %s",case_name,cycles,reason);
  endtask

  // Sample the registered pulses as consumers do: before the NBA update.
  always @(posedge clk) if(count_enabled) begin
    if(ce!==1'b1) fail("continuous CE requirement violated");
    if(emitted-popped>max_occupancy) max_occupancy=emitted-popped;
    if(push && emitted-popped>=128) fail("producer would overflow real FIFO");
    if(pop) begin
      if(empty) fail("pop while empty");
      popped=popped+1;
      if(popped>emitted) fail("more FIFO pops than accepted fixture bytes");
    end
    if(blk_start) begin
      if(block_open) fail("new block started before previous block ended");
      if(starts>=96 || mb!==(starts/6) || blk!==(starts%6))
        fail($sformatf("block start order %0d mb=%0d blk=%0d",starts,mb,blk));
      if(cbuf!==(starts%2)) fail("coefficient buffer did not alternate at block start");
      active_mb=mb; active_blk=blk; block_open=1;
      $display("BLOCK_START case=%s ordinal=%0d mb=%0d blk=%0d cbuf=%0d",case_name,starts,mb,blk,cbuf);
      starts=starts+1;
    end
    if(cw_v) begin
      if(!block_open || mb!==active_mb || blk!==active_blk)
        fail("coefficient outside its ordered active block");
      actual_event={7'd0,mb,blk,cw_idx,cw_val};
      if(coefficients>=expected_len)
        fail($sformatf("extra coefficient event actual=%08h",actual_event));
      else if(actual_event!==expected[coefficients])
        fail($sformatf("coefficient[%0d] expected=%08h actual=%08h",coefficients,expected[coefficients],actual_event));
      $display("COEFFICIENT case=%s ordinal=%0d mb=%0d blk=%0d index=%0d value=%0d raw=%03h",case_name,coefficients,mb,blk,cw_idx,$signed(cw_val),cw_val);
      coefficients=coefficients+1;
    end
    if(blk_end) begin
      if(!block_open || ends>=starts) fail("block end without an ordered active block");
      $display("BLOCK_END case=%s ordinal=%0d mb=%0d blk=%0d",case_name,ends,active_mb,active_blk);
      block_open=0; ends=ends+1;
    end
    if(qcx_wr || qw_v || nrl_v || run_v)
      fail("unexpected quantizer/table/null/palette output");
    if(done && !last_done) begin
      completions=completions+1;
      if(first_done<0) first_done=cycles;
      if(ends!=96) fail("completion before all 96 block ends");
    end
    last_done=done;
  end

  task automatic run_case(input string name, input integer bytes_n, input integer events_n);
    integer before_errors, settle;
    bit completed;
    case_name=name; payload_len=bytes_n; expected_len=events_n;
    if(bytes_n>=128 || events_n>157) $fatal(1,"Fixture bound exceeds private bench arrays");
    for(integer i=0;i<128;i=i+1) payload[i]='x;
    for(integer i=0;i<157;i=i+1) expected[i]='x;
    $readmemh({vector_dir,"/",name,".payload.hex"},payload,0,bytes_n-1);
    $readmemh({vector_dir,"/",name,".expected.hex"},expected,0,events_n-1);
    before_errors=errors; cases=cases+1; count_enabled=0;
    cycles=0; emitted=0; popped=0; starts=0; ends=0; coefficients=0;
    completions=0; max_occupancy=0; first_done=-1; block_open=0; last_done=0;
    active_mb=-1; active_blk=-1; settle=0; completed=0;
    @(negedge clk); reset=1; init=0; start=0; push=0; producer_complete=0;
    repeat(3) @(negedge clk);
    reset=0; init=1;
    @(negedge clk); init=0; start=1;
    @(negedge clk); start=0; count_enabled=1;
    for(integer tick=0;tick<MAX_CYCLES;tick=tick+1) begin
      cycles=tick;
      push=emitted<payload_len;
      if(push) begin
        push_d=payload[emitted];
        if($isunknown(push_d)) $fatal(1,"Unknown fixture input byte");
      end
      @(posedge clk); #1;
      if(push) emitted=emitted+1;
      producer_complete=emitted==payload_len;
      if(!busy && tick>4) begin
        settle=settle+1;
        if(settle>=8) begin completed=1; break; end
      end else settle=0;
      @(negedge clk);
    end
    @(negedge clk); push=0;
    repeat(3) @(negedge clk);
    count_enabled=0;
    if(!completed) fail("fixed 12000-cycle budget exhausted");
    if(starts!=96 || ends!=96 || block_open) fail($sformatf("block totals starts=%0d ends=%0d open=%0d",starts,ends,block_open));
    if(coefficients!=expected_len) fail($sformatf("coefficient count expected=%0d actual=%0d",expected_len,coefficients));
    if(completions!=1 || first_done<0 || !done || busy) fail($sformatf("completion count=%0d done=%0d busy=%0d",completions,done,busy));
    if(emitted!=payload_len || popped!=payload_len || !empty)
      fail($sformatf("fixture transfer bytes=%0d emitted=%0d popped=%0d empty=%0d",payload_len,emitted,popped,empty));
    $display("CASE_RESULT name=%s bytes=%0d emitted=%0d popped=%0d starts=%0d ends=%0d coefficients=%0d completions=%0d first_done=%0d cycles=%0d max_fifo_occupancy=%0d errors=%0d",name,payload_len,emitted,popped,starts,ends,coefficients,completions,first_done,cycles,max_occupancy,errors-before_errors);
  endtask

  initial begin
    if(!$value$plusargs("VECTOR_DIR=%s",vector_dir)) $fatal(1,"Explicit private VECTOR_DIR required");
    run_case("nz_dc_predictors",94,96);
    run_case("nz_ac_yuv_runs",97,104);
    run_case("nz_ac_overrun_alignment",117,157);
    $display("NONZERO_SUITE_RESULT cases=%0d errors=%0d continuous_ce=1 actual_fifo=1",cases,errors);
    if(errors!=0) $fatal(1,"Nonzero interface control mismatch");
    $finish;
  end
  initial begin #1000000; $fatal(1,"Private bench wall-clock simulation budget exhausted"); end
endmodule
