`timescale 1ns/1ps
// Interface tests of the unmodified PC-FX RAINBOW VLC and receive FIFO.
// No hierarchical reads, forced state, BIOS, game data or copyrighted fixtures.
module tb_rainbow_vlc;
  import huc6271_pkg::*;
  parameter integer HELD_Q_MODEL = 0;
  logic clk=0;
  always #5 clk=~clk;
  logic reset=1, ce=1, init=0, start=0;
  logic [7:0] kind=8'hf8;
  wire empty, pop;
  wire [7:0] q;
  wire in_end;
  wire qcx_wr, qw_v, cbuf, blk_start, blk_end, cw_v, nrl_v, run_v, busy, done;
  wire [3:0] qcx_d, mb;
  wire [6:0] qw_a, run_idx;
  wire [7:0] qw_d;
  wire [2:0] blk;
  wire [5:0] cw_idx;
  wire [11:0] cw_val;
  wire [4:0] nrl_n;
  wire [8:0] run_len;
  logic [1:0] cb_full=0;
  logic push=0;
  logic [7:0] push_d=0;
  logic producer_complete=0;
  byte unsigned payload[0:2047];
  bit bits[0:16383];
  integer bit_count=0, payload_len=0, emitted=0, popped=0;
  integer starts=0, ends=0, coefficients=0, qcx_count=0;
  integer errors=0, cases=0, cycles=0, first_done=-1;
  integer expected_qcx[0:15];
  integer expected_qcx_count=0;
  integer current_split_after=-1;
  bit count_enabled=0;
  string current_case="setup";
  logic [31:0] rng;
  integer seed_value=32'h62a1f00d;

  generate if (!HELD_Q_MODEL) begin : actual_fifo
    wire rx_v, reqr_n;
    wire [7:0] rx_d;
    huc6271_kr_rx rx (
      .clk, .reset, .ce, .flush(init),
      .kr_d(8'd0), .kr_wr_n(1'b1), .rx_v, .rx_d,
      .want(!producer_complete), .reqr_n,
      .push, .push_d, .empty, .pop, .q
    );
  end else begin : held_model
    integer wr=0, rd=0;
    logic [7:0] qr=0;
    byte unsigned fifo_mem[0:2047];
    always @(posedge clk) begin
      if (reset || init) begin wr<=0; rd<=0; qr<=0; end
      else if (ce) begin
        if(push) begin fifo_mem[wr]<=push_d; wr<=wr+1; end
        if(pop && !empty) begin qr<=fifo_mem[rd]; rd<=rd+1; end
      end
    end
    assign empty=(rd==wr);
    assign q=qr;
  end endgenerate

  // Match huc6271.sv: end is true only once the parser has sent all bytes
  // and its FIFO is empty, not merely when the producer has finished.
  assign in_end=producer_complete && empty;
  huc6271_vlc dut (
    .clk, .reset, .ce, .init, .start, .kind,
    .empty, .pop, .q, .in_end,
    .qcx_wr, .qcx_d, .qw_v, .qw_a, .qw_d,
    .cb_full, .cbuf, .blk_start, .blk_end, .mb, .blk,
    .cw_v, .cw_idx, .cw_val, .pipe_idle(1'b1), .nrl_v, .nrl_n,
    .run_rdy(1'b1), .rle_full(1'b0), .run_v, .run_idx, .run_len,
    .busy, .done
  );

  task automatic fail(input string reason);
    errors=errors+1;
    if(errors<35) $display("CHECK_FAIL case=%s cycle=%0d %s",current_case,cycles,reason);
  endtask

  // Score pulses exactly as enabled downstream logic sees them: before
  // the NBA update, only on enabled edges. Stalled pulses are not duplicated.
  always @(posedge clk) if (ce && count_enabled) begin
    if (pop) begin
      if(empty) fail("pop while empty");
      popped=popped+1;
    end
    if (blk_start) begin
      if(mb!==(starts/6) || blk!==(starts%6))
        fail($sformatf("block order start=%0d mb=%0d blk=%0d",starts,mb,blk));
      starts=starts+1;
    end
    if (blk_end) ends=ends+1;
    if (cw_v) begin
      if(cw_idx!==0 || cw_val!==0)
        fail($sformatf("expected only DC0, got idx=%0d val=%h block=%0d",cw_idx,cw_val,coefficients));
      coefficients=coefficients+1;
    end
    if (qcx_wr) begin
      if(current_split_after>=0 && emitted<=current_split_after)
        fail("QCx emitted before delayed code byte arrived");
      if(qcx_count>=expected_qcx_count)
        fail($sformatf("unexpected QCx %0d",qcx_d));
      else if(qcx_d!==expected_qcx[qcx_count])
        fail($sformatf("QCx[%0d] expected=%0d actual=%0d",qcx_count,expected_qcx[qcx_count],qcx_d));
      qcx_count=qcx_count+1;
    end
    if(qw_v || nrl_v || run_v) fail("unexpected Q-table/null-run/palette output in F8 DC0 vector");
    if(done && first_done<0) first_done=cycles;
  end

  task automatic append_bits(input integer value, input integer length);
    for(integer n=length-1;n>=0;n=n-1) begin
      bits[bit_count]=((value>>n)&1)!=0;
      bit_count=bit_count+1;
    end
  endtask
  task automatic base_strip;
    bit_count=0;
    expected_qcx_count=0;
    for(integer macroblock=0;macroblock<16;macroblock=macroblock+1) begin
      for(integer block_no=0;block_no<6;block_no=block_no+1) begin
        if(block_no<4) append_bits(0,3); // independent DC-Y size0 prefix
        else append_bits(0,2);          // independent DC-UV size0 prefix
        append_bits(31,5);              // EOB = 11111
      end
    end
  endtask
  task automatic append_qcx(input integer value);
    append_bits(31,5);
    append_bits(value,4);
    expected_qcx[expected_qcx_count]=value;
    expected_qcx_count=expected_qcx_count+1;
  endtask
  task automatic pack_bytes;
    while((bit_count%8)!=0) append_bits(0,1);
    payload_len=bit_count/8;
    for(integer b=0;b<payload_len;b=b+1) begin
      payload[b]=0;
      for(integer j=0;j<8;j=j+1) payload[b]=(payload[b]<<1)|bits[b*8+j];
    end
  endtask

  // modes: 0 constant ce, all arrivals; 1 deterministic ce/buffer/FIFO
  // stalls; 2 pseudorandom ce/buffer/FIFO stalls; 3 isolated ce gaps;
  // 4 exactly alternating ce (synthetic pause stress, not normal operation),
  // 5/6 deterministic/random FIFO+coefficient stalls with normal constant ce,
  // 7 one pause immediately after an external pop, 8 an idle pre-input pause.
  task automatic run_case(input string name, input integer mode,
      input integer split_after, input bit expect_complete);
    integer before_errors, hold_cycles, settle, pause_left;
    bit completed, pause_fired, pause_reported;
    current_case=name;
    before_errors=errors;
    cases=cases+1;
    count_enabled=0;
    emitted=0; popped=0; starts=0; ends=0; coefficients=0; qcx_count=0;
    cycles=0; first_done=-1; hold_cycles=0; settle=0; completed=0;
    pause_left=0; pause_fired=0; pause_reported=0; current_split_after=split_after;
    rng=seed_value;
    @(negedge clk); reset=1; init=0; start=0; ce=1; push=0;
    cb_full=0; producer_complete=0;
    repeat(3) @(negedge clk);
    reset=0; init=1;
    @(negedge clk); init=0; start=1;
    @(negedge clk); start=0; count_enabled=1;
    for(integer tick=0;tick<12000;tick=tick+1) begin
      cycles=tick;
      rng={rng[30:0],rng[31]^rng[21]^rng[1]^rng[0]};
      case(mode)
        1: ce=(tick%7)!=2 && (tick%7)!=3;
        2: ce=rng[0] || rng[4];
        3: ce=(tick%11)!=5;
        4: ce=(tick%2)==0;
        7: begin
          if(!pause_fired && popped>=5) begin
            pause_left=7; pause_fired=1;
            $display("PAUSE_BEGIN case=%s cycle=%0d popped=%0d q=%h",name,tick,popped,q);
          end
          ce=(pause_left==0);
          if(pause_left>0) pause_left=pause_left-1;
          else if(pause_fired && !pause_reported) begin
            pause_reported=1;
            $display("PAUSE_END case=%s cycle=%0d popped=%0d q=%h",name,tick,popped,q);
          end
        end
        8: ce=tick>=10;
        default: ce=1;
      endcase
      cb_full=0;
      if((mode==1 || mode==5) && (tick%37)<11) cb_full=3;
      if((mode==2 || mode==6) && rng[9:7]==0) cb_full=3;
      push=0;
      if(ce && emitted<payload_len) begin
        if(split_after>=0 && emitted==split_after && (ends<96 || hold_cycles<100)) begin
          if(ends>=96) hold_cycles=hold_cycles+1;
        end else if(((mode!=1 && mode!=5) || (tick%5)!=0) &&
                    ((mode!=2 && mode!=6) || rng[6:5]!=0)) begin
          push=1; push_d=payload[emitted];
        end
      end
      @(posedge clk);
      #1;
      if(ce && push) emitted=emitted+1;
      producer_complete=(emitted==payload_len);
      if(!busy && !start && tick>4) begin
        settle=settle+1;
        if(settle>=4) begin completed=1; break; end
      end else settle=0;
      if(!expect_complete && tick>=2500) break;
      @(negedge clk);
    end
    @(negedge clk); push=0; ce=1;
    repeat(3) @(negedge clk);
    count_enabled=0;
    if(expect_complete) begin
      if(!completed) fail("timeout waiting for busy to clear");
      if(starts!=96 || ends!=96 || coefficients!=96)
        fail($sformatf("block totals start=%0d end=%0d DC=%0d expected96",starts,ends,coefficients));
      if(qcx_count!=expected_qcx_count)
        fail($sformatf("QCx count=%0d expected=%0d",qcx_count,expected_qcx_count));
      if(first_done<0) fail("complete strip never asserted done");
    end else begin
      if(completed || first_done>=0 || !busy) fail("truncated entropy falsely completed");
      if(ends>=96) fail("truncated entropy produced 96 completed blocks");
    end
    $display("CASE_RESULT name=%s model=%0d mode=%0d bytes=%0d emitted=%0d popped=%0d starts=%0d ends=%0d DC=%0d qcx=%0d first_done=%0d cycles=%0d errors=%0d",name,HELD_Q_MODEL,mode,payload_len,emitted,popped,starts,ends,coefficients,qcx_count,first_done,cycles,errors-before_errors);
  endtask

  initial begin
    if($value$plusargs("SEED=%d",seed_value)) begin end
    base_strip(); pack_bytes();
    run_case("DC0_no_tail_constant_ce",0,-1,1);
    base_strip(); append_qcx(10); pack_bytes();
    run_case("one_tail_QCx_constant_ce",0,-1,1);
    base_strip(); append_qcx(0); append_qcx(5); append_qcx(15); pack_bytes();
    run_case("multiple_tail_QCx_constant_ce",0,-1,1);
    base_strip(); append_qcx(11); pack_bytes();
    run_case("QCx_split_at_byte_boundary",0,93,1);
    base_strip(); append_bits(255,8); pack_bytes();
    run_case("incomplete_tail_QCx_true_end",0,-1,1);
    base_strip(); pack_bytes(); payload_len=payload_len-1;
    run_case("truncated_entropy_no_false_done",0,-1,0);
    base_strip(); append_qcx(3); append_qcx(12); pack_bytes();
    run_case("constant_ce_deterministic_input_buffer_stalls",5,-1,1);
    base_strip(); append_qcx(3); append_qcx(12); pack_bytes();
    run_case("constant_ce_random_input_buffer_stalls",6,-1,1);
    base_strip(); append_qcx(7); pack_bytes();
    run_case("idle_pause_before_input",8,-1,1);
    base_strip(); append_qcx(7); pack_bytes();
    run_case("one_pause_after_fifth_pop",7,-1,1);
    base_strip(); append_qcx(3); append_qcx(12); pack_bytes();
    run_case("deterministic_stalls",1,-1,1);
    base_strip(); append_qcx(3); append_qcx(12); pack_bytes();
    run_case("pseudorandom_stalls",2,-1,1);
    base_strip(); append_qcx(7); pack_bytes();
    run_case("isolated_ce_gaps",3,-1,1);
    base_strip(); append_qcx(7); pack_bytes();
    run_case("alternating_ce",4,-1,1);
    $display("SUITE_RESULT cases=%0d errors=%0d HELD_Q_MODEL=%0d seed=%0d",cases,errors,HELD_Q_MODEL,seed_value);
    if(errors!=0) $fatal(1,"RAINBOW baseline failures recorded");
    $finish;
  end
  initial begin #20000000; $fatal(1,"global testbench timeout"); end
endmodule
