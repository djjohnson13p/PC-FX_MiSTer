`timescale 1ns/1ps
// Independent queue scoreboard for the actual 128-entry kr_rx/cache_ram_dp.
// Checks response timing at public ports; no hierarchical state or forces.
module tb_rainbow_fifo;
  parameter bit DISABLE_RAM_INIT = 0;
  import huc6271_pkg::*;
  logic clk=0;
  always #5 clk=~clk;
  logic reset=1, flush=0, ce=1, push=0, pop=0;
  logic [7:0] push_d=0;
  wire empty, reqr_n, rx_v;
  wire [7:0] q, rx_d;
  huc6271_kr_rx dut (
    .clk, .reset, .ce, .flush,
    .kr_d(8'd0), .kr_wr_n(1'b1), .rx_v, .rx_d,
    .want(1'b1), .reqr_n, .push, .push_d, .empty, .pop, .q
  );
  // Optional explicit vendor-simulation profile only. A blank initialization
  // filename disables nonexistent init-file loading; every checked FIFO byte
  // is written by this test. Core source/defaults are unchanged.
  defparam dut.u_fifo.MEM_INIT_FILE = DISABLE_RAM_INIT ? "" : " ";
  byte unsigned reference_queue[0:127];
  integer head=0, tail=0, count=0;
  bit pending=0;
  byte unsigned expected_reply=0;
  integer cases=0, errors=0, steps=0, accepted_pushes=0, accepted_pops=0;
  integer consumed_replies=0, immediate_reply_checks=0, pause_reply_checks=0;
  integer read_wraps=0, simultaneous_accepts=0, full_push_rejections=0;
  integer pending_resets=0, pending_flushes=0;
  string case_name="setup";
  logic [31:0] rng=32'h7a91c42d;

  task automatic fail(input string text);
    errors=errors+1;
    if(errors<25) $display("CHECK_FAIL case=%s step=%0d %s",case_name,steps,text);
  endtask
  function automatic byte unsigned pattern(input integer serial);
    return 8'((serial*73)^((serial>>3)*11)^8'ha5);
  endfunction

  task automatic step(input bit enabled, input bit put, input bit get,
                      input bit rst, input bit fl, input byte unsigned value);
    bit accepted_put, accepted_get;
    byte unsigned popped_value;
    @(negedge clk);
    ce=enabled; push=put; pop=get; reset=rst; flush=fl; push_d=value;
    @(posedge clk);
    steps=steps+1;
    if(empty!==(count==0)) fail($sformatf("pre-edge empty mismatch count=%0d empty=%b",count,empty));
    accepted_get=enabled && get && count>0 && !rst && !fl;
    accepted_put=enabled && put && count<128 && !rst && !fl;
    if(enabled && put && count==128 && !rst && !fl) full_push_rejections++;
    if(accepted_get && accepted_put) simultaneous_accepts++;
    if(rst || fl) begin
      if(pending && rst) pending_resets++;
      if(pending && fl) pending_flushes++;
      head=0; tail=0; count=0; pending=0;
    end else begin
      if(enabled && pending) begin
        consumed_replies++;
        if(q!==expected_reply)
          fail($sformatf("consumed response expected=%h actual=%h",expected_reply,q));
      end
      if(accepted_get) begin
        popped_value=reference_queue[head];
        expected_reply=popped_value;
        head=(head+1)%128;
        if(head==0) read_wraps++;
        accepted_pops++;
      end
      if(accepted_put) begin
        reference_queue[tail]=value;
        tail=(tail+1)%128;
        accepted_pushes++;
      end
      count=count+integer'(accepted_put)-integer'(accepted_get);
      if(enabled) pending=accepted_get;
    end
    #1;
    if(empty!==(count==0)) fail($sformatf("post-edge empty mismatch count=%0d empty=%b",count,empty));
    if(accepted_get) begin
      immediate_reply_checks++;
      if(q!==popped_value)
        fail($sformatf("one-clock response expected=%h actual=%h",popped_value,q));
    end
    if(!enabled && pending && !rst && !fl) begin
      pause_reply_checks++;
      if(q!==expected_reply)
        fail($sformatf("paused pending response expected=%h actual=%h",expected_reply,q));
    end
  endtask
  task automatic clean;
    step(1,0,0,1,0,0);
    step(1,0,0,0,0,0);
  endtask
  task automatic drain;
    while(count>0) step(1,0,1,0,0,0);
    step(1,0,0,0,0,0); // consume the final registered reply
  endtask

  initial begin
    integer pause_length, before_errors;
    case_name="continuous_ce_wrap_simultaneous"; cases++;
    before_errors=errors;
    clean();
    for(integer i=0;i<100;i++) step(1,1,0,0,0,pattern(i));
    for(integer i=100;i<1800;i++) step(1,1,1,0,0,pattern(i));
    drain();
    $display("FIFO_CASE_RESULT name=%s errors=%0d",case_name,errors-before_errors);

    // Every read address, four pause lengths, including the final in-flight
    // byte when FIFO becomes empty. Each starts with a full128-byte ring.
    before_errors=errors;
    for(integer address=0;address<128;address++) begin
      for(integer length_kind=0;length_kind<4;length_kind++) begin
        case(length_kind) 0:pause_length=1; 1:pause_length=2;
                         2:pause_length=7; default:pause_length=33; endcase
        case_name=$sformatf("pause_address_%0d_length_%0d",address,pause_length);
        cases++;
        clean();
        for(integer i=0;i<128;i++) step(1,1,0,0,0,pattern(i+address));
        // Full FIFO rejects this extra push; no pop is requested on this edge.
        step(1,1,0,0,0,8'hfe);
        for(integer i=0;i<=address;i++) step(1,0,1,0,0,0);
        for(integer i=0;i<pause_length;i++)
          step(0,1,1,0,0,8'hff); // attempted requests must be ignored under ce0
        drain();
      end
    end
    $display("FIFO_CASE_GROUP name=all128addresses_four_pause_lengths cases=512 errors=%0d",errors-before_errors);

    before_errors=errors;
    for(integer clear_kind=0;clear_kind<2;clear_kind++) begin
      for(integer enabled_clear=0;enabled_clear<2;enabled_clear++) begin
        for(integer gap=0;gap<4;gap++) begin
          case_name=$sformatf("pending_clear_kind%0d_ce%0d_gap%0d",clear_kind,enabled_clear,gap);
          cases++;
          clean();
          for(integer i=0;i<16;i++) step(1,1,0,0,0,pattern(i));
          step(1,0,1,0,0,0);
          for(integer i=0;i<gap;i++) step(0,0,0,0,0,0);
          step(enabled_clear,0,0,clear_kind==0,clear_kind==1,0);
          step(1,0,0,0,0,0);
          for(integer i=0;i<5;i++) step(1,1,0,0,0,pattern(i+51));
          drain();
        end
      end
    end
    $display("FIFO_CASE_GROUP name=reset_flush_pending_reply cases=16 errors=%0d",errors-before_errors);

    case_name="pseudorandom_queue_50000cycles"; cases++;
    before_errors=errors;
    clean();
    for(integer i=0;i<50000;i++) begin
      rng={rng[30:0],rng[31]^rng[21]^rng[1]^rng[0]};
      step(rng[0]||rng[4], rng[2]||rng[7], rng[3]||rng[8],
           rng[19:10]==0, rng[29:20]==0, pattern(i));
    end
    drain();
    $display("FIFO_CASE_RESULT name=%s errors=%0d",case_name,errors-before_errors);
    $display("FIFO_COVERAGE steps=%0d pushes=%0d pops=%0d consumed=%0d immediate_checks=%0d paused_checks=%0d wraps=%0d simultaneous=%0d full_rejections=%0d pending_resets=%0d pending_flushes=%0d",steps,accepted_pushes,accepted_pops,consumed_replies,immediate_reply_checks,pause_reply_checks,read_wraps,simultaneous_accepts,full_push_rejections,pending_resets,pending_flushes);
    $display("FIFO_SUITE_RESULT cases=%0d errors=%0d",cases,errors);
    if(errors) $fatal(1,"FIFO transport assertions failed");
    $finish;
  end
  initial begin #100000000; $fatal(1,"FIFO global timeout"); end
endmodule
