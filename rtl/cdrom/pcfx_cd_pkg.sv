// CD drive constants, timing and saved state.
//
// Time runs on the drive's own 16.9344 MHz crystal (ce_x) and on
// crystal / 192 = 88.2 kHz (ce_t, 11.34 us), the tick everything slower
// than a bus handshake counts in. Positions are absolute frame numbers,
// 00:00:00 = 0, so LBA 0 = 150; that is also the block number on the HPS
// slot.

package pcfx_cd_pkg;

	// Slot buffer (64 KB): a ring of 24 2448-byte blocks, then the TOC
	// record. Blocks come six to a request, aligned as Linux caches them.
	localparam logic [4:0]  RING_N    = 5'd24;
	localparam logic [4:0]  REQ_N     = 5'd6;
	localparam logic [15:0] TOC_BASE  = 16'd58752;  // track t at +4(t-1): F, S, M, control
	localparam logic [15:0] IDX0_BASE = 16'd59152;  // track t pregap start at +4(t-1)

	// Mechanism, slow ticks
	localparam logic [19:0] M_SPINUP  = 20'd160500; // 1.82 s from reset or a new disc
	localparam logic [19:0] M_NODISC  = 20'd532300; // 6.04 s looking for a disc that is not there
	localparam logic [15:0] M_SEEK0   = 16'd9700;   // 110 ms local overhead
	localparam logic [18:0] M_SEEK_LOCAL = 19'd12000;
	localparam logic [7:0]  M_RA_MAX  = 8'd100;     // sectors the read-ahead buffer holds
	localparam logic [4:0]  M_THRU    = 5'd16;      // forward gap read through, not sought

	// Sense codes as an index; sense_kc gives {key, sub-error}
	localparam logic [3:0] E_NONE      = 4'd0;
	localparam logic [3:0] E_NOT_READY = 4'd1;   // 2/04
	localparam logic [3:0] E_NO_DISC   = 4'd2;   // 2/0B
	localparam logic [3:0] E_HEADER    = 4'd3;   // 3/16
	localparam logic [3:0] E_NOT_AUDIO = 4'd4;   // 3/1C
	localparam logic [3:0] E_NOT_DATA  = 4'd5;   // 3/1D
	localparam logic [3:0] E_BAD_CMD   = 4'd6;   // 5/20
	localparam logic [3:0] E_BAD_ADDR  = 4'd7;   // 5/21
	localparam logic [3:0] E_BAD_PARAM = 4'd8;   // 5/22
	localparam logic [3:0] E_EOV       = 4'd9;   // 5/25
	localparam logic [3:0] E_BAD_CDB   = 4'd10;  // 5/27
	localparam logic [3:0] E_NOT_PLAY  = 4'd11;  // 5/2C
	localparam logic [3:0] E_RESET     = 4'd12;  // 6/29
	localparam logic [3:0] E_CHANGED   = 4'd13;  // 6/28

	function automatic logic [11:0] sense_kc(input logic [3:0] e);
		case (e)
		E_NOT_READY: sense_kc = 12'h204;
		E_NO_DISC:   sense_kc = 12'h20B;
		E_HEADER:    sense_kc = 12'h316;
		E_NOT_AUDIO: sense_kc = 12'h31C;
		E_NOT_DATA:  sense_kc = 12'h31D;
		E_BAD_CMD:   sense_kc = 12'h520;
		E_BAD_ADDR:  sense_kc = 12'h521;
		E_BAD_PARAM: sense_kc = 12'h522;
		E_EOV:       sense_kc = 12'h525;
		E_BAD_CDB:   sense_kc = 12'h527;
		E_NOT_PLAY:  sense_kc = 12'h52C;
		E_RESET:     sense_kc = 12'h629;
		E_CHANGED:   sense_kc = 12'h628;
		default:     sense_kc = 12'h000;
		endcase
	endfunction

	// Mechanism operations
	localparam logic [3:0] MO_READ   = 4'd1;   // position for reading at lba
	localparam logic [3:0] MO_SEEK   = 4'd2;   // done on arrival
	localparam logic [3:0] MO_PLAY   = 4'd3;   // play lba .. end (mode: 0 mute, 4 repeat, else stereo)
	localparam logic [3:0] MO_SEARCH = 4'd4;   // seek lba, then play to the end or pause (mode[0])
	localparam logic [3:0] MO_SETEND = 4'd5;   // new end and mode, play on
	localparam logic [3:0] MO_PAUSE  = 4'd6;   // still
	localparam logic [3:0] MO_RESUME = 4'd7;
	localparam logic [3:0] MO_STOP   = 4'd8;   // audio stops
	localparam logic [3:0] MO_ABORT  = 4'd9;   // message out: reading and audio stop
	localparam logic [3:0] MO_FLUSH  = 4'd10;  // read-ahead dropped (new data format)
	localparam logic [3:0] MO_REZERO = 4'd11;  // stop and home

	// Play states as READ SUBCODE-Q reports them
	localparam logic [1:0] PS_PLAY   = 2'd0;
	localparam logic [1:0] PS_STILL  = 2'd1;
	localparam logic [1:0] PS_SEARCH = 2'd2;
	localparam logic [1:0] PS_STOP   = 2'd3;

	typedef struct packed {
		logic        p0e_immed, p0e_b3;
		logic [1:0]  p0e_sel0, p0e_sel1;
		logic [31:0] p28;
		logic [7:0]  p29;
		logic [15:0] p2a;
		logic [7:0]  p2b;
		logic [7:0]  vflags;
		logic [7:0]  vrecovery;
		logic [3:0]  vretry;
		logic [7:0]  density;
		logic [11:0] block_len;
		logic        custom;
		logic        p28_applied;
	} scsi_mode_t;

	typedef struct packed {
		logic [3:0]  ph;        // bus state
		logic [2:0]  pl;        // phase lines {MSG, C/D, I/O}
		logic [1:0]  hs;        // handshake: 0 wait, 1 REQ, 2 ACK seen
		logic [15:0] dly;
		logic        dslow;     // dly counts slow ticks
		logic        rst_d;
		logic [95:0] cdb;       // byte 0 in [95:88]
		logic [3:0]  cdb_n;
		logic [4:0]  xs;        // command step
		logic        second;    // the address unit works on the end address
		logic [3:0]  rt;        // reply kind
		logic [10:0] len;       // reply bytes
		logic [10:0] pidx;      // next reply byte to build
		logic [3:0]  fld;       // byte within a TOC entry
		logic [6:0]  ent;       // TOC entry
		logic [6:0]  trk;       // its track
		logic [2:0]  fs;        // fetch step
		logic        efetched;
		logic [7:0]  e_m, e_s, e_f, e_c;
		logic [19:0] xr;        // an LBA value for replies
		logic [18:0] tgt, tgt2; // addresses for the mechanism
		logic [3:0]  mop;
		logic [2:0]  mmode;
		logic [23:0] scnt;      // sectors left to build
		logic [11:0] sk;        // next sector byte to build
		logic        mode2;
		logic [1:0]  sec_init;
		logic [11:0] sect_src;
		logic [11:0] sect_len;
		logic        sfirst;    // still the command's first sector
		logic        waited;    // the first sector came off the disc
		logic [7:0]  nb;        // next byte
		logic        nbv;
		logic        nb_first;  // it starts a sector
		logic        nb_last;
		logic        pdone;
		logic [7:0]  dbo;
		logic        dbv;
		logic        db_last;
		logic        dfirst;    // nothing sent yet in this phase
		logic [3:0]  sense;
		logic [1:0]  ua;        // 1 reset, 2 disc changed
		logic [3:0]  err;
		logic [18:0] cv;        // frames = m * 4500 + s * 75 + f
		logic [6:0]  cv_m;
		logic [5:0]  cv_s;
		logic        cv_busy;
		scsi_mode_t  mode;
		scsi_mode_t  ml_mode;
		logic [7:0]  ml_left;   // MODE SELECT list bytes to come
		logic [2:0]  ml_st;
		logic [7:0]  ml_n;
		logic [7:0]  ml_page;
		logic [7:0]  ml_i;
		logic        ml_err;
		logic        ml_bd;     // a block descriptor came: the buffer is dropped
		logic [1:0]  msg_return;
		logic        m_go;
		logic        p_next;
		logic        bus_rst;
	} scsi_t;

	typedef struct packed {
		logic [1:0]  spin;      // 0 no disc, 1 spinning up, 2 ready
		logic        newdisc;   // spinning up a new disc
		logic [19:0] timer;
		logic [2:0]  mode;      // what the head does
		logic [2:0]  after;     // mode when a seek arrives
		logic [18:0] head;      // sector under the head next
		logic [18:0] rd_end;    // read into the buffer up to here
		logic [18:0] p;         // next sector the host gets
		logic [18:0] tgt;       // seek target, or where a pause loops
		logic [18:0] pend;      // play end, exclusive
		logic [18:0] pstart;
		logic [2:0]  pmode;
		logic [1:0]  pst;       // play state
		logic [3:0]  ft;        // slow ticks within the frame
		logic [6:0]  fr;        // frame within the sector
		logic        resume;    // the hit found the head looping
		logic        op_pend;
		logic        obusy;
		logic        done;
		logic        wait_arr;  // the op is done on arrival
		logic [3:0]  err;
		logic [3:0]  q_ctrl;    // Q of the head, BCD
		logic [7:0]  q_tno;
		logic        q_idx;
		logic [23:0] q_rel, q_abs;
		logic        q_pend;    // head moved: Q to rebuild
		logic [18:0] rd_limit;
		logic        rd_valid;
		logic        toc_pend;
		logic        f_start;
	} mech_t;

	typedef struct packed {
		logic [7:0]  sh;        // P-W byte being shifted out, P first
		logic [15:0] crc;       // Q CRC of the block so far
		logic        exck_d;
	} sub_t;

	typedef struct packed {
		logic [11:0] acc;       // crystal phase against clk_sys
		logic [7:0]  div;       // crystal / 192
		logic        fs;        // every other slow tick: 44.1 kHz
	} clk_t;

	localparam int CLK_BITS  = $bits(clk_t);
	localparam int SCSI_BITS = $bits(scsi_t);
	localparam int MECH_BITS = $bits(mech_t);
	localparam int SUB_BITS  = $bits(sub_t);
	/* verilator lint_off UNUSEDPARAM */
	localparam int SS_BITS   = CLK_BITS + SCSI_BITS + MECH_BITS + SUB_BITS;
	/* verilator lint_on UNUSEDPARAM */

endpackage
