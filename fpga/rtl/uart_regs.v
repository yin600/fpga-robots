// 串口寄存器接口 + 遥测上报（115200 8N1）
//
// 命令帧(PC->FPGA)：0x55, cmd, addr, hi, lo, chk       chk = (cmd+addr+hi+lo) & 0xFF
//   cmd=0x01 写寄存器；cmd=0x02 读寄存器（FPGA 回 0x55,0x82,addr,hi,lo,chk）
// 遥测帧(FPGA->PC，每 20ms 一帧，19 字节)：0xA5,0x5A, 8 个 16 位大端字, chk
//   字顺序：roll, pitch, yaw (Q7 度, 即 度*128, 16 位回绕, PC 端自己展开), 距离左/中/右(mm), 左速, 右速(计数/10ms)
//   chk = 第 3 个字节起到最后一个数据字节之和 & 0xFF
//
// 寄存器表见 README.md
module uart_regs #(
    parameter CLK_HZ = 50_000_000,
    parameter BAUD   = 115200
)(
    input                     clk,
    input                     rst_n,
    input                     rx,
    output                    tx,

    // ---- 可写配置 ----
    output reg [2:0]          mode,
    output reg signed [15:0]  v_set,
    output reg signed [15:0]  w_set,
    output reg signed [15:0]  rot_deg,
    output reg                p_yaw_zero,
    output reg                p_start,
    output reg                p_fclr,
    output reg                wd_dis,
    output reg signed [15:0]  kp_spd,
    output reg signed [15:0]  ki_spd,
    output reg signed [15:0]  kf_spd,
    output reg [15:0]         stop_mm,
    output reg [15:0]         slow_mm,
    output reg [3:0]          n_legs,
    output reg signed [15:0]  w_avoid,
    output reg signed [15:0]  kp_head,
    output reg signed [15:0]  wmax,
    output reg signed [15:0]  wmin,
    output reg signed [15:0]  tol_deg,
    output reg [127:0]        turn_bus,
    output reg [255:0]        dist_bus,
    output reg                frame_ok,      // 收到一个合法帧（喂狗）

    // ---- 只读状态 ----
    input      [15:0]         s_roll,
    input      [15:0]         s_pitch,
    input      [15:0]         s_yaw,
    input      [15:0]         s_dl,
    input      [15:0]         s_dc,
    input      [15:0]         s_dr,
    input      [15:0]         s_spd_l,
    input      [15:0]         s_spd_r,
    input      [15:0]         s_status,
    input      [31:0]         s_pos
);
    // ---------------- UART 收发 ----------------
    wire [7:0] rd;
    wire       rv;
    reg  [7:0] tdata;
    reg        tvalid;
    wire       tready;

    uart_rx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_rx (.clk(clk), .rst_n(rst_n), .rx(rx), .data(rd), .valid(rv));
    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_tx (.clk(clk), .rst_n(rst_n), .data(tdata), .valid(tvalid), .ready(tready), .tx(tx));

    // ---------------- 解析 ----------------
    reg [2:0]  pi;
    reg [7:0]  b_cmd, b_addr, b_hi, b_lo;
    reg [23:0] rx_to;
    reg        exec;
    reg        rep_pend;
    reg        load_rep;
    reg [7:0]  rep_addr;

    wire [7:0]  sum4 = b_cmd + b_addr + b_hi + b_lo;
    wire [15:0] wd   = {b_hi, b_lo};

    // 读寄存器（组合逻辑，按 rep_addr 选择）
    reg [15:0] rv16;
    wire [7:0] a = rep_addr;
    always @* begin
        begin
            case (a)
                8'h00: rv16 = {13'd0, mode};
                8'h01: rv16 = v_set;
                8'h02: rv16 = w_set;
                8'h03: rv16 = rot_deg;
                8'h04: rv16 = {15'd0, wd_dis};
                8'h05: rv16 = kp_spd;
                8'h06: rv16 = ki_spd;
                8'h07: rv16 = kf_spd;
                8'h08: rv16 = stop_mm;
                8'h09: rv16 = slow_mm;
                8'h0A: rv16 = {12'd0, n_legs};
                8'h0B: rv16 = w_avoid;
                8'h0C: rv16 = kp_head;
                8'h0D: rv16 = wmax;
                8'h0E: rv16 = wmin;
                8'h0F: rv16 = tol_deg;
                8'h40: rv16 = s_roll;
                8'h41: rv16 = s_pitch;
                8'h42: rv16 = s_yaw;
                8'h43: rv16 = s_dl;
                8'h44: rv16 = s_dc;
                8'h45: rv16 = s_dr;
                8'h46: rv16 = s_spd_l;
                8'h47: rv16 = s_spd_r;
                8'h48: rv16 = s_status;
                8'h49: rv16 = s_pos[15:0];
                8'h4A: rv16 = s_pos[31:16];
                default: begin
                    if (a[7:5] == 3'b001) begin               // 0x20~0x3F：航点
                        case (a[1:0])
                            2'd0:    rv16 = turn_bus[a[4:2] * 16 +: 16];
                            2'd1:    rv16 = dist_bus[a[4:2] * 32 + 16 +: 16];
                            2'd2:    rv16 = dist_bus[a[4:2] * 32 +: 16];
                            default: rv16 = 16'd0;
                        endcase
                    end else rv16 = 16'd0;
                end
            endcase
        end
    end

    always @(posedge clk) begin
        p_yaw_zero <= 1'b0;
        p_start    <= 1'b0;
        p_fclr     <= 1'b0;
        frame_ok   <= 1'b0;
        exec       <= 1'b0;
        if (!rst_n) begin
            pi <= 3'd0; rx_to <= 24'd0;
            b_cmd <= 8'd0; b_addr <= 8'd0; b_hi <= 8'd0; b_lo <= 8'd0;
            rep_pend <= 1'b0; rep_addr <= 8'd0;
            mode <= 3'd0; v_set <= 16'sd0; w_set <= 16'sd0; rot_deg <= 16'sd90;
            wd_dis <= 1'b0;
            kp_spd <= 16'sd300; ki_spd <= 16'sd30; kf_spd <= 16'sd3000;
            stop_mm <= 16'd250; slow_mm <= 16'd600; n_legs <= 4'd0;
            w_avoid <= 16'sd30; kp_head <= 16'sd16; wmax <= 16'sd60; wmin <= 16'sd12; tol_deg <= 16'sd3;
            turn_bus <= 128'd0; dist_bus <= 256'd0;
        end else begin
            // ---- 字节接收 ----
            if (rv) begin
                rx_to <= 24'd0;
                case (pi)
                    3'd0: if (rd == 8'h55) pi <= 3'd1;
                    3'd1: begin b_cmd  <= rd; pi <= 3'd2; end
                    3'd2: begin b_addr <= rd; pi <= 3'd3; end
                    3'd3: begin b_hi   <= rd; pi <= 3'd4; end
                    3'd4: begin b_lo   <= rd; pi <= 3'd5; end
                    default: begin
                        pi <= 3'd0;
                        if (rd == sum4) exec <= 1'b1;
                    end
                endcase
            end else begin
                rx_to <= rx_to + 24'd1;
                if (rx_to == 24'd1_000_000) pi <= 3'd0;     // 20ms 没有后续字节则丢弃半帧
            end

            // ---- 执行 ----
            if (exec) begin
                frame_ok <= 1'b1;
                if (b_cmd == 8'h01) begin
                    case (b_addr)
                        8'h00: mode    <= wd[2:0];
                        8'h01: v_set   <= wd;
                        8'h02: w_set   <= wd;
                        8'h03: rot_deg <= wd;
                        8'h04: begin
                            p_yaw_zero <= wd[0];
                            p_start    <= wd[1];
                            p_fclr     <= wd[2];
                            wd_dis     <= wd[4];
                        end
                        8'h05: kp_spd  <= wd;
                        8'h06: ki_spd  <= wd;
                        8'h07: kf_spd  <= wd;
                        8'h08: stop_mm <= wd;
                        8'h09: slow_mm <= wd;
                        8'h0A: n_legs  <= wd[3:0];
                        8'h0B: w_avoid <= wd;
                        8'h0C: kp_head <= wd;
                        8'h0D: wmax    <= wd;
                        8'h0E: wmin    <= wd;
                        8'h0F: tol_deg <= wd;
                        default: begin
                            if (b_addr[7:5] == 3'b001) begin
                                case (b_addr[1:0])
                                    2'd0: turn_bus[b_addr[4:2] * 16 +: 16]      <= wd;
                                    2'd1: dist_bus[b_addr[4:2] * 32 + 16 +: 16] <= wd;
                                    2'd2: dist_bus[b_addr[4:2] * 32 +: 16]      <= wd;
                                    default: ;
                                endcase
                            end
                        end
                    endcase
                end else if (b_cmd == 8'h02) begin
                    rep_pend <= 1'b1;
                    rep_addr <= b_addr;
                end
            end

            // ---- 回复装载在发送逻辑里清除 rep_pend ----
            if (load_rep) rep_pend <= 1'b0;
        end
    end

    // ---------------- 发送 ----------------
    reg  [151:0] fb;
    reg  [4:0]   tlen, tidx;
    reg  [1:0]   hdr;
    reg  [7:0]   cacc;
    reg          tbusy;
    reg  [19:0]  tele_cnt;
    reg          tele_pend;

    wire [127:0] words = {s_spd_r, s_spd_l, s_dr, s_dc, s_dl, s_yaw, s_pitch, s_roll};

    integer k;
    reg [151:0] fbn;
    wire [7:0] cur_byte = (tidx == tlen - 5'd1) ? cacc : fb[tidx * 8 +: 8];

    always @(posedge clk) begin
        tvalid   <= 1'b0;
        load_rep <= 1'b0;
        if (!rst_n) begin
            tbusy <= 1'b0; tidx <= 5'd0; tlen <= 5'd0; hdr <= 2'd0; cacc <= 8'd0;
            tele_cnt <= 20'd0; tele_pend <= 1'b0; fb <= 152'd0; tdata <= 8'd0;
        end else begin
            if (tele_cnt == 20'd999_999) begin
                tele_cnt  <= 20'd0;
                tele_pend <= 1'b1;
            end else tele_cnt <= tele_cnt + 20'd1;

            if (!tbusy) begin
                if (rep_pend) begin
                    fbn = 152'd0;
                    fbn[7:0]   = 8'h55;
                    fbn[15:8]  = 8'h82;
                    fbn[23:16] = rep_addr;
                    fbn[31:24] = rv16[15:8];
                    fbn[39:32] = rv16[7:0];
                    fb <= fbn; tlen <= 5'd6; hdr <= 2'd1; tidx <= 5'd0; cacc <= 8'd0;
                    tbusy <= 1'b1; load_rep <= 1'b1;
                end else if (tele_pend) begin
                    fbn = 152'd0;
                    fbn[7:0]  = 8'hA5;
                    fbn[15:8] = 8'h5A;
                    for (k = 0; k < 8; k = k + 1) begin
                        fbn[(2 + 2*k) * 8 +: 8] = words[k*16 + 8 +: 8];
                        fbn[(3 + 2*k) * 8 +: 8] = words[k*16 +: 8];
                    end
                    fb <= fbn; tlen <= 5'd19; hdr <= 2'd2; tidx <= 5'd0; cacc <= 8'd0;
                    tbusy <= 1'b1; tele_pend <= 1'b0;
                end
            end else if (!tvalid && tready) begin
                tdata  <= cur_byte;
                tvalid <= 1'b1;
                if (tidx >= {3'd0, hdr} && tidx < tlen - 5'd1) cacc <= cacc + cur_byte;
                if (tidx == tlen - 5'd1) tbusy <= 1'b0;
                else                     tidx  <= tidx + 5'd1;
            end
        end
    end
endmodule
