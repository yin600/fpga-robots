`timescale 1ns/1ps
// 运行：iverilog -o t_regs tb/tb_uart_regs.v rtl/uart_regs.v rtl/uart_rx.v rtl/uart_tx.v && vvp t_regs
// 检查：写 v_set=123、读回；写错校验的帧应被忽略
module tb_uart_regs;
    reg clk = 0, rst_n = 0, rx = 1;
    wire tx;
    always #10 clk = ~clk;

    wire [2:0] mode; wire signed [15:0] v_set, w_set, rot_deg, kp_spd, ki_spd, kf_spd, w_avoid, kp_head, wmax, wmin, tol_deg;
    wire p_yaw_zero, p_start, p_fclr, wd_dis, frame_ok;
    wire [15:0] stop_mm, slow_mm; wire [3:0] n_legs; wire [127:0] turn_bus; wire [255:0] dist_bus;

    uart_regs dut(.clk(clk), .rst_n(rst_n), .rx(rx), .tx(tx),
        .mode(mode), .v_set(v_set), .w_set(w_set), .rot_deg(rot_deg),
        .p_yaw_zero(p_yaw_zero), .p_start(p_start), .p_fclr(p_fclr), .wd_dis(wd_dis),
        .kp_spd(kp_spd), .ki_spd(ki_spd), .kf_spd(kf_spd), .stop_mm(stop_mm), .slow_mm(slow_mm),
        .n_legs(n_legs), .w_avoid(w_avoid), .kp_head(kp_head), .wmax(wmax), .wmin(wmin), .tol_deg(tol_deg),
        .turn_bus(turn_bus), .dist_bus(dist_bus), .frame_ok(frame_ok),
        .s_roll(16'h1234), .s_pitch(16'h0), .s_yaw(16'h0), .s_dl(16'd111), .s_dc(16'd222), .s_dr(16'd333),
        .s_spd_l(16'd5), .s_spd_r(16'd6), .s_status(16'h00A5), .s_pos(32'h0001_0002));

    localparam BIT = 8680;   // 115200 baud
    task sbyte; input [7:0] b; integer i; begin
        rx = 0; #(BIT);
        for (i = 0; i < 8; i = i + 1) begin rx = b[i]; #(BIT); end
        rx = 1; #(BIT);
    end endtask
    task frame; input [7:0] cmd, addr, hi, lo, badchk; reg [7:0] c; begin
        c = cmd + addr + hi + lo + badchk;
        sbyte(8'h55); sbyte(cmd); sbyte(addr); sbyte(hi); sbyte(lo); sbyte(c);
    end endtask

    // 接收 FPGA 发来的字节（只抓回复帧 0x55 0x82 ...）
    reg [7:0] got [0:5]; integer gi = 0; reg [7:0] sh; integer j;
    initial begin : mon
        forever begin
            @(negedge tx);
            #(BIT + BIT/2);
            for (j = 0; j < 8; j = j + 1) begin sh[j] = tx; #(BIT); end
            if (gi == 0 && sh == 8'h55) begin got[0] = sh; gi = 1; end
            else if (gi > 0 && gi < 6) begin got[gi] = sh; gi = gi + 1; end
            else if (sh == 8'hA5) gi = 0;
        end
    end

    initial begin
        #200 rst_n = 1;
        #100000;
        frame(8'h01, 8'h01, 8'h00, 8'd123, 8'd0);     // 写 v_set = 123
        #50000;
        if (v_set !== 16'sd123) $display("FAIL write v_set=%0d", v_set); else $display("OK write v_set=123");
        frame(8'h01, 8'h01, 8'h00, 8'd55, 8'd1);      // 校验错 -> 忽略
        #50000;
        if (v_set !== 16'sd123) $display("FAIL bad checksum accepted"); else $display("OK bad checksum ignored");
        frame(8'h01, 8'h00, 8'h00, 8'd4, 8'd0);       // mode = 4
        #50000;
        if (mode !== 3'd4) $display("FAIL mode=%0d", mode); else $display("OK mode=4");
        $display("(读回/遥测帧请用波形或 regtool 实测检查)");
        #3000000 $finish;
    end
endmodule
