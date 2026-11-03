// 避障限速：根据左/中/右三路超声波距离(mm)给出 速度缩放(0~256) 与 避让转向 w
//   距离 >= slow_mm      ：scale=256（全速），不干预
//   stop_mm ~ slow_mm    ：分 4 档线性减速
//   <= stop_mm           ：scale=0（停车），并原地朝较空旷一侧转（方向在停车瞬间锁存，避免来回抖）
// w>0 表示向左转（左轮慢右轮快）；avoid_active=1 时上层应放弃航向保持，只用 avoid_w
module speed_limit (
    input                     clk,
    input                     rst_n,
    input             [15:0]  dl,
    input             [15:0]  dc,
    input             [15:0]  dr,
    input             [15:0]  stop_mm,
    input             [15:0]  slow_mm,
    input      signed [15:0]  w_avoid,
    output reg        [8:0]   scale,
    output reg signed [15:0]  avoid_w,
    output reg                avoid_active,
    output reg                blocked
);
    wire [15:0] mn_lr = (dl < dr) ? dl : dr;
    wire [15:0] mn    = (dc < mn_lr) ? dc : mn_lr;

    wire [15:0] range = (slow_mm > stop_mm) ? (slow_mm - stop_mm) : 16'd4;
    wire [15:0] t1 = stop_mm + (range >> 2);
    wire [15:0] t2 = stop_mm + (range >> 1);
    wire [15:0] t3 = stop_mm + (range >> 1) + (range >> 2);

    wire left_blk   = (dl < slow_mm);
    wire right_blk  = (dr < slow_mm);
    wire center_blk = (dc < slow_mm);

    // 1 = 向左转（w>0）
    wire go_left = center_blk ? (dl >= dr) :
                   (left_blk && !right_blk) ? 1'b0 :
                   (right_blk && !left_blk) ? 1'b1 : (dl >= dr);

    reg        dir_l;     // 停车时锁存的方向
    reg        was_zero;

    wire [8:0] scale_n = (mn <= stop_mm) ? 9'd0   :
                         (mn <= t1)      ? 9'd64  :
                         (mn <= t2)      ? 9'd128 :
                         (mn <= t3)      ? 9'd192 :
                         (mn <  slow_mm) ? 9'd224 : 9'd256;

    always @(posedge clk) begin
        if (!rst_n) begin
            scale        <= 9'd256;
            avoid_w      <= 16'sd0;
            avoid_active <= 1'b0;
            blocked      <= 1'b0;
            dir_l        <= 1'b1;
            was_zero     <= 1'b0;
        end else begin
            scale        <= scale_n;
            avoid_active <= left_blk || right_blk || center_blk;
            blocked      <= (scale_n == 9'd0);
            was_zero     <= (scale_n == 9'd0);
            if (scale_n == 9'd0 && !was_zero) dir_l <= go_left;
            if (!(left_blk || right_blk || center_blk))
                avoid_w <= 16'sd0;
            else if (scale_n == 9'd0)
                avoid_w <= (was_zero ? dir_l : go_left) ? w_avoid : -w_avoid;
            else
                avoid_w <= go_left ? w_avoid : -w_avoid;
        end
    end
endmodule
