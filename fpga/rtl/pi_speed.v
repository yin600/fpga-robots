// 单侧速度环：前馈 + PI，每 10ms 运行一次
// 单位：target / meas = 编码器计数 / 10ms；kf/kp/ki 为 Q8 定点（256 = 1.0）
//   cmd = (target*kf + err*kp + integ*ki) >>> 8，限幅到 ±PERIOD
module pi_speed #(
    parameter PERIOD = 2500
)(
    input                     clk,
    input                     rst_n,
    input                     tick,
    input                     en,
    input  signed [15:0]      target,
    input  signed [15:0]      meas,
    input  signed [15:0]      kf,
    input  signed [15:0]      kp,
    input  signed [15:0]      ki,
    output reg signed [12:0]  cmd
);
    localparam signed [23:0] INT_MAX = 24'sd400000;

    reg  signed [23:0] integ;

    wire signed [16:0] err = {target[15], target} - {meas[15], meas};
    wire signed [39:0] p_ff = target * kf;
    wire signed [39:0] p_p  = err * kp;
    wire signed [39:0] p_i  = integ * ki;
    wire signed [39:0] sum  = (p_ff + p_p + p_i) >>> 8;

    wire sat_hi = (sum >  PERIOD);
    wire sat_lo = (sum < -PERIOD);
    // 抗积分饱和：已饱和且误差仍朝同方向推时不再积分
    wire hold_i = (sat_hi && !err[16]) || (sat_lo && err[16]);

    always @(posedge clk) begin
        if (!rst_n || !en || target == 16'sd0) begin
            integ <= 24'sd0;
            cmd   <= 13'sd0;
        end else if (tick) begin
            if (!hold_i) begin
                if (integ + err > INT_MAX)        integ <= INT_MAX;
                else if (integ + err < -INT_MAX)  integ <= -INT_MAX;
                else                              integ <= integ + err;
            end
            if (sat_hi)      cmd <= PERIOD;
            else if (sat_lo) cmd <= -PERIOD;
            else             cmd <= sum[12:0];
        end
    end
endmodule
