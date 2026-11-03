// 航向控制（P 控制 + 最小转速 + 限幅），每 10ms 运行一次
//   err = tgt - yaw （Q16 度）；w = err[度] * kp / 8，限幅 ±wmax，|w| 过小时抬到 wmin 克服静摩擦
//   w 的单位与速度环一致（编码器计数/10ms，左右轮差速的一半）
//   at_target：|err| < tol_deg 连续 20 个周期（200ms）
module heading_ctrl (
    input                     clk,
    input                     rst_n,
    input                     tick,
    input                     en,
    input  signed [31:0]      yaw_q16,
    input  signed [31:0]      tgt_q16,
    input  signed [15:0]      kp,        // 默认 16 -> w = 2*err[度]
    input  signed [15:0]      wmax,
    input  signed [15:0]      wmin,
    input  signed [15:0]      tol_deg,   // 容差（度）
    output reg signed [15:0]  w,
    output reg                at_target
);
    wire signed [31:0] err_q16 = tgt_q16 - yaw_q16;
    wire signed [31:0] err_q8  = err_q16 >>> 8;
    wire signed [47:0] raw     = (err_q8 * kp) >>> 11;     // = err[度]*256*kp/2048
    wire signed [47:0] tol_q8  = tol_deg <<< 8;

    wire err_neg = err_q16[31];
    wire signed [31:0] aerr = err_neg ? -err_q8 : err_q8;
    wire inside = (aerr < tol_q8);

    reg [7:0] hold;
    reg signed [15:0] w_n;

    always @* begin
        if (raw > wmax)       w_n = wmax;
        else if (raw < -wmax) w_n = -wmax;
        else                  w_n = raw[15:0];
        if (!inside) begin
            if (w_n >= 0 && w_n < wmin && !err_neg) w_n = wmin;
            if (w_n <= 0 && w_n > -wmin && err_neg) w_n = -wmin;
        end else begin
            w_n = 16'sd0;
        end
    end

    always @(posedge clk) begin
        if (!rst_n || !en) begin
            w         <= 16'sd0;
            at_target <= 1'b0;
            hold      <= 8'd0;
        end else if (tick) begin
            w <= w_n;
            if (inside) begin
                if (hold != 8'd255) hold <= hold + 8'd1;
                if (hold >= 8'd20)  at_target <= 1'b1;
            end else begin
                hold      <= 8'd0;
                at_target <= 1'b0;
            end
        end
    end
endmodule
