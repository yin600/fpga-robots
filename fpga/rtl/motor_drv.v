// TB6612 电机驱动接口：带符号命令 -> PWM + IN1/IN2 + STBY
// cmd 范围 -PERIOD ~ +PERIOD；brake=1 时 IN1=IN2=1 短接制动
module motor_drv #(
    parameter PERIOD = 2500,
    parameter W      = 12
)(
    input                clk,
    input                rst_n,
    input  signed [12:0] cmd,
    input                en,
    input                brake,
    output               pwm,
    output reg           in1,
    output reg           in2,
    output               stby
);
    wire [12:0]  mag  = cmd[12] ? (~cmd + 13'd1) : cmd;
    wire [12:0]  lim  = (mag > PERIOD) ? PERIOD : mag;
    wire [W-1:0] duty = lim[W-1:0];

    pwm_gen #(.PERIOD(PERIOD), .W(W)) u_pwm (
        .clk(clk), .rst_n(rst_n), .duty(duty), .pwm(pwm)
    );

    assign stby = en;

    always @(posedge clk) begin
        if (!rst_n || !en) begin
            in1 <= 1'b0;
            in2 <= 1'b0;
        end else if (brake) begin
            in1 <= 1'b1;
            in2 <= 1'b1;
        end else if (cmd == 13'sd0) begin
            in1 <= 1'b0;
            in2 <= 1'b0;
        end else if (!cmd[12]) begin
            in1 <= 1'b1;
            in2 <= 1'b0;
        end else begin
            in1 <= 1'b0;
            in2 <= 1'b1;
        end
    end
endmodule
