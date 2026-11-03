// PWM 发生器：周期 = PERIOD 个时钟（50MHz/2500 = 20kHz），占空比 = duty/PERIOD
module pwm_gen #(
    parameter PERIOD = 2500,
    parameter W      = 12
)(
    input          clk,
    input          rst_n,
    input  [W-1:0] duty,
    output reg     pwm
);
    reg [W-1:0] cnt;

    always @(posedge clk) begin
        if (!rst_n) begin
            cnt <= {W{1'b0}};
            pwm <= 1'b0;
        end else begin
            if (cnt == PERIOD - 1) cnt <= {W{1'b0}};
            else                   cnt <= cnt + 1'b1;
            pwm <= (cnt < duty);
        end
    end
endmodule
