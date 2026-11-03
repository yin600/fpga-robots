// 3 路 HC-SR04 轮流测距（左/中/右），输出毫米
// 注意：HC-SR04 的 ECHO 是 5V，必须经 BSS138 电平转换后再接 FPGA（3.3V）
// d* = 4000 表示该次超时无回波（视为前方无障碍）；fault[i]=1 表示该路传感器从未出现回波上升沿（多半是线没接好）
module ultrasonic_scan #(
    parameter CLK_HZ = 50_000_000
)(
    input             clk,
    input             rst_n,
    input      [2:0]  echo,
    output reg [2:0]  trig,
    output reg [15:0] d0,
    output reg [15:0] d1,
    output reg [15:0] d2,
    output reg [2:0]  fault,
    output reg        updated
);
    localparam integer US_DIV = CLK_HZ / 1_000_000;

    localparam [2:0] S_TRIG = 3'd0, S_RISE = 3'd1, S_MEAS = 3'd2, S_GAP = 3'd3;

    reg [7:0]  uc;
    reg [2:0]  es0, es1;
    reg [2:0]  st;
    reg [1:0]  idx;
    reg [15:0] t;

    wire us_tick = (uc == US_DIV - 1);
    wire e = es1[idx];
    wire [31:0] mm = (t * 32'd11240) >> 16;     // us * 0.1715 = mm

    always @(posedge clk) begin
        es0 <= echo;
        es1 <= es0;
        if (!rst_n || us_tick) uc <= 8'd0;
        else                   uc <= uc + 8'd1;
    end

    task store;
        input [1:0]  i;
        input [15:0] v;
        begin
            case (i)
                2'd0: d0 <= v;
                2'd1: d1 <= v;
                default: d2 <= v;
            endcase
        end
    endtask

    always @(posedge clk) begin
        updated <= 1'b0;
        if (!rst_n) begin
            st    <= S_TRIG;
            idx   <= 2'd0;
            t     <= 16'd0;
            trig  <= 3'b000;
            d0    <= 16'd4000;
            d1    <= 16'd4000;
            d2    <= 16'd4000;
            fault <= 3'b111;
        end else if (us_tick) begin
            case (st)
                S_TRIG: begin
                    trig      <= 3'b000;
                    trig[idx] <= 1'b1;
                    t         <= t + 16'd1;
                    if (t >= 16'd12) begin
                        trig <= 3'b000;
                        t    <= 16'd0;
                        st   <= S_RISE;
                    end
                end
                S_RISE: begin
                    if (e) begin
                        t  <= 16'd0;
                        st <= S_MEAS;
                        fault[idx] <= 1'b0;
                    end else if (t >= 16'd5000) begin
                        store(idx, 16'd4000);
                        fault[idx] <= 1'b1;
                        t  <= 16'd0;
                        st <= S_GAP;
                    end else begin
                        t <= t + 16'd1;
                    end
                end
                S_MEAS: begin
                    if (!e) begin
                        store(idx, mm[15:0]);
                        t  <= 16'd0;
                        st <= S_GAP;
                    end else if (t >= 16'd30000) begin
                        store(idx, 16'd4000);
                        t  <= 16'd0;
                        st <= S_GAP;
                    end else begin
                        t <= t + 16'd1;
                    end
                end
                default: begin   // S_GAP：等回波衰减，避免串扰
                    if (t >= 16'd30000) begin
                        t  <= 16'd0;
                        st <= S_TRIG;
                        if (idx == 2'd2) begin
                            idx     <= 2'd0;
                            updated <= 1'b1;
                        end else begin
                            idx <= idx + 2'd1;
                        end
                    end else begin
                        t <= t + 16'd1;
                    end
                end
            endcase
        end
    end
endmodule
