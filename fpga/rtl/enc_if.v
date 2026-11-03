// 正交编码器接口：四倍频计数 + 每 10ms 输出一次速度（单位：计数 / 10ms）
// 正转计数增加；方向反了把 INVERT 改为 1
module enc_if #(
    parameter CLK_HZ = 50_000_000,
    parameter INVERT = 0
)(
    input                    clk,
    input                    rst_n,
    input                    a_in,
    input                    b_in,
    output reg signed [31:0] pos,
    output reg signed [15:0] speed,
    output reg               tick10      // 每 10ms 一个时钟周期的脉冲
);
    localparam integer TICK_CYC = CLK_HZ / 100;

    reg [1:0]  a_s, b_s;
    reg [1:0]  prev;
    wire [1:0] cur = {a_s[1], b_s[1]};
    reg [31:0] tc;
    reg signed [31:0] last;

    always @(posedge clk) begin
        a_s <= {a_s[0], a_in};
        b_s <= {b_s[0], b_in};
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            pos  <= 32'sd0;
            prev <= 2'b00;
        end else begin
            prev <= cur;
            case ({prev, cur})
                4'b0001, 4'b0111, 4'b1110, 4'b1000: pos <= (INVERT != 0) ? pos - 32'sd1 : pos + 32'sd1;
                4'b0010, 4'b1011, 4'b1101, 4'b0100: pos <= (INVERT != 0) ? pos + 32'sd1 : pos - 32'sd1;
                default: ;
            endcase
        end
    end

    always @(posedge clk) begin
        tick10 <= 1'b0;
        if (!rst_n) begin
            tc    <= 32'd0;
            last  <= 32'sd0;
            speed <= 16'sd0;
        end else if (tc == TICK_CYC - 1) begin
            tc     <= 32'd0;
            tick10 <= 1'b1;
            speed  <= pos - last;
            last   <= pos;
        end else begin
            tc <= tc + 32'd1;
        end
    end
endmodule
