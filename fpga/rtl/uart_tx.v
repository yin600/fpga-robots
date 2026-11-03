// UART 发送：8N1，低位先发
// 用法：ready=1 时拉高 valid 一个时钟周期并给出 data，即开始发送
module uart_tx #(
    parameter CLK_HZ = 50_000_000,
    parameter BAUD   = 115200
)(
    input        clk,
    input        rst_n,
    input  [7:0] data,
    input        valid,
    output       ready,
    output reg   tx
);
    localparam integer DIV = CLK_HZ / BAUD;

    reg [15:0] cnt;
    reg [3:0]  bit_i;
    reg [9:0]  sh;      // {stop, data[7:0], start}，sh[0] 先发
    reg        busy;

    assign ready = ~busy;

    always @(posedge clk) begin
        if (!rst_n) begin
            busy  <= 1'b0;
            tx    <= 1'b1;
            cnt   <= 16'd0;
            bit_i <= 4'd0;
            sh    <= 10'h3FF;
        end else if (!busy) begin
            if (valid) begin
                sh    <= {1'b1, data, 1'b0};
                busy  <= 1'b1;
                cnt   <= DIV - 1;
                bit_i <= 4'd0;
                tx    <= 1'b0;                 // 起始位
            end
        end else begin
            if (cnt == 16'd0) begin
                cnt <= DIV - 1;
                if (bit_i == 4'd9) begin       // 停止位发送完毕
                    busy <= 1'b0;
                    tx   <= 1'b1;
                end else begin
                    bit_i <= bit_i + 4'd1;
                    tx    <= sh[bit_i + 4'd1];
                end
            end else begin
                cnt <= cnt - 16'd1;
            end
        end
    end
endmodule
