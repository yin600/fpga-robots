// UART 接收：8N1，在每一位的中点采样；valid 为一个时钟周期的脉冲
module uart_rx #(
    parameter CLK_HZ = 50_000_000,
    parameter BAUD   = 115200
)(
    input            clk,
    input            rst_n,
    input            rx,
    output reg [7:0] data,
    output reg       valid
);
    localparam integer DIV = CLK_HZ / BAUD;

    reg [1:0]  rs;       // 两级同步
    reg [15:0] cnt;
    reg [3:0]  n;
    reg        run;
    reg [7:0]  sh;

    always @(posedge clk) begin
        rs    <= {rs[0], rx};
        valid <= 1'b0;
        if (!rst_n) begin
            run  <= 1'b0;
            cnt  <= 16'd0;
            n    <= 4'd0;
            sh   <= 8'd0;
            data <= 8'd0;
        end else if (!run) begin
            if (rs[1] == 1'b0) begin           // 检测到起始位下降沿
                run <= 1'b1;
                cnt <= (DIV / 2) - 1;          // 先走到起始位中点
                n   <= 4'd0;
            end
        end else if (cnt != 16'd0) begin
            cnt <= cnt - 16'd1;
        end else begin
            cnt <= DIV - 1;
            if (n == 4'd0) begin               // 起始位中点复核
                if (rs[1]) run <= 1'b0;        // 毛刺，放弃
                else       n   <= 4'd1;
            end else if (n <= 4'd8) begin      // 8 个数据位，低位先收
                sh <= {rs[1], sh[7:1]};
                n  <= n + 4'd1;
            end else begin                     // 停止位
                run <= 1'b0;
                if (rs[1]) begin
                    data  <= sh;
                    valid <= 1'b1;
                end
            end
        end
    end
endmodule
