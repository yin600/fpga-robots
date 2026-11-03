// SPI 主机，模式 3（CPOL=1, CPHA=1），MSB 先发；一次传 1 字节，CS 由上层控制
// SCLK = CLK_HZ / (4*HALF)；默认 HALF=13 -> 50M/52 ≈ 0.96 MHz（ICM-42688 最高 24MHz，1MHz 足够）
module spi_master #(
    parameter HALF = 13
)(
    input            clk,
    input            rst_n,
    input            start,
    input      [7:0] tx,
    output reg [7:0] rx,
    output reg       busy,
    output reg       done,
    output reg       sclk,
    output reg       mosi,
    input            miso
);
    reg [7:0] sh;
    reg [2:0] bitn;
    reg [7:0] cnt;
    reg       ph;
    reg [1:0] ms;

    always @(posedge clk) begin
        ms   <= {ms[0], miso};
        done <= 1'b0;
        if (!rst_n) begin
            busy <= 1'b0;
            sclk <= 1'b1;
            mosi <= 1'b0;
            rx   <= 8'd0;
            sh   <= 8'd0;
            bitn <= 3'd0;
            cnt  <= 8'd0;
            ph   <= 1'b0;
        end else if (!busy) begin
            if (start) begin
                busy <= 1'b1;
                sh   <= tx;
                bitn <= 3'd0;
                sclk <= 1'b0;          // 下降沿：送出最高位
                mosi <= tx[7];
                cnt  <= HALF - 1;
                ph   <= 1'b0;
            end
        end else if (cnt != 8'd0) begin
            cnt <= cnt - 8'd1;
        end else begin
            cnt <= HALF - 1;
            if (!ph) begin             // 上升沿：采样 MISO
                sclk <= 1'b1;
                rx   <= {rx[6:0], ms[1]};
                ph   <= 1'b1;
            end else begin
                if (bitn == 3'd7) begin
                    busy <= 1'b0;
                    done <= 1'b1;
                end else begin
                    sclk <= 1'b0;      // 下降沿：送出下一位
                    bitn <= bitn + 3'd1;
                    sh   <= {sh[6:0], 1'b0};
                    mosi <= sh[6];
                    ph   <= 1'b0;
                end
            end
        end
    end
endmodule
