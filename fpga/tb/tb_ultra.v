`timescale 1ns/1ps
// 运行：iverilog -o t_us tb/tb_ultra.v rtl/ultrasonic_scan.v && vvp t_us
// 模型：每路收到 trig 后 100us 回波拉高 2000us -> 应测得约 343 mm
module tb_ultra;
    reg clk = 0, rst_n = 0;
    always #10 clk = ~clk;
    wire [2:0] trig; reg [2:0] echo = 0;
    wire [15:0] d0, d1, d2; wire [2:0] fault; wire updated;
    ultrasonic_scan dut(.clk(clk), .rst_n(rst_n), .echo(echo), .trig(trig), .d0(d0), .d1(d1), .d2(d2), .fault(fault), .updated(updated));

    genvar g;
    generate for (g = 0; g < 3; g = g + 1) begin : m
        always @(posedge trig[g]) begin
            #100000 echo[g] = 1;
            #(2000000 + 1000000 * g) echo[g] = 0;     // 三路依次 2.0/3.0/4.0 ms
        end
    end endgenerate

    initial begin
        #100 rst_n = 1;
        @(posedge updated);
        $display("d0=%0d (期望≈343)  d1=%0d (期望≈514)  d2=%0d (期望≈686)  fault=%b", d0, d1, d2, fault);
        if (d0 > 330 && d0 < 356 && d1 > 500 && d1 < 530 && d2 > 670 && d2 < 700) $display("TB_ULTRA PASS");
        else $display("TB_ULTRA FAIL");
        $finish;
    end
endmodule
