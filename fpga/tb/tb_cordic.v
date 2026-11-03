`timescale 1ns/1ps
// 运行：iverilog -o t_cordic tb/tb_cordic.v rtl/cordic_atan2.v && vvp t_cordic
module tb_cordic;
    reg clk = 0, rst_n = 0, start = 0;
    reg signed [15:0] x, y;
    wire busy, done;
    wire signed [31:0] angle;
    cordic_atan2 dut(.clk(clk), .rst_n(rst_n), .start(start), .x(x), .y(y), .busy(busy), .done(done), .angle(angle));
    always #10 clk = ~clk;

    integer errs = 0;
    task run;
        input signed [15:0] xi, yi;
        input real expect_deg;
        real got;
        begin
            @(posedge clk); x <= xi; y <= yi; start <= 1;
            @(posedge clk); start <= 0;
            wait (done);
            @(posedge clk);
            got = angle / 65536.0;
            $display("atan2(%0d,%0d) = %f deg (expect %f)", yi, xi, got, expect_deg);
            if ((got - expect_deg) > 1.2 || (got - expect_deg) < -1.2) begin
                errs = errs + 1; $display("  ** FAIL");
            end
        end
    endtask
    initial begin
        #100 rst_n = 1;
        run( 1000,     0,    0.0);
        run( 1000,  1000,   45.0);
        run(    0,  1000,   90.0);
        run(-1000,  1000,  135.0);
        run(-1000,     0,  180.0);
        run(-1000, -1000, -135.0);
        run(    0, -1000,  -90.0);
        run( 1000, -1000,  -45.0);
        run( 16000,  1000,  3.576);   // 地面静止 roll 约 3.6°
        run(     0,     0,   0.0);
        if (errs == 0) $display("TB_CORDIC PASS"); else $display("TB_CORDIC FAIL (%0d)", errs);
        $finish;
    end
endmodule
