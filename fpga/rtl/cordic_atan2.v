// CORDIC 矢量模式 atan2：输入 x,y(16 位有符号)，输出角度 Q16 度（65536 = 1°），范围 -180°~+180°
// 迭代 16 次，约 19 个时钟出结果；最坏误差约 1°，典型 <0.1°。(0,0) 输出 0。
module cordic_atan2 (
    input                    clk,
    input                    rst_n,
    input                    start,
    input  signed [15:0]     x,
    input  signed [15:0]     y,
    output reg               busy,
    output reg               done,
    output reg signed [31:0] angle
);
    reg signed [19:0] xr, yr;
    reg signed [31:0] zr, off;
    reg [4:0] i;

    function signed [31:0] atan_tab;
        input [4:0] k;
        begin
            case (k)
            5'd0: atan_tab = 32'sd2949120;
            5'd1: atan_tab = 32'sd1740967;
            5'd2: atan_tab = 32'sd919879;
            5'd3: atan_tab = 32'sd466945;
            5'd4: atan_tab = 32'sd234379;
            5'd5: atan_tab = 32'sd117304;
            5'd6: atan_tab = 32'sd58666;
            5'd7: atan_tab = 32'sd29335;
            5'd8: atan_tab = 32'sd14668;
            5'd9: atan_tab = 32'sd7334;
            5'd10: atan_tab = 32'sd3667;
            5'd11: atan_tab = 32'sd1833;
            5'd12: atan_tab = 32'sd917;
            5'd13: atan_tab = 32'sd458;
            5'd14: atan_tab = 32'sd229;
            5'd15: atan_tab = 32'sd115;
                default: atan_tab = 32'sd0;
            endcase
        end
    endfunction

    wire signed [19:0] xs = xr >>> i;
    wire signed [19:0] ys = yr >>> i;

    always @(posedge clk) begin
        done <= 1'b0;
        if (!rst_n) begin
            busy  <= 1'b0;
            angle <= 32'sd0;
            xr <= 20'sd0; yr <= 20'sd0; zr <= 32'sd0; off <= 32'sd0; i <= 5'd0;
        end else if (!busy) begin
            if (start) begin
                if (x == 16'sd0 && y == 16'sd0) begin
                    angle <= 32'sd0;
                    done  <= 1'b1;
                end else begin
                    if (x[15]) begin                      // 第二、三象限：先翻到右半平面
                        xr  <= -{{4{x[15]}}, x};
                        yr  <= -{{4{y[15]}}, y};
                        off <= y[15] ? -32'sd11796480 : 32'sd11796480;
                    end else begin
                        xr  <= {{4{x[15]}}, x};
                        yr  <= {{4{y[15]}}, y};
                        off <= 32'sd0;
                    end
                    zr   <= 32'sd0;
                    i    <= 5'd0;
                    busy <= 1'b1;
                end
            end
        end else begin
            if (i == 5'd16) begin
                angle <= zr + off;
                busy  <= 1'b0;
                done  <= 1'b1;
            end else begin
                if (!yr[19]) begin
                    xr <= xr + ys;
                    yr <= yr - xs;
                    zr <= zr + atan_tab(i);
                end else begin
                    xr <= xr - ys;
                    yr <= yr + xs;
                    zr <= zr - atan_tab(i);
                end
                i <= i + 5'd1;
            end
        end
    end
endmodule
