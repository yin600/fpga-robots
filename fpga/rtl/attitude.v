// 姿态解算：陀螺积分 + 加速度计互补滤波（横滚/俯仰），偏航只靠陀螺积分
// 单位 Q16 度（65536 = 1°）。陀螺量程 ±2000dps 时 16.384 LSB/dps，500Hz 采样 ->  每 LSB 对应 8/65536 度（刚好 *8）
// 上电后前 512 个样本（约 1 秒）求陀螺零偏，期间车必须静止；cal_done=1 之后才开始积分。
// GXS/GYS/GZS = +1/-1：装反方向时改这里；ASEL 见下
module attitude #(
    parameter GXS   = 1,
    parameter GYS   = 1,
    parameter GZS   = 1,
    parameter DEAD  = 2,      // 陀螺死区（LSB），抑制静止漂移
    parameter ALPHA = 8       // 互补滤波系数 1/2^ALPHA：8 -> 时间常数约 0.5s
)(
    input                    clk,
    input                    rst_n,
    input                    din_valid,
    input  signed [15:0]     ax,
    input  signed [15:0]     ay,
    input  signed [15:0]     az,
    input  signed [15:0]     gx,
    input  signed [15:0]     gy,
    input  signed [15:0]     gz,
    input                    yaw_zero,
    output reg signed [31:0] roll_q16,
    output reg signed [31:0] pitch_q16,
    output reg signed [31:0] yaw_q16,
    output reg signed [15:0] gz_c,
    output reg               cal_done,
    output reg               upd
);
    // ---- 零偏标定 ----
    reg [9:0]         ncal;
    reg signed [31:0] sgx, sgy, sgz;
    reg signed [15:0] bgx, bgy, bgz;

    // ---- 锁存的样本 ----
    reg signed [15:0] lax, lay, laz;

    // ---- CORDIC ----
    reg               c_start;
    reg signed [15:0] c_x, c_y;
    wire              c_done;
    wire signed [31:0] c_ang;

    cordic_atan2 u_cordic (
        .clk(clk), .rst_n(rst_n), .start(c_start), .x(c_x), .y(c_y),
        .busy(), .done(c_done), .angle(c_ang)
    );

    function signed [15:0] dz;      // 死区
        input signed [15:0] v;
        begin
            if (v > DEAD || v < -DEAD) dz = v;
            else                       dz = 16'sd0;
        end
    endfunction

    wire signed [15:0] ngx = dz(gx - bgx);
    wire signed [15:0] ngy = dz(gy - bgy);
    wire signed [15:0] ngz = dz(gz - bgz);

    localparam [2:0] A_IDLE = 3'd0, A_R = 3'd1, A_RW = 3'd2, A_P = 3'd3, A_PW = 3'd4, A_UPD = 3'd5;
    reg [2:0] a;
    reg signed [31:0] ra;

    always @(posedge clk) begin
        c_start <= 1'b0;
        upd     <= 1'b0;
        if (!rst_n) begin
            a <= A_IDLE;
            ncal <= 10'd0;
            sgx <= 32'sd0; sgy <= 32'sd0; sgz <= 32'sd0;
            bgx <= 16'sd0; bgy <= 16'sd0; bgz <= 16'sd0;
            cal_done <= 1'b0;
            roll_q16 <= 32'sd0; pitch_q16 <= 32'sd0; yaw_q16 <= 32'sd0;
            gz_c <= 16'sd0;
            lax <= 16'sd0; lay <= 16'sd0; laz <= 16'sd0;
            c_x <= 16'sd0; c_y <= 16'sd0;
            ra  <= 32'sd0;
        end else begin
            if (yaw_zero) yaw_q16 <= 32'sd0;
            case (a)
                A_IDLE: if (din_valid) begin
                    lax <= ax; lay <= ay; laz <= az;
                    if (!cal_done) begin
                        sgx <= sgx + gx; sgy <= sgy + gy; sgz <= sgz + gz;
                        ncal <= ncal + 10'd1;
                        if (ncal == 10'd511) begin
                            bgx <= (sgx + gx) >>> 9;
                            bgy <= (sgy + gy) >>> 9;
                            bgz <= (sgz + gz) >>> 9;
                            cal_done <= 1'b1;
                        end
                    end else begin
                        gz_c <= (GZS > 0) ? ngz : -ngz;
                        yaw_q16   <= yaw_q16   + ((GZS > 0) ? (ngz <<< 3) : -(ngz <<< 3));
                        roll_q16  <= roll_q16  + ((GXS > 0) ? (ngx <<< 3) : -(ngx <<< 3));
                        pitch_q16 <= pitch_q16 + ((GYS > 0) ? (ngy <<< 3) : -(ngy <<< 3));
                    end
                    a <= A_R;
                end
                A_R: begin                       // roll_acc = atan2(ay, az)
                    c_x <= laz; c_y <= lay; c_start <= 1'b1; a <= A_RW;
                end
                A_RW: if (c_done) begin
                    ra <= c_ang;
                    a  <= A_P;
                end
                A_P: begin                       // pitch_acc = atan2(-ax, az)
                    c_x <= laz; c_y <= -lax; c_start <= 1'b1; a <= A_PW;
                end
                A_PW: if (c_done) begin
                    if (!cal_done) begin         // 标定期间直接用加速度计角度做初值
                        roll_q16  <= ra;
                        pitch_q16 <= c_ang;
                    end else begin
                        roll_q16  <= roll_q16  + ((ra    - roll_q16 ) >>> ALPHA);
                        pitch_q16 <= pitch_q16 + ((c_ang - pitch_q16) >>> ALPHA);
                    end
                    upd <= 1'b1;
                    a   <= A_IDLE;
                end
                default: a <= A_IDLE;
            endcase
        end
    end
endmodule
