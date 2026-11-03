// 顶层：任务1~4 全部在这一个文件里连起来
//   电机编号：0=左前 1=左后 2=右前 3=右后（左侧两个电机用同一指令，右侧同理）
//   编码器：左右各取 1 路做速度反馈（INVERT 参数改计数方向，MOT_INV 参数改电机转向）
//   急停：estop_ok=1 才允许输出；用常闭触点串 3.3V + 下拉电阻，断线/按下都会变 0 -> 立即断电机
module robot_top #(
    parameter CLK_HZ    = 50_000_000,
    parameter PERIOD    = 2500,          // 50MHz/2500 = 20kHz PWM
    parameter ENC_INV_L = 0,
    parameter ENC_INV_R = 0,
    parameter MOT_INV_L = 0,
    parameter MOT_INV_R = 1,             // 右侧电机镜像安装，通常需要反向；上板后按实际转向改
    parameter WD_CYC    = 100_000_000    // 2 秒没收到上位机帧，手动模式自动停车
)(
    input         clk50,
    input         rst_key_n,
    input         estop_ok,
    input         btn_start_n,

    input         uart_rx,
    output        uart_tx,

    output [3:0]  m_pwm,
    output [3:0]  m_in1,
    output [3:0]  m_in2,
    output [1:0]  m_stby,

    input         enc_l_a,
    input         enc_l_b,
    input         enc_r_a,
    input         enc_r_b,

    output [2:0]  us_trig,
    input  [2:0]  us_echo,

    output        imu_cs_n,
    output        imu_sclk,
    output        imu_mosi,
    input         imu_miso,

    output [3:0]  led
);
    wire clk = clk50;

    // ---------------- 复位 ----------------
    reg [15:0] por = 16'd0;
    reg [1:0]  key_s = 2'b00;
    always @(posedge clk) begin
        key_s <= {key_s[0], rst_key_n};
        if (por != 16'hFFFF) por <= por + 16'd1;
    end
    wire rst_n = (por == 16'hFFFF) && key_s[1];

    // ---------------- 串口寄存器 ----------------
    wire [2:0]         mode_r;
    wire signed [15:0] v_set, w_set, rot_deg;
    wire               p_yaw_zero, p_start, p_fclr, wd_dis, frame_ok;
    wire signed [15:0] kp_spd, ki_spd, kf_spd, w_avoid, kp_head, wmax, wmin, tol_deg;
    wire [15:0]        stop_mm, slow_mm;
    wire [3:0]         n_legs;
    wire [127:0]       turn_bus;
    wire [255:0]       dist_bus;

    wire [15:0] s_roll, s_pitch, s_yaw, s_status;
    wire [15:0] dl, dc, dr;
    wire signed [15:0] spd_l, spd_r;
    wire signed [31:0] pos_l, pos_r;
    wire signed [31:0] pos_avg = (pos_l + pos_r) >>> 1;

    uart_regs #(.CLK_HZ(CLK_HZ)) u_regs (
        .clk(clk), .rst_n(rst_n), .rx(uart_rx), .tx(uart_tx),
        .mode(mode_r), .v_set(v_set), .w_set(w_set), .rot_deg(rot_deg),
        .p_yaw_zero(p_yaw_zero), .p_start(p_start), .p_fclr(p_fclr), .wd_dis(wd_dis),
        .kp_spd(kp_spd), .ki_spd(ki_spd), .kf_spd(kf_spd),
        .stop_mm(stop_mm), .slow_mm(slow_mm), .n_legs(n_legs), .w_avoid(w_avoid),
        .kp_head(kp_head), .wmax(wmax), .wmin(wmin), .tol_deg(tol_deg),
        .turn_bus(turn_bus), .dist_bus(dist_bus), .frame_ok(frame_ok),
        .s_roll(s_roll), .s_pitch(s_pitch), .s_yaw(s_yaw),
        .s_dl(dl), .s_dc(dc), .s_dr(dr),
        .s_spd_l(spd_l), .s_spd_r(spd_r), .s_status(s_status), .s_pos(pos_avg)
    );

    // ---------------- IMU + 姿态（任务1） ----------------
    wire signed [15:0] ax, ay, az, gx, gy, gz;
    wire               imu_valid, who_ok, imu_ready;
    wire signed [31:0] roll_q16, pitch_q16, yaw_q16;
    wire signed [15:0] gz_c;
    wire               cal_done, att_upd;

    imu_icm42688 u_imu (
        .clk(clk), .rst_n(rst_n),
        .spi_cs_n(imu_cs_n), .spi_sclk(imu_sclk), .spi_mosi(imu_mosi), .spi_miso(imu_miso),
        .ax(ax), .ay(ay), .az(az), .gx(gx), .gy(gy), .gz(gz),
        .din_valid(imu_valid), .who_ok(who_ok), .ready(imu_ready)
    );

    attitude u_att (
        .clk(clk), .rst_n(rst_n), .din_valid(imu_valid),
        .ax(ax), .ay(ay), .az(az), .gx(gx), .gy(gy), .gz(gz),
        .yaw_zero(p_yaw_zero),
        .roll_q16(roll_q16), .pitch_q16(pitch_q16), .yaw_q16(yaw_q16),
        .gz_c(gz_c), .cal_done(cal_done), .upd(att_upd)
    );

    assign s_roll  = roll_q16[24:9];     // Q16 度 >>9 -> Q7 度
    assign s_pitch = pitch_q16[24:9];
    assign s_yaw   = yaw_q16[24:9];

    // ---------------- 超声波（任务3） ----------------
    wire [2:0] us_fault;
    wire       us_upd;
    ultrasonic_scan #(.CLK_HZ(CLK_HZ)) u_us (
        .clk(clk), .rst_n(rst_n), .echo(us_echo), .trig(us_trig),
        .d0(dl), .d1(dc), .d2(dr), .fault(us_fault), .updated(us_upd)
    );

    wire [8:0]         scale;
    wire signed [15:0] avoid_w;
    wire               avoid_active, blocked;
    speed_limit u_lim (
        .clk(clk), .rst_n(rst_n), .dl(dl), .dc(dc), .dr(dr),
        .stop_mm(stop_mm), .slow_mm(slow_mm), .w_avoid(w_avoid),
        .scale(scale), .avoid_w(avoid_w), .avoid_active(avoid_active), .blocked(blocked)
    );

    // ---------------- 编码器 ----------------
    wire tick10, tick_r;
    enc_if #(.CLK_HZ(CLK_HZ), .INVERT(ENC_INV_L)) u_enc_l (
        .clk(clk), .rst_n(rst_n), .a_in(enc_l_a), .b_in(enc_l_b),
        .pos(pos_l), .speed(spd_l), .tick10(tick10)
    );
    enc_if #(.CLK_HZ(CLK_HZ), .INVERT(ENC_INV_R)) u_enc_r (
        .clk(clk), .rst_n(rst_n), .a_in(enc_r_a), .b_in(enc_r_b),
        .pos(pos_r), .speed(spd_r), .tick10(tick_r)
    );

    // ---------------- 启动按键（独立运行任务4用） ----------------
    reg [1:0]  bs;
    reg        b_stable, b_prev;
    reg [19:0] bcnt;
    reg        btn_run;
    wire       btn_edge = b_stable && !b_prev;      // 按下（低电平有效）的上升沿
    always @(posedge clk) begin
        bs     <= {bs[0], ~btn_start_n};
        b_prev <= b_stable;
        if (!rst_n) begin
            bcnt <= 20'd0; b_stable <= 1'b0; btn_run <= 1'b0;
        end else begin
            if (bs[1] != b_stable) begin
                if (bcnt == 20'd1_000_000) begin b_stable <= bs[1]; bcnt <= 20'd0; end
                else bcnt <= bcnt + 20'd1;
            end else bcnt <= 20'd0;
            if (btn_edge && estop_ok) btn_run <= 1'b1;
            if (!estop_ok || p_fclr)  btn_run <= 1'b0;
        end
    end

    // ---------------- 看门狗：手动模式(1,2)下串口失联自动停车 ----------------
    reg [26:0] wd_cnt;
    reg        wd_trip;
    always @(posedge clk) begin
        if (!rst_n || frame_ok || wd_dis) begin
            wd_cnt  <= 27'd0;
            wd_trip <= 1'b0;
        end else if (wd_cnt >= WD_CYC) wd_trip <= 1'b1;
        else wd_cnt <= wd_cnt + 27'd1;
    end

    wire [2:0] mode_wd  = ((mode_r == 3'd1 || mode_r == 3'd2) && wd_trip) ? 3'd0 : mode_r;
    wire [2:0] mode_eff = !estop_ok ? 3'd0 : (btn_run ? 3'd4 : mode_wd);
    // 按键启动：等 mode 切到 4 之后再发 start（延迟 4 个时钟），否则 nav_ctrl 会把它当成模式切换吞掉
    reg [3:0] be_d;
    always @(posedge clk) be_d <= {be_d[2:0], btn_edge & estop_ok};
    wire       start_p  = p_start | be_d[3];

    // ---------------- 运动/导航 ----------------
    wire signed [15:0] v_out, w_out;
    wire               brake, nav_done, nav_fault, at_target;
    wire [2:0]         leg_idx;

    nav_ctrl u_nav (
        .clk(clk), .rst_n(rst_n), .tick(tick10),
        .mode(mode_eff), .v_set(v_set), .w_set(w_set), .rot_deg(rot_deg),
        .mission_start(start_p), .fault_clr(p_fclr),
        .yaw_q16(yaw_q16), .pos_avg(pos_avg),
        .scale(scale), .avoid_w(avoid_w), .avoid_active(avoid_active),
        .kp_head(kp_head), .wmax(wmax), .wmin(wmin), .tol_deg(tol_deg),
        .n_legs(n_legs), .turn_bus(turn_bus), .dist_bus(dist_bus),
        .v_out(v_out), .w_out(w_out), .brake(brake),
        .leg_idx(leg_idx), .done(nav_done), .fault(nav_fault), .at_target(at_target)
    );

    // ---------------- 左右轮速度环 ----------------
    wire signed [15:0] tgt_l = v_out - w_out;
    wire signed [15:0] tgt_r = v_out + w_out;
    wire signed [12:0] cmd_l, cmd_r;
    wire               run_en = estop_ok && (mode_eff != 3'd0) && !brake;

    pi_speed #(.PERIOD(PERIOD)) u_pi_l (
        .clk(clk), .rst_n(rst_n), .tick(tick10), .en(run_en),
        .target(tgt_l), .meas(spd_l), .kf(kf_spd), .kp(kp_spd), .ki(ki_spd), .cmd(cmd_l)
    );
    pi_speed #(.PERIOD(PERIOD)) u_pi_r (
        .clk(clk), .rst_n(rst_n), .tick(tick10), .en(run_en),
        .target(tgt_r), .meas(spd_r), .kf(kf_spd), .kp(kp_spd), .ki(ki_spd), .cmd(cmd_r)
    );

    wire signed [12:0] cl = (MOT_INV_L != 0) ? -cmd_l : cmd_l;
    wire signed [12:0] cr = (MOT_INV_R != 0) ? -cmd_r : cmd_r;

    // 刹车只在 brake=1 且急停正常时有效（急停时 en=0，TB6612 进入待机，电机自由）
    genvar gi;
    generate
        for (gi = 0; gi < 4; gi = gi + 1) begin : g_mot
            motor_drv #(.PERIOD(PERIOD)) u_m (
                .clk(clk), .rst_n(rst_n),
                .cmd((gi < 2) ? cl : cr),
                .en(estop_ok), .brake(brake),
                .pwm(m_pwm[gi]), .in1(m_in1[gi]), .in2(m_in2[gi]), .stby()
            );
        end
    endgenerate
    assign m_stby = {2{estop_ok}};

    // ---------------- 状态字与指示灯 ----------------
    assign s_status = {wd_trip, us_fault, imu_ready, avoid_active & 1'b1, blocked, at_target,
                       estop_ok, leg_idx, nav_fault, nav_done, cal_done, who_ok};

    reg [24:0] hb;
    always @(posedge clk) hb <= hb + 25'd1;
    assign led = {estop_ok, who_ok, cal_done, hb[24]};
endmodule
