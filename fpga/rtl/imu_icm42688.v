// ICM-42688-P SPI 读取：复位 -> 校验 WHO_AM_I(0x47) -> 开启加速度计+陀螺仪(低噪声) -> 每 2ms 突发读 12 字节
// 输出 din_valid 脉冲时 ax..gz 有效（默认量程 ±16g / ±2000dps，ODR 1kHz）
// 模块必须引出 CS/SCLK/SDI/SDO 才能用 SPI；只有 SDA/SCL 的是 I2C 模块，需换板或改 I2C
module imu_icm42688 #(
    parameter T_BOOT   = 5_000_000,   // 100ms @50MHz
    parameter T_RESET  =   500_000,   // 10ms
    parameter T_START  = 2_500_000,   // 50ms（陀螺仪启动）
    parameter T_PERIOD =   100_000,   // 2ms -> 500Hz
    parameter DLY      = 20
)(
    input                    clk,
    input                    rst_n,
    output                   spi_cs_n,
    output                   spi_sclk,
    output                   spi_mosi,
    input                    spi_miso,
    output reg signed [15:0] ax,
    output reg signed [15:0] ay,
    output reg signed [15:0] az,
    output reg signed [15:0] gx,
    output reg signed [15:0] gy,
    output reg signed [15:0] gz,
    output reg               din_valid,
    output reg               who_ok,
    output reg               ready
);
    // ---------------- SPI 字节引擎 ----------------
    reg        s_start;
    reg  [7:0] s_tx;
    wire [7:0] s_rx;
    wire       s_busy, s_done;

    spi_master u_spi (
        .clk(clk), .rst_n(rst_n), .start(s_start), .tx(s_tx), .rx(s_rx),
        .busy(s_busy), .done(s_done), .sclk(spi_sclk), .mosi(spi_mosi), .miso(spi_miso)
    );

    // ---------------- 事务引擎 ----------------
    localparam [2:0] E_IDLE = 3'd0, E_CSW = 3'd1, E_ADDR = 3'd2, E_DATA = 3'd3, E_END = 3'd4, E_GAP = 3'd5;

    reg [2:0]  eng;
    reg        cs_n;
    reg        go;
    reg        eng_done;
    reg  [6:0] t_addr;
    reg  [7:0] t_wdata;
    reg        t_read;
    reg  [4:0] t_len;
    reg  [4:0] bidx;
    reg  [7:0] dly;
    reg [95:0] rbuf;

    assign spi_cs_n = cs_n;

    always @(posedge clk) begin
        s_start  <= 1'b0;
        eng_done <= 1'b0;
        if (!rst_n) begin
            eng  <= E_IDLE;
            cs_n <= 1'b1;
            bidx <= 5'd0;
            dly  <= 8'd0;
            rbuf <= 96'd0;
            s_tx <= 8'd0;
        end else begin
            case (eng)
                E_IDLE: begin
                    cs_n <= 1'b1;
                    if (go) begin
                        cs_n <= 1'b0;
                        dly  <= DLY;
                        bidx <= 5'd0;
                        eng  <= E_CSW;
                    end
                end
                E_CSW: begin
                    if (dly == 8'd0) begin
                        s_tx    <= {t_read, t_addr};
                        s_start <= 1'b1;
                        eng     <= E_ADDR;
                    end else dly <= dly - 8'd1;
                end
                E_ADDR: begin
                    if (s_done) begin
                        s_tx    <= t_read ? 8'h00 : t_wdata;
                        s_start <= 1'b1;
                        eng     <= E_DATA;
                    end
                end
                E_DATA: begin
                    if (s_done) begin
                        if (t_read) rbuf <= {rbuf[87:0], s_rx};
                        bidx <= bidx + 5'd1;
                        if (bidx == t_len - 5'd1) begin
                            dly <= DLY;
                            eng <= E_END;
                        end else begin
                            s_tx    <= 8'h00;
                            s_start <= 1'b1;
                        end
                    end
                end
                E_END: begin
                    if (dly == 8'd0) begin
                        cs_n <= 1'b1;
                        dly  <= DLY;
                        eng  <= E_GAP;
                    end else dly <= dly - 8'd1;
                end
                default: begin  // E_GAP：CS 高电平保持一段时间
                    if (dly == 8'd0) begin
                        eng_done <= 1'b1;
                        eng      <= E_IDLE;
                    end else dly <= dly - 8'd1;
                end
            endcase
        end
    end

    // ---------------- 主时序 ----------------
    localparam [3:0] M_BOOT = 4'd0, M_SOFT = 4'd1, M_SOFTW = 4'd2, M_RSTW = 4'd3, M_WHO = 4'd4,
                     M_WHOW = 4'd5, M_PWR = 4'd6, M_PWRW = 4'd7, M_STW = 4'd8, M_WAIT = 4'd9,
                     M_BURST = 4'd10, M_BURSTW = 4'd11;

    reg [3:0]  m;
    reg [23:0] tmr;

    always @(posedge clk) begin
        go        <= 1'b0;
        din_valid <= 1'b0;
        if (!rst_n) begin
            m       <= M_BOOT;
            tmr     <= T_BOOT;
            who_ok  <= 1'b0;
            ready   <= 1'b0;
            t_addr  <= 7'd0;
            t_wdata <= 8'd0;
            t_read  <= 1'b0;
            t_len   <= 5'd1;
            ax <= 16'sd0; ay <= 16'sd0; az <= 16'sd0;
            gx <= 16'sd0; gy <= 16'sd0; gz <= 16'sd0;
        end else begin
            case (m)
                M_BOOT: if (tmr == 24'd0) m <= M_SOFT; else tmr <= tmr - 24'd1;
                M_SOFT: begin                       // DEVICE_CONFIG(0x11)=0x01 软复位
                    t_addr <= 7'h11; t_wdata <= 8'h01; t_read <= 1'b0; t_len <= 5'd1;
                    go <= 1'b1; m <= M_SOFTW;
                end
                M_SOFTW: if (eng_done) begin tmr <= T_RESET; m <= M_RSTW; end
                M_RSTW: if (tmr == 24'd0) m <= M_WHO; else tmr <= tmr - 24'd1;
                M_WHO: begin                        // WHO_AM_I(0x75)
                    t_addr <= 7'h75; t_read <= 1'b1; t_len <= 5'd1;
                    go <= 1'b1; m <= M_WHOW;
                end
                M_WHOW: if (eng_done) begin who_ok <= (rbuf[7:0] == 8'h47); m <= M_PWR; end
                M_PWR: begin                        // PWR_MGMT0(0x4E)=0x0F 加速度计+陀螺仪 低噪声模式
                    t_addr <= 7'h4E; t_wdata <= 8'h0F; t_read <= 1'b0; t_len <= 5'd1;
                    go <= 1'b1; m <= M_PWRW;
                end
                M_PWRW: if (eng_done) begin tmr <= T_START; m <= M_STW; end
                M_STW: if (tmr == 24'd0) begin ready <= 1'b1; tmr <= T_PERIOD; m <= M_WAIT; end
                       else tmr <= tmr - 24'd1;
                M_WAIT: if (tmr == 24'd0) m <= M_BURST; else tmr <= tmr - 24'd1;
                M_BURST: begin                      // 从 0x1F 起读：ACCEL_X/Y/Z, GYRO_X/Y/Z，各 2 字节大端
                    t_addr <= 7'h1F; t_read <= 1'b1; t_len <= 5'd12;
                    tmr <= T_PERIOD;
                    go <= 1'b1; m <= M_BURSTW;
                end
                default: begin                      // M_BURSTW
                    if (tmr != 24'd0) tmr <= tmr - 24'd1;
                    if (eng_done) begin
                        ax <= rbuf[95:80]; ay <= rbuf[79:64]; az <= rbuf[63:48];
                        gx <= rbuf[47:32]; gy <= rbuf[31:16]; gz <= rbuf[15:0];
                        din_valid <= 1'b1;
                        m <= M_WAIT;                // tmr 继续倒计时，保证 2ms 周期
                    end
                end
            endcase
        end
    end
endmodule
