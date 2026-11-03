// 运动/导航控制：把 mode + 传感器 变成左右轮的 (v, w) 指令
//   mode 0 停车(刹车)      mode 1 手动 v/w(开环给速度环)
//   mode 2 直行+航向保持+避障(任务3)   mode 3 原地转 rot_deg 度(任务2)
//   mode 4 航点任务(任务4)：每条腿 = 先转 turn_i 度，再直行 dist_i 个编码器计数；最多 8 条
// mission_start 脉冲：mode 2/3 重新锁定目标航向；mode 4 启动任务
// w>0 = 向左转（yaw 增大）。上层：tgt_l = v - w，tgt_r = v + w
module nav_ctrl (
    input                     clk,
    input                     rst_n,
    input                     tick,
    input      [2:0]          mode,
    input  signed [15:0]      v_set,
    input  signed [15:0]      w_set,
    input  signed [15:0]      rot_deg,
    input                     mission_start,
    input                     fault_clr,
    input  signed [31:0]      yaw_q16,
    input  signed [31:0]      pos_avg,
    input      [8:0]          scale,
    input  signed [15:0]      avoid_w,
    input                     avoid_active,
    input  signed [15:0]      kp_head,
    input  signed [15:0]      wmax,
    input  signed [15:0]      wmin,
    input  signed [15:0]      tol_deg,
    input      [3:0]          n_legs,
    input      [127:0]        turn_bus,    // 8 x s16
    input      [255:0]        dist_bus,    // 8 x u32（编码器计数）
    output reg signed [15:0]  v_out,
    output reg signed [15:0]  w_out,
    output reg                brake,
    output reg [2:0]          leg_idx,
    output reg                done,
    output reg                fault,
    output                    at_target
);
    localparam [2:0] ST_IDLE = 3'd0, ST_TURN = 3'd1, ST_DRIVE = 3'd2, ST_NEXT = 3'd3, ST_DONE = 3'd4;

    reg [2:0]         st;
    reg [2:0]         prev_mode;
    reg signed [31:0] tgt;
    reg signed [31:0] pos0;
    reg [15:0]        tmr;

    wire signed [15:0] turn_i = turn_bus[leg_idx * 16 +: 16];
    wire signed [15:0] turn_n = turn_bus[(leg_idx + 3'd1) * 16 +: 16];
    wire [31:0]        dist_i = dist_bus[leg_idx * 32 +: 32];

    wire signed [31:0] travelled = pos_avg - pos0;

    wire h_en = (mode == 3'd2) || (mode == 3'd3) ||
                (mode == 3'd4 && (st == ST_TURN || st == ST_DRIVE));
    wire signed [15:0] w_h;

    heading_ctrl u_head (
        .clk(clk), .rst_n(rst_n), .tick(tick), .en(h_en),
        .yaw_q16(yaw_q16), .tgt_q16(tgt),
        .kp(kp_head), .wmax(wmax), .wmin(wmin), .tol_deg(tol_deg),
        .w(w_h), .at_target(at_target)
    );

    wire signed [25:0] vm = v_set * $signed({1'b0, scale});
    wire signed [15:0] v_scaled = (v_set > 0) ? vm[23:8] : v_set;

    // ---------------- 状态机 ----------------
    always @(posedge clk) begin
        prev_mode <= mode;
        if (!rst_n) begin
            st        <= ST_IDLE;
            prev_mode <= 3'd0;
            tgt       <= 32'sd0;
            pos0      <= 32'sd0;
            tmr       <= 16'd0;
            leg_idx   <= 3'd0;
            done      <= 1'b0;
            fault     <= 1'b0;
        end else begin
            if (fault_clr) fault <= 1'b0;
            if (mode != prev_mode) begin
                st   <= ST_IDLE;
                done <= 1'b0;
                tmr  <= 16'd0;
                if (mode == 3'd2)      tgt <= yaw_q16;
                else if (mode == 3'd3) tgt <= yaw_q16 + {rot_deg, 16'd0};
            end else if (mission_start && mode == 3'd2) begin
                tgt <= yaw_q16;
            end else if (mission_start && mode == 3'd3) begin
                tgt <= yaw_q16 + {rot_deg, 16'd0};
            end else if (mission_start && mode == 3'd4) begin
                leg_idx <= 3'd0;
                done    <= 1'b0;
                fault   <= 1'b0;
                tmr     <= 16'd0;
                if (n_legs == 4'd0) begin
                    st   <= ST_DONE;
                    done <= 1'b1;
                end else begin
                    tgt <= yaw_q16 + {turn_bus[15:0], 16'd0};
                    st  <= ST_TURN;
                end
            end else if (tick && mode == 3'd4) begin
                case (st)
                    ST_TURN: begin
                        tmr <= tmr + 16'd1;
                        if ((tmr >= 16'd3 && at_target) || tmr >= 16'd800) begin
                            if (tmr >= 16'd800) fault <= 1'b1;
                            pos0 <= pos_avg;
                            tmr  <= 16'd0;
                            st   <= ST_DRIVE;
                        end
                    end
                    ST_DRIVE: begin
                        tmr <= tmr + 16'd1;
                        if (travelled >= $signed(dist_i) || tmr >= 16'd4000) begin
                            if (tmr >= 16'd4000) fault <= 1'b1;
                            tmr <= 16'd0;
                            st  <= ST_NEXT;
                        end
                    end
                    ST_NEXT: begin
                        if ({1'b0, leg_idx} + 4'd1 >= n_legs) begin
                            st   <= ST_DONE;
                            done <= 1'b1;
                        end else begin
                            leg_idx <= leg_idx + 3'd1;
                            tgt     <= tgt + {turn_n, 16'd0};
                            tmr     <= 16'd0;
                            st      <= ST_TURN;
                        end
                    end
                    default: ;
                endcase
            end
        end
    end

    // ---------------- 输出选择 ----------------
    always @(posedge clk) begin
        if (!rst_n) begin
            v_out <= 16'sd0;
            w_out <= 16'sd0;
            brake <= 1'b1;
        end else begin
            case (mode)
                3'd1: begin
                    v_out <= v_set;
                    w_out <= w_set;
                    brake <= 1'b0;
                end
                3'd2: begin
                    v_out <= v_scaled;
                    w_out <= avoid_active ? avoid_w : w_h;
                    brake <= 1'b0;
                end
                3'd3: begin
                    v_out <= 16'sd0;
                    w_out <= w_h;
                    brake <= 1'b0;
                end
                3'd4: begin
                    case (st)
                        ST_TURN: begin
                            v_out <= 16'sd0;
                            w_out <= w_h;
                            brake <= 1'b0;
                        end
                        ST_DRIVE: begin
                            v_out <= v_scaled;
                            w_out <= avoid_active ? avoid_w : w_h;
                            brake <= 1'b0;
                        end
                        default: begin
                            v_out <= 16'sd0;
                            w_out <= 16'sd0;
                            brake <= 1'b1;
                        end
                    endcase
                end
                default: begin
                    v_out <= 16'sd0;
                    w_out <= 16'sd0;
                    brake <= 1'b1;
                end
            endcase
        end
    end
endmodule
