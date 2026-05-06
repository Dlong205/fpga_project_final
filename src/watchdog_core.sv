// =============================================================================
// Module  : watchdog_core
// Desc    : FSM + Timers + Debounce — TPS3431-like watchdog
// FSM     : DISABLED → ARMING → RUNNING → FAULT → RUNNING
// Clock   : 27 MHz (Kiwi 1P5)
// =============================================================================
module watchdog_core #(
    parameter int CLK_MHZ = 27
)(
    input  logic        clk,
    input  logic        rst_n,

    // Raw button inputs (active-low)
    input  logic        btn_wdi_n,      // S1: WDI kick
    input  logic        btn_en_n,       // S2: Enable

    // UART kick + enable override
    input  logic        uart_kick,      // 1-cycle pulse từ UART CMD KICK
    input  logic        uart_en,        // EN_SW từ CTRL register
    input  logic        wdi_src,        // Mode: 0=HW+UART, 1=UART only (SW)

    // Config từ regfile
    input  logic [31:0] tWD_ms,
    input  logic [31:0] tRST_ms,
    input  logic [15:0] arm_delay_us,
    input  logic        clr_fault,

    // Outputs
    output logic        wdo_n,          // WDO active-low
    output logic        enout,          // ENOUT
    output logic        en_effective,   // STATUS[0]
    output logic        fault_active,   // STATUS[1]
    output logic        wdi_kick_out,   // kick pulse (cho debug log)
    output logic        last_kick_src   // 0=button, 1=UART
);

// ── Tick generators ────────────────────────────────────────────────
localparam int US_DIV = CLK_MHZ;            // 27 cycles = 1µs
localparam int MS_DIV = CLK_MHZ * 1000;     // 27000 cycles = 1ms

logic [4:0]  us_cnt;
logic [14:0] ms_cnt;
logic        tick_us, tick_ms;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        us_cnt  <= '0; ms_cnt  <= '0;
        tick_us <= '0; tick_ms <= '0;
    end else begin
        tick_us <= (us_cnt == 5'(US_DIV - 2));
        us_cnt  <= (us_cnt == 5'(US_DIV - 1)) ? '0 : us_cnt + 1'b1;

        tick_ms <= (ms_cnt == 15'(MS_DIV - 2));
        ms_cnt  <= (ms_cnt == 15'(MS_DIV - 1)) ? '0 : ms_cnt + 1'b1;
    end
end

// ── Debounce + sync (2-FF + counter) ──────────────────────────────
// Generic debounce: 10ms @ 27MHz = 270_000 cycles
localparam int DEB_CYC = 10 * CLK_MHZ * 1000;
localparam int DEB_W   = $clog2(DEB_CYC + 1);

// WDI button
logic s1_ff1, s1_ff2, s1_stable;
logic [DEB_W-1:0] s1_cnt;
logic s1_out, s1_prev, wdi_btn_kick;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        {s1_ff2,s1_ff1} <= 2'b11;
        s1_stable <= 1'b1; s1_cnt <= '0;
        s1_out <= 1'b0; s1_prev <= 1'b0;
    end else begin
        {s1_ff2,s1_ff1} <= {s1_ff1, btn_wdi_n};
        // debounce
        if (s1_ff2 == s1_stable) s1_cnt <= '0;
        else if (s1_cnt == DEB_W'(DEB_CYC-1)) begin
            s1_stable <= s1_ff2; s1_cnt <= '0;
        end else s1_cnt <= s1_cnt + 1'b1;
        // edge detect
        s1_prev <= s1_out;
        s1_out  <= ~s1_stable;  // active-high
    end
end
assign wdi_btn_kick = s1_out & ~s1_prev;   // rising edge = button pressed

// EN button - toggle: press = OFF, release = ON
logic s2_ff1, s2_ff2, s2_stable, s2_prev_stable;
logic [DEB_W-1:0] s2_cnt;
logic en_btn;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        {s2_ff2,s2_ff1} <= 2'b11;
        s2_stable <= 1'b1; s2_cnt <= '0;
    end else begin
        {s2_ff2,s2_ff1} <= {s2_ff1, btn_en_n};
        if (s2_ff2 == s2_stable) s2_cnt <= '0;
        else if (s2_cnt == DEB_W'(DEB_CYC-1)) begin
            s2_stable <= s2_ff2; s2_cnt <= '0;
        end else s2_cnt <= s2_cnt + 1'b1;
    end
end

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        s2_prev_stable <= 1'b1;
    end else begin
        s2_prev_stable <= s2_stable;
    end
end

// Press = OFF (EN=1), Release = ON (EN=0)
assign en_btn = ~s2_stable;

// Combined signals
// wdi_src: 0 = HW+UART (default), 1 = UART only (SW only)
logic en_combined, kick_combined;
assign en_combined   = ~(en_btn | uart_en);
assign kick_combined = wdi_src ? uart_kick : (wdi_btn_kick | uart_kick);
assign wdi_kick_out  = kick_combined;

// Last kick source
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)          last_kick_src <= 1'b0;
    else if (uart_kick)  last_kick_src <= 1'b1;
    else if (wdi_btn_kick) last_kick_src <= 1'b0;
end

// ── FSM ───────────────────────────────────────────────────────────
typedef enum logic [1:0] {
    ST_DISABLED = 2'b00,
    ST_ARMING   = 2'b01,
    ST_RUNNING  = 2'b10,
    ST_FAULT    = 2'b11
} fsm_t;

fsm_t state;

logic [15:0] arm_cnt;
logic [31:0] twd_cnt, trst_cnt;
logic        arm_done, twd_exp, trst_done;

assign arm_done  = (arm_cnt  >= arm_delay_us);
assign twd_exp   = (twd_cnt  >= tWD_ms);
assign trst_done = (trst_cnt >= tRST_ms);

// EN edge detect
logic en_prev2;
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) en_prev2 <= 1'b0;
    else        en_prev2 <= en_combined;
end

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state    <= ST_DISABLED;
        arm_cnt  <= '0;
        twd_cnt  <= '0;
        trst_cnt <= '0;
    end else begin
        case (state)
            ST_DISABLED: begin
                arm_cnt <= '0; twd_cnt <= '0; trst_cnt <= '0;
                if (en_combined) state <= ST_ARMING;
            end

            ST_ARMING: begin
                twd_cnt <= '0; trst_cnt <= '0;
                if (!en_combined) begin
                    state <= ST_DISABLED; arm_cnt <= '0;
                end else if (arm_done) begin
                    state <= ST_RUNNING; arm_cnt <= '0;
                end else if (tick_us) begin
                    arm_cnt <= arm_cnt + 1'b1;
                end
            end

            ST_RUNNING: begin
                arm_cnt <= '0; trst_cnt <= '0;
                if (!en_combined) begin
                    state <= ST_DISABLED; twd_cnt <= '0;
                end else if (twd_exp) begin
                    state <= ST_FAULT; twd_cnt <= '0;
                end else if (kick_combined) begin
                    twd_cnt <= '0;          // kick reset
                end else if (tick_ms) begin
                    twd_cnt <= twd_cnt + 1'b1;
                end
            end

            ST_FAULT: begin
                arm_cnt <= '0; twd_cnt <= '0;
                if (!en_combined) begin
                    state <= ST_DISABLED; trst_cnt <= '0;
                end else if (clr_fault || trst_done) begin
                    state <= ST_RUNNING; trst_cnt <= '0;
                end else if (tick_ms) begin
                    trst_cnt <= trst_cnt + 1'b1;
                end
            end
        endcase
    end
end

// ── Outputs ───────────────────────────────────────────────────────
always_comb begin
    wdo_n        = 1'b1;
    enout        = 1'b0;
    en_effective = 1'b0;
    fault_active = 1'b0;

    case (state)
        ST_DISABLED: begin wdo_n=1; enout=0; en_effective=0; fault_active=0; end
        ST_ARMING:   begin wdo_n=1; enout=0; en_effective=1; fault_active=0; end
        ST_RUNNING:  begin wdo_n=1; enout=1; en_effective=1; fault_active=0; end
        ST_FAULT:    begin wdo_n=0; enout=1; en_effective=1; fault_active=1; end
        default:     begin wdo_n=1; enout=0; en_effective=0; fault_active=0; end
    endcase
end

endmodule