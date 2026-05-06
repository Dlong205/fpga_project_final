// =============================================================================
// Module  : watchdog_top
// Board   : Kiwi 1P5 (Gowin GW1N-UV1P5), 27 MHz
// =============================================================================
module watchdog_top (
    input  logic clk,           // 27 MHz — pin 4

    // Buttons (active-low)
    input  logic btn_wdi_n,     // S1/KEY1 — pin 35
    input  logic btn_en_n,      // S2/KEY2 — pin 36

    // UART (USB-UART GWU2U)
    input  logic uart_rx,       // pin 33
    output logic uart_tx,       // pin 34

    // LEDs (active-high)
    output logic led_wdo,       // D3/LED1 — pin 27 — sáng khi FAULT
    output logic led_enout      // D4/LED2 — pin 28 — sáng khi armed
);

// ── Power-on reset (không cần nút reset ngoài) ────────────────────
logic [7:0] por_cnt = '0;
logic       rst_n;

always_ff @(posedge clk) begin
    if (!por_cnt[7]) por_cnt <= por_cnt + 1'b1;
end
assign rst_n = por_cnt[7];  // reset ~4.7µs sau khi bật nguồn

// ── Internal wires ─────────────────────────────────────────────────
logic        en_sw, wdi_src, clr_fault;
logic [31:0] tWD_ms, tRST_ms;
logic [15:0] arm_delay_us;
logic        wdo_n, enout, en_effective, fault_active;
logic        uart_kick, wdi_kick_out, last_kick_src;
logic        soft_rst;

logic [7:0]  reg_addr;
logic [31:0] reg_wdata, reg_rdata;
logic        reg_wen, reg_ren;

// ═══════════════════════════════════════════════════════════════════
// Instances
// ═══════════════════════════════════════════════════════════════════

watchdog_core #(.CLK_MHZ(27)) u_core (
    .clk          (clk),
    .rst_n        (rst_n),
    .btn_wdi_n    (btn_wdi_n),
    .btn_en_n     (btn_en_n),
    .uart_kick    (uart_kick),
    .uart_en      (en_sw),
    .wdi_src     (wdi_src),
    .tWD_ms       (tWD_ms),
    .tRST_ms      (tRST_ms),
    .arm_delay_us (arm_delay_us),
    .clr_fault    (clr_fault),
    .wdo_n        (wdo_n),
    .enout        (enout),
    .en_effective (en_effective),
    .fault_active (fault_active),
    .wdi_kick_out (wdi_kick_out),
    .last_kick_src(last_kick_src)
);

regfile u_reg (
    .clk          (clk),
    .rst_n        (rst_n),
    .addr         (reg_addr),
    .wdata        (reg_wdata),
    .wen          (reg_wen),
    .rdata        (reg_rdata),
    .ren          (reg_ren),
    .soft_rst     (soft_rst),
.en_sw        (en_sw),
    .wdi_src     (wdi_src),
    .clr_fault   (clr_fault),
    .tWD_ms       (tWD_ms),
    .tRST_ms      (tRST_ms),
    .arm_delay_us (arm_delay_us),
    .en_effective (en_effective),
    .fault_active (fault_active),
    .enout        (enout),
    .wdo_n        (wdo_n),
    .last_kick_src(last_kick_src)
);

uart_frame #(.CLK_MHZ(27), .BAUD(115200)) u_uart (
    .clk          (clk),
    .rst_n        (rst_n),
    .rx           (uart_rx),
    .tx           (uart_tx),
    .reg_addr     (reg_addr),
    .reg_wdata    (reg_wdata),
    .reg_wen      (reg_wen),
    .reg_rdata    (reg_rdata),
    .reg_ren      (reg_ren),
    .uart_kick    (uart_kick),
    .soft_rst     (soft_rst),
    .en_effective (en_effective),
    .fault_active (fault_active),
    .enout        (enout),
    .wdi_kick     (wdi_kick_out)
);

// ── LED ───────────────────────────────────────────────────────────
assign led_wdo   = ~wdo_n;  // D3 sáng = FAULT
assign led_enout = enout;   // D4 sáng = armed

endmodule
