// =============================================================================
// Module  : regfile
// Desc    : Configuration registers + STATUS
// Addr map (word index = byte_addr >> 2):
//   0x00 CTRL        [bit0=EN_SW, bit1=WDI_SRC(0=HW+UART,1=UART only), bit2=CLR_FAULT]
//   0x04 tWD_ms      default 1600
//   0x08 tRST_ms     default 200
//   0x0C arm_delay_us default 150
//   0x10 STATUS      [bit0=EN_EFF, bit1=FAULT, bit2=ENOUT, bit3=WDO, bit4=KICK_SRC]
// =============================================================================
module regfile (
    input  logic        clk,
    input  logic        rst_n,

    // Write/Read interface từ uart_frame
    input  logic [7:0]  addr,
    input  logic [31:0] wdata,
    input  logic        wen,
    output logic [31:0] rdata,
    input  logic        ren,

    // Soft reset từ UART
    input  logic        soft_rst,

    // Outputs → watchdog_core
    output logic        en_sw,
    output logic        wdi_src,      // bit 1 of CTRL: 0=HW+UART, 1=UART only (SW)
    output logic        clr_fault,
    output logic [31:0] tWD_ms,
    output logic [31:0] tRST_ms,
    output logic [15:0] arm_delay_us,

    // Status inputs ← watchdog_core
    input  logic        en_effective,
    input  logic        fault_active,
    input  logic        enout,
    input  logic        wdo_n,
    input  logic        last_kick_src
);

logic [31:0] ctrl_reg;
logic [31:0] twd_reg;
logic [31:0] trst_reg;
logic [15:0] arm_reg; 

// ── Write ──────────────────────────────────────────────────────────
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n || soft_rst) begin
        ctrl_reg <= 32'h0;
        twd_reg  <= 32'd1600;
        trst_reg <= 32'd200;
        arm_reg  <= 16'd150;
    end else begin
        ctrl_reg[2] <= 1'b0;    // CLR_FAULT auto-clear

        if (wen) begin
            case (addr)
                8'h00: ctrl_reg <= wdata;
                8'h04: twd_reg  <= wdata;
                8'h08: trst_reg <= wdata;
                8'h0C: arm_reg  <= wdata[15:0];
                default: ;
            endcase
        end
    end
end

// ── Read ───────────────────────────────────────────────────────────
logic [31:0] status;
assign status = {27'h0,
                 last_kick_src,  // bit4
                 ~wdo_n,         // bit3
                 enout,          // bit2
                 fault_active,   // bit1
                 en_effective};  // bit0

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) rdata <= '0;
    else if (ren) begin
        case (addr)
            8'h00:   rdata <= ctrl_reg;
            8'h04:   rdata <= twd_reg;
            8'h08:   rdata <= trst_reg;
            8'h0C:   rdata <= {16'h0, arm_reg};
            8'h10:   rdata <= status;
            default: rdata <= 32'hDEADBEEF;
        endcase
    end
end

// ── Output assignments ─────────────────────────────────────────────
assign en_sw        = ctrl_reg[0];
assign wdi_src      = ctrl_reg[1];  // Mode: 0=HW+UART, 1=HW only
assign clr_fault    = ctrl_reg[2];
assign tWD_ms       = twd_reg;
assign tRST_ms      = trst_reg;
assign arm_delay_us = arm_reg;

endmodule