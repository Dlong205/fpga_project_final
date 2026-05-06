
module uart_frame #(
    parameter integer CLK_MHZ = 27,
    parameter integer BAUD    = 115200
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        rx,
    output reg         tx,

    output reg  [7:0]  reg_addr,
    output reg  [31:0] reg_wdata,
    output reg         reg_wen,
    input  wire [31:0] reg_rdata,
    output reg         reg_ren,

    output reg         uart_kick,
    output reg         soft_rst,

    input  wire        en_effective,
    input  wire        fault_active,
    input  wire        enout,
    input  wire        wdi_kick
);

// ════════════════════════════════════════════════════════════════════
// Parameters
// ════════════════════════════════════════════════════════════════════
localparam integer BIT_CYC  = (CLK_MHZ * 1000000) / BAUD;
localparam integer HALF_CYC = BIT_CYC / 2;

// ════════════════════════════════════════════════════════════════════
// 1. UART RX
// ════════════════════════════════════════════════════════════════════
reg rx_s1, rx_s2;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin rx_s1<=1'b1; rx_s2<=1'b1; end
    else        begin rx_s1<=rx; rx_s2<=rx_s1; end
end

localparam RX_IDLE=2'd0, RX_START=2'd1, RX_DATA=2'd2, RX_STOP=2'd3;
reg [1:0]   rx_state;
reg [8:0]   rx_cnt;     // 9 bit
reg [3:0]   rx_bit;     // 4 bit
reg [7:0]   rx_shift;
reg [7:0]   rx_data;
reg         rx_valid;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rx_state<=RX_IDLE; rx_cnt<=9'd0;
        rx_bit<=4'd0; rx_shift<=8'd0;
        rx_data<=8'd0; rx_valid<=1'b0;
    end else begin
        rx_valid <= 1'b0;
        case (rx_state)
            RX_IDLE: begin
                if (!rx_s2) begin
                    rx_state <= RX_START;
                    rx_cnt   <= 9'd0;
                end
            end
            RX_START: begin
                if (rx_cnt == 9'(HALF_CYC-1)) begin
                    rx_cnt <= 9'd0;
                    if (!rx_s2) begin rx_state<=RX_DATA; rx_bit<=4'd0; end
                    else         rx_state<=RX_IDLE;
                end else rx_cnt<=rx_cnt+9'd1;
            end
            RX_DATA: begin
                if (rx_cnt == 9'(BIT_CYC-1)) begin
                    rx_cnt   <= 9'd0;
                    rx_shift <= {rx_s2, rx_shift[7:1]};
                    if (rx_bit==4'd7) rx_state<=RX_STOP;
                    else rx_bit<=rx_bit+4'd1;
                end else rx_cnt<=rx_cnt+9'd1;
            end
            RX_STOP: begin
                if (rx_cnt == 9'(BIT_CYC-1)) begin
                    rx_cnt   <= 9'd0;
                    rx_state <= RX_IDLE;
                    if (rx_s2) begin rx_data<=rx_shift; rx_valid<=1'b1; end
                end else rx_cnt<=rx_cnt+9'd1;
            end
            default: rx_state<=RX_IDLE;
        endcase
    end
end

// ════════════════════════════════════════════════════════════════════
// 2. TX Engine
// ════════════════════════════════════════════════════════════════════
reg [7:0]  tx_byte;
reg        tx_send;
reg        tx_busy;
reg [8:0]  tx_cnt;
reg [9:0]  tx_shift;
reg [3:0]  tx_bits;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        tx<=1'b1; tx_cnt<=9'd0; tx_bits<=4'd0;
        tx_shift<=10'h3FF; tx_busy<=1'b0;
    end else begin
        if (!tx_busy && tx_send) begin
            tx_shift <= {1'b1, tx_byte, 1'b0};
            tx_cnt   <= 9'd0;
            tx_bits  <= 4'd0;
            tx_busy  <= 1'b1;
            tx       <= 1'b0;
        end else if (tx_busy) begin
            if (tx_cnt == 9'(BIT_CYC-1)) begin
                tx_cnt   <= 9'd0;
                tx_bits  <= tx_bits+4'd1;
                tx_shift <= {1'b1, tx_shift[9:1]};
                tx       <= tx_shift[1];
                if (tx_bits==4'd9) begin tx_busy<=1'b0; tx<=1'b1; end
            end else tx_cnt<=tx_cnt+9'd1;
        end
    end
end

// ════════════════════════════════════════════════════════════════════
// 3. TX Arbiter
// ════════════════════════════════════════════════════════════════════
reg [7:0] frame_byte; reg frame_req;
reg [7:0] debug_byte; reg debug_req;
reg       tx_busy_d;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) tx_busy_d<=1'b0;
    else        tx_busy_d<=tx_busy;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin tx_byte<=8'd0; tx_send<=1'b0; end
    else begin
        tx_send <= 1'b0;
        if (!tx_busy && !tx_busy_d && !tx_send) begin
            if (frame_req) begin
                tx_byte <= frame_byte; tx_send <= 1'b1;
            end else if (debug_req) begin
                tx_byte <= debug_byte; tx_send <= 1'b1;
            end
        end
    end
end

// ════════════════════════════════════════════════════════════════════
// 4. Frame Parser + Response
// ════════════════════════════════════════════════════════════════════
localparam FP_IDLE=3'd0, FP_CMD=3'd1, FP_ADDR=3'd2,
           FP_LEN=3'd3,  FP_DATA=3'd4, FP_CHK=3'd5;

reg [2:0]  fp_state;
reg [7:0]  fp_cmd, fp_addr_r, fp_len;
reg [7:0]  fp_d0, fp_d1, fp_d2, fp_d3;
reg [7:0]  fp_chk;
reg [1:0]  fp_didx;

reg [7:0]  resp [0:6];
reg [2:0]  resp_total;
reg [2:0]  resp_idx;
reg        resp_busy;
reg        build_read_resp;
reg        build_read_resp_d;
reg [3:0]  reset_idx;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        fp_state<=FP_IDLE; fp_chk<=8'd0; fp_didx<=2'd0;
        reg_wen<=1'b0; reg_ren<=1'b0; uart_kick<=1'b0;
        reg_addr<=8'd0; reg_wdata<=32'd0;
        resp_busy<=1'b0; resp_idx<=3'd0; resp_total<=3'd0;
        frame_req<=1'b0; frame_byte<=8'd0;
        fp_cmd<=8'd0; fp_addr_r<=8'd0; fp_len<=8'd0;
        fp_d0<=8'd0; fp_d1<=8'd0; fp_d2<=8'd0; fp_d3<=8'd0;
        resp[0]<=8'd0; resp[1]<=8'd0; resp[2]<=8'd0; resp[3]<=8'd0;
        resp[4]<=8'd0; resp[5]<=8'd0; resp[6]<=8'd0;
        build_read_resp<=1'b0;
        build_read_resp_d<=1'b0;
        reset_idx<=4'd0;
        soft_rst<=1'b0;
    end else begin
        reg_wen           <= 1'b0;
        reg_ren           <= 1'b0;
        uart_kick         <= 1'b0;
        frame_req         <= 1'b0;
        build_read_resp   <= 1'b0;
        build_read_resp_d <= build_read_resp;

        if (rx_valid) begin
            case (fp_state)
                FP_IDLE: if (rx_data==8'h55) fp_state<=FP_CMD;
                FP_CMD: begin
                    fp_cmd<=rx_data; fp_chk<=rx_data; fp_state<=FP_ADDR;
                end
                FP_ADDR: begin
                    fp_addr_r<=rx_data; fp_chk<=fp_chk^rx_data; fp_state<=FP_LEN;
                end
                FP_LEN: begin
                    fp_len<=rx_data; fp_chk<=fp_chk^rx_data; fp_didx<=2'd0;
                    fp_state<=(rx_data==8'h0) ? FP_CHK : FP_DATA;
                end
                FP_DATA: begin
                    fp_chk<=fp_chk^rx_data;
                    case (fp_didx)
                        2'd0: fp_d0<=rx_data;
                        2'd1: fp_d1<=rx_data;
                        2'd2: fp_d2<=rx_data;
                        2'd3: fp_d3<=rx_data;
                    endcase
                    if (fp_didx==fp_len[1:0]-2'd1) fp_state<=FP_CHK;
                    else fp_didx<=fp_didx+2'd1;
                end
                FP_CHK: begin
                    fp_state<=FP_IDLE;
                    if (rx_data==fp_chk) begin
                        case (fp_cmd)
                            8'h01: begin // WRITE
                                reg_addr  <=fp_addr_r; reg_wen<=1'b1;
                                reg_wdata <={fp_d3,fp_d2,fp_d1,fp_d0};
                                resp[0]<=8'h55; resp[1]<=8'hAA; resp[2]<=8'hAA;
                                resp_total<=3'd3; resp_idx<=3'd0; resp_busy<=1'b1;
                            end
                            8'h02: begin // READ
                                reg_addr<=fp_addr_r; reg_ren<=1'b1;
                                build_read_resp<=1'b1;
                            end
                            8'h03: begin // KICK
                                uart_kick<=1'b1;
                                resp[0]<=8'h55; resp[1]<=8'hAA; resp[2]<=8'hAA;
                                resp_total<=3'd3; resp_idx<=3'd0; resp_busy<=1'b1;
                            end
                            8'h05: begin // RESET - clear CTRL only
                                reg_addr<=8'h00; reg_wen<=1'b1; reg_wdata<=32'd0;
                                resp[0]<=8'h55; resp[1]<=8'hAA; resp[2]<=8'hAA;
                                resp_total<=3'd3; resp_idx<=3'd0; resp_busy<=1'b1;
                            end
                            8'h06: begin // RESET ALL via soft_rst (1 frame)
                                soft_rst<=1'b1;
                                resp[0]<=8'h55; resp[1]<=8'hAA; resp[2]<=8'hAA;
                                resp_total<=3'd3; resp_idx<=3'd0; resp_busy<=1'b1;
                            end
                            8'h04: begin // GET_STATUS
                                reg_addr<=8'h10; reg_ren<=1'b1;
                                build_read_resp<=1'b1;
                            end
                            default: begin
                                resp[0]<=8'h55; resp[1]<=8'hFF; resp[2]<=8'hFF;
                                resp_total<=3'd3; resp_idx<=3'd0; resp_busy<=1'b1;
                            end
                        endcase
                    end else begin
                        resp[0]<=8'h55; resp[1]<=8'hFF; resp[2]<=8'hFF;
                        resp_total<=3'd3; resp_idx<=3'd0; resp_busy<=1'b1;
                    end
                end
                default: fp_state<=FP_IDLE;
            endcase
        end

        if (build_read_resp_d) begin
            resp[0] <= 8'h55;
            resp[1] <= 8'hAA;
            resp[2] <= reg_rdata[7:0];
            resp[3] <= reg_rdata[15:8];
            resp[4] <= reg_rdata[23:16];
            resp[5] <= reg_rdata[31:24];
            resp[6] <= 8'hAA ^ reg_rdata[7:0] ^ reg_rdata[15:8]
                             ^ reg_rdata[23:16] ^ reg_rdata[31:24];
            resp_total<=3'd7; resp_idx<=3'd0; resp_busy<=1'b1;
        end

        // ── Send response bytes ───────────────────────────────────
        // FIX: Đợi tín hiệu !tx_busy_d và !tx_send để chống trôi byte
        if (resp_busy && !tx_busy && !tx_busy_d && !tx_send && !frame_req) begin
            case (resp_idx)
                3'd0: frame_byte<=resp[0];
                3'd1: frame_byte<=resp[1];
                3'd2: frame_byte<=resp[2];
                3'd3: frame_byte<=resp[3];
                3'd4: frame_byte<=resp[4];
                3'd5: frame_byte<=resp[5];
                3'd6: frame_byte<=resp[6];
                default: frame_byte<=8'd0;
            endcase
            frame_req<=1'b1;
            if (resp_idx==resp_total-3'd1) resp_busy<=1'b0;
            else resp_idx<=resp_idx+3'd1;
        end
    end
end

// ════════════════════════════════════════════════════════════════════
// 5. Debug Log — Combinational ROM (Fixes Gowin BRAM packing & latency)
// ════════════════════════════════════════════════════════════════════
localparam DBG_IDLE=2'd0, DBG_LOAD=2'd1,
           DBG_WAIT_START=2'd2, DBG_WAIT_DONE=2'd3;

reg [1:0]  dbg_state;
reg [2:0]  dbg_msg;
reg [4:0]  dbg_idx;

reg [4:0] msg_len;
always @(*) begin
    case (dbg_msg)
        3'd0: msg_len = 5'd19;
        3'd1: msg_len = 5'd16;
        3'd2: msg_len = 5'd17;
        3'd3: msg_len = 5'd15;
        3'd4: msg_len = 5'd13;
        3'd5: msg_len = 5'd16;
        3'd6: msg_len = 5'd15;
        default: msg_len = 5'd0;
    endcase
end

reg [7:0] rom_char;
always @(*) begin
    rom_char = 8'h20;
    case (dbg_msg)
        3'd0: begin
            case (dbg_idx)
                5'd0:  rom_char = 8'h3D;
                5'd1:  rom_char = 8'h3D;
                5'd2:  rom_char = 8'h3D;
                5'd3:  rom_char = 8'h20;
                5'd4:  rom_char = 8'h57;
                5'd5:  rom_char = 8'h44;
                5'd6:  rom_char = 8'h47;
                5'd7:  rom_char = 8'h20;
                5'd8:  rom_char = 8'h52;
                5'd9:  rom_char = 8'h45;
                5'd10: rom_char = 8'h41;
                5'd11: rom_char = 8'h44;
                5'd12: rom_char = 8'h59;
                5'd13: rom_char = 8'h20;
                5'd14: rom_char = 8'h3D;
                5'd15: rom_char = 8'h3D;
                5'd16: rom_char = 8'h3D;
                5'd17: rom_char = 8'h0D;
                5'd18: rom_char = 8'h0A;
                default: rom_char = 8'h00;
            endcase
        end
        3'd1: begin
            case (dbg_idx)
                5'd0:  rom_char = 8'h5B;
                5'd1:  rom_char = 8'h57;
                5'd2:  rom_char = 8'h44;
                5'd3:  rom_char = 8'h47;
                5'd4:  rom_char = 8'h5D;
                5'd5:  rom_char = 8'h20;
                5'd6:  rom_char = 8'h44;
                5'd7:  rom_char = 8'h49;
                5'd8:  rom_char = 8'h53;
                5'd9:  rom_char = 8'h41;
                5'd10: rom_char = 8'h42;
                5'd11: rom_char = 8'h4C;
                5'd12: rom_char = 8'h45;
                5'd13: rom_char = 8'h44;
                5'd14: rom_char = 8'h0D;
                5'd15: rom_char = 8'h0A;
                default: rom_char = 8'h00;
            endcase
        end
        3'd2: begin
            case (dbg_idx)
                5'd0:  rom_char = 8'h5B;
                5'd1:  rom_char = 8'h57;
                5'd2:  rom_char = 8'h44;
                5'd3:  rom_char = 8'h47;
                5'd4:  rom_char = 8'h5D;
                5'd5:  rom_char = 8'h20;
                5'd6:  rom_char = 8'h41;
                5'd7:  rom_char = 8'h52;
                5'd8:  rom_char = 8'h4D;
                5'd9:  rom_char = 8'h49;
                5'd10: rom_char = 8'h4E;
                5'd11: rom_char = 8'h47;
                5'd12: rom_char = 8'h2E;
                5'd13: rom_char = 8'h2E;
                5'd14: rom_char = 8'h2E;
                5'd15: rom_char = 8'h0D;
                5'd16: rom_char = 8'h0A;
                default: rom_char = 8'h00;
            endcase
        end
        3'd3: begin
            case (dbg_idx)
                5'd0:  rom_char = 8'h5B;
                5'd1:  rom_char = 8'h57;
                5'd2:  rom_char = 8'h44;
                5'd3:  rom_char = 8'h47;
                5'd4:  rom_char = 8'h5D;
                5'd5:  rom_char = 8'h20;
                5'd6:  rom_char = 8'h52;
                5'd7:  rom_char = 8'h55;
                5'd8:  rom_char = 8'h4E;
                5'd9:  rom_char = 8'h4E;
                5'd10: rom_char = 8'h49;
                5'd11: rom_char = 8'h4E;
                5'd12: rom_char = 8'h47;
                5'd13: rom_char = 8'h0D;
                5'd14: rom_char = 8'h0A;
                default: rom_char = 8'h00;
            endcase
        end
        3'd4: begin
            case (dbg_idx)
                5'd0:  rom_char = 8'h5B;
                5'd1:  rom_char = 8'h57;
                5'd2:  rom_char = 8'h44;
                5'd3:  rom_char = 8'h47;
                5'd4:  rom_char = 8'h5D;
                5'd5:  rom_char = 8'h20;
                5'd6:  rom_char = 8'h4B;
                5'd7:  rom_char = 8'h49;
                5'd8:  rom_char = 8'h43;
                5'd9:  rom_char = 8'h4B;
                5'd10: rom_char = 8'h21;
                5'd11: rom_char = 8'h0D;
                5'd12: rom_char = 8'h0A;
                default: rom_char = 8'h00;
            endcase
        end
        3'd5: begin
            case (dbg_idx)
                5'd0:  rom_char = 8'h5B;
                5'd1:  rom_char = 8'h57;
                5'd2:  rom_char = 8'h44;
                5'd3:  rom_char = 8'h47;
                5'd4:  rom_char = 8'h5D;
                5'd5:  rom_char = 8'h20;
                5'd6:  rom_char = 8'h54;
                5'd7:  rom_char = 8'h49;
                5'd8:  rom_char = 8'h4D;
                5'd9:  rom_char = 8'h45;
                5'd10: rom_char = 8'h4F;
                5'd11: rom_char = 8'h55;
                5'd12: rom_char = 8'h54;
                5'd13: rom_char = 8'h21;
                5'd14: rom_char = 8'h0D;
                5'd15: rom_char = 8'h0A;
                default: rom_char = 8'h00;
            endcase
        end
        3'd6: begin
            case (dbg_idx)
                5'd0:  rom_char = 8'h5B;
                5'd1:  rom_char = 8'h57;
                5'd2:  rom_char = 8'h44;
                5'd3:  rom_char = 8'h47;
                5'd4:  rom_char = 8'h5D;
                5'd5:  rom_char = 8'h20;
                5'd6:  rom_char = 8'h52;
                5'd7:  rom_char = 8'h45;
                5'd8:  rom_char = 8'h43;
                5'd9:  rom_char = 8'h4F;
                5'd10: rom_char = 8'h56;
                5'd11: rom_char = 8'h45;
                5'd12: rom_char = 8'h52;
                5'd13: rom_char = 8'h0D;
                5'd14: rom_char = 8'h0A;
                default: rom_char = 8'h00;
            endcase
        end
        default: rom_char = 8'h00;
    endcase
end

reg en_prev_d, fault_prev_d, enout_prev_d, kick_prev_d;
reg boot_sent;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        en_prev_d<=1'b0; fault_prev_d<=1'b0;
        enout_prev_d<=1'b0; kick_prev_d<=1'b0;
    end else begin
        en_prev_d    <= en_effective;
        fault_prev_d <= fault_active;
        enout_prev_d <= enout;
        kick_prev_d  <= wdi_kick;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        dbg_state<=DBG_IDLE; dbg_msg<=3'd0;
        dbg_idx<=5'd0; debug_req<=1'b0; debug_byte<=8'd0;
        boot_sent<=1'b0;
    end else begin
        debug_req<=1'b0;
        case (dbg_state)
            DBG_IDLE: begin
                if (!boot_sent && !resp_busy && !tx_busy) begin
                    dbg_msg<=3'd0; dbg_idx<=5'd0;
                    dbg_state<=DBG_LOAD; boot_sent<=1'b1;
                end else if (!resp_busy && !tx_busy) begin
                    if (fault_active && !fault_prev_d)
                        begin dbg_msg<=3'd5; dbg_idx<=5'd0; dbg_state<=DBG_LOAD; end
                    else if (!fault_active && fault_prev_d && en_effective)
                        begin dbg_msg<=3'd6; dbg_idx<=5'd0; dbg_state<=DBG_LOAD; end
                    else if (wdi_kick && !kick_prev_d)
                        begin dbg_msg<=3'd4; dbg_idx<=5'd0; dbg_state<=DBG_LOAD; end
                    else if (enout && !enout_prev_d)
                        begin dbg_msg<=3'd3; dbg_idx<=5'd0; dbg_state<=DBG_LOAD; end
                    // BỎ TÍNH NĂNG IN CHỮ ARMING VÌ BỊ TRÙNG TIMING VỚI LỆNH UART  
                  //  else if (en_effective && !en_prev_d && !enout)
                  //      begin dbg_msg<=3'd2; dbg_idx<=5'd0; dbg_state<=DBG_LOAD; end
                    else if (!en_effective && en_prev_d)
                        begin dbg_msg<=3'd1; dbg_idx<=5'd0; dbg_state<=DBG_LOAD; end
                end
            end

            DBG_LOAD: begin
                if (!tx_busy && !frame_req && !debug_req) begin
                    debug_byte <= rom_char;
                    debug_req  <= 1'b1;
                    dbg_state  <= DBG_WAIT_START;
                end
            end

            DBG_WAIT_START: begin
                if (tx_busy) dbg_state<=DBG_WAIT_DONE;
            end

            DBG_WAIT_DONE: begin
                if (!tx_busy) begin
                    if (dbg_idx == msg_len - 5'd1)
                        dbg_state<=DBG_IDLE;
                    else begin
                        dbg_idx  <=dbg_idx+5'd1;
                        dbg_state<=DBG_LOAD;
                    end
                end
            end

            default: dbg_state<=DBG_IDLE;
        endcase
    end
end

endmodule