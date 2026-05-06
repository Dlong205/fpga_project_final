/*`timescale 1ns/1ps
module tb_watchdog;

localparam real CLK_P = 37.037;
logic clk=0, rst_n=0;
always #(CLK_P/2) clk=~clk;

// DUT: watchdog_core với tham số nhỏ để sim nhanh
logic btn_wdi_n=1, btn_en_n=1;
logic uart_kick=0, uart_en=0, clr_fault=0;
logic [31:0] tWD_ms=2, tRST_ms=2;
logic [15:0] arm_delay_us=50;
logic wdo_n, enout, en_effective, fault_active, wdi_kick_out, last_kick_src;

watchdog_core #(.CLK_MHZ(27)) dut (
    .clk(clk), .rst_n(rst_n),
    .btn_wdi_n(btn_wdi_n), .btn_en_n(btn_en_n),
    .uart_kick(uart_kick), .uart_en(uart_en),
    .tWD_ms(tWD_ms), .tRST_ms(tRST_ms), .arm_delay_us(arm_delay_us),
    .clr_fault(clr_fault),
    .wdo_n(wdo_n), .enout(enout),
    .en_effective(en_effective), .fault_active(fault_active),
    .wdi_kick_out(wdi_kick_out), .last_kick_src(last_kick_src)
);

localparam int MS = 27_000;
localparam int US = 27;
int pass_cnt=0, fail_cnt=0;

task kick_uart;
    @(posedge clk); uart_kick=1;
    @(posedge clk); uart_kick=0;
endtask
task wait_ms(int n); repeat(n*MS) @(posedge clk); endtask
task wait_us(int n); repeat(n*US) @(posedge clk); endtask
task chk(string s, logic a, logic e);
    if(a===e) begin $display("[PASS] %s",s); pass_cnt++; end
    else      begin $display("[FAIL] %s exp=%b got=%b",s,e,a); fail_cnt++; end
endtask

initial begin
    repeat(10) @(posedge clk); rst_n=1; repeat(5) @(posedge clk);

    // TC1: Normal kick
    $display("\n=== TC1: Normal Kick ===");
    uart_en=1; wait_us(60);
    chk("TC1 enout",enout,1);
    kick_uart; wait_ms(1); kick_uart; wait_ms(1);
    chk("TC1 no fault",fault_active,0);
    uart_en=0; wait_ms(1);

    // TC2: Timeout
    $display("\n=== TC2: Timeout ===");
    uart_en=1; wait_us(60);
    wait_ms(3);
    chk("TC2 fault",fault_active,1);
    chk("TC2 wdo_n",wdo_n,0);
    wait_ms(3);
    chk("TC2 recover",fault_active,0);
    uart_en=0; wait_ms(1);

    // TC3: Disable
    $display("\n=== TC3: Disable ===");
    uart_en=0; kick_uart; wait_ms(4);
    chk("TC3 no fault",fault_active,0);
    chk("TC3 enout=0",enout,0);

    // TC4: arm_delay
    $display("\n=== TC4: arm_delay ===");
    uart_en=1; wait_us(25);
    chk("TC4 enout=0 in arm",enout,0);
    wait_us(30); chk("TC4 enout=1 after arm",enout,1);
    uart_en=0; wait_ms(1);

    // TC5: param change
    $display("\n=== TC5: Param change ===");
    tWD_ms=1; uart_en=1; wait_us(60);
    wait_ms(2); chk("TC5 fault tWD=1ms",fault_active,1);
    clr_fault=1; @(posedge clk); clr_fault=0;
    wait_ms(1); chk("TC5 clr_fault",wdo_n,1);
    uart_en=0;

    $display("\n============================");
    $display("RESULT: %0d PASS / %0d FAIL",pass_cnt,fail_cnt);
    $display("============================");
    $finish;
end

initial begin
    $dumpfile("tb_watchdog.vcd");
    $dumpvars(0,tb_watchdog);
    #50_000_000; $display("TIMEOUT"); $finish;
end
endmodule*/

`timescale 1ns/1ps
module tb_watchdog;

localparam real CLK_P = 37.037;
logic clk=0, rst_n=0;
always #(CLK_P/2) clk=~clk;

// DUT: watchdog_core với tham số nhỏ để sim nhanh
logic btn_wdi_n=1, btn_en_n=1;
logic uart_kick=0, uart_en=0, clr_fault=0;
logic [31:0] tWD_ms=2, tRST_ms=2;
logic [15:0] arm_delay_us=50;
logic wdo_n, enout, en_effective, fault_active, wdi_kick_out, last_kick_src;

watchdog_core #(.CLK_MHZ(27)) dut (
    .clk(clk), .rst_n(rst_n),
    .btn_wdi_n(btn_wdi_n), .btn_en_n(btn_en_n),
    .uart_kick(uart_kick), .uart_en(uart_en),
    .tWD_ms(tWD_ms), .tRST_ms(tRST_ms), .arm_delay_us(arm_delay_us),
    .clr_fault(clr_fault),
    .wdo_n(wdo_n), .enout(enout),
    .en_effective(en_effective), .fault_active(fault_active),
    .wdi_kick_out(wdi_kick_out), .last_kick_src(last_kick_src)
);

localparam int MS = 27_000;
localparam int US = 27;
int pass_cnt=0, fail_cnt=0;

task kick_uart;
    @(posedge clk); uart_kick=1;
    @(posedge clk); uart_kick=0;
endtask

task wait_ms(int n); repeat(n*MS) @(posedge clk); endtask
task wait_us(int n); repeat(n*US) @(posedge clk); endtask

task chk(string s, logic a, logic e);
    if(a===e) begin $display("[PASS] %s",s); pass_cnt++; end
    else      begin $display("[FAIL] %s exp=%b got=%b",s,e,a); fail_cnt++; end
endtask

initial begin
    repeat(10) @(posedge clk); rst_n=1; repeat(5) @(posedge clk);

    // TC1: Normal kick
    $display("\n=== TC1: Normal Kick ===");
    uart_en=1; wait_us(60);
    chk("TC1 enout",enout,1);
    kick_uart; wait_ms(1); kick_uart; wait_ms(1);
    chk("TC1 no fault",fault_active,0);
    uart_en=0; wait_ms(1);

    // TC2: Timeout
    $display("\n=== TC2: Timeout ===");
    uart_en=1; wait_us(60);
    wait_ms(3);
    chk("TC2 fault",fault_active,1);
    chk("TC2 wdo_n",wdo_n,0);
    
    // Đã sửa: Đợi đúng thời gian tRST (2ms) + 100us để FSM kịp cập nhật
    wait_ms(2); wait_us(100); 
    chk("TC2 recover",fault_active,0);
    uart_en=0; wait_ms(1);

    // TC3: Disable
    $display("\n=== TC3: Disable ===");
    uart_en=0; kick_uart; wait_ms(4);
    chk("TC3 no fault",fault_active,0);
    chk("TC3 enout=0",enout,0);

    // TC4: arm_delay
    $display("\n=== TC4: arm_delay ===");
    uart_en=1; wait_us(25);
    chk("TC4 enout=0 in arm",enout,0);
    wait_us(30); chk("TC4 enout=1 after arm",enout,1);
    uart_en=0; wait_ms(1);

    // TC5: Param change
    $display("\n=== TC5: Param change ===");
    tWD_ms=1; uart_en=1; wait_us(60);
    wait_ms(2); chk("TC5 fault tWD=1ms",fault_active,1);
    
    // Bắn xung xóa lỗi
    clr_fault=1; @(posedge clk); clr_fault=0;
    
    // Đã sửa: Chỉ chờ 10us (tránh bị timeout cắn tiếp) rồi check ngay
    wait_us(10); 
    chk("TC5 clr_fault",wdo_n,1);
    uart_en=0;

    $display("\n============================");
    $display("RESULT: %0d PASS / %0d FAIL",pass_cnt,fail_cnt);
    $display("============================");
    $finish;
end

initial begin
    $dumpfile("tb_watchdog.vcd");
    $dumpvars(0,tb_watchdog);
    #50_000_000; $display("TIMEOUT"); $finish;
end
endmodule
