import serial
import time
import struct

# Đổi 'COM3' thành cổng COM thật của board Kiwi 1P5 trên máy bạn (hoặc /dev/ttyUSB0 nếu dùng Linux)
PORT = 'COM3' 
BAUD = 115200

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
except Exception as e:
    print(f"Lỗi mở cổng COM: {e}")
    exit()

def send_frame(cmd, addr, data_int=None):
    if data_int is not None:
        # Pack số nguyên thành 4 byte Little-Endian
        data_bytes = list(struct.pack('<I', data_int))
        length = 4
    else:
        data_bytes = []
        length = 0
        
    # Tính Checksum
    chk = cmd ^ addr ^ length
    for b in data_bytes:
        chk ^= b
        
    # Ghép Frame
    frame = [0x55, cmd, addr, length] + data_bytes + [chk]
    
    # Gửi đi
    print(f"\n[TX] Gửi: {' '.join([f'{x:02X}' for x in frame])}")
    ser.write(bytearray(frame))
    
    # Đọc phản hồi (tối đa 10 byte)
    time.sleep(0.05)
    resp = ser.read(10)
    if resp:
        print(f"[RX] Nhận: {' '.join([f'{x:02X}' for x in resp])}")
    else:
        print("[RX] Timeout: Không có phản hồi!")

# ==================== KỊCH BẢN TEST ====================
print("--- TEST WATCHDOG UART ---")

# 1. Đọc thử STATUS (CMD=04, ADDR=0x10)
print("1. Đọc Status ban đầu:")
send_frame(0x04, 0x10)

# 2. Đổi tWD_ms thành 5000ms (CMD=01, ADDR=0x04)
print("\n2. Đổi tWD = 5000ms:")
send_frame(0x01, 0x04, 5000)

# 3. Bật EN_SW qua UART (CMD=01, ADDR=0x00, Data=1)
print("\n3. Bật Watchdog (EN_SW=1):")
send_frame(0x01, 0x00, 1)

# 4. Kick vài phát
print("\n4. Bơm Kick liên tục:")
for i in range(3):
    send_frame(0x03, 0x00) # CMD KICK
    time.sleep(1)

ser.close()