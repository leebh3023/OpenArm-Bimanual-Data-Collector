import can
import time
import sys

def test_interface(channel):
    print(f"--- Testing Interface: {channel} ---")
    
    # 1. Check Interface operstate
    try:
        with open(f"/sys/class/net/{channel}/operstate", "r") as f:
            state = f.read().strip()
        print(f"[{channel}] Operstate: {state}")
        if state not in ["up", "unknown"]:
            print(f"[{channel}] Interface is DOWN. Skipping.")
            return
    except Exception as e:
        print(f"[{channel}] Could not check operstate: {e}")

    # 2. Connect SocketCAN
    try:
        bus = can.interface.Bus(channel=channel, bustype='socketcan', fd=True, bitrate=1000000, data_bitrate=5000000)
        print(f"[{channel}] Connected to SocketCAN.")
    except Exception as e:
        print(f"[{channel}] Failed to connect: {e}")
        return

    # 3. Send Enable Command to Motor ID 1 (Test)
    # Enable Motor (0xFC)
    motor_id = 1
    data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]
    msg = can.Message(arbitration_id=motor_id, data=data, is_extended_id=False, is_fd=True, bitrate_switch=True)
    
    print(f"[{channel}] Sending ENABLE to Motor ID {motor_id}...")
    try:
        bus.send(msg)
        print(f"[{channel}] Message sent.")
    except Exception as e:
        print(f"[{channel}] Failed to send off: {e}")

    # 4. Listen for traffic for 3 seconds
    print(f"[{channel}] Listening for 3 seconds...")
    start = time.time()
    count = 0
    while time.time() - start < 3.0:
        msg = bus.recv(timeout=0.1)
        if msg:
            count += 1
            print(f"[{channel}] RX: ID=0x{msg.arbitration_id:X} Len={len(msg.data)} Data={msg.data.hex()}")
            if count >= 10:
                print(f"[{channel}] ... (more messages supressed) ...")
                break
    
    if count == 0:
        print(f"[{channel}] NO MESSAGES RECEIVED.")
    else:
        print(f"[{channel}] Total {count} messages received.")
            
    bus.shutdown()
    print("-" * 30)

if __name__ == "__main__":
    test_interface('can0') # Right
    print("\n")
    test_interface('can1') # Left
