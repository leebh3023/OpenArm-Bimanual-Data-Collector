import pyrealsense2 as rs
import time

def reset_devices():
    ctx = rs.context()
    devices = ctx.query_devices()
    
    if len(devices) == 0:
        print("No devices found to reset.")
        return

    for dev in devices:
        print(f"Resetting device: {dev.get_info(rs.camera_info.name)} S/N: {dev.get_info(rs.camera_info.serial_number)}")
        try:
            dev.hardware_reset()
            print("Reset command sent.")
        except Exception as e:
            print(f"Failed to reset: {e}")
    
    print("Waiting 5 seconds for devices to re-enumerate...")
    time.sleep(5)

if __name__ == "__main__":
    reset_devices()
