import pyrealsense2 as rs

def list_devices():
    ctx = rs.context()
    devices = ctx.query_devices()
    
    if len(devices) == 0:
        print("No RealSense devices connected.")
        return

    print(f"Found {len(devices)} RealSense devices:")
    print("-" * 40)
    
    for i, dev in enumerate(devices):
        name = dev.get_info(rs.camera_info.name)
        serial = dev.get_info(rs.camera_info.serial_number)
        usb_type = dev.get_info(rs.camera_info.usb_type_descriptor)
        print(f"Device {i}: {name}")
        print(f"  Serial Number: {serial}")
        print(f"  USB Type: {usb_type}")
        print("-" * 40)

if __name__ == "__main__":
    list_devices()
