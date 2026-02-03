import sys
import os
import time
import signal
import numpy as np

# 같은 디렉토리의 can_controller를 임포트하기 위함
try:
    from can_controller import OpenArmCANController
except ImportError:
    # 만약 경로 문제로 실패하면 현재 파일 위치를 경로에 추가
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    from can_controller import OpenArmCANController

def signal_handler(sig, frame):
    print("\nExiting...")
    # 메인 루프에서 예외를 발생시키거나 플래그를 수정하는 방식이 안전하지만,
    # 여기서는 간단히 sys.exit 호출 (finally 블록은 실행됨)
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, signal_handler)
    
    print("Initializing OpenArmCANController...")
    controller = OpenArmCANController()
    
    if not controller.connect():
        print("Failed to connect to robot.")
        return

    print("Connected. Starting data stream (5Hz)...")
    print("Press Ctrl+C to stop.")
    
    controller.start()
    
    # Enable FreeDrive (Passive) for monitoring
    controller.enable_freedrive('left')
    controller.enable_freedrive('right')
    
    try:
        while True:
            state = controller.get_robot_state()
            timestamp = state['timestamp']
            left_j = state['left']['joints']
            right_j = state['right']['joints']
            
            # Format output
            l_str = ", ".join([f"{x:6.3f}" for x in left_j])
            r_str = ", ".join([f"{x:6.3f}" for x in right_j])
            
            print(f"[Time: {timestamp:.3f}]")
            print(f"  Left : [{l_str}]")
            print(f"  Right: [{r_str}]")
            print("-" * 50)
            
            time.sleep(0.2) # 5Hz
            
    except SystemExit:
        pass
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        print("Stopping controller...")
        controller.stop()

if __name__ == "__main__":
    main()
