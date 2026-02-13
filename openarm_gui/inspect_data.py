import h5py
import numpy as np
import sys

def inspect_h5(file_path):
    try:
        with h5py.File(file_path, 'r') as f:
            print(f'--- H5 File Structure: {file_path} ---')
            def print_attrs(name, obj):
                shift = '  ' * name.count('/')
                if isinstance(obj, h5py.Dataset):
                    print(f'{shift}Dataset: {name}, Shape: {obj.shape}, Dtype: {obj.dtype}')
                elif isinstance(obj, h5py.Group):
                    print(f'{shift}Group: {name}')
            
            f.visititems(print_attrs)
            
            print('\n--- Sample Data (Last 5 steps) ---')
            if 'observations/left_q' in f:
                l_q = f['observations/left_q'][-5:]
                print(f'Left Joint Positions (Meters/Radians):\n{l_q}')
                # Specifically joint 8 (index 7) for gripper
                gripper_l = l_q[:, 7]
                print(f'Left Gripper positions: {gripper_l}')
                
            if 'observations/right_q' in f:
                r_q = f['observations/right_q'][-5:]
                print(f'\nRight Joint Positions:\n{r_q}')
                gripper_r = r_q[:, 7]
                print(f'Right Gripper positions: {gripper_r}')

            if 'observations/timestamp' in f:
                ts = f['observations/timestamp'][-5:]
                print(f'\nTimestamps: {ts}')
                
    except Exception as e:
        print(f'Error reading H5 file: {e}')

if __name__ == '__main__':
    if len(sys.argv) > 1:
        inspect_h5(sys.argv[1])
    else:
        print('Please provide HDF5 file path.')
