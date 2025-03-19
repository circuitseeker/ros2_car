#!/usr/bin/env python3

import cv2
import time

def print_camera_properties(cap):
    # List of properties to check
    props = [
        (cv2.CAP_PROP_FRAME_WIDTH, "Width"),
        (cv2.CAP_PROP_FRAME_HEIGHT, "Height"),
        (cv2.CAP_PROP_FPS, "FPS"),
        (cv2.CAP_PROP_BRIGHTNESS, "Brightness"),
        (cv2.CAP_PROP_CONTRAST, "Contrast"),
        (cv2.CAP_PROP_SATURATION, "Saturation"),
        (cv2.CAP_PROP_GAIN, "Gain"),
        (cv2.CAP_PROP_EXPOSURE, "Exposure"),
        (cv2.CAP_PROP_AUTOFOCUS, "Autofocus"),
        (cv2.CAP_PROP_FOCUS, "Focus")
    ]
    
    print("\nCamera properties:")
    for prop_id, prop_name in props:
        value = cap.get(prop_id)
        print(f"{prop_name}: {value}")

def main():
    print("Testing camera access...")
    
    # Try different camera indices
    for camera_index in range(3):
        print(f"\nTrying camera index {camera_index}")
        cap = cv2.VideoCapture(camera_index)
        
        if not cap.isOpened():
            print(f"Failed to open camera {camera_index}")
            continue
            
        print(f"Successfully opened camera {camera_index}")
        
        # Set camera properties for Logitech C270
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cap.set(cv2.CAP_PROP_FPS, 30)
        
        # Print camera properties
        print_camera_properties(cap)
        
        # Try to read multiple frames
        for i in range(5):
            print(f"\nAttempting to read frame {i+1}/5")
            ret, frame = cap.read()
            if ret:
                print(f"Successfully read frame {i+1}")
                print(f"Frame shape: {frame.shape}")
                print(f"Frame data type: {frame.dtype}")
                print(f"Frame min/max values: {frame.min()}/{frame.max()}")
                
                # Save the frame
                filename = f'camera_{camera_index}_frame_{i+1}.jpg'
                cv2.imwrite(filename, frame)
                print(f"Saved frame as {filename}")
                
                # Small delay between frames
                time.sleep(0.5)
            else:
                print(f"Failed to read frame {i+1}")
        
        # Release this camera before trying the next one
        print(f"\nReleasing camera {camera_index}")
        cap.release()
        time.sleep(1)  # Give camera time to release

if __name__ == '__main__':
    main() 