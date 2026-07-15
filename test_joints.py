#!/usr/bin/env python3
from pymycobot import MyArmM
import time
import sys

# Change this if your port is different (e.g., /dev/ttyUSB0)
PORT = "/dev/ttyACM0"
BAUD = 1000000

print(f"Connecting to myArm on {PORT}...")
try:
    arm = MyArmM(PORT, BAUD)
    print("✅ Connected successfully!")
    
    # Power on the servos safely, checking for multiple SDK method names
    if hasattr(arm, 'power_on'):
        arm.power_on()
    elif hasattr(arm, 'set_robot_power_on'):
        arm.set_robot_power_on()
    else:
        print("⚠️ No explicit power_on method found. The robot might auto-power on.")
        
    time.sleep(1)
    
except Exception as e:
    print(f"❌ Connection failed: {e}")
    sys.exit(1)

# Helper function to handle sending angles across different SDK versions
def safe_send_angles(angles, speed):
    if hasattr(arm, 'set_joints_angle'):
        arm.set_joints_angle(angles, speed)
    elif hasattr(arm, 'send_angles'):
        arm.send_angles(angles, speed)
# print("Printing joint limits...")
arm.get_joints_min()
arm.get_joints_max()
# Keep track of where the arm is
current_angles = [0.0] * 7

print("Moving to default Zero Position safely...")
safe_send_angles(current_angles, 30) # Speed 30 for safety
time.sleep(2)

while True:
    print("\n" + "="*30)
    print(" 🤖 myArm M750 Joint Tester")
    print("="*30)
    print(f"Current Angles: {current_angles}")
    print("-" * 30)
    print("[1] Joint 1: Base (Yaw left/right)")
    print("[2] Joint 2: Shoulder (Pitch forward/back)")
    print("[3] Joint 3: Upper Arm (Roll twist)")
    print("[4] Joint 4: Elbow (Pitch up/down)")
    print("[5] Joint 5: Forearm (Roll twist)")
    print("[6] Joint 6: Wrist (Pitch up/down)")
    print("[7] Joint 7: End Effector (Spin like a drill)")
    print("-" * 30)
    print("[0] Reset all joints to 0.0")
    print("[Q] Quit")
    
    choice = input("\nSelect a joint to move (0-7 or Q): ").strip().upper()
    
    if choice == 'Q':
        print("Releasing motors and exiting...")
        if hasattr(arm, 'release_all_servos'):
            arm.release_all_servos()
        elif hasattr(arm, 'set_robot_power_off'):
            arm.set_robot_power_off()
        break
        
    if choice == '0':
        print("Resetting all joints to 0.0...")
        current_angles = [0.0] * 7
        safe_send_angles(current_angles, 30)
        continue
        
    if choice in ['1', '2', '3', '4', '5', '6', '7']:
        joint_idx = int(choice) - 1
        val = input(f"Enter angle for Joint {choice} (-150 to 150): ")
        
        try:
            angle = float(val)
            if -150.0 <= angle <= 150.0:
                current_angles[joint_idx] = angle
                print(f"Moving Joint {choice} to {angle}°...")
                safe_send_angles(current_angles, 30)
            else:
                print("❌ Error: Hardware limits are -150 to 150 degrees!")
        except ValueError:
            print("❌ Error: Please enter a valid number.")
    else:
        print("❌ Invalid selection.")