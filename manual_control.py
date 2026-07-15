import threading
from pymycobot import MyArmMControl, utils
import keyboard
import time
m = MyArmMControl("COM5",1000000)
def init():
    # 设置初始角度
    # m.write_angles([-10.19, 8.62, 30.65, 2.19, 50.53, -4.83], 100)
    m.write_angles([0, 0, 0, 0, 0, 0], 100)
    time.sleep(1)
    m.set_gripper_state(0,100)
    time.sleep(1)

# 用于键盘输入检测的函数
def keyborad_ctrl():
    print("Start control...")
    print("Press the 0 keyboard to return to the home position")
    print("Press the 1 keyboard to move to the test position")
    print("Press the W keyboard to move in the X+ direction")
    print("Press the S keyboard to move in the X- direction")
    print("Press the D keyboard to move in the Y+ direction")
    print("Press the A keyboard to move in the Y- direction")
    print("Press the Q keyboard to move in the Z+ direction")
    print("Press the Z keyboard to move in the Z- direction")
    print("Press the T keyboard to move in the J5+ direction")
    print("Press the Y keyboard to move in the J5- direction")
    print("Press the X keyboard to move in the J6+ direction")
    print("Press the E keyboard to move in the J6- direction")
    print("Press the F keyboard to get the current coordinates")
    print("Press the R keyboard to get the current angles")
    print("Press the G keyboard to close the gripper")
    print("Press the H keyboard to open the gripper")
    print("Press the ESC keyboard to end the program")
    blocked_keys = ['w', 'a', 's', 'd', 'q', 'z','f','e','x','g','h','0','r','t','y','1']
    for key in blocked_keys:
        keyboard.block_key(key)   
    key_processed = {
        'w': False,
        'a': False,
        's': False,
        'd': False,
        'q': False,
        'z': False,
        'f':False,
        'e': False,
        'x':False,
        'g':False,
        'h':False,
        'r':False,
        't':False,
        'y':False,
        '0':False,
        '1':False
    }
    else_executed = False  
    try:
        while True:
            if keyboard.is_pressed('esc'):
                print("Exit control...")
                break           
            for key in blocked_keys:
                if keyboard.is_pressed(key) and not key_processed[key]:
                    threading.Thread(target=handle_key, args=(key,)).start()
                    key_processed[key] = True
                    else_executed = False            
            if all(not keyboard.is_pressed(key) for key in blocked_keys) and not else_executed:
                m.stop()
                else_executed = True      
            for key in key_processed:
                if not keyboard.is_pressed(key):
                    key_processed[key] = False
            time.sleep(0.01)      
    finally:
        keyboard.unhook_all()

def handle_key(key):
    if key == 'w':
        m.jog_coord(1, 1, 60)
    elif key == 'a':
        m.jog_coord(2, 0, 60)
    elif key == 'd':
        m.jog_coord(2, 1, 60)
    elif key == 's':
        m.jog_coord(1, 0, 60)
    elif key == 'q':
        m.jog_coord(3, 1, 60)
    elif key == 'z':
        m.jog_coord(3, 0, 60)
    elif key == 't':
        m.jog_angle(5,0,50)   
    elif key == 'y':
        m.jog_angle(5,1,50)
    elif key == 'e':
        m.jog_angle(6,0,50)   
    elif key == 'x':
        m.jog_angle(6,1,50)
    elif key == 'f':
        print("Current coordinates=",m.get_coords())
    elif key == 'r':
        print("Current angles=",m.get_angles())
    elif key == 'g':
        m.set_gripper_state(1,100)
    elif key == 'h':
        m.set_gripper_state(0,100)
    elif key == '0':
        m.write_angles([0, 0, 0, 0, 0, 0], 10)
    elif key == '1':
        m.write_angles([-10.19, 8.62, 30.65, 2.19, 50.53, -4.83], 10)

if __name__ == "__main__": 
    init()
    keyborad_ctrl()