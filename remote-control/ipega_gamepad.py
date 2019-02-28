#import evdev
from evdev import InputDevice, categorize, ecodes
import sys, tty, termios, time
import socket
import numpy as np

def remap(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

UDP_IP = "192.168.178.242"
UDP_PORT = 4210

gamepad = InputDevice('/dev/input/event16')

# Constants
FORWARD = 1;
REVERSE = 0;

#prints out device info at start
print(gamepad)

# Save values
left_x_val = 127
left_y_val = 127
right_x_val = 127
right_y_val = 127
left_trigger = 0
right_trigger = 0

for event in gamepad.read_loop():

    if event.type == ecodes.EV_ABS:
        absevent = categorize(event)

        if ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_X":
            left_x_val = absevent.event.value
        elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_Y":
            left_y_val = absevent.event.value
        elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_Z":
            right_x_val = absevent.event.value
        elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_RZ":
            right_y_val = absevent.event.value
        elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_BRAKE":
            left_trigger = absevent.event.value
        elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_GAS":
            right_trigger = absevent.event.value

    # Map controllers command to create linear and rotation velocitiesl
    joy_power = remap(left_y_val, 255, 0, -1000, 1000)
    joy_steering = remap(left_x_val, 0, 255, -1000, 1000)

    camera_yaw = remap(right_x_val, 0, 255, 140, 0)
    camera_pitch = remap(right_y_val, 0, 255, 180, 0)

    left_motor = joy_power + (joy_steering * 2) / 2
    right_motor = joy_power + (-joy_steering * 2) / 2

    if left_motor > 1023:
        left_motor = 1023
    if right_motor > 1023:
        right_motor = 1023
    if left_motor < -1023:
        left_motor = -1023
    if right_motor < -1023:
        right_motor = -1023
    

    # print("Left Motor: {0}, Right Motor: {1}, LeftX: {2}, LeftY: {3}".format(left_motor, right_motor, joy_steering, joy_power))

    angular_dir_left = 1
    angular_dir_right = 1

    if left_motor > 100:
        angular_dir_left = FORWARD
    elif left_motor < -100:
        angular_dir_left = REVERSE

    if right_motor > 100:
        angular_dir_right = FORWARD
    elif right_motor < -100:
        angular_dir_right = REVERSE

    send_list = []

    send_list.append(str(abs(left_motor)))
    send_list.append(str(abs(right_motor)))
    send_list.append(str(angular_dir_left))
    send_list.append(str(angular_dir_right))
    send_list.append(str(camera_yaw))
    send_list.append(str(camera_pitch))

    string_to_send = "-".join(send_list)

    print string_to_send

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(string_to_send, (UDP_IP, UDP_PORT))