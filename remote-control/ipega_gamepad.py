#import evdev
import sys, tty, termios, time
import socket
import pygame
import numpy as np

def remap(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

pygame.display.init()
pygame.joystick.init()
pygame.joystick.Joystick(0).init()

# Prints the joystick's name
JoyName = pygame.joystick.Joystick(0).get_name()
print "Name of the joystick:"
print JoyName

UDP_IP = "192.168.178.242"
UDP_PORT = 4210

# Constants
FORWARD = 1;
REVERSE = 0;


while True:
    pygame.event.pump()
    left_x_val = pygame.joystick.Joystick(0).get_axis(0) # Left X Axis
    left_y_val = pygame.joystick.Joystick(0).get_axis(1) # Left Y Axis
    right_x_val = pygame.joystick.Joystick(0).get_axis(2) # Right X Axis
    right_y_val = pygame.joystick.Joystick(0).get_axis(3) # Right Y Axis

    left_motor = left_y_val + (left_x_val * 2) / 2
    right_motor = left_y_val + (-left_x_val * 2) / 2

    left_motor *= 1023
    right_motor *= 1023

    camera_yaw = remap(right_x_val, -1, 1, 140, 0)
    camera_pitch = remap(right_y_val, -1, 1, 180, 0)

    if left_motor > 1023:
        left_motor = 1023
    if right_motor > 1023:
        right_motor = 1023
    if left_motor < -1023:
        left_motor = -1023
    if right_motor < -1023:
        right_motor = -1023
    

    # print("Left Motor: {0}, Right Motor: {1}, LeftX: {2}, LeftY: {3}".format(left_motor, right_motor, joy_steering, joy_power))

    angular_dir_left = FORWARD
    angular_dir_right = FORWARD

    if left_motor > 100:
        angular_dir_left = FORWARD
    elif left_motor < -100:
        angular_dir_left = REVERSE

    if right_motor > 100:
        angular_dir_right = FORWARD
    elif right_motor < -100:
        angular_dir_right = REVERSE

    send_list = []

    send_list.append(str(abs(int(left_motor))))
    send_list.append(str(abs(int(right_motor))))
    send_list.append(str(angular_dir_left))
    send_list.append(str(angular_dir_right))
    send_list.append(str(int(camera_yaw)))
    send_list.append(str(int(camera_pitch)))

    string_to_send = "-".join(send_list)

    # print string_to_send
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(string_to_send, (UDP_IP, UDP_PORT))
    time.sleep(0.01)