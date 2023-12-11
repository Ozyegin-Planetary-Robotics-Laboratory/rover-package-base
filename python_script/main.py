import sys
import time
import math
from typing import Required
from TMotorCANControl.servo_can import TMotorManager_servo_can
import rospy
from sensor_msgs.msg import Joy


class MotorControlState:
    def __init__(self, rotation, speed, valid_until):
        self.rotation = rotation
        self.speed = speed
        self.valid_until = valid_until


default_state = MotorControlState(0, 0, -1)
state = default_state


def millis():
    return round(time.time() * 1000)


def emit(value):
    global state
    state = value


def is_message_valid():
    return millis() > state.valid_until


def joy_callback(data):
    global ctrl_flag
    global ctrl_valid_until
    global ctrl_user_timeout_millis
    current_millis = millis()
    ctrl_flag = 2
    ctrl_valid_until = current_millis + ctrl_user_timeout_millis
    global state
    global state_timeout_millis
    rotation = data.axes[0]
    speed = data.axes[1]
    valid_until = current_millis + state_timeout_millis
    emit(MotorControlState(rotation, speed, valid_until))
    print(rotation, speed, valid_until)


def msg_callback(data):
    global ctrl_flag
    global ctrl_valid_until
    global ctrl_user_timeout_millis
    current_millis = millis()
    if ctrl_flag > 1 and ctrl_valid_until >= current_millis:
        return
    ctrl_flag = 1
    global state
    global state_timeout_millis
    rotation = data.axes[0]
    speed = data.axes[1]
    valid_until = current_millis + state_timeout_millis
    emit(MotorControlState(rotation, speed, valid_until))
    print(rotation, speed, valid_until)


def calculate_velocity_targets(rotation, speed):
    # Rover width: 64cm, wheel diameter: 30cm, distance between two wheels at the same side: 96cm
    velocity_diff = math.tan(rotation) * speed
    velocity_left_group = (speed + velocity_diff) / 2
    velocity_right_group = -velocity_diff + velocity_left_group
    return [velocity_right_group, velocity_left_group]


def update_motor_collection(motor_collection, state):
    velocities = calculate_velocity_targets(state.rotation, state.speed)
    motor_collection[0].set_velocity(velocities[0])
    motor_collection[1].set_velocity(velocities[1])
    motor_collection[2].set_velocity(velocities[0])
    motor_collection[3].set_velocity(velocities[1])
    for motor in motor_collection:
        motor.update()


def tick(motor_collection):
    global state
    if is_message_valid():
        update_motor_collection(motor_collection, state)
    else:
        update_motor_collection(motor_collection, default_state)


ctrl_flag = 0  # 0: unoccupied, 1: system, 2: user (joystick)
ctrl_valid_until = 0
ctrl_user_timeout_millis = 5000

state_timeout_millis = 100

rover_width = 64 * 0.01
rover_height = 96 * 0.01
wheel_diameter = 30 * 0.01

wheel_distance_from_center = math.sqrt(rover_width ** 2 + rover_height ** 2) / 2

try:
    rospy.init_node("ozurover-locomotion", anonymous=True)
    subscription = rospy.Subscriber("/joy", Joy, joy_callback)
    with TMotorManager_servo_can(motor_type='AK70-10', motor_ID=1) as motor1:
        with TMotorManager_servo_can(motor_type='AK70-10', motor_ID=2) as motor2:
            with TMotorManager_servo_can(motor_type='AK70-10', motor_ID=3) as motor3:
                with TMotorManager_servo_can(motor_type='AK70-10', motor_ID=4) as motor4:
                    motor_collection = [motor1, motor2, motor3, motor4]  # R, L, R, L
                    for motor in motor_collection:
                        motor.enter_velocity_control()
                    while rospy.is_shutdown() is False:
                        tick(motor_collection)

                        time.sleep(0.02)
except KeyboardInterrupt:
    sys.exit(0)
