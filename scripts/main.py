#!/usr/bin/python3
# Not necessary?
# from typing import Required
import sys

import time
import math
import requests
from TMotorCANControl.servo_can import TMotorManager_servo_can
import rospy
from multiprocessing import Process
from sensor_msgs.msg import Joy

telemetry_server = "localhost:8080"
telemetry_uri = "/rover-data"
telemetry_enabled = False


class MotorControlState:
    def __init__(self, rotation, speed, valid_until):
        self.rotation = rotation
        self.speed = speed
        self.valid_until = valid_until


default_state = MotorControlState(0, 0, -1)
state = default_state


def millis():
    return round(time.time() * 1000)


start_millis = millis()


def emit(value):
    global state
    state = value


def is_message_valid():
    return millis() > state.valid_until


def joy_callback(data):
    global ctrl_flag
    global ctrl_valid_until
    global ctrl_user_timeout_millis
    print("Joy callback {} {}", data.axes[0], data.axes[1])
    current_millis = millis()
    ctrl_flag = 2
    ctrl_valid_until = current_millis + ctrl_user_timeout_millis
    global state
    global state_timeout_millis
    rotation = data.axes[0]
    speed = data.axes[1]
    valid_until = current_millis + state_timeout_millis
    emit(MotorControlState(math.asin(rotation), speed, valid_until))
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
    rotation = data.axes[1]
    speed = data.axes[0]
    valid_until = current_millis + state_timeout_millis
    emit(MotorControlState(rotation, speed, valid_until))
    print(rotation, speed, valid_until)


def calculate_velocity_targets(rotation, speed):
    # Rover width: 64cm, wheel diameter: 30cm, distance between two wheels at the same side: 96cm
    velocity_diff = math.tan(rotation * 0.95) * speed
    velocity_left_group = (speed + (velocity_diff / 2))
    velocity_right_group = velocity_diff - velocity_left_group
    return [velocity_right_group, velocity_left_group]


def update_motor_collection(motor_collection, state):
    velocities = calculate_velocity_targets(state.rotation, state.speed)
    motor_collection[0].velocity = velocities[0]
    motor_collection[1].velocity = velocities[1]
    motor_collection[2].velocity = velocities[0]
    motor_collection[3].velocity = velocities[1]
    for motor in motor_collection:
        motor.update()


def tick(motor_collection):
    global state
    if is_message_valid():
        update_motor_collection(motor_collection, state)
    else:
        update_motor_collection(motor_collection, default_state)


def get_motor_state_map(motor):
    return {"current": motor.get_current_qaxis_amps(), "speed": motor.get_motor_velocity_radians_per_second(),
            "temperature": float(motor.temperature)}


def get_motor_collection_state(motor_collection):
    state_map = []
    for i, m in enumerate(motor_collection):
        state_map += get_motor_state_map(m)
    return state_map


def telemetryTask(motor_collection):
    global telemetry_uri
    global telemetry_server
    global start_millis
    global ctrl_flag
    while rospy.is_shutdown() is False:
        epoch = millis()
        delta_t = epoch - start_millis
        motor_data = {
            "LocoMotorData": get_motor_collection_state(motor_collection),
            "Mode": "idle",
            "InitialModeIndex": 0,
            "Connected": True,
            "BatteryVoltage": 10.0,
            "ctrl": {
                "flag": ctrl_flag
            },
            "DirtTemperature": 10.0,
            "DirtHumidity": 10.0,
            "timestamp": epoch,
            "delta_t": delta_t,
            "consoleLog": [],

        }
        requests.post(telemetry_server + telemetry_uri, json=motor_data)
        time.sleep(0.2)


def scheduleTelemetryTaskProcess(motor_collection):
    p = Process(target=telemetryTask, args=[motor_collection])
    p.start()


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
    with TMotorManager_servo_can(motor_type='AK70-10', motor_ID=1) as motor1:
        with TMotorManager_servo_can(motor_type='AK70-10', motor_ID=2) as motor2:
            with TMotorManager_servo_can(motor_type='AK70-10', motor_ID=3) as motor3:
                with TMotorManager_servo_can(motor_type='AK70-10', motor_ID=4) as motor4:
                    motor_collection = [motor1, motor2, motor3, motor4]  # R, L, R, L
                    if telemetry_enabled:
                        scheduleTelemetryTaskProcess(motor_collection)
                    rospy.Subscriber("/joy", Joy, joy_callback)
                    for motor in motor_collection:
                        motor.enter_velocity_control()
                    while rospy.is_shutdown() is False:
                        tick(motor_collection)
                        time.sleep(0.02)
                        
except KeyboardInterrupt:
    sys.exit(0)
