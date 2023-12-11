import sys

import cv2
import mediapipe as mp
import time
import rospy
import moveit_commander
import geometry_msgs.msg
import cv2
from cv_bridge import CvBridge, CvBridgeError

cap = cv2.VideoCapture(0)

mpHands = mp.solutions.hands
hands = mpHands.Hands(static_image_mode=False,
                      max_num_hands=2,
                      min_detection_confidence=0.5,
                      min_tracking_confidence=0.5)
mpDraw = mp.solutions.drawing_utils

pTime = 0
cTime = 0

try:
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    group_name = robot.get_group_names()[0]
    print(group_name)
    move_group = moveit_commander.MoveGroupCommander(group_name)
    scene = moveit_commander.PlanningSceneInterface()

    # Add the ground plane as collision object
    plane_pose = geometry_msgs.msg.PoseStamped()
    plane_pose.header.frame_id = "world"
    plane_pose.pose.orientation.w = 1.0
    scene.add_plane("ground_plane", plane_pose)

    move_group.set_max_acceleration_scaling_factor(1)
    move_group.set_max_velocity_scaling_factor(1)

    target_joint_states = [
        [1.207, -1.731, 1.205, -1.642, -1.355, -0.295],
        [2.261, -1.088, 0.997, -1.732, -2.251, 0.601],
        [0.628, -1.073, 1.032, -1.471, -0.870, -1.136],
        [1.458, -1.341, 2.492, -3.641, -1.558, -0.044],
        [1.458, -0.720, 0.531, -1.635, -1.559, -0.043],
    ]

    ki = 0
    while True:
        success, img = cap.read()
        img = cv2.flip(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), 1)
        imgRGB = img
        results = hands.process(imgRGB)
        # print(results.multi_hand_landmarks)

        mx = -1
        my = -1
        tx = -1
        ty = -1
        nx = -1
        ny = -1
        kx = -1
        ky = -1

        if results.multi_hand_landmarks:
            for handLms in results.multi_hand_landmarks:
                for id, lm in enumerate(handLms.landmark):
                    print(id, lm)
                    h, w, c = img.shape
                    cx, cy = int(lm.x * w), int(lm.y * h)
                    if id == 4:
                        tx = cx
                        ty = cy
                    if id == 12:
                        mx = cx
                        my = cy
                    if id == 1:
                        nx = cx
                        ny = cy
                    if id == 17:
                        kx = cx
                        ky = cy

                    # if id ==0:
                    cv2.putText(img, str(id), (cx, cy), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)
                    cv2.circle(img, (cx, cy), 3, (255, 0, 255), cv2.FILLED)

                mpDraw.draw_landmarks(img, handLms, mpHands.HAND_CONNECTIONS)
        cTime = time.time()
        fps = 1 / (cTime - pTime)
        pTime = cTime

        cv2.putText(img, str(int(fps)), (10, 70), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)

        if nx != kx:
            dist = int((abs(mx - tx) + abs(my - ty)) / (abs(nx - kx) + abs(ny - ky)) * 100)
            if dist < 80:
                print(ki)
                success = False
                move_group.set_joint_value_target(target_joint_states[ki])
                success, trajectory, planning_time, error_code = move_group.plan()
                move_group.execute(trajectory, wait=False)
                ki = ki + 1
                ki = (ki + 1) % 5



        cv2.imshow("Image", img)
        cv2.waitKey(1)
except KeyboardInterrupt:
    cap.release()
    sys.exit(0)
