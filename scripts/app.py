from flask import Flask, request, jsonify
import rospy
from ozurover_messages import GPS, Abort, AddMarker
from ozurover_messages.srv import GoalEnqueue, GoalAbort

# Initialize ROS node
rospy.init_node('flask_ros_bridge', anonymous=True)

@app.route('/goal/enqueue', methods=['POST'])
def enqueue_goal():
    data = request.json
    requestMsg = AddMarker()
    requestMsg.gps = GPS()
    requestMsg.gps.latitude = data['gps'][0]
    requestMsg.gps.longitude = data['gps'][1]
    requestMsg.type = data['type']

    # Send ROS service request to /ares/goal/enqueue
    rospy.wait_for_service('/ares/goal/enqueue')
    try:
        goal_enqueue = rospy.ServiceProxy('/ares/goal/enqueue', GoalEnqueue)
        response = goal_enqueue(requestMsg)
        success = response.success
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
        success = False
    
    return jsonify(success=success)

@app.route('/goal/abort', methods=['POST'])
def abort_goal():
    rospy.wait_for_service('/ares/goal/abort')
    try:
        goal_abort = rospy.ServiceProxy('/ares/goal/abort', GoalAbort)
        response = goal_abort()  # No arguments needed for the service call
        success = response.success  
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
        success = False
    
    return jsonify(success=success)
