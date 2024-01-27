from flask import Flask, request, jsonify
import rospy
from ozurover_messages.msg import GPS, Abort, AddMarker
from ozurover_messages.srv import GoalEnqueue, GoalAbort
from threading import Thread

app = Flask(__name__)

# Initialize ROS node
rospy.init_node('flask_ros_bridge', anonymous=True)

# Global variable to store rover's GPS data
rover_gps_coordinates = [0, 0]  # Default latitude and longitude

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

def read_base_gps_coordinates():
    try:
        with open('base_gps_coordinates.txt', 'r') as file:
            coordinates = file.read().strip().split(',')
            return [float(coordinates[0]), float(coordinates[1])]
    except Exception as e:
        print(f"Error reading base GPS coordinates: {e}")
        return [0, 0]

def update_rover_gps():
    def callback(data):
        global rover_gps_coordinates
        rover_gps_coordinates = [data.latitude, data.longitude]

    rospy.Subscriber("ares/gps", GPS, callback)
    rospy.spin()

Thread(target=update_rover_gps).start()

@app.route('/gps/base', methods=['GET'])
def get_base_gps():
    coordinates = read_base_gps_coordinates()
    return jsonify({"Coordinates": coordinates})

@app.route('/gps/rover', methods=['GET'])
def get_rover_gps():
    return jsonify({"Coordinates": rover_gps_coordinates})

if __name__ == '__main__':
    app.run(debug=True)
