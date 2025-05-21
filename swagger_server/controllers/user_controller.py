import connexion
import time
import math
import rospy
import yaml
import threading
import subprocess
from geometry_msgs.msg import PoseWithCovarianceStamped

from swagger_server.models.user import User  # noqa: E501
from swagger_server import util

def parse_rostopic_output(output):
    """
    Parses the YAML-like output from `rostopic echo` into a usable Python dictionary.
    """
    try:
        docs = yaml.load_all(output, yaml.FullLoader)
        for doc in docs:
            return doc
    except yaml.YAMLError as e:
        rospy.logerr(f"Error parsing YAML: {e}")
        return None

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

def get_robot_status(robot_id):  # noqa: E501

    command = ['rostopic', 'echo', f'/{robot_id}/move_base/status', '-n', '1']
    
    # Run the command and capture the output
    result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    
    if result.returncode != 0:
        rospy.logerr(f"Error running command: {result.stderr}")
        return None

    output = parse_rostopic_output(result.stdout)

    # Navigation Status referenced from actionlib_msgs/GoalStatus
    # Link: https://docs.ros.org/en/noetic/api/actionlib_msgs/html/msg/GoalStatus.html


    print(f"output = {output}")

    navigation_status = 10 # IDLE
    navigation_message = ""

    if len(output["status_list"]) != 0:
        navigation_status = output["status_list"][0]["status"]
        navigation_message = output["status_list"][0]["text"]
    else:
        navigation_message = "Idling"

    robot_status = {
        'map_name': 'L1',
        'battery': 0.9,
        'navigation_status': navigation_status,
        'msg': navigation_message
    }

    return robot_status

def get_robot_position(robot_id):  # noqa: E501
    command = ['rostopic', 'echo', f'/{robot_id}/amcl_pose', '-n', '1']
    
    # Run the command and capture the output
    result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    
    if result.returncode != 0:
        rospy.logerr(f"Error running command: {result.stderr}")
        return None

    output = parse_rostopic_output(result.stdout)

    _, _, yaw_z = euler_from_quaternion(
        output['pose']['pose']['orientation']['x'],
        output['pose']['pose']['orientation']['y'],
        output['pose']['pose']['orientation']['z'],
        output['pose']['pose']['orientation']['w']
    )

    robot_position = {
        'location_x': output['pose']['pose']['position']['x'],
        'location_y': output['pose']['pose']['position']['y'],
        'location_th': yaw_z
    }

    return robot_position

def dock_robot(robot_id):  # noqa: E501
    
    # TODO(cardboardcode): Check if robot is already docked
    # If not, execute below.
    # Otherwise, return "Robot already docked."

    if robot_id == "tb3_0":
        input_location_x = -3.995364293097464
        input_location_y = 8.01519874566102
        input_location_th = -0.005692177206749547
    elif robot_id == "tb3_1":
        input_location_x = 4.028384051144058
        input_location_y = 8.016791981786778
        input_location_th =  0.004432755549700517
    elif robot_id == "robot0":
        input_location_x = 0.49379590649562627
        input_location_y = -0.012615691842014616
        input_location_th = 3.028522675911123

    # Define the rostopic publish command
    topic = f'/{robot_id}/move_base/goal'
    message_type = 'move_base_msgs/MoveBaseActionGoal'
    # message = body['msg']  # The message must be enclosed in quotes

    qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, input_location_th)

    message = { 
          "header": {
            "seq": 0,
            "stamp": {
              "secs": 0,
              "nsecs": 0
            },
            "frame_id": "map"
          },
          "goal_id": {
            "stamp": {
              "secs": 0,
              "nsecs": 0
            },
            "id": ""
          },
          "goal": {
            "target_pose": {
              "header": {
                "seq": 0,
                "stamp": {
                  "secs": 0,
                  "nsecs": 0
                },
                "frame_id": "map"
              },
              "pose": {
                "position": {
                  "x": input_location_x,
                  "y": input_location_y,
                  "z": 0.0
                },
                "orientation": {
                  "x": qx,
                  "y": qy,
                  "z": qz,
                  "w": qw
                }
              }
            }
          }
        }

    # Construct the rostopic command
    nav_cmd = ['rostopic', 'pub', '-1', topic, message_type, str(message)]

    try:
        # Use subprocess to run the command
        result = subprocess.run(nav_cmd, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

        # Print the output of the command
        print(f"Message published:\n{result.stdout}")

    except subprocess.CalledProcessError as e:
        # Handle errors if the subprocess fails
        print(f"Failed to publish message: {e.stderr}")

    # If not near destination already, check for navigation to start.
    curr_status = get_robot_status(robot_id)

    while curr_status["navigation_status"] != 1:
        print(f"navigation_status = {curr_status['navigation_status']}")
        curr_status = get_robot_status(robot_id)

    return 'Docking Started...'

def stop_robot(robot_id, body=None):  # noqa: E501
    # Define the rostopic publish command
    topic = f'/{robot_id}/move_base/cancel'
    message_type = 'actionlib_msgs/GoalID'

    message = "{}"
    # Construct the rostopic command
    cmd = ['rostopic', 'pub', '-1', topic, message_type, str(message)]

    try:
        # Use subprocess to run the command
        result = subprocess.run(cmd, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        
        # Print the output of the command
        print(f"Message published:\n{result.stdout}")
        
    except subprocess.CalledProcessError as e:
        # Handle errors if the subprocess fails
        print(f"Failed to publish message: {e.stderr}")

    return f'Cancelling navigation task of [{robot_id}]...'

def ping_system():  # noqa: E501
    return 'system online'

def publish_ros1_string(body=None):  # noqa: E501
    # Define the rostopic publish command
    topic = '/talker'
    message_type = 'std_msgs/String'
    message = body['msg']  # The message must be enclosed in quotes

    # Construct the rostopic command
    cmd = ['rostopic', 'pub', '-1', topic, message_type, message]

    try:
        # Use subprocess to run the command
        result = subprocess.run(cmd, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        
        # Print the output of the command
        print(f"Message published:\n{result.stdout}")
        
    except subprocess.CalledProcessError as e:
        # Handle errors if the subprocess fails
        print(f"Failed to publish message: {e.stderr}")

    return 'Publishing ROS 1 string...'

def euler_to_quaternion(th_x, th_y, th_z):
    # Calculate cos and sin for each Euler angle
    cy = math.cos(th_z * 0.5)
    sy = math.sin(th_z * 0.5)
    cr = math.cos(th_x * 0.5)
    sr = math.sin(th_x * 0.5)
    cp = math.cos(th_y * 0.5)
    sp = math.sin(th_y * 0.5)

    # Compute quaternion components
    qw = (cy * cr * cp + sy * sr * sp)
    qx = (cy * sr * cp - sy * cr * sp)
    qy = (cy * cr * sp + sy * sr * cp)
    qz = (sy * cr * cp - cy * sr * sp)

    return qx, qy, qz, qw

def send_nav_goal(robot_id, body=None):  # noqa: E501

    # Check if a navigation task is currently being processed.
    # If true, reject.
    # If not near destination already, check for navigation to start.
    curr_status = get_robot_status(robot_id)

    # if curr_status["navigation_status"] == 1 or curr_status["navigation_status"] == 2:
    #   error_response = "Still processing previous. Rejecting navigation request."
    #   return error_response, 400

    # Check if robot is already near waypoint.
    robot_position = get_robot_position(robot_id)
    input_robot_name = robot_id
    input_location_x = body['location_x']
    input_location_y = body['location_y']
    input_location_th = body['location_th']

    is_already_at_destination = is_within_threshold(
        robot_position["location_x"],
        robot_position["location_y"],
        input_location_x,
        input_location_y,
        threshold=0.5
        )
    
    if is_already_at_destination:
        return 'Already at destination.'

    # Define the rostopic publish command
    # /tb3_1/move_base_simple/goal
    topic = f'/{input_robot_name}/move_base/goal'
    print(f"Publishing on topic {topic}")
    message_type = 'move_base_msgs/MoveBaseActionGoal'

    qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, input_location_th)

    message = { 
          "header": {
            "seq": 0,
            "stamp": {
              "secs": 0,
              "nsecs": 0
            },
            "frame_id": "map"
          },
          "goal_id": {
            "stamp": {
              "secs": 0,
              "nsecs": 0
            },
            "id": ""
          },
          "goal": {
            "target_pose": {
              "header": {
                "seq": 0,
                "stamp": {
                  "secs": 0,
                  "nsecs": 0
                },
                "frame_id": "map"
              },
              "pose": {
                "position": {
                  "x": input_location_x,
                  "y": input_location_y,
                  "z": 0.0
                },
                "orientation": {
                  "x": qx,
                  "y": qy,
                  "z": qz,
                  "w": qw
                }
              }
            }
          }
        }

    # Construct the rostopic command
    cmd = ['rostopic', 'pub', '-1', topic, message_type, str(message)]

    try:
        # Use subprocess to run the command
        result = subprocess.run(cmd, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        
        # Print the output of the command
        print(f"Message published:\n{result.stdout}")
        
    except subprocess.CalledProcessError as e:
        # Handle errors if the subprocess fails
        print(f"Failed to publish message: {e.stderr}")

    # If not near destination already, check for navigation to start.
    curr_status = get_robot_status(robot_id)

    while curr_status["navigation_status"] != 1:
        print(f"navigation_status = {curr_status['navigation_status']}")
        curr_status = get_robot_status(robot_id)

    return 'Navigation Started...'

def localise_robot(robot_id,body=None):  # noqa: E501

    input_location_x = body['location_x']
    input_location_y = body['location_y']
    input_location_th = body['location_th']

    # Define the rostopic publish command
    topic = f'/{robot_id}/initialpose'
    message_type = 'geometry_msgs/PoseWithCovarianceStamped'
    # message = body['msg']  # The message must be enclosed in quotes

    qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, input_location_th)

    covariance = [
        0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]

    message = { 
          "header": {
            "seq": 0,
            "stamp": {
              "secs": 0,
              "nsecs": 0
            },
            "frame_id": "map"
          },
          "pose": {
            "pose": {
              "position": {
                "x": input_location_x,
                "y": input_location_y,
                "z": 0.0
              },
              "orientation": {
                "x": qx,
                "y": qy,
                "z": qz,
                "w": qw
              }
            },
            "covariance": covariance
          }
        }

    # Construct the rostopic command
    cmd = ['rostopic', 'pub', '-1', topic, message_type, str(message)]

    try:
        # Use subprocess to run the command
        result = subprocess.run(cmd, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        
        # Print the output of the command
        print(f"Message published:\n{result.stdout}")
        
    except subprocess.CalledProcessError as e:
        # Handle errors if the subprocess fails
        print(f"Failed to publish message: {e.stderr}")

    return 'Navigation Request Sent.'

def is_within_threshold(x1: float, y1: float, x2: float, y2: float, threshold: float) -> bool:
    """
    Calculates the Euclidean distance between two (x, y) coordinates and
    returns True if the distance is within the specified threshold, False otherwise.

    Args:
        x1: The x-coordinate of the first point.
        y1: The y-coordinate of the first point.
        x2: The x-coordinate of the second point.
        y2: The y-coordinate of the second point.
        threshold: The maximum distance allowed for the points to be considered within.

    Returns:
        True if the distance between the points is less than or equal to the threshold,
        False otherwise.
    """
    distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return distance <= threshold