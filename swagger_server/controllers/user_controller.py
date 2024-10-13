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

def get_robot_position():  # noqa: E501
    command = ['rostopic', 'echo', '/amcl_pose', '-n', '1']
    
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

def dock_robot():  # noqa: E501
    
    # TODO(cardboardcode): Check if robot is already docked
    # If not, execute below.
    # Otherwise, return "Robot already docked."

    input_location_x = 0.49379590649562627
    input_location_y = -0.012615691842014616
    input_location_th = 3.028522675911123

    # Define the rostopic publish command
    topic = '/move_base/goal'
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

    # TODO(cardboardcode): Check that robot has indeed reached home waypoint before publishing dock action.
    is_robot_home = False
    while not is_robot_home:
        print("[INFO] - Robot navigating to [home]...")
        time.sleep(1)
        # Check robot position
        buffer = 0.4
        robot_pos = get_robot_position()
        x_delta = abs(robot_pos['location_x'] - 0.49379590649562627)
        y_delta = abs(robot_pos['location_y'] - -0.012615691842014616)
        th_delta = abs(abs(robot_pos['location_th']) - 3.028522675911123)

        print(f"[{x_delta}, {y_delta}, {th_delta}]")
        if ( x_delta < buffer and y_delta < buffer and th_delta < buffer):
            is_robot_home = True

    print(f"[INFO] - Navigation completed. Robot is [home]...")
    print(f"[INFO] - Issuing autodocking")

    dock_cmd = ['rostopic', 'pub', '/autodock_action/goal', 'autodock_core/AutoDockingActionGoal', '{}', '--once']

    try:
        # Use subprocess to run the command
        result = subprocess.run(dock_cmd, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        
        # Print the output of the command
        print(f"Message published:\n{result.stdout}")
        
    except subprocess.CalledProcessError as e:
        # Handle errors if the subprocess fails
        print(f"Failed to publish message: {e.stderr}")

    return 'Navigation Request Sent.'

def move_robot(body=None):  # noqa: E501
    # Define the rostopic publish command
    topic = '/cmd_vel'
    message_type = 'geometry_msgs/Twist'

    message = {
            "linear":{
                "x": body["linear_speed"],
                "y": 0,
                "z": 0
            },
            "angular":{
                "x": 0,
                "y": 0,
                "z": body["angular_speed"]
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

    return 'Publishing twist to Robot...'

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

def send_nav_goal(body=None):  # noqa: E501

    input_location_x = body['location_x']
    input_location_y = body['location_y']
    input_location_th = body['location_th']

    # Define the rostopic publish command
    topic = '/move_base/goal'
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