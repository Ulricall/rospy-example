import sys
import select
import math
import rospy
import random
from geometry_msgs.msg import Twist, Vector3
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from nav_msgs.msg import Odometry
from mavros_msgs.srv import SetMavFrame

# 非阻塞输入
def non_blocking_input(prompt):
    sys.stdout.write(prompt)
    sys.stdout.flush()
    ready, _, _ = select.select([sys.stdin], [], [], 0)
    if ready:
        return sys.stdin.readline().rstrip('\n')
    else:
        return None

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

drone_odom = Odometry()

def odometry_cb(msg):
    global drone_odom
    drone_odom = msg

state_sub = rospy.Subscriber("mavros/state", State, callback=state_cb)
drone_odom_sub = rospy.Subscriber("mavros/local_position/odom", Odometry, callback=odometry_cb)

velocity_for_publish = Twist()

def main():
    global velocity_for_publish

    rospy.init_node("offboard_node")

    vec_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)

    rospy.wait_for_service("mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    set_mav_frame_client = rospy.ServiceProxy("mavros/setpoint_velocity/mav_frame", SetMavFrame)
    set_mav_frame_client(8)

    rate = rospy.Rate(20)

    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    velocity_for_publish.linear.x = 0
    velocity_for_publish.linear.y = 0
    velocity_for_publish.linear.z = 1.0
    velocity_for_publish.angular.x = 0
    velocity_for_publish.angular.y = 0
    velocity_for_publish.angular.z = 0

    for _ in range(100):
        if(rospy.is_shutdown()):
            break

        vec_pub.publish(velocity_for_publish)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    mission_step = 0

    while(not rospy.is_shutdown()):
        if (current_state.mode == "MANUAL" or current_state.mode == "STABILIZED"):
            rospy.loginfo("Mode is MANUAL or STABILIZED, exiting.")
            print("Mode is MANUAL or STABILIZED, exiting.")
            exit()
        
        if(current_state.mode != 'OFFBOARD' and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()

        else:
            if mission_step == 0:
                rospy.logwarn_once("Mission Step 0.")
                if (not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                    if (arming_client.call(arm_cmd).success):
                        rospy.loginfo("Vehicle armed")
                        mission_step = 1
                    last_req = rospy.Time.now()
            
            elif mission_step == 1:
                rospy.logwarn_once("Mission Step 1. TAKEOFF.")

                velocity_for_publish.linear.x = 0
                velocity_for_publish.linear.y = 0
                velocity_for_publish.linear.z = 2.0
                velocity_for_publish.angular.x = 0
                velocity_for_publish.angular.y = 0
                velocity_for_publish.angular.z = 0

                if (rospy.Time.now() - last_req) > rospy.Duration(6.0):
                    mission_step = 2
                    last_req = rospy.Time.now()
                
            elif mission_step == 2:
                rospy.logwarn_once("Mission Step 2. Go forward.")

                velocity_for_publish.linear.x = 2.0
                velocity_for_publish.linear.y = 0
                velocity_for_publish.linear.z = 0
                velocity_for_publish.angular.x = 0
                velocity_for_publish.angular.y = 0
                velocity_for_publish.angular.z = 0

                if (rospy.Time.now() - last_req) > rospy.Duration(3.0):
                    mission_step = 3
                    last_req = rospy.Time.now()

            elif mission_step == 3:
                rospy.logwarn_once("Mission Step 3. Go left.")

                velocity_for_publish.linear.x = 0
                velocity_for_publish.linear.y = 2.0
                velocity_for_publish.linear.z = 0
                velocity_for_publish.angular.x = 0
                velocity_for_publish.angular.y = 0
                velocity_for_publish.angular.z = 0

                if (rospy.Time.now() - last_req) > rospy.Duration(3.0):
                    mission_step = 4
                    last_req = rospy.Time.now()
            
            elif mission_step == 4:
                rospy.logwarn_once("Mission Step 4. Go backward.")

                velocity_for_publish.linear.x = -2.0
                velocity_for_publish.linear.y = 0
                velocity_for_publish.linear.z = 0
                velocity_for_publish.angular.x = 0
                velocity_for_publish.angular.y = 0
                velocity_for_publish.angular.z = 0

                if (rospy.Time.now() - last_req) > rospy.Duration(3.0):
                    mission_step = 5
                    last_req = rospy.Time.now()
            
            elif mission_step == 5:
                rospy.logwarn_once("Mission Step 5. Go right.")

                velocity_for_publish.linear.x = 0
                velocity_for_publish.linear.y = -2.0
                velocity_for_publish.linear.z = 0
                velocity_for_publish.angular.x = 0
                velocity_for_publish.angular.y = 0
                velocity_for_publish.angular.z = 0

                if (rospy.Time.now() - last_req) > rospy.Duration(3.0):
                    mission_step = 6
                    last_req = rospy.Time.now()
                
            elif mission_step == 6:
                rospy.logwarn_once("Mission Step 6. LANDING.")
                offb_set_mode.custom_mode = "AUTO.LAND"
                if (current_state.mode != "AUTO.LAND" and (rospy.Time.now() - last_req) > rospy.Duration(1.5)):
                    if (set_mode_client.call(offb_set_mode).mode_sent):
                        rospy.loginfo("AUTO.LAND enabled")
                        mission_step = 7
                    last_req = rospy.Time.now()
            
            else:
                rospy.logwarn_once("Mission Completed.")
                break

        vec_pub.publish(velocity_for_publish)
        rate.sleep()
        
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass


    