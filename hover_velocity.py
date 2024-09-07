import math
import rospy
from geometry_msgs.msg import Twist, Vector3
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from nav_msgs.msg import Odometry

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

drone_odom = Odometry()

def odometry_cb(msg):
    global drone_odom
    drone_odom = msg

err_x = 0
err_y = 0
err_z = 0
err_x0 = 0
err_y0 = 0
err_z0 = 0
err_x_err = 0
err_y_err = 0
err_z_err = 0
velocity = Vector3()
adj_kp = 1.0
adj_kd = 1.88

def velocity_control(target_x, target_y, target_z):
    global err_x, err_y, err_z
    global err_x0, err_y0, err_z0
    global err_x_err, err_y_err, err_z_err
    global velocity
    global adj_kp, adj_kp

    err_x = target_x - drone_odom.pose.pose.position.x
    err_y = target_y - drone_odom.pose.pose.position.y
    err_z = target_z - drone_odom.pose.pose.position.z

    err_x_err = err_x - err_x0
    err_y_err = err_y - err_y0
    err_z_err = err_z - err_z0

    err_x0 = err_x
    err_y0 = err_y
    err_z0 = err_z

    velocity.x = adj_kp * err_x + adj_kd * err_x_err
    velocity.y = adj_kp * err_y + adj_kd * err_y_err
    velocity.z = adj_kp * err_z + adj_kd * err_z_err

    if(velocity.x > 2.0):
        velocity.x = 2.0
    elif(velocity.x < -2.0):
        velocity.x = -2.0
    
    if(velocity.y > 2.0):
        velocity.y = 2.0
    elif(velocity.y < -2.0):
        velocity.y = -2.0
    
    if(velocity.z > 2.0):
        velocity.z = 2.0
    elif(velocity.z < -2.0):
        velocity.z = -2.0


def if_reached(target_x: float, target_y: float, target_z: float, threshold: float = 0.20) -> bool:
    current_x = drone_odom.pose.pose.position.x
    current_y = drone_odom.pose.pose.position.y
    current_z = drone_odom.pose.pose.position.z
    distance_x = abs(target_x - current_x)
    distance_y = abs(target_y - current_y)
    distance_z = abs(target_z - current_z)
    distance = math.sqrt(math.pow(distance_x, 2) + math.pow(distance_y, 2) + math.pow(distance_z, 2))
    rospy.loginfo("Reached {}. Distance {}.".format(distance <= threshold, distance))

    return distance <= threshold

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

    rate = rospy.Rate(20)

    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    home_position = Vector3()
    target_position = Vector3()

    home_position.x = drone_odom.pose.pose.position.x
    home_position.y = drone_odom.pose.pose.position.y
    home_position.z = drone_odom.pose.pose.position.z
    velocity_control(home_position.x, home_position.y, home_position.z)

    velocity_for_publish.linear.x = velocity.x
    velocity_for_publish.linear.y = velocity.y
    velocity_for_publish.linear.z = velocity.z
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

    hover_cnt = 0
    mission_step = 0
    home_position_get = False

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
                        if home_position_get == False:
                            home_position.x = drone_odom.pose.pose.position.x
                            home_position.y = drone_odom.pose.pose.position.y
                            home_position.z = drone_odom.pose.pose.position.z
                            home_position_get = True
                            rospy.loginfo("Home position get: x: {}, y: {}, z: {}".format(
                                home_position.x, home_position.y, home_position.z))
                        
                        mission_step = 1
                    last_req = rospy.Time.now()
            
            elif mission_step == 1:
                rospy.logwarn_once("Mission Step 1. TAKEOFF and HOVERING.")
                target_position.x = home_position.x
                target_position.y = home_position.y
                target_position.z = home_position.z + 3.0

                velocity_control(target_position.x, target_position.y, target_position.z)

                velocity_for_publish.linear.x = velocity.x
                velocity_for_publish.linear.y = velocity.y
                velocity_for_publish.linear.z = velocity.z
                velocity_for_publish.angular.x = 0
                velocity_for_publish.angular.y = 0
                velocity_for_publish.angular.z = 0

                if if_reached(target_position.x, target_position.y, target_position.z, threshold=0.5) == True:
                    rospy.loginfo("Target reached. Current position: x: {} y: {} z: {}".format(
                        drone_odom.pose.pose.position.x, drone_odom.pose.pose.position.y, drone_odom.pose.pose.position.z))
                    if hover_cnt == 150:
                        mission_step = 2
                        velocity_control(home_position.x, home_position.y, home_position.z)

                        velocity_for_publish.linear.x = velocity.x
                        velocity_for_publish.linear.y = velocity.y
                        velocity_for_publish.linear.z = velocity.z
                        velocity_for_publish.angular.x = 0
                        velocity_for_publish.angular.y = 0
                        velocity_for_publish.angular.z = 0

                        last_req = rospy.Time.now()

                    hover_cnt += 1
                
                rospy.loginfo_once("Target Position: x: {} y: {} z: {}".format(
                    target_position.x, target_position.y, target_position.z))
            
            elif mission_step == 2:
                rospy.logwarn_once("Mission Step 2. LANDING.")
                offb_set_mode.custom_mode = "AUTO.LAND"
                if (current_state.mode != "AUTO.LAND" and (rospy.Time.now() - last_req) > rospy.Duration(1.5)):
                    if (set_mode_client.call(offb_set_mode).mode_sent):
                        rospy.loginfo("AUTO.LAND enabled")
                        mission_step = 3
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


    