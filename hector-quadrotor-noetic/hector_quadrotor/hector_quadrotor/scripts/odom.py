import rospy
from sensor_msgs.msg import Imu
v_x = 0
v_y = 0
v_z = 0
x = 0
y = 0
z = 0
def imu_callback(data):
    global v_x
    global v_y
    global v_z
    global x
    global y
    global z    
    
    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    dt = (current_time - last_time).to_sec()
    dt = 0.01
    v_x = v_x + data.linear_acceleration.x * dt
    v_y = v_y + data.linear_acceleration.y * dt
    v_z = v_z + (data.linear_acceleration.z - 9.83) * dt

    x = x + v_x * dt
    y = y + v_y * dt
    z = z + v_z * dt

    print(x)

    
    

if __name__ == '__main__': 
  try:
    rospy.init_node('odom_node', anonymous=True)
    imu_sub = rospy.Subscriber("/raw_imu",Imu,imu_callback)
    
    #drive_pub = rospy.Publisher('/tianracer/ackermann_cmd', AckermannDrive, queue_size=1)
    rospy.spin()
  except rospy.ROSInterruptException:
    pass


