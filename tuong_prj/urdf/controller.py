#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import termios
import tty

# Khởi tạo ROS node
rospy.init_node('teleop_control', anonymous=True)

# Publisher cho robot di chuyển
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# Publisher cho khớp cánh tay
arm_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)

# Tần số gửi lệnh
rate = rospy.Rate(10)

# Cấu hình tốc độ
speed = 0.5  # Tốc độ tiến/lùi
turn = 1.0    # Tốc độ xoay
joint_speed = 0.2  # Bước thay đổi góc khớp

# Trạng thái góc của các khớp
joint_angles = {"link1_joint": 0.0, "link2_joint": 0.0}

# Hàm đọc phím từ bàn phím
def getKey():
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    return key

# Lưu trạng thái terminal
old_settings = termios.tcgetattr(sys.stdin)

# Hướng dẫn điều khiển
print("""
Điều khiển robot 4 bánh:
W - Tiến
S - Lùi
A - Quay trái
D - Quay phải

Điều khiển cánh tay:
I - Tăng góc Link1
K - Giảm góc Link1
J - Tăng góc Link2
L - Giảm góc Link2

Q - Thoát
""")

try:
    while not rospy.is_shutdown():
        key = getKey()
        twist = Twist()
        
        # Điều khiển di chuyển
        if key == 'w':  # Tiến
            twist.linear.x = speed
        elif key == 's':  # Lùi
            twist.linear.x = -speed
        elif key == 'a':  # Quay trái
            twist.angular.z = turn
        elif key == 'd':  # Quay phải
            twist.angular.z = -turn
        elif key == 'q':  # Thoát chương trình
            break
        cmd_vel_pub.publish(twist)

        # Điều khiển cánh tay robot
        if key == 'i':  # Tăng góc link1_joint
            joint_angles["link1_joint"] += joint_speed
        elif key == 'k':  # Giảm góc link1_joint
            joint_angles["link1_joint"] -= joint_speed
        elif key == 'j':  # Tăng góc link2_joint
            joint_angles["link2_joint"] += joint_speed
        elif key == 'l':  # Giảm góc link2_joint
            joint_angles["link2_joint"] -= joint_speed

        # Gửi lệnh điều khiển joints
        joint_msg = JointTrajectory()
        joint_msg.joint_names = ["link1_joint", "link2_joint"]
        point = JointTrajectoryPoint()
        point.positions = [joint_angles["link1_joint"], joint_angles["link2_joint"]]
        point.time_from_start = rospy.Duration(1.0)
        joint_msg.points.append(point)
        arm_pub.publish(joint_msg)

        rate.sleep()

except Exception as e:
    print(e)

finally:
    # Dừng robot trước khi thoát
    twist = Twist()
    cmd_vel_pub.publish(twist)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
