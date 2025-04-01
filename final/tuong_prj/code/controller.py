#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import pygame

# Khởi tạo pygame
pygame.init()
win = pygame.display.set_mode((400, 400))
pygame.display.set_caption("Keyboard Control")
font = pygame.font.Font(None, 24)

# Khởi tạo node ROS
rospy.init_node("two_dof_arm_controller")

# Tạo Publisher cho các bộ điều khiển
first_motor_pub = rospy.Publisher("/link1_joint_controller/command", Float64, queue_size=10)
second_motor_pub = rospy.Publisher("/link2_joint_controller/command", Float64, queue_size=10)
gripper_control_pub = rospy.Publisher("/gripper_control_controller/command", Float64, queue_size=10)
gripper_left_pub = rospy.Publisher("/gripper_left_controller/command", Float64, queue_size=10)

# Tạo Publisher cho điều khiển bánh xe
cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

# Tần số gửi lệnh
rate = rospy.Rate(10)

# Giá trị vận tốc
linear_speed = 2.0  # Tốc độ tiến/lùi
angular_speed = 2.0  # Tốc độ quay

# Danh sách Publisher cho gripper với trạng thái đảo chiều
gripper_pubs = [(gripper_control_pub, False), (gripper_left_pub, True)]

# Khởi tạo vị trí hiện tại
current1, current2, current_gripper = 0.0, 0.0, 0.0

def smooth_move(pub_list, current, target, steps=100, rate=50):
    delta = (target - current) / steps
    for _ in range(steps):
        current += delta
        msg = Float64()
        for pub, inv in pub_list:
            msg.data = -current if inv else current
            pub.publish(msg)
        rospy.sleep(1.0 / rate)
    return current

def get_key_input():
    twist = Twist()
    keys = pygame.key.get_pressed()

    # Điều khiển bánh xe (đã đảo ngược w/s, a/d)
    if keys[pygame.K_s]:
        twist.linear.x = linear_speed
    elif keys[pygame.K_w]:
        twist.linear.x = -linear_speed
    if keys[pygame.K_d]:
        twist.angular.z = angular_speed
    elif keys[pygame.K_a]:
        twist.angular.z = -angular_speed

    return twist

def draw_controls():
    win.fill((30, 30, 30))
    controls = [
        "W - Move Backward", "S - Move Forward", "A - Turn Right", "D - Turn Left",
        "E - Arm 1 Up", "Q - Arm 1 Down", "X - Arm 2 Up", "C - Arm 2 Down",
        "R - Open Gripper", "F - Close Gripper"
    ]
    y = 20
    for text in controls:
        text_surf = font.render(text, True, (255, 255, 255))
        win.blit(text_surf, (20, y))
        y += 30
    pygame.display.flip()

def main():
    global current1, current2, current_gripper
    running = True
    while running and not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        draw_controls()

        # Điều khiển bánh xe
        twist = get_key_input()
        cmd_vel_pub.publish(twist)

        # Điều khiển tay máy
        keys = pygame.key.get_pressed()
        if keys[pygame.K_e]:
            current1 = smooth_move([(first_motor_pub, False)], current1, current1 + 0.1)
        elif keys[pygame.K_q]:
            current1 = smooth_move([(first_motor_pub, False)], current1, current1 - 0.1)

        if keys[pygame.K_x]:
            current2 = smooth_move([(second_motor_pub, False)], current2, current2 + 0.1)
        elif keys[pygame.K_c]:
            current2 = smooth_move([(second_motor_pub, False)], current2, current2 - 0.1)

        if keys[pygame.K_r]:
            current_gripper = smooth_move(gripper_pubs, current_gripper, current_gripper + 0.1)
        elif keys[pygame.K_f]:
            current_gripper = smooth_move(gripper_pubs, current_gripper, current_gripper - 0.1)

        rate.sleep()

    pygame.quit()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
