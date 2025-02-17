# control.py
# PID Control and Inverse Kinematics for UR-3 Robotic Arm
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, setpoint, measured_value):
        error = setpoint - measured_value
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

# ROS Node for Joint Control
rospy.init_node('ur3_joint_controller')
control_pub = rospy.Publisher('/ur3/joint_position_controller/command', Float64, queue_size=10)

pid = PIDController(1.0, 0.1, 0.05)
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    setpoint = 1.5  # Target joint position (example value)
    measured_value = 1.0  # Simulated joint feedback
    control_signal = pid.compute(setpoint, measured_value)
    control_pub.publish(control_signal)
    rate.sleep()
