import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from pylimo import limo

class ApplyControl(Node):
  def __init__(self, limo_instance):
    super().__init__('apply_control')
    self.limo = limo_instance
    self.linear_velocity = 0.0
    self.steering_angle = 0.0
    self.linear_velocity_subscriber = self.create_subscription(
        Float32, "/linear_velocity", self.velocity_callback, 10)
    self.steering_angle_subscriber = self.create_subscription(
        Float32, "/steering_angle", self.steering_callback, 10)

    # Timer that calls the update method every 0.1 seconds (or any desired interval)
    self.timer = self.create_timer(0.1, self.update)

  def velocity_callback(self, msg):
    print("---------------------------------")
    print("setting linear velocity to: ", msg.data, " m/s")
    self.linear_velocity = msg.data

  def steering_callback(self, msg):
    print("setting steering angle to: ", msg.data, " rad")
    self.steering_angle = msg.data

  def update(self):
    # This method is called periodically by the timer
    self.send_motion_command()
    self.print_info()

  def send_motion_command(self):
    self.limo.SetMotionCommand(
      linear_vel = self.linear_velocity, 
      angular_vel = 0, 
      lateral_velocity = 0, 
      steering_angle = self.steering_angle)
    
  def print_info(self):
    data1 = self.limo.GetLinearVelocity()
    data2 = self.limo.GetAngularVelocity()
    data3 = self.limo.GetSteeringAngle()
    data4 = self.limo.GetLateralVelocity()
    data5 = self.limo.GetControlMode()
    data6 = self.limo.GetBatteryVoltage()
    data7 = self.limo.GetErrorCode()
    data8 = self.limo.GetIMUAccelData()
    data9 = self.limo.GetIMUGyroData()
    print("linear velocity: " + str(data1))
    print("angular velocity: " + str(data2))
    print("steering angle: " + str(data3))  
    print("lateral velocity: " + str(data4))
    print("control mode: " + str(data5))
    print("battery voltage: " + str(data7))
    print("error code: " + str(data8))
    print("data from accelerometer: " + str(data9))
    print("gyroscope data: " + str(data10))
    
    
def main(args=None):
  rclpy.init(args=args)

  limo_instance = limo.LIMO()
  limo_instance.EnableCommand()

  apply_control_node = ApplyControl(limo_instance)
  print("\nThe commands are being applied to the vehicle!\n")
  rclpy.spin(apply_control_node)

  apply_control_node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()