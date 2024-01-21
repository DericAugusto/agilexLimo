import rclpy
from pylimo import limo
import time
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String

limo = limo.LIMO()
limo.EnableCommand()

class DonneesVoiture(Node):
    def __init__(self):
        super().__init__('camera')
        self.publisher_ = self.create_publisher(String, 'données voiture', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0


    def timer_callback(self):
        msg = Float64MultiArray()


        data1 = limo.GetLinearVelocity()
        data2 = limo.GetAngularVelocity()
        data3 = limo.GetSteeringAngle()
        data4 = limo.GetLateralVelocity()
        data5 = limo.GetControlMode()
        #data6 = limo.GetLeftWheelOdom() # ça ne marche pas
        data7 = limo.GetBatteryVoltage()
        data8 = limo.GetErrorCode()
        data9 = limo.GetIMUAccelData()
        data10 = limo.GetIMUGyroData()
        #data11 = limo.GetIMUYaw() # ça ne marche pas
        #data12 = limo.GetIMUPitch() # ça ne marche pas
        #data13 = limo.GetIMURoll() # ça ne marche pas
        data = [data1,data2,data3,data4,data5,data7,data8,data9,data10]
        msg.data = data
        self.publisher_.publish(msg)
        donnees = DonneesVoiture()
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    rclpy.spin(donnees)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
