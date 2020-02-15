import rospy
from geometry_msgs import Quaternion
from std_msgs.msg import Float64
import math

class QuaternionConverter():
  def init(self):
    self.pitch = 0
    self.roll = 0
    self.yaw = 0

    rospy.Subscriber("/rov/sensor/yaw", Float64, yaw_update)
    rospy.Subscriber("/rov/sensor/roll", Float64, roll_update)
    rospy.Subscriber("/rov/sensor/pitch", Float64, pitch_update)

    converter_pub = rospy.Publisher('/quaternion', Quaternion, queue_size=5)

    rospy.init_node('quaternion_converter', anonymous = True) #anonymous = True simply makes sure the node name is unique

    r = rospy.Rate(30)

    while not rospy.is_shutdown():


      cy = math.cos(self.yaw * 0.5);
      sy = math.sin(self.yaw * 0.5);
      cp = math.cos(self.pitch * 0.5);
      sp = math.sin(self.pitch * 0.5);
      cr = math.cos(self.roll * 0.5);
      sr = math.sin(self.roll * 0.5);

      q = Quaternion()

      q.w = cy * cp * cr + sy * sp * sr;
      q.x = cy * cp * sr - sy * sp * cr;
      q.y = sy * cp * sr + cy * sp * cr;
      q.z = sy * cp * cr - cy * sp * sr;

      converter_pub.publish(q)
      r.sleep()


  def yaw_update(self, data):
    self.yaw = data.data

  def roll_update(self, data):
    self.roll = data.data

  def pitch_update(self, data):
    self.pitch = data.data








if __name__ == '__main__':
  try:
    QuaternionConverter()
  except rospy.ROSInterruptException:
    pass
