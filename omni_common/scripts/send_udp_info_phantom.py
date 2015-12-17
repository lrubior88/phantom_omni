#! /usr/bin/env python
import rospy, time, math, os
# Sockets
import socket, struct
# Messages
from StringIO import StringIO
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, TwistStamped
from omni_msgs.msg import OmniState, OmniFeedback, OmniButtonEvent

_struct_10d = struct.Struct("<10d")
_struct_2d = struct.Struct("<2d")

class Send_udp_info:
  def __init__(self):
      
    # Read parameters
    self.write_ip = self.read_parameter('~write_ip', '127.0.0.1')
    self.write_port_pose = self.read_parameter('~write_port_pose', '34900')
    self.write_port_button = self.read_parameter('~write_port_button', '34901')
    rospy.logwarn('write port button: %s' % (self.write_port_button))
    self.publish_rate = self.read_parameter('~publish_rate', '1000.0')
    self.omni_name = self.read_parameter('~omni_name', 'phantom')
    self.pose_topic = "/%s/state" % self.omni_name
    self.button_topic = "/%s/button" % self.omni_name
      
    # Setup Subscriber
    rospy.Subscriber(self.pose_topic, OmniState, self.omni_command_cb)
    self.omni_msg = OmniState()
    rospy.Subscriber(self.button_topic, OmniButtonEvent, self.button_command_cb)
    self.button_msg = OmniButtonEvent()
    self.button_msg.grey_button = 0.0
    self.button_msg.white_button = 0.0

    # Set up write socket
    self.write_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    self.timer = rospy.Timer(rospy.Duration(1.0/self.publish_rate), self.send_udp)
    rospy.spin()

  def omni_command_cb(self, msg):
    self.omni_msg = msg

  def button_command_cb(self, msg):
    self.button_msg = msg

  def send_udp(self, event):
    try:
        # Serialize a OmniStamp msg
        buff = StringIO()
        pos_x = self.omni_msg.pose.position.x
        pos_y = self.omni_msg.pose.position.y 
        pos_z = self.omni_msg.pose.position.z 
        rot_x = self.omni_msg.pose.orientation.x
        rot_y = self.omni_msg.pose.orientation.y
        rot_z = self.omni_msg.pose.orientation.z
        rot_w = self.omni_msg.pose.orientation.w
        vel_x = self.omni_msg.velocity.x
        vel_y = self.omni_msg.velocity.y 
        vel_z = self.omni_msg.velocity.z
        
        buff.write(_struct_10d.pack(pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, rot_w, vel_x, vel_y, vel_z))

        # Serialize Button msg
        buff2 = StringIO()
        button_1 = self.button_msg.grey_button
        button_2 = self.button_msg.white_button
        buff2.write(_struct_2d.pack(button_1, button_2))

        # Send values
        self.write_socket.sendto(buff.getvalue(), (self.write_ip, self.write_port_pose))
        self.write_socket.sendto(buff2.getvalue(), (self.write_ip, self.write_port_button))
   
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)
    except socket.error, e:
        result = self.write_socket.bind((self.write_ip,self.write_port_pose))
        if result:
            rospy.logwarn('Connection refused1')
        result = self.write_socket.bind((self.write_ip,self.write_port_button))
        if result:
            rospy.logwarn('Connection refused2')
  def read_parameter(self, name, default):
    if not rospy.has_param(name):
      rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
    return rospy.get_param(name, default)


if __name__ == '__main__':
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  interface = Send_udp_info()
