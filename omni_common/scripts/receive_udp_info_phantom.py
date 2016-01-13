#! /usr/bin/env python
import rospy, time, math, os
import numpy as np
# Sockets
import socket, struct
# Messages
from StringIO import StringIO
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from omni_msgs.msg import OmniState, OmniFeedback, OmniButtonEvent

import roslib.message

_struct_d3 = struct.Struct("<3d")


class Receive_udp_info:
  def __init__(self):
      
    # Read parameters
    self.read_ip = self.read_parameter('~read_ip', '127.0.0.1')
    self.read_port = self.read_parameter('~read_port', '34900')
    self.publish_rate = self.read_parameter('~publish_rate', '1000.0')
    self.frame_id = self.read_parameter('~frame_id', 'world')
    self.omni_name = self.read_parameter('~omni_name', 'phantom')
    self.force_topic = "/%s/force_feedback" % self.omni_name
    
    self.force_topic_pub = rospy.Publisher(self.force_topic, OmniFeedback)

    # Setup read socket
    self.read_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.read_socket.bind((self.read_ip, self.read_port))
    rospy.loginfo('UDP Socket listening on port [%d]' % (self.read_port))

    # Register rospy shutdown hook
    rospy.on_shutdown(self.shutdown_hook)

    while True:
      try:
        data,addr=self.read_socket.recvfrom(1024)
        if data:
            forces = _struct_d3.unpack(data[0:24])
            
            f_msg = OmniFeedback()
            f_msg.force.x = forces[0]
            f_msg.force.y = forces[1]
            f_msg.force.z = forces[2]
            f_msg.position.x = 0.0
            f_msg.position.y = 0.0
            f_msg.position.z = 0.0
            
            self.force_topic_pub.publish(f_msg)
      except:
        pass
                
                
  def read_parameter(self, name, default):
    if not rospy.has_param(name):
      rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
    return rospy.get_param(name, default)
        
  #~ def loginfo(self, msg):
    #~ rospy.logwarn(self.colors.OKBLUE + str(msg) + self.colors.ENDC)

  def shutdown_hook(self):
    # Do some cleaning depending on the app
    self.read_socket.close()
    pass

if __name__ == '__main__':
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  interface = Receive_udp_info()
