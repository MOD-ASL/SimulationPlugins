## @package gztopic Defines classes used for communication using the same thread
import socket
import time
import sys
import re
import threading
import copy
from datetime import datetime

from pygazebo.msg.packet_pb2     import Packet
from pygazebo.msg.publish_pb2    import Publish
from pygazebo.msg.request_pb2    import Request
from pygazebo.msg.response_pb2   import Response
from pygazebo.msg.pose_pb2       import Pose
from pygazebo.msg.vector2d_pb2   import Vector2d
from pygazebo.msg.vector3d_pb2   import Vector3d
from pygazebo.msg.quaternion_pb2 import Quaternion
from pygazebo.msg.model_pb2      import Model
from pygazebo.msg.joint_pb2      import Joint
from pygazebo.msg.subscribe_pb2  import Subscribe
## Gztopic pulisher object
class GzCommunicator(object):
  ## Constructor 
  # @param self Object pointer
  # @param masterip Master ip address
  # @param masterport Master port number
  def __init__(self,masterip = '127.0.0.1',masterport = 11345):
    """Init function"""
    self.MASTER_TCP_IP   = masterip
    self.MASTER_TCP_PORT = masterport

    self.NODE_TCP_IP     = ''
    self.NODE_TCP_PORT   = 11451

    self.TCP_BUFFER_SIZE = 40960
  ## Build the socket and start the connection
  # @param self Object pointer
  # @param topic Gztopic string
  # @param messagetype Name of the messgae object
  def StartCommunicator(self,topic,messagetype):
    """Start the communication through gztopic"""
    # Listen for Subscribers
    s_sub = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s_sub.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s_sub.bind((self.NODE_TCP_IP, self.NODE_TCP_PORT))
    s_sub.listen(5)


    # Register as a Publisher with Gazebo
    pk            = Packet()
    pk.stamp.sec  = int(time.time())
    pk.stamp.nsec = datetime.now().microsecond
    pk.type       = "advertise"

    pub           = Publish()
    pub.topic     = topic       #"/gazebo/Configuration/configSubscriber"
    pub.msg_type  = messagetype #'config_message.msgs.ConfigMessage'
    pub.host      = self.NODE_TCP_IP
    pub.port      = self.NODE_TCP_PORT

    pk.serialized_data = pub.SerializeToString()

    self.s_reg = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.s_reg.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    self.s_reg.connect((self.MASTER_TCP_IP, self.MASTER_TCP_PORT))
    self.s_reg.send(hex(pk.ByteSize()).rjust(8))
    self.s_reg.send(pk.SerializeToString())

    print "wating for reply"
    # Respond to a subscriber
    try:
        self.conn, address = s_sub.accept()
        data = self.conn.recv(self.TCP_BUFFER_SIZE)

    except Exception as e:
        print "Cannot connect to the server."
        print e
        print

    else:
        print "wating for reply"
        # Decode Incomming Packet
        pk_sub = Packet()
        pk_sub.ParseFromString(data[8:])
        print "Packet:\n", pk_sub

        # Decode Subscription Request
        sub = Subscribe()
        sub.ParseFromString(pk_sub.serialized_data)
        print "Sub:\n", sub
        print
  ## Close the connection
  # @param self Object pointer
  def stop(self):
    """Stop the communicator"""
    self.s_reg.close()
    self.conn.close()
  ## Publish message
  # @param self Object pointer
  # @param msg Message object
  def publish(self,msg):

    self.conn.send(hex(msg.ByteSize()).rjust(8))
    self.conn.send(msg.SerializeToString())

    time.sleep(0.1)