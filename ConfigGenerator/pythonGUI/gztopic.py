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

class GzCommunicator(object):
    def __init__(self,masterip = '127.0.0.1',masterport = 11345):
        """Init function"""
        self.MASTER_TCP_IP   = masterip
        self.MASTER_TCP_PORT = masterport

        self.NODE_TCP_IP     = '127.0.0.1'
        self.NODE_TCP_PORT   = 11451

        self.TCP_BUFFER_SIZE = 40960

    def StartCommunicator(self):
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
        pub.topic     = "/gazebo/Configuration/configSubscriber"
        pub.msg_type  = 'config_message.msgs.ConfigMessage'
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

    def stop(self):
        """Stop the communicator"""
        self.s_reg.close()

    def publish(self,msg):
        """Function used to send 2D velocity command to gztopic"""
        # Pack Data for Reply

        # Publish Packet to Subscriber
        pk_pub            = Packet()
        pk_pub.stamp.sec  = int(time.time())
        pk_pub.stamp.nsec = datetime.now().microsecond
        pk_pub.type       = Vector2d.DESCRIPTOR.full_name
        pk_pub.serialized_data = msg.SerializeToString()

        self.conn.send(hex(msg.ByteSize()).rjust(8))
        self.conn.send(msg.SerializeToString())

        time.sleep(0.1)