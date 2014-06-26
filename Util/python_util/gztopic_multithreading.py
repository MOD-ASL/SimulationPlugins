import socket
import time
import sys
import threading
import select
# import copy
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

class GzPublisher:
  def __init__(self,topic,messagetype,GzCommunicator):
    self.TOPIC = topic
    self.MESSAGETYPE = messagetype
    self.GZCOMMUNICATOR = GzCommunicator

  def Publish(self,msg):
    self.GZCOMMUNICATOR.Publish(self.TOPIC, msg)

class GzSubscriber:
  def __init__(self, topic, messagetype, callback):
    self.TOPIC = topic
    self.MESSAGETYPE = messagetype
    self.CALLBACK = callback

class GzCommunicator(threading.Thread):
  def __init__(self,masterip = '127.0.0.1',masterport = 11345,selfport = 11451):
    threading.Thread.__init__(self)
    """Initiate server"""
    self.MASTER_TCP_IP   = masterip
    self.MASTER_TCP_PORT = masterport

    self.NODE_TCP_IP     = ''
    self.NODE_TCP_PORT   = selfport

    self.TCP_BUFFER_SIZE = 4096

    self.outputs = []
    self.clientmap = {}
    # self.subscribers = []

    self.runserver = 1

    self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    self.server.setblocking(0)
    self.server.bind((self.NODE_TCP_IP, self.NODE_TCP_PORT))
    self.server.listen(5)

  def run(self):
    inputs = [self.server]   
    while self.runserver:
      try:
        # Setting 1 second timeout
        inputready,outputready,exceptready = select.select(inputs, self.outputs, [], 1)
      except select.error, e:
        print "Suffering a select error"
        break
      except socket.error, e:
        print "Suffering a socket error"
        break
      # Running comunication management
      for s in inputready:
        if s == self.server:
          client, address = s.accept()
          message = client.recv(self.TCP_BUFFER_SIZE)
          pk_sub = Packet()
          pk_sub.ParseFromString(message[8:])
          print "Packet:\n", pk_sub
          sub = Subscribe()
          sub.ParseFromString(pk_sub.serialized_data)
          print "Sub:\n", sub
          client.setblocking(0)
          self.outputs.append(client)
          inputs.append(client)
          self.clientmap[sub.topic] = client
        else:
          print "Receive information from others"
          # handle all other sockets
          try:
            message = s.recv(self.TCP_BUFFER_SIZE)
            if not message:             
              print 'Client with topic "',self.WhichCLient(self.clientmap,s),'" hung up'
              s.close()
              inputs.remove(s)
              self.outputs.remove(s)
              break
            else:
              print "Other message: ", message
                   
          except socket.error, e:
            print 'Client with topic "',self.WhichCLient(self.clientmap,s), \
                '" Suffering socket error'
            # Remove
            inputs.remove(s)
            self.outputs.remove(s)
            break
      time.sleep(0.001)
    for eachsocket in inputs:
      eachsocket.close()
    # self.server.close()

  def WhichCLient(self, dic, avalue):
    for keystr, val in dic.iteritems():
      if avalue == val:
        return keystr
    return ''

  def CreatePulisher(self, topic, messagetype):
    # print "Dictionary len: ",len(self.clientmap)
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

    s_reg = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s_reg.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    timeout = time.time() + 60
    has_waited = False
    while time.time() < timeout:
      try:
        s_reg.connect((self.MASTER_TCP_IP, self.MASTER_TCP_PORT))
      except Exception, e:
        print 'Cannot connect to master, retrying ...'
        has_waited = True
      else:
        if has_waited:
          time.sleep(1)
        s_reg.sendall(hex(pk.ByteSize()).rjust(8)+pk.SerializeToString())
        print "Finish sending"
        break
      time.sleep(1)
    else:
      print 'Failed to connect to master, abort!'
    s_reg.close()
    return GzPublisher(topic,messagetype,self)

  # def CreateSubscriber(self, topic, messagetype, callbackhandle):
  #   # Register as a Subscriber with Gazebo
  #   pk            = Packet()
  #   pk.stamp.sec  = int(time.time())
  #   pk.stamp.nsec = datetime.now().microsecond
  #   pk.type       = "subscribe"

  #   sub           = Subscribe()
  #   sub.topic     = topic       #"/gazebo/Configuration/configSubscriber"
  #   sub.msg_type  = messagetype #'config_message.msgs.ConfigMessage'
  #   sub.host      = '158.130.72.168'
  #   sub.port      = self.NODE_TCP_PORT

  #   pk.serialized_data = sub.SerializeToString()

  #   s_reg = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  #   s_reg.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
  #   timeout = time.time() + 60

  #   while time.time() < timeout:
  #     try:
  #       s_reg.connect((self.MASTER_TCP_IP, self.MASTER_TCP_PORT))
  #     except Exception, e:
  #       print 'Cannot connect to master, retrying ...'
  #     else:
  #       # s_reg.send(hex(pk.ByteSize()).rjust(8))
  #       # s_reg.send(pk.SerializeToString())
  #       s_reg.sendall(hex(pk.ByteSize()).rjust(8)+pk.SerializeToString())
  #       break
  #     time.sleep(1)
  #   else:
  #     print 'Failed to connect to master, abort!'

  #   s_reg.close()
  #   newsubscriber = GzSubscriber(topic, messagetype, callbackhandle)
  #   self.subscribers.append(newsubscriber)
  #   # return GzPublisher(topic,messagetype,self)

  def Close(self):
    self.runserver = 0

  def Publish(self, topic, msg):
    self.clientmap[topic].send(hex(msg.ByteSize()).rjust(8))
    self.clientmap[topic].sendall(msg.SerializeToString())

  # def FindSubscriber(self, topic):
  #   for eachsub in self.subscribers:
  #     if eachsub.TOPIC == topic:
  #       return eachsub