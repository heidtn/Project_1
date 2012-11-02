from joy import *
from socket import socket, AF_INET, SOCK_STREAM, error as SocketError
from json import loads as json_loads
import ckbot.logical as L
import math

wheel_radius = 0.0619 # meter
rad_per_click = math.pi/3000
robot_width = 0.149 # meter

class SensorPlan( Plan ):
  """
  SensorPlan is a concrete Plan subclass that uses the self.app's
  remote plan to read and decode WayPoint Task sensor and waypoint
  updates.
  """
  def __init__( self, app, peer, *arg, **kw ):
    Plan.__init__(self, app, *arg, **kw )
    self.sock = None
    self.peer = peer
    self.message_received = False
    self.r = self.robot
    self.estimator = state_estimator(self.r.at.LEFT.get_pos(), self.r.at.RIGHT.get_pos())

  def _connect( self ):
    s = socket(AF_INET, SOCK_STREAM)
    try:
       s.connect( self.peer )
    except SocketError, se:
       progress("Failed to connect: "+str(se))
       return
    s.setblocking(0)
    self.sock = s

  def stop( self ):
    if self.sock is not None:
      self.sock.close()
    self.sock = None

  def behavior( self ):
    while True:
      # if not connected --> try to connect
      if self.sock is None:
        self._connect()
      # if not connected --> sleep for a bit
      if self.sock is None:
        yield self.forDuration(0.1)
        continue
      # receive an update / skip
      try:
        msg = self.sock.recv(1024)
      except SocketError, se:
        # If there was no data on the socket --> not a real error, else
        if se.errno != 11:
          progress("Connection failed: "+str(se))
          self.sock.close()
          self.sock = None
        yield
        continue
      ts = self.app.now
      dic = json_loads(msg)
      assert type(dic) is dict
      dic = dic.items()
      dic.sort()
      progress("Message received at: " + str(ts))
      for k,v in dic:
        progress("   %s : %s" % (k,repr(v)))
	
      if('b' and 'f' in dic):
        self.lpos = self.r.at.LEFT.get_pos()
        self.rpos = self.r.at.RIGHT.get_pos()
        self.estimator.estimate_state(dic('b'), dic('f'), self.lpos, self.rpos, dic('w'))
      else:
				yield
				continue

      self.message_received = True
      yield self.forDuration(0.3)


class state_estimator:
	def __init__(self, initial_L, initial_R):
		self.angle_est = math.pi
		self.L_prev = initial_L
		self.R_prev = initial_R
		self.angle_prev = math.pi
		self.x = 0
		self.y = 0
		self.theta = 0

	def estimate_state(self, b, f, encoder_L, encoder_R, ways):
		self.delta_L = encoder_L - self.L_prev
		self.delta_R = encoder_R - sefl.R_prev
		self.L_prev = encoder_L
		self.R_prev = encoder_R

		self.angle_prev = self.angle_est
		
		
		self.delta_x = math.cos(self.theta)*(self.delta_L+self.delta_R)*rad_per_click*wheel_radius/2
		self.delta_y = math.sin(self.theta)*(self.delta_L+self.delta_R)*rad_per_click*wheel_radius/2
		self.delta_theta = math.atan((self.delta_R-self.delta_L)*rad_per_click*wheel_radius/robot_width)

		self.x = self.x + self.delta_x
		self.y = self.y + self.delta_y
		self.theta = self.theta + self.delta_theta


class encoder_plan( Plan ):
  def __init__(self, *arg, **kw):
    Plan.__init__(self,*arg,**kw)
    self.r = self.robot
    self.message_received = False
    
  def onStart( self ): 
    self.L_prev = self.r.at.LEFT.get_pos()
    self.R_prev = self.r.at.RIGHT.get_pos() 

  def behavior( self ):
    while True:
        self.L = self.r.at.LEFT.get_pos()
        self.R = self.r.at.RIGHT.get_pos()
	
        progress("L: %s  , R: %s" % (self.L, self.R))
        yield self.forDuration(0.3)
  

  
class navigator( Plan ):
  def __init__(self, *arg, **kw):
    Plan.__init__(self,*arg,**kw)
    self.r = self.robot
  
  def onStart( self ):
    self.r.at.LEFT.set_torque(0)
    self.r.at.RIGHT.set_torque(0)

  def behavior( self ):
    while 1:
      self.dummy = 1
  def onEvent( self, evt):
    self.dummy = 1



class Joy_interface( JoyApp ):
  
  def __init__(self,spec,*arg,**kw): 
    L.DEFAULT_PORT = "/dev/ttyACM0"
    JoyApp.__init__(self, robot = {'count':3}, *arg,**kw)
    self.teleop = True
    # JoyApp.__init__(self, *arg, **kw)
  def onStart( self ):
    # Set up the sensor receiver plan
    self.sensor = SensorPlan(self,("67.194.202.70",8080), robot = self.robot)
    self.sensor.start()
    # self.encoder = encoder_plan(self, robot = self.robot)
    # self.encoder.start()
    
  def onEvent( self, evt ):
    
    if(evt.type == MIDIEVENT):
      #teleoperation commands
      if(self.teleop == True):
        if(evt.kind=='slider' and evt.index==1):
          if(evt.value < 66 and evt.value > 60):
            self.robot.at.LEFT.set_torque(0)
          else:
            self.robot.at.LEFT.set_torque((evt.value - 127.0/2)/(127.0/2))
        elif(evt.kind=='slider' and evt.index==2):
          if(evt.value < 66 and evt.value > 60):
            self.robot.at.RIGHT.set_torque(0)
          else:
            self.robot.at.RIGHT.set_torque(-1*(evt.value - 127.0/2)/(127.0/2))
        elif(evt.kind=='knob' and evt.index==1):
          self.robot.at.LASER.set_torque((evt.value - 63.5)/63.5)
      if(evt.kind == 'play' and evt.value == 127):
        self.teleop = not self.teleop 

    elif(evt.type == TIMEREVENT):
      #every time tick, navigate
      if(self.sensor.message_received):
        self.sensor.message_received = False  
    return JoyApp.onEvent(self, evt)
    
  
  def onStop( self ):
    self.robot.at.LEFT.set_torque(0)
    self.robot.at.RIGHT.set_torque(0)
    self.robot.at.LASER.set_torque(0)
    self.sensor.stop()
    # self.encoder.stop()
    return super( Joy_interface, self ).onStop()
      
if __name__=="__main__":
  print """
  Running the waypoint sensor demo
  
  Connects to waypoint application and reads sensor.
  
  The waypoint sensor send JSON maps with keys:
  'f', 'b' : front and back sensor values
  'w' : list of lists. Each sub-list is of length 2. List of waypoint
    coordinates, including the next waypoint. Each time the next 
    waypoint changes, it means the previous waypoint was reached.
  """
  app=Joy_interface("#output ")
  app.run()

