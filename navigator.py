from joy import *
from socket import socket, AF_INET, SOCK_STREAM, error as SocketError
from json import loads as json_loads
import ckbot.logical as L
import math
import PD
import getopt

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
    self.navigator = navigator(self.robot)    
    self.autonomy = False
    self.sensor_enable = True

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
      if(self.sensor_enable):
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
        
        try: 
          dic = json_loads(msg)
        except ValueError:
          continue
        assert type(dic) is dict
        # dic = dic.items()
        # dic.sort()
        # progress("Message received at: " + str(ts))
        # for k,v in dic:
        #  progress("   %s : %s" % (k,repr(v)))
        
        # dic = {'b':255, 'f':255}      
        self.lpos = self.r.at.LEFT.get_pos()
        self.rpos = self.r.at.RIGHT.get_pos()
        # self.estimator.estimate_state(dic['b'], dic['f'], self.lpos, self.rpos, [])
        progress(repr(dic))      
      if(self.autonomy):
        if(not self.sensor_enable):
          dic = {'b':255, 'f':255}
        self.navigator.navigate(self.estimator, dic['b'], dic['f'])
          
       
      yield self.forDuration(0.3)
 
  def set_pgain(self, setter):
    self.navigator.PD.set_pgain(setter)

  def set_dgain(self, setter):
    self.navigator.PD.set_dgain(setter)
   
  def set_autonomous(self, setter):
    self.autonomy = setter


class navigator:
  def __init__(self, robot):
    self.max_speed = .4
    self.center_speed = .1
    self.r = robot.at
    self.PD = PD.PD(0.005, 0.1)  

  def navigate(self, estimator, b, f):
    if(f < 3 and b < 3):
      return
      # in this case we have most likely lost sensor data.  
      # Wander or try and find the way back
    self.error = abs(-b + f)/2.0 #get average error
    if(b < f):
      self.error *= -1
    # self.error = int(raw_input("enter an error value: "))
    self.correction = self.PD.process(self.error, 1.0)
    self.set_dir(self.correction)
    progress("error: %s  correction: %s" %  (str(self.error), str(self.correction)))    

  def set_dir(self, direction):
    # direction is value from 1 (right) to -1 (left)
    self.right_speed = self.center_speed + direction/30.0
    self.left_speed = self.center_speed - direction/30.0
    self.right_speed = -1*self.saturate(self.right_speed)
    self.left_speed = self.saturate(self.left_speed)
    self.r.LEFT.set_torque(self.left_speed)
    self.r.RIGHT.set_torque(self.right_speed)
  
  def saturate(self, val):
    if(val < 0):
      return 0
    elif(val > self.max_speed):
      return self.max_speed
    else:
      return val

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
    self.delta_R = encoder_R - self.R_prev
    self.L_prev = encoder_L
    self.R_prev = encoder_R
    
    self.angle_prev = self.angle_est
    
    self.delta_theta = (self.delta_R-self.delta_L) * rad_per_click * wheel_radius/robot_width
    self.delta_x = math.cos(self.theta+self.delta_theta/2.0) * (self.delta_L+self.delta_R) * rad_per_click * wheel_radius/2
    self.delta_y = math.sin(self.theta+self.delta_theta/2.0) * (self.delta_L+self.delta_R) * rad_per_click * wheel_radius/2
		

    self.x = self.x + self.delta_x
    self.y = self.y + self.delta_y
    self.theta = self.theta + self.delta_theta

    print("x: " + str(self.x))
    print("y: " + str(self.y))
    print("theta: " + str(self.theta) + '\n') 

class Joy_interface( JoyApp ):
  
  def __init__(self,spec,*arg,**kw): 
    self.sensor_enable = True
    self.motor_enable = True    

    try:
      opts, args = getopt.getopt(sys.argv[1:],"hwsm",["wireless","sensor_disable","motor_disable"])
    except getopt.GetoptError:
      print """                 -h for help
               -w (--wireless) for wireless mode
               -s (--sensor_disable) for sensor disable
               -m (--motor_disable) for motor disable"""
      sys.exit(2)
    for opt, arg in opts:
      if opt == '-h': 
        print """  -h for help
  -w (--wireless) for wireless mode
  -s (--sensor_disable) for sensor disable
  -m (--motor_disable) for motor disable"""
        sys.exit()
      elif opt in ("-w", "--wireless"):
        L.DEFAULT_PORT = "/dev/ttyACM0"
        progress("starting in wireless mode")
      elif opt in ("-s", "--sensor_disable"):
        self.sensor_enable = False
        progress("sensor has been disabled")   
      elif opt in ("-m", "--motor_disable"):
        self.motor_enable = False
    
    JoyApp.__init__(self, robot = {'count':3}, *arg,**kw)
    self.teleop = True
    self.knob_pos = 0
    self.laser_PD = PD.PD(-79.0/1270.0, -32.0/127.0)
    # JoyApp.__init__(self, *arg, **kw)

  def onStart( self ):
    # Set up the sensor receiver plan
    self.sensor = SensorPlan(self,("67.194.202.70",8080), robot = self.robot)
    self.sensor.start()
    if(self.sensor_enable == False):
      self.sensor.sensor_enable = False
    
  def onEvent( self, evt ):
    
    if(evt.type == MIDIEVENT):
      #teleoperation commands
      if(self.teleop == True):
        if(evt.kind=='slider' and evt.index==1):
          if(evt.value < 66 and evt.value > 60):
            self.robot.at.LEFT.set_torque(0)
          else:
            self.robot.at.LEFT.set_torque((evt.value - 63.5)/(63.5))
        elif(evt.kind=='slider' and evt.index==2):
          if(evt.value < 66 and evt.value > 60):
            self.robot.at.RIGHT.set_torque(0)
          else:
            self.robot.at.RIGHT.set_torque(-1*(evt.value - 63.5)/(63.5))
        elif(evt.kind=='knob' and evt.index==1):
          self.knob_pos = (evt.value/127.0)*30000.0
          self.laser_PD_controller()
        elif(evt.kind=='knob' and evt.index==2):
          self.laser_PD.set_pgain(-1*evt.value/1270.0)
        elif(evt.kind=='knob' and evt.index==3):
          self.laser_PD.set_dgain(-1*evt.value/127.0)
        elif(evt.kind=='knob' and evt.index==4):
          self.sensor.set_pgain(-1*evt.value/1270.0)
        elif(evt.kind=='knob' and evt.index==5):
          self.sensor.set_dgain(-1*evt.value/127.0)
      if(evt.kind == 'play' and evt.value == 127):
        self.teleop = not self.teleop
        self.sensor.set_autonomous(not self.teleop)
        if(self.teleop):
          progress("entering teleoperation mode")
        else:
          progress("entering autonomous mode") 
    if(evt.type is KEYDOWN and evt.key in [ord('q'),27]):
      self.stop()

    #elif(evt.type == TIMEREVENT):
      #if(self.teleop):
      #  self.laser_PD_controller()
  
  def laser_PD_controller( self ):
    self.laser_pos = self.robot.at.LASER.get_pos()
    self.laser_error = self.laser_pos % 30000.0
    self.laser_error -= self.knob_pos
    self.laser_error /= 1000.0
    if(self.laser_error == 14043.0):
      # upper dead band
      self.new_torque = self.laser_PD.process_deadband(self.laser_error, 1.0)
    elif(self.laser_error == -14016):
      # lower dead band
      self.new_torque = self.laser_PD.process_deadband(self.laser_error, 1.0)
    else:
      self.new_torque =  self.laser_PD.process(self.laser_error, 1.0)

    if(self.new_torque > 1):
      self.new_torque = 1
    elif(self.new_torque < -1):
      self.new_torque = -1 
    self.robot.at.LASER.set_torque(self.new_torque)

  def onStop( self ):
    self.robot.at.LEFT.go_slack()
    self.robot.at.RIGHT.go_slack()
    self.robot.at.LASER.go_slack()
    self.sensor.stop()
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

