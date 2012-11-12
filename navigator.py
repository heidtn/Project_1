from joy import *
from socket import socket, AF_INET, SOCK_STREAM, error as SocketError
from json import loads as json_loads
import ckbot.logical as L
import math
import PD
import getopt

wheel_radius = 0.0619 # meter
rad_per_click = (math.pi*2)/40095
robot_width = 0.149 # meter
encoder_limit = 435036.0 + 14026.0
laser_encoder_limit = 14043.0 + 14016.0

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
    self.spinning = False
    self.ways = way_points()

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
        
        if('w' in dic):
          self.ways.newValueCheck(dic['w'])

      self.lpos = self.r.at.LEFT.get_pos()
      self.rpos = self.r.at.RIGHT.get_pos()
      
      try:
        if(self.sensor_enable and self.ways.has_ways):      
          self.estimator.estimate_state(dic['b'], dic['f'], self.lpos, self.rpos, self.ways)
        else: 
          self.estimator.estimate_state(0, 0, self.lpos, self.rpos, self.ways)

        yield
        if(self.autonomy):
          if(not self.sensor_enable):
            dic = {'b':255, 'f':255}
          if(self.ways.way_changed or self.spinning):
            self.spinning = self.navigator.turn(self.estimator, self.ways.psi)
          else: 
            self.navigator.navigate(self.estimator, dic['b'], dic['f'], self.ways)
        
        progress("estimated angle:  %s    estimated psi:  %s" % (str(self.estimator.theta * 180.0/math.pi), str(self.ways.psi*180.0/math.pi)))    
        self.ways.way_changed = False
      except Exception, ex:
        self.r.at.LEFT.set_torque(0)
        self.r.at.RIGHT.set_torque(0)
        print ex.message
        
      yield self.forDuration(0.3)
 
  def set_pgain(self, setter):
    self.navigator.PD.set_pgain(setter)

  def set_dgain(self, setter):
    self.navigator.PD.set_dgain(setter)
   
  def set_autonomous(self, setter):
    self.autonomy = setter


class way_points:
  def __init__( self ):
    self.way_changed = False
    self.has_ways = False
    self.WPData = []    
    self.yn = 0
    self.xn = 0
    self.psi = 0

  def newValueCheck(self, newWPData):
    if (len(newWPData) == len(self.WPData)):  #if waypoint value is empty
      self.WPData = newWPData #no change in WP data
    else: #if new WP is different from old WP
      self.WPData = newWPData #chage WP to new values
      print "New waypoint values found:"
      print repr(newWPData)
      self.yp = newWPData[0][1] #replace the previous waypoint information with the new one we just reached
      self.xp = newWPData[0][0]
      self.yn = newWPData[1][1] #the y of new waypoint
      self.xn = newWPData[1][0]#the x of new waypoint
      if(self.xp == self.xn and self.yn > self.yp):
        self.psi = math.pi/2.0
      elif(self.xp == self.xn and self.yn < self.yp):
        self.psi = -math.pi/2.0
      elif(self.yp == self.yn and self.xn > self.xp):
        self.psi = 0.0
      elif(self.yp == self.yn and self.xn < self.xp):
        self.psi = math.pi
      elif(self.yp == self.yn and self.xn == self.xp):
        self.psi = math.pi
      else:
        self.psi = math.atan((self.yp-self.yn)/(self.xp-self.xn)) #angle of the line
      
      if(self.xn < self.xp):
        self.psi += math.pi

      if(self.psi < 0):
        self.psi += 2.0*math.pi

      self.x = self.xp #update the estimate position of the robot
      self.y = self.yp
      self.way_changed = True
      self.has_ways = True
      print(repr(newWPData) + '\n')
      print("new x: " + str(self.x) + '\n')
      print("new y: " + str(self.y) + '\n')



class navigator:
  def __init__(self, robot):
    self.max_speed = .4
    self.center_speed = .1
    self.r = robot.at
    self.PD = PD.PD(0.005, 0.1)  

  def navigate(self, estimator, b, f, ways):
    if(f < 3 and b < 3):
      #self.set_dir(.1)
      return
      # in this case we have most likely lost sensor data.  
      # Wander or try and find the way back
    self.error = abs(-b + f)/2.0 #get average error
    if(b < f):
      self.error *= -1
    # self.error = int(raw_input("enter an error value: "))
    self.correction = self.PD.process(self.error, 1.0)
    self.set_dir(self.correction)
    #  progress("error: %s  correction: %s" %  (str(self.error), str(self.correction))) 

  def turn(self, estimator, psi):
    # estimator contains both necessary thetas
    progress("turning")
    self.error = abs(estimator.theta - psi)
    #if(self.error > math.pi):
    #  self.error -= 2.0*math.pi
    #elif(self.error < -math.pi):
    #  self.error += 2.0*math.pi
   
    if(self.error < math.pi/8):
      return False  
     
    self.r.LEFT.set_torque(.1)
    self.r.RIGHT.set_torque(.1)
    return True

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
    self.L_prev = initial_L+14026
    self.R_prev = initial_R+14026
    self.angle_prev = math.pi
    self.x = 0
    self.y = 0
    self.theta = 0
    self.theta_1 = 0
    self.theta_2 = 0
    self.P1 = .2
    self.P2 = .8
    self.d = 38
    
  def estimate_state(self, b, f, encoder_L, encoder_R, ways):
    
    encoder_L += 14026
    encoder_R += 14026 
    self.delta_L = encoder_L - self.L_prev
    self.delta_R = -encoder_R + self.R_prev
    if(self.delta_L > encoder_limit/2.0):
      self.delta_L -= encoder_limit
    elif(self.delta_L < -encoder_limit/2.0):
      self.delta_L += encoder_limit

    if(self.delta_R > encoder_limit/2.0):
      self.delta_R -= encoder_limit
    elif(self.delta_R < -encoder_limit/2.0):
      self.delta_R += encoder_limit

    self.L_prev = encoder_L
    self.R_prev = encoder_R
    
    self.angle_prev = self.angle_est
    
    self.delta_theta = (self.delta_R-self.delta_L) * rad_per_click * wheel_radius/robot_width
    self.delta_theta /= 2.0

    self.delta_x = math.cos(self.theta+self.delta_theta/2.0) * (self.delta_L+self.delta_R) * rad_per_click * wheel_radius/2
    self.delta_y = math.sin(self.theta+self.delta_theta/2.0) * (self.delta_L+self.delta_R) * rad_per_click * wheel_radius/2
		

    self.x = self.x + self.delta_x
    self.y = self.y + self.delta_y
    
    if(ways.way_changed):
      self.x = ways.x
      self.y = ways.y
    
    self.theta_1 = self.theta + self.delta_theta
    self.theta_1 %= 2.0*math.pi

    if(b > 50 and f > 50 and ways.has_ways and False):
      self.psi = ways.psi

      if((b + f)/self.d > 1):
        self.theta_2 = self.psi
      elif (self.theta>self.psi):
        self.theta_2 = (self.psi + math.acos((b+f)/self.d))
      else:
        self.theta_2 = (self.psi - math.acos((b+f)/self.d))
      
      self.theta = self.P1*self.theta_1 + self.P2*self.theta_2    
    else:
      self.theta = self.theta_1
     
class Joy_interface( JoyApp ):
  
  def __init__(self,spec,*arg,**kw): 
    self.sensor_enable = True
    self.motor_enable = True    
    self.teleop = True

    try:
      opts, args = getopt.getopt(sys.argv[1:],"hwsma",["wireless","sensor_disable","motor_disable","autonomous_mode"])
    except getopt.GetoptError:
      print """                 -h for help
               -w (--wireless) for wireless mode
               -s (--sensor_disable) for sensor disable
               -m (--motor_disable) for motor disabl
               -a (--autonomous_mode) for autonomous"""
      sys.exit(2)
    for opt, arg in opts:
      if opt == '-h': 
        print """  -h for help
  -w (--wireless) for wireless mode
  -s (--sensor_disable) for sensor disable
  -m (--motor_disable) for motor disable
  -a (--autonomous_mode) for autonomous"""
        sys.exit()
      elif opt in ("-w", "--wireless"):
        L.DEFAULT_PORT = "/dev/ttyACM0"
        progress("starting in wireless mode")
      elif opt in ("-s", "--sensor_disable"):
        self.sensor_enable = False
        progress("sensor has been disabled")   
      elif opt in ("-m", "--motor_disable"):
        self.motor_enable = False
      elif opt in ("-a", "--autonomous_mode"):
        self.teleop = False
    
    JoyApp.__init__(self, robot = {'count':3}, *arg,**kw)
    self.laser_initial_pos = math.pi/2 
    self.knob_pos = 0
    self.laser_PD = PD.PD(-790.0/1270.0, -320.0/1270.0)
    # JoyApp.__init__(self, *arg, **kw)

  def onStart( self ):
    # Set up the sensor receiver plan
    self.sensor = SensorPlan(self,("67.194.202.70",8080), robot = self.robot)
    self.sensor.start()
    if(self.sensor_enable == False):
      self.sensor.sensor_enable = False
    if(self.teleop == False):
      self.sensor.set_autonomous(True)    
    self.initial_laser = self.robot.at.LASER.get_pos() + 14016
    
    
  def onEvent( self, evt ):
    try:
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
           # self.robot.at.LASER.set_pos((evt.value))
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

      if(evt.type is KEYDOWN and self.teleop):
        if(evt.key == ord('w')):
          self.robot.at.LEFT.set_torque(.2)
          self.robot.at.RIGHT.set_torque(-.2)
        if(evt.key == ord('d')):
          self.robot.at.LEFT.set_torque(.2)
          self.robot.at.RIGHT.set_torque(.2)
        if(evt.key == ord('a')):
          self.robot.at.LEFT.set_torque(-.2)
          self.robot.at.RIGHT.set_torque(-.2)
        if(evt.key == ord('s')):
          self.robot.at.LEFT.set_torque(-.2)
          self.robot.at.RIGHT.set_torque(.2)
      if(evt.type is KEYUP and self.teleop):
        self.robot.at.LEFT.set_torque(0)
        self.robot.at.RIGHT.set_torque(0)  

      if(evt.type is KEYDOWN and evt.key == ord('p')):
        self.teleop = not self.teleop
        self.sensor.set_autonomous(not self.teleop)
        if(self.teleop):
          progress("entering teleoperation mode")
        else:
          progress("entering autonomous mode")
      
      if(evt.type == TIMEREVENT):
        self.laser_PD_controller()
    except Exception, ex:
      self.r.at.LEFT.set_torque(0)
      self.r.at.RIGHT.set_torque(0)
      print ex.message
  
  def laser_PD_controller( self ):
    try:
      self.laser_pos_reading = self.robot.at.LASER.get_pos()    

      self.laser_pos = self.laser_pos_reading + 14016.0
      """if(self.laser_pos > laser_encoder_limit/2.0):
        self.laser_pos -= laser_encoder_limit
      elif(self.laser_pos < -laser_encoder_limit/2.0):
        self.laser_pos += laser_encoder_limit
      """
      self.laser_pos *= (math.pi*2.0/laser_encoder_limit)
      self.laser_pos -= self.laser_initial_pos
      self.laser_pos %= math.pi*2
      self.laser_error = self.laser_pos - (math.pi/2.0 - self.sensor.estimator.theta)
      if(self.laser_error > math.pi):
        self.laser_error -= 2.0*math.pi
      elif(self.laser_error < -math.pi):
        self.laser_error += 2.0*math.pi    
      # progress("laser pos reading: %s" % (str(self.laser_pos_reading)))
      #progress("estimated laser angle:  %s	estimated laser error:	%s" % (str(self.laser_pos * 180.0/math.pi), str(self.laser_error * 180.0/math.pi)))    
      # self.laser_error %= 2.0*math.pi
      if(self.laser_pos_reading >= 14043.0):
        # upper dead band
        self.new_torque = self.laser_PD.process_deadband(self.laser_error, 1.0)
      elif(self.laser_pos_reading <= -14016):
        # lower dead band
        self.new_torque = self.laser_PD.process_deadband(self.laser_error, 1.0)
      else:
        self.new_torque =  self.laser_PD.process(self.laser_error, 1.0)

      if(self.new_torque > 1):
        self.new_torque = 1
      elif(self.new_torque < -1):
        self.new_torque = -1

      self.robot.at.LASER.set_torque(self.new_torque)
    except Exception, ex:
      self.r.at.LASER.set_torque(0)
      print ex.message
   

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

