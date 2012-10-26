from joy import *

class HelloJoyApp( JoyApp ):
  """HelloJoyApp
  
     The "hello world" of JoyApp programming.
     This JoyApp pipes the y coordinate of mouse positions (while left
     button is pressed) to a specified setter. By default this setter is
     given by "#output " -- i.e. it is a debug message. 
     
     See JoyApp.setterOf() for a specification of possible outputs
  """
  left_dir = 1
  right_dir = 1
  laser_dir = 1

  def __init__(self,spec,*arg,**kw):
    # This is a "constructor". It initializes the JoyApp object.
    # Because we added an additional parameter, we extend the 
    # JoyApp constructor. The first step is to call the superclass
    # constructor so that we'll have a valid JoyApp instance.
    JoyApp.__init__(self, robot = {'count':3}, *arg,**kw)
    # Store output specifier for later use
    self.spec = spec
    
  def onStart(self):
    # This function is called when the JoyApp is ready to start up,
    # i.e. after all PyGame devices have been activated, robot Cluster
    # is populated, scratch interface is live, etc.
    self.output = self.setterOf(self.spec)

  def onEvent(self,evt):
    # All unknown events --> punt to superclass
    if evt.type != MIDIEVENT:
      return JoyApp.onEvent(self,evt)
    
    if(evt.kind=='slider' and evt.index==1):
	self.robot.at.LEFT.set_torque(self.left_dir*evt.value/127.0)
    elif(evt.kind=='slider' and evt.index==2):
	self.robot.at.RIGHT.set_torque(self.right_dir*evt.value/127.0)
    elif(evt.kind=='knob' and evt.index==1):
    	self.robot.at.LASER.set_torque(self.right_dir*evt.value/127.0)
    elif(evt.kind=='btnU'):
	if(evt.index == 1 and evt.value == 127):
	    self.left_dir *= -1
	elif(evt.index == 2 and evt.value == 127):
	    self.right_dir *= -1    
    

    # If we reach this line, it was a MOUSEMOTION with button pressed
    #   so we send the value out to the output
    # self.output( evt.pos[1] )
    
  
app = HelloJoyApp("#output ")
app.run()
