class PD:
  def __init__( self, pgain, dgain):
    self.pgain = pgain
    self.dgain = dgain
    self.first_call = True

  def process( self, error, timestamp):
    if self.first_call: #if this is the first function call
      self.prev_error = error  #initialize the prev_value to value
      self.first_call = False
    self.P = error
    self.D = (error - self.prev_error)/timestamp
    self.prev_error = error
    self.val = self.pgain*self.P + self.dgain*self.D
    return self.val

  def set_pgain(self, pgain):
    self.pgain = pgain

  def set_dgain(self, dgain):
    self.dgain = dgain 
