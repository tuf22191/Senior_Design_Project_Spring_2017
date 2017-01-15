
#THE IMPORTS: please pip install or use another p-manager
import numpy as np
import pylab
import math
import random
from numpy.linalg import inv
#THE ALGORITHM
# prediction equations:
#     X = Axk-1 + Buk-1
#     p =APA^T +Q
# update equations:
#     K=p(H^T)(HpH^T +R )^-1
#     xk= X+K(z-HX)
#     pxk=(I-KH)p
# please refer to :http://robotsforroboticists.com/kalman-filtering/
# please refer to: http://greg.czerniak.info/guides/kalman1/kalman2.py.txt

#CLASS DECLERATION
class KFilter:
    """Use the constructor to pass in the correct matrices."""

    def __init__(self,_Xn,_Pn,_A,_B,_H,_Q,_R,_Y,_K):
        # self.Un = _Un
        # self.Zn = _Zn
        self.Xn = _Xn
        self.Pn = _Pn
        self.A = _A
        self.B = _B
        self.H = _H
        self.Q = _Q
        self.R = _R
        self.Y = _Y #for the actual measurement
        self.K = _K
    def getCurrentState(self):
        return self.Xn
    def stepOver(self,cont_vect,meas_vect):
        Un=cont_vect
        Zn=meas_vect
        #predict , observe, update
        #predict
        predictedX = np.add(np.dot(self.A,self.Xn),np.dot(self.B, Un))
        predictedP = np.add(np.dot(self.A,np.dot(self.Pn,np.transpose(self.A))),  self.Q)

        #observe
        innov_y = np.subtract(Zn, np.dot(self.H,predictedX))
        innov_y_covariance = np.add(np.dot(self.H,np.dot(predictedP ,np.transpose(self.H))),self.R)

        #update
        Kalman_Gain = np.dot(predictedP, np.dot(np.transpose(self.H),inv(innov_y_covariance)))
        self.Xn = np.add(predictedX  ,np.dot(Kalman_Gain,innov_y) )
        dimensions = int(self.Pn.shape[0])
        self.Pn = np.dot(np.subtract(np.eye(dimensions), np.dot(Kalman_Gain,self.H)),predictedP)

def main():
    #kf = KFilter()
    print(KFilter.__doc__)

#if(__name__ == "__main__"):
#    main()

# Simulates the classic physics problem of a cannon shooting a ball in a
# parabolic arc.  In addition to giving "true" values back, you can also ask
# for noisy values back to test Kalman filters.
class Cannon:
  #--------------------------------VARIABLES----------------------------------
  angle = 45 # The angle from the ground to point the cannon.
  muzzle_velocity = 100 # Muzzle velocity of the cannon.
  gravity = [0,-9.81] # A vector containing gravitational acceleration.
  # The initial velocity of the cannonball
  velocity = [muzzle_velocity*math.cos(angle*math.pi/180), muzzle_velocity*math.sin(angle*math.pi/180)]
  loc = [0,0] # The initial location of the cannonball.
  acceleration = [0,0] # The initial acceleration of the cannonball.
  #---------------------------------METHODS-----------------------------------
  def __init__(self,_timeslice,_noiselevel):
    self.timeslice = _timeslice
    self.noiselevel = _noiselevel
  def add(self,x,y):
    return x + y
  def mult(self,x,y):
    return x * y
  def GetX(self):
    return self.loc[0]
  def GetY(self):
    return self.loc[1]
  def GetXWithNoise(self):
    return random.gauss(self.GetX(),self.noiselevel)
  def GetYWithNoise(self):
    return random.gauss(self.GetY(),self.noiselevel)
  def GetXVelocity(self):
    return self.velocity[0]
  def GetYVelocity(self):
    return self.velocity[1]
  # Increment through the next timeslice of the simulation.
  def Step(self):
    # We're gonna use this vector to timeslice everything.
    timeslicevec = [self.timeslice,self.timeslice]
    # Break gravitational force into a smaller time slice.
    sliced_gravity = map(self.mult,self.gravity,timeslicevec)
    # The only force on the cannonball is gravity.
    sliced_acceleration = sliced_gravity
    # Apply the acceleration to velocity.
    self.velocity = map(self.add, self.velocity, sliced_acceleration)
    sliced_velocity = map(self.mult, self.velocity, timeslicevec )
    # Apply the velocity to location.
    self.loc = map(self.add, self.loc, sliced_velocity)
    # Cannonballs shouldn't go into the ground.
    if self.loc[1] < 0:
      self.loc[1] = 0

#=============================REAL PROGRAM START================================
# Let's go over the physics behind the cannon shot, just to make sure it's
# correct:
# sin(45)*100 = 70.710 and cos(45)*100 = 70.710
# vf = vo + at
# 0 = 70.710 + (-9.81)t
# t = 70.710/9.81 = 7.208 seconds for half
# 14.416 seconds for full journey
# distance = 70.710 m/s * 14.416 sec = 1019.36796 m

timeslice = 0.1 # How many seconds should elapse per iteration?
iterations = 144 # How many iterations should the simulation run for?
# (notice that the full journey takes 14.416 seconds, so 145 iterations will
# cover the whole thing when timeslice = 0.10)
noiselevel = 30  # How much noise should we add to the noisy measurements?
muzzle_velocity = 100 # How fast should the cannonball come out?
angle = 45 # Angle from the ground.

# These are arrays to store the data points we want to plot at the end.
x = []
y = []
nx = []
ny = []
kx = []
ky = []

# Let's make a cannon simulation.
c = Cannon(timeslice,noiselevel)

speedX = muzzle_velocity*math.cos(angle*math.pi/180)
speedY = muzzle_velocity*math.sin(angle*math.pi/180)

# This is the state transition vector, which represents part of the kinematics.
# 1, ts, 0,  0  =>  x(n+1) = x(n) + vx(n)
# 0,  1, 0,  0  => vx(n+1) =        vx(n)
# 0,  0, 1, ts  =>  y(n+1) =              y(n) + vy(n)
# 0,  0, 0,  1  => vy(n+1) =                     vy(n)
# Remember, acceleration gets added to these at the control vector.
state_transition = np.matrix([[1,timeslice,0,0],[0,1,0,0],[0,0,1,timeslice],[0,0,0,1]])

control_matrix = np.matrix([[0,0,0,0],[0,0,0,0],[0,0,1,0],[0,0,0,1]])
# The control vector, which adds acceleration to the kinematic equations.
# 0          =>  x(n+1) =  x(n+1)
# 0          => vx(n+1) = vx(n+1)
# -9.81*ts^2 =>  y(n+1) =  y(n+1) + 0.5*-9.81*ts^2
# -9.81*ts   => vy(n+1) = vy(n+1) + -9.81*ts
control_vector = np.matrix([[0],[0],[0.5*-9.81*timeslice*timeslice],[-9.81*timeslice]])

# After state transition and control, here are the equations:
#  x(n+1) = x(n) + vx(n)
# vx(n+1) = vx(n)
#  y(n+1) = y(n) + vy(n) - 0.5*9.81*ts^2
# vy(n+1) = vy(n) + -9.81*ts
# Which, if you recall, are the equations of motion for a parabola.  Perfect.

# Observation matrix is the identity matrix, since we can get direct
# measurements of all values in our example.
observation_matrix = np.eye(4)

# This is our guess of the initial state.  I intentionally set the Y value
# wrong to illustrate how fast the Kalman filter will pick up on that.
initial_state = np.matrix([[0],[speedX],[500],[speedY]])

initial_probability = np.eye(4)

process_covariance = np.zeros(4)
measurement_covariance = np.eye(4)*0.2
#(self,_Xn,_Pn,_A,_B,_H,_Q,_R,_Y,_K)
kf = KFilter(_A = state_transition, _B=control_matrix, _H=observation_matrix, _Xn=initial_state, _Pn=initial_probability, _Q =process_covariance, _R =measurement_covariance,_Y=None,_K=None)

# Iterate through the simulation.
for i in range(iterations):
    x.append(c.GetX())
    y.append(c.GetY())
    newestX = c.GetXWithNoise()
    newestY = c.GetYWithNoise()
    nx.append(newestX)
    ny.append(newestY)
    # Iterate the cannon simulation to the next timeslice.
    c.Step()
    dfdfdf=kf.getCurrentState()
    kx.append(dfdfdf[0,0])
    ky.append(dfdfdf[2,0])
    kf.stepOver(cont_vect =control_vector,meas_vect = np.matrix([[newestX],[c.GetXVelocity()],[newestY],[c.GetYVelocity()]]))

# Plot all the results we got.
pylab.plot(x,y,'-',nx,ny,':',kx,ky,'--')
pylab.xlabel('X position')
pylab.ylabel('Y position')
pylab.title('Measurement of a Cannonball in Flight')
pylab.legend(('true','measured','kalman'))
pylab.show()
