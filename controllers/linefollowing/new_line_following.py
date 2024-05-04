"""linefollowing controller."""

# You may need to import some classes of the controller module. Ex:
import numpy as np
from matplotlib import pyplot as plt
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot,Camera, Display



# create the Robot instance.
robot = Robot()
timestep = int(robot.getBasicTimeStep())

MAX_SPEED = 6.28
yw = 0.0277
w,x,xw =0,0,0 
alpha = 1.57
# get the time step of the current world.
timestep = 32
angles=np.linspace(3.1415,-3.1415,360)

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
lidar = robot.getDevice('LDS-01')
lidar.enable(timestep)
lidar.enablePointCloud()

display = robot.getDevice('display')

gs = []
for i in range(3):
   gs.append(robot.getDevice('gs'+str(i)))
   gs[-1].enable(timestep)

# Main loop:asdas
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:

    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    g = []
    for gsensor in gs:
        g.append(gsensor.getValue())
    #rocess sensor data here.
    print(g)
    ranges = lidar.getRangeImage()
    
    A= np.cos(alpha)
    B=np.cos(alpha+ 1.57)
    C=np.cos(alpha- 1.57)
    x_r=[]
    y_r=[]
    x_w=[]
    y_w=[]
    for i,angle in enumerate(angles):

        x_i=round(ranges[i]*np.cos(angle),2)
        y_i=round(ranges[i]*np.sin(angle),2)
        x_r.append(x_i)
        y_r.append(y_i)
        A= round(np.cos(alpha),2)
        B=round(np.cos(alpha+ 1.57),2)
        C=round(np.cos(alpha- 1.57),2)
        
        x_w.append(A* x_i + B* y_i+xw)
        y_w.append(C* x_i + A* y_i+yw)
    
    plt.ion()
    plt.plot(x_w,y_w,'.')
    plt.pause(0.01)
    plt.show()

    if(g[0]>500 and g[1]<350 and g[2]>500):
        lspeed,rspeed = MAX_SPEED,MAX_SPEED
    elif (g[2]<550):
        lspeed,rspeed = 0.25*MAX_SPEED, -0.1 *MAX_SPEED
    elif (g[0]<550):
        lspeed,rspeed = -0.1*MAX_SPEED , 0.25*MAX_SPEED
    else :
        lspeed,rspeed = 0,0

         
    
    leftMotor.setVelocity(lspeed)
    rightMotor.setVelocity(rspeed)
    
    ld = 2.01 * lspeed
    rd = 2.01 * rspeed
    
    dT = timestep/1000
    
    dX = ((ld + rd)/2) * dT
    x = x + dX
    print("displasment",x)
    dW = (rd - ld)/ 5.2 * dT
    w = w + dW
    print("radis",w)
    xw = xw + np.cos(alpha)*dX
    yw = yw + np.sin(alpha)*dX
    
    alpha = alpha + dW
    
    print("x :", x, "xw :", xw, "yw :", yw, "alpha :", alpha, sep="__")
    #Distance error
    print("error",np.sqrt(xw**2 + yw**2))

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass
