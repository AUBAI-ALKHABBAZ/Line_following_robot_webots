"""trajectory_follower controller."""



# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot,Camera, Display,GPS,Compass ,CameraRecognitionObject,Supervisor
import numpy as np
import cv2
from matplotlib import pyplot as plt
from scipy import signal

# create the Robot instance.
robot = Supervisor()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

motor_left = robot.getDevice('left wheel motor')
motor_right = robot.getDevice('right wheel motor')

motor_left.setPosition(float('Inf'))
motor_right.setPosition(float('Inf'))
lidar=robot.getDevice('LDS-01')
lidar.enable(timestep)
lidar.enablePointCloud()
display = robot.getDevice('display')

# Initialize camera
camera = robot.getDevice('camera')
camera.enable(timestep)
camera.recognitionEnable(timestep)
num_objects = camera.getRecognitionNumberOfObjects()
camera.enableRecognitionSegmentation()
display_1 = robot.getDevice('display(1)')
width = camera.getWidth()
height = camera.getHeight()

gps = robot.getDevice('gps')
gps.enable(timestep)

compass = robot.getDevice('compass')
compass.enable(timestep)

max_speed = 6.28

r = 0.0201 # radius of the wheels

d = 0.052 # distance between the 2 wheels

X = 0 # Initializing total displacement for deltaX

W = 0 # Initializing total rotation for deltaW

# Initializing robot position and orientation in world co-ordinate frame
# at time=0
xw = 0
yw = 0.028
alpha = 1.5708
angles=np.linspace(3.1415,-3.1415,360)

# Initialize the ground sensors
gs=[]
gsNames = ['gs0', 'gs1', 'gs2']
for i in range(3):
    gs.append(robot.getDevice(gsNames[i]))
    gs[-1].enable(timestep)
   
map = np.zeros((300,300))
kernel= np.ones((20,20)) 

def world2map(xw,yw):
   px =int((xw+0.5-0.305)*300)
   py =int(299-(yw+0.25)*300)
   
   
   px = min(px,299)
   py = min(py,299)
   px = max(px,0)
   py = max(py,0)
   return [px,py]

WP = [(0,0.2),(0,0.68),(0.44,0.68),(0.66,0.51),(0.35,0.24),(0,-0.169)]
index = 0
marker = robot.getFromDef("marker").getField("translation")

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    
      
   
    data = camera.getImage()
    
    objects = camera.getRecognitionObjects()
    #print(CameraRecognitionObject.getId())
    number_of_objects = camera.getRecognitionNumberOfObjects()
    #print(f'Recognized {number_of_objects} objects.')
    #camera = getPosition()
    counter = 1
    for object in objects:
      
      id_object = object.getId()
      position = object.getPosition()
      position = object.getPosition()
      orientation = object.getOrientation()
      size = object.getSize()
      position_on_image = object.getPositionOnImage()
      size_on_image = object.getSizeOnImage()
      number_of_colors = object.getNumberOfColors()
      colors = object.getColors()
      
      
      #print(f' Object {counter}/{number_of_objects}: {object.getModel()} (id = {object.getId()})')
      #print(f' Position: {position[0]} {position[1]} {position[2]}')
      #print(f' Orientation: {object.orientation[0]} {orientation[1]} {orientation[2]} {orientation[3]}')
      #print(f' Size: {size[0]} x {size[1]}')
      #print(f' Position on camera image: {position_on_image[0]} {position_on_image[1]}')
      #print(f' Size on camera image: {size_on_image[0]} x {size_on_image[1]}')
      #print(object.getPositionOnImage())
      #print(id_object)
      
      #print ('id_object ' + str(id_object))
    
    if data:
        ir = display_1.imageNew(data, Display.BGRA, width, height)
        display_1.imagePaste(ir, 0, 0, False)
        display_1.imageDelete(ir)
    xw = gps.getValues()[0]
    yw = gps.getValues()[1]
    

    alpha=np.arctan2(compass.getValues()[0],compass.getValues()[1])
    
    marker.setSFVec3f([*WP[index],0])
    
    rho = np.sqrt((xw-WP[index][0])**2+(yw-WP[index][1])**2)
    
    theta = np.arctan2(WP[index][1]-yw,WP[index][0]-xw)-alpha
    if (theta >np.pi):
        theta = theta -2*np.pi
    print(rho,theta/3.1415*180)
    if (rho <0.1):
        index +=1
    #print(xw,yw,alpha)
    g = []
    for i in range(3):
        g.append(gs[i].getValue())

    # Process sensor data here.
    #print(g)

    
    A= np.cos(alpha)
    B=np.cos(alpha+ 1.57)
    C=np.cos(alpha- 1.57)
    x_r=[]
    y_r=[]
    x_w=[]
    y_w=[]
    
    #w_R_r = np.array([[np.cos(alpha),- np.sin(alpha)],
               #[np.sin(alpha),np.cos(alpha)]] )
               
    w_T_r = np.array([[np.cos(alpha),- np.sin(alpha),xw],
               [np.sin(alpha),np.cos(alpha),yw],
               [0,0,1]] )
    ranges = np.array(lidar.getRangeImage())
    ranges[ranges == np.inf] =100         
    #X_w = np.array([[xw],[yw]]) 
    x_i = np.array([ranges*np.cos(angles),ranges*np.sin(angles),np.ones(360,)])
    
    D = w_T_r @ x_i
    
    px,py = world2map(xw,yw)
    
    display.setColor(0xFF0000)
    display.drawPixel(px,py)

    #print(len(x_i))
    #plt.ion()
    #plt.plot(D[0,:],D[1,:],'.')
    #plt.pause(0.01)
    #plt.show()
    '''for i,angle in enumerate(angles):

        x_i=round(ranges[i]*np.cos(angle),2)
        y_i=round(ranges[i]*np.sin(angle),2)
        x_r.append(x_i)
        y_r.append(y_i)
        #A= round(np.cos(alpha),2)
        #B=round(np.cos(alpha+ 1.57),2)
        #C=round(np.cos(alpha- 1.57),2)
        #D = w_R_r @ np.array([[x_i],[y_i]])+ X_w
        D = w_T_r @ np.array([[x_i],[y_i],[1]])
        #x_w.append(A* x_i + B* y_i+xw)
        #y_w.append(C* x_i + A* y_i+yw)
        x_w.append(D[0])
        y_w.append(D[1])
    plt.ion()
    plt.plot(x_w,y_w,'.')
    plt.pause(0.01)
    plt.show()'''
    
    for d in D.transpose():
        px,py = world2map(d[0],d[1])
        map[px,py]+= 0.01
        if (map[px,py]>0.8):
            map[px,py] =1

        v=int(map[px,py]*255)
        
        #color=0xFFFFFF
        color=(v*256**2+v*256+v)
        display.setColor(int(color))
        display.drawPixel(px,py)
    # normalizing sensor data with 1600
    # conv  map and kernal 20x20
    #cmap = signal.convolve2d(map,kernel,mode='same')
    #cspace = cmap>0.9
    #plt.ion()
    #plt.imshow(cspace)
    #plt.pause(0.01)
   # plt.show()

    if g[0]>600 and g[1]<350 and g[2]>600: # forward
        phildot = g[0]/1600 * (2*np.pi) 
        phirdot = g[2]/1600 * (2*np.pi) 
    elif g[2]<600: # Turn right
        phildot = (g[0]/1600 + g[1]/1600) * (2*np.pi) 
        phirdot = (g[2]/1600 - g[1]/1600) * (2*np.pi) 
    elif g[0]<600: # Turn left
        phildot = (g[0]/1600 - g[1]/1600) * (2*np.pi) 
        phirdot = (g[2]/1600 + g[1]/1600) * (2*np.pi)


    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    p1 =6.28 *0.5
    p2 = 62.8
    
    phildot= -theta *p1 +rho*p2
    phirdot= theta*p1 +rho *p2
   
    motor_left.setVelocity(max(min(phildot,6.28),-6.28))
    motor_right.setVelocity(max(min(phirdot,6.28),-6.28))
    
    '''Deltax = (r/2)*(phildot+phirdot)*(timestep/1000)
    #print(Deltax)
    #X = X + Deltax
    
    Deltaw = (r/d)*(phirdot-phildot)*(timestep/1000)
    
    #W = (W + Deltaw)    
    #W_deg = W*180/3.1425    
    #print(W_deg)
    xw = xw + np.cos(alpha)*Deltax
    yw = yw + np.sin(alpha)*Deltax
    alpha = alpha + Deltaw
    alpha_deg = alpha*180/3.1425'''
    
    #print("[xw(m) yw(m) alpha(deg)] = " + str( [xw, yw, alpha_deg] ) )
    #print("Position Error(m) = " + str(np.sqrt(xw**2 + yw**2)) )
    
    pass

# Enter here exit cleanup code.
