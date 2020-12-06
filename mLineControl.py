#name: David Cubrilla 
#pid: 730167540
from controller import Robot, DistanceSensor, Motor, GPS
# time in [ms] of a simulation step
TIME_STEP = 64

MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

# initialize devices
ps = []
psNames = [
    'so0', 'so1', 'so2', 'so3', 'so4', 'so5', 
    'so6', 'so7', 'so8', 'so9', 'so10', 
    'so11', 'so12', 'so13', 'so14', 'so15' 
]

gps = robot.getGPS('gps')
gps.enable(TIME_STEP)

for i in range(15):
    ps.append(robot.getDistanceSensor(psNames[i]))
    ps[i].enable(TIME_STEP)

leftMotor = robot.getMotor('left wheel')
rightMotor = robot.getMotor('right wheel')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

#turns the Robot
def turn(time, leftSpeed, rightSpeed):
    currTime = 0
    while robot.step(TIME_STEP) != -1 :
        leftMotor.setVelocity(leftSpeed * MAX_SPEED)
        rightMotor.setVelocity(rightSpeed * MAX_SPEED)
        currTime += 1 
        if currTime == time:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break 
                    
#Orient towards goal
turn(8, 0.5, -0.5)

#used to turn 90 degrees right or left
#turn right => turn(13, 0.5, -0.5) 
#turn left => turn(13, -0.5, 0.5)

def goAround() :
    
    oT = 0
  
    turn(12, 0.5, -0.5) 
    while robot.step(TIME_STEP) != -1:
        #startTimer
        oT += 1
        
        psValues = []
        for i in range(15):
            psValues.append(ps[i].getValue())
           
        noLeftSide = psValues[0] < 400.0 and (psValues[9] < 445.0)
       
        lS = 0.5 * MAX_SPEED
        rS = 0.5 * MAX_SPEED
    
        leftMotor.setVelocity(lS)
        rightMotor.setVelocity(rS)
        
        if oT == 50 :
            #turnLeft; endTimer
            turn(12, -0.5, 0.5)
            break
    
    while robot.step(TIME_STEP) != -1: 
        psValues = []
        for i in range(15):
            psValues.append(ps[i].getValue())
            
        noLeftSide = psValues[0] < 400.0 and psValues[9] < 510.0
       
        lS = 0.5 * MAX_SPEED
        rS = 0.5 * MAX_SPEED
    
        leftMotor.setVelocity(lS)
        rightMotor.setVelocity(rS)
        if noLeftSide :
            #turnLeft; endTimer
            turn(12, -0.5, 0.5)
            break
          
    oTSave = oT
    while robot.step(TIME_STEP) != -1: 
        oT -= 1
        psValues = []
        for i in range(15):
            psValues.append(ps[i].getValue())
            
        noLeftSide = psValues[0] < 400.0 and psValues[9] < 510.0
       
        lS = 0.5 * MAX_SPEED
        rS = 0.5 * MAX_SPEED
    
        leftMotor.setVelocity(lS)
        rightMotor.setVelocity(rS)
        
        #print(oT)
        
        if oT == 30 :
            #turn right towards goal; set oT to 0
            turn(13, 0.5, -0.5)
            oT = 0
            break
t = 0        
while robot.step(TIME_STEP) != -1:
    #read sensors outputs
    psValues = []
    t += 1
    for i in range(15):
        psValues.append(ps[i].getValue())
     
    
    #detect obstacles
    front_obstacle = psValues[4] > 975.0 or psValues[3] > 975.0
    noLeftSide = psValues[0] < 400.0 and psValues[10] < 440.0
    
    if front_obstacle :
       goAround()

    lS = 0.5 * MAX_SPEED
    rS = 0.5 * MAX_SPEED
    
    leftMotor.setVelocity(lS)
    rightMotor.setVelocity(rS)
    
    if t == 257 :
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        break
    
  
    


