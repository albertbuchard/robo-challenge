import paho.mqtt.client as mqtt
import json
import math as m
import numpy as np
import time

#SERVER = "10.10.10.30"
SERVER = "127.0.0.1"
PORT = 1883

PLAYER_NAME = "TheRegressor"

GAME_STATE = 0 # 0 is waiting, 1 is playing
DISTANCE_BETWEEN_WEELS = 25210.14298575622/90
CONVERT_COUNT_DIST = 3/10
goodPoints = np.zeros(shape=(45,2))
badPoints = np.zeros(shape=(5,2))
targetPoint = 0 #index of goodPoints which is the current target
game_data = {}
game_log = []
i = 0

WAIT_FOR_EXEC_FLAG = False
targets = []
obstacles = []
coordT = []



class Robot(object):
    """docstring for ClassName"""
    def __init__(self):
        self.x = 640
        self.y = 480
        self.rightCount = 0
        self.leftCount = 0
        self.angle = 0
        self.currentTarget = {}
        self.goodPoints = []
        self.badPoints = []
        self.visitedGoodPoints = []
        self.visitedBadPoints = []

        self.beginAt = time.time()



    # Mouvements
    def moveForward(self, value):
        client.publish('robot/process', '{"command": "forward", "args": ' + str(value) + '}', qos=0, retain=False)

    def moveBackward(self, value):
        client.publish('robot/process', '{"command": "backward", "args": ' + str(value) + '}', qos=0, retain=False)

    def turnRight(self, degree):
        client.publish('robot/process', '{"command": "right", "args": ' + str(degree) + '}', qos=0, retain=False)

    def turnLeft(self, degree):
        client.publish('robot/process', '{"command": "left", "args": ' + str(degree) + '}', qos=0, retain=False)

    def stop(self):
        client.publish('robot/process', '{"command": "stop"}', qos=0, retain=False)

    def increment(self, vRight, vLeft):
        dRight = vRight - self.rightCount
        dLeft = vLeft - self.leftCount
        dCenter = (dRight+dLeft)*CONVERT_COUNT_DIST/2
        dAngle = (dLeft-dRight)/DISTANCE_BETWEEN_WEELS
        self.x = self.x + dCenter*m.cos(self.angle*m.pi/180)
        self.y = self.y - dCenter*m.sin(self.angle*m.pi/180)
        self.angle = self.angle + dAngle*180/m.pi
        self.rightCount = vRight
        self.leftCount = vLeft

    def updateCoord(self, coordDict):
        self.x = coordDict['x']
        self.y = coordDict['y']

    def updateAngle(self, angle):
        self.angle = angle

    def checkArrival(self,coord):
        global WAIT_FOR_EXEC_FLAG
        global targets
        global coordT
        print((self.x-coord[0])*(self.x-coord[0]) + (self.y-coord[1])*(self.y-coord[1]))
        if (self.x-coord[0])*(self.x-coord[0]) + (self.y-coord[1])*(self.y-coord[1]) < 100:
            coordT = targets.pop()
            WAIT_FOR_EXEC_FLAG = False



    # Trig functions
    def getTargetAngle(self, xTarget, yTarget):
        xDiff = xTarget - self.x
        yDiff = yTarget - self.y
        if xDiff == 0:
            if yDiff == 0:
                targetAngle = 0
            else:
                targetAngle = np.sign(yDiff)*90
        else:
            targetAngle = m.atan(yDiff/xDiff)*180/m.pi
            if xDiff < 0 :
                targetAngle = np.sign(yDiff)*180 + targetAngle
        return (-targetAngle)

    '''
    # Path finding
    def getNonVisitedGoodPoints():
        # loop through positive points
        points = []
        for (index, point) in enumerate(self.goodPoints):
            if index in self.visitedGoodPoints:
                points.append(point)
            pass

        return points

    def getNearestNeighbour(fromPoint = [self.x, self.y]):
        self.currentTarget
        minDistance = 100000
        nextSucker = None
        for point in self.getNonVisitedGoodPoints()):
            distance= np.sqrt(np.dot((point-fromPoint),(point-fromPoint)))
            if minDistance < distance:
                nextSucker=point

        return nextSucker;
    '''

    def gotToPoint(self, coordT):
        robot.stop()
        xT = coordT[0]
        yT = coordT[1]
        aT = robot.getTargetAngle(xT,yT)
        #print('angle To Point')
        print(robot.x)
        print(robot.y)
        print(robot.angle)
        print(aT)
        print((aT-robot.angle)%360)
        if (aT-robot.angle)%360<180:
            print(abs(int((aT-robot.angle)%360)))
            robot.turnRight(abs(int((aT-robot.angle)%360)))

        else:
            print(abs(int((aT-robot.angle)%360-360)))
            robot.turnLeft(abs(int((aT-robot.angle)%360-360)))
        #robot.turnLeft(90)
        distanceToTarge = m.sqrt((xT-robot.x)*(xT-robot.x)+(yT-robot.y)*(yT-robot.y))
        robot.moveForward(int(distanceToTarge/CONVERT_COUNT_DIST))


    '''
    def getRemainingTime():
        timeSpentInMs = time.time() - self.beginAt
        remainingTime = 120000 - timeSpentInMs

        if remainingTime < 0:
            remainingTime = 0
            
        return remainingTime


    '''



# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe('players/' + PLAYER_NAME + '/#')
    client.subscribe('robot/state')
    client.subscribe('robot/error')
    client.subscribe('players/' + PLAYER_NAME + '/incoming')
    client.subscribe('players/' + PLAYER_NAME + '/game')
    client.publish('players/' + PLAYER_NAME , '{"command": "register"}')



# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global GAME_STATE
    global game_log
    global i
    global targets
    global WAIT_FOR_EXEC_FLAG
    global coordT

    #print(msg.topic)
    obj = json.loads(msg.payload.decode("utf-8"))
    #print(obj)

    game_log.append(obj)


    if GAME_STATE == 2:
        if msg.topic == 'robot/state':
            robot.increment(obj['right_motor'], obj['left_motor'])
            robot.updateAngle(obj['angle'])

        elif (msg.topic == 'players/%s/game' % PLAYER_NAME):
            print("********** STORED GAME_DATA ************")
            game_data = obj
            print(coordT)
            #print(WAIT_FOR_EXEC_FLAG)
            # print(obj['robot'])
            # print(robot.x)
            # print(robot.y)
            # print(robot.angle)
            robot.updateCoord(obj['robot'])

            '''
            if WAIT_FOR_EXEC_FLAG:
                robot.checkArrival(coordT)
            else:
                coordT = targets.pop()
                robot.gotToPoint(coordT)
                WAIT_FOR_EXEC_FLAG = True
            '''

            robot.gotToPoint(coordT)
            robot.checkArrival(coordT)




        
        

    elif GAME_STATE == 1:
        if (msg.topic == 'players/%s/game' % PLAYER_NAME):
            generateTargetsAndObstacles(obj['points'])
            coordT = targets.pop()
            GAME_STATE = 2


    elif (GAME_STATE == 0):
        if ((msg.topic=='players/%s/incoming' % PLAYER_NAME) and ("command" in obj)):
            if (obj['command'] == "start"):
                print("********** RECEIVED START FROM SERVER ************")
                client.publish('players/' + PLAYER_NAME , '{"command": "start"}')
                GAME_STATE = 1
            elif (obj['command'] == "finished"):
                print("********** RECEIVED FINISHED FROM SERVER ************")
                GAME_STATE = 0
                client.disconnect()
                exit()


    i += 1
    if (i>=10):
        i = 0
        with open("data.txt","w") as f: #in write mode
            f.write("{}".format(game_log))


def generateTargetsAndObstacles(points):
    global targets
    global obstacles
    for point in points:
        if point['score'] == 1:
            targets.append([point['x'], point['y']])
        else:
            obstacles.append([point['x'], point['y']])
    print(targets)
    print(obstacles)









if __name__ == '__main__':

    robot = Robot()
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    client.connect(SERVER, PORT, 60)

    # Blocking call that processes network traffic, dispatches callbacks and
    # handles reconnecting.
    # Other loop*() functions are available that give a threaded interface and a
    # manual interface.
    try:
        client.loop_forever()
    except (KeyboardInterrupt, SystemExit):
        print("Tearing down...")
        client.disconnect()
        raise
