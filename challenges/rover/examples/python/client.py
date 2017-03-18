import paho.mqtt.client as mqtt
import json
import math as m
import numpy as np
import time

SERVER = "127.0.0.1"
PORT = 1883

PLAYER_NAME = "TheRegressor"

DISTANCE_BETWEEN_WHEELS = 25210.14298575622/90
CONVERT_COUNT_DIST = 3/10
goodPoints = numpy.zeros(shape=(45,2))
badPoints = numpy.zeros(shape=(5,2))
targetIndices = list(range(45)) #can be reordered
lastPoints
targetAngle = 0
game_data = {}
game_log = []
i = 0

WAIT_FOR_EXEC_FLAG = False


class Robot(object):
    """docstring for ClassName"""
    def __init__(self):
        self.x = 640
        self.y = 480
        self.rightCount = 0
        self.leftCount = 0
        self.angle = 0

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

    def setOdometry(self, vRight, vLeft):
        dRight = vRight - self.rightCount
        dLeft = vLeft - self.leftCount
        dCenter = (dRight+dLeft)*CONVERT_COUNT_DIST/2
        dAngle = (dLeft-dRight)/DISTANCE_BETWEEN_WEELS
        self.x = self.x + dCenter*m.cos(self.angle*m.pi/180)
        self.y = self.y - dCenter*m.sin(self.angle*m.pi/180)
        self.angle = self.angle + dAngle*180/m.pi
        self.rightCount = vRight
        self.leftCount = vLeft

    # Trig functions
    def getTargetAngle(self, xTarget, yTarget):
        xDiff = self.x - xTarget
        yDiff = self.y - yTarget
        targetAngle = abs(m.atan(yDiff/xDiff))
        if yDiff > 0:
            if xDiff < 0:
                targetAngle = 360 - targetAngle
        else:
            if xDiff > 0:
                targetAngle = 180 - targetAngle
            else:
                targetAngle = 180 + targetAngle

    def getNearestNeighbour():
        # self.currentTarget
        startingPoint = [self.x, self.y]
        minDistance = 100000
        nextSucker = None
        for index in targetIndices:
            distance = np.sqrt(np.dot((goodPoints[index]-fromPoint),(goodPoints[index]-startingPoint)))
            if distance < minDistance:
                nextSucker = point
                minDistance = distance

        return nextSucker;


    def checkArrival(points):
        goodCounter2 = 0
        for point in points:
            if points[i]["score"] == 1:
                goodCounter2 += 1
                if points[i]["collected"] != lastPoints[i]["collected"]:
                    list(filter((goodCounter2).__ne__, targetIndices))
                    setNextPath()
        lastPoints = points

    def setNextPath():
        xT = goodPoints[targetIndices[0]][0]
        yT = goodPoints[targetIndices[0]][1]
        self.setTargetAngle(xT,yT)
        print('angle To Point')
        print(targetAngle)
        self.turnRight((targetAngle-self.angle)%360)
        distanceToTarge = m.sqrt((xT-self.x)*(xT-self.x)+(yT-self.y)*(yT-self.y))
        self.moveForward(distanceToTarge/CONVERT_COUNT_DIST)

    def checkHeading(self):
        if abs(self.angle - targetAngle) > 5:
            setNextPath()

    def getRemainingTime():
        timeSpentInMs = time.time() - self.beginAt
        remainingTime = 120000 - timeSpentInMs

        if remainingTime < 0:
            remainingTime = 0

        return remainingTime

def generateTargets(points):
    goodCounter = 0
    badCounter = 0
    for point in points:
        if point["score"] == 1:
            goodPoints[goodCounter] = [point["x"], point["y"]]
            goodCounter += 1
        else:
            badPoints[badCounter] = [point["x"], point["y"]]
            badCounter += 1
    print("Good points:" + goodPoints)
    print("Bad points:" + badPoints)


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


GAME_STATE = 0 # 0 is waiting, 1 is playing
firstState = True
# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global game_log
    global i
    global WAIT_FOR_EXEC_FLAG

    #print(msg.topic)
    obj = json.loads(msg.payload.decode("utf-8"))
    #print(obj)

    game_log.append(obj)


    if GAME_STATE == 2:
        if msg.topic == 'robot/state':
            robot.setOdometry(obj['right_motor'], obj['left_motor'])
            if counter == 0:
                robot.setNextPath()
                firstState = False
            robot.checkHeading()

        elif (msg.topic == 'players/%s/game' % PLAYER_NAME):
            print("********** STORED GAME_DATA ************")
            robot.checkArrival(obj['points'])


            # game_data = obj
            # print(obj['robot'])
            # print(robot.x)
            # print(robot.y)
            # print(robot.angle)
            # robot.moveForward(20)
            # robot.turnRight(5)



        if WAIT_FOR_EXEC_FLAG:
            pass
            #robot.check()
        else:
            robot.gotToPoint()
            WAIT_FOR_EXEC_FLAG = True



    elif GAME_STATE == 1:
        if (msg.topic == 'players/%s/game' % PLAYER_NAME):
            generateTargets(obj['points'])
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
