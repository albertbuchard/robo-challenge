import paho.mqtt.client as mqtt
import json

SERVER = "127.0.0.1"
PORT = 1883

PLAYER_NAME = "foo"

GAME_STATE = 0 # 0 is waiting, 1 is playing


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
    print(msg.topic)
    obj = json.loads(msg.payload.decode("utf-8"))
    print(obj)
    if GAME_STATE == 1:
        xDiff = obj["robot"]["x"] - obj["points"]["x"]
        yDiff = obj["robot"]["y"] - obj["points"]["y"]
        targetAngle = math.abs(math.tan(xDiff/yDiff))
        if yDiff > 0:
            if xDiff < 0:
                targetAngle = 360 - targetAngle
        else:
             if xDiff > 0:
                 targetAngle = 180 - targetAngle
             else:
                 targetAngle = 180 + targetAngle
        client.publish('robot/process', '{"command": "forward", "args": 600}')
    elif (GAME_STATE == 0) and (msg.topic=='players/foo/incoming'):
        client.publish('players/' + PLAYER_NAME , '{"command": "start"}')
        GAME_STATE = GAME_STATE + 1

    #client.publish('players/' + PLAYER_NAME , '{"command": "backward", "args": 100}', qos=0, retain=False)

    # TODO: implement algorithm
'''

def forward(value):
    print '{\"command\": forward\", \"args\": ' + str(value) + '}'
    return '{"command": forward", "args": ' + str(value) + '}'

def backward(value):
    return '{"command": backward", "args": ' + str(value) + '}'

def right(degree):
    return '{"command": right", "args": ' + str(degree) + '}'

def left(degree):
    return '{"command": left", "args": ' + str(degree) + '}'
'''



if __name__ == '__main__':

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
