import paho.mqtt.client as mqtt
import json

SERVER = "127.0.0.1"
PORT = 1883

PLAYER_NAME = "TheRegressor"

GAME_STATE = 0 # 0 is waiting, 1 is playing

game_data = {}
game_log = []
i = 0

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

    print(msg.topic)
    obj = json.loads(msg.payload.decode("utf-8"))
    print(obj)

    game_log.append(obj)


    if GAME_STATE == 1:
        # Pick a target
        # Make a decision on the angle if not turned before
        # Start moving
        client.publish('robot/process', '{"command": "forward", "args": 600}')
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
        elif (msg.topic == 'players/%s/game' % PLAYER_NAME):
            print("********** STORED GAME_DATA ************")
            game_data = obj

    i += 1
    if (i>=10):
        i = 0
        with open("data.txt","w") as f: #in write mode
            f.write("{}".format(game_log))

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
