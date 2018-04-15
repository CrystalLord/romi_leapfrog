import serial
import time
s

serial_port = serial.Serial('/dev/tty.usbserial-DN02BDUM', 9600)

BotA = '\x00\x0A'
BotC = '\x00\x0C'


def print_data(data):
    """
    This method is called whenever data is received
    from the associated XBee device. Its first and
    only argument is the data contained within the
    frame.
    """
    print(data)

def sendMotorCommand(RobotID, LPWM, RPWM):
    ''' Message example: LDIR LMOTOR RDIR RMOTOR '''
    # '$' denotes the beginning of a message and '@' denotes the end of a message
    if (LPWM < 0):
        LDIR = 0
    else:
        LDIR = 1

    if (RPWM < 0):
        RDIR = 0
    else:
        RDIR = 1
    RPWM = abs(RPWM)
    LPWM = abs(LPWM)

    command = '$M ' + str(LDIR) + ' ' + str(LPWM) + ' ' + str(RDIR) + ' ' + str(RPWM) + '@'
    xbee.tx(dest_addr = RobotID, data = command)



def requestUpdate(RobotID):
    ''' Message that request updates. Returns: FrontDist LeftDist RightDist RightWheel LeftWheel'''
    command = '$S @'
    xbee.tx(dest_addr = RobotID, data = command)
    update = xbee.wait_read_frame()
    data = update['rf_data'].decode().split(' ')[:-1]
    data = [int(x) for x in data]
    return data

def straightLine(EncDes, rightWheel):
    ''' Testing straightline movement. Match one wheel encoder to the other'''
    difference = EncDes - rightWheel
    Kp = 0.13
    return int(difference * Kp)

xbee = XBee(serial_port)

while True:
    try:
        UpdatedData = requestUpdate(BotC)
        leftEnc = UpdatedData[4]
        rightEnc = UpdatedData[3]
        frontDist = UpdatedData[2]

        if frontDist >= 700:
            sendMotorCommand(BotC, 0, 0)
        else:
            sendMotorCommand(BotC, 0, 0)

        print(UpdatedData)
        time.sleep(0.5)
    except KeyboardInterrupt:
        break

xbee.halt()
serial_port.close()