"""
Development test suite for UGV Software Team 1
"""

import time, math, requests
import xbee
import maestro
from .. import MPU, motor_control

RGHT = 0
LEFT = 1
BOTH = 2
FWD  = 0
REV  = 1

motor = MotorClass() # Motor object 
# mpuObj = mpuClass.mpuClass_2_5() mpu Object 

comm_port = "/dev/ttyUSB0"  # Changes based on where XBee is instantiated (Max is working on it)
                            # is not affected by reboots, only unplugging devices
baud_rate = "9600"

device = DigiMeshDevice(port=comm_port, baud_rate=baud_rate)
device.open()

servo = maestro.Controller(ttyStr='/dev/ttyACM0')
arms = 0.0
claws = 0.0
max_speed = 10
conversionValue = 2

servoMinRange = 448
servoMaxRange = 2554

def sample_callback(message):
    print('Received: ', message.data, '\nFrom:', message.remote_device.get_node_id())
    print(xbee.ToERU.deserialize(message.data))

device.add_data_received_callback(sample_callback)

try:
    while True:
        print('Waiting....')
        time.sleep(1)
except KeyboardInterrupt:
    print('stopping')
finally:
    device.del_data_received_callback(sample_callback)

telemetry_data = ToGCS(0, 0, Orientation(0,0,0), LatLng(0.0, 0.0), 0.98, True, 0, False, LatLng(0,0), 0, False, True)
gcs_lock = threading.Lock()
gcs_addr = None
geoFence = []

hiker_pos = LatLng(0,0)
current_pos = LatLng(35.082094, -120.512722)    # CPP lol
current_state = 0

packet_buffer = b''
packet_counter = 0

def packet_received_with(packet):
    print('Received packet from ', packet.remote_device.get_node_id())
    global gcs_addr 
    global packet_counter
    global packet_buffer
    global current_state
    global hiker_position
    global man_ctrl

    with gcs_lock:
        gcs_addr = packet.remote_device.get_64bit_addr()
    data = None

    if packet_counter is 0:
        packet_counter = struct.unpack("I", packet.data[:4])[0] -1
        data = packet.data[4:]
        print("expecting ", packet_counter," packets")
        packet_buffer = b''
    else:
        packet_counter -= 1
        data = packet.data

    packet_buffer += data

    if packet_counter is 0:
        with xbee.read_lock: # Acquire lock to read command data from GCS  ||  Read data from GCS
            command_data = ToERU.deserialize(packet_buffer)
            # geoFence = command_data.geo_fence

            if command_data.stop:
                print("STOPPING AT ", current_pos)     # Emergency Stop
            hiker_pos = command_data.hiker_position    # Use command_data.(whatever) to receive whatever data is needed
            current_state = command_data.perform_state # How many states are there? 
            man_ctrl = command_data.manual_control    # Manual Control Data from XBee
            
#Pass data to GCS
def transmit_packet():
    with telemetry_data.lock: # Acquire lock to update telemetry data
        telemetry_data.hiker_positon = hiker_pos
        telemetry_data.gps = current_pos
        telemetry_data.orientation.yaw += 1
        telemetry_data.battery -= 0.0001
        telemetry_data.current_state = current_state

    if gcs_addr:
        telemetry_data.serialize().transmit(device, gcs_addr)
        print("Transmitting")

    time.sleep(0.25)

def setServo(servoNum, minimum, maximum) :
    servo.setRange(servoNum,minimum,maximum)
    print(f"The target =  {servo.getPosition(servoNum)}.")
    print(f"The target is moving = {servo.isMoving(servoNum)}")
    servo.setAccel(servoNum,0)
    servo.setSpeed(servoNum,15)

def clamp(value, minimum, maximum):
    if(value < minimum):
        return minimum
    if(value > maximum):
        return maximum
    return value

def MotorMove(servoNumber, target):
    print("Clamped Value: ", clamp(target, servoMinRange, servoMaxRange))
    servo.setTarget(servoNumber, clamp(target, servoMinRange, servoMaxRange) * conversionValue)

try:
    transmitThread.start()

    setServo(0,servoMinRange*2,servoMaxRange*2)
    setServo(1,servoMinRange*2,servoMaxRange*2)

    while True:

        # ================== Receive data from GCS ==================
        device.add_data_received_callback(packet_received_with)

        if current_state is 1: # (1 == Ready/Idle & Waiting Command)
            print("Awaiting command data")

        # TODO: implement the rest of the states (not 2: Landing Seq nor 3: Landed)

        elif current_state is 4: # Possible Change: elif current_state is 4 (4 == Waypoint Nav/Nav to Hiker)
           print("Performing normal operations for state ", current_state)
         #   move_vector = hiker_pos - current_pos
         #   print("Need to move ", move_vector)
         #   move_vector.lat /= 1000
         #   move_vector.lng /= 1000
         #   current_pos += move_vector
            # waypoint nav function idk 

        elif current_state is 5: # (5 == Upside Down)
            print("Upside Down")

        elif current_state is 6: # 6 == Manual Control (Nav and Payload Retrieval))
            vertical = man_ctrl.vertical
            horizontal = man_ctrl.horizontal
            speed = man_ctrl.speed 
            arms = man_ctrl.arm
            claws = man_ctrl.claw
            
            # motor.percentage(RGHT, FWD, speed*max(-1, min(horizontal+vertical, 1)))
            # motor.percentage(LEFT, FWD, speed*max(-1, min(-horizontal+vertical, 1)))
            
            stepSize = 300
            targetArms = int(servo.getPosition(0)/2)
            targetClaw = int(servo.getPosition(1)/2)

            if arms < 0:
                targetArms -= int(stepSize * abs(arms))
            elif arms > 0:
                targetArms += int(stepSize * abs(arms))
            if claws < 0:
                targetClaws -= int(stepSize * abs(claws))
            elif claws > 0:
                targetClaws += int(stepSize * abs(claws))
                
            MotorMove(0, targetArms)
            MotorMove(1, targetClaw)
            time.sleep(.5)

        elif current_state is 7:  # (7 == Hiker Secured)
            print("Hiker Secured")

        elif current_state is 8: # (8 == Hiker Not Secured)
               print("Hiker Not Secured")

        elif current_state is 9: # (9 == Navigating to EVAC site)
               print("Navigating to EVAC site")
            #    Waypoint Nav? Manual? idk

        elif current_state is 10: # (10 == Hiker Delivered)
               print("Hiker Delivered")

        elif current_state is 11: # (11 == Navigating to Ground Control)
               print("Navigating to Ground Control")
            #    Waypoint Nav? Manual? idk

        elif current_state is -1: # (-1 == Error)
               print("Error")

        # ================== Transmit data to GCS ==================
        transmitThread = xbee.TransmitThread(transmit_packet)

        time.sleep(1)

except KeyboardInterrupt:
    print('Stopping')
finally:
    device.del_data_received_callback(packet_received_with)
    motor.destroy()
    servo.close 