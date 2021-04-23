"""
Development script for UGV Software Team 1 to test on Max's local Xbee
"""

import time, math, requests
import xbee
from .. import MPU

comm_port = "/dev/ttyUSB0"  # changes based on where XBee is instantiated (Max is working on it)
                            # is not affected by reboots, only unplugging devices
baud_rate = "9600"

device = DigiMeshDevice(port=comm_port, baud_rate=baud_rate)
device.open()

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
current_pos = LatLng(35.082094, -120.512722)
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
#            geoFence = command_data.geo_fence

            if command_data.stop:
                print("STOPPING AT ", current_pos)     # Emergency Stop
            hiker_pos = command_data.hiker_position     # use command_data.(whatever) to receive whatever data is needed
            current_state = command_data.perform_state

device.add_data_received_callback(packet_received_with)

#Pass data to GCS
def transmit_packet():
    with telemetry_data.lock: # Acquire lock to update telemetry data
        telemetry_data.hiker_positon = hiker_pos
        telemetry_data.gps = current_pos
        telemetry_data.orientation.yaw += 1
        telemetry_data.battery -= 0.0001
        telemetry_data.current_state = current_state

# TODO: set up sampleGCS.py to work locally with Brendon's hardware
# TODO: receive and manipulate data from MPU and send to GCS
# TODO: set up parsing of GCS data packets

    if gcs_addr:
        telemetry_data.serialize().transmit(device, gcs_addr)
        print("Transmitting")

    time.sleep(0.25)

transmitThread = xbee.TransmitThread(transmit_packet)

try:
    transmitThread.start()

    while True:
        if current_state is 0:
            print("Awaiting command data")
        else:
            print("Performing normal operations for state ", current_state)
            move_vector = hiker_pos-current_pos
            print("Need to move ", move_vector)
            move_vector.lat /= 1000
            move_vector.lng /= 1000
            current_pos += move_vector

        current_state = not current_state

        time.sleep(1)
except KeyboardInterrupt:
    print('Stopping')
finally:
    device.del_data_received_callback(packet_received_with)






# inside of navigation function:
    #     geoFence = command_data.geo_fence
