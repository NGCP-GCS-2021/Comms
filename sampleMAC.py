import time
from digi.xbee.devices import DigiMeshDevice
import xbee
from xbee import ToMAC, ToGCS, Orientation, LatLng, ManualControl, Geofence
import threading
import struct

comm_port = "/dev/ttyUSB0" # can be swapped out for "/dev/ttyUSB0" for serial connection
baud_rate = "9600"

device = DigiMeshDevice(port=comm_port, baud_rate=baud_rate)
device.open()

print("This device's name: ", device.get_node_id())

current_pos = LatLng(33.933923, -117.630088)

telemetry_data = ToGCS(100, 0, Orientation(0,0,0), current_pos, 0.98, True, 0, False, LatLng(0,0), 0, False, True)
gcs_lock = threading.Lock()
gcs_addr = None

hiker_pos = LatLng(33.933923, -117.630088)
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
        with xbee.read_lock: # Acquire lock to read command data from GCS
            command_data = ToMAC.deserialize(packet_buffer)
            if command_data.stop:
                print("STOPPPING AT ", current_pos)
            hiker_pos = command_data.hiker_position
            current_state = command_data.perform_state

device.add_data_received_callback(packet_received_with)

def transmit_packet():
    global hiker_pos
    with telemetry_data.lock: # Acquire lock to update telemetry data
        telemetry_data.hiker_position = hiker_pos
        telemetry_data.gps = current_pos
        telemetry_data.orientation.yaw += 1
        telemetry_data.battery -= 0.0001
        telemetry_data.current_state = current_state
        if gcs_addr:
            telemetry_data.serialize().transmit(device, gcs_addr)
            print("Transmitting:  ",telemetry_data)

    time.sleep(1)

transmitThread = xbee.TransmitThread(transmit_packet)

try:
    transmitThread.start()

    step = .00008

    delta_lat = -step
    delta_lng = step

    while True:
        if current_state is 0:
            print("Awaiting command data")
        else:
            print("Performing normal operations for state ", current_state)

        if current_pos.lat >= 33.935026:
            delta_lat = -step
        elif current_pos.lat <= 33.934990:
            delta_lat = step
        if current_pos.lng >= -117.632780:
                delta_lng = -step
        elif current_pos.lng <= -117.632712:
            delta_lng = step
        
        current_pos.lat += delta_lat
        current_pos.lng += delta_lng

        time.sleep(1)
except KeyboardInterrupt:
    print('Stopping')
finally:
    device.del_data_received_callback(packet_received_with)
