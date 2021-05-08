import time
from digi.xbee.devices import DigiMeshDevice
import xbee
from xbee import ToERU, ToMAC, ToGCS, Orientation, LatLng, ManualControl, Geofence, SearchArea
import threading
import struct

comm_port = "/dev/ttyUSB1" # can be swapped out for "/dev/ttyUSB0" for serial connection
baud_rate = "9600"

device = DigiMeshDevice(port=comm_port, baud_rate=baud_rate)
device.open()

network = device.get_network()
network.start_discovery_process()

while network.is_discovery_running():
    time.sleep(.01)

devices = {dev.get_node_id():dev._64bit_addr for dev in network.get_devices()}
devices[device.get_node_id()] = device._64bit_addr

print("This device's name: ", device.get_node_id())
print("Discovered ", devices)

geo_bounds = [Geofence(True, [LatLng(1,0),LatLng(0,1),LatLng(-1,0),LatLng(0,-1)])]
geo_bounds.append(Geofence(False, [LatLng(1,1),LatLng(2,1),LatLng(2,-1),LatLng(1,-1)]))

area = SearchArea([LatLng(1,0),LatLng(0,1),LatLng(-1,0),LatLng(0,-1)])

hiker_pos = LatLng(35.083519, -120.534821)

state = 1
stop = False

packet_buffers = {}
packet_counters = {}

def packet_received(packet):
    print('Received packet from ', packet.remote_device.get_node_id())
    global packet_buffers 
    global packet_counters
    global current_state
    global hiker_position

    dev_addr = packet.remote_device.get_64bit_addr()
    data = None

    if dev_addr not in packet_counters or packet_counters[dev_addr] is 0:
        packet_counters[dev_addr] = struct.unpack("I", packet.data[:4])[0] -1 
        data = packet.data[4:]
        print("expecting ", packet_counters[dev_addr]," packets")
        packet_buffers[dev_addr] = b''
    else:
        packet_counters[dev_addr] -= 1
        data = packet.data

    packet_buffers[dev_addr] += data

    if packet_counters[dev_addr] is 0:
        with xbee.read_lock: # Acquire lock to read command data from GCS
            telemetry_data = ToGCS.deserialize(packet_buffers[dev_addr])
            print(packet.remote_device.get_node_id(), ": ", telemetry_data)


device.add_data_received_callback(packet_received)

try:
    while True:
        cmd = input("Enter command (+,-,s,e,m): ")
        if cmd is '+':
            state += 1
            print("New state", state)
        if cmd is '-':
            state -= 1
            print("New state", state)
        if cmd is 's':
            stop = not stop
        if cmd is 'e':
            ToERU(stop, state, hiker_pos, geo_bounds, LatLng(5,5), LatLng(5.5,5.5), False, None, True).serialize().transmit(device, devices['eru'])
        if cmd is 'm':
            ToMAC(None, state, hiker_pos, geo_bounds, [area], LatLng(5,5), LatLng(5.5,5.5), True).serialize().transmit(device, devices['mac'])
        
except KeyboardInterrupt:
    print('Stopping')
finally:
    device.del_data_received_callback(packet_received)
