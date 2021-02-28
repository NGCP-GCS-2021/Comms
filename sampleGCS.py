import time
from digi.xbee.devices import DigiMeshDevice
import xbee
from xbee import ToERU, ToGCS, Orientation, LatLng, ManualControl, Geofence
import threading

comm_port = "/dev/ttyUSB0" # can be swapped out for "/dev/ttyUSB0" for serial connection
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

geo_bounds = [Geofence(True, [LatLng(1,0),LatLng(0,1),LatLng(-1,0),LatLng(0,-1)])]
geo_bounds.append(Geofence(False, [LatLng(1,1),LatLng(2,1),LatLng(2,-1),LatLng(1,-1)]))

hiker_pos = LatLng(35.083519, -120.534821)

state = 1
stop = False

def packet_received(packet):
    print('Received packet from ', message.remote_device.get_node_id())
    with xbee.read_lock: # Acquire lock to read command data from GCS
        telemetry_data = ToGCS.deserialize(packet.data)
        print(message.remote_device.get_node_id(),": ", telemetry_data)


device.add_data_received_callback(packet_received)

try:
    while True:
        cmd = input("Enter command (+,-,s,t): ")
        if cmd is '+':
            state += 1
            print("New state", state)
        if cmd is '-':
            state -= 1
            print("New state", state)
        if cmd is 's':
            stop = not stop
        if cmd is 't':
            ToERU(stop, state, hiker_pos, geo_bounds, LatLng(5,5), LatLng(5.5,5.5), False, None, True).serialize().transmit(device, devices['eru'])
        
except KeyboardInterrupt:
    print('Stopping')
finally:
    device.del_data_received_callback(packet_received)