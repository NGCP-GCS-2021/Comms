import time
from digi.xbee.devices import DigiMeshDevice

comm_port = "/dev/ttyAMA0"
baud_rate = "9600"

device = DigiMeshDevice(port=comm_port, baud_rate=baud_rate)
device.open()

network = device.get_network()

network.start_discovery_process()

while network.is_discovery_running():
    time.sleep(.125)

devices = {dev.get_node_id():dev for dev in network.get_devices()}

print("This device's name: ", device.get_node_id())
print("Other devices: ",', '.join(list(devices.keys())))