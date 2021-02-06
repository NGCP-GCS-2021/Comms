import time
from digi.xbee.devices import DigiMeshDevice
import xbee

comm_port = "/dev/ttyUSB0"
baud_rate = "9600"

device = DigiMeshDevice(port=comm_port, baud_rate=baud_rate)
device.open()

print("This device's name: ", device.get_node_id())


def sample_callback(message):
    print('Received: ', message.data, '\nFrom:', message.remote_device.get_node_id())
    print(xbee.ToGCS.deserialize(message.data))
    
device.add_data_received_callback(sample_callback)

try:
    while True:
        print('Waiting....')
        time.sleep(1)
except KeyboardInterrupt:
    print('stopping')
finally:
    device.del_data_received_callback(sample_callback)