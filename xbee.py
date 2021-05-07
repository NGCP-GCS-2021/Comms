from __future__ import annotations
import time
import typing
import struct
from digi.xbee.devices import DigiMeshDevice
from enum import Enum
from threading import Lock, Thread
from math import ceil

'''
Python docs:
https://xbplib.readthedocs.io/en/latest/getting_started_with_xbee_python_library.html

'''

## Setup for the device

# comm_port = "/dev/ttyAMA0"
# baud_rate = "9600"


# device = DigiMeshDevice(port=comm_port, baud_rate=baud_rate)
# device.open()

# network = device.get_network()

# network.start_discovery_process()

# while network.is_discovery_running():
#     time.sleep(.01)

# devices = {dev.get_node_id():dev._64bit_addr for dev in network.get_devices()}
# devices[device.get_node_id()] = device._64bit_addr

# devices = {'gcs': bytearray(b'\x00\x13\xa2\x00A\xc1d\xa4'), 'eru': bytearray(b'\x00\x13\xa2\x00A\xc1\x86D')}

read_lock = Lock()

## Data Classes

class TransmitThread (Thread):
    def __init__(self, update_function):
        Thread.__init__(self)
        self.function = update_function

    def run(self):
        while True:
            self.function()

class Packet:
    def __init__(self, data: bytearray):
        self.data = data

    def transmit(self, device, recipient):
        packet_size = int.from_bytes(device.get_parameter('NP'), byteorder='big')-5
        count = ceil(len(self.data)/float(packet_size))
        print("Sending ", count," packets of ", packet_size, " length")
        self.data = struct.pack('I', int(count)) + self.data

        packets = [self.data[i:i+packet_size] for i in range(0,len(self.data), packet_size)]
        for packet in packets:
            device.send_data_64(recipient, packet)

    def __repr__(self):
        return str(self.__dict__)[1:-1]



# ToGCS used to transmit all data needed to the GCS from a given vehicle
class ToGCS:
    def __init__(self, altitude: float, speed: float, orientation: Orientation, gps: LatLng, battery: float, sensors_ok: bool, current_state: int, state_complete: bool,
                hiker_position: LatLng, status: int, propulsion: bool, geofence_compliant: bool):
        self.altitude = altitude
        self.speed = speed
        self.orientation = orientation
        self.gps = gps
        self.battery = battery
        self.sensors_ok = sensors_ok
        self.current_state = current_state
        self.state_complete = state_complete
        self.hiker_position = hiker_position
        self.status = status
        self.propulsion = propulsion
        self.geofence_compliant = geofence_compliant

        self.lock = Lock()
    
    def serialize(self):
        header = struct.pack("8f?B?2fB2?", self.altitude, self.speed, self.orientation.pitch, self.orientation.roll, self.orientation.yaw, self.gps.lat,
                            self.gps.lng, self.battery, self.sensors_ok, self.current_state, self.state_complete, self.hiker_position.lat, self.hiker_position.lng,
                            self.status, self.propulsion, self.geofence_compliant)
        
        return Packet(header)

    @classmethod
    def deserialize(cls, data: bytearray):
        raw = [*struct.unpack("8f?B?2fB2?", data)]
        raw.insert(2, Orientation(raw.pop(2), raw.pop(2), raw.pop(2)))
        raw.insert(3, LatLng(raw.pop(3), raw.pop(3)))
        raw.insert(8, LatLng(raw.pop(8), raw.pop(8)))

        return cls(*raw)


    def __repr__(self):
        return str(self.__dict__)[1:-1]

# ToMAC
class ToMAC:
    def __init__(self, stop_coord, perform_state: int, hiker_position: LatLng, geofence: list[Geofence], search_area: list[SearchArea],
                 travel_to: LatLng, drop_location: LatLng, armed: bool):
        if stop_coord:
            self.stop_coord = stop_coord
            self.stop = True
        else:
            self.stop = False
        self.perform_state = perform_state
        self.hiker_position = hiker_position
        self.geofence_length = len(geofence)
        self.geofence = geofence
        self.search_area_length = len(search_area)
        self.search_area = search_area
        self.travel_to = travel_to
        self.drop_location = drop_location
        self.armed = armed

        self.lock = Lock()

    def serialize(self):
        header = struct.pack("?B2f2I4f?", self.stop, self.perform_state, self.hiker_position.lat, self.hiker_position.lng, self.geofence_length, self.search_area_length,
                            self.travel_to.lat, self.travel_to.lng, self.drop_location.lat, self.drop_location.lng, self.armed)
        body = bytearray()
        for geofence_obj in self.geofence:
            body += struct.pack("?B",geofence_obj.keep_in, geofence_obj.coord_length)
            for point in geofence_obj.coords:
                body += struct.pack("2f", point.lat, point.lng)

        for search_area_obj in self.search_area:
            body += struct.pack("B", search_area_obj.coord_length)
            for point in search_area_obj.coords:
                body += struct.pack("2f", point.lat, point.lng)
        
        if self.stop:
            body += struct.pack("3f", *self.stop_coord)

        header += body

        return Packet(header)

    @classmethod
    def deserialize(cls, data: bytearray):
        header = data[:37]
        body = data[37:]
        raw = [*struct.unpack("?B2f2I4f?", header)]
        raw.insert(2, LatLng(raw.pop(2), raw.pop(2)))
        raw.insert(5, LatLng(raw.pop(5), raw.pop(5)))
        raw.insert(6, LatLng(raw.pop(6), raw.pop(6)))
        stop = raw[0]
        geofence_length = raw.pop(3)
        search_area_length = raw.pop(3)

        geofence = []
        for i in range(geofence_length):
            keep, coord_length = struct.unpack("?B", body[:2])
            coords = []
            body = body[2:]
            for j in range(coord_length):
                coords += [LatLng(*struct.unpack("2f", body[:8]))]
                body = body[8:]

            geofence += [Geofence(keep, coords)]
        
        search_area = []
        for i in range(search_area_length):
            (coord_length, ) = struct.unpack("B", body[:1])
            coords = []
            body = body[1:]
            for j in range(coord_length):
                coords += [LatLng(*struct.unpack("2f", body[:8]))]
                body = body[8:]

            search_area += [SearchArea(coords)]

        if stop:
            raw[0] = struct.unpack("3f", body)
        else:
            raw[0] = None

        raw.insert(3, search_area)
        raw.insert(3, geofence)

        return cls(*raw)

    def __repr__(self):
        return str(self.__dict__)[1:-1]

# ToERU
class ToERU:
    def __init__(self, stop, perform_state: int, hiker_position: LatLng, geofence: list[Geofence], ez_zone: LatLng, travel_to: LatLng,
                 execute_loading: bool, manual_control: ManualControl, armed: bool):
        self.stop = stop
        self.perform_state = perform_state
        self.hiker_position = hiker_position
        self.geofence_length = len(geofence)
        self.geofence = geofence
        self.ez_zone = ez_zone
        self.travel_to = travel_to
        self.execute_loading = execute_loading
        if manual_control:
            self.control_data = manual_control
            self.manual_control = True
        else:
            self.manual_control = False
        self.armed = armed

        self.lock = Lock()

    def serialize(self):
        header = struct.pack("?B2fI4f3?", self.stop, self.perform_state, self.hiker_position.lat, self.hiker_position.lng, 
                            self.geofence_length, self.ez_zone.lat, self.ez_zone.lng, self.travel_to.lat, self.travel_to.lng, self.execute_loading, 
                            self.manual_control, self.armed)
        body = bytearray()
        for geofence_obj in self.geofence:
            body += struct.pack("?B",geofence_obj.keep_in, geofence_obj.coord_length)
            for point in geofence_obj.coords:
                body += struct.pack("2f", point.lat, point.lng)
        
        if self.manual_control:
            body += struct.pack("5f", self.control_data.vertical, self.control_data.horizontal, self.control_data.arm, self.control_data.claw, self.control_data.speed)

        header += body

        return Packet(header)

    @classmethod
    def deserialize(cls, data: bytearray):
        header = data[:35]
        body = data[35:]
        raw = [*struct.unpack("?B2fI4f3?", header)]
        raw.insert(2, LatLng(raw.pop(2), raw.pop(2)))
        raw.insert(4, LatLng(raw.pop(4), raw.pop(4)))
        raw.insert(5, LatLng(raw.pop(5), raw.pop(5)))
        stop = raw[0]
        geofence_length = raw.pop(3)
        manual_control = raw.pop(6)

        geofence = []
        for i in range(geofence_length):
            keep, coord_length = struct.unpack("?B", body[:2])
            coords = []
            body = body[2:]
            for j in range(coord_length):
                coords += [LatLng(*struct.unpack("2f", body[:8]))]
                body = body[8:]
            geofence += [Geofence(keep, coords)]

        control_data = None
        if manual_control:
            control_data = ManualControl(*struct.unpack("5f",body))

        raw.insert(6, control_data)
        raw.insert(3, geofence)

        return cls(*raw)

    def __repr__(self):
        return str(self.__dict__)[1:-1]

# ToMEA
class ToMEA:
    def __init__(self, stop_coord, perform_state: int, geofence: list[Geofence], ez_zone: LatLng, 
                travel_to: LatLng, execute_loading: bool, armed: bool):
        if stop_coord:
            self.stop_coord = stop_coord
            self.stop = True
        else:
            self.stop = False
        self.perform_state = perform_state
        self.geofence_length = len(geofence)
        self.geofence = geofence
        self.ez_zone = ez_zone
        self.travel_to = travel_to
        self.execute_loading = execute_loading
        self.armed = armed

        self.lock = Lock()
        
    def serialize(self):
        header = struct.pack("?BI4f2?", self.stop, self.perform_state, self.geofence_length, 
                            self.ez_zone.lat, self.ez_zone.lng, self.travel_to.lat, self.travel_to.lng, 
                            self.execute_loading, self.armed)
        body = bytearray()
        for geofence_obj in self.geofence:
            body += struct.pack("?B",geofence_obj.keep_in, geofence_obj.coord_length)
            for point in geofence_obj.coords:
                body += struct.pack("2f", point.lat, point.lng)
        
        if self.stop:
            body += struct.pack("3f", *self.stop_coord)

        header += body

        return Packet(header)

    @classmethod
    def deserialize(cls, data: bytearray):
        header = data[:26]
        body = data[26:]
        raw = [*struct.unpack("?BI4f2?", header)]
        raw.insert(3, LatLng(raw.pop(3), raw.pop(3)))
        raw.insert(4, LatLng(raw.pop(4), raw.pop(4)))
        stop = raw[0]
        geofence_length = raw.pop(2)

        geofence = []
        for i in range(geofence_length):
            keep, coord_length = struct.unpack("?B", body[:2])
            coords = []
            body = body[2:]
            for j in range(coord_length):
                coords += [LatLng(*struct.unpack("2f", body[:8]))]
                body = body[8:]

            geofence += [Geofence(keep, coords)]

        if stop:
            raw[0] = struct.unpack("3f", body)
        else:
            raw[0] = None

        raw.insert(2, geofence)

        return cls(*raw)

    def __repr__(self):
        return str(self.__dict__)[1:-1]


## Helper methods

## Defines custom data structures needed
class LatLng():
    lat: float
    lng: float

    def __init__(self, lat: float=0, lng: float=0):
        self.lat = lat
        self.lng = lng

    def __add__(self, o):
        return LatLng(self.lat + o.lat, self.lng + o.lng)

    def __sub__(self, o):
        return LatLng(self.lat - o.lat, self.lng - o.lng)

    def __EQ__(self, o):
        return self.lat == o.lat and self.lng == o.lng

    def __NE__(self, o):
        return self.lat != o.lat or self.lng != o.lng

    def __repr__(self):
        return f"{{Latitude: {self.lat}, Longitude: {self.lng}}}"

class Geofence():

    def __init__(self, keep_in: bool, coords: list[LatLng]):
        self.keep_in = keep_in
        self.coords = coords
        self.coord_length = len(coords)

    def __iadd__(self, o):
        self.coords += o.coords
        return self

    # TODO: Implement method to calculate if a point is inside the given coords
    def inside(self, point: LatLng):
        return False 

    def __repr__(self):
        return '{' + str(self.__dict__)[1:-1] + '}'

class SearchArea():

    def __init__(self, coords: list[LatLng]):
        self.coords = coords
        self.coord_length = len(coords)
    
    # TODO: Implement method to calculate if a point is inside the given coords
    def inside(self, point: LatLng):
        return False 

    def __repr__(self):
        return '{' + str(self.__dict__)[1:-1] + '}'

class Orientation():
    pitch: float
    roll: float
    yaw: float

    def __init__(self, pitch: float, roll: float, yaw: float):
        self.pitch = pitch
        self.roll = roll
        self.yaw = yaw

    def __repr__(self):
        return f"{{Pitch: {self.pitch}*, Roll: {self.roll}*, Yaw: {self.yaw}*}}"

class ManualControl():
    vertical: float
    horizontal: float
    arm: float
    claw: float
    speed: float

    def __init__(self, vertical, horizontal, arm, claw, speed):
        self.vertical = vertical
        self.horizontal = horizontal
        self.arm = arm
        self.claw = claw
        self.speed = speed

    def __repr__(self):
        return str(self.__dict__)[1:-1]


## CODE FOR TESTING
orientation = Orientation(1,1,1)
gps = LatLng(2,2.0005)
hiker = LatLng(1.5, 1.5)

togcs = ToGCS(1.5,10,orientation,gps,.9,True,3,False,hiker,1,True,True)

geo_bounds = [Geofence(True, [LatLng(1,0),LatLng(0,1),LatLng(-1,0),LatLng(0,-1)])]
geo_bounds.append(Geofence(False, [LatLng(1,1),LatLng(2,1),LatLng(2,-1),LatLng(1,-1)]))

geo_empty = []
mancon = ManualControl(.8,-.1,0,0,.8)

area = SearchArea([LatLng(1,0),LatLng(0,1),LatLng(-1,0),LatLng(0,-1)])

tomac = ToMAC(None, 1, LatLng(1,1), geo_bounds, [area], LatLng(.5,.5), LatLng(.6,0), False)
tomac2 = ToMAC((2.2,3.3,500), 1, LatLng(1,1), geo_empty, [area,area], LatLng(.5,.5), LatLng(.6,0), False)

toeru = ToERU(False, 1, LatLng(1,1), geo_bounds, LatLng(5,5), LatLng(5.5,5.5), False, mancon, True)
toeru2 = ToERU(False, 1, LatLng(1,1), geo_empty, LatLng(5,5), LatLng(5.5,5.5), False, None, True)

tomea = ToMEA(None, 1, geo_empty, LatLng(6.5,7.5), LatLng(1.1,1.1), False, True)
tomea2 = ToMEA((5.5,5.5,1000), 1, geo_bounds, LatLng(6.5,7.5), LatLng(1.1,1.1), False, True)
