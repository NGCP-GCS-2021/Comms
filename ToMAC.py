class ToMAC:
	def __init__(self, stop, perform_stage, hiker_position, geofence_length, search_area_length, travel_to, drop_location, armed):
		self.stop = stop
		self.perform_stage = perform_stage
		self.hiker_position = hiker_position
		self.geofence_length = geofence_length
		self.search_area_length = search_area_length
		self.travel_to = travel_to
		self.drop_location = drop_location
		self.armed = armed

	def serialize(self):
		header = struct.pack("?B2f2I4f?", self.stop, self.perform_stage, self.hiker_position.lat, self.hiker_position.lng, self.geofence_length, self.search_area_length,
							self.travel_to.lat, self.travel_to.lng, self.drop_location.lat, self.drop_location.lng, self.armed)
		return (header)

	@classmethod
	def deserialize(cls, data: bytearray):
		raw = [*struct.unpack("?B2f2I4f?", data)]
		raw.insert(2, LatLng(raw.pop(2), raw.pop(2)))
		raw.insert(5, LatLng(raw.pop(5), raw.pop(5)))
		raw.insert(6, LatLng(raw.pop(6), raw.pop(6)))

		return cls(*raw)

	def __repr__(self):
		return str(self.__dict__)[1:-1]