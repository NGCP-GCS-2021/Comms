class ToMEA:
	def __init__(self, , stop, perform_stage, geofence_length, ez_zone, travel_to, armed, execute_loading, geofence_point):
		self.undefined = undefined
		self.stop = stop
		self.perform_stage = perform_stage
		self.geofence_length = geofence_length
		self.ez_zone = ez_zone
		self.travel_to = travel_to
		self.armed = armed
		self.execute_loading = execute_loading
		self.geofence_point = geofence_point
		
	def serialize(self):
		header = struct.pack("?B2f2I4f?", self.stop, self.perform_stage, self.hiker_position.lat, self.hiker_position.lng, self.geofence_length, self.search_area_length,
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

		return Packet('bwb', header)

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