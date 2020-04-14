import logging
import time
from abc import ABC, abstractmethod

import datetime 

class Component(ABC):
	"""
	A Component represents a runing or available instance.
	Here we need to think about 
	Normal Component lifecycle:
	|active | running | function_called | 
	|false  | false   | --------------- |
	|true   | false   | start           |
	|true   | false   | --------------- |
	|true   | true    | FB_ROS started  |
	|true   | true    | --------------- |
	|false  | true    | stop            |
	|false  | true    | --------------- |
	|false  | false   | FB_ROS exited   |
	"""
	def __init__ (self, cfg):
		self._MAX_INSTANCES = cfg['max_instances'] # is set by config
		self._NAME = cfg['name'] # is set by config ans must be unqiue

		self._active = False
		self._running = False
		self._instances = 0 
		self._id = -1 #instance_id will be set when started to run

		self._start_time = -1
		self._stop_time = -1
		

	def __str__(self):
		if self._running:
			return 'Comp: %15s,   ID: %3s,   Started: %10s ,   UpTime: %10s'%(\
				self._NAME, self.id,datetime.datetime.fromtimestamp(self._start_time) , datetime.timedelta(milliseconds=self.get_uptime()) ) # )
		else:
			return 'Comp: {}'.format(
				self._NAME )

	def instance_closed(self):

		if self._instances > 0: 
			self._instances -= 1
		else:
			logging.error('Confirmed closed instance but no instances is running')

	@property
	def instances(self):
		return self._instances

	@property
	def id(self):
		"""
		returns unique id which is set when instanciated
		"""
		return self._id


	def get_uptime(self):
		"""
		returns unique id which is set when instanciated
		"""
		if self._running:
			return time.time()  -self._start_time
		else:
			return 0
		
	@id.setter
	def id(self,id):
		self._id = id

	@property
	def name(self):
		"""
		returns unique id which is set when instanciated
		"""
		return self._NAME

	@property
	def max_instances(self):
		return self._MAX_INSTANCES

	@property
	def available(self):
		return self._instances <= self._MAX_INSTANCES

	@abstractmethod 
	def start(self):
		self._running = True
		self._start_time = time.time() 
		print("STARTED", self._NAME)
		#goes into container and launches/runs
		pass
	
	@abstractmethod 
	def stop(self):
		
		self._stop_time = time.time()
		#goes into container and stops
		pass
	
	@abstractmethod 
	def update(self):
		"""
		check if component is still running and returns result
		"""
		pass


class RosComponent(Component):
		
	def __init__ (self, cfg):
		"""
		ToDo: Check if cfg valid
		"""
		super(RosComponent, self).__init__(cfg)
		self._cfg = cfg

	def start(self):
		super(RosComponent, self).start()
		#goes into container and launches/runs
		pass

	def stop(self):
		super(RosComponent, self).stop()
		#goes into container and stops
		pass

	def update(self):
		pass


class UnityComponent(Component):

	def __init__ (self, cfg):
		"""
		ToDo: Check if cfg valid
		"""
		super(UnityComponent, self).__init__(cfg)
		self._cfg = cfg

	def start(self): 
		super(UnityComponent, self).start()
		#goes into container and launches/runs
		pass

	def stop(self):
		super(UnityComponent, self).stop()
		#goes into container and stops
		pass

	def update(self):
		pass
