import logging
import time
from abc import ABC, abstractmethod

import datetime 
from dataclasses import dataclass


@dataclass
class AvailCompInfo:
	PRETTY_NAME: str
	max_instances: int
	urdf_content: str
	urdf_tooltip: str
	docker_info: str
	instances : 0

@dataclass
class InstInfo:
	inst_id : int
	start_time: float
	stop_time: float
	active: bool
	running: bool
	killed: bool
	stop: bool
	

class Instance():
	def __init__(self, comp, inst_id):
		"""
		an instances manages a container.

		if active instance is doing stuff 
		if running container is running

		if killed container is closed 
		if stop (instances tries to stop container)

		"""
		self._comp = comp #base component
		self._ii = InstInfo(
			inst_id = inst_id,
			start_time= time.time(),
			stop_time= None,
			active = True, 
			running = False,
			killed = False,
			stop = False 	)
	
	def __str__(self):
			return 'Comp: %15s,   ID: %3s,   Started: %10s ,   UpTime: %10s'%(\
				self._comp.name, self._ii.inst_id, datetime.datetime.fromtimestamp(self._ii.start_time) , datetime.timedelta(milliseconds=self.get_uptime()) )

	def get_uptime(self):
		"""
		returns unique id which is set when instanciated
		"""
		if self._ii.active:
			return time.time()  -self._ii.start_time
		else:
			return 0

	@property
	def id(self):
		return self._ii.inst_id

	@property
	def name(self):
		"""
		returns unique id which is set when instanciated
		"""
		return self._comp.name

	def remove(self):
		self._comp.instance_closed()

	def start(self):
		pass

	def stop(self):
		pass

	def update(self):
		return True
		

class Component(ABC):
	"""
	a component defines how to actually interact with a container
	"""
	def __init__ (self, cfg):
		self._aci = AvailCompInfo(
			PRETTY_NAME = cfg['pretty_name'],
			max_instances = cfg['max_instances'],
			urdf_content = cfg['urdf_content'],
			urdf_tooltip = cfg['urdf_tooltip'],
			docker_info = cfg['docker_info'],
			instances = 0
		)

	def __str__(self):
		return 'Comp: %15s, Instances: %3s'%(self._aci.PRETTY_NAME, self._aci.instances)

	def instance_closed(self):

		if self._aci.instances > 0: 
			self._aci.instances -= 1
		else:
			logging.error('Confirmed closed instance but no instances is running')

	def add_instance(self):
		self._aci.instances += 1

	@property
	def instances(self):
		return self._aci.instances

	@property
	def name(self):
		"""
		returns unique id which is set when instanciated
		"""
		return self._aci.PRETTY_NAME

	@property
	def max_instances(self):
		return self._aci.max_instances

	@property
	def available(self):
		return self._aci.instances <= self._aci.max_instances


	@abstractmethod 
	def start(self):
		
		print("STARTED", self._aci.PRETTY_NAME)
		#goes into container and launches/runs
	
	@abstractmethod 
	def stop(self):
		print("STOP", self._aci.PRETTY_NAME)
	
		#goes into container and stops
	
	@abstractmethod 
	def update(self):
		pass

import docker 

class RosComponent(Component):
		
	def __init__ (self, cfg):
		"""
		ToDo: Check if cfg valid
		"""
		super(RosComponent, self).__init__(cfg)

	def start(self):

		super(RosComponent, self).start()
		#goes into container and launches/runs

	def stop(self):

		super(RosComponent, self).stop()
		#goes into container and stops

	def update(self):

		super(RosComponent, self).update()


class UnityComponent(Component):

	def __init__ (self, cfg):
		"""
		ToDo: Check if cfg valid
		"""
		super(UnityComponent, self).__init__(cfg)

	def start(self): 
		super(UnityComponent, self).start()
		#goes into container and launches/runs

	def stop(self):
		super(UnityComponent, self).stop()
		#goes into container and stops

	def update(self):
		super(UnityComponent, self).update()




#class AvailCompInfo():
if __name__ == '__main__':
	inst = AvailCompInfo('s1',2,'s3','s5','s5',1)

	inst.docker_info = 'assda'
	print(inst.docker_info)
	print(inst)
