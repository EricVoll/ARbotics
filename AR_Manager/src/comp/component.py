import logging
import time
from abc import ABC, abstractmethod

import datetime 
from dataclasses import dataclass
import docker 
from dataclasses import dataclass
from dataclasses_json import dataclass_json

@dataclass_json
@dataclass
class AvailCompInfo:
	pretty_name: str
	max_instances: int
	instances: int
	comp_type: str

@dataclass_json
@dataclass
class InstInfo:
	inst_id : int
	start_time: float
	stop_time: float
	active: bool
	running: bool
	killed: bool
	stop: bool
	urdf_dyn: str

@dataclass_json
@dataclass
class RosCompInfo:
	urdf_stat: str
	docker: dict

class Instance():
	def __init__(self, comp, inst_id):
		"""
		"""
		self._comp = comp #base component
		if self._comp.comp_type == 'unity':
			urdf_dyn = 'empty'
		elif self._comp.comp_type == 'ros':
			urdf_dyn = self._comp.get_data()['comp']['urdf_stat']

		self._ii = InstInfo(
			inst_id = inst_id,
			start_time= time.time(),
			stop_time= None,
			active = True, 
			running = False,
			killed = False,
			stop = False,
			urdf_dyn= urdf_dyn)

		self._container = self._comp.start()
	
	def __str__(self):
			return 'Comp: %15s,   ID: %3s,   Started: %10s ,   UpTime: %10s'%(\
				self._comp.name, self._ii.inst_id, datetime.datetime.fromtimestamp(self._ii.start_time) , datetime.timedelta(milliseconds=self.get_uptime()) )

	def get_uptime(self):
		"""
		returns unique id which is set when instanciated
		"""
		if self._ii.active:
			return time.time() - self._ii.start_time
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

	def stop(self):
		try:
			#print('My continaer is: ', self._container.id)			
			#self._container.stop()
			self._container.kill()
			self._container.remove()

			#print("Told container to stop")
		except:
			pass
			#print("Object already destroyed")
		

	def update_urdf_dyn(self, data):
		self._ii.urdf_dyn = data
	
	def get_data(self):
		#.to_dict()
		d = {'inst': self._ii.to_dict(), 'comp' : self._comp.get_data()['comp']  }
		return d

	def update(self):
		try: 
			self._container.reload()
			if self._container.status == 'exited':
				self._ii.running = False
			else:
				self._ii.running = True
			#print('My continaer is: ', self._container.id, self._container.status)
		
		except:
			self._ii.running = False
		return self._ii.running
		

class Component(ABC):
	"""
	a component defines how to actually interact with a container
	"""
	def __init__ (self, cfg, docker_client):
		self._aci = AvailCompInfo(
			comp_type= cfg['comp_type'], 
			pretty_name = cfg['pretty_name'],
			max_instances = cfg['max_instances'],
			instances = 0
		)
		self.docker_client = docker_client

	def __str__(self):
		return 'Comp: %15s, Instances: %3s'%(self._aci.pretty_name, self._aci.instances)

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
		return self._aci.pretty_name

	@property
	def max_instances(self):
		return self._aci.max_instances

	@property
	def available(self):
		return self._aci.instances <= self._aci.max_instances

	@property
	def comp_type(self):
		return self._aci.comp_type

	@abstractmethod 
	def start(self):
		self.add_instance()
		print("STARTED: ", self._aci.pretty_name)
		#goes into container and launches/runs
	
	@abstractmethod 
	def stop(self):
		print("STOP: ", self._aci.pretty_name)

		#goes into container and stops
	
	@abstractmethod 
	def update(self):
		pass
	
	@abstractmethod 
	def get_data(self):
		#.to_dict()
		d = {'comp': self._aci.to_dict()}
		return d




class RosComponent(Component):
		
	def __init__ (self, cfg, docker_client):
		"""
		ToDo: Check if cfg valid
		"""
		super(RosComponent, self).__init__(cfg, docker_client)
		self._rci = RosCompInfo(
			urdf_stat = cfg['urdf']['stat'],
			docker = cfg['docker'])

	def start(self):

		super(RosComponent, self).start()
		self._rci.docker['command']= '''bash -c '%s' '''%self._rci.docker['command']
		print('START ROS CONTAINER')
		print(self._rci.docker['image'], self._rci.docker['network'],self._rci.docker['ports'], self._rci.docker['volumes'] )
		container = self.docker_client.containers.run(** self._rci.docker)
		print('executed docker run got container:', container)
		
		return container 
		#goes into container and launches/runs

	def stop(self):

		super(RosComponent, self).stop()
		#goes into container and stops

	def update(self):

		super(RosComponent, self).update()

	def get_data(self):
		d = {'comp': dict(self._aci.to_dict(), **self._rci.to_dict())}
		return d

class UnityComponent(Component):

	def __init__ (self, cfg, docker_client):
		"""
		ToDo: Check if cfg valid
		"""
		super(UnityComponent, self).__init__(cfg, docker_client)

	def start(self): 
		super(UnityComponent, self).start()
		#goes into container and launches/runs

	def stop(self):
		super(UnityComponent, self).stop()
		#goes into container and stops

	def update(self):
		super(UnityComponent, self).update()

	def get_data(self):
		return super(UnityComponent, self).get_data()
	