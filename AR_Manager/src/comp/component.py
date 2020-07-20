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
	"""Instance is based on a Component Object
	"""	
	def __init__(self, comp, inst_id):
		"""Initalization of the Instance

		Parameters
		----------
		comp : Component
				base component
		inst_id : int
				unique instance ID
		"""	

		self._comp = comp
		if self._comp.comp_type == 'unity':
			urdf_dyn = 'empty'
		elif self._comp.comp_type == 'ros':
			urdf_dyn = self._comp.get_data()['comp']['urdf_stat']

		#use dataclass to store data of instance
		self._ii = InstInfo(
			inst_id = inst_id,
			start_time= time.time(),
			stop_time= None,
			active = True, 
			running = False,
			killed = False,
			stop = False,
			urdf_dyn= urdf_dyn)

		#calls start methode of base_component which returns the 
		#container reference the instance is running in
		self._container = self._comp.start()
	
	def __str__(self):
			return 'Comp: %15s,   ID: %3s,   Started: %10s ,   UpTime: %10s'%(\
				self._comp.name, self._ii.inst_id, datetime.datetime.fromtimestamp(self._ii.start_time) , datetime.timedelta(milliseconds=self.get_uptime()) )

	def get_uptime(self):
		"""Operation time of instance

		Returns
		-------
		float
				Operation time in sec
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
		"""Base component name 

		Returns
		-------
		string
				name of base component
		"""		
		return self._comp.name

	def remove(self):
		self._comp.instance_closed()

	def stop(self):
		"""Tries to kill the instances docker-container and removes it.
		"""		
		try:
			self._container.kill()
			self._container.remove()
		except:
			pass

	def update_urdf_dyn(self, data):
		"""Updates the dynamic URDF. 
		Can be used to add and remove tooltips

		Parameters
		----------
		data : string
				new urdf as string
		"""
		self._ii.urdf_dyn = data
	
	def get_data(self):
		"""Instance and base component data summary

		Returns
		-------
		dict
				contains instance data and base component data
		"""		
		return {'inst': self._ii.to_dict(), 'comp' : self._comp.get_data()['comp']  }

	def update(self):
		"""Checks if docker container of instance is still running

		Returns
		-------
		bool
				indicates if instance is still running
		"""	
		try: 
			self._container.reload()
			if self._container.status == 'exited':
				self._ii.running = False
			else:
				self._ii.running = True
		except:
			self._ii.running = False
		return self._ii.running
		
class Component(ABC):
	"""Abstract Component Class which defines the minimal methods and functions that need to be implemented
	Implements basic component behavior common for all Components

	Parameters
	----------
	ABC : metaclass
			--
	"""	
	def __init__ (self, cfg, docker_client):
		"""Initalization of component

		Parameters
		----------
		cfg : dict
				contains all releant informaion which is stored in AvailCompInfo dataclass
		docker_client : DockerClient
				Reference to host docker client to start and stop instance docker containers 
		"""		
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
			logging.error('Received confirmation of a closed instance but no instances is running')

	def add_instance(self):
		self._aci.instances += 1

	@property
	def instances(self):
		return self._aci.instances
	
	@property
	def name(self):
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

	@abstractmethod 
	def stop(self):
		print("STOP: ", self._aci.pretty_name)

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
		"""Init

		Parameters
		----------
		cfg : dict
				contains all relevant component info
		docker_client : DockerClient
				DockerClient reference to start instances
		"""
		super(RosComponent, self).__init__(cfg, docker_client)
		self._rci = RosCompInfo(
			urdf_stat = cfg['urdf']['stat'],
			docker = cfg['docker'])

	def start(self):
		"""start docker container

		Returns
		-------
		Container
				Container reference
		"""		
		super(RosComponent, self).start()
		self._rci.docker['command']= '''bash -c '%s' '''%self._rci.docker['command']
		#print(self._rci.docker['image'], self._rci.docker['network'],self._rci.docker['ports'], self._rci.docker['volumes'] )
		container = self.docker_client.containers.run(** self._rci.docker)
		#print('executed docker run got container:', container)
		
		return container 

	def stop(self):
		"""stop container
		"""
		super(RosComponent, self).stop()
		
	def update(self):
		"""updates component
		"""
		super(RosComponent, self).update()

	def get_data(self):
		"""get data of component

		Returns
		-------
		dict
				component data based of AvailableCompInfo dataclass
		"""		
		d = {'comp': dict(self._aci.to_dict(), **self._rci.to_dict())}
		return d

class UnityComponent(Component):

	def __init__ (self, cfg, docker_client):
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
	