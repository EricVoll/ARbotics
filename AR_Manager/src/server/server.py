import logging
from client import Client
import yaml
from component import RosComponent
import copy
class Server():
	"""
	implements all functionality
	"""
	def __init__(	self, \
								cfg_ros_comp = 'src/config/cfg_ros_comp.yml', \
								cfg_unity_comp = 'src/config/cfg_unity_comp.yml', \
								client = None):
		"""
		cfg_ros_comp = stores all information about available ros comp
		  is read in when initalized
		cfg_unity_comp = stores all information about available unity comp
		  is read in when initalized
		every client is able to start or stop components

		Idee of components: 
		YML-files -> ComponentList
		RosComponent UR3
		RosComponent UR5

		call create_component(UR3)
			if UR3 in list:
				if CompInstanceCounter < UR3.cfg:
					copy UR3 to instances list
					instance UR3.start()
					increase ComponentInstance counter

		Extension later: Allow groups in config (set of mutiple instances) (nice to have)
		In most cases this will be done by roslaunch directly
		"""

		self._client_list = []
		if isinstance( client, Client):
			self._client_list.append(client)
		elif isinstance( client, None):
			logging.info('Started Server withour Client')
		else:
			logging.warning("Init without client: \
				Tried to object type",type(client), 'as a Client')

		self._ros_comp_list = cfg_to_components(cfg_ros_comp)
		self._unity_comp_list = cfg_to_components(cfg_unity_comp)
		
		#containes all running components
		#components are always copied and then they are instances
		#instances might be ros or unity comp
		self._instances = [] 
		self._instance_counter = 0 

	def __str__(self):
		return "Server total started instances: {} \n \
		 				Registered ROS Comp: {} \n \
						Registered Unity Comp: {} \n \
						Running Instances: {}".format(self._instance_counter, 
						self._ros_comp_list, 
						self._unity_comp_list, 
						self._instances)

	def server_close(self):
		
		pass

	def add_client(self, client):
		
		if isinstance( client, Client):
			self._client_list.append(client)
		else:
			logging.warning("Tried to add None Client Type object")

	def remove_client(self, client_id):

		for c in self._client_list:
			if c.id() == client_id:
				self._client_list.remove(c)

	@property
	def client_ids(self):
  		
		ids = []
		for c in self._client_list:
			ids.append(c.id)
		return ids

	def my_clients_do_something(self):
  		
		for cl in self._client_list:
			cl.do_something()


	def start(self, comp_name):

		for comp in self._ros_comp_list + self._unity_comp_list:
			if comp.name == comp_name:
				self._instances.append( copy.deepcopy(comp) )
				self._instance_counter += 1
				self._instances[-1].id = self._instance_counter
				self._instances[-1].start()
				return self._instance_counter
		logging.error('Cant find component')
		return -1

	def remove_instance(self, inst_id):
  		
		for inst in self._instances:
			if inst.id() == inst_id:
				temp = inst.name
				self._instances.remove(inst)
				for comp in self._ros_comp_list + self._unity_comp_list:
					if comp.name == temp:
						comp.instance_closed()
				logging.error('Cant find component corrosponding to instance')
		logging.error('Cant find instance id')

	def spin(self):
		"""
		updates all component states
		spin must be called cyclic
		removes stoped instances
		"""
		for inst in self._instances:
			running = inst.update()
			if not running: 
				self.remove_instance(inst.id)

def cfg_to_components(cfg_file):
	"""
	reads in yml-file: 
	creates component objects based on this file

	ToDo: Check if cfg file is valid
	"""
	#open the template
	with open(cfg_file) as f:
		data = yaml.load(f, Loader=yaml.FullLoader)
		print(data)
	print(data['components'])

	comp_list = []
	for single_cfg in data['components']:
		comp_list.append( RosComponent(cfg = single_cfg) )

	return comp_list