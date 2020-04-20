import sys
sys.path.append('/home/jonas/Documents/Repos/3D_Vision_AR_RobotVis/AR_Manager/src')
sys.path.append('/home/jonas/Documents/Repos/3D_Vision_AR_RobotVis/AR_Manager/src/')

import logging
import yaml
from comp import RosComponent, UnityComponent, Instance
import copy
from flask import Flask
from flask_restful import Resource, Api
from apscheduler.schedulers.background import BackgroundScheduler

import docker
"""
This will be implemented based on flask and REST-API
"""

class Server():
	"""
	implements all functionality
	"""
	def __init__(	self, \
								cfg_ros_comp = 'src/cfg/cfg_ros_comp.yml', \
								cfg_unity_comp = 'src/cfg/cfg_unity_comp.yml'):
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
		self._docker_client = docker.from_env()

		self._avail_comps = cfg_to_comps( cfg_ros_comp, self._docker_client, comp_type='ros' )
		self._avail_comps += cfg_to_comps( cfg_unity_comp, self._docker_client, comp_type='unity')
		
		#containes all running components
		#components are always copied and then they are instances
		#instances might be ros or unity comp
		self._instances = [] 
		self._instance_counter = 0 



	def __str__(self):
		string = "Server Summary:\n"
		
		string += "="*45+' Instances '+'='*45 +'\n'
		for inst in self._instances: 
			string += ' {}\n'.format(inst)
		string += '-'*101+'\n'+'\n'

		string += "="*45+' Comp '+'='*45 +'\n'
		for comp in self._avail_comps: 
			string += ' {}\n'.format(comp)
		string += '-'*101+'\n'+'\n'

		return string

	def server_close(self):
		
		pass

	def start(self, comp_name):

		for comp in self._avail_comps:
			if comp.name == comp_name:

				self._instances.append( Instance(comp=comp,inst_id=self._instance_counter) )
			

				
				return self._instance_counter
		logging.error('Cant find component')
		return -1

	def remove_instance(self, inst_id):  		
		for inst in self._instances:
			if inst.id() == inst_id:
				inst.remove()
				
				self._instances.remove(inst)
			
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
		print("spin")
def cfg_to_comps(cfg_file, docker_client, comp_type='ros'):

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

	comps = []
	for single_cfg in data['components']:
		if comp_type == 'ros':
			comps.append( RosComponent(single_cfg, docker_client) )
		elif comp_type == 'unity':
			comps.append( UnityComponent(single_cfg, docker_client) )
		else:
			logging.warning('Comp type ({}) not defined'.format(comp_type))
	return comps




if __name__ == '__main__':
	app = Flask(__name__)
	api = Api(app)

	# api.add_resource(Instances,'/Instances/<string:name>')
	# api.add_resource(RosComponents,'/RosComponents/<string:name>')


	s = Server()
	s.start("UR3")
	s.start("UR5")

	scheduler = BackgroundScheduler()
	def spin_job():
		s.spin()
	job = scheduler.add_job(spin_job, 'interval', minutes=1/60)
	scheduler.start()

	app.run(debug=True) 
	
	print(s)