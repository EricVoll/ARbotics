import sys
sys.path.append('/home/jonas/Documents/Repos/3D_Vision_AR_RobotVis/AR_Manager/src')
sys.path.append('/home/jonas/Documents/Repos/3D_Vision_AR_RobotVis/AR_Manager/')

from flask_restful import Resource, reqparse

class ResInstances(Resource):
	def __init__(self, server):
		self.s = server

	def get(self):
		return {'Running':True,
						'Uptime':100,
						'Started':'time'} #returns the id of the instance
	def delete(self):
		#delete all runing comps
		return {'Running':True,
					'Uptime':100,
					'Started':'time'} #returns the id of the instance
	

class ResInstance(Resource):
	def __init__(self, server):
		self.s = server

	def get(self,inst_id):
		parser = reqparse.RequestParser()
		parser.add_argument('int_bar', type=int)
		args = parser.parse_args()

		return {'Running':True,
						'Uptime':100,
						'Started':'time'} #returns the id of the instance
	
	def post(self,inst_id):
		
		return {'Running':True,
						'Uptime':100,
						'Started':'time'} #returns the id of the instance
	
	def delete(self,inst_id):
		return {'Running':True,
						'Uptime':100,
						'Started':'time'} #returns the id of the instance


class ResAvailComps(Resource):

	def __init__(self, server):
		self.s = server

	def get(self):
		return {'Running':True,
						'Uptime':100,
						'Started':'time'} #returns the id of the instance

	def post(self):
		#add mutiple avail res
		return {'instance_id': 100} #returns the id of the instancep


class ResAvailComp(Resource):

	def __init__(self, server):
		self.s = server

	def get(self,name):
	
		return {'Running':True,
						'Uptime':100,
						'Started':'time'} #returns the id of the instance

	def post(self,name):
		#add mutiple avail res
		return {'instance_id': 100} #returns the id of the instancep
	
	def delete(self,name):
		#delete avail 
		return {	'suc': True,
							'instance_id_stopped': 123}
