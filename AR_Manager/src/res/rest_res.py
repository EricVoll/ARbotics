
	
class RosComponents(Resource):
	#is there an component available

	def get(self,name):
		#returns if a specific instance
		return {'Running':True,
						'Uptime':100,
						'Started':'time'} #returns the id of the instance
	
	#create a new component -> write flag to add to yml
	def post(self,name):
		#starts a specific instance
		return {'instance_id': 100} #returns the id of the instancep
		
	#delete component -> write flag to change yml
	def delete(self,name):
		#stop instance
		return {	'suc': True,
							'instance_id_stopped': 123}



class Instances(Resource):
	
	def get(self,name):
		#returns if a specific instance

		return {'Running':True,
						'Uptime':100,
						'Started':'time'} #returns the id of the instance


	def post(self,name):
		#starts a specific instance
		return {'instance_id': 100} #returns the id of the instancep
		
	def delete(self,name):
		#stop instance
		return {	'suc': True,
							'instance_id_stopped': 123}
