import os 
from server import Server
from rest_res import ResInstances, ResInstance, ResAvailComps, ResAvailComp
from apscheduler.schedulers.background import BackgroundScheduler

import logging
from flask import Flask
from flask_restful import Resource, Api

import signal
import time
import sys

if __name__ == '__main__':

	app = Flask(__name__)
	

	api = Api(app)
	
	s = Server()

	def signal_handler(signal, frame):
			# your code here
		s.server_close()
		if len(s.get_instances()) == 0:
			sys.exit(0)
		else:
			print("Container still running")
	signal.signal(signal.SIGINT, signal_handler)
	
	
	print('RUNRUN!RUN!RUN!RUN!RUN!RUN!RUN!RUN!!')

	api.add_resource(ResInstances,'/Instances/',
		resource_class_kwargs={'server': s})
	api.add_resource(ResInstance,'/Instances/<string:inst_id>',
		resource_class_kwargs={'server': s})

	api.add_resource(ResAvailComps,'/AvailComp/',
		resource_class_kwargs={'server': s})
	api.add_resource(ResAvailComp,'/AvailComp/<string:name>',
		resource_class_kwargs={'server': s})
	
	# api.add_resource(ResComps,'/Comp/',
	# 	resource_class_kwargs={'server': s})

	scheduler = BackgroundScheduler()

	s.start("UR5")

	def spin_job():
		s.spin()
		print(s)
		print(s.get_instace(0))
	job = scheduler.add_job(spin_job, 'interval', minutes=1/60)
	scheduler.start()

	
	app.run(debug=True) 