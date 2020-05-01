import os 
from server import Server
from rest_res import ResInstances, ResInstance, ResAvailComps, ResAvailComp,ResInstanceUrdfDyn
from apscheduler.schedulers.background import BackgroundScheduler

import logging
import coloredlogs

logging = logging.getLogger(__name__)
coloredlogs.install(level='WARNING')

from flask import Flask
from flask_restful import Resource, Api

import signal
import time
import sys

if __name__ == '__main__':

	try:
		app = Flask(__name__)
		stop_signal = False

		api = Api(app)
		
		s = Server()

		def signal_handler(signal, frame):
			# your code here
			s.server_close()

			if len(s.get_instances()) == 0:
				sys.exit(0)
			else:
			
				logging.warning("Containers are still running")
				raise KeyboardInterrupt("STRG C")

		signal.signal(signal.SIGINT, signal_handler)
		

		api.add_resource(ResInstances,'/Instances',
			resource_class_kwargs={'server': s})
		api.add_resource(ResInstance,'/Instances/<int:inst_id>',
			resource_class_kwargs={'server': s})

		api.add_resource(ResAvailComps,'/AvailComps',
			resource_class_kwargs={'server': s})
		api.add_resource(ResAvailComp,'/AvailComps/<string:name>',
			resource_class_kwargs={'server': s})

		api.add_resource(ResInstanceUrdfDyn,'/Instances/<int:inst_id>/inst/urdf_dyn',
			resource_class_kwargs={'server': s})
		
		scheduler = BackgroundScheduler()

		def spin_job():
			s.spin()
			s.ros_publish()
			#print(s)
		job = scheduler.add_job(spin_job, 'interval', minutes=1/120)
		scheduler.start()

		s.start('ros-sharp-com')
		
		app.run(debug=False,host='0.0.0.0') 
	except Exception as e:
		print("ERROR", e)
