
from abc import ABC, abstractmethod
import rospy

"""



"""




def spawn_robot ():
	return rospy_tutorials.srv.SpawnRobotResponse("DONE")


class Client(ABC):
	"""
	interface for all sockets
	"""
	def __init__(self,id = None):
		
		self._id = id

	@property
	def id(self):
		print("propert was called", self._id)
		return self._id

	@abstractmethod 
	def do_something(self):
		
		pass




def on_shutdown_function():
	print('node got shut down')

class RosClient(Client):
	"""
	Implementation Consideration:
		based on services -> would imply some meaningful feedback
		stateless might be superior therefore use topics instead to which the RosClient is listening to
	Topics should be created by the RosSharp Bridge. 
	"""
	def __init__(self,id = None):
  		
		super(RosClient, self).__init__(id)
		rospy.init_node('ar_manager_client')
		rospy.on_shutdown(on_shutdown_function)
		rospy.signal_shutdown("RosClient shut down itself manual")

		while not rospy.is_shutdown():
			pass #do some work

	def do_something(self):
		
		print("doing something ros client")
		return

	def run(self):
		pass


class CommandLineClient(Client):

	def __init__(self,id = None):
		
		super(CommandLineClient, self).__init__(id)

	def do_something(self):
		
		print("doning somethin command line")
		return