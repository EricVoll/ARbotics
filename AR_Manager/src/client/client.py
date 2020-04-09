
from abc import ABC, abstractmethod

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

class RosClient(Client):
	"""
	Implementation based on services
	"""
	def __init__(self,id = None):

		super(RosClient, self).__init__(id)

	def do_something(self):
		
		print("doing something ros client")
		return


class CommandLineClient(Client):

	def __init__(self,id = None):
		
		super(CommandLineClient, self).__init__(id)

	def do_something(self):
		
		print("doning somethin command line")
		return