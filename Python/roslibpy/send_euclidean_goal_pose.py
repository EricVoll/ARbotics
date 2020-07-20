import os
import time
import roslibpy
import numpy as np
from scipy.spatial.transform import Rotation as R
import math
import requests
import json
from pynput.keyboard import Key, Listener
from rest_keyboard_interface import RestKeyboardListener

import base64
import matplotlib
#from matplotlib import plt
import matplotlib.pyplot as plt
from math import pi
import queue
from io import BytesIO
from functools import partial
from PIL import Image
import numpy


ip ='http://vservice-host-001.chera.ch' #127.0.0.1
ip_ros ='vservice-host-001.chera.ch' 
port_rest = 5000
port_rossharp = 9090
 
class Plot ():
	def __init__(self, line_cfgs):
		"""
		provide a list of line_names
		"""
		self.fig_joint = plt.figure(dpi=50)
		self.ax_joint = self.fig_joint.add_subplot(111)
		self.ax_joint.set_ylim(-pi,pi)
		self.ax_joint.set_xlim(0,101)
		self.ax_joint.set_ylabel("Radians")
		self.lines = []
		for line in line_cfgs:
			tmp, = self.ax_joint.plot([],line['color'],label=line['name'])
			self.lines.append(tmp)

		self.ax_joint.legend()
		self.fig_joint.canvas.draw()
		self.axbackground = self.fig_joint.canvas.copy_from_bbox(self.ax_joint.bbox)
		s, (width, height) = self.fig_joint.canvas.print_to_buffer()
		self.w = width
		self.h = height
		#plt.show(block=False)

	def update(self,x, ys):
		"""
		Input x is array of values for x-Axis
		ys is an list of arrays containing y values with same length as x
		"""
		if not (len(x) == len(ys[0])):
			raise TypeError()

		for i, line in enumerate(self.lines):
			line.set_data(x,ys[i])

		self.fig_joint.canvas.restore_region(self.axbackground)
		for i, line in enumerate(self.lines):
			self.ax_joint.draw_artist(line)
			
		self.fig_joint.canvas.blit(self.ax_joint.bbox)
		self.fig_joint.canvas.flush_events()

	def image_png(self):
		#im = fig2img(self.fig_joint)
		def fig2rgb_array(fig):
			buf = self.fig_joint.canvas.tostring_rgb() 
			return np.fromstring(buf, dtype=np.uint8).reshape(self.h, self.w, 3)

		#print(self.fig_joint.canvas.get_width_height())
		buf = fig2rgb_array(self.fig_joint)

		im = Image.fromarray(buf)
		#im.show()
		#print(self.fig_joint.canvas.get_width_height())
		#im = Image.frombytes('RGB', self.fig_joint.canvas.get_width_height(),self.fig_joint.canvas.tostring_rgb())
		img = BytesIO()
		im.save(img, "PNG")
		#plt.savefig(img, format='png', dpi=50,quality=1)
		#img.seek(0)
		return img


class PlotSub ():
	def __init__(self, line_cfgs):
		"""
		provide a list of line_namessub
		plot(2,1,1)
		xticks([]), yticks([])
		title('subplot(2,1,1)')
		plot(t,s)

		subplot(2,1,2)
		xticks([]), yticks([])
		title('subplot(2,1,2)')
		plot(t,s,'r-')
		"""
		self.fig_joint = plt.figure(dpi=50)
		self.ax_joint = []
		self.lines = []
		for i,line in enumerate(line_cfgs):

			self.ax_joint.append(self.fig_joint.add_subplot(len(line_cfgs),1,i+1)) 
			self.ax_joint[i].set_ylim(-pi,pi)
			self.ax_joint[i].set_xlim(0,101)
			self.ax_joint[i].set_ylabel(line['ylabel'])
			tmp, = self.ax_joint[i].plot([],line['color'],label=line['name'])
			self.lines.append(tmp)
			self.ax_joint[i].legend()


		self.fig_joint.canvas.draw()
		self.ax_back = []
		for ax in self.ax_joint:
			self.ax_back.append( self.fig_joint.canvas.copy_from_bbox(ax.bbox) )

		s, (width, height) = self.fig_joint.canvas.print_to_buffer()
		self.w = width
		self.h = height
		

	def update(self,x, ys):
		"""
		Input x is array of values for x-Axis
		ys is an list of arrays containing y values with same length as x
		"""
		if not (len(x) == len(ys[0])):
			raise TypeError()

		for i, line in enumerate(self.lines):
			line.set_data(x,ys[i])
		for a in self.ax_back:
			self.fig_joint.canvas.restore_region(a)
		for i, line in enumerate(self.lines):
			self.ax_joint[i].draw_artist(line)
			self.fig_joint.canvas.blit(self.ax_joint[i].bbox)

		self.fig_joint.canvas.flush_events()

	def image_png(self):
		def fig2rgb_array(fig):
			buf = self.fig_joint.canvas.tostring_rgb() 
			return np.fromstring(buf, dtype=np.uint8).reshape(self.h, self.w, 3)

		buf = fig2rgb_array(self.fig_joint)
		im = Image.fromarray(buf)
		img = BytesIO()
		im.save(img, "PNG")
		return img


def create_MultiArray32(data):
	dim = []
	for i in range(0,len(data.shape)):
		out = 'axis'+str(i)
		dim.append( {'label': out, 'size': data.shape[i], 'stride':1 })

	return { 'layout': {'dim': dim , 'data_offset':0}, \
			'data': data.tolist() }

def create_Pose(t,xyz):
	q = R.from_euler('xyz' , xyz).as_quat()
	return { 'position': {'x':t[0], 'y':t[1], 'z':t[2]} , 'orientation':{'x':q[0] ,'y':q[1],'z':q[2],'w':q[3] } }

def create_Img(data):
	"""creates ros compatible img
	Parameters
	----------
	data : numpy.array
		width height RGB
	"""
	header = {'seq': int(10), 'time':0, 'frame_id':0}
	height = int(data.shape[0])
	width = int(data.shape[1])
	encoding = 'rgb8'
	is_bigendian = int(0)
	step = int( 1 )
	data = data.tolist()

	msg = {'header':header,
	'heihgt': height,
	'width':width,
	'encoding':encoding,
	'is_bigendian':is_bigendian,
	'step':step,
	'data':data }
	return msg

def base64_string( img ):
	img_b64 = base64.b64encode(img.getvalue()).decode()
	msg = {'data': img_b64}
	return msg

def create_ModelState(data):
	model_name = 'robot'
	pose = create_Pose([0,0,0],[0,0,0,1])
	twist = {'linear':{'x':0,'y':0,'z':0}, 'angular':{'x':0,'y':0,'z':0}}
	reference_frame = 'reference_frame'
	return {
		'model_name':model_name,
		'pose':pose,
		'twist':twist,
		'reference_frame':reference_frame
	}

   
ros = roslibpy.Ros(host=ip_ros, port= port_rossharp)
ros.run()
xyz = []

talker = roslibpy.Topic(ros, '/euclidean_goal_pose', 'geometry_msgs/Pose')
talker_img_joint_position = roslibpy.Topic(ros, '/img/joint_position', 'String')
talker_img_tooltip_position = roslibpy.Topic(ros, '/img/tool_position', 'String')

def monitor_topic(topic,ros,size):
	lis = roslibpy.Topic(ros, topic,  ros.get_topic_type(topic))
	inp = [queue.Queue(maxsize=size)]
	def cb_topic (message, inp ):
		if inp[0].full():
			inp[0].get()
		if 'position' in message.keys() :
			inp[0].put(message['position'])
		
		if  'twist' in message.keys():
			lin = message['twist'][6][linear]
			inp[0].put([lin['x'],lin['y'],lin['z']])
	
	lis.subscribe( partial(cb_topic, inp = inp) )
	return inp


length = 100

scene_data = monitor_topic('/gazebo/link_states',ros,length)
joint_data = monitor_topic('/joint_states',ros,length)

joint_position_plot = Plot([{'name':'JointAngle-0','color':'r'} ,{'name':'JointAngle','color':'b'}])
tool_position_plot = PlotSub([{'name':'X-Tool','color':'r','ylabel':'Meter'} ,
											{'name':'Y-Tool','color':'g','ylabel':'Meter'} ,
											{'name':'Z-Tool','color':'b','ylabel':'Meter'}])


i = 0
amp = 0.4
keylis = RestKeyboardListener(ip_adr=ip, port_rest=str(port_rest),ros=ros )
keylis.listener.start()

def menu(fps):
	os.system('cls' if os.name == 'nt' else 'clear')
	l = 6
	l2 = 74
	print(''.ljust(l,'-') + '+'.ljust(l2,'-')+'+')
	print('Key:'.ljust(l, ' ') + '| Action:'.ljust(l2, ' ') + '|' )
	print(''.ljust(l,'-') + '+'.ljust(l2,'-')+'+')
	print('s'.ljust(l, ' ') + '| Adds Tooltip Tool-Position to UR5-Instance Dyn-URDF'.ljust(l2, ' ') + '|' )
	print('f'.ljust(l, ' ') +'| Adds Tooltip Joint-Position to UR5-Instance Dyn-URDF'.ljust(l2, ' ') + '|' )
	print('d'.ljust(l, ' ') +'| Resets UR5-Instance Dyn-URDF via AR-Manager REST-Request'.ljust(l2, ' ') + '|' )

	print('t'.ljust(l, ' ') +'| Toggle between follow cube and random motion'.ljust(l2, ' ') + '|' )
	print(''.ljust(l, ' ') + '|'.ljust(l2, ' ') + '|' )
	print(''.ljust(l,'-') + '+'.ljust(l2,'-')+'+')
	print('Key:'.ljust(l, ' ') + '| Value:'.ljust(l2, ' ') + '|' )
	print(''.ljust(l,'-') + '+'.ljust(l2,'-')+'+')
	print('fps'.ljust(l, ' ') +('| %d'%int(fps)).ljust(l2, ' ') + '|' )

	print(''.ljust(l,'-') + '+'.ljust(l2,'-')+'+')

t_start = time.time()
time.sleep(1)
while ros.is_connected:

	#menu and fps
	fps= 1/(time.time()-t_start)
	menu(fps)
	t_start = time.time()
	

	#plotting
	jp_list = np.array(list(joint_data[0].queue))
	y1 = jp_list[:,0].tolist()
	y2 = jp_list[:,1].tolist()
	x1 =  range(0,len(y1))

	link_list = np.array(list(joint_data[0].queue))
	y3 = link_list[:,0].tolist()
	y4 = link_list[:,1].tolist()
	y5 = link_list[:,2].tolist()
	x2 =  range(0,len(y3))

	if keylis.t_status == False:
		random_col = np.random.randint(0,255)
		i += pi/4
		xyz = [math.sin(i)*amp,math.sin(i)*amp,math.sin(i)*amp]
		msg =  create_Pose( xyz, [0,0,0] ) 
		talker.publish(roslibpy.Message( msg ) )
	
	joint_position_plot.update(x1, [y1,y2])
	tool_position_plot.update(x2, [y3,y4,y5])

	img = joint_position_plot.image_png()
	img3 = tool_position_plot.image_png()

	msg = base64_string(img)
	msg2 = base64_string(img3)
	
	talker_img_joint_position.publish(  roslibpy.Message( msg ) )
	talker_img_tooltip_position.publish(  roslibpy.Message( msg2 ) )
	time.sleep(0.01)

talker.unadvertise()
ros.terminate()