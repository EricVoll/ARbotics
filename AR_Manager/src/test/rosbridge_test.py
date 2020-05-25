import roslibpy
import numpy as np
from s
from scipy.spatial.transform import Rotation as R
def create_MultiArray32(data):
	"""creates multiarray

	Parameters
	----------
	data : np.array float32
			[description]
	"""
	dim = []
	for i in range(0,len(data.shape)):
		out = 'axis'+str(i)
		dim.append( {'label': out, 'size': data.shape[i], 'stride':1 })

	return { 
			'layout': {'dim': dim , 'data_offset':0},

			'data': data.tolist()
	}

def create_Pose(t,xyz):
	q = R.from_euler('xyz' , xyz).as_quat()
	return { 'position': {'x':t[0] ,'y':t[1]: ,'z':t[2]} , 'orientation':{'x':q[0] ,'y':q[1]: ,'z':q[2]},'w': q[3] } }


print(create_MultiArray32( np.ones((3,100))))

ros = roslibpy.Ros(host='127.0.0.1', port=9090)
ros.run()

print(ros.is_connected)




# listener = roslibpy.Topic(ros, '/joint_states', 'sensor_msgs/JointState')
# listener = roslibpy.Topic(ros, '/euclidean_goal_pose', 'geomoetry_msgs/Pose')
# listener.subscribe(lambda message: print('Heard talking: ' + str(message)))


# try:
# 	while True:
# 			pass
# except KeyboardInterrupt:
#     ros.terminate()

import time
talker = roslibpy.Topic(ros, '/euclidean_goal_pose', 'geometry_msgs/Pose')
while ros.is_connected:
	msg =  create_Pose( [0,0.3,0.3], [0,0,0] ) ) 
	talker.publish(roslibpy.Message( msg ) )
	print('Sending message...' + str(msg))
	time.sleep(0.5)

talker.unadvertise()
ros.terminate()




# import time
# talker = roslibpy.Topic(ros, '/ar_joint_states', 'std_msgs/Float32MultiArray')
# while ros.is_connected:
# 	msg =  create_MultiArray32(np.ones((5) ,dtype=np.float32) ) 
# 	talker.publish(roslibpy.Message( msg ) )
# 	print('Sending message...' + str(msg))
# 	time.sleep(0.5)

# talker.unadvertise()
# ros.terminate()