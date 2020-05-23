
import roslibpy
import base64
import time
import logging
import threading
import json



class DebugReceiver:

  def __init__(self, ip:str):
    self.client = roslibpy.Ros(host=ip, port=9090)
    self.client.run()

  
  def listen(self):
    # listener = roslibpy.Topic(self.client, '/euclidian_goal_pose', 'geometry_msgs/Pose')
    listener = roslibpy.Topic(self.client, '/log/unity_debug', 'std_msgs/String')
    listener.subscribe(self.rec_handler)

  def rec_handler(self, message):
    try:
      debug_msg = json.loads(message['data'])
      print('Heard talking: ' + debug_msg['content'])
    except Exception:
      print('ERROR: ' + str(message))





class RLPReceiver:

  def __init__(self, ip:str):
    self.client = roslibpy.Ros(host=ip, port=9090)
    self.client.run()

  
  def listen(self):
    print("test")
    # listener = roslibpy.Topic(self.client, '/euclidian_goal_pose', 'geometry_msgs/Pose')
    listener = roslibpy.Topic(self.client, '/chatter', 'std_msgs/String')
    listener.subscribe(self.rec_handler)
    time.sleep(3)

  def rec_handler(self, message):
    print('Heard talking: ' + str(message))



class RLPSender:
  def __init__(self, ip:str):
    self.client = roslibpy.Ros(host=ip, port=9090)
    self.client.run()
    self.talker = roslibpy.Topic(self.client, '/euclidean_goal_pose', 'geometry_msgs/Pose')

  def control_joint(self):
    message = roslibpy.Message(
      {'position': {
        'x': 1,
        'y': 1,
        'z': 1
      },
      'orientation': {
        'x': 1,
        'y': 1,
        'z': 1,
        'w': 1 
      }
    })
    self.talker.publish(message)

  def terminate(self):
    self.client.terminate()

if __name__ == "__main__":
    host = 'vservice-host-001.chera.ch'
    
    hololens_log = DebugReceiver(host)
    hololens_log.listen()

    try:
      while True:
          time.sleep(10)
    except KeyboardInterrupt:
      pass
    # receiver = RLPReceiver(host)
    # threading.Thread(target=receiver.listen).start()
    # time.sleep(1)
    # RLPSender(host).control_joint()
    