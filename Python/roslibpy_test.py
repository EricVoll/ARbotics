import roslibpy

client = roslibpy.Ros(host='vservice-host-001.chera.ch', port=9090)
client.run()
print('Is ROS connected?', client.is_connected)
client.terminate()
