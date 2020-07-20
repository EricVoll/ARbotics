import roslibpy

client = roslibpy.Ros(host='127.0.0.1', port=5000)
client.run()
print('Is ROS connected?', client.is_connected)
client.terminate()
