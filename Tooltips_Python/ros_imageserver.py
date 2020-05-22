from __future__ import print_function
import numpy as np
import numpy.random as npr

import matplotlib.pyplot as plt
import argparse
from io import BytesIO

import logging
import roslibpy
import time
import base64

import threading
import concurrent.futures as cc
import multiprocessing
import os
import uuid
import queue

logger = logging.getLogger('imgserver')


class RLPSender:
  def __init__(self, ip:str):
    client = roslibpy.Ros(host=ip, port=9090)
    client.run()

    img_gen = ImageGenerator()

    talker = roslibpy.Topic(client, '/img', 'std_msgs/String')

    try:
      while client.is_connected:
          img_b64 = base64.b64encode(img_gen.image_png().getvalue()).decode()
          talker.publish(roslibpy.Message({'data': img_b64}))
          print('Sending message...')
          time.sleep(1)
    except KeyboardInterrupt:
      logging.info("Shutting down sender.")

    talker.unadvertise()
    client.terminate()



class RLPReceiver:
  def __init__(self, ip:str):
    client = roslibpy.Ros(host=ip, port=9090)
    client.run()

    listener = roslibpy.Topic(client, '/img', 'std_msgs/String')
    listener.subscribe(self.rec_handler)    

    try:
        while True:
            pass
    except KeyboardInterrupt:
        client.terminate()

  def rec_handler(self, message):
    print('Heard talking: ' + message['data'])


class RenderTask:
  def __init__(self, task_f, *args):
    self.id = str(uuid.uuid4())
    self.task_f = task_f
    self.args = args
    self.done = False
    
  def render(self):
    img = self.task_f(*(self.args))
    self.done = True
    return img


class PlotRenderer:
  def __init__(self):
    self.n_workers = multiprocessing.cpu_count() - 1
    self.n_tasks = 10
    self.logger = multiprocessing.get_logger()
    self.proc = os.getpid()
    self.queue_in = queue.Queue()
    self.queue_out = queue.Queue()

    self.shutdown = False

  def push(self, task_f, *args):
    task = RenderTask(task_f, *args)
    self.queue_in.put(task)

  
  def try_pop(self):
    return self.queue_out.get()
  
  def executor(self):
    with cc.ThreadPoolExecutor(max_workers=self.n_workers) as executor:
      while not self.shutdown:
        task = self.queue_in.get(block=True)
        # print(f'Received a task {task.id}')
        future = executor.submit(task.render())
        self.queue_out.put(future)

class Consumer:
  def __init__(self, renderer):
    self.renderer = renderer

  def consume(self):
    start = time.time_ns()
    count = 0
    while True:
      count += 1
      self.renderer.try_pop()

      if (time.time_ns() - start) > 1e9:
        print(f'Consumer: {count} {self.renderer.queue_in.qsize()}', flush=True)
        count = 0
        start = time.time_ns()


class ImageGenerator:

  def __init__(self):
    fig, ax = plt.subplots()
    ax_bg = fig.canvas.copy_from_bbox(ax.bbox)

    dmp = npr.normal()
    x = np.linspace(0, 10)
    y = np.sin(x) * x/dmp
    line = ax.plot(x, y)[0]

    fig.canvas.draw()  

    self.fig = fig
    self.line = line
    self.y = y
    self.x = x
    self.ax = ax
    self.ax_bg = ax_bg

  def index(self):
    return ''' <img src="image.png" width="640" height="480" border="0" /> '''
  
  def image_png(self):
    img = BytesIO()
    self.plot(img)
    img.seek(0)
    return img
  
  def plot(self, image):

    dmp = npr.normal()
    y = np.sin(self.x) * self.x / dmp
    self.line.set_data(self.x, y)
    
    self.fig.canvas.restore_region(self.ax_bg)
    self.ax.draw_artist(self.line)
    self.fig.canvas.blit(self.ax.bbox)

    self.fig.canvas.flush_events()
    plt.savefig(image, format='png')

def str2bool(v):
    if isinstance(v, bool):
       return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')


if __name__ == '__main__':

  renderer = PlotRenderer()
  consumer = Consumer(renderer)
  img_gen = ImageGenerator()

  render_executor = threading.Thread(target=renderer.executor)
  consumer_executor = threading.Thread(target=consumer.consume) 

  render_executor.start()
  consumer_executor.start()

  start = time.time_ns()
  count = 0
  while True:
    count += 1
    img_gen.image_png()
    if (time.time_ns() - start) > 1e9:
      print(f'{count}')
      count = 0
      start = time.time_ns()



  parser = argparse.ArgumentParser(description='RosLibPy Example')
  parser.add_argument("--receiver", type=str2bool, nargs='?',
                          const=True, default=False,
                          help="Activate receiver mode.")

  parser.add_argument("--sender", type=str2bool, nargs='?',
                          const=True, default=False,
                          help="Activate sender mode.")

  parser.add_argument("--ip", type=str, help="Master IP", required=True)

  args = parser.parse_args()

  if args.sender:
    logger.info("Sender Mode")
    RLPSender(args.ip)
  elif args.receiver:
    logger.info("Receiver Mode")
    RLPReceiver(args.ip)
  else:
    logger.info("No mode selected.")
