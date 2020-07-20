# -*- coding: utf-8 -*-
# Copyright (c) Vispy Development Team. All Rights Reserved.
# Distributed under the (new) BSD License. See LICENSE.txt for more info.
"""
Demonstrates rendering a canvas to an image at higher resolution than the
original display.
"""

import time
import vispy.plot as vp
import vispy.io as io
import numpy as np

import threading
import concurrent.futures as cc
import multiprocessing
import os
import uuid
import queue

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
      renderer.try_pop()

      if (time.time_ns() - start) > 1e9:
        print(f'Consumer: {count} {renderer.queue_in.qsize()}', flush=True)
        count = 0
        start = time.time_ns()

if __name__ == '__main__': 

  renderer = PlotRenderer()
  consumer = Consumer(renderer)
  render_executor = threading.Thread(target=renderer.executor)
  consumer_executor = threading.Thread(target=consumer.consume) 

  render_executor.start()
  consumer_executor.start()

  fig = vp.Fig()
  ax_left = fig[0, 0]
  ax_right = fig[0, 1]

  data = np.random.randn(10, 2)
  ax_left.plot(data)
  ax_right.histogram(data[1])

  start = time.time_ns()
  count = 0
  while True:
    count += 1

    image = fig.render()
    # io.imsave("wonderful.png",image)
    io.write_png("wonderful.png", image)
    # renderer.push(io._make_png, image, 0)
    # io._make_png(image, level=0)

    if (time.time_ns() - start) > 1e9:
      print(f'Producer: {count} {renderer.queue_in.qsize()}', flush=True)
      count = 0
      start = time.time_ns()