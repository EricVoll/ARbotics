import numpy as np
import numpy.random as npr

import matplotlib.pyplot as plt
import cherrypy
from io import BytesIO

class ImageServer:

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

  @cherrypy.expose
  def index(self):
    return ''' <img src="image.png" width="640" height="480" border="0" /> '''
  
  @cherrypy.expose
  def image_png(self):
    img = BytesIO()
    self.plot(img)
    img.seek(0)
    return cherrypy.lib.static.serve_fileobj(img,
      content_type="png",
      name="image.png"
    )
  
  def plot(self, image):

    dmp = npr.normal()
    y = np.sin(self.x) * self.x / dmp
    self.line.set_data(self.x, y)
    
    self.fig.canvas.restore_region(self.ax_bg)
    self.ax.draw_artist(self.line)
    self.fig.canvas.blit(self.ax.bbox)

    self.fig.canvas.flush_events()

    plt.savefig(image, format='png')

if __name__ == '__main__':
  cherrypy.server.socket_host = '0.0.0.0'
  cherrypy.quickstart(ImageServer())
