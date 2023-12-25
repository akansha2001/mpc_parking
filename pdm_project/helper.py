import numpy as np
import csv
class FileOp:
        def __init__(self, file_name):
            self.output_file=file_name
            self.counter=0
            with open(self.output_file, 'w'):
                pass
          
        def write(self,arr):
            if self.counter%50==0:
                # arr=np.array([value[0],value[1]])
                with open(self.output_file, 'a') as f:
                    # create the csv writer
                    writer = csv.writer(f)
                    # write a row to the csv file
                    writer.writerow(['%.5f' % v for v in arr])
                    self.counter=0
            self.counter+=1
              

def carPolygon(x,y,yaw): #returns a polygon for the car given a position and heading
    return Polygon(shell=((x + carLength/2*np.cos(yaw) + carWidth/2*np.sin(yaw), y + carLength/2*np.sin(yaw) - carWidth/2*np.cos(yaw)),
                     (x - carLength/2*np.cos(yaw) + carWidth/2*np.sin(yaw), y - carLength/2*np.sin(yaw) - carWidth/2*np.cos(yaw)),
                     (x - carLength/2*np.cos(yaw) - carWidth/2*np.sin(yaw), y - carLength/2*np.sin(yaw) + carWidth/2*np.cos(yaw)),
                     (x + carLength/2*np.cos(yaw) - carWidth/2*np.sin(yaw), y + carLength/2*np.sin(yaw) + carWidth/2*np.cos(yaw))))

def obstaclePolygon(obstacle):
    position = np.array([obstacle.position()[0], obstacle.position()[1]])
    width = obstacle.width()
    length = obstacle.length()
    return Polygon(shell=((position[0]+width/2, position[1]+length/2),
                                    (position[0]+width/2, position[1]-length/2),
                                    (position[0]-width/2, position[1]-length/2),
                                    (position[0]-width/2, position[1]+length/2)))

def wallPolygon(obstacle):
    position = np.array([obstacle.position()[1], obstacle.position()[0]])
    width = obstacle.width()
    length = obstacle.length()
    return Polygon(shell=((position[0]+width/2, position[1]+length/2),
                                    (position[0]+width/2, position[1]-length/2),
                                    (position[0]-width/2, position[1]-length/2),
                                    (position[0]-width/2, position[1]+length/2)))
 
def plot_polygon(ax, poly, **kwargs):
    path = Path.make_compound_path(
        Path(np.asarray(poly.exterior.coords)[:, :2]),
        *[Path(np.asarray(ring.coords)[:, :2]) for ring in poly.interiors])

    patch = PathPatch(path, **kwargs)
    collection = PatchCollection([patch], **kwargs)
    
    ax.add_collection(collection, autolim=True)
    ax.autoscale_view()
    return collection    