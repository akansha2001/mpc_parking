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
              
    
    