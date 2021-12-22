import sys
import os
import numpy as np

name_of_model = str(sys.argv[1])
print('testing '+ name_of_model)
while os.path.isfile('./data/'+name_of_model+'.txt')==False:
    pass
os.system('python eval_single_file.py '+ name_of_model)
f = open('./log/ready.txt','a')
f.close()
print('testing '+ name_of_model + ' over.')
