
import sys
#import pandas as pd



f=open('path_square_clock.bag.txt.smoothed','r')

for lines in f:
    #lines.append(list(line.strip('\n').split(',')))
     #if type(line) == type({}):
        # print ("yes")
     #print ("no")
     mydict=eval(lines)
     #print (type(mydict))
     print mydict['x'],",",mydict['y'],",",mydict['s'],",",mydict['kappa'],",",mydict['dkappa'],",",mydict['theta']
    # result=pd.DataFrame(lines)
#for result in lines.values():
    #print (result)



