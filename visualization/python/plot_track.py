import numpy as np
import matplotlib.pyplot as plt
import math
f = open("track_points_3.txt", "r")
x = []
y = []
x_rail_1 = []
y_rail_1 = []
x_rail_2 = []
y_rail_2 = []
x_subset = []
y_subset = []
count = 0
buf = 0.8
for i in f:
  a = i.split()
  #print(a);
  x.append(-1*float(a[0])) #to show figure as gazebo
  y.append(float(a[1]))
  """if count==0:
    x_rail_1.append(x[0])
    y_rail_1.append(y[0]-buf)
    x_rail_2.append(x[0])
    y_rail_2.append(y[0]+buf)
    x_subset.append(x[0])
    y_subset.append(y[0])
  else:
    x_diff = x[count]-x[count-1]
    y_diff = y[count]-y[count-1]
    mag = math.sqrt(x_diff**2 + y_diff**2)
    x_diff_norm = x_diff/mag
    y_diff_norm = y_diff/mag
    #print(x_diff_norm, " ", y_diff_norm)
    x_rail_1.append(x[count]+buf*(-1*y_diff_norm))
    y_rail_1.append(y[count]+buf*(x_diff_norm))
    
    #mag_rail_1 = math.sqrt((x_rail_1[count]-x[count])**2 + (y_rail_1[count]-y[count])**2)
    
    x_rail_2.append(x[count]+buf*(y_diff_norm))
    y_rail_2.append(y[count]+buf*(-1*x_diff_norm))
    #mag_rail_2 = math.sqrt((x_rail_2[count]-x[count])**2 + (y_rail_2[count]-y[count])**2)
    #print(mag_rail_1, " ", mag_rail_2)
    x_subset.append(x[count])
    y_subset.append(y[count])
  count+=1"""



#plt.scatter(y_rail_1,x_rail_1)
#plt.scatter(y_rail_2,x_rail_2)

plt.scatter(y,x) #to show figure as gazebo
#plt.scatter(y_subset,x_subset) #to show figure as gazebo
plt.show()
