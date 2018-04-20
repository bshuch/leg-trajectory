# -*- coding: utf-8 -*-
"""
Created on Thu Sep  1 01:41:25 2016

@author: Ben
"""

# In[ ]:
#get_ipython().magic('matplotlib inline')
# Import the right packages
# In[ ]:
import matplotlib.pyplot as plt
plt.ion()
import math
import numpy
import scipy.optimize
from shapely.geometry.polygon import LinearRing, Polygon
#from descartes import PolygonPatch
from shapely import affinity
from foldable_robotics.layer import Layer
# In[ ]:
p1=0,0 #0,0
p2=0,-1.0 #5,0
p3=1,-1.75 #9,1
p4=1,-2.25 #1,1
p5=3,-2.25
p6=3,-0.25
p7=2.75,-0.25
p8=1,-0.25
p9=2,0.5
p10=1,0.5
points=p1,p2,p3,p4,p5,p6,p7,p8,p9,p10
points
points2=numpy.array(points)
# Create two functions which calculate the length of a vector, and 2) the angle between two vectors
def length(v):              #this is a function that solves for the length between two points
    length=(v.dot(v))**.5   
    return length
def angle(v1,v2):
    l1=length(v1)
    l2=length(v2)
    sint=numpy.cross(v1,v2)/l1/l2
    cost=numpy.dot(v1,v2)/l1/l2
    theta=math.atan2(sint,cost)
    return theta    
# In[ ]:
global_ini = points2.flatten().tolist()
# In[ ]:
def generate_equations(theta1,theta2,l1,l2,l3,l4,l5,l6,l7,l8,l9,l10):
    def equations(arrayin):
        x1,y1,x2,y2,x3,y3,x4,y4,x5,y5,x6,y6,x7,y7,x8,y8,x9,y9,x10,y10=arrayin

#-------Define Points----------------------------------------------------------
        
        p1=numpy.array([x1,y1]) #these are points
        p2=numpy.array([x2,y2]) #there are four points on a four bar mechanism
        p3=numpy.array([x3,y3]) #they each have an x and y component
        p4=numpy.array([x4,y4]) 
        p5=numpy.array([x5,y5])
        p6=numpy.array([x6,y6])
        p7=numpy.array([x7,y7])
        p8=numpy.array([x8,y8])
        p9=numpy.array([x9,y9])
        p10=numpy.array([x10,y10])
        
#-------Create equations for length--------------------------------------------
        
        eq1=length(p2-p1)-l1    #I have an equation to define each length
        eq2=length(p3-p2)-l2    #which is just one theoretical point - another
        eq3=length(p8-p3)-l3    #I can subtract a number, such as l3
        eq4=length(p8-p4)-l4    #and now that becomes the length of the linkage
        eq5=length(p4-p5)-l5
        eq6=length(p5-p6)-l6
        eq7=length(p8-p6)-l7
        eq8=length(p8-p7)-l8
        eq9=length(p9-p7)-l9
        eq10=length(p10-p9)-l10
        
#-------Create equations to constrain points-----------------------------------
        
        eq11=p1[0]-0
        eq12=p1[1]-0
        eq13=p8[0]-1
        eq14=p8[1]+0 #0.25
        eq15=p10[0]-1
        eq16=p10[1]-1 #-0.75
        
        #transmission output
#-------Creating equations to make angles--------------------------------------        
        eq17=angle(p8-p3,p8-p4)-0*math.pi/180
        eq18=angle(p8-p6,p8-p7)-0*math.pi/180        
        eq19=angle(numpy.r_[1,0],p2-p1)-theta1*math.pi/180        
        eq20=angle(numpy.r_[0,1],p10-p9)-theta2*math.pi/180        
        
        eq=[eq1,eq2,eq3,eq4,eq5,eq6,eq7,eq8,eq9,eq10,
            eq11,eq12,eq13,eq14,eq15,eq16,eq17,eq18,eq19,eq20]
        eq=numpy.array(eq) 
        eq=eq**2
        result=eq.sum()  
        return result 
    return equations

def plotme(x):
    plt.plot(x[(0,1),0],x[(0,1),1])
    plt.plot(x[(1,2),0],x[(1,2),1])
    plt.plot(x[(2,3),0],x[(2,3),1])    
    plt.plot(x[(3,4),0],x[(3,4),1])
    plt.plot(x[(4,5),0],x[(4,5),1])
    plt.plot(x[(5,6),0],x[(5,6),1])
    plt.plot(x[(6,7),0],x[(6,7),1])
    plt.plot(x[(7,2),0],x[(7,2),1])
    plt.plot(x[(8,6),0],x[(8,6),1])
    plt.plot(x[(9,8),0],x[(9,8),1])

def plottext(x):
    plt.text(x[0,0],x[0,1],'P1')
    plt.text(x[3,0],x[3,1],'P4')
    plt.text(x[9,0],x[9,1],'P10')
    
def plotpointsred(x):
    plt.plot(x[4,0],x[4,1],'ro')
    #print (x[3,0],x[3,1])
    #plt.text(x[0,0],x[0,1],'test')
    
def plotpointsblue(x):
    plt.plot(x[4,0],x[4,1],'bo')
    
def plotpointsgreen(x):
    plt.plot(x[4,0],x[4,1],'go--')    
    
def plotsin(x):
    plt.plot([t],[theta1],'ro')
    plt.plot([t],[theta2],'bo')

#plt.plot(points2[:,0],points2[:,1])
# In[ ]:

path = []
q_in = []
q_lets = []

w_offset = 90 #90
t1_A = 50 #60
t2_A = 50 #60
t1_A_offset = -90 #-90
t2_A_offset = 90 #50 #62

#Left servo motion from left to right
ini = global_ini 
P_act = []
#plt.figure()
solutions = []
timearray = numpy.r_[0:360:21j] #21
t1t2 = []
counter = 1

#fig = plt.figure()
#ax = fig.add_subplot(111)

for t in timearray: 
    
    """
    A sine function is used to control the outputs going from 0 to 360
    Give it an initial offset that will be true when t = 0 in sin(t) + offset
    Multiply sine wave by amount to add when t = 1, being x * sin(t)
    Turn inside of sin into radians by multiplying by 2pi/360
    """
    l_ary = numpy.array([1,1.35,1.5,2.5,2.5,2.5,2.5,1.5,1.35,1]) #when 1 and 1.35 are both 1.35 increased range of motion
    theta1 = t1_A*math.sin((t)*(2*math.pi)/360)+t1_A_offset #has a good range of motion, can stay mostly the same
    print(theta1)    
    theta2 = t2_A*math.sin((t+w_offset)*(2*math.pi)/360)+t2_A_offset
    # equations = generate_equations(theta1,theta2,1,1.2,1.2,2,2,2,2,1.2,1.2,1) #1,1
    equations = generate_equations(theta1,theta2,l_ary[0],l_ary[1],l_ary[2],
                                   l_ary[3],l_ary[4],l_ary[5],l_ary[6],l_ary[7],l_ary[8],l_ary[9]) #1,1
    result=scipy.optimize.minimize(equations,ini)  
    t1t2.append([theta1,theta2])
    if abs(result.fun)<1e-1:
        #ini = x
        x = result.x.reshape(10,2)
        solutions.append(x)
        x = numpy.r_[x,x[0:1,:]]
        #plt.plot(x[:,0],x[:,1])  
        newpoint = x[4,0],x[4,1]        
        P_act.append(newpoint)
        #P_act.append(x[3,1])
        if counter == 1:
            plotme(x)       
#        plottext(x)
#        plotpointsred(x)
        #ax.plot(x[4,0],x[4,1],'ro--')
        counter = 2
        #Jacobian Start
        path.append(x)
        q_in.append(t)
        q_lets.append(t)
        #plotsin(x)
#        print('error={0:.2e},theta1={1:3.0f}(degrees) worked.'.format(result.fun,theta1))
    else:
        pass
#        print('error={0:.2e},theta1={1:3.0f}(degrees) is an invalid condition.'.format(result.fun,theta1))
    ini = result.x #uses last result to create new result
plt.axis("equal")

act = numpy.array(P_act)

#fig = plt.figure()
#ax = add_subplot(111)
all_i = numpy.r_[0:20:1]
for i in all_i:
    plt.plot(act[(i,i+1),0],act[(i,i+1),1],'ro--')
plt.show()
#plt.plot(P_act)

path = numpy.array(path)
q_in = numpy.array(q_in)

solutions = numpy.array(solutions)
t1t2 = numpy.array(t1t2)

m1 = path[:,3]-path[:,2]
v = path[:,2]+m1*.6
#m1 = m1/((m1*m1).sum(1))**.5
T = numpy.array([[0,1],[-1,0]])
m2 = m1.dot(T)
v2 = v+m2*.4

range_min = 0
range_max = 21

y_out = v2[range_min:range_max]
q_in = q_in[range_min:range_max]

dy = y_out[2:,:]-y_out[:-2,:]
dq = q_in[2:]-q_in[:-2]

J=(dy.T/dq.T)
J.shape

m = 0.396/4.0 #kg
g = 9.81 #m/s^2
fx = -m*g
fy = m*g #N
#f = numpy.zeros((2,len(y_out)))
#f[0,:] = fx
#f[1,:] = fy
f = numpy.array([[fx,fy]]).T
t = J.T.dot(f)

print(t) #N-m
print(t.max())
print(t.min())

# In[ ]:

path = []
q_in = []
q_lets = []

w_offset = 90 #90
t1_A = 30 #60
t2_A = 30 #60
t1_A_offset = -90 #-90
t2_A_offset = 90 #50 #62

#Left servo motion from left to right
ini = global_ini 
P_act = []
#plt.figure()
solutions = []
timearray = numpy.r_[0:360:21j] #21
t1t2 = []
counter = 1
for t in timearray: 
    
    """
    A sine function is used to control the outputs going from 0 to 360
    Give it an initial offset that will be true when t = 0 in sin(t) + offset
    Multiply sine wave by amount to add when t = 1, being x * sin(t)
    Turn inside of sin into radians by multiplying by 2pi/360
    """
    l_ary = numpy.array([1,1.35,1.5,2.5,2.5,2.5,2.5,1.5,1.35,1]) #when 1 and 1.35 are both 1.35 increased range of motion
    theta1 = t1_A*math.sin((t)*(2*math.pi)/360)+t1_A_offset #has a good range of motion, can stay mostly the same
    print(theta1)    
    theta2 = t2_A*math.sin((t+w_offset)*(2*math.pi)/360)+t2_A_offset
    # equations = generate_equations(theta1,theta2,1,1.2,1.2,2,2,2,2,1.2,1.2,1) #1,1
    equations = generate_equations(theta1,theta2,l_ary[0],l_ary[1],l_ary[2],
                                   l_ary[3],l_ary[4],l_ary[5],l_ary[6],l_ary[7],l_ary[8],l_ary[9]) #1,1
    result=scipy.optimize.minimize(equations,ini)  
    t1t2.append([theta1,theta2])
    if abs(result.fun)<1e-1:
        #ini = x
        x = result.x.reshape(10,2)
        solutions.append(x)
        x = numpy.r_[x,x[0:1,:]]
        #plt.plot(x[:,0],x[:,1])  
        newpoint = x[4,0],x[4,1]        
        P_act.append(newpoint)
        #P_act.append(x[3,1])
        if counter == 1:        
            plotme(x)       
#        plottext(x)
        plotpointsblue(x)
        counter = 2
        #Jacobian Start
        path.append(x)
        q_in.append(t)
        q_lets.append(t)
        #plotsin(x)
#        print('error={0:.2e},theta1={1:3.0f}(degrees) worked.'.format(result.fun,theta1))
    else:
        pass
#        print('error={0:.2e},theta1={1:3.0f}(degrees) is an invalid condition.'.format(result.fun,theta1))
    ini = result.x #uses last result to create new result
plt.axis("equal")

act = numpy.array(P_act)

#fig = plt.figure()
#ax = add_subplot(111)
all_i = numpy.r_[0:20:1]
for i in all_i:
    plt.plot(act[(i,i+1),0],act[(i,i+1),1],'bo--')
plt.show()

path = numpy.array(path)
q_in = numpy.array(q_in)

solutions = numpy.array(solutions)
t1t2 = numpy.array(t1t2)

#a = f.add_subplot(111)

m1 = path[:,3]-path[:,2]
v = path[:,2]+m1*.6
#m1 = m1/((m1*m1).sum(1))**.5
T = numpy.array([[0,1],[-1,0]])
m2 = m1.dot(T)
v2 = v+m2*.4

range_min = 0
range_max = 21

path = numpy.concatenate((path,v[:,None,:],v2[:,None,:]),1)

#a = f.add_subplot(111)
"""
for item in path[range_min:range_max]:
    a.plot(item[(0,1,2,4,5,4,3,0),0],item[(0,1,2,4,5,4,3,0),1],'ro-')
plt.axis('equal')

y_out = v2[range_min:range_max]
q_in = q_in[range_min:range_max]

dy = y_out[2:,:]-y_out[:-2,:]
dq = q_in[2:]-q_in[:-2]

J=(dy.T/dq.T)
J.shape

m = 0.396/4.0 #kg
g = 9.81 #m/s^2
fx = -m*g
fy = m*g #N
#f = numpy.zeros((2,len(y_out)))
#f[0,:] = fx
#f[1,:] = fy
f = numpy.array([[fx,fy]]).T
t = J.T.dot(f)

print(t) #N-m
print(t.max())
print(t.min())
"""
# In[ ]:
path = []
q_in = []
q_lets = []

w_offset = 90 #90
t1_A = 70 #60
t2_A = 70 #60
t1_A_offset = -90 #-90
t2_A_offset = 90 #50 #62

ini = global_ini 
P_act = []
#plt.figure()
solutions = []
timearray = numpy.r_[0:360:21j] #21
t1t2 = []
counter = 1

for t in timearray: 
    
    l_ary = numpy.array([1,1.35,1.5,2.5,2.5,2.5,2.5,1.5,1.35,1]) #when 1 and 1.35 are both 1.35 increased range of motion
    theta1 = t1_A*math.sin((t)*(2*math.pi)/360)+t1_A_offset #has a good range of motion, can stay mostly the same
    print(theta1)    
    theta2 = t2_A*math.sin((t+w_offset)*(2*math.pi)/360)+t2_A_offset
    equations = generate_equations(theta1,theta2,l_ary[0],l_ary[1],l_ary[2],
                                   l_ary[3],l_ary[4],l_ary[5],l_ary[6],l_ary[7],l_ary[8],l_ary[9]) #1,1
    result=scipy.optimize.minimize(equations,ini)  
    t1t2.append([theta1,theta2])
    if abs(result.fun)<1e-1:
        x = result.x.reshape(10,2)
        solutions.append(x)
        x = numpy.r_[x,x[0:1,:]]
        newpoint = x[4,0],x[4,1]        
        P_act.append(newpoint)
        if counter == 1:
            plotme(x)       
        plotpointsgreen(x)
        counter = 2
        
        #Jacobian Start
        path.append(x)
        q_in.append(t)
        q_lets.append(t)
        #plotsin(x)
#        print('error={0:.2e},theta1={1:3.0f}(degrees) worked.'.format(result.fun,theta1))
    else:
        pass
#        print('error={0:.2e},theta1={1:3.0f}(degrees) is an invalid condition.'.format(result.fun,theta1))
    ini = result.x #uses last result to create new result
plt.axis("equal")

act = numpy.array(P_act)

#fig = plt.figure()
#ax = add_subplot(111)
all_i = numpy.r_[0:20:1]
for i in all_i:
    plt.plot(act[(i,i+1),0],act[(i,i+1),1],'go--')
plt.show()

path = numpy.array(path)
q_in = numpy.array(q_in)

solutions = numpy.array(solutions)
t1t2 = numpy.array(t1t2)

m1 = path[:,3]-path[:,2]
v = path[:,2]+m1*.6
#m1 = m1/((m1*m1).sum(1))**.5
T = numpy.array([[0,1],[-1,0]])
m2 = m1.dot(T)
v2 = v+m2*.4

range_min = 0
range_max = 21

y_out = v2[range_min:range_max]
q_in = q_in[range_min:range_max]

dy = y_out[2:,:]-y_out[:-2,:]
dq = q_in[2:]-q_in[:-2]

J=(dy.T/dq.T)
J.shape

m = 0.396/4.0 #kg
g = 9.81 #m/s^2
fx = -m*g
fy = m*g #N
#f = numpy.zeros((2,len(y_out)))
#f[0,:] = fx
#f[1,:] = fy
f = numpy.array([[fx,fy]]).T
t = J.T.dot(f)

print(t) #N-m
print(t.max())
print(t.min())

"""
multiplier = 2.5
servo1 = numpy.array([x[0,0],x[0,1]])
servo2 = numpy.array([x[9,0],x[9,1]])
mount = numpy.array([x[7,0],x[7,1]])
s1s2dist = length(servo2-servo1)*multiplier
mounttservo = length(servo2-mount)*multiplier
mountdist = math.sqrt(abs(((mounttservo)**2-(s1s2dist/2.0)**2)))
print("Distance between servos or mounts:", s1s2dist)
print("Distance that mount should be away from top of device:", mountdist)

print("\nBuild sizes")
print("Servo length =", l_ary[0]*multiplier)
print("From servo to linkage =", l_ary[1]*multiplier)
print("From linkage to device =", l_ary[2]*multiplier, "away from mount point")
print("Primary linkage length =", l_ary[3]*multiplier)
print("Place rivet holes about 1 cm away from the main attachment locations")
"""