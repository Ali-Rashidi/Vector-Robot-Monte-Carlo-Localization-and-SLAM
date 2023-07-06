#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
import numpy as np
from math import *
from os.path import expanduser
import matplotlib.pyplot as plt
import scipy


## Constants and parameters :
num_prtcls = 100



laser_var = 0.024471/5
v_var = 0.0005
w_var = 0.0015
g_var = 0.005

grid_size = 0.02

x = 0.0
y = 0.0 
theta = 0.0

PI = 3.1415926535897

laser_data=0
laser=0

desired_time=1 # dt is fixed but we calculate speed

# Functions :

def callback_laser(msg):
    global laser
    laser= msg.range

def new_odometry(msg):
    global x
    global y
    global theta
 
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
 
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


def motion_model(prtcl_weight, v , w, dt , v_var = v_var , w_var = w_var , g_var = g_var ):
    #alpha=0.005
    grid_sz = 0.02
    update_prtcl=[]
    for i in range(len(prtcl_weight)):
        part= prtcl_weight[i]
        xx = part[0]
        yy = part[1]
        tt = part[2]
        ww = part[3]
        mm = part[4]
        traj = part[5]

        v_hat = v + np.random.normal(0, v_var)
        w_hat = w + np.random.normal(0, w_var)
        if w_hat==0:
            w_hat=1e-9
        g_hat = np.random.normal(0, g_var  )

        xx = xx + (v_hat/w_hat)* ( - sin(tt) +sin(tt+ w_hat*dt) ) 

        yy = yy + (v_hat/w_hat)* (   cos(tt) -cos(tt+ w_hat*dt) )

        tt = tt+ dt * (w_hat + g_hat) + 10*PI
        tt = tt - int(tt/(2*PI))*2*PI
        
        traj1 = np.array([ xx , yy , tt]).reshape(1,3)
        traj = np.concatenate( (traj , traj1) , axis=0)
        new = [ xx , yy , tt , ww , mm , traj]

        update_prtcl.append(new)

    return update_prtcl


def measurment_model (prtcl_weight , z ,laser_var = laser_var  ) :
    max_laser = 0.4
    grid_size=0.02
    max_grid = int ( max_laser / grid_size)
    sensor_var = laser_var
    occ_thresh = 0.75
    update_prtcl1=[]
    summ=0
    for i in range(len(prtcl_weight)):
        part= prtcl_weight[i]
        xx = part[0]
        yy = part[1]
        tt = part[2]
        ww = part[3]
        mm = part[4]
        traj = part[5]
        dist=0.4
        for n in range ( 1 , max_grid):
            x_pose = int ( np.round (( xx + n * grid_size * cos(tt))/ grid_size))
            y_pose = int ( np.round (( yy + n * grid_size * sin(tt))/ grid_size))

            if mm [x_pose][y_pose] >= occ_thresh:
                dist = np.linalg.norm ( [xx - x_pose*grid_size , yy - y_pose*grid_size])
                break
        
        zz = int (z/grid_size) * grid_size
        ww = scipy.stats.norm(dist , sensor_var).pdf(zz)
        summ=summ+ww

        new = [ xx , yy , tt , ww , mm , traj]
        update_prtcl1.append(new)

    update_prtcl=[]
    for i in range(len(update_prtcl1)):
        part= update_prtcl1[i]
        xx = part[0]
        yy = part[1]
        tt = part[2]
        ww = part[3] / summ
        mm = part[4]
        traj = part[5]

        new = [ xx , yy , tt , ww , mm , traj]
        update_prtcl.append(new)


    return update_prtcl

def update_occ_grid (prtcl_weight , z ):

    grid_size=0.02
    free_prob = 0.35
    occ_prob = 0.9
    update_prtcl=[]
    for i in range ( len (prtcl_weight)):
        part= prtcl_weight[i]
        xx = part[0] # / grid_size
        yy = part[1] # / grid_size
        tt = part[2]
        ww = part[3]
        mm = part[4]
        traj = part[5]
        #mapupdate = np.zeros((120,120))

        #T = np.matrix (  [ [cos(tt) , -sin(tt) , xx] , [ sin(tt) , cos(tt) , yy ] , [ 0 , 0 , 1]])
        if z<=0.39:
            z_grid = int(z / grid_size)
            for n in range ( 1 , z_grid):
                x_pose = int ( np.round (( xx + n * grid_size * cos(tt))/ grid_size))
                y_pose = int ( np.round (( yy + n * grid_size * sin(tt))/ grid_size))
                mm[x_pose][y_pose] = (mm[x_pose][y_pose] + free_prob)/2 
            
            x_pose = int ( np.round (( xx + z_grid  * grid_size * cos(tt))/ grid_size))
            y_pose = int ( np.round (( yy + z_grid  * grid_size * sin(tt))/ grid_size))
            mm [x_pose][y_pose] = (mm[x_pose][y_pose] + occ_prob )/2 

        else :
            z_grid = int(z / grid_size)
            for n in range ( 1 , z_grid):
                x_pose = int ( np.round (( xx + n * grid_size * cos(tt))/ grid_size))
                y_pose = int ( np.round (( yy + n * grid_size * sin(tt))/ grid_size))

                mm [x_pose][y_pose] = (mm[x_pose][y_pose] + free_prob)/2 
            
            x_pose = int ( np.round (( xx + z_grid  * grid_size * cos(tt))/ grid_size))
            y_pose = int ( np.round (( yy + z_grid  * grid_size * sin(tt))/ grid_size))
            mm [x_pose][y_pose] = (mm[x_pose][y_pose] + free_prob)/2 

        #mm = mm - logOdds
        #print(mapupdate)


        new = [ xx , yy , tt , ww , mm , traj]
        update_prtcl.append(new)
    
    return update_prtcl




def gen_prtcl( num_prtcls = num_prtcls ):

    prtcl_weight_map = []
    for i in range (num_prtcls):
        prtcl_x = 0.5
        prtcl_y = 0.5
        prtcl_theta = 0
        weights = 0
        maps = np.ones((160,160)) * 0.5
        trajectory = np.array([prtcl_x  , prtcl_y , prtcl_theta]).reshape(1,3)
        part = [ prtcl_x  , prtcl_y , prtcl_theta , weights , maps , trajectory]
        prtcl_weight_map.append(part)
    return prtcl_weight_map




##### Main Program ####################

rospy.init_node("Monte_Carlo_Localization")
Odometry_reader= rospy.Subscriber("/odom", Odometry, new_odometry)
velocity_publisher = rospy.Publisher('/vector/cmd_vel', Twist, queue_size=1)
laser_reader = rospy.Subscriber('/vector/laser', Range, callback_laser, queue_size = 1)
vel_msg = Twist()

# initial particles from a uniform distribution:
X_W = gen_prtcl() # generate particle from a uniform distribution with normalized weights
###################### ********* Scan Matching + Movement design
q=0
x_real , y_real , theta_real = x , y , theta
plt.figure()
val_dist = 0.05
val_angel = PI/18
dist=val_dist
angle=val_angel
k=0
move_forward=True
while not rospy.is_shutdown():
    q=q+1
    print("**************")
    print ( " iteration  : " + str(q))

    t0 = rospy.Time.now().to_sec()
    while t0<=0:
        t0 = rospy.Time.now().to_sec()

    dt=0
    speed_t = (dist/desired_time)
    velocity_x = speed_t
    speed_r=(angle/desired_time)
    velocity_z = speed_r
    laser_reading = laser
    update=False
    move=False
    
    
    if  laser_reading>= 0.15 and ((k==0) or (k==36)) and move_forward==True :
        while(dt<desired_time):
            move_forward=False
            speed_r = 0
            if (laser_reading>0.02) :
                
                vel_msg.linear.x = velocity_x
                vel_msg.angular.z=0
                velocity_publisher.publish(vel_msg)
                t1=rospy.Time.now().to_sec()
                dt=t1-t0
                move=True
                update=True
            else:
                break

            laser_reading = laser

    elif (laser_reading<0.15) or move_forward==False and dt==0:
        while(dt<desired_time):
            speed_t = 0
            if (laser_reading>=0.01) :
                
                vel_msg.linear.x = 0
                vel_msg.angular.z=velocity_z*2
                velocity_publisher.publish(vel_msg)
                t1=rospy.Time.now().to_sec()
                dt=t1-t0
                update=True
                k=k+1
                
            if k == 36 or laser_reading>=0.1 :
                move_forward=True
                k=0
            if (laser_reading<0.01):
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
                velocity_publisher.publish(vel_msg)
                angle=-angle
                break

            laser_reading = laser

    laser_reading = laser
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

    
    if update == True:
        

        # update particles : x(t) = p (x(t-1) , u(t))

        X_W = motion_model(X_W , v = speed_t , w = speed_r , dt = dt)

        laser_reading = laser

        x_real , y_real , theta_real = x , y , theta

        X_W = measurment_model(X_W , z=laser_reading ) # Weight calculation

        X_W = update_occ_grid(X_W , z = laser_reading) # map update

        weights=[]
        for i in range(len(X_W)):
            part=X_W[i]
            ww = part[3]
            weights.append(ww)

        best = np.argmax(weights)
        best_prtcl = X_W[best]
        xxx = best_prtcl[0]
        yyy = best_prtcl[1]
        ttt = best_prtcl[2]
        www = best_prtcl[3]
        mmm = best_prtcl[4]
        trajectory = best_prtcl[5]
        #print( " robot trajectory : " , trajectory)
        #print(mmm)
        #print(np.max(mmm) , np.min(mmm))
        #print(www)

        #plt.imshow(mmm>0.6,vmin=0, vmax=1)
        #plt.pause(0.1)
        plt.clf()
        #plt.gca().invert_yaxis()
        #plt.gca().invert_xaxis()
        #print(trajectory.shape[0])
        for i in range(trajectory.shape[0]):
            traj = trajectory[i]
            xx = traj[0]
            yy = traj[1]
            tt = traj[2]
            plt.arrow(yy , xx , 1e-5*sin(tt) , 1e-5*cos(tt) , color='black' , head_width = 0.02, overhang = 0.6)
        plt.draw()
        #print(mmm.shape)
        map_ind = np.where(mmm>0.6)
        #print(map_ind)
        if len(map_ind[0]) >0:
            for i in range(len(map_ind[0])) :
                xx , yy = map_ind[0][i]* grid_size , map_ind[1][i]* grid_size
                circle = plt.Circle((yy, xx), grid_size/2, color='r')
                plt.gca().add_patch(circle)
        plt.xlim(-2, 2)
        plt.ylim(-2, 2)
        plt.draw()
        plt.pause(0.1)

   
        ## resample :
        N_eff = 1 / (np.sum(np.array(weights)**2))
        print("N_eff = " , N_eff)
        if N_eff < num_prtcls/2:
            print("Resampling... , N_eff = " , N_eff)
            index = np.random.choice( num_prtcls , num_prtcls , p=weights)
            keep_prtcl=[]
            for i in range(num_prtcls):
                keep_prtcl.append(X_W[index[i]])
            
            X_W = keep_prtcl






