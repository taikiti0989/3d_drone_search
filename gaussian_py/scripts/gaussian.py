#!/usr/bin/env python3
import rospy
from itertools import product
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from gaussian_py.msg import kl_suzu
from gaussian_py.msg import kl_suzuki
from statistics import stdev
import time
import math

from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, ConstantKernel as C
def min_max(x,axis=None):

    min = x.min(axis=axis, keepdims=True)
    max = x.max(axis=axis, keepdims=True)
    result = (x-min)/(max-min)
    return result
semi = kl_suzu()
#tf = int
tf = 0
t1=0
def klCallback1(msg):
    #tf = int = 0
    global tf;
    global t1;
    if tf == 0:
        tf = 1
        t1 = time.time()
        #print("t1 :",t1)
    else:
  #if msg.KL[0].kl_score != 0:
        t2 = time.time() - t1
        #print("tt :",t1)
        #print("tt :",t2)
        size=len(msg.KL)
        #X = np.empty([1,2])
        #y = np.empty([1,1])
        for i in range(0,size):
            if i==0:
                X=np.array([[msg.KL[i].kl_x,msg.KL[i].kl_y,msg.KL[i].kl_z]])
                y=np.array(msg.KL[0].kl_score)
            else:
                b=np.array([[msg.KL[i].kl_x,msg.KL[i].kl_y,msg.KL[i].kl_z]])
                X=np.vstack((X,b))
                c=np.array(msg.KL[i].kl_score)
                y=np.hstack((y,c))
        
        # Input space
        x1 = np.linspace(-1.5, 1.5) #p
        x2 = np.linspace(-0.75, 0.75) #q
        x3 = np.linspace(0.8, 1.8)
        x = (np.array([x1, x2,x3])).T
        #kernel = C(1.0, (1e-3, 1e3)) * RBF([5,5], (1e-2, 1e2))
        kernel = RBF()+C()
        dy = 0.5   #std_max
        gp = GaussianProcessRegressor(kernel=kernel,alpha=dy**2, n_restarts_optimizer=5)

        gp.fit(X, y)

        x1x2x3 = np.array(list(product(x1, x2,x3)))
	
        y_pred, MSE = gp.predict(x1x2x3, return_std=True)

        X0p, X1p, X2p = x1x2x3[:,0].reshape(50,50,50), x1x2x3[:,1].reshape(50,50,50), x1x2x3[:,2].reshape(50,50,50)
        Zp = np.reshape(y_pred,(50,50,50))
        pos = np.unravel_index(np.argmin(Zp),Zp.shape)
        Zpp = np.reshape(MSE,(50,50,50))
        print("t2 :",t2)
        si = np.power(Zpp,1/2)
        kl = Zp/si
        zzz=kl-10
        ZP = np.power(math.e,zzz*0.1)
        ZP = 1/(1+ZP)

        pos = np.unravel_index(np.argmax(ZP),ZP.shape)
        pos1 = np.unravel_index(np.argmin(Zp),Zp.shape)
        
        lb_x = X0p[pos[0]][pos[1]][pos[2]]
        lb_y = X1p[pos[0]][pos[1]][pos[2]]
        lb_z = X2p[pos[0]][pos[1]][pos[2]]
        lb_kl = ZP[pos[0]][pos[1]][pos[2]]

        min_x = X0p[pos1[0]][pos1[1]][pos1[2]]
        min_y = X1p[pos1[0]][pos1[1]][pos1[2]]
        min_z = X2p[pos1[0]][pos1[1]][pos1[2]]
        min_kl = Zp[pos1[0]][pos1[1]][pos1[2]]
        #print("1111: ",Zp[pos1[0]][pos1[1]])
        #print("2222: ",Zp[pos[0]][pos[1]])
        #print(min_x)
        #print(min_y)
        #print(min_kl)
        #semi = kl_suzu()
        semi.kl_score = min_kl
        semi.kl_x = lb_x
        semi.kl_y = lb_y
        semi.kl_z = lb_z
        print("lb_score: ",lb_kl)
        print("lb_x: ",lb_x)
        print("lb_y: ",lb_y)
        print("lb_z: ",lb_z)
        print("")
        print("kl_score: ",min_kl)
        print("min_x: ",min_x)
        print("min_y: ",min_y)
        print("min_z: ",min_z)
        #print("kl_z: ",semi.kl_z)
        print("X.shape:", X.shape)
        print("y.shape:", y.shape)
        print("//////////////////////////////")

        # kl_py_pub.publish(semi)
        # """
        # global flagp
        # print("flag",flagp)
        # if flagp == 0:
        #     flagp = 1
        #     print("flag",flagp)
        #     fig = plt.figure(figsize=(10,8))

        # xdata = X[:,0]
        # ydata = X[:,1]
        # ax = fig.add_subplot(111, projection='3d')      
        # surf = ax.plot_surface(X0p, X1p, Zp, rstride=1, cstride=1, cmap='jet', linewidth=0, antialiased=False)
        # ax.scatter3D(xdata, ydata, y, c=y, cmap='binary');  
        # plt.show()
        # if flagg == 0:
        #     flagg=1
        #     print(flagg)
        #     """
        # plt.ion()
      
        # print("X0p shape ~ ", X0p.shape)
        # print("X1p shape ~ ", X1p.shape)
        # print("X2p shape ~ ", X2p.shape)
        
	
def gauss_generate():
    kl_py_pub = rospy.Publisher('semi_opt', kl_suzu, queue_size = 10)
    rospy.init_node('gauss_generate')
    kl_sub1 = rospy.Subscriber("kl_suzuki_msg", kl_suzuki, klCallback1, queue_size=1)
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        kl_py_pub.publish(semi)
         #print(semi.kl_score)
        r.sleep()
        
    # rospy.spin()
if __name__ == '__main__':
    try:
        gauss_generate()
    except rospy.ROSInterruptException: pass