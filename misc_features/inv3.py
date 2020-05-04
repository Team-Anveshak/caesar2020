import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

class invkin():
    def __init__(self):
        count = 0
        i=0
        #length of links
        self.l1=7.0
        self.l2=5.0
# z is always positive , d coordinate axis is along the links and passes through z axis ,
# positve in the direction of end-effector
        fig = plt.figure()
        self.ax = fig.add_subplot( 111 , projection = '3d')
        self.ax.set_xlim3d(-13, 13)
        self.ax.set_ylim3d(-13, 13)
        self.ax.set_zlim3d(0, 13)

        #get coordinates
        self.x1 = input("\nenter x1 coordinate :")
        self.y1 = input("\nenter y1 coordinate :")
        self.z1 = input("\nenter z1 coordinate :")

        self.x2 = input("\nenter x2 coordinate :")
        self.y2 = input("\nenter y2 coordinate :")
        self.z2 = input("\nenter z2 coordinate :")

        self.xt=float(self.x1)
        self.yt=float(self.y1)
        self.zt=float(self.z1)

        self.a = self.x2 - self.x1
        self.b = self.y2 - self.y1
        self.c = self.z2 - self.z1

    def d_calc(self,xt,yt):
        # calculate d for current position
        x = np.power((xt),2)
        y = np.power((yt),2)
        d = np.power(x+y,0.5)
        return d

    def theta_finder(self):
        t = 0.0
        while(t<=1):
            self.theta1 = np.arctan2(self.yt,self.xt)
            self.dt = self.d_calc(self.xt,self.yt)
            self.gamma = np.arctan2(self.zt,self.dt)
            b1 = self.l1*self.l1 + self.l2*self.l2 - self.dt*self.dt - self.zt*self.zt
            b2 = 2*self.l1*self.l2
            b = (b1/b2)
            b = max(-1,b)
            b = min(1,b)
            self.beta = np.arccos(b)
            a1 = self.l1*self.l1 - self.l2*self.l2 + self.dt*self.dt + self.zt*self.zt
            a2 = 2*self.l1*np.power((self.dt*self.dt + self.zt*self.zt),0.5)
            a = (a1/a2)
            a = max(-1,a)
            a = min(1,a)
            self.alpha = np.arccos(a)
            self.theta2 = self.gamma + self.alpha
            self.theta3 = self.beta - np.pi
            #self.xplot.append()
            self.plotter(self.theta1,self.theta2,self.theta3)
            t = t + 0.05
            self.xt = self.a*t + self.x1
            self.yt = self.b*t + self.y1
            self.zt = self.c*t + self.z1
            plt.pause(0.1)

        self.plot_path()

    def plotter(self,theta1,theta2,theta3):
        z = self.l1*np.sin(theta2) + self.l2*np.sin(theta3+theta2)
        t = self.l1*np.cos(theta2) + self.l2*np.cos(theta3+theta2)
        x = t*np.cos(theta1)
        y = t*np.sin(theta1)
        zl = self.l1*np.sin(theta2)
        tl = self.l1*np.cos(theta2)
        xl = tl*np.cos(theta1)
        yl = tl*np.sin(theta1)
        self.ax.plot([0,xl],[0,yl],[0,zl],'red')
        self.ax.plot([xl,x],[yl,y],[zl,z],'blue')
        #self.ax.plot([self.x1,self.x2],[self.y1,self.y2],[self.z1,self.z2],'green')


    def plot_path(self):
        self.ax.plot([self.x1,self.x2],[self.y1,self.y2],[self.z1,self.z2],'green')
        plt.show()

if __name__ == '__main__':
    p=invkin()
    p.theta_finder()
