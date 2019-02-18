# -*- coding: utf-8 -*-
"""
Created on Fri Jan 18 16:14:34 2019

@author: Liaowei
"""

import numpy as np
from  math import sin,cos,pi
import matplotlib.pyplot as plt
import pandas as pd

'''
pitch predict
1 2 3 4 5 7 10 11 12 13
'''

class Fit_Curve(object):
    def __init__(self,Fly_Count=1):
        self.G = 9.8
        self.rad_angle = 57.2957795
        self.log_fre = 130
        self.T_k = -125
        self.T_b = 75
        self.Fly_Count = Fly_Count
        self.path = '../cache/curve_data/'+str(Fly_Count)+'data.csv'
        self.df = pd.read_csv(self.path, index_col=0)
        
        self.t = np.array(self.df.index / self.log_fre)
        self.num = len(self.t)
    
    def gyro_fit(self):
        throtle = self.df.RC_C2.values
        T = (self.T_k * throtle + self.T_b) / self.log_fre
        W = 2*pi / T
        fan = np.zeros(self.num)
        sum_fan = 0
        for i in range(1,self.num):
            if W[i] != W[i-1]:
                sum_fan += (W[i-1]*self.t[i-1] - W[i]*self.t[i])
            fan[i] = sum_fan
        
        u1 = self.df.RC_C1.values
        vx_b = self.df.Vehicle_VelX.values
        
        x0 = np.array([cos(self.t[i]*W[i]+fan[i]) for i in range(self.num)]).reshape((self.num,1))
        x1 = np.array([sin(self.t[i]*W[i]+fan[i]) for i in range(self.num)]).reshape((self.num,1))
        x2 = u1.reshape((self.num,1))
        x3 = np.ones((self.num,1))
        
        M = np.hstack([x0, x1, x2, x3])
        GyroY = (self.df.IMU_GyroY.values).reshape((self.num,1))
        
        mMat = np.mat(M)
        yMat = np.mat(GyroY)
        
        w = (mMat.T * mMat).I * M.T * yMat
        w0 = w[0,0]
        w1 = w[1,0]
        w2 = w[2,0]
        w3 = w[3,0]
        GyroY_ = w0*x0 + w1*x1 + w2*x2 + w3*x3
        
        Pitch = self.df.ATT_Pitch.values
        Pitch0 = Pitch[0]
        Pitch_ = np.zeros(self.num)
        Pitch_[0] = Pitch0
        for i in range(1,self.num):
            Pitch0 += GyroY_[i] / self.log_fre
            Pitch_[i] = Pitch0
        
        plt.figure(1, figsize=(6,4))
        plt.subplot(211)
        plt.margins(0,0)
        plt.title('gyro predict')
        plt.ylabel('GyroY rad/s')
        plt.plot(self.t, GyroY, 'r-')
        plt.plot(self.t, GyroY_, 'g--')
        
        plt.subplot(212)
        plt.margins(0,0)
        plt.ylabel('Pitch rad')
        plt.xlabel('time s')
        plt.plot(self.t, Pitch, 'r-')
        plt.plot(self.t, Pitch_, 'g--')
        img_name = '../img/curve_fit/'+str(self.Fly_Count)+'GyroY_Fit.jpg'
        plt.savefig(img_name,bbox_inches = 'tight')
        
    def acc_fit(self):
        throtle = self.df.RC_C2.values
        T = (self.T_k * throtle + self.T_b) / self.log_fre
        W = 2*pi / T
        fan = np.zeros(self.num)
        sum_fan = 0
        for i in range(1,self.num):
            if W[i] != W[i-1]:
                sum_fan += (W[i-1]*self.t[i-1] - W[i]*self.t[i])
            fan[i] = sum_fan
            
        Pitch = self.df.ATT_Pitch.values
        u4 = self.df.RC_C2.values
        
        x0 = np.array([cos(self.t[i]*W[i]+fan[i]) for i in range(self.num)]).reshape((self.num,1))
        x1 = np.array([sin(self.t[i]*W[i]+fan[i]) for i in range(self.num)]).reshape((self.num,1))
        x2 = np.array([self.G*sin(i) for i in Pitch]).reshape((self.num,1))
        x3 = (u4).reshape((self.num,1))
        x4 = np.ones((self.num,1))
        
        M = np.hstack([x0, x1, x2, x3, x4])
        AccX = (self.df.IMU_AccX.values).reshape((self.num,1))
        
        mMat = np.mat(M)
        yMat = np.mat(AccX)
        
        w = (mMat.T * mMat).I * M.T * yMat
        
        w0 = w[0,0]
        w1 = w[1,0]
        w2 = w[2,0]
        w3 = w[3,0]
        w4 = w[4,0]
        AccX_ = w0*x0 + w1*x1 + w2*x2 + w3*x3 + w4*x4
        
        VelX = self.df.Vehicle_VelX.values
        VelX_ = np.zeros(self.num)
        VelX0 = VelX[0]
        VelX_[0] = VelX[0]
        for i in range(1,self.num):
            VelX0 += AccX_[i] / self.log_fre
            VelX_[i] = VelX0
    
        plt.figure(2, figsize=(6,4))
        plt.subplot(211)
        plt.margins(0,0)
        plt.title('acc x predict')
        plt.ylabel('acc m/s2')
        plt.plot(self.t, AccX, 'r-')
        plt.plot(self.t, AccX_, 'g--')
        
        plt.subplot(212)
        plt.margins(0,0)
        plt.ylabel('Vel m/x')
        plt.xlabel('time s')
        plt.plot(self.t, VelX, 'r-')
        plt.plot(self.t, VelX_, 'g--')
        img_name = '../img/curve_fit/'+str(self.Fly_Count)+'AccX_Fit.jpg'
        plt.savefig(img_name,bbox_inches = 'tight')
'''
pitch predict
1 2 3 4 5 7 10 11 12 13
'''
if __name__ == '__main__':
    FitCurve = Fit_Curve(Fly_Count=4)
    FitCurve.gyro_fit()
    FitCurve.acc_fit()