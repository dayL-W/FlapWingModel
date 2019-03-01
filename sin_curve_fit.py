# -*- coding: utf-8 -*-
"""
Created on Fri Jan 18 16:14:34 2019

@author: Liaowei
"""

import numpy as np
from  math import sin,cos,pi
import matplotlib.pyplot as plt
import pandas as pd
import data_visualiz
'''
pitch predict
1 2 3 4 5 7 10 11 12 13
'''

class Fit_Curve(object):
    def __init__(self,Fly_Count=1):
        self.G = 9.8
        self.rad_angle = 57.2957795
#        self.log_fre = 130
#        self.T_k = -125
#        self.T_b = 78#75
        self.log_fre = 1
        self.T_k = -0.961
        self.T_b = 0.60#0.61 #59
        self.Fly_Count = Fly_Count
        self.path = '../cache/curve_data/'+str(Fly_Count)+'data.csv'
        self.df = pd.read_csv(self.path, index_col=0)
        self.df_pred = pd.DataFrame(columns=self.df.columns,index=self.df.index)
        self.df_pred.TIME_StartTime = self.df.TIME_StartTime
        
        self.t = np.array(self.df.TIME_StartTime * 1e-6)
        self.t = self.t - self.t[0]
        self.num = len(self.t)
        
        throtle = self.df.RC_C2.values
        T = (self.T_k * throtle + self.T_b) / self.log_fre
        self.W = 2*pi / T
        self.fan = np.zeros(self.num)
        sum_fan = 0
        for i in range(1,self.num):
            if self.W[i] != self.W[i-1]:
                sum_fan += (self.W[i-1]*self.t[i-1] - self.W[i]*self.t[i])
            self.fan[i] = sum_fan
            
        self.gyro_cols = ['IMU_GyroX', 'IMU_GyroY', 'IMU_GyroZ']
        self.acc_cols = ['IMU_AccX', 'IMU_AccY', 'IMU_AccZ']
        self.angle_cols = ['ATT_Roll', 'ATT_Pitch', 'ATT_Yaw']
        self.vel_cols = ['LPOS_VX', 'LPOS_VY', 'LPOS_VZ']
        self.pos_cols = ['LPOS_X', 'LPOS_Y', 'LPOS_Z']
        self.q_cols = ['ATT_qw', 'ATT_qx', 'ATT_qy', 'ATT_qz']
        self.acc_ned_cols = ['NED_AccX', 'NED_AccY', 'NED_AccZ']
        self.insert_cols = ['Ctr_Roll','Ctr_Pitch','Ctr_Yaw',\
                            'Ctr_GX','Ctr_GY','Ctr_GZ']
        self.ctrG_cols = ['Ctr_GX','Ctr_GY','Ctr_GZ']
        self.ctrA_cols = ['Ctr_Roll','Ctr_Pitch','Ctr_Yaw']
        
        temp = pd.DataFrame(columns=self.insert_cols)
        self.df_pred = pd.concat([self.df_pred,temp])
        
    #坐标转换
    def frame_transform(self,data,q):
        data = np.matrix(data)
        q0,q1,q2,q3 = q

        R = np.matrix(np.zeros((3,3)))
        R[0,0] = 1 - 2*q2**2 - 2*q3**2
        R[0,1] = 2*q1*q2 - 2*q0*q3
        R[0,2] = 2*q1*q3 + 2*q0*q2
        R[1,0] = 2*q1*q2 + 2*q0*q3
        R[1,1] = 1 - 2*q1**2 - 2*q3**2
        R[1,2] = 2*q2*q3 - 2*q0*q1
        R[2,0] = 2*q1*q3 - 2*q0*q2
        R[2,1] = 2*q2*q3 + 2*q0*q1
        R[2,2] = 1 - 2*q1**2 - 2*q2**2
        
        data = np.array(np.dot(R, data.T),dtype=np.float32)
        data = np.squeeze(data)
        data = data.reshape((len(data),))
        return data
    def gyro_fit(self):
        #GyroX
        x0 = np.array([cos(self.t[i]*self.W[i]+self.fan[i]) for i in range(self.num)]).reshape((self.num,1))
        x1 = np.array([sin(self.t[i]*self.W[i]+self.fan[i]) for i in range(self.num)]).reshape((self.num,1))
        x2 = np.ones((self.num,1))
        M = np.hstack([x0, x1, x2])
        GyroX = (self.df.IMU_GyroX.values).reshape((self.num,1))
        mMat = np.mat(M)
        yMat = np.mat(GyroX)
        
        self.gyroX_w = (mMat.T * mMat).I * M.T * yMat
        w0 = self.gyroX_w[0,0]
        w1 = self.gyroX_w[1,0]
        w2 = self.gyroX_w[2,0]
        GyroX_ = w0*x0 + w1*x1 + w2*x2
        self.df_pred.IMU_GyroX = GyroX_
        self.df_pred.Ctr_GX =  w2*x2
        
        #GyroY
        u1 = self.df.RC_C1.values
        GyroY = (self.df.IMU_GyroY.values).reshape((self.num,1))
        x0 = np.array([cos(self.t[i]*self.W[i]+self.fan[i]) for i in range(self.num)]).reshape((self.num,1))
        x1 = np.array([sin(self.t[i]*self.W[i]+self.fan[i]) for i in range(self.num)]).reshape((self.num,1))
        x2 = u1.reshape((self.num,1))
        x3 = np.ones((self.num,1))
        M = np.hstack([x0, x1, x2, x3])
        mMat = np.mat(M)
        yMat = np.mat(GyroY)
        
        self.gyroY_w = (mMat.T * mMat).I * M.T * yMat
        w0 = self.gyroY_w[0,0]
        w1 = self.gyroY_w[1,0]
        w2 = self.gyroY_w[2,0]
        w3 = self.gyroY_w[3,0]
        GyroY_ = w0*x0 + w1*x1 + w2*x2 + w3*x3
        self.df_pred.IMU_GyroY = GyroY_
        self.df_pred.Ctr_GY =  w2*x2 + w3*x3
        
        #GyroZ
        u3 = (self.df.RC_C0.values).reshape((self.num,1))
        GyroZ = (self.df.IMU_GyroZ.values).reshape((self.num,1))
        x0 = np.array([cos(self.t[i]*self.W[i]+self.fan[i]) for i in range(self.num)]).reshape((self.num,1))
        x1 = np.array([sin(self.t[i]*self.W[i]+self.fan[i]) for i in range(self.num)]).reshape((self.num,1))
        x2 = u3.reshape((self.num,1))
        x3 = np.ones((self.num,1))
        
        M = np.hstack([x0, x1, x2, x3])
        mMat = np.mat(M)
        yMat = np.mat(GyroZ)
        self.gyroZ_w = (mMat.T * mMat).I * M.T * yMat
        w0 = self.gyroZ_w[0,0]
        w1 = self.gyroZ_w[1,0]
        w2 = self.gyroZ_w[2,0]
        w3 = self.gyroZ_w[3,0]
        GyroZ_ = x0 * w0 + x1 * w1 + x2 * w2 + x3 * w3
        self.df_pred.IMU_GyroZ = GyroZ_
        self.df_pred.Ctr_GZ =  w2*x2 + w3*x3
        
    def estimate_angle(self):
        index = self.df.index
        angle0 = self.df.loc[index[0],self.angle_cols]
        self.df_pred.loc[index[0],self.angle_cols] = angle0
        ctr_angle0 = self.df.loc[index[0],self.angle_cols]
        self.df_pred.loc[index[0],self.ctrA_cols] = ctr_angle0
        for i in index[1:]:
            gyro = np.array(self.df_pred.loc[i,self.gyro_cols])
            dt = (self.df.TIME_StartTime[i] - self.df.TIME_StartTime[i-1]) / 1e6
            angle0 += gyro * dt
            self.df_pred.loc[i,self.angle_cols] = angle0
            ctr_gyro = np.array(self.df_pred.loc[i,self.ctrG_cols])
            ctr_angle0 += ctr_gyro * dt
            self.df_pred.loc[i,self.ctrA_cols] = ctr_angle0.values
            
            q = np.array([0.0,0.0,0.0,0.0])
            q[0] = cos(angle0[0]/2)*cos(angle0[1]/2)*cos(angle0[2]/2) + sin(angle0[0]/2)*sin(angle0[1]/2)*sin(angle0[2]/2)
            q[1] = sin(angle0[0]/2)*cos(angle0[1]/2)*cos(angle0[2]/2) - cos(angle0[0]/2)*sin(angle0[1]/2)*sin(angle0[2]/2)
            q[2] = cos(angle0[0]/2)*sin(angle0[1]/2)*cos(angle0[2]/2) + sin(angle0[0]/2)*cos(angle0[1]/2)*sin(angle0[2]/2)
            q[3] = cos(angle0[0]/2)*cos(angle0[1]/2)*sin(angle0[2]/2) - sin(angle0[0]/2)*sin(angle0[1]/2)*cos(angle0[2]/2)
            self.df_pred.loc[i,self.q_cols] = q
            
    def acc_fit(self):
        #AccX   
        Pitch = self.df_pred.ATT_Pitch.values
        u4 = self.df.RC_C2.values
        AccX = (self.df.IMU_AccX.values).reshape((self.num,1))
        x0 = np.array([cos(self.t[i]*self.W[i]+self.fan[i]) for i in range(self.num)]).reshape((self.num,1))
        x1 = np.array([sin(self.t[i]*self.W[i]+self.fan[i]) for i in range(self.num)]).reshape((self.num,1))
        x2 = np.array([self.G*sin(i) for i in Pitch]).reshape((self.num,1))
        x3 = (u4).reshape((self.num,1))
        x4 = np.ones((self.num,1))
        M = np.hstack([x0, x1, x2, x3, x4])
        mMat = np.mat(M)
        yMat = np.mat(AccX)
        self.AccX_w = (mMat.T * mMat).I * M.T * yMat
        w0 = self.AccX_w[0,0]
        w1 = self.AccX_w[1,0]
        w2 = self.AccX_w[2,0]
        w3 = self.AccX_w[3,0]
        w4 = self.AccX_w[4,0]
        AccX_ = w0*x0 + w1*x1 + w2*x2 + w3*x3 + w4*x4
        self.df_pred.IMU_AccX = AccX_
        
        #AccY   
        Roll = self.df_pred.ATT_Roll.values
        u2 = self.df.RC_C0.values
        AccY = (self.df.IMU_AccY.values).reshape((self.num,1))
        x0 = np.array([cos(self.t[i]*self.W[i]+self.fan[i]) for i in range(self.num)]).reshape((self.num,1))
        x1 = np.array([sin(self.t[i]*self.W[i]+self.fan[i]) for i in range(self.num)]).reshape((self.num,1))
        x2 = np.array([self.G*sin(i) for i in Roll]).reshape((self.num,1))
        x3 = (u2).reshape((self.num,1))
        x4 = np.ones((self.num,1))
        M = np.hstack([x0, x1, x2, x3, x4])
        mMat = np.mat(M)
        yMat = np.mat(AccY)
        self.AccY_w = (mMat.T * mMat).I * M.T * yMat
        w0 = self.AccY_w[0,0]
        w1 = self.AccY_w[1,0]
        w2 = self.AccY_w[2,0]
        w3 = self.AccY_w[3,0]
        w4 = self.AccY_w[4,0]
        AccY_ = w0*x0 + w1*x1 + w2*x2 + w3*x3 + w4*x4
        self.df_pred.IMU_AccY = AccY_
        
        #AccZ
        Pitch = self.df_pred.ATT_Pitch.values
        u4 = self.df.RC_C2.values
        AccZ = (self.df.IMU_AccZ.values).reshape((self.num,1))
        x0 = np.array([cos(self.t[i]*self.W[i]+self.fan[i]) for i in range(self.num)]).reshape((self.num,1))
        x1 = np.array([sin(self.t[i]*self.W[i]+self.fan[i]) for i in range(self.num)]).reshape((self.num,1))
        x2 = np.array([self.G*cos(i) for i in Pitch]).reshape((self.num,1))
        x3 = (u4).reshape((self.num,1))
        x4 = np.ones((self.num,1))
        M = np.hstack([x0, x1, x2, x3, x4])
        mMat = np.mat(M)
        yMat = np.mat(AccZ)
        self.AccZ_w = (mMat.T * mMat).I * M.T * yMat
        w0 = self.AccZ_w[0,0]
        w1 = self.AccZ_w[1,0]
        w2 = self.AccZ_w[2,0]
        w3 = self.AccZ_w[3,0]
        w4 = self.AccZ_w[4,0]
        AccZ_ = w0*x0 + w1*x1 + w2*x2 + w3*x3 + w4*x4
        self.df_pred.IMU_AccZ = AccZ_
    
    #估计对地加速度、速度、位置
    def estimate_pos(self):
        #获取初始值
        index = self.df.index
        pos0 = self.df.loc[index[0], self.pos_cols].values
        vel0 = self.df.loc[index[0], self.vel_cols].values
        acc0 = self.df.loc[index[0], self.acc_cols].values
        q0 = self.df.loc[index[0], self.q_cols].values
        acc0 = self.frame_transform(acc0,q0)
        acc0[2] += self.G
            
        self.df_pred.loc[index[0], self.vel_cols] = vel0
        self.df_pred.loc[index[0], self.pos_cols] = pos0
        self.df_pred.loc[index[0], self.acc_ned_cols] = acc0   
        vel1 = vel0
        pos1 = pos0
        for t in index[1:]:
            acc1 = self.df_pred.loc[t, self.acc_cols].values
            q1 = self.df_pred.loc[t, self.q_cols].values
            acc1 = self.frame_transform(acc1,q1)
            acc1[2] += self.G
            
            #积分时间
            dt = (self.df_pred.loc[t,'TIME_StartTime'] - self.df_pred.loc[t-1,'TIME_StartTime']) / 1e6
            #预估速度和位置
            pos1 += vel1 * dt + 0.5 * acc1 * dt**2
            vel1 += acc1*dt
            
            self.df_pred.loc[t, self.vel_cols] = vel1
            self.df_pred.loc[t, self.pos_cols] = pos1
            self.df_pred.loc[t, self.acc_ned_cols] = acc1
            vel0 = vel1
            pos0 = pos1
#    def control_angle(self):
#        index = self.df_pred.index
#        max_angle = 0
#        min_angle = 0
#        first = 1
#        for i in range(1,len(index)):
#            if self.df_pred.IMU_GyroY[index[i-1]] > 0.0 and self.df_pred.IMU_GyroY[index[i]] < 0.0:
#                max_angle = self.df_pred.ATT_Pitch[index[i]]
#                if first==1:
#                    first = 0
#                else:
#                    self.df_pred.Ctr_Pitch[:i] = (max_angle + min_angle)/2
#            if self.df_pred.IMU_GyroY[index[i-1]] < 0.0 and self.df_pred.IMU_GyroY[index[i]] > 0.0:
#                min_angle = self.df_pred.ATT_Pitch[index[i]]
#                if first==1:
#                    first = 0
#                else:
#                    self.df_pred.Ctr_Pitch[index[i]] = (max_angle + min_angle)/2    
'''
周期拟合：
0.59：  1，2，3，4，7,8
0.61：11
0.63：10，12，13
不好：9，10，11,12，13，14
'''
if __name__ == '__main__':
    Fly_Count = 8
    FitCurve = Fit_Curve(Fly_Count=Fly_Count)
    FitCurve.gyro_fit()
    FitCurve.estimate_angle()
    FitCurve.acc_fit()
    FitCurve.estimate_pos()
#    FitCurve.control_angle()
    
    fit_show = data_visualiz.DataShow(df=FitCurve.df, df_pred=FitCurve.df_pred, path='curve_fit/')
    fit_show.gyro(Fly_Count=Fly_Count)
    fit_show.acc(Fly_Count=Fly_Count)
    fit_show.angle(Fly_Count=Fly_Count)
    fit_show.vel(Fly_Count=Fly_Count)
#    FitCurve.accX_fit()