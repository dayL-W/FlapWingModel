# -*- coding: utf-8 -*-
"""
Created on Sat Dec 29 12:50:57 2018

@author: Liaowei
"""

import pandas as pd
import numpy as np
import time
from math import tan,cos,atan2,asin,sin
import data_visualiz
import os
from tinyekf import EKF
import queue

#二阶低通滤波器
class Low_Pass_Filter(object):
    def __init__(self, sample_freq=100, cutoff_freq=30):
        self.M_PI_F = 3.14159265358979323846
        self.sample_freq = sample_freq
        self.cutoff_freq = cutoff_freq
        self.fr = self.sample_freq/self.cutoff_freq
        self.ohm = tan(self.M_PI_F/self.fr)
        self.c = 1.0+2.0*cos(self.M_PI_F/4.0)*self.ohm + self.ohm**2
        self._b0 = self.ohm**2/self.c
        self._b1 = 2.0*self._b0
        self._b2 = self._b0
        self._a1 = 2.0*(self.ohm**2-1.0)/self.c
        self._a2 = (1.0-2.0*cos(self.M_PI_F/4.0)*self.ohm+self.ohm**2)/self.c
        self._delay_element_1 = 0
        self._delay_element_2 = 0
        
    def apply(self, sample):
        if self.cutoff_freq <= 0.0:
            return sample
    
        delay_element_0 = sample - self._delay_element_1 * self._a1 - \
                            self._delay_element_2 * self._a2
        output = delay_element_0 * self._b0 + self._delay_element_1 * \
                    self._b1 + self._delay_element_2 * self._b2
        
        self._delay_element_2 = self._delay_element_1
        self._delay_element_1 = delay_element_0

        return output

class Moving_Average_Filter(object):
    def __init__(self, N=30):
        self.N = N
        self.accx = queue.Queue(maxsize=self.N)
        self.accy = queue.Queue(maxsize=self.N)
        self.accz = queue.Queue(maxsize=self.N)
    
    def apply(self, sample):
        x,y,z = sample
        if self.accx.full():
            tmp = self.accx.get_nowait()
            tmp = self.accy.get_nowait()
            tmp = self.accz.get_nowait()
            tmp = tmp
        self.accx.put_nowait(x)
        self.accy.put_nowait(y)
        self.accz.put_nowait(z)
        x = np.sum(self.accx.queue) / self.accx.qsize()
        y = np.sum(self.accy.queue) / self.accy.qsize()
        z = np.sum(self.accz.queue) / self.accz.qsize()
        
        return np.array([x,y,z])
#数据处理
class Data(object):
    def __init__(self, file):
        self.file = file
        self.data_df = pd.DataFrame()
        self.Fly_Count = 1
    def get_Fly_Count(self):
        count = 1
        Fly_Count_array = [1]
        last_index = self.data_df.index[0]
        
        for i in self.data_df.index[1:]:
            time_gap = self.data_df.loc[i,'TIME_Sec'] - self.data_df.loc[last_index,'TIME_Sec']
            if time_gap>= 4:
                count += 1
            Fly_Count_array.append(count)
            last_index = i
        
        return np.array(Fly_Count_array)
    
    def read_data(self):
        self.data_df = pd.read_csv(filepath_or_buffer=self.file)
        return self.data_df
    
    def process_data(self, df, test=True):
        index = df.TIME_StartTime.drop_duplicates().index
        df = df.loc[index,:]
        index = df.RC_C2.dropna().index
        df = df.loc[index,:]
        
        if test:
            df.LAND_Landed = 0
        else:
            #选取位置估算合法的数据
            drop_index = df.loc[df.LPOS_X==0].index
            df.drop(drop_index,inplace=True)
        #选取飞行数据
        index = df.loc[df.LAND_Landed==0].index
        df = df.loc[index,:]
        #去除油门为0的数据
        drop_index = df.loc[df.RC_C2==0].index
        df.drop(drop_index,inplace=True)
        df['TIME_Sec'] = df['TIME_StartTime'] / 1000000
        self.data_df = df.copy()
        df['Fly_Count'] = self.get_Fly_Count()
        
        df.to_csv('../cache/fly_data.csv')
        return df
#姿态解算
class Imu(object):
    def __init__(self, init_q, imu_freq, Kp, Ki):
        self.q = init_q
        self.Kp = Kp
        self.Ki = Ki
        self.halfT = 1/(2*imu_freq)
        self.exInt = 0
        self.eyInt = 0
        self.ezInt = 0
        self.gyro_xoff = 0.000
        self.gyro_yoff = 0.000
        self.gyro_zoff = -0.026
        self.acc_xoff = -0.224
        self.acc_yoff = 0.026
        self.acc_zoff = 0.203
        self.yaw = 0

    def update(self, gyro, acc):   
        q0 = self.q[0]
        q1 = self.q[1]
        q2 = self.q[2]
        q3 = self.q[3]
        
        q0q0 = q0 * q0
        q0q1 = q0 * q1
        q0q2 = q0 * q2
        q1q1 = q1 * q1
        q1q3 = q1 * q3
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q3q3 = q3 * q3
        acc = acc / np.sqrt(np.sum(np.multiply(acc,acc)))
        
        vx = 2*(q1q3 - q0q2);								
        vy = 2*(q0q1 + q2q3);
        vz = q0q0 - q1q1 - q2q2 + q3q3;
        
        ax = -1*acc[0]
        ay = -1*acc[1]
        az = -1*acc[2]
        ex = ay * vz - az * vy
        ey = az * vx - ax * vz
        ez = ax * vy - ay * vx    
        
        self.exInt = self.exInt + ex * self.Ki
        self.eyInt = self.eyInt + ey * self.Ki
        self.ezInt = self.ezInt + ez * self.Ki    
        
        gx = gyro[0] + self.gyro_xoff
        gy = gyro[1] + self.gyro_yoff
        gz = gyro[2] + self.gyro_zoff
        gx = gx + self.Kp * ex + self.exInt
        gy = gy + self.Kp * ey + self.eyInt
        gz = gz + self.Kp * ez + self.ezInt
        
        q0t = self.q[0]
        q1t = self.q[1]
        q2t = self.q[2]
        q3t = self.q[3]
        self.q[0] = q0t + (-q1t*gx - q2t*gy -q3t*gz) * self.halfT
        self.q[1] = q1t + (q0t*gx + q2t*gz -q3t*gy) * self.halfT
        self.q[2] = q2t + (q0t*gy - q1t*gz +q3t*gx) * self.halfT
        self.q[3] = q3t + (q0t*gz + q1t*gy -q2t*gx) * self.halfT
        self.q = self.q / np.sqrt(np.sum(np.multiply(self.q, self.q)))
        
        q0 = self.q[0]
        q1 = self.q[1]
        q2 = self.q[2]
        q3 = self.q[3]
        
        roll = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2))
        pitch = asin(2*(q0*q2 - q3*q1))
        yaw = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3))
        
        return (self.q, np.array([roll, pitch, yaw]))
class Integral_Angle(object):
    def __init__(self,init_a):
        self.angle = init_a
        
    def euler_to_quate(self, euler):
        q = np.array([0.0,0.0,0.0,0.0])
        
        q[0] = cos(euler[0]/2)*cos(euler[1]/2)*cos(euler[2]/2) + sin(euler[0]/2)*sin(euler[1]/2)*sin(euler[2]/2)
        q[1] = sin(euler[0]/2)*cos(euler[1]/2)*cos(euler[2]/2) - cos(euler[0]/2)*sin(euler[1]/2)*sin(euler[2]/2)
        q[2] = cos(euler[0]/2)*sin(euler[1]/2)*cos(euler[2]/2) + sin(euler[0]/2)*cos(euler[1]/2)*sin(euler[2]/2)
        q[3] = cos(euler[0]/2)*cos(euler[1]/2)*sin(euler[2]/2) - sin(euler[0]/2)*sin(euler[1]/2)*cos(euler[2]/2)
        return q
    
    def update(self, gyro, dt):
        self.angle += gyro * dt
        q = self.euler_to_quate(self.angle)
        
        return (q, self.angle)
class ACC_EKF(EKF):
    '''
    An abstract class for fusing baro and sonar.  
    '''

    def __init__(self, x, freq):
        EKF.__init__(self, 9, 6, qval=[0.001]*9,rval=[0.008,0.008,0.008,0.04,0.04,0.04],init_x=x)
        self.dt = 1/freq

    def f(self, x):
        A = np.eye(9)
        A[0,3] = self.dt
        A[1,4] = self.dt
        A[2,5] = self.dt
        return np.dot(A,x), A


    def h(self, x):
        h = np.array([x[0],x[1],x[2],x[6],x[7],x[8]])
        H = np.zeros((6,9))
        H[0,0] = 1
        H[1,1] = 1
        H[2,2] = 1
        H[3,6] = 1
        H[4,7] = 1
        H[5,8] = 1
        return h, H
            
class POS_EKF(EKF):
    '''
    An abstract class for fusing baro and sonar.  
    '''

    def __init__(self, x, freq):
        EKF.__init__(self, 9, 3, qval=[0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001],rval=[0.09,0.09,0.09],init_x=x)
        self.dt = 1/freq

    def f(self, x):
        A = np.eye(9)
        A[0,3] = self.dt
#        A[0,6] = 0.5*self.dt**2
        A[1,4] = self.dt
#        A[1,7] = 0.5*self.dt**2
        A[2,5] = self.dt
#        A[2,8] = 0.5*self.dt**2
        
        A[3,6] = self.dt
        A[4,7] = self.dt
        A[5,8] = self.dt
        return np.dot(A,x), A


    def h(self, x):
        h = np.array([x[6],x[7],x[8]])
        H = np.zeros((3,9))
        H[0,6] = 1
        H[1,7] = 1
        H[2,8] = 1
        return h, H
'''
模型测试
1、完成姿态估计
2、完成位置估计
'''
class ModelTest(object):
    def __init__(self, file, Kp, Ki, log_freq=100, test=True):
        self.file = file
        self.Kp = Kp
        self.Ki = Ki
        self.test = test
        self.log_freq = log_freq
        
        self.Data = Data(file=file)
        self.df = self.Data.read_data()
        self.df_pred = pd.DataFrame(columns=self.df.columns,index=self.df.index)
        self.data_len = len(self.df)
        
        self.Gyro_lp = Low_Pass_Filter(sample_freq=500, cutoff_freq=30)
        self.Acc_lp = Low_Pass_Filter(sample_freq=500, cutoff_freq=30)
        self.Angle_acc_lp = Low_Pass_Filter(sample_freq=500, cutoff_freq=30)
        
        self.insert_cols = ['IMU_AngleAccX', 'IMU_AngleAccY', 'IMU_AngleAccZ', 'NED_AccX', 'NED_AccY', 'NED_AccZ', \
                            'Avg_AccX', 'Avg_AccY', 'Avg_AccZ', 'Fly_Count','TIME_Sec',\
                            'Vehicle_VelX','Vehicle_VelY','Vehicle_VelZ'] 
        self.gyro_cols = ['IMU_GyroX', 'IMU_GyroY', 'IMU_GyroZ']
        self.acc_cols = ['IMU_AccX', 'IMU_AccY', 'IMU_AccZ']
        self.angle_cols = ['ATT_Roll', 'ATT_Pitch', 'ATT_Yaw']
        self.vel_cols = ['LPOS_VX', 'LPOS_VY', 'LPOS_VZ']
        self.veh_vel_cols = ['Vehicle_VelX','Vehicle_VelY','Vehicle_VelZ']
        self.pos_cols = ['LPOS_X', 'LPOS_Y', 'LPOS_Z']
        self.acc_avg_cols = ['Avg_AccX', 'Avg_AccY', 'Avg_AccZ']
        self.acc_ned_cols = ['NED_AccX', 'NED_AccY', 'NED_AccZ']
        self.q_cols = ['ATT_qw', 'ATT_qx', 'ATT_qy', 'ATT_qz']
        self.ekf_in_cols = ['IMU_GyroX', 'IMU_GyroY', 'IMU_GyroZ','IMU_AccX', 'IMU_AccY', 'IMU_AccZ']
        self.ekf_out_cols = ['IMU_GyroX', 'IMU_GyroY', 'IMU_GyroZ','IMU_AngleAccX', 'IMU_AngleAccY', 'IMU_AngleAccZ', 'IMU_AccX', 'IMU_AccY', 'IMU_AccZ']
        self.angle_acc_cols = ['IMU_AngleAccX', 'IMU_AngleAccY', 'IMU_AngleAccZ']
        
        self.save_columns = ['TIME_StartTime', 'TIME_Sec','LAND_Landed','Fly_Count',
                        'ATT_Roll', 'ATT_Pitch', 'ATT_Yaw','ATT_RollRate', 'ATT_PitchRate', 'ATT_YawRate',
                        'IMU_AccX', 'IMU_AccY', 'IMU_AccZ', 'IMU_GyroX', 'IMU_GyroY','IMU_GyroZ', 
                        'IMU_MagX', 'IMU_MagY', 'IMU_MagZ', 'LPOS_X', 'LPOS_Y','LPOS_Z', 
                        'LPOS_Dist', 'LPOS_DistR', 'LPOS_VX', 'LPOS_VY', 'LPOS_VZ', 
                        'ATT_qw', 'ATT_qx', 'ATT_qy', 'ATT_qz','RC_C0','RC_C1','RC_C2','RC_C3','RC_C4',
                        'IMU_AngleAccX', 'IMU_AngleAccY', 'IMU_AngleAccZ','NED_AccX','NED_AccY','NED_AccZ',
                        'Avg_AccX', 'Avg_AccY', 'Avg_AccZ','Vehicle_VelX','Vehicle_VelY','Vehicle_VelZ']

    #数据滤波,加速度的滤波一定要放在最前面，不然会有不连贯的问题
    def filter_data(self):
        for fly in self.df.Fly_Count.unique():
            fly_index = self.df.loc[self.df.Fly_Count==fly].index
            init_index = fly_index[0]
            
            gyro = self.df.loc[init_index,self.gyro_cols]
            acc = self.df.loc[init_index,self.acc_cols]
            x = np.hstack([gyro, [0,0,0], acc])
            self.acc_ekf = ACC_EKF(x=x, freq=self.log_freq)
            for i in fly_index:
                raw_data = self.df.loc[i, self.ekf_in_cols].values
                raw_data = np.maximum(raw_data,np.array([-2, -3.5, -1.5, -5, -7, -20]))
                raw_data = np.minimum(raw_data,np.array([2, 3.5, 1, 7, 7, 0]))
                filter_data = self.acc_ekf.step(raw_data)
                self.df_pred.loc[i,self.ekf_out_cols] = filter_data
            
            self.acc_average_filter = Moving_Average_Filter(N=15)
            self.gyro_average_filter = Moving_Average_Filter(N=1)
            for i in fly_index:
                acc = self.df_pred.loc[i, self.acc_cols].values
                gyro = self.df_pred.loc[i, self.gyro_cols].values
                
#                acc = np.maximum(acc, np.array([-5, -7, -20]))
#                gyro = np.maximum(gyro, np.array([-2, -3.5, -1.5]))
#                acc = np.minimum(acc, np.array([7, 7, 0]))
#                gyro = np.minimum(gyro, np.array([2, 3.5, 1]))
                
                acc = self.acc_average_filter.apply(acc)
                gyro = self.gyro_average_filter.apply(gyro)
                self.df_pred.loc[i, self.acc_cols] = acc
                self.df_pred.loc[i, self.gyro_cols] = gyro
    
    #估计角度
    def estimate_angle(self):
        for fly in self.df.Fly_Count.unique():
            fly_index = self.df.loc[self.df.Fly_Count==fly].index
            init_index = fly_index[0]
            
            init_q = self.df.loc[init_index, self.q_cols]
            init_angle = self.df.loc[init_index, self.angle_cols]
            self.Imu = Imu(init_q, imu_freq=self.log_freq, Kp=self.Kp, Ki=self.Ki)
            self.Integral_Angle = Integral_Angle(init_a=init_angle)
            
            self.df_pred.loc[init_index,self.q_cols] = init_q
            self.df_pred.loc[init_index,self.angle_cols] = init_angle
            
            for t in fly_index[1:]:
                Gyro = np.array(self.df_pred.loc[t,self.gyro_cols])
                Acc = np.array(self.df_pred.loc[t,self.acc_cols])
                dt = (self.df_pred.loc[t,'TIME_StartTime'] - self.df_pred.loc[t-1,'TIME_StartTime']) / 1e6
#                (q,Angle) = self.Imu.update(Gyro, Acc)
                (q,Angle) = self.Integral_Angle.update(Gyro, dt)
                
                self.df_pred.loc[t,self.angle_cols] = Angle 
                self.df_pred.loc[t,self.q_cols] = q
        
    #估计角加速度
    def estimate_angle_acc(self):
        len_data = len(self.df)
        self.df_pred.loc[0, self.angle_acc_cols] = [0,0,0]
        for t in range(1,len_data):
            dt = 1 / self.log_freq
            Angle_rate_add = (self.df_pred.loc[t, self.gyro_cols] - self.df_pred.loc[t-1, self.gyro_cols]).values
            self.df_pred.loc[t, self.angle_acc_cols] = Angle_rate_add / dt
    
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
        
        return np.squeeze(np.array( np.dot(R, data.T) ))
    
    def estimate_pos(self):
        for fly in self.df.Fly_Count.unique():
            fly_index = self.df.loc[self.df.Fly_Count==fly].index
            init_index = fly_index[0]
            #获取初始值
            vel0 = self.df.loc[init_index, self.vel_cols].values
            pos0 = self.df.loc[init_index, self.pos_cols].values
            acc0_raw = self.df.loc[init_index, self.acc_cols].values
            q0 = self.df.loc[init_index, self.q_cols].values
            acc0 = self.frame_transform(acc0_raw,q0)
            acc0[2] += 9.8
                
            self.df_pred.loc[init_index, self.vel_cols] = vel0
            self.df_pred.loc[init_index, self.pos_cols] = pos0
            self.df_pred.loc[init_index, self.acc_ned_cols] = acc0
            self.df.loc[init_index, self.acc_ned_cols] = acc0 
            
            vel1 = vel0
            pos1 = pos0
            x = np.hstack([pos0,vel0,acc0])
            self.pos_ekf = POS_EKF(x=x,freq=self.log_freq)
            for t in fly_index[1:]:
                #计算地面坐标下加速度
                acc1_raw = self.df_pred.loc[t, self.acc_cols]
                q1 = self.df_pred.loc[t, self.q_cols]
                acc1 = self.frame_transform(acc1_raw,q1)
                acc1[2] += 9.8
                
                #积分时间
                dt = (self.df_pred.loc[t,'TIME_StartTime'] - self.df_pred.loc[t-1,'TIME_StartTime']) / 1e6
                #预估速度和位置
                pos1 += vel1 * dt + 0.5 * acc1 * dt**2
                vel1 += acc1*dt
                
                #卡尔曼方法
#                x = self.pos_ekf.step(acc1)
#                pos1 = x[0:3]
#                vel1 = x[3:6]
#                acc1 = x[6:9]
                
                self.df_pred.loc[t, self.vel_cols] = vel1
                self.df_pred.loc[t, self.pos_cols] = pos1
                self.df_pred.loc[t, self.acc_ned_cols] = acc1
                
                acc_df = self.df.loc[t, self.acc_cols]
                q_df = self.df.loc[t, self.q_cols]
                acc_df = self.frame_transform(acc_df, q_df)
                acc_df[2] += 9.8
                self.df.loc[t, self.acc_ned_cols] = acc_df
                
#                acc0 = acc1
                vel0 = vel1
                pos0 = pos1
                #记录下未滤波的对地加速度
                acc1_raw_df = self.df.loc[t, self.acc_cols]
                q1_df = self.df.loc[t, self.q_cols]
                acc1_df = self.frame_transform(acc1_raw_df,q1_df)
                acc1_df[2] += 9.8
                self.df.loc[t, self.acc_ned_cols] = acc1_df
    def acc_average_filter(self):
        for fly in self.df_pred.Fly_Count.unique():
            fly_index = self.df_pred.loc[self.df_pred.Fly_Count==fly].index
            self.acc_average_filter = Moving_Average_Filter()
            for i in fly_index:
                acc = self.df_pred.loc[i, self.acc_ned_cols]
                acc = self.acc_average_filter.apply(acc)
                self.df_pred.loc[i, self.acc_avg_cols] = acc
                
        for fly in self.df.Fly_Count.unique():
            fly_index = self.df.loc[self.df.Fly_Count==fly].index
            self.acc_average_filter = Moving_Average_Filter()
            for i in fly_index:
                acc = self.df.loc[i, self.acc_ned_cols]
                acc = self.acc_average_filter.apply(acc)
                self.df.loc[i, self.acc_avg_cols] = acc
    def estimate_vehicle_vel(self):
        for fly in self.df.Fly_Count.unique():
            fly_index = self.df.loc[self.df.Fly_Count==fly].index
            v0 = self.df.loc[fly_index[0],['LPOS_VX', 'LPOS_VY', 'LPOS_VZ']].values
            self.df_pred.loc[fly_index[0], self.veh_vel_cols] = v0
            for i in fly_index[1:]:
                acc = self.df_pred.loc[i, self.acc_cols].values
                acc[2] += 9.8
                dt = (self.df_pred.loc[i,'TIME_StartTime'] - self.df_pred.loc[i-1,'TIME_StartTime']) / 1e6
                v0 = v0 + dt * acc
                self.df_pred.loc[i, self.veh_vel_cols] = v0
    def process_data(self):
        temp = pd.DataFrame(columns=self.insert_cols)
        self.df_pred = pd.concat([self.df_pred,temp])
        self.df = pd.concat([self.df,temp])
        
        self.df = self.Data.process_data(self.df, test=self.test)
        self.df = self.df.loc[:,self.save_columns]
        self.df_pred = self.df_pred.loc[self.df.index, self.save_columns]
        same_cols = ['TIME_StartTime','TIME_Sec','LAND_Landed','Fly_Count','RC_C0','RC_C1','RC_C2','RC_C3','RC_C4']
        self.df_pred.loc[:,same_cols] = self.df.loc[:,same_cols]
        self.df.reset_index(inplace=True)
        self.df_pred.reset_index(inplace=True)
        
        
        if not os.path.exists('../cache/filter_df.csv'):
            self.filter_data()
            self.df_pred.to_csv('../cache/filter_df.csv',index=False)
        else:
            self.df_pred = pd.read_csv('../cache/filter_df.csv')
            
        self.estimate_angle()
#        self.df_pred.ATT_Yaw = self.df_pred.ATT_Yaw.apply(lambda x: x if x<= 0 else (x-3.14*2))
#        self.df_pred.ATT_Yaw += 2.5
#        self.df.ATT_Yaw = self.df.ATT_Yaw.apply(lambda x: x if x<= 0 else (x-3.14*2))
#        self.df.ATT_Yaw += 2.5
        
        self.estimate_angle_acc()
        self.estimate_pos()
        
        self.estimate_vehicle_vel()
        self.df.to_csv('../cache/df.csv',index=False)
        self.df_pred.to_csv('../cache/df_pred.csv',index=False)

def save_curve_data(df,Fly_count=1):
    index =  df.loc[df.Fly_Count==Fly_count].index
    pitch_df = df.loc[index,['ATT_Pitch','ATT_Roll','ATT_Yaw','RC_C0','RC_C1','RC_C2','RC_C3','LPOS_VX','LPOS_VY','LPOS_VZ','Vehicle_VelX','IMU_GyroY','IMU_AccX']]
    path = '../cache/curve_data/'+str(Fly_count)+'data.csv'
    pitch_df.to_csv(path)
if __name__ == '__main__':
    t0 = time.time()
    
    model_test = ModelTest(file='../raw_data/fly-3.2.csv', Kp=3, Ki=0.02, log_freq=89, test=False)
    model_test.process_data()
    show = data_visualiz.DataShow(df=model_test.df, df_pred=model_test.df_pred)
    t1 = time.time()
    print('time:',t1-t0)