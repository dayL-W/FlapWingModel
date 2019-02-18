# -*- coding: utf-8 -*-
"""
Created on Fri Jan  4 10:18:39 2019

@author: Liaowei
"""
import matplotlib.pyplot as plt

time_div = 89
#figsize(12.5, 4) # 设置 figsize
plt.rcParams['savefig.dpi'] = 100 #图片像素
plt.rcParams['figure.dpi'] = 100 #分辨率
# 默认的像素：[6.0,4.0]，分辨率为100，图片尺寸为 600&400
# 指定dpi=200，图片尺寸为 1200*800
# 指定dpi=300，图片尺寸为 1800*1200
# 设置figsize可以在不改变分辨率情况下改变比例

class DataShow(object):
    def __init__(self, df, df_pred):
        self.df = df
        self.df_pred = df_pred
        
    def axis(self, Fly_Count=1, axis='X'):
        col=['IMU_Acc'+axis,'LPOS_V'+axis, 'LPOS_'+axis]
        index = self.df.loc[self.df.Fly_Count==Fly_Count].index
        acc_z = self.df.loc[index, col[0]]
        vz = self.df.loc[index, col[1]]
        z = self.df.loc[index, col[2]]
        
        plt.figure(1, figsize=(6,12))
        plt.margins(0,0)

        plt.subplot(311)
        plt.title('axis '+axis)
        plt.ylabel('acc m/s2')
        plt.plot(index/time_div, acc_z, 'r--')
        
        plt.subplot(312)
        plt.ylabel('v m/s')
        plt.plot(index/time_div, vz, 'b--')
        
        plt.subplot(313)
        plt.xlabel('time s')
        plt.ylabel('pos m')
        plt.plot(index/time_div, z, 'g--')
        
        img_name = '../img/'+str(Fly_Count)+'_axis_'+ axis+'.jpg'
        plt.savefig(img_name,bbox_inches = 'tight')
        
    def pos(self, Fly_Count=1, axis='X'):
        col='LPOS_'+axis
        index = self.df.loc[self.df.Fly_Count==Fly_Count].index
        pos = self.df.loc[index, col]
        pos_pred = self.df_pred.loc[index, col]
        
        plt.figure(1, figsize=(6,4))
        plt.margins(0,0)
        
        plt.title('pos '+axis)
        plt.xlabel('time s')
        plt.ylabel(axis+' m')
        plt.plot(index/time_div, pos, 'r-')
        plt.plot(index/time_div, pos_pred, 'g-')
        
        img_name = '../img/'+str(Fly_Count)+'_pos_'+ axis+'.jpg'
        plt.savefig(img_name,bbox_inches = 'tight')
        
    def vel(self, Fly_Count=1, axis='X'):
        col='LPOS_V'+axis
        index = self.df.loc[self.df.Fly_Count==Fly_Count].index
        vel = self.df.loc[index, col]
        vel_pred = self.df_pred.loc[index, col]
        
        plt.figure(1, figsize=(6,4))
        plt.margins(0,0)
        
        plt.title('vel '+axis)
        plt.ylabel(axis+' m/s')
        plt.plot(index/time_div, vel, 'r--')
        plt.plot(index/time_div, vel_pred, 'g--')
        
        img_name = '../img/'+str(Fly_Count)+'_vel_'+ axis+'.jpg'
        plt.savefig(img_name,bbox_inches = 'tight')
 
    def acc(self,Fly_Count=1, axis='X'):
        index = self.df.loc[self.df.Fly_Count==Fly_Count].index
        col = 'IMU_Acc'+axis
        acc = self.df.loc[index, col]
        acc_pred =self.df_pred.loc[index, col]
        
        plt.figure(1, figsize=(6,4))
        plt.margins(0,0)
        
        plt.title('Acc '+axis)
        plt.xlabel('time s')
        plt.ylabel(axis+' m/s2')
        plt.plot(index/time_div, acc, 'r-')
        plt.plot(index/time_div, acc_pred, 'g-')
        
        img_name = '../img/'+str(Fly_Count)+'_acc_'+ axis+'.jpg'
        plt.savefig(img_name, bbox_inches = 'tight')
    def acc_avg(self,Fly_Count=1, axis='X'):
        index = self.df.loc[self.df.Fly_Count==Fly_Count].index
        
        col0 = 'NED_Acc'+axis
        col1 = 'Avg_Acc'+axis
        acc_raw = self.df_pred.loc[index, col0]
        acc_avg =self.df_pred.loc[index, col1]
        
        plt.figure(1, figsize=(6,4))
        plt.margins(0,0)
        
        plt.title('Acc '+axis)
        plt.xlabel('time s')
        plt.ylabel(axis+' m/s2')
        plt.plot(index/time_div, acc_raw, 'r-')
        plt.plot(index/time_div, acc_avg, 'g-')
        
        img_name = '../img/'+str(Fly_Count)+'_accavg_'+ axis+'.jpg'
        plt.savefig(img_name, bbox_inches = 'tight')
        
    def acc_ned(self,Fly_Count=1, axis='X'):
        index = self.df.loc[self.df.Fly_Count==Fly_Count].index
        col = 'NED_Acc'+axis
        acc = self.df.loc[index, col]
        acc_pred =self.df_pred.loc[index, col]
        
        plt.figure(1, figsize=(6,4))
        plt.margins(0,0)
        
        plt.title('NED Acc '+axis)
        plt.xlabel('time s')
        plt.ylabel(axis+' m/s2')
        plt.plot(index/time_div, acc, 'r-')
        plt.plot(index/time_div, acc_pred, 'g-')
        
        img_name = '../img/'+str(Fly_Count)+'_ned_acc_'+ axis+'.jpg'
        plt.savefig(img_name, bbox_inches = 'tight')
        
    def gyro(self,Fly_Count=1, axis='X'):
        index = self.df.loc[self.df.Fly_Count==Fly_Count].index
        col = 'IMU_Gyro'+axis
        gyro = self.df.loc[index, col]
        gyro_pred =self.df_pred.loc[index, col]
        
        plt.figure(1, figsize=(6,4))
        plt.margins(0,0)
        
        plt.title('Gyro filter')
        plt.ylabel(axis+' rad')
        plt.plot(index/time_div, gyro, 'r-')
        plt.plot(index/time_div, gyro_pred, 'g-')
        
        img_name = '../img/'+str(Fly_Count)+'_gyro_'+ axis+'.jpg'
        plt.savefig(img_name,bbox_inches = 'tight')
        
    def track_xy(self, Fly_Count=1):
        index = self.df.loc[self.df.Fly_Count==Fly_Count].index
        x = self.df.loc[index,'LPOS_X']
        y = self.df.loc[index,'LPOS_Y']
        
        x_pred = self.df_pred.loc[index,'LPOS_X']
        y_pred = self.df_pred.loc[index,'LPOS_Y']

        plt.figure(1, figsize=(6,4))
        plt.margins(0,0)
        
        plt.title('track')
        plt.xlabel('x pos m')
        plt.ylabel('y pos m')
        plt.plot(x, y, 'r--')
        plt.plot(x_pred, y_pred, 'g--')
        img_name = '../img/'+str(Fly_Count)+'_track_xy.jpg'
        plt.savefig(img_name,bbox_inches = 'tight')
        
#        plt.figure(1, figsize=(6,8))
#        plt.margins(0,0)
#        
#        plt.subplot(211)
#        plt.title('track')
##        plt.xlabel('x pos m')
#        plt.ylabel('y pos m')
#        plt.plot(x, y, 'r--')
#        
#        plt.subplot(212)
#        plt.xlabel('x pos m')
#        plt.ylabel('y pos m')
#        plt.plot(x_pred, y_pred, 'g--')
#        img_name = '../img/'+str(Fly_Count)+'_track_xy.jpg'
#        plt.savefig(img_name,bbox_inches = 'tight')
#        ax = plt.gca()
#        
#        ax.spines['right'].set_color('none') 
#        ax.spines['top'].set_color('none')
#        
#        ax.xaxis.set_ticks_position('bottom')   
#        ax.yaxis.set_ticks_position('left')
#        
#        ax.spines['bottom'].set_position(('data', 0))
#        ax.spines['left'].set_position(('data', 0))
        
    def angle(self, Fly_Count=1):
        index = self.df.loc[self.df.Fly_Count==Fly_Count].index
        Pitch = self.df.loc[index,'ATT_Pitch']
        Roll = self.df.loc[index,'ATT_Roll']
        Yaw = self.df.loc[index,'ATT_Yaw']
        Pitch_pre = self.df_pred.loc[index,'ATT_Pitch']
        Roll_pre = self.df_pred.loc[index,'ATT_Roll']
        Yaw_pre = self.df_pred.loc[index,'ATT_Yaw']
        
        plt.figure(1, figsize=(6,12))
        plt.margins(0,0)
        
        plt.subplot(311)
        plt.ylabel('roll rad')
        plt.plot(index / 100, Roll, 'r-')
        plt.plot(index / 100, Roll_pre, 'g-')
        
        plt.subplot(312)
        plt.title('att')
        plt.ylabel('pitch rad')
        plt.plot(index / 100, Pitch, 'r-')
        plt.plot(index / 100, Pitch_pre, 'g-')
        
        plt.subplot(313)
        plt.xlabel('time s')
        plt.ylabel('yaw rad')
        plt.plot(index / 100, Yaw, 'r-')
        plt.plot(index / 100, Yaw_pre, 'g-')
        img_name = '../img/'+str(Fly_Count)+'_angle.jpg'
        plt.savefig(img_name,bbox_inches = 'tight')
        
    def angle_error(self,Fly_Count=1):
        index = self.df.loc[self.df.Fly_Count==Fly_Count].index
        col = 'ATT_Roll'
        Roll = self.df.loc[index, col]
        Roll_pre =self.df_pred.loc[index, col]
        Roll_Error = (Roll_pre - Roll) 
        
        col = 'ATT_Pitch'
        Pitch = self.df.loc[index, col]
        Pitch_pre =self.df_pred.loc[index, col]
        Pitch_Error = (Pitch_pre - Pitch) 
        
        col = 'ATT_Yaw'
        Yaw = self.df.loc[index, col]
        Yaw_pre =self.df_pred.loc[index, col]
        Yaw_Error = (Yaw_pre - Yaw) 
        
        plt.figure(1, figsize=(6,12))
        plt.margins(0,0)
        
        plt.subplot(311)
        plt.title('att err')
        plt.title('Pitch err')
        plt.ylabel('Pitch rad')
        plt.plot(index/time_div, Pitch_Error, 'r-')
        
        plt.subplot(312)
        plt.ylabel('Roll rad')
        plt.plot(index/time_div, Roll_Error, 'r-')
        
        plt.subplot(313)
        plt.title('Yaw err')
        plt.xlabel('time s')
        plt.ylabel('Yaw rad')
        plt.plot(index/time_div, Yaw_Error, 'r-')
        img_name = '../img/'+str(Fly_Count)+'_angle_err.jpg'
        plt.savefig(img_name,bbox_inches = 'tight')
        
    def angle_acc(self,Fly_Count=1,axis='X'):
        index = self.df.loc[self.df.Fly_Count==Fly_Count].index
        col1 = 'IMU_Gyro' + axis
        col2 = 'IMU_AngleAcc' + axis
        gyro = self.df_pred.loc[index, col1]
        angle_acc =self.df_pred.loc[index, col2]
        
        plt.figure(1, figsize=(6,8))
        plt.margins(0,0)
        
        plt.subplot(211)
        plt.title('Angle acc')
        plt.ylabel(axis+' gyro rad/s')
        plt.plot(index/time_div, gyro, 'r-')
        
        plt.subplot(212)
        plt.xlabel('time s')
        plt.ylabel(axis+' angle_acc rad/s2')
        plt.plot(index/time_div, angle_acc, 'r-')
        img_name = '../img/'+str(Fly_Count)+'_angle_acc_'+ axis+'.jpg'
        plt.savefig(img_name,bbox_inches = 'tight')
    def throttle(self,Fly_Count=1):
        index = self.df.loc[self.df.Fly_Count==Fly_Count].index
        col = 'RC_C2'
        thro = self.df.loc[index, col]
        
        plt.figure(1, figsize=(6,4))
        plt.margins(0,0)
        
        plt.title('throttle')
        plt.xlabel('time s')
        plt.plot(index/time_div, thro, 'r-')

        img_name = '../img/'+str(Fly_Count)+'throttle'+'.jpg'
        plt.savefig(img_name,bbox_inches = 'tight')
        
    def elevator(self,Fly_Count=1):
        index = self.df.loc[self.df.Fly_Count==Fly_Count].index
        col = 'RC_C1'
        elev = self.df.loc[index, col]
        
        plt.figure(1, figsize=(6,4))
        plt.margins(0,0)
        
        plt.title('elevator')
        plt.xlabel('time s')
        plt.plot(index/time_div, elev, 'r-')

        img_name = '../img/'+str(Fly_Count)+'elevator'+'.jpg'
        plt.savefig(img_name,bbox_inches = 'tight')
        
    def rudder(self,Fly_Count=1):
        index = self.df.loc[self.df.Fly_Count==Fly_Count].index
        col = 'RC_C0'
        rudder = self.df.loc[index, col]
        
        plt.figure(1, figsize=(6,4))
        plt.margins(0,0)
        
        plt.title('rudder')
        plt.xlabel('time s')
        plt.plot(index/time_div, rudder, 'r-')

        img_name = '../img/'+str(Fly_Count)+'rudder'+'.jpg'
        plt.savefig(img_name,bbox_inches = 'tight')
    