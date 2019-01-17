function [ x_aposteriori,P_aposteriori,RotMatrix,roll,pitch,yaw] = AttltitudeEKF( updateVect,dt,z ,q,r,x_aposteriori_k,P_aposteriori_k)
%AttltitudeEKF 状态估计的拓展卡尔曼滤波方法
%输入：
%   updateVect：指示哪些数据进行了更新
%   dt:更新周期
%   z:测量值
%   q:系统噪声，r:测量噪声
%   H_k:测量矩阵
%   x_aposteriori_k：上一时刻的状态估计
%   P_aposteriori_k：上一时刻估计协方差
%输出：
%   x_aposteriori：当前时刻的状态估计
%   P_aposteriori：当前时刻的估计协方差
%   RotMatrix:旋转矩阵
%   roll、pitch、yaw：欧拉角

%   O=[ 0,wz, -wy;
%       -wz, 0 ,wx;
%       wy, -wx, 0];
    O=[0,x_aposteriori_k(3),-x_aposteriori_k(2);
      -x_aposteriori_k(3),0,x_aposteriori_k(1);
       x_aposteriori_k(2),-x_aposteriori_k(1),0];
    bCn = eye(3,3) +O*dt;%旋转矩阵
%更新先验状态矩阵
  x_apriori(1:3) = x_aposteriori_k(1:3) + x_aposteriori_k(4:6)*dt;%角速度
  x_apriori(4:6) = x_aposteriori_k(4:6);%角加速度
  x_apriori(7:9) = bCn*x_aposteriori_k(7:9);%加速度
  x_apriori(10:12) = bCn*x_aposteriori_k(10:12);%磁场
  %更新状态转移矩阵
%   r_a=[ 0,  -az, ay;    r_m=[ 0,  -mz, my;
%         az,  0,  -ax;         mz,  0,  -mx;
%        -ay, ax,  0];         -my,  mx,  0];
  r_a = [0,-x_aposteriori_k(9),x_aposteriori_k(8);
         x_aposteriori_k(9),0,-x_aposteriori_k(7);
         -x_aposteriori_k(8),x_aposteriori_k(7),0];
  r_m = [0, -x_aposteriori_k(12),x_aposteriori_k(11);
         x_aposteriori_k(12),0,-x_aposteriori_k(10);
         -x_aposteriori_k(11),x_aposteriori_k(10),0];
 A_lin = [eye(3,3),eye(3,3)*dt,zeros(3,3),zeros(3,3);
          zeros(3,3),eye(3,3),zeros(3,3),zeros(3,3);
          r_a*dt,zeros(3,3),(eye(3,3)+O*dt),zeros(3,3);
          r_m*dt,zeros(3,3),zeros(3,3),(eye(3,3)+O*dt)];
  %预测误差协方差矩阵
  Q=[eye(3,3)*q(1),zeros(3,3),zeros(3,3),zeros(3,3);
     zeros(3,3),eye(3,3)*q(2),zeros(3,3),zeros(3,3);
     zeros(3,3),zeros(3,3),eye(3,3)*q(3),zeros(3,3);
     zeros(3,3),zeros(3,3),zeros(3,3),eye(3,3)*q(4)];
 P_apriori = A_lin*P_aposteriori_k*A_lin'+A_lin*Q*A_lin';

 if((updateVect(1)~=0) && (updateVect(2)~=0) && updateVect(3)~=0)
 %卡尔曼增益
 R=[eye(3,3)*r(1),zeros(3,3),zeros(3,3);
     zeros(3,3),eye(3,3)*r(2),zeros(3,3);
     zeros(3,3),zeros(3,3),eye(3,3)*r(3)];
 H_k=[eye(3,3),zeros(3,3),zeros(3,3),zeros(3,3);
      zeros(3,3),zeros(3,3),eye(3,3),zeros(3,3);
      zeros(3,3),zeros(3,3),zeros(3,3),eye(3,3)];
  K_k=(P_apriori*H_k')/(H_k*P_apriori*H_k'+R);
  %状态估计矩阵
  x_aposteriori=x_apriori'+K_k*(z - H_k*x_apriori');
  %估计误差协方差
 % P_aposteriori=(eye(12,12)-K_k*H_k)*P_apriori;%计算方式简单但是容易失去正定性，PX4中使用这种方式
  P_aposteriori=(eye(12,12)-K_k*H_k)*P_apriori*(eye(12,12)-K_k*H_k)'+K_k*R*K_k';

elseif((updateVect(1)~=0) && (updateVect(2)==0) && updateVect(3)==0)
   R=eye(3,3)*r(1);
   H_k=[eye(3,3),zeros(3,3),zeros(3,3),zeros(3,3)];
   K_k=(P_apriori*H_k')/(H_k*P_apriori*H_k'+R);
   x_aposteriori=x_apriori'+K_k*(z(1:3) - H_k*x_apriori');
   P_aposteriori=(eye(12,12)-K_k*H_k)*P_apriori*(eye(12,12)-K_k*H_k)'+K_k*R*K_k';

 elseif((updateVect(1)~=0) && (updateVect(2)~=0) && updateVect(3)==0) 
     R=[eye(3,3)*r(1),zeros(3,3);
        zeros(3,3),eye(3,3)*r(2)];
H_k=[eye(3,3),zeros(3,3),zeros(3,3),zeros(3,3);
    zeros(3,3),zeros(3,3),eye(3,3),zeros(3,3)];
    K_k=(P_apriori*H_k')/(H_k*P_apriori*H_k'+R);
   x_aposteriori=x_apriori'+K_k*(z(1:6) - H_k*x_apriori');
   P_aposteriori=(eye(12,12)-K_k*H_k)*P_apriori*(eye(12,12)-K_k*H_k)'+K_k*R*K_k';
   
  elseif((updateVect(1)~=0) && (updateVect(2)==0) && updateVect(3)~=0) 
     R=[eye(3,3)*r(1),zeros(3,3);
         zeros(3,3),eye(3,3)*r(3)];
    H_k=[eye(3,3),zeros(3,3),zeros(3,3),zeros(3,3);
    zeros(3,3),zeros(3,3),zeros(3,3),eye(3,3)];
      K_k=(P_apriori*H_k')/(H_k*P_apriori*H_k'+R);
   x_aposteriori=x_apriori'+K_k*([z(1:3);z(7:9)] - H_k*x_apriori');
   P_aposteriori=(eye(12,12)-K_k*H_k*P_apriori)*(eye(12,12)-K_k*H_k)'+K_k*R*K_k';
 end

  %得到旋转矩阵和角度
  k = -x_aposteriori(7:9) /norm(x_aposteriori(7:9),2);
  i = x_aposteriori(10:12) /norm(x_aposteriori(10:12),2);
  j=cross(k,i);
  j=j/norm(j,2);
  RotMatrix=[i';j';k'];
  
  roll=atan2(RotMatrix(3,2),RotMatrix(3,3))*57.3;
  pitch=-asin(RotMatrix(3,1))*57.3;
  yaw=atan2(RotMatrix(2,1),RotMatrix(1,1))*57.3;
end
--------------------- 
作者：wsh_ 
来源：CSDN 
原文：https://blog.csdn.net/hxudhdjuf/article/details/79594866 
版权声明：本文为博主原创文章，转载请附上博文链接！