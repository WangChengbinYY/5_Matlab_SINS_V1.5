clear; clc;

%  理论值设定
    att = [25,15,5]      % 度   zyx  航向 俯仰 横滚
    q = quaternion(att,'eulerd','ZYX','frame');
    Cbn = rotmat(q, 'frame');   %  这里的写法代表的是n到b的转动

%  设计三个 参考矢量
    r1 = [2,2,6]';        r1 = normalize(r1,'norm',2);
    r2 = [5,-3,2]';      r2 = normalize(r2,'norm',2);
    r3 = [-3,4,8]';      r3 = normalize(r3,'norm',2);
%  设计三个 载体测量矢量
    b1 = Cbn*r1;
    b2 = Cbn*r2;
    b3 = Cbn*r3;

% 计算B矩阵
    B = b1*r1' + b2*r2' + b3*r3';
    
% 注意：后面计算的A 代表的是Cbn (表示n到b)    

%--------------第一种方法  B的极分解  B=WH   A=W
%>>>>>>>>>>实验成功<<<<<<<<<<<
    % 对B进行极分解
        P = sqrtm(B'*B);
        U = B*inv(P);
        B_new = U*P;
        
        A = U; 
        % 计算 姿态矩阵
            tmp_quat = quaternion(A,'rotmat','frame');
            tmp_att = eulerd(tmp_quat, 'ZYX', 'frame')

%--------------第二种方法 SVD 分解法
%>>>>>>>>>>实验成功<<<<<<<<<<<
    % 对B进行奇异值分解
        [U,S,V] = svd(B);           % B=U*S*V'    U和V都是正交矩阵 S是对角矩阵
        A = U*V';
        % 计算 姿态矩阵
            tmp_quat = quaternion(A,'rotmat','frame');
            tmp_att = eulerd(tmp_quat, 'ZYX', 'frame')            
            
%--------------第三种方法 求取K的最大特征值及对应的特征矢量
%>>>>>>>>>>实验成功<<<<<<<<<<<            
       S = B + B';
       z = [B(2,3)-B(3,2);B(3,1)-B(1,3);B(1,2)-B(2,1)];
       K = [S-eye(3)*trace(B),z;z',trace(B)];
       % 对K进行特征值分解
            [V_K,D_K] = eig(K);    % K最大的特征值是 D_K对角线最后一个值
            tmp_V = V_K(:,4);       % 对应第4列特征矢量
            tmp_q = quaternion(tmp_V(4),tmp_V(1),tmp_V(2),tmp_V(3));  %  **这里要注意，四元数的放置顺序**
            tmp_q = normalize(tmp_q);
        % 计算 姿态矩阵
            tmp_att = eulerd(tmp_q, 'ZYX', 'frame')                     
            
            
       
       
       
       
       
            