import numpy as np
import pandas as pd
from sklearn.preprocessing import StandardScaler
from sklearn.mixture import GaussianMixture
import joblib
from sklearn.neural_network import MLPRegressor
from sklearn.cluster import KMeans

# 读取并处理Excel文件
ori_data1 = pd.read_excel(r'C:\Users\admin\Desktop\IML\capstone-2025\深度学习网络模型\11mT new1.xlsx', header=None).values
indice1 = np.arange(68, ori_data1.shape[1], 50)  # MATLAB索引69转换为Python索引68
data1 = ori_data1[:, indice1]

ori_data2 = pd.read_excel(r'C:\Users\admin\Desktop\IML\capstone-2025\深度学习网络模型\11mT new2.xlsx', header=None).values
indice2 = np.arange(68, ori_data2.shape[1], 50)
data2 = ori_data2[:, indice2]

ori_data3 = pd.read_excel(r'C:\Users\admin\Desktop\IML\capstone-2025\深度学习网络模型\11mT new3.xlsx', header=None).values
indice3 = np.arange(68, ori_data3.shape[1], 50)
data3 = ori_data3[:, indice3]

ori_data4 = pd.read_excel(r'C:\Users\admin\Desktop\IML\capstone-2025\深度学习网络模型\11mT new4.xlsx', header=None).values
indice4 = np.arange(69, ori_data4.shape[1], 50)  # MATLAB索引70转换为Python索引69
data4 = ori_data4[:, indice4]

ori_data5 = pd.read_excel(r'C:\Users\admin\Desktop\IML\capstone-2025\深度学习网络模型\11mT new5.xlsx', header=None).values
indice5 = np.arange(85, ori_data5.shape[1], 60)  # MATLAB索引86转换为Python索引85
data5 = ori_data5[:, indice5]

ori_data6 = pd.read_excel(r'C:\Users\admin\Desktop\IML\capstone-2025\深度学习网络模型\11mT new6.xlsx', header=None).values
indice6 = np.arange(85, ori_data6.shape[1], 60)
data6 = ori_data6[:, indice6]

ori_data7 = pd.read_excel(r'C:\Users\admin\Desktop\IML\capstone-2025\深度学习网络模型\11mT new7.xlsx', header=None).values
indice7 = np.arange(85, ori_data7.shape[1], 60)
data7 = ori_data7[:, indice7]

# 合并数据并删除首行
data = np.hstack([data1])
#data = np.hstack([data1, data2, data3, data4, data5, data6, data7])
data = data[1:, :]  # 删除第一行

# 提取各数据段
motion_data = data[7:9, :]           # 第8-9行（Python索引7-8）
magnet_data = data[[0, 1, 14], :]    # 第1、2、15行（Python索引0、1、14）
start_motion_data = data[5:7, :]     # 第6-7行（Python索引5-6）

# 计算坐标变换
now_point_data = motion_data - start_motion_data
now_point_data_without_last_col = now_point_data[:, :-1]

# 构建previous_point_data
zeros_col = np.array([[0], [0]])  # 初始零点列
previous_point_data = np.hstack((zeros_col, now_point_data_without_last_col))

# 构建训练数据
u_data = magnet_data[1:3, :]  # 取dr和pr
X_train = np.vstack([previous_point_data, u_data, now_point_data]).T

# % 标准化数据，并获取均值和标准差
scaler = StandardScaler()
X_normalized = scaler.fit_transform(X_train)
mu, sigma = scaler.mean_, scaler.scale_     #从标准化器中提取训练数据的均值 mu 和标准差 sigma。



# 训练GMM模型
gmm = GaussianMixture(n_components=30, covariance_type='full')
gmm.fit(X_normalized)

# 保存模型和参数
joblib.dump({'gmm': gmm, 'mu': mu, 'sigma': sigma}, 'trained_net_6inputs.pkl')

print('finish training!')

