import zmq
import numpy as np
import pandas as pd
from scipy.stats import multivariate_normal
from scipy.linalg import block_diag
import joblib

#加代码，一个function()
def DGMM_prediction(next_state_x, next_state_y, current_state_x, current_state_y):

    def load_model(model_path):
        """加载训练好的DGMM模型及标准化参数"""
        try:
            model_data = joblib.load(model_path)
            # 提取 GMM、均值和标准差
            gmm = model_data['gmm']
            mu = model_data['mu']  # 直接使用保存的 mu
            sigma = model_data['sigma']  # 直接使用保存的 sigma
            # 返回与 Matlab 一致的结果
            return gmm, mu, sigma

        except FileNotFoundError:
            raise FileNotFoundError(f"模型文件 {model_path} 未找到")
        except KeyError as e:
            raise ValueError(f"模型文件缺少必要参数: {str(e)}")

    def preprocess_data(data_path):
        """预处理测试数据"""
        # 读取原始数据
        ori_data = pd.read_excel(data_path, header=None).values

        # 删除首行（与Matlab的data(1,:) = []对应）
        data = ori_data[1:, :]

        # 生成索引 (Matlab的(25+44)=69，Python从0开始索引68)
        start_idx = 68
        step = 50
        max_col = data.shape[1]

        # 处理索引越界
        if start_idx >= max_col:
            start_idx = max_col - 1
        indices = np.arange(start_idx, max_col, step)

        # 确保至少选择一个索引
        if len(indices) == 0:
            indices = [max_col - 1]

        return data[:, indices]

    # 加载测试数据
    # 配置参数
    MODEL_PATH = 'F:\\capstone2025\\small_myRIO_Helmholtz DGMM controller\\trained_net_6inputs.pkl'

    # 加载模型
    gmm, mu, sigma = load_model(MODEL_PATH)

    # 获取模型参数
    means = gmm.means_
    covariances = gmm.covariances_
    weights = gmm.weights_
    n_components = gmm.n_components

    current_state = np.array([current_state_x, current_state_y])
    #print('size of current_state:  ', current_state.shape)  # 使用 .shape，推荐这种方式因为它与NumPy兼容)
    current_normalized = (current_state - mu[:2]) / sigma[:2]
    print('current_normalized:  ', current_normalized)
    target_state = np.array([next_state_x, next_state_y])
    target_normalized = (target_state - mu[4:6]) / sigma[4:6]
    print('target_normalized:  ', target_normalized)

    # 存储中间结果
    prob_components = np.zeros(n_components)
    mu_cond_store = np.zeros((n_components, 2))

    for k in range(n_components):
        # 获取当前成分的参数
        mu_k = means[k]
        sigma_k = covariances[k]

        # 分割均值和协方差矩阵
        mu_S_t = mu_k[:2]
        mu_A_t = mu_k[2:4]
        mu_S_next = mu_k[4:6]

        # 协方差分块 (6x6矩阵)
        sigma_S_t = sigma_k[:2, :2]        # Cov(S_t, S_t)
        sigma_A_St = sigma_k[2:4, :2]       # Cov(A, S_t)
        sigma_Snext_St = sigma_k[4:6, 0:2]  # Cov(S_next, S_t)
        sigma_Snext_A = sigma_k[4:6, 2:4]  # Cov(S_next, A)
        sigma_A_Snext = sigma_k[2:4, 4:6]  # Cov(A_t, S_next)
        sigma_Snext = sigma_k[4:6, 4:6]    # Cov(S_next, S_next)

        # 构造协方差矩阵 Σ_{A,S} = [Cov(A, S_t), Cov(A, S_{t+1})]
        # 注意：A 是 2维，S_t 和 S_{t+1} 各是 2维 → Σ_{A,S} 是 2x4
        sigma_A_S = np.hstack([
            sigma_A_St,  # A 与 S_t 的协方差（2x2）
            sigma_A_Snext  # A 与 S_{t+1} 的协方差（2x2）
        ])

        # 构造协方差矩阵 Σ_{S,S} = Cov([S_t, S_{t+1}], [S_t, S_{t+1}])
        # 形状为 4x4：
        # [Cov(S_t,S_t)   Cov(S_t,S_{t+1})]
        # [Cov(S_{t+1},S_t) Cov(S_{t+1},S_{t+1})]
        sigma_S_S = np.block([
            [sigma_S_t, sigma_Snext_St.T],  # S_t 的协方差和 S_t与S_{t+1}的协方差
            [sigma_Snext_St, sigma_Snext]  # S_{t+1}与S_t的协方差和 S_{t+1}的协方差
        ])
        #print("The Σ_{S,S} is:   ", sigma_S_S)

        # 计算逆矩阵 Σ_{S,S}^{-1}
        sigma_S_S_inv = np.linalg.inv(sigma_S_S)
        #print("The inverse of Σ_{S,S} is:   ", sigma_S_S_inv)

        # 计算观测值与均值的差值 ΔS = [S_t - μ_S_t, S_{t+1} - μ_S_{t+1}]
        s_obs = np.concatenate([current_normalized, target_normalized])
        mu_S = np.concatenate([mu_S_t, mu_S_next])
        delta_S = s_obs - mu_S
        #print("The ΔS = [S_t - μ_S_t, S_{t+1} - μ_S_{t+1}] is:   ", sigma_S_S_inv)

        second_term_mu_cond = sigma_A_S @ sigma_S_S_inv @ delta_S
        #print("The second_term_mu_cond is:   ", second_term_mu_cond)
        mu_cond = mu_A_t + second_term_mu_cond
        print("The mu_cond is:   ", mu_cond)

        # 计算条件协方差（添加正则化防止奇异矩阵）
        reg = 1e-8 * np.eye(2)
        #term1 = sigma_A_S @ np.linalg.solve(sigma_S_t + reg, sigma_A_S.T)
        #term2 = sigma_Snext_A.T @ np.linalg.solve(sigma_Snext + reg, sigma_Snext_A)
        sigma_AA = sigma_k[2:4, 2:4]  # A_t 的自协方差（2x2）
        sigma_S_A = sigma_A_S.T       # Σ_{S,A} 是 Σ_{A,S} 的转置（4x2）

        sigma_cond = sigma_AA - sigma_A_S @ sigma_S_S_inv @ sigma_S_A

        # 计算联合概率 P(S_t, S_next | k) 观测数据的匹配程度（似然）
        joint_mu = np.concatenate([mu_S_t, mu_S_next])
        print("The joint_mu is:   ", joint_mu)
        joint_cov = block_diag(sigma_S_t, sigma_Snext)
        print("The joint_cov is:   ", joint_cov)
        prob_joint = multivariate_normal.pdf(
            np.concatenate([current_normalized, target_normalized]),
            mean=joint_mu,
            cov=joint_cov #+ 1e-12 * np.eye(4)  # 添加正则化
        )
        print("The prob_joint is:   ", prob_joint)

        # 存储结果
        # #后验概率是根据观测数据动态调整后的权重，反映了成分对观测数据的“解释能力”。
        prob_components[k] = weights[k] * prob_joint  # weights[k]是第 k 个高斯成分的先验权重；前验概率是成分的初始权重，反映模型对成分的“先验信心”。
        mu_cond_store[k] = mu_cond

    # 归一化权重
    prob_components /= np.sum(prob_components)
    print("The prob_components is:   ", prob_components)
    weighted_action = np.dot(prob_components,
                             mu_cond_store)  # dd_ref_normalized = sum(prob_component' .* mu_cond_store, 1);

    # 反标准化
    A_ref = np.zeros(2)
    A_ref[0] = weighted_action[0] * sigma[2] + mu[2]
    A_ref[1] = weighted_action[1] * sigma[3] + mu[3]
    predicted_output_dr = weighted_action[0] * sigma[2] + mu[2]
    predicted_output_pr = weighted_action[1] * sigma[3] + mu[3]
    print("The predicted_output_dr is:   ", predicted_output_dr)
    print("The predicted_output_pr is:   ", predicted_output_pr)

    return predicted_output_dr, predicted_output_pr


context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://*:0217")

while True:
    message = socket.recv_json()
    # print("Received message:", message)  # Debugging: Print the received message
    try:
        next_state_x, next_state_y, current_state_x, current_state_y = message['next_state_x'], message['next_state_y'], message['current_state_x'], message['current_state_y']

        predicted_output_dr, predicted_output_pr = DGMM_prediction(next_state_x, next_state_y, current_state_x, current_state_y) #调用函数

        output = {'predicted_output_dr': predicted_output_dr, 'predicted_output_pr': predicted_output_pr}

        socket.send_json(output)

    except KeyError as e:
        print(f"KeyError: {e}. Message received: {message}")
        socket.send_json({'error': str(e), 'message': message})

