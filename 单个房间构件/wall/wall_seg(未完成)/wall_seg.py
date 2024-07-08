#LTPD_DBSCAN
import numpy as np
from sklearn.neighbors import NearestNeighbors
from sklearn.cluster import DBSCAN
import open3d as o3d
def dataOpen(path):
    '''
    :param path:读取文件路径
    :return: 输出点云
    '''

    if path.endswith('xyz') or path.endswith('txt'):
        if path.endswith('txt'):
            data = np.loadtxt(path)
        else:
            data = np.genfromtxt(path)
    elif path.endswith('npy'):
        data = np.load(path)
        _, M = np.shape(data)
        if M == 6:
            data = data[:, :3]
    elif path.endswith('ply'):
        pcd = o3d.io.read_point_cloud(path, format='ply')
        data = np.asarray(pcd.points)
    elif path.endswith('pcd'):
        pcd = o3d.io.read_point_cloud(path, format='pcd')
        data = np.asarray(pcd.points)
    return data[:,:3]

def vis(data):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(data[:,:3])

    pcd.paint_uniform_color([1, 0, 0])
    o3d.visualization.draw_geometries([pcd], window_name="ctz",
                                      width=1024, height=768,
                                      left=50, top=50,
                                      mesh_show_back_face=False)

def power_iteration(A, num_iterations=1000):
    n = len(A)
    # 初始化随机向量
    v = np.random.rand(n)

    for _ in range(num_iterations):
        # 进行矩阵-向量乘法
        Av = np.dot(A, v)

        # 计算特征值估计
        eigenvalue = np.linalg.norm(Av)

        # 归一化向量
        v = Av / eigenvalue

    # 返回特征值和特征向量
    return eigenvalue, v

def compute_k_largest_eig(A, k=3, num_simulations=100):
    # 初始化空列表
    eigenvalues = []
    eigenvectors = []

    for i in range(k):

        # 使用幂迭代法求解当前矩阵的最大特征值和对应特征向量
        eigenvalue, eigenvector = power_iteration(A, num_simulations)

        # 将当前特征值和特征向量保存到列表中
        eigenvalues.append(eigenvalue)
        eigenvectors.append(eigenvector)
        A = A - eigenvalue * np.dot(eigenvector, eigenvector.T)
    return eigenvalues, (np.array(eigenvectors)).T

def normal_compute(point_cloud):
    k_neighbors = 30
    # 构建最近邻模型
    nn_model = NearestNeighbors(n_neighbors=k_neighbors)
    nn_model.fit(point_cloud)
    # 查询最近邻点
    distances, indices = nn_model.kneighbors(point_cloud)
    # 计算每个点的法向量
    normals = np.zeros_like(point_cloud)
    for i in range(len(point_cloud)):
        neighbor_points = point_cloud[indices[i]]
        # 使用最小二乘法拟合平面，得到法向量
        A = np.column_stack([neighbor_points[:, 0], neighbor_points[:, 1], np.ones_like(neighbor_points[:, 0])])
        b = neighbor_points[:, 2]
        normal, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        normal /= np.linalg.norm(normal)  # 归一化法向量
        normals[i] = normal
    return normals
def normal_compute_open3d(data):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(data[:,:3])
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=25)
    )
    return np.array(pcd.normals)
def ltpd(data):
    #1. 计算点云的normal
    normal = normal_compute_open3d(data)
    #2. 计算平面方程与ltpd方程
    N, M = np.shape(data)
    ltpd = np.zeros([N,N])
    for i in range(N):
        nor = normal[i, :]
        dat = data[i, :]
        d = -1 * sum(nor * dat)
        fenmu = np.sqrt(sum(nor**2))
        ltpd[i,:] = (abs(nor[0]*data[:, 0] + nor[1]*data[:, 1] + nor[2]*data[:, 2] + d) / fenmu).T
    for i in range(N):
        for j in range(N):  # 仅迭代上三角部分
            if ltpd[i, j] != ltpd[j, i]:
                ltpd[i, j] = max(ltpd[i, j], ltpd[j, i])
                ltpd[j, i] = ltpd[i, j]
    #3. 重构点云数据
    A = -0.5*ltpd
    B = np.zeros([N,N])
    A_li = np.average(A, axis=0)#列均值
    A_hang = np.average(A, axis=1)#行均值
    A_total = np.average(A)#列均值
    for i in range(N):
        for j in range(N):  # 仅迭代上三角部分
            B[i, j] = A[i, j] - A_hang[i] - A_li[j] + A_total
    #得到矩阵B的特征值与特征向量
    eigenvalues, eigenvectors = np.linalg.eigh(B)
    X = np.zeros([N,3])
    for i in range(3):
        X[:, i] = np.sqrt(eigenvalues[N-1-i]) * eigenvectors[:, N-1-i]
    return X

def getOptMatrix(X, k):
    neigh = NearestNeighbors(n_neighbors=k + 1, algorithm="ball_tree",).fit(X)
    N, _ = np.shape(X)
    distance = []
    destiny = []
    weight = []
    index = []
    res = []
    # 通过邻接矩阵，计算单独点的特征
    for i in range(N):
        dis_temp, index_temp = neigh.kneighbors([X[i,:]])
        distance.append(np.squeeze(dis_temp))
        weight.append(np.exp(-np.squeeze(dis_temp)))
        destiny.append(np.exp(-np.sum(np.squeeze(dis_temp)/k)))
        index.append(np.squeeze(index_temp))
    for i in range(N):
        a = weight[i]
        b = np.asarray(destiny)[index[i]]
        c = destiny[i]
        res.append(c+sum(a*b))
    return res, distance

def Adaptive_DBSCAN(points):
    N, _ = np.shape(points)
    k = 4
    LDen, dis = getOptMatrix(points, k)

    indexed_list = list(enumerate(LDen))
    #找寻跳跃点
    # 对索引化列表进行排序，按照元素值进行降序排序
    # 注意：这里的x[1]表示我们根据元素值排序，x[0]是原始索引
    sorted_indexed_list = sorted(indexed_list, key=lambda x: x[1], reverse=True)
    # 如果你只需要排序后的值
    sorted_values = [element for index, element in sorted_indexed_list]
    sorted_indices = [index for index, element in sorted_indexed_list]

    Trend_max = 0
    for i in range(N):
        if i == 0:
            Trend = ((sorted_values[i] - sorted_values[i+1])/sorted_values[i])
        elif i == N-1:
            Trend = (sorted_values[i]/(sorted_values[i-1] - sorted_values[i]))
        else:
            Trend = ((sorted_values[i]-sorted_values[i+1])/(sorted_values[i-1] - sorted_values[i]))
        if Trend > Trend_max:
            C = i
            C_Dlen = sorted_values[i]
            Trend_max = Trend
    Eps = max(dis[sorted_indices[C]])
    MinPts = k
    #分层
    dense_layer = points[sorted_indices[:C]]
    sparse_layer = points[sorted_indices[C:]]
    db = DBSCAN(eps=Eps, min_samples=MinPts).fit(dense_layer)
    labels = db.labels_
    #为稀疏层找label
    sparse_dense = NearestNeighbors(n_neighbors=2).fit(dense_layer)
    for i in range(N - C):
        dis_temp, index_temp = sparse_dense.kneighbors([sparse_layer[i,:]])
        print(1)
# 示例使用
# D:\ctz\code\PC_D\src\MeanShift_py-master\MeanShift_py-master\wall - Cloud.txt   无效
# D:\ctz\code\PC_D\data\cubic15.txt 有效
# D:\ctz\code\PC_D\data\s3disfull\raw\Area_1_hallway_1.npy 有效
# D:\ctz\code\PC_D\data\s3disfull\raw\Area_1_office_21.npy
# D:\ctz\code\PC_D\src\LTPD-DBSCAN\LTPD-DBSCAN\filename.txt
path = r'/PC_I/LTPD-DBSCAN/LTPD-DBSCAN/filename.txt'
data = dataOpen(path)
recons_data = ltpd(data)
Adaptive_DBSCAN(recons_data)

# np.savetxt("cubic15.txt", recons_data)