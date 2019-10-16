import numpy as np
import cv2
import glob
import os
import pdb
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import seaborn as sns
sns.set_style("darkgrid")

def rs(x):
    return np.sqrt(x**2)


def camera_pointToRTmat(posture,corners):
    corners_all_array = np.array(corners)
    Normal_vector = []
    camera_refarence_point = []
    camera_point_array =np.array([])
    RT_mat = np.array([])

    for p_num in range(len(posture)):
        M = []
        m_num = []
        for in_num in posture:
            if p_num==in_num:
                continue
            Q = corners_all_array[posture[p_num]]-corners_all_array[in_num]
            M.append(np.dot(Q.T,Q))

        for e_num in range(len(M)):
            e_value , e_vector = np.linalg.eig(M[e_num])
            m_num.append(e_vector[np.argmin(e_value)])

        Normal_vector.append(np.cross(m_num[0],m_num[1])/np.linalg.norm(np.cross(m_num[0],m_num[1])))
        pdb.set_trace()
        if Normal_vector[p_num][2] > 0:
            Normal_vector[p_num] *= -1

        tj = np.dot(Normal_vector[p_num][np.newaxis,:],corners_all_array[p_num].T).T+CtoM_dist[p_num]
        camera_refarence_point.append((-2*(np.dot(Normal_vector[p_num][np.newaxis,:],corners_all_array[p_num].T).T+CtoM_dist[p_num]))*Normal_vector[p_num]+corners_all_array[p_num])

        if camera_point_array.shape[0] == 0:
            camera_point_array = np.append(camera_refarence_point[p_num],np.ones([camera_refarence_point[p_num].shape[0],1]),axis=1).T
        else :
            camera_point_array = np.append(camera_point_array,np.append(camera_refarence_point[p_num],np.ones([camera_refarence_point[p_num].shape[0],1]),axis=1).T,axis=1)

    return camera_refarence_point,camera_point_array,Normal_vector

cols = 9 #chessboardの横方向の交点の数
rows = 6 #chessboardの縦方向の交点の数
CtoM_dist = np.array([0.492,0.520,0.522,0.479,0.475])#カメラと平面鏡の距離を測ったもの
square_size = 0.048 #単位(m)

View_angle = 78

#-------------Mirrored object point-------------
#カメラ画像のパスリスト読み込み
img_path = open('../color_image_list/raw_image_list.txt','r')
img_list = img_path.read()
img_list = img_list.split('\n')[:len(img_list.split('\n'))-1]
img_path.close()

img = []

cam_rad = (View_angle/2)*3.14/180

#パスリストを用いて各画像読み込み
for path in img_list:
    img_buf = cv2.imread(path)
    img.append(img_buf)

#コーナー点をまとめる空のリストを作成
corners_all = []
corners_all_m = []
corners_flag = []

#コーナー点を検出できたもののコーナー点をリストに追加
for sample in img:
    corners_m = []
    gray = cv2.cvtColor(sample,cv2.COLOR_BGR2GRAY)
    #画像中心と視射角を用いてz方向のpxを求める
    center_h, center_w = np.array(gray.shape)/2
    center_z = np.sqrt(center_h**2 + center_w**2 )/np.tan(cam_rad)
    #チェスボードが見つかるかの判定
    c_flag, corners = cv2.findChessboardCorners(gray, (cols, rows))

    if c_flag:
        #整形
        corners = corners.reshape(corners.shape[0],corners.shape[2])
        #各コーナー点にzを関連付ける
        for i in range(corners.shape[0]):
            if i+cols < corners.shape[0]:
                px_to_m = square_size/rs(corners[i][1]-corners[i+cols][1])
            else:
                px_to_m = square_size/rs(corners[i][1]-corners[i-cols][1])

            corners_m.append(np.append(corners[i]-np.array([center_w,center_h]),center_z)*px_to_m)
            '''
            center_dis = np.sqrt(np.sqrt((center_h-y)**2)**2+np.sqrt((center_w-x)**2)**2)
            point_theta = np.arctan(center_dis / center_z)
            pdb.set_trace()
            '''
        corners_all_m.append(np.array(corners_m))
        corners_all.append(corners)
        corners_flag.append(True)
        print('corners found!!!!!!!')
    else:
        print('failed!!!!!!!!!!!!!!')
        corners_flag.append(False)

#コーナーを検出できなかった距離を省く
Effective_dist = CtoM_dist[corners_flag]

#-------------Reference object point-------------
#チェスボード画像読み込み、コーナー点検出
chess_img = cv2.imread("../hamming_color_code/chessboard_6_10.jpg")
chess_img_gray = cv2.cvtColor(chess_img,cv2.COLOR_BGR2GRAY)
f_flag, chess_corners = cv2.findChessboardCorners(chess_img_gray,(cols,rows))
if f_flag:
    print('chessboard corners found!!!!!!!')

chess_corners = chess_corners.reshape([chess_corners.shape[0],chess_corners.shape[2]])
chess_corners = np.append(chess_corners,np.zeros([chess_corners.shape[0],1]),axis=1)
chess_corners_m = []
for i in range(chess_corners.shape[0]):
    if i+cols < chess_corners.shape[0]:
        chess_px_to_m = square_size/rs(chess_corners[i][1]-chess_corners[i+cols][1])
    else:
        chess_px_to_m = square_size/rs(chess_corners[i][1]-chess_corners[i-cols][1])

    chess_corners_m.append(chess_px_to_m*chess_corners[i])
chess_board_array = np.append(np.array(chess_corners_m),np.ones([np.array(chess_corners_m).shape[0],1]),axis=1).T
#-------------ここから計算-------------

posture = [0,1,2]

#Mjj'
camera_refarence_point,camera_point_array,Nj = camera_pointToRTmat(posture,corners_all_m)
RT_mat = np.dot(camera_point_array,np.linalg.pinv(np.tile(chess_board_array,(1,3))))
#pdb.set_trace()
print('RT_mat:{}'.format(RT_mat))

'''
R = RT_mat[i][0:3,0:3]
T = RT_mat[i][0:3,3]
xPoint = np.array(chess_corners_m).T
'''
#pdb.set_trace()
fig = plt.figure()
ax = Axes3D(fig)

#軸にラベルを付けたいときは書く
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
#.plotで描画
#linestyle='None'にしないと初期値では線が引かれるが、3次元の散布図だと大抵ジャマになる
#markerは無難に丸
ax.plot(camera_refarence_point[0].T[0],camera_refarence_point[0].T[1],camera_refarence_point[0].T[2],marker="o",linestyle='None',color='red')
ax.plot(corners_all_m[0].T[0],corners_all_m[0].T[1],corners_all_m[0].T[2],marker="o",linestyle='None',color='blue')
ax.plot([0],[0],[0],marker="o",linestyle='None',color='green')
ax.plot([Nj[0][0]],[Nj[0][1]],[Nj[0][2]],marker="o",linestyle='None',color='cyan')

#最後に.show()を書いてグラフ表示
plt.show()

#pdb.set_trace()


'''
Q0 = (corners_all_array[1]-corners_all_array[0])
Q1 = (corners_all_array[1]-corners_all_array[2])

M0 = np.dot(Q0.T,Q0)
M1 = np.dot(Q1.T,Q1)

M0_w, M0_v=np.linalg.eig(M0)
M1_w, M1_v=np.linalg.eig(M1)

m0 = M0_v[np.argmin(M0_w)]
m1 = M1_v[np.argmin(M1_w)]

nj = np.cross(m0,m1)/np.linalg.norm(np.cross(m0,m1))
'''
