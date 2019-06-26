import open3d
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pdb



if __name__ == '__main__':
    pointcloud = open3d.read_point_cloud("reconstructed.ply")
    open3d.draw_geometries([pointcloud])
    pcloud = np.asarray([pointcloud.points])
    fig = plt.figure()
    ax = Axes3D(fig)
    #pdb.set_trace()
    x,y,z = pcloud[:,:,0].reshape(pcloud.shape[1]),pcloud[:,:,1].reshape(pcloud.shape[1]),pcloud[:,:,2].reshape(pcloud.shape[1])

    ax.set_xlim(min(x),max(x))
    ax.set_ylim(min(y),max(y))
    ax.set_zlim(min(z),max(z))

    ax.plot(x,y,z,'o',color="g")
    plt.show()
