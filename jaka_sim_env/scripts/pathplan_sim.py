#!/usr/bin/python3.8
# encoding: utf-8
import open3d as o3d
import copy
import numpy as np
import math
import rospy
from geometry_msgs.msg import PoseStamped
from pyquaternion import Quaternion
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from scipy.spatial.transform import Rotation as R
from nav_msgs.msg import Path

## 函数区
# 考虑x/y方向的路径
def XdirectionZigPath(xmin,xmax,ymin,ymax,Nx):
    paths = []
    dx = float(xmax-xmin)/(Nx-1)  # the y step-over
    flag=1      #奇偶分别
    path=[]
    for n in range(0,Nx):
        x = xmin+n*dx              # current y-coordinate 
        if flag==1:
            p1 = [x,ymin,0]   # start-point of line
            p2 = [x,ymax,0]   # end-point of line
            # print(flag)
        if flag==-2:
            p1 = [x,ymax,0]   # start-point of line
            p2 = [x,ymin,0]   # end-point of line
            # print(flag)
        path.append(p1)       # add the line to the path
        path.append(p2)
        flag=~flag
    return path

# 得到步进x/y方向的路径
def feedPath(path,step,step_num):
    flag=1      #奇偶分别
    fpath=[]
    n=len(path)
    for i in range(n//2):
        for j in range(step_num):
            if flag==1:
                p=[path[2*i][0],path[2*i][1]+j*step,path[2*i][2]]
            if flag==-2:
                p=[path[2*i][0],path[2*i][1]-j*step,path[2*i][2]]
            fpath.append(p)
        flag=~flag
    return fpath

# 依据点云坐标得到z方向的坐标
def ZdirectionPath(fpath,points,normals,scan_height,rapid_height):
    zpath=[]
    npath=[]
    n=len(fpath)
    # 初始点
    zpath.append([fpath[0][0],fpath[0][1],rapid_height])
    npath.append([0,0,-1])
    # 中间点
    for i in range(n):
        temp=[]
        for m in range(len(points)):
            temp.append(math.sqrt((fpath[i][0]-points[m][0])**2+(fpath[i][1]-points[m][1])**2))
        min_index=np.argmin(temp)
        # print(min_index)
        scan_z=points[min_index][2]+scan_height
        zpath.append([fpath[i][0],fpath[i][1],scan_z])
        npath.append([normals[min_index][0],normals[min_index][1],normals[min_index][2]])
    # 结束点
    zpath.append([fpath[n-1][0],fpath[n-1][1],rapid_height])
    npath.append([0,0,-1])
    return zpath,npath

class PointCloudSubscriber(object):
    def __init__(self):
        self.sub=rospy.Subscriber("/depth_pointcloud",PointCloud2,self.callBack)
        self.depth_pc = rospy.Publisher('/workpiece_pointcloud', PointCloud2, queue_size=1)
        self.pathplanning = rospy.Publisher('/pathplanning', Path, queue_size=10)
        self.published_path = 0
        

    def callBack(self,pcd_msg):
        if self.published_path == 1:
            return
        assert isinstance(pcd_msg, PointCloud2)
        pc = point_cloud2.read_points(pcd_msg, field_names=("x", "y", "z"))
        points = []
        point_range = [[-0.1, 0.01],[-0.01,0.15]]
        for p in pc:
            if p[0] > point_range[0][0] and p[0] < point_range[0][1] and p[1] > point_range[1][0] and p[1] < point_range[1][1]:
                if p[2] < 1.4:
                    points.append( [p[0],p[1],p[2]] )

        print("receive ",len(points),"  point")
        # origin
        mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
        # Create Open3D point cloud
        data = o3d.geometry.PointCloud()# 传入3d点云
        data.points = o3d.utility.Vector3dVector(points)
        # # o3d.visualization.draw_geometries([data,mesh_frame])

        # xyzmat=np.array(points)
        # points_z_max=np.max(xyzmat[:,2])
        # pcd = copy.deepcopy(data).translate((0, 0, -points_z_max))
        # plane_model, inliers = pcd.segment_plane(distance_threshold=0.22,
        #                                         ransac_n=100,
        #                                         num_iterations=1000)
        # outlier_cloud = pcd.select_by_index(inliers, invert=True)

        # pts=outlier_cloud.points
        # xyzmat=np.array(pts)
        # pts_z_max=np.max(xyzmat[:,2])
        # outlier_cloud = copy.deepcopy(outlier_cloud).translate((0, 0, -pts_z_max))

        # bounding_ploy = np.array([
        #                     [ -0.1, -0.01, 0 ],
        #                     [ 0.01, -0.01, 0 ],
        #                     [ 0.01, 0.15, 0 ],
        #                     [ -0.1, 0.15, 0 ]                            
        #                  ], dtype = np.float32).reshape([-1, 3]).astype("float64")
        # bounding_polygon = np.array(bounding_ploy, dtype = np.float64) 
        # vol = o3d.visualization.SelectionPolygonVolume() 
        # #The Z-axis is used to define the height of the selected region
        # vol.orthogonal_axis = "Z"
        # vol.axis_max = 8
        # vol.axis_min =0 
        # vol.bounding_polygon = o3d.utility.Vector3dVector(bounding_polygon)
        # comp = vol.crop_point_cloud(outlier_cloud)

        # 重新变换到相机坐标系
        pcd_in_camera = copy.deepcopy(data)#points_z_max+pts_z_max))
        # print(pcd_in_camera)
        xyz_load = np.asarray(pcd_in_camera.points)
        print("down sampled point cloud size is ", xyz_load.size)
        pts_in_camera=[]
        #点云坐标系相对于相机坐标系,点云x轴方向相当于相机的负y轴,点云y轴方向相当于相机的负z轴
        rotation_p_c=np.array([[0,0,1],
                               [-1,0,0],
                               [0,-1,0]])
        for i in range(len(xyz_load)):
            xyz_load[i,:]=np.dot(rotation_p_c,xyz_load[i,:])
            pts_in_camera.append(xyz_load[i,:])
        pcd_in_camera.points = o3d.utility.Vector3dVector(pts_in_camera)

        # 变换到机器人坐标系 
        pcd_in_link0 = o3d.geometry.PointCloud()# 传入3d点云
        pts_in_link0=[]
        rotation=np.array([[0,0,-1],
                           [0,1,0],
                           [1,0,0]])#绕y轴-90度，对应0.707 +0.000i +0.707j +0.000k
        for i in range(len(xyz_load)):
            pts_in_link0.append(np.dot(rotation.T,xyz_load[i,:])
                                        +np.array([
                                        0.0125, 
                                        0.2175, 
                                        1.5]))
        pcd_in_link0.points = o3d.utility.Vector3dVector(pts_in_link0)

        try:
            # 规划路径
            pcd_in_link0.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamKNN(knn=200)                        # 计算近邻的20个点
            )
            o3d.geometry.PointCloud.orient_normals_to_align_with_direction(pcd_in_link0, orientation_reference=np.array([0.0, 0.0, -1.0]))  #设定法向量在z轴方向上，全部z轴正方向一致
            normals = np.array(pcd_in_link0.normals)    # 法向量结果与点云维度一致(N, 3)
            points = np.array(pcd_in_link0.points)
            xmin=np.min(points[:,0])
            xmax=np.max(points[:,0])
            ymin=np.min(points[:,1])
            ymax=np.max(points[:,1])
            # 确定x方向分Nx条线
            Nx=10  # number of lines in the x-direction
            path = XdirectionZigPath(xmin,xmax,ymin,ymax,Nx)
            
            # 设置每条线走几步
            step_num=10
            step=(ymax-ymin)/(step_num-1)
            fpath=feedPath(path,step,step_num)

            scan_height=0.01
            rapid_height=0.2#0.15
            zpath,npath=ZdirectionPath(fpath,points,normals,scan_height,rapid_height)
            # print(zpath)

            # 将路径画在点云上,可视化
            #绘制顶点
            lines=[]
            for i in range(len(zpath)-1):
                l=[i,i+1]
                lines.append(l) #连接的顺序
            color = [[0, 0, 0.8] for i in range(len(lines))] 
            #添加顶点，点云
            points_pcd = o3d.geometry.PointCloud()# 传入3d点云
            points_pcd.points = o3d.utility.Vector3dVector(zpath)  # point_points 二维 numpy 矩阵,将其转换为 open3d 点云格式
            points_pcd.paint_uniform_color([0, 0.8, 0]) #点云颜色 
            points_pcd.normals= o3d.utility.Vector3dVector(npath)
            #绘制线条
            lines_pcd = o3d.geometry.LineSet()
            lines_pcd.lines = o3d.utility.Vector2iVector(lines)
            lines_pcd.colors = o3d.utility.Vector3dVector(color) #线条颜色
            lines_pcd.points = o3d.utility.Vector3dVector(zpath)
            # 可视化
            # o3d.visualization.draw_geometries([points_pcd,lines_pcd,pcd_in_link0,mesh_frame])
            # o3d.visualization.draw_geometries([points_pcd,lines_pcd,mesh_frame],point_show_normal=True)


            # publish
            header = Header()
            header.frame_id = "Link_0" #pcd_msg.header.frame_id #"d435_depth_frame"
            header.stamp = pcd_msg.header.stamp
            fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)
            ]
            pc = point_cloud2.create_cloud(header, fields, points_pcd.points)        
            self.depth_pc.publish(pc)

            # 发布话题
            rate=rospy.Rate(1)
            path_msg = Path()
            path_msg.header.frame_id="Link_0"
            path_msg.header.stamp=rospy.Time.now()#pcd_msg.header.stamp
            ps = PoseStamped()
            ps.header.frame_id="Link_0"
            ps.header.stamp=rospy.Time.now()#pcd_msg.header.stamp
            # print(zpath)
            for i in range(len(zpath)):
                # x/y/z坐标
                ps.pose.position.x=zpath[i][0]
                ps.pose.position.y=zpath[i][1]
                ps.pose.position.z=zpath[i][2]

                # 构建旋转矩阵
                rotate_matrix=[[0,0,0],[0,0,0],[0,0,0]]
                Z = np.array([npath[i][0], npath[i][1], npath[i][2]])
                # X = np.array([Z[1], -Z[0], 0])
                X = np.array([0, -Z[2], Z[1]])
                Z = Z/np.linalg.norm(Z)
                X = X/np.linalg.norm(X)
                rotate_matrix[0][2]=Z[0]
                rotate_matrix[1][2]=Z[1]
                rotate_matrix[2][2]=Z[2]
                rotate_matrix[0][0]=X[0]
                rotate_matrix[1][0]=X[1]
                rotate_matrix[2][0]=X[2]
                # 计算叉乘
                Y = np.cross(Z,X)
                Y = Y/np.linalg.norm(Y)
                rotate_matrix[0][1]=Y[0]
                rotate_matrix[1][1]=Y[1]
                rotate_matrix[2][1]=Y[2]
                rotateMatrix = np.array(rotate_matrix)
                # print(rotateMatrix)
                q = Quaternion(matrix=rotateMatrix)
                ps.pose.orientation.x=q.x
                ps.pose.orientation.y=q.y
                ps.pose.orientation.z=q.z
                ps.pose.orientation.w=q.w
                # print(ps)
                path_msg.poses.append(ps)
                ps = PoseStamped()
                ps.header.frame_id="Link_0"
                ps.header.stamp=rospy.Time.now()#pcd_msg.header.stamp
            # 发布话题
            if self.published_path == 0:
                self.pathplanning.publish(path_msg)
                rate.sleep()
                # rospy.loginfo("%s",str(path_msg))
                rospy.loginfo("Published {} waypoints.".format(len(path_msg.poses))) 
                self.published_path = 1 
        except:
            print("没有读取到点，即将进入下次回调！")


# 主函数
if __name__ == "__main__": 
    rospy.init_node("pathplan")
    PointCloudSubscriber()
    rospy.spin()