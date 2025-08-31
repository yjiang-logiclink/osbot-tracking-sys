import sys
import vtk
from PyQt5 import QtWidgets
from PyQt5.QtCore import Qt
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
import math
import multiprocessing
import time
import threading
import struct
import binascii
import socket
import math
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R

from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot
import open3d as o3d

def optimize_rotation_angle(P, stl_model_path, num_restarts=20):
    """
    优化旋转角度，使输入点云与 STL 模型点云最佳对齐。

    :param P: ndarray, 输入点云坐标 (N, 3)
    :param stl_model_path: str, STL 模型文件路径
    :param num_restarts: int, 随机重启次数，默认10次
    :return: ndarray, 优化后的旋转角度 (以度为单位，形状为 (3,))
    """
    # 加载 STL 模型
    stl_mesh = o3d.io.read_triangle_mesh(stl_model_path)
    stl_point_cloud = stl_mesh.sample_points_uniformly(number_of_points=10000)
    stl_points = np.asarray(stl_point_cloud.points)

    # 初始化已知点的对应点 Q
    Q = np.zeros_like(P)
    Q[0] = [0, 0, 0]

    # Step 1: 计算平移向量
    translation = Q[0] - P[0]

    # 平移后的点云
    P_translated = P + translation

    # 定义旋转范围
    rotate_range = np.pi / 2

    # Step 2: 定义目标函数（计算每个点的距离）
    def residuals(rotation_vector):
        """
        目标函数：计算旋转后的点到 STL 表面的最小距离平方和。
        :param rotation_vector: 旋转向量 (3,)
        :return: 残差列表
        """
        R = Rot.from_rotvec(rotation_vector).as_matrix()

        distances = []
        for i in range(1, len(P)):
            # 将点云点绕 Q[0] 旋转
            transformed_point = R @ (P_translated[i] - Q[0]) + Q[0]
            # 计算点到 STL 表面的最近距离
            diff = stl_points - transformed_point
            distance = np.min(np.linalg.norm(diff, axis=1))
            distances.append(distance)

        return distances

    # Step 3: 定义随机重启优化函数
    def optimize_with_random_restarts():
        best_result = None
        best_error = float('inf')

        for restart in range(num_restarts):
            # 随机初始化旋转向量
            rotation_vector_init = (np.random.rand(3) - 0.5) * 2 * (rotate_range / 2)

            # 使用 least_squares 进行优化
            result = least_squares(
                residuals,
                rotation_vector_init,
                bounds=([-rotate_range] * 3, [rotate_range] * 3),
                method='trf',
                ftol=1e-8,
                xtol=1e-8,
                gtol=1e-8,
                verbose=0
            )

            # 计算总误差平方和
            total_error = np.sum(result.fun**2)

            # 如果当前解更优，更新最佳结果
            if total_error < best_error:
                best_error = total_error
                best_result = result

        print(best_error)
        return best_result

    # Step 4: 使用随机重启法优化
    best_result = optimize_with_random_restarts()

    # 提取优化结果
    rotation_vector_optimized = best_result.x

    # 将旋转向量转换为角度
    rotation_angle_deg = np.degrees(rotation_vector_optimized)

    return rotation_angle_deg



def rotate_and_translate(coord, rotation, translation):
    """
    计算点的旋转和平移后的新坐标

    参数:
    coord: tuple (x, y, z) - 原始点坐标
    rotation: tuple (rx, ry, rz) - 绕 x, y, z 轴的旋转角度 (单位: 度)
    translation: tuple (tx, ty, tz) - 平移向量

    返回:
    numpy 数组，旋转和平移后的新坐标
    """
    # 提取参数
    x, y, z = coord
    rx, ry, rz = rotation
    tx, ty, tz = translation

    # 将角度转换为弧度
    rx_rad = np.radians(rx)
    ry_rad = np.radians(ry)
    rz_rad = np.radians(rz)

    # 定义绕 x 轴的旋转矩阵
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(rx_rad), -np.sin(rx_rad)],
        [0, np.sin(rx_rad), np.cos(rx_rad)]
    ])

    # 定义绕 y 轴的旋转矩阵
    R_y = np.array([
        [np.cos(ry_rad), 0, np.sin(ry_rad)],
        [0, 1, 0],
        [-np.sin(ry_rad), 0, np.cos(ry_rad)]
    ])

    # 定义绕 z 轴的旋转矩阵
    R_z = np.array([
        [np.cos(rz_rad), -np.sin(rz_rad), 0],
        [np.sin(rz_rad), np.cos(rz_rad), 0],
        [0, 0, 1]
    ])

    # 组合旋转矩阵
    R = R_z @ R_y @ R_x

    # 原始点的坐标向量
    b = np.array([x, y, z])

    # 旋转后的坐标
    b_rotated = R @ b

    # 平移
    b_translated = b_rotated + np.array([tx, ty, tz])

    return b_translated

def rotate_and_translate(coord, rotation, translation):
    """
    计算点的旋转和平移后的新坐标

    参数:
    coord: tuple (x, y, z) - 原始点坐标
    rotation: tuple (rx, ry, rz) - 绕 x, y, z 轴的旋转角度 (单位: 度)
    translation: tuple (tx, ty, tz) - 平移向量

    返回:
    numpy 数组，旋转和平移后的新坐标
    """
    # 转换角度为弧度
    rx, ry, rz = np.radians(rotation)

    # 计算旋转矩阵
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(rx), -np.sin(rx)],
        [0, np.sin(rx), np.cos(rx)]
    ])

    Ry = np.array([
        [np.cos(ry), 0, np.sin(ry)],
        [0, 1, 0],
        [-np.sin(ry), 0, np.cos(ry)]
    ])

    Rz = np.array([
        [np.cos(rz), -np.sin(rz), 0],
        [np.sin(rz), np.cos(rz), 0],
        [0, 0, 1]
    ])

    # 综合旋转矩阵
    R = Rz @ Ry @ Rx

    # 应用旋转和平移
    new_coord = R @ np.array(coord) + np.array(translation)
    return new_coord

def rotate_and_translate(coord, rotation, translation):
    """
    计算点的旋转和平移后的新坐标

    参数:
    coord: tuple (x, y, z) - 原始点坐标
    rotation: tuple (rx, ry, rz) - 绕 x, y, z 轴的旋转角度 (单位: 度)
    translation: tuple (tx, ty, tz) - 平移向量

    返回:
    numpy 数组，旋转和平移后的新坐标
    """
    # 转换角度为弧度
    rx, ry, rz = np.radians(rotation)

    # 计算旋转矩阵
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(rx), -np.sin(rx)],
        [0, np.sin(rx), np.cos(rx)]
    ])

    Ry = np.array([
        [np.cos(ry), 0, np.sin(ry)],
        [0, 1, 0],
        [-np.sin(ry), 0, np.cos(ry)]
    ])

    Rz = np.array([
        [np.cos(rz), -np.sin(rz), 0],
        [np.sin(rz), np.cos(rz), 0],
        [0, 0, 1]
    ])

    # 综合旋转矩阵
    R = Rz @ Ry @ Rx

    # 应用旋转和平移
    new_coord = R @ np.array(coord) + np.array(translation)
    return new_coord

def find_fixed_point_lsq(test_6dof, initial_guess, range_sizes):
    """
    使用最小二乘法计算一个点，使得经过一组旋转和平移后尽可能接近固定点。

    参数:
    test_6dof: list of lists [[translation, rotation], ...] - 每个点的平移和旋转集合
    initial_guess: tuple (x, y, z) - 固定点的初始猜测值
    range_sizes: tuple (rx_range, ry_range, rz_range) - 每个维度的搜索范围大小

    返回:
    优化后尽可能接近固定点的坐标
    """
    bounds = (
        np.array(initial_guess) - np.array(range_sizes),
        np.array(initial_guess) + np.array(range_sizes)
    )

    def residuals(fixed_point):
        # 计算每个点变换后的坐标与固定点的距离
        errors = []
        for translation, rotation in test_6dof:
            transformed = rotate_and_translate(fixed_point, rotation, translation)
            errors.append(np.linalg.norm(transformed - fixed_point))
        return errors

    # 使用最小二乘法优化
    result = least_squares(residuals, initial_guess, bounds=bounds)

    if result.success:
        return result.x
    else:
        raise ValueError("优化失败: " + result.message)



def compute_new_b_6dof(self, smoothed_positions, smoothed_rotations):
    """
    根据刚性连接计算b的新6DOF。

    参数：
        self.bone_cam_trans: 初始a的位置。
        self.bone_cam_rot: 初始a的旋转欧拉角。
        self.bone_trans: 初始b的位置。
        self.bone_rot: 初始b的旋转欧拉角。
        smoothed_positions: a的新位置。
        smoothed_rotations: a的新旋转欧拉角。

    返回：
        b的新位置和旋转欧拉角。
    """
    # 创建a和b的初始全局变换
    initial_transform_a = euler_to_transform(self.bone_cam_trans, self.bone_cam_rot)
    initial_transform_b = euler_to_transform(self.bone_trans, self.bone_rot)

    # 计算初始相对变换T_ab = T_a^-1 * T_b
    transform_a_inverse = vtk.vtkTransform()
    transform_a_inverse.DeepCopy(initial_transform_a)
    transform_a_inverse.Inverse()

    transform_ab = vtk.vtkTransform()
    transform_ab.Concatenate(transform_a_inverse)
    transform_ab.Concatenate(initial_transform_b)

    # 计算a的新全局变换
    new_transform_a = euler_to_transform(smoothed_positions, smoothed_rotations)

    # 计算b的新全局变换 T_b' = T_a' * T_ab
    new_transform_b = vtk.vtkTransform()
    new_transform_b.DeepCopy(new_transform_a)
    new_transform_b.Concatenate(transform_ab)

    # 提取b的新6DOF
    new_b_position, new_b_euler_angles = transform_to_euler(new_transform_b)
    return new_b_position, new_b_euler_angles




def calculate_a_dof(r_ab_pre, b_pos_post, b_rot_post):
    # 点 a 在移动后的全局位置
    a_pos_post = b_pos_post + np.dot(b_rot_post, r_ab_pre)

    # 点 a 的移动后旋转与 b 一致（刚体约束）
    a_rot_post = b_rot_post

    return a_pos_post, a_rot_post

def euler_to_transform(position, euler_angles):
    """
    将位置和欧拉角（XYZ顺序，角度制）转换为 vtkTransform。
    """
    transform = vtk.vtkTransform()
    transform.Translate(*position)

    # 计算旋转矩阵并扩展为 4x4 矩阵
    rotation_matrix = R.from_euler('xyz', euler_angles, degrees=True).as_matrix()
    matrix_4x4 = np.eye(4)
    matrix_4x4[:3, :3] = rotation_matrix  # 插入旋转矩阵

    # 转换为平铺的列表形式并应用到 vtkTransform
    transform.Concatenate(matrix_4x4.flatten())
    return transform

def transform_to_euler(transform):
    """
    从 vtkTransform 中提取位置和欧拉角。
    返回值为 (position, euler_angles)。
    """
    # 提取位置
    position = transform.GetPosition()

    # 将 vtkMatrix4x4 转换为 NumPy 数组
    vtk_matrix = transform.GetMatrix()
    matrix_4x4 = np.zeros((4, 4))
    for i in range(4):
        for j in range(4):
            matrix_4x4[i, j] = vtk_matrix.GetElement(i, j)

    # 提取旋转矩阵 (3x3)
    rotation_matrix = matrix_4x4[:3, :3]

    # 将旋转矩阵转换为欧拉角
    euler_angles = R.from_matrix(rotation_matrix).as_euler('xyz', degrees=True)

    return position, euler_angles


class DeviceMessage:
    def __init__(self, message_bytes):
        self.valid = False
        self.device_id = None
        self.coordinates = None
        self.rotations = None
        self.parse_message(message_bytes)
        
    def parse_message(self, message_bytes):
        # Unpack the message based on the format
        try:
            fixed, msg_length, device_id, x, y, z, xr, yr, zr,extra_data, _, _ = struct.unpack('!HHB7f12s4s', message_bytes)
            
            if fixed == 0x0F0F:
                crc_received = message_bytes[-4:]
                crc_calculated = self.calculate_crc(message_bytes[:-4])
                
                if crc_received == crc_calculated:
                    self.valid = True
                    self.device_id = device_id
                    self.coordinates = (x, y, z)
                    self.rotations = (xr, yr, zr)
                    
        except struct.error:
            pass  # handle the error, e.g., log it
    
    def calculate_crc(self, data):
        # Implement the CRC calculation here
        crc = binascii.crc32(data)
        return struct.pack('!I', crc)

class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)

        self.active_bone_stl = "bone-feature-cali.stl"
        # self.active_bone_stl = "femul-l.stl"
        self.active_tunnel_stl_1 = "t_f_l_1.stl"
        self.active_tunnel_stl_2 = "t_f_l_2.stl"

        self.sub_view_centre = [0,0,0]

        self.resize(1024, 768)

        self.main_frame = QtWidgets.QFrame()
        self.main_layout = QtWidgets.QGridLayout(self.main_frame)

        self.vtkWidget_main  = QVTKRenderWindowInteractor(self.main_frame)
        self.main_layout.addWidget(self.vtkWidget_main, 0, 0, 3, 2)

        self.vtkWidget_top = QVTKRenderWindowInteractor(self.main_frame)
        self.vtkWidget_front = QVTKRenderWindowInteractor(self.main_frame)
        self.vtkWidget_side = QVTKRenderWindowInteractor(self.main_frame)



        self.main_layout.addWidget(self.vtkWidget_top, 0, 2, 1, 1)     # 顶视图，占用 1 行 1 列
        self.main_layout.addWidget(self.vtkWidget_front, 1, 2, 1, 1)   # 前视图，占用 1 行 1 列
        self.main_layout.addWidget(self.vtkWidget_side, 2, 2, 1, 1)    # 侧视图，占用 1 行 1 列

        self.main_layout.setColumnStretch(0, 1)  # 第 0 列的拉伸比例为 1
        self.main_layout.setColumnStretch(1, 1)  # 第 1 列的拉伸比例为 1
        self.main_layout.setColumnStretch(2, 1)  # 第 2 列的拉伸比例为 1
        # 初始化渲染器
        self.ren_main = vtk.vtkRenderer()
        self.ren_top = vtk.vtkRenderer()
        self.ren_front = vtk.vtkRenderer()
        self.ren_side = vtk.vtkRenderer()

        self.ren_window_list = [self.vtkWidget_main,self.vtkWidget_top,self.vtkWidget_front,self.vtkWidget_side] 
        self.ren_list = [self.ren_main,self.ren_top,self.ren_front,self.ren_side]
        
        
        for ren_window, ren in zip(self.ren_window_list, self.ren_list):
            # ren_window.SetSwapBuffers(1)
            # ren_window.SetDesiredUpdateRate(15)
            ren.SetAllocatedRenderTime(1.0 / 15)


        # 绑定渲染器到交互窗口
        self.vtkWidget_main.GetRenderWindow().AddRenderer(self.ren_main)
        self.vtkWidget_top.GetRenderWindow().AddRenderer(self.ren_top)
        self.vtkWidget_front.GetRenderWindow().AddRenderer(self.ren_front)
        self.vtkWidget_side.GetRenderWindow().AddRenderer(self.ren_side)

        self.iren_main = self.vtkWidget_main.GetRenderWindow().GetInteractor()
        self.iren_top = self.vtkWidget_top.GetRenderWindow().GetInteractor()
        self.iren_front = self.vtkWidget_front.GetRenderWindow().GetInteractor()
        self.iren_side = self.vtkWidget_side.GetRenderWindow().GetInteractor()

        # self.ren = vtk.vtkRenderer()
        # self.vtkWidget.GetRenderWindow().AddRenderer(self.ren)
        # self.iren = self.vtkWidget.GetRenderWindow().GetInteractor()


        self.init_vtk()

        self.iren_main.Initialize()
        self.iren_top.Initialize()
        self.iren_front.Initialize()
        self.iren_side.Initialize()

        self.init_ui()
        self.position_dict = {}
        self.server_thread_running = True
        self.lock = threading.Lock()
        self.server_thread = threading.Thread(target=self.update_coordinates)
        self.server_thread.start()

        self.probe_vec_x = 0
        self.probe_vec_y = 0
        self.probe_vec_z = 0
        self.drill_vec_x = 0
        self.drill_vec_y = 0
        self.drill_vec_z = 0

        self.mouse_picker = vtk.vtkWorldPointPicker()
        self.iren_main.AddObserver("MouseMoveEvent", self.mouse_move_callback)

        self.previous_positions = {}
        self.previous_rotations = {}
        self.smoothing_factor = 0.1  # 控制平滑程度，值越小越平滑

        self.hip_coordinates = []
        self.finding_hip = False
        self.is_registed = False
        self.is_reg_probe = True
        self.bone_rot = [0,0,0] 
        self.bone_cam_trans = [0,0,0] 
        self.bone_cam_rot = [0,0,0] 

        self.probe_trans_vector = [0,160,-4]
        self.drill_trans_vector = [0,188,-70]
        self.active_trans_vector = self.probe_trans_vector

        self.test_counter_1 = 0
        self.test_counter_2 = 0
        self.test_6dof = []

    def set_camera_position(self, x, y, z):
        camera = self.ren_main.GetActiveCamera()
        camera.SetPosition(x, y, z)  # 设置相机位置
        camera.SetFocalPoint(0, 0, 600)  # 对准原点
        if x == 0 and y == 0:
            camera.SetViewUp(0, 1, 0)  # 设置Y轴为“上”方向
        else:
            camera.SetViewUp(0, 0, -1)  # 设置Z轴为“上”方向
        self.ren_main.ResetCameraClippingRange()  # 更新裁剪范围
        self.vtkWidget_main.GetRenderWindow().Render()  # 请求渲染以更新视图

    def mouse_move_callback(self, obj, event):
        x, y = self.iren_main.GetEventPosition()
        self.mouse_picker.Pick(x, y, 0, self.ren_main)
        picked_position = self.mouse_picker.GetPickPosition()
        picked_position_int = tuple(int(round(coord)) for coord in picked_position)
        self.mouse_position_label.setText(f"Mouse Position: {picked_position_int}")
        self.vtkWidget_main.GetRenderWindow().Render()

    def update_opacity(self, actor, value):
        opacity = value / 100.0  # 将滑块值映射到 0.0 ~ 1.0
        actor.GetProperty().SetOpacity(opacity)  # 设置透明度
        self.vtkWidget_main.GetRenderWindow().Render()
        self.vtkWidget_front.GetRenderWindow().Render()
        self.vtkWidget_top.GetRenderWindow().Render()
        self.vtkWidget_side.GetRenderWindow().Render()

    def toggle_probe_drill(self, index):
        selected_option = self.reg_probe_combobox.currentText()
        if selected_option == "Probe":
            self.is_reg_probe = True
            self.active_trans_vector = self.probe_trans_vector
        elif selected_option == "Drill":
            self.is_reg_probe = False
            self.active_trans_vector = self.drill_trans_vector

        print(f"Reg Probe Enabled: {self.is_reg_probe}")  # 打印当前状态（调试用）

    def init_ui(self):
        self.button_start = QtWidgets.QPushButton('Start', self)
        self.button_start.clicked.connect(self.start_animation)

        self.button_reset = QtWidgets.QPushButton('Reset Camera', self)
        self.button_reset.clicked.connect(self.reset_camera)

        self.button_camera1 = QtWidgets.QPushButton('xy-plane', self)
        self.button_camera1.clicked.connect(lambda: self.set_camera_position(0, 0, -300))

        self.button_camera2 = QtWidgets.QPushButton('xz-plane', self)
        self.button_camera2.clicked.connect(lambda: self.set_camera_position(0, -800, 600))

        self.button_camera3 = QtWidgets.QPushButton('yz-plane', self)
        self.button_camera3.clicked.connect(lambda: self.set_camera_position(800, 0, 600))

        self.button_find_hip = QtWidgets.QPushButton('Registration', self)
        self.button_find_hip.clicked.connect(self.start_finding_hip)

        layout = QtWidgets.QHBoxLayout()
        layout.addWidget(self.button_start)
        layout.addWidget(self.button_reset)
        layout.addWidget(self.button_camera1)
        layout.addWidget(self.button_camera2)
        layout.addWidget(self.button_camera3)
        self.mouse_position_label = QtWidgets.QLabel("Mouse Position: (0, 0, 0)", self)


        # layout.addWidget(self.mouse_position_label)
        # Add a container widget for the mouse position label
        mouse_position_container = QtWidgets.QWidget()
        mouse_position_layout = QtWidgets.QHBoxLayout()
        mouse_position_container.setLayout(mouse_position_layout)

        # Add the label to the container
        mouse_position_layout.addWidget(self.mouse_position_label)

        # Set a fixed percentage width (e.g., 20%)
        mouse_position_container.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Preferred)
        mouse_position_container.setFixedWidth(int(self.width() * 0.3))  # 20% of the main window's width

        # Add the container to the main layout
        layout.addWidget(mouse_position_container)

        self.main_layout.addLayout(layout, 3, 0, 1, 3)

        self.find_hip_points = 6
        self.progress_bar = QtWidgets.QProgressBar(self)
        self.progress_bar.setRange(0, self.find_hip_points)
        self.progress_bar.setValue(0)

        find_hip_layout = QtWidgets.QHBoxLayout()
        find_hip_layout.addWidget(self.button_find_hip, stretch=1)  # Button takes 50%
        find_hip_layout.addWidget(self.progress_bar, stretch=1)    # Progress bar takes 50%

        self.reg_probe_combobox = QtWidgets.QComboBox(self)
        self.reg_probe_combobox.addItems(["Probe", "Drill"])  # 添加选项
        self.reg_probe_combobox.setCurrentIndex(0)  # 默认选中第一个选项（Probe）
        self.reg_probe_combobox.currentIndexChanged.connect(self.toggle_probe_drill)  # 绑定事件
        find_hip_layout.addWidget(self.reg_probe_combobox)

        self.bone_opacity_slider = QtWidgets.QSlider(Qt.Horizontal, self)
        self.bone_opacity_slider.setRange(0, 100)  # 范围从 0 到 100
        self.bone_opacity_slider.setValue(100)  # 默认值为 100% 不透明
        self.bone_opacity_slider.valueChanged.connect(lambda value: self.update_opacity(self.actors[2], value))

        self.tunnel_opacity_slider = QtWidgets.QSlider(Qt.Horizontal, self)
        self.tunnel_opacity_slider.setRange(0, 100)  # 范围从 0 到 100
        self.tunnel_opacity_slider.setValue(100)  # 默认值为 100% 不透明
        self.tunnel_opacity_slider.valueChanged.connect(lambda value: self.update_opacity(self.actors[3], value))  # 绑定事件
        self.tunnel_opacity_slider.valueChanged.connect(lambda value: self.update_opacity(self.actors[4], value))  # 绑定事件

        find_hip_layout.addWidget(QtWidgets.QLabel("Opacity_B:"))  # 添加标签
        find_hip_layout.addWidget(self.bone_opacity_slider)

        find_hip_layout.addWidget(QtWidgets.QLabel("Opacity_T:"))  # 添加标签
        find_hip_layout.addWidget(self.tunnel_opacity_slider)

        self.main_layout.addLayout(find_hip_layout, 4, 0, 1, 3)


        self.probe_vector_x = QtWidgets.QLineEdit(self)
        self.probe_vector_y = QtWidgets.QLineEdit(self)
        self.probe_vector_z = QtWidgets.QLineEdit(self)
        self.drill_vector_x = QtWidgets.QLineEdit(self)
        self.drill_vector_y = QtWidgets.QLineEdit(self)
        self.drill_vector_z = QtWidgets.QLineEdit(self)

        self.probe_vector_x.setText("0.0")
        self.probe_vector_y.setText("160.0")
        self.probe_vector_z.setText("-4.0")
        self.drill_vector_x.setText("0.0")
        self.drill_vector_y.setText("188.0")
        self.drill_vector_z.setText("-70.0")

        input_layout = QtWidgets.QHBoxLayout()

        input_layout.addWidget(QtWidgets.QLabel("Probe_X:"))
        input_layout.addWidget(self.probe_vector_x)
        input_layout.addWidget(QtWidgets.QLabel("Probe_Y:"))
        input_layout.addWidget(self.probe_vector_y)
        input_layout.addWidget(QtWidgets.QLabel("Probe_Z:"))
        input_layout.addWidget(self.probe_vector_z)

        input_layout.addWidget(QtWidgets.QLabel("Drill_X:"))
        input_layout.addWidget(self.drill_vector_x)
        input_layout.addWidget(QtWidgets.QLabel("Drill_Y:"))
        input_layout.addWidget(self.drill_vector_y)
        input_layout.addWidget(QtWidgets.QLabel("Drill_Z:"))
        input_layout.addWidget(self.drill_vector_z)

        self.button_apply = QtWidgets.QPushButton('Apply Transform', self)
        self.button_apply.clicked.connect(self.adjust_probe_drill_vec)
        input_layout.addWidget(self.button_apply)

        self.main_layout.addLayout(input_layout,5, 0, 1, 3)

        self.main_frame.setLayout(self.main_layout)
        self.setCentralWidget(self.main_frame)
        self.show()

    def start_finding_hip(self):
        # self.hip_coordinates.clear()
        # self.progress_bar.setValue(0)
        self.finding_hip = True
        self.is_registed = False

    def adjust_probe_drill_vec(self):
        try:
            self.probe_vec_x = float(self.probe_vector_x.text())
            self.probe_vec_y = float(self.probe_vector_y.text())
            self.probe_vec_z = float(self.probe_vector_z.text())

            self.drill_vec_x = float(self.drill_vector_x.text())
            self.drill_vec_y = float(self.drill_vector_y.text())
            self.drill_vec_z = float(self.drill_vector_z.text())

            self.probe_trans_vector = [ self.probe_vec_x, self.probe_vec_y, self.probe_vec_z]
            self.drill_trans_vector = [ self.drill_vec_x, self.drill_vec_y, self.drill_vec_z]


        except ValueError:
            QtWidgets.QMessageBox.warning(self, "Input Error", "Please enter valid numbers!")

    def init_cubes(self, position, color):
        cube = vtk.vtkCubeSource()
        cube.SetXLength(2)
        cube.SetYLength(2)
        cube.SetZLength(2)

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(cube.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.SetPosition(position)
        actor.GetProperty().SetColor(color)

        #self.ren.AddActor(actor)

        return actor
    
    def init_pin(self, position, color,size,center, opacity=1.0):
        cube = vtk.vtkCubeSource()
        cube.SetXLength(size[0])
        cube.SetYLength(size[1])
        cube.SetZLength(size[2])

        cube.SetCenter(center)

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(cube.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.SetPosition(position)
        actor.GetProperty().SetColor(color)
        actor.GetProperty().SetOpacity(opacity) 

        self.ren_main.AddActor(actor)
        self.ren_top.AddActor(actor)
        self.ren_front.AddActor(actor)
        self.ren_side.AddActor(actor)

        return actor

    def init_stl_actor(self, filename, color=(1.0, 1.0, 1.0), opacity=1.0, reduction=0.9):
        reader = vtk.vtkSTLReader()
        reader.SetFileName(filename)

        # 简化多边形
        decimate = vtk.vtkQuadricDecimation()
        decimate.SetInputConnection(reader.GetOutputPort())
        decimate.SetTargetReduction(reduction)

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(decimate.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)

        actor.GetProperty().SetColor(color)
        actor.GetProperty().SetOpacity(opacity)

        return actor

    def init_vtk(self):
        self.actors = []

        actor_a = self.init_cubes((0, 0, 0), (1, 0, 0))
        self.actors.append(actor_a)

        actor_b = self.init_stl_actor("drill-mod-3.stl", color=(1.0, 1.0, 0), opacity=0)
        actor_b.SetPosition(0,0,100)
        self.actors.append(actor_b)


        # actor_c = self.init_stl_actor("bone-feature-cali.stl", color=(1.0, 1.0, 1.0), opacity=1)
        actor_c = self.init_stl_actor(self.active_bone_stl, color=(1.0, 1.0, 1.0), opacity=0.8)
        self.actors.append(actor_c)

        actor_t_1 = self.init_stl_actor(self.active_tunnel_stl_1, color=(1.0, 1.0, 0.0), opacity=0.5)
        self.actors.append(actor_t_1)

        actor_t_2 = self.init_stl_actor(self.active_tunnel_stl_2, color=(1.0, 1.0, 0.0), opacity=0.5)
        self.actors.append(actor_t_2)        

        self.guide_pin = self.init_pin((0, 0, 20), (0, 1, 0),(2,200,2),(0,100,0),0.2)
        self.real_pin = self.init_pin((0, 0, 20), (0, 1, 1),(2,200,2),(0,-100,0))

        for actor in self.actors:
            self.ren_main.AddActor(actor)
            self.ren_top.AddActor(actor)
            self.ren_front.AddActor(actor)
            self.ren_side.AddActor(actor)

        self.reset_camera()

        self.timer_id = None

    def update_bone_transform(self, smoothed_positions, smoothed_rotations):
        new_b_position, new_b_rotations = compute_new_b_6dof(
            self, smoothed_positions, smoothed_rotations
        )
        return new_b_position, new_b_rotations

    def update_coordinates(self):
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind(('localhost', 12341))
        server_socket.listen(5)
        server_socket.settimeout(1)

        try:
            while self.server_thread_running:
                try:
                    client_socket, addr = server_socket.accept()
                    if self.server_thread_running:
                        client_thread = threading.Thread(target=self.handle_client, args=(client_socket,))
                        client_thread.start()
                except socket.timeout:
                    continue
        finally:
            server_socket.close()

    def handle_client(self, client_socket):
        try:
            while self.server_thread_running:
                message_bytes = client_socket.recv(50)
                if not message_bytes:
                    break

                message = DeviceMessage(message_bytes)
                
                if message.valid:
                    with self.lock:
                        self.position_dict[message.device_id] = (message.coordinates, message.rotations)
                        
        except Exception as e:
            print(f"An error occurred while handling client: {e}")
        finally:
            client_socket.close()

    def start_animation(self):
        if self.timer_id is None:
            self.timer_id = self.startTimer(55)
            self.button_start.setText('Stop')
        else:
            self.killTimer(self.timer_id)
            self.timer_id = None
            self.button_start.setText('Start')

    def smooth(self, prev_value, new_value):
            return (1 - self.smoothing_factor) * np.array(prev_value) + self.smoothing_factor * np.array(new_value)

    def normalize_angle(self, angle):
        return (angle + 180) % 360 - 180

    def smooth_angle(self, prev_angle, new_angle):
        prev_angle = self.normalize_angle(prev_angle)
        new_angle = self.normalize_angle(new_angle)

        delta = new_angle - prev_angle
        if delta > 180:
            new_angle -= 360
        elif delta < -180:
            new_angle += 360

        smoothed_angle = (1 - self.smoothing_factor) * prev_angle + self.smoothing_factor * new_angle
        return self.normalize_angle(smoothed_angle)
    
    def calculate_distance(self, point1, point2):
        return sum((p1 - p2) ** 2 for p1, p2 in zip(point1, point2)) ** 0.5
    
    def move_bone_tunnel(self, pos, rot):
        self.actors[2].SetPosition(0, 0, 0)
        self.actors[2].SetOrientation(rot)
        self.actors[2].SetPosition(pos)
        self.actors[3].SetPosition(0, 0, 0)
        self.actors[3].SetOrientation(rot)
        self.actors[3].SetPosition(pos)
        self.actors[4].SetPosition(0, 0, 0)
        self.actors[4].SetOrientation(rot)
        self.actors[4].SetPosition(pos)     

        self.sub_view_centre = pos  

    def timerEvent(self, event):
        with self.lock:
            for i, actor in enumerate(self.actors):
                position_rotation = self.position_dict.get(i)
                if position_rotation:
                    position, rotations = position_rotation
                    tx, ty, tz = position
                    rx, ry, rz = rotations

                    if i == 1:
                        rx_new = rx 
                        ry_new = ry 
                        rz_new = rz 

                        tx_new = tx 
                        ty_new = ty 
                        tz_new = tz 

                        new_rotations_degrees = [rx_new, ry_new, rz_new]
                        new_position_fixed = [tx_new, ty_new, tz_new]
                    elif i == 2:
                        rx_new = -rx + 180 
                        ry_new = -ry 
                        rz_new = rz + 180 

                        tx_new = tx
                        ty_new = ty
                        tz_new = tz

                        # new_rotations_degrees = [rx_new, ry_new, rz_new]
                        new_rotations_degrees= [rx, ry, rz]
                        new_position_fixed = [tx_new, ty_new, tz_new]

                    else:
                        new_rotations_degrees = [rx, ry, rz]
                        new_position_fixed = [tx, ty, tz]

                    prev_rotations = self.previous_rotations.get(i, new_rotations_degrees)
                    prev_positions = self.previous_positions.get(i, new_position_fixed)

                    smoothed_rotations = [
                        self.smooth_angle(prev_rotations[j], new_rotations_degrees[j]) for j in range(3)
                    ]
                    smoothed_positions = self.smooth(prev_positions, new_position_fixed)

                    self.previous_rotations[i] = smoothed_rotations
                    self.previous_positions[i] = smoothed_positions

                    if i == 1:

                        actor.SetPosition(0, 0, 0)
                        actor.SetOrientation(smoothed_rotations)
                        actor.SetPosition(smoothed_positions.tolist())
                        
                        b_pos_post = np.array(smoothed_positions)  # 点 b 的移动后位置
                        b_rot_post_quat = R.from_euler('xyz', smoothed_rotations, degrees=True).as_quat()
                        b_rot_post = R.from_quat(b_rot_post_quat).as_matrix()
                        rx, ry, rz = smoothed_rotations
                        new_smoothed_rotations = [rx, ry, rz]

                        self.test_counter_1 = self.test_counter_1 +1
                        if self.test_counter_1 == 120:
                            self.test_counter_1 = 0
                            print(smoothed_positions, smoothed_rotations)
                            self.test_6dof.append([smoothed_positions, smoothed_rotations])
                            self.test_counter_2 = self.test_counter_2 + 1
                            if self.test_counter_2 == 50:
                                self.test_counter_2 = 0
                                size_range = (5, 20, 20)
                                res= find_fixed_point_lsq(self.test_6dof,self.active_trans_vector,size_range)
                                print(self.test_6dof)
                                # print(res)
                                self.test_6dof= []


                        maker_position = rotate_and_translate( self.active_trans_vector,new_smoothed_rotations,b_pos_post)
                        formatted = [f"{num:8.2f}" for num in maker_position]
                        print(formatted)


                        self.guide_pin.SetPosition(0, 0, 0)
                        self.guide_pin.SetOrientation(smoothed_rotations)
                        self.guide_pin.SetPosition(maker_position.tolist())

                        self.real_pin.SetPosition(0, 0, 0)
                        self.real_pin.SetOrientation(smoothed_rotations)
                        self.real_pin.SetPosition(maker_position.tolist())

                        if self.finding_hip:
                            if len(self.hip_coordinates) < self.find_hip_points:
                                if len(self.hip_coordinates) == 0:
                                    self.move_bone_tunnel(maker_position.tolist(),(0,0,0))
                                    self.reset_sub_camera() 
                                
                                self.hip_coordinates.append(maker_position)
                                self.progress_bar.setValue(len(self.hip_coordinates))
               

                            if len(self.hip_coordinates) == self.find_hip_points:
                                self.finding_hip = False
                                self.is_registed = True
                                formatted_coordinates = [coord.tolist() for coord in self.hip_coordinates]
                                # print(formatted_coordinates)
                                try:
                                    self.bone_cam_trans = self.previous_positions[2]
                                    self.bone_cam_rot = self.previous_rotations[2]
                                except IndexError:
                                    print("bone tracker isn't working")
                                self.bone_trans = formatted_coordinates[0]
                                self.bone_rot = -optimize_rotation_angle(formatted_coordinates,"bone-feature-cali.stl")
                                # print(self.bone_rot)
                                self.move_bone_tunnel(self.bone_trans,self.bone_rot)
                                self.reset_sub_camera() 
                               
                                self.hip_coordinates.clear()

                            self.finding_hip = False


                    if i==2 and self.is_registed:
                        new_smoothed_positions, new_smoothed_rotations = self.update_bone_transform(smoothed_positions, smoothed_rotations)
                        self.move_bone_tunnel(new_smoothed_positions,new_smoothed_rotations)
                            
        self.vtkWidget_main.GetRenderWindow().Render()
        self.vtkWidget_front.GetRenderWindow().Render()
        self.vtkWidget_top.GetRenderWindow().Render()
        self.vtkWidget_side.GetRenderWindow().Render()

    def reset_sub_camera(self):
        xc,yc,zc = self.sub_view_centre 

        camera_front = self.ren_front.GetActiveCamera()
        camera_front.SetPosition(xc, yc-300, zc)
        camera_front.SetFocalPoint(xc,yc,zc)
        camera_front.SetViewUp(0, 0, -1)
        self.ren_front.ResetCameraClippingRange()
        self.vtkWidget_front.GetRenderWindow().Render()

        camera_top = self.ren_top.GetActiveCamera()
        camera_top.SetPosition(xc,yc,zc-300)
        camera_top.SetFocalPoint(xc,yc,zc)
        camera_top.SetViewUp(0, 1, 0)
        self.ren_top.ResetCameraClippingRange()        
        self.vtkWidget_top.GetRenderWindow().Render()

        camera_side = self.ren_side.GetActiveCamera()
        camera_side.SetPosition(xc-300, yc, zc)
        camera_side.SetFocalPoint(xc,yc,zc)
        camera_side.SetViewUp(0, 0, -1)
        self.ren_side.ResetCameraClippingRange()          
        self.vtkWidget_side.GetRenderWindow().Render()

    def reset_camera(self):
        camera_main = self.ren_main.GetActiveCamera()
        camera_main.SetPosition(1200, 1200, 1200)
        camera_main.SetFocalPoint(0, 0, 0)
        camera_main.SetViewUp(0, 0, 1)
        self.ren_main.ResetCameraClippingRange()
        self.vtkWidget_main.GetRenderWindow().Render()

        self.reset_sub_camera()


    def closeEvent(self, event):
        self.vtkWidget_main.GetRenderWindow().Finalize()
        self.iren_main.TerminateApp()
        
        self.vtkWidget_top.GetRenderWindow().Finalize()
        self.iren_top.TerminateApp()

        self.vtkWidget_front.GetRenderWindow().Finalize()
        self.iren_front.TerminateApp()

        self.vtkWidget_side.GetRenderWindow().Finalize()
        self.iren_side.TerminateApp()

        event.accept()  # 接受关闭事件

        with self.lock:
            self.server_thread_running = False  
            self.server_thread.join()  

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    sys.exit(app.exec_())
