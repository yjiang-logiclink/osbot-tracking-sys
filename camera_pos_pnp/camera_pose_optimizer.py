import numpy as np
import math
import time
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R


class CameraPoseOptimizer:
    def __init__(self):
        # 初始猜测：四元数 (0,1,0,0) + 平移 (0,0,400)
        self.initial_guess = np.array([0, 1, 0, 0, 0, 0, 400])
        self.last_good_guess = self.initial_guess.copy()

    @staticmethod
    def projection_residuals(params, K_f, K_m, points_f, points_m):
        residuals = []

        # 提取四元数和平移
        q = np.array([params[0], params[1], params[2], params[3]])
        q = q / np.linalg.norm(q)  # 归一化四元数
        translation_f = params[4:]

        # 相机1变换矩阵
        rotation = R.from_quat(q)
        RT_f = np.zeros((3, 4))
        RT_f[:3, :3] = rotation.as_matrix()
        RT_f[:3, 3] = translation_f

        # 相机2变换矩阵（逆变换）
        rotation_m = R.from_matrix(RT_f[:3, :3]).inv()
        RT_m = np.hstack(
            (rotation_m.as_matrix(), -rotation_m.as_matrix() @ translation_f.reshape(-1, 1))
        )

        # 投影残差 - 相机F
        for (X, Y, Z, u, v) in points_f:
            P = np.array([X, Y, Z, 1])
            proj = K_f @ RT_f @ P
            u_hat = proj[0] / proj[2]
            v_hat = proj[1] / proj[2]
            residuals.append(u_hat - u)
            residuals.append(v_hat - v)

        # 投影残差 - 相机M
        for (X, Y, Z, u, v) in points_m:
            P = np.array([X, Y, Z, 1])
            proj = K_m @ RT_m @ P
            u_hat = proj[0] / proj[2]
            v_hat = proj[1] / proj[2]
            residuals.append(u_hat - u)
            residuals.append(v_hat - v)

        return residuals

    def optimize_camera_pose(self, camera_f, camera_m, coord_f, coord_m):
        """给定前向相机和主相机的点坐标，优化位姿"""
        K_f = camera_f.camMatrix
        K_m = camera_m.camMatrix

        if len(camera_f.marker_loc) != len(coord_f) or len(camera_m.marker_loc) != len(coord_m):
            raise ValueError("Number of marker locations and coordinates do not match.")

        # 组合点
        points_f = [
            (marker[0], marker[1], marker[2], coord[0], coord[1])
            for marker, coord in zip(camera_f.marker_loc, coord_f)
        ]
        points_m = [
            (marker[0], marker[1], marker[2], coord[0], coord[1])
            for marker, coord in zip(camera_m.marker_loc, coord_m)
        ]

        # 优化设置
        lower_bounds = [-1, -1, -1, -1, -2000, -2000, 200]
        upper_bounds = [1, 1, 1, 1, 2000, 2000, 2000]

        start_time = time.time()
        result = least_squares(
            self.projection_residuals,
            self.initial_guess,
            args=(K_f, K_m, points_f, points_m),
            bounds=(lower_bounds, upper_bounds)
        )
        elapsed = time.time() - start_time

        optimized_params = result.x
        precision = np.sum(result.fun ** 2)

        # 结果处理
        q = optimized_params[:4]
        q = q / np.linalg.norm(q)
        rotation = R.from_quat(q)
        euler_angles_rad = rotation.as_euler('xyz')
        optimized_T = np.array(optimized_params[4:])

        if precision < 1e3:
            self.last_good_guess = optimized_params
        else:
            optimized_T = self.last_good_guess[4:]
            euler_angles_rad = R.from_quat(self.last_good_guess[:4]).as_euler('xyz')

        self.initial_guess = optimized_params

        return {
            "Euler Angles": np.degrees(euler_angles_rad),
            "Optimized T": optimized_T,
            "Computation Time": elapsed,
            "Precision": precision,
        }


# ---------------- Camera 类 ----------------
class Camera:
    def __init__(self, cam_matrix, marker_loc, dist_coeffs):
        self.camMatrix = cam_matrix
        self.marker_loc = marker_loc
        self.dist_coeffs = dist_coeffs
