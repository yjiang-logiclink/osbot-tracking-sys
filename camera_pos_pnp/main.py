import multiprocessing
import socket
import json
import time
import numpy as np
import cv2
from concurrent.futures import ProcessPoolExecutor, as_completed

from camera_pose_optimizer import CameraPoseOptimizer, Camera
# from device_client import DeviceClient   # 暂时不用


# ---------------- UDP Listener ----------------
def udp_listener(shared_dict, port=9000):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", port))
    print(f"[INFO ] UDP listening on {port}")

    while True:
        data, _ = sock.recvfrom(4096)
        try:
            packet = json.loads(data.decode("utf-8"))
            shared_dict["latest"] = packet   # 覆盖旧数据
        except Exception as e:
            print(f"[WARN ] Failed to parse UDP packet: {e}")


# ---------------- 单个计算任务 ----------------
def compute_task(device_id, coord_f, coord_m, camera_f, camera_m):
    optimizer = CameraPoseOptimizer()
    try:
        # undistort
        coord_f_np = np.array(coord_f, dtype=np.float32).reshape(-1,1,2)
        coord_m_np = np.array(coord_m, dtype=np.float32).reshape(-1,1,2)

        undist_f = cv2.undistortPoints(
            coord_f_np, camera_f.camMatrix, camera_f.dist_coeffs, None, camera_f.camMatrix
        ).reshape(-1,2)
        undist_m = cv2.undistortPoints(
            coord_m_np, camera_m.camMatrix, camera_m.dist_coeffs, None, camera_m.camMatrix
        ).reshape(-1,2)

        res = optimizer.optimize_camera_pose(camera_f, camera_m, undist_f, undist_m)
        return device_id, res, undist_f, undist_m

    except Exception as e:
        return device_id, {"error": str(e)}, None, None


# ---------------- 主计算循环 ----------------
def compute_and_print(shared_dict, camera_f, camera_ms):
    # device_client = DeviceClient("localhost", 12341)  # 暂时不用

    with ProcessPoolExecutor(max_workers=2) as executor:
        while True:
            if "latest" not in shared_dict:
                time.sleep(0.01)
                continue

            packet = shared_dict["latest"]
            cam1, cam2, cam3 = packet.get("Cam_1", []), packet.get("Cam_2", []), packet.get("Cam_3", [])
            if len(cam1) < 4 or len(cam2) < 2 or len(cam3) < 2:
                continue

            # device1 = cam1(0,1) + cam2(0,1)
            coord_f1, coord_m1 = [tuple(cam1[0]), tuple(cam1[1])], [tuple(cam2[0]), tuple(cam2[1])]
            # device2 = cam1(2,3) + cam3(0,1)
            coord_f2, coord_m2 = [tuple(cam1[2]), tuple(cam1[3])], [tuple(cam3[0]), tuple(cam3[1])]

            futures = [
                executor.submit(compute_task, 1, coord_f1, coord_m1, camera_f, camera_ms[0]),
                executor.submit(compute_task, 2, coord_f2, coord_m2, camera_f, camera_ms[1])
            ]

            results = {}
            for f in as_completed(futures):
                device_id, res, undist_f, undist_m = f.result()
                if "error" in res:
                    print(f"[ERR  ] Device {device_id} failed: {res['error']}")
                elif res["Precision"] < 1e3:
                    results[device_id] = (res, undist_f, undist_m)
                else:
                    print(f"[WARN ] Device {device_id}: precision too low")

            # 打印两个 device 的结果
            if len(results) == 2:
                res1, res2 = results[1][0], results[2][0]

                line = (
                    f"[RESULT] "
                    f"Dev1 T=({res1['Optimized T'][0]:7.2f},{res1['Optimized T'][1]:7.2f},{res1['Optimized T'][2]:7.2f}) "
                    f"Euler=({res1['Euler Angles'][0]:7.2f},{res1['Euler Angles'][1]:7.2f},{res1['Euler Angles'][2]:7.2f}) | "
                    f"Dev2 T=({res2['Optimized T'][0]:7.2f},{res2['Optimized T'][1]:7.2f},{res2['Optimized T'][2]:7.2f}) "
                    f"Euler=({res2['Euler Angles'][0]:7.2f},{res2['Euler Angles'][1]:7.2f},{res2['Euler Angles'][2]:7.2f})"
                )

                print(line)


# ---------------- Main ----------------
def main():
    manager = multiprocessing.Manager()
    shared_dict = manager.dict()

    # ==== Define cameras (沿用之前配置) ====
    camera_f = Camera(
        cam_matrix=np.array([[965.94630676, 0, 983.97728317],
                             [0, 965.72766751, 646.97090995],
                             [0, 0, 1]]),
        marker_loc=np.array([[-5, -22.5, 3],[5, 35, 3]]),
        dist_coeffs=np.array([-0.07194899,0.07804442,0.00244278,-0.00132859,-0.04441922])
    )

    camera_pat = Camera(
        cam_matrix=np.array([[965.94630676, 0, 983.97728317],
                             [0, 965.72766751, 646.97090995],
                             [0, 0, 1]]),
        marker_loc=np.array([[-5, -22.5, 3],[5, 35, 3]]),
        dist_coeffs=np.array([-0.07194899,0.07804442,0.00244278,-0.00132859,-0.04441922])
    )

    camera_probe = Camera(
        cam_matrix=np.array([[965.94630676, 0, 983.97728317],
                             [0, 965.72766751, 646.97090995],
                             [0, 0, 1]]),
        marker_loc=np.array([[-5, -22.5, 3],[5, 35, 3]]),
        dist_coeffs=np.array([-0.07194899,0.07804442,0.00244278,-0.00132859,-0.04441922])
    )

    camera_ms = [camera_pat, camera_probe]  # device1, device2 对应相机

    # ==== 启动 UDP 接收进程 ====
    p = multiprocessing.Process(target=udp_listener, args=(shared_dict, 9000), daemon=True)
    p.start()

    # ==== 主计算循环 ====
    try:
        compute_and_print(shared_dict, camera_f, camera_ms)
    except KeyboardInterrupt:
        print("\n[INFO ] Shutting down...")
    finally:
        p.terminate()
        p.join()


if __name__ == "__main__":
    main()
