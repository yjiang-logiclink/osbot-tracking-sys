import cv2
import numpy as np
from scipy.optimize import linear_sum_assignment

class LedTracker:
    def __init__(self, expected_width=None, expected_height=None):
        self.expected_width = expected_width
        self.expected_height = expected_height
        self.last_leds = [(0, 0), (0, 0), (0, 0), (0, 0)]
        self.tracking_enabled = False

    @staticmethod
    def find_n_brightest_points(gray, n=2, threshold_ratio=0.8, check_size=18, min_threshold=120, min_dist=30):
        """找到灰度图中 n 个最亮点，保证点之间间距 >= min_dist"""
        points = []
        for _ in range(n):
            _, max_val, _, max_loc = cv2.minMaxLoc(gray)
            threshold = max_val * threshold_ratio

            if threshold <= min_threshold:
                # 没有足够的亮点，填充 (0,0)
                points.append((0, 0))
                continue

            x, y = max_loc

            # 检查与已有点的距离
            too_close = False
            for (px, py) in points:
                if (px, py) == (0, 0):
                    continue
                dx, dy = x - px, y - py
                if dx*dx + dy*dy < min_dist*min_dist:
                    too_close = True
                    break

            if too_close:
                # 这个点太靠近已有点 → 清零并跳过
                gray[y, x] = 0
                points.append((0, 0))
                continue

            # 计算局部平均位置
            x1, y1 = max(0, x - check_size), max(0, y - check_size)
            x2, y2 = min(gray.shape[1], x + check_size), min(gray.shape[0], y + check_size)
            roi = gray[y1:y2, x1:x2]
            binary_mask = (roi >= threshold).astype(np.uint8)
            pts = np.argwhere(binary_mask > 0)

            if pts.size > 0:
                avg_x = int(np.mean(pts[:, 1])) + x1
                avg_y = int(np.mean(pts[:, 0])) + y1
                points.append((avg_x, avg_y))
                gray[y1:y2, x1:x2] = 0
            else:
                points.append((0, 0))

        # 确保长度为 n
        while len(points) < n:
            points.append((0, 0))

        return points

    @staticmethod
    def _within_distance(p1, p2, threshold=200):
        """检查两点是否在指定距离内"""
        if p1 == (0, 0):  # 之前没记录过 → 直接接受
            return True
        dx, dy = p1[0] - p2[0], p1[1] - p2[1]
        return dx*dx + dy*dy <= threshold*threshold

    def update_leds(self, frame, leds_to_track=[0, 1, 2, 3]):
        if len(frame.shape) == 3 and frame.shape[2] == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame

        n = len(leds_to_track)
        cal_leds = self.find_n_brightest_points(gray, n=n)
        detected_flags = [False] * len(self.last_leds)

        if n == 1:
            if cal_leds[0] != (0, 0):
                self.last_leds[leds_to_track[0]] = cal_leds[0]
                detected_flags[leds_to_track[0]] = True
            return self.last_leds, detected_flags

        last_leds = [self.last_leds[i] for i in leds_to_track]
        cost_matrix = np.zeros((n, n), dtype=np.float32)
        for i, last in enumerate(last_leds):
            for j, cur in enumerate(cal_leds):
                if cur == (0, 0):
                    cost_matrix[i, j] = 1e9
                else:
                    cost_matrix[i, j] = (last[0] - cur[0])**2 + (last[1] - cur[1])**2

        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        for r, c in zip(row_ind, col_ind):
            if cal_leds[c] != (0, 0):
                idx = leds_to_track[r]
                if (not self.tracking_enabled) or self._within_distance(self.last_leds[idx], cal_leds[c]):
                    self.last_leds[idx] = cal_leds[c]
                    detected_flags[idx] = True
                else:
                    detected_flags[idx] = False

        return self.last_leds, detected_flags