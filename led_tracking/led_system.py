import cv2
import time
from camera_source import CameraSource
from led_controller import LedController
from led_tracker import LedTracker
import numpy as np
from led_publisher import LedPublisher


class LedTrackingSystem:
    def __init__(self, udp_ip="0.0.0.0", udp_port=8000,
                 show_windows=True, resize_scale=0.5,
                 publish_ip="127.0.0.1", publish_port=9000,
                 debug=False):
        self.camera = CameraSource(udp_ip=udp_ip, udp_port=udp_port)
        self.is_arm = self.camera.is_arm
        self.show_windows = show_windows and (not self.is_arm)
        self.resize_scale = resize_scale

        self.frame_count = 0
        self.start_time = time.time()
        self.fps = 0.0
        self.frame_counter = 0
        self.state_delay_frames = 6

        self.state = 0
        self.state_entry_frame = 0
        self.retry_counter = 0
        self.miss_counter = [0, 0, 0, 0]

        self.trackers = {
            "Cam_1": {"tracker": LedTracker()},
            "Cam_2": {"tracker": LedTracker()},
            "Cam_3": {"tracker": LedTracker()},
        }

        # if self.show_windows:
        #     for name in self.trackers:
        #         cv2.namedWindow(name)

        self.ctrl = LedController(username="cam", password="osbot", hostname="osbot-pi5-1")
        self.publisher = LedPublisher(ip=publish_ip, port=publish_port)

        self.debug = debug

        self.state_desc = {
            0:  "Init: all LEDs off, clear last_leds",
            1:  "Turn on LED1 & LED3",
            2:  "Wait 3 frames",
            3:  "Cam1 & Cam2 update_leds [0]",
            4:  "Turn on LED2 & LED4",
            5:  "Wait 3 frames",
            6:  "Cam1 & Cam2 update_leds [1]",
            7:  "Turn on LED1 & LED5",
            8:  "Wait 3 frames",
            9:  "Cam1 update_leds [2], Cam3 [0]",
            10: "Turn on LED2 & LED6",
            11: "Wait 3 frames",
            12: "Cam1 update_leds [3], Cam3 [1]",
            13: "Turn on ALL LEDs",
            14: "Wait 3 frames",
            15: "Tracking mode: Cam1 [0,1,2,3], Cam2 [0,1], Cam3 [0,1]"
        }

    def state_elapsed(self) -> bool:
        """检查当前状态是否已持续指定帧数"""
        return (self.frame_counter - self.state_entry_frame) >= self.state_delay_frames

    def set_state(self, new_state):
        self.state = new_state
        self.state_entry_frame = self.frame_counter
        self.retry_counter = 0
        desc = self.state_desc.get(new_state, "")
        print(f"[STATE ] Frame {self.frame_counter:05d} | Enter state {new_state:2d} | {desc}")
        
    def update_and_check(self, cam_name, frame, leds_to_track, check_loss=False):
        tracker = self.trackers[cam_name]["tracker"]
        leds, detected_flags = tracker.update_leds(frame, leds_to_track=leds_to_track)

        if check_loss:
            ok = True
            for i in leds_to_track:
                if not detected_flags[i]:
                    self.miss_counter[i] += 1
                    if self.miss_counter[i] >= 30: 
                        ok = False
                else:
                    self.miss_counter[i] = 0
            return ok
        else:
            return all(detected_flags[i] for i in leds_to_track)

    def run(self):
        try:
            while True:
                frame = self.camera.get_frame()
                if frame is None:
                    continue

                h, w = frame.shape[:2]
                half_h, half_w = h // 2, w // 2
                sub_imgs = {
                    "Cam_1": frame[0:half_h, 0:half_w],
                    "Cam_2": frame[0:half_h, half_w:w],
                    "Cam_3": frame[half_h:h, 0:half_w],
                }
                self.frame_counter += 1

                # --- 状态机逻辑 (完整复制原来的) ---
                # 和你原始代码 state 0~15 部分完全一致
                # 我就不改动了
                # --------------------------------------------------
                if self.state == 0:
                    self.ctrl.control_led([0,0,0,0,0,0])
                    for t in self.trackers.values():
                        t["tracker"].last_leds = [(0,0)]*4
                    self.set_state(1)

                elif self.state == 1:
                    self.ctrl.control_led([1,0,1,0,0,0])  # LED1, LED3
                    self.set_state(2)

                elif self.state == 2:
                    if self.state_elapsed():
                        self.set_state(3)

                elif self.state == 3:
                    ok1 = self.update_and_check("Cam_1", sub_imgs["Cam_1"], [0])
                    ok2 = self.update_and_check("Cam_2", sub_imgs["Cam_2"], [0])
                    if ok1 and ok2:
                        self.set_state(4)
                    else:
                        self.retry_counter += 1
                        if self.retry_counter >= 60:
                            self.set_state(0)

                elif self.state == 4:
                    self.ctrl.control_led([0,1,0,1,0,0])  # LED2, LED4
                    self.set_state(5)

                elif self.state == 5:
                    if self.state_elapsed():
                        self.set_state(6)

                elif self.state == 6:
                    ok1 = self.update_and_check("Cam_1", sub_imgs["Cam_1"], [1])
                    ok2 = self.update_and_check("Cam_2", sub_imgs["Cam_2"], [1])
                    if ok1 and ok2:
                        self.set_state(7)
                    else:
                        self.retry_counter += 1
                        if self.retry_counter >= 60:
                            self.set_state(0)

                elif self.state == 7:
                    self.ctrl.control_led([1,0,0,0,1,0])  # LED1, LED5
                    self.set_state(8)

                elif self.state == 8:
                    if self.state_elapsed():
                        self.set_state(9)

                elif self.state == 9:
                    ok1 = self.update_and_check("Cam_1", sub_imgs["Cam_1"], [2])
                    ok3 = self.update_and_check("Cam_3", sub_imgs["Cam_3"], [0])
                    if ok1 and ok3:
                        self.set_state(10)
                    else:
                        self.retry_counter += 1
                        if self.retry_counter >= 60:
                            self.set_state(0)

                elif self.state == 10:
                    self.ctrl.control_led([0,1,0,0,0,1])  # LED2, LED6
                    self.set_state(11)

                elif self.state == 11:
                    if self.state_elapsed():
                        self.set_state(12)

                elif self.state == 12:
                    ok1 = self.update_and_check("Cam_1", sub_imgs["Cam_1"], [3])
                    ok3 = self.update_and_check("Cam_3", sub_imgs["Cam_3"], [1])
                    if ok1 and ok3:
                        self.set_state(13)
                    else:
                        self.retry_counter += 1
                        if self.retry_counter >= 60:
                            self.set_state(0)

                elif self.state == 13:
                    self.ctrl.control_led([1,1,1,1,1,1])
                    self.set_state(14)

                elif self.state == 14:
                    if self.state_elapsed():
                        self.set_state(15)

                elif self.state == 15:

                    # 打开所有 tracker 的 tracking
                    for t in self.trackers.values():
                        t["tracker"].tracking_enabled = True

                    ok1 = self.update_and_check("Cam_1", sub_imgs["Cam_1"], [0,1,2,3], check_loss=True)
                    ok2 = self.update_and_check("Cam_2", sub_imgs["Cam_2"], [0,1], check_loss=True)
                    ok3 = self.update_and_check("Cam_3", sub_imgs["Cam_3"], [0,1], check_loss=True)

                    leds_cam1 = self.trackers["Cam_1"]["tracker"].last_leds  # 4 LEDs
                    leds_cam2 = self.trackers["Cam_2"]["tracker"].last_leds[:2]  # 2 LEDs
                    leds_cam3 = self.trackers["Cam_3"]["tracker"].last_leds[:2]  # 2 LEDs

                    data = self.publisher.package(leds_cam1, leds_cam2, leds_cam3)
                    self.publisher.send(data)

                    if self.debug:
                        cam1 = " ".join(f"({x:4d},{y:4d})" for x,y in leds_cam1)
                        cam2 = " ".join(f"({x:4d},{y:4d})" for x,y in leds_cam2)
                        cam3 = " ".join(f"({x:4d},{y:4d})" for x,y in leds_cam3)
                        print(f"Frame {self.frame_counter:05d} | Cam_1: {cam1} | Cam_2: {cam2} | Cam_3: {cam3}")

                    if not (ok1 and ok2 and ok3):
                        print(
                            f"[WARN  ] Frame {self.frame_counter:05d} | LED lost!\n"
                            f"         Cam1: {ok1}\n"
                            f"         Cam2: {ok2}\n"
                            f"         Cam3: {ok3}"
                        )
                        self.set_state(0)
                        # 打开所有 tracker 的 tracking
                        for t in self.trackers.values():
                            t["tracker"].tracking_enabled = False

                # --- PC 上显示 ---
                if self.show_windows:
                    # 计算 FPS
                    self.frame_count += 1
                    elapsed = time.time() - self.start_time
                    if elapsed >= 1.0:
                        self.fps = self.frame_count / elapsed
                        self.frame_count = 0
                        self.start_time = time.time()
                    res_str = f"{half_w}x{half_h}"

                    # 按比例缩小分辨率
                    sub_imgs_resized = {}
                    for name, img in sub_imgs.items():
                        new_w = int(img.shape[1] * self.resize_scale)
                        new_h = int(img.shape[0] * self.resize_scale)
                        sub_imgs_resized[name] = cv2.resize(img, (new_w, new_h))

                    # 准备白底
                    blank = 255 * np.ones_like(sub_imgs_resized["Cam_1"])

                    # 拼接成 2x2 （右下角是白块）
                    top_row = np.hstack((sub_imgs_resized["Cam_1"], sub_imgs_resized["Cam_2"]))
                    bottom_row = np.hstack((sub_imgs_resized["Cam_3"], blank))
                    big_frame = np.vstack((top_row, bottom_row))

                    # 画 LED 标记
                    colors = [(0,0,255),(0,255,0),(255,0,0),(0,255,255)]
                    for name, tracker in self.trackers.items():
                        leds = tracker["tracker"].last_leds

                        # 偏移量（决定画在哪一块）
                        if name == "Cam_1":
                            offset_x, offset_y = 0, 0
                        elif name == "Cam_2":
                            offset_x, offset_y = sub_imgs_resized["Cam_1"].shape[1], 0
                        elif name == "Cam_3":
                            offset_x, offset_y = 0, sub_imgs_resized["Cam_1"].shape[0]
                        else:
                            continue

                        for i, pt in enumerate(leds):
                            if pt != (0,0):
                                x = int(pt[0] * self.resize_scale) + offset_x
                                y = int(pt[1] * self.resize_scale) + offset_y
                                cv2.circle(big_frame, (x, y), 3, colors[i], -1)
                                cv2.putText(big_frame, f"LED{i+1}", (x+5, y-5),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, colors[i], 1)

                    # 显示一个窗口
                    cv2.imshow("Tracking 2x2", big_frame)
                    cv2.setWindowTitle("Tracking 2x2", f"Tracking 2x2 | {res_str} | {self.fps:.1f} FPS")

                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
        except KeyboardInterrupt:
            print("Ctrl+C pressed, exiting...")
        except Exception as e:
            print("Error:", e)
        finally:
            self.camera.close()
            if self.show_windows:
                cv2.destroyAllWindows()
            self.ctrl.close()
