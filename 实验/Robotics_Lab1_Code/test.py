#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2
import video

# 常量 生成颜色模板时需要用到
LOWER_BLUE = np.array([0., 60., 32.])
UPPER_BLUE = np.array([180., 255., 255.])
LOWER_GREEN = np.array([35., 43., 46.])
UPPER_GREEN = np.array([77., 255., 255.])
LOWER_RED = np.array([0, 43, 46])
UPPER_RED = np.array([10, 255, 255])


class App(object):
    def __init__(self, color):
        self.cam = video.create_capture(1)  # 捕获摄像头设备并创建一个对象
        self.frame = None
        cv2.namedWindow('camshift')  # cv2窗口对话框名称
        self.hist_roi = None
        self.selection = None
        self.tracking_state = 0
        self.hide_background = False  # 是否需要隐藏背景，默认显示

        if color == 'red':
            self.flag = 'red'
            self.roi = cv2.imread('red.jpg')  # 读取red.jpg作为region of interest
        elif color == 'green':
            self.flag = 'green'
            self.roi = cv2.imread('green.jpg')  # 读取green.jpg作为region of interest
        else:
            # detect blue by default
            self.flag = 'blue'
            self.roi = cv2.imread('blue.jpg')  # 读取blue.jpg作为region of interest

    def start(self):
        # 初始化状态参数
        self.selection = (0, 0, 640, 480)  # 选取该区域作为颜色识别检测区域
        self.tracking_state = 1  # 是否需要跟踪检测

    def get_mask(self, hsv_image, color='blue'):
        # 获得hsv_image对应颜色的蒙板
        if color not in ['blue', 'green', 'red']:
            return cv2.inRange(hsv_image, np.array([0., 0., 0.]), np.array([255., 255., 255.]))
        elif color == 'blue':
            return cv2.inRange(hsv_image, LOWER_BLUE, UPPER_BLUE)
        elif color == 'green':
            return cv2.inRange(hsv_image, LOWER_GREEN, UPPER_GREEN)
        elif color == 'red':
            return cv2.inRange(hsv_image, LOWER_RED, UPPER_RED)

    def show_hist(self):
        # 展示图片的直方图
        bin_count = self.hist_roi.shape[0]
        bin_w = 24
        img = np.zeros((256, bin_count * bin_w, 3), np.uint8)
        for i in range(bin_count):
            h = int(self.hist_roi[i])
            cv2.rectangle(img, (i * bin_w + 2, 255), ((i + 1) * bin_w - 2, 255 - h),
                          (int(180.0 * i / bin_count), 255, 255), -1)
        img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        cv2.imshow('hist', img)

    def run(self):
        roi = self.roi  # 获取ROI
        self.start()
        while True:
            ret, self.frame = self.cam.read()
            vis = self.frame.copy()
            hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)  # 将当前帧从RGB格式转换为HSV格式

            # 获得当前hsv图像的蒙板
            mask = self.get_mask(hsv, color=self.flag)

            if self.selection:
                x0, y0, x1, y1 = self.selection
                self.track_window = (x0, y0, x1, y1)  # 追踪子区域

                # 对ROI进行颜色格式转换和阈值限制
                hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                mask_roi = self.get_mask(hsv_roi, color=self.flag)
                # 绘制ROI图像的一维直方图
                hist_roi = cv2.calcHist([hsv_roi], [0], mask_roi, [16], [0, 180])
                # 对hist做归一化
                cv2.normalize(hist_roi, hist_roi, 0, 255, cv2.NORM_MINMAX)

                # 将hist向量reshape为1列并存入self.hist中
                self.hist_roi = hist_roi.reshape(-1)
                self.show_hist()
                # 可见区域
                vis_roi = vis[y0:y1, x0:x1]
                cv2.bitwise_not(vis_roi, vis_roi)  # 对每个像素进行二进制取反操作

                # 在vis中，置mask中为0的对应位置也为0
                vis[mask == 0] = 0

            if self.tracking_state == 1:
                self.selection = None  # 取消ROI模板

                prob = cv2.calcBackProject([hsv], [0], self.hist_roi, [0, 180], 1)  # 反向投影法
                prob &= mask  # 与mask进行与运算 得到所求颜色的直方图概率分布

                criteria_term = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)  # CamShift算法的停止条件

                # 运用CamShift算法对track_window内的图像进行prob检测
                track_box, self.track_window = cv2.CamShift(prob, self.track_window, criteria_term)

                if track_box[1][1] <= 1:
                    # 如果没有检测到 重置检测状态
                    self.start()
                else:
                    # 检测到目标颜色
                    if self.hide_background:
                        # 如果需要隐藏背景， 使用prob直方概率分布图替换vis图像
                        vis[:] = prob[..., np.newaxis]
                    try:
                        '''
                        track_box: [[center, axes], [angle, startAngle], endAngle]
                        '''
                        cv2.ellipse(vis, track_box, (0, 0, 255), 2)  # 在track_box内部绘制椭圆图像
                        print ('center: %.2f\taxes: %.2f\tangle: %.2f\tstart_angle: %.2f\tend_angle: %.2f') % (
                            track_box[0][0], track_box[0][1], track_box[1][0], track_box[1][1], track_box[2]
                        )
                    except:
                        print (track_box)

            cv2.imshow('camshift', vis)

            ch = 0xFF & cv2.waitKey(5)
            if ch == 27 or ch == ord('q'):
                # Press ESC or q to exit
                break
            if ch == ord('b'):
                # 输入b改变是否需要显示背景
                self.hide_background = not self.hide_background
            if ch == ord('r'):
                # 重新开始检测颜色
                self.start()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    import sys

    try:
        print (sys.argv[1])
        color = sys.argv[1]
    except IndexError:
        # 命令行参数未指定检测的颜色
        color = 'red'

    a = App(color)
    a.run()
