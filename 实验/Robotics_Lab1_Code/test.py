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
        elif color == 'hhh':
            self.flag = 'hhh'
            self.roi = cv2.imread('hhh.png')  # 读取hhh.jpg作为region of interest
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
        # 颜色空间转换
        img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        cv2.imshow('hist', img)

    def run(self):
        # 获取感兴趣的例图
        roi = self.roi  # 获取ROI
        # 开启检测
        self.start()
        # 在输入错误的情况下仍然可以继续循环
        while True:
            # 读取视频帧
            ret, self.frame = self.cam.read()
            vis = self.frame.copy()
            hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)  # 将当前帧从RGB格式转换为HSV格式

            # 获得当前视频帧hsv图像的掩膜
            """
               掩膜的概念：图像处理中，选定一块图像、图形或物体，对处理的图像（全部或局部）进行遮挡，来控制图像处理的区域或处理过程
               这里遮挡处理的图像是当前摄像头获取的视频帧
            """
             # 三个维度对应H、S、V，numpy数组设定了三个维度分别的范围，在mask区域里设为255，其它区域设为0，实际上变为黑白图；
             # 这里避免由于低光引起的错误值，使用cv.inRange()函数丢弃低光值；即选定的范围是掩模，是后续要进行处理的区域。
            mask = self.get_mask(hsv, color=self.flag)
            # 选定搜索区域后，设置当前窗口，开始检测。
            # 这时获得示例图的HSV图像和设置掩模(与视频帧的掩模相对应)。。。
            # [辅助处理] 为了增强图像的对比度，方便检测和追踪，
            # 此时绘制例图的H通道信息的一维直方图，并进行均衡化。
            # 若开启检测
            if self.selection:
                x0, y0, x1, y1 = self.selection
                self.track_window = (x0, y0, x1, y1)  # 追踪子区域

                # 对ROI进行颜色格式转换和阈值限制
                # 例图的HSV格式图像
                hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                # 例图的掩膜（与当前视频帧的掩膜对应）
                mask_roi = self.get_mask(hsv_roi, color=self.flag)
                # 绘制ROI图像的一维直方图
                hist_roi = cv2.calcHist([hsv_roi], [0], mask_roi, [16], [0, 180])
                # 对hist做归一化
                # 直方图均衡化，[0,255]表示均衡后的范围
                cv2.normalize(hist_roi, hist_roi, 0, 255, cv2.NORM_MINMAX)

                # 将hist向量reshape为1列并存入self.hist中
                self.hist_roi = hist_roi.reshape(-1)
                self.show_hist()
                # 可见区域
                vis_roi = vis[y0:y1, x0:x1]
                cv2.bitwise_not(vis_roi, vis_roi)  # 对每个像素进行二进制取反操作

                # 在vis中，置mask中为0的对应位置也为0
                vis[mask == 0] = 0
            # 如果检测到了图像中存在的感兴趣颜色区域
            if self.tracking_state == 1:
                # 追踪到对象之后，不用再设置搜索区域
                self.selection = None  # 取消ROI模板
                """
                   反向投影：重置视频帧的像素点的值后，最亮的即值最大的，在例图的灰度直方图中出现的个数最多，极有可能是要追踪的目标；
                            最后相当于得到一个概率图
                """
                # 区分颜色的是H（范围为0-180），以例图的直方图作反向投影，得到颜色概率分布图
                prob = cv2.calcBackProject([hsv], [0], self.hist_roi, [0, 180], 1)  # 反向投影法
                # 使要寻找的颜色区域更亮
                prob &= mask  # 与mask进行与运算 得到所求颜色的直方图概率分布
                # 迭代到10次或误差小于1，搜索结束
                criteria_term = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)  # CamShift算法的停止条件
                """
                   CamShift算法：对meanShift进行改进，可以自适应的调整椭圆的大小和角度
                                 追踪目标颜色质心
                """
                # 运用CamShift算法对track_window内的图像进行prob检测
                track_box, self.track_window = cv2.CamShift(prob, self.track_window, criteria_term)
                # 目标距离太远，重置算法，重新搜索
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
                        # 绘制红色的椭圆标记
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
