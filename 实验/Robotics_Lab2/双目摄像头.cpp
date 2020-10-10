#include <opencv2/opencv.hpp>
using namespace std;

int main()
{
    //初始化并分配内存给相机来的视频流  宽高比为320*240
    cv::VideoCapture camera0(1);
    camera0.set(CV_CAP_PROP_FRAME_WIDTH,320);
    camera0.set(CV_CAP_PROP_FRAME_HEIGHT,240);
    cv::VideoCapture camera1(0);
    camera1.set(CV_CAP_PROP_FRAME_WIDTH,320);
    camera1.set(CV_CAP_PROP_FRAME_HEIGHT,240);

    //当两个摄像头都打开时进入循环，否则退出
    if( !camera0.isOpened() ) return 1;
    if( !camera1.isOpened() ) return 1;

    while(true) {
        //抓取视频序列的每一帧
        cv::Mat3b frame0;
        camera0 >> frame0;
        cv::Mat3b frame1;
        camera1 >> frame1;

        cv::imshow("Video0", frame0);
        cv::imshow("Video1", frame1);
//      std::cout << frame1.rows() << std::endl;
        //wait for 40 milliseconds
        int c = cvWaitKey(40);

        //exit the loop if user press "Esc" key  (ASCII value of "Esc" is 27) 
        if(27 == char(c)) break;
    }

    return 0;
}
