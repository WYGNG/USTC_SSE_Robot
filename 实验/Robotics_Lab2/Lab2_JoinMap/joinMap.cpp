#include <iostream>
#include <fstream>
using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Geometry> 
#include <boost/format.hpp>  // for formating strings
#include <pcl/point_types.h> 
#include <pcl/io/pcd_io.h> 
#include <pcl/visualization/pcl_visualizer.h>

int main( int argc, char** argv )
{
    /*彩色图和灰度图各5张，所以用容器来存储*/
    vector<cv::Mat> colorImgs, depthImgs;

    /*vector有两个参数，后面的参数一般是默认的，
    这里用适合Eigen库的对齐方式来初始化容器，
    总共有5张图片 所以对应着5个位姿矩阵*/
    vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;         // 相机位姿
    
    //寻找当前目录下的pose.txt文件
    ifstream fin("./pose.txt");
    if (!fin)
    {
        cerr<<"请在有pose.txt的目录下运行此程序"<<endl;
        return 1;
    }
    
    /*循环读取图像*/
    for ( int i=0; i<5; i++ )
    {
        /*用boost中的format格式类，来循环读取图片，否则单张读取图片就会有问题
         * 当在命令行中执行的时候这里必须要为../  在当前ide中执行的时候要修改为./ */
        boost::format fmt( "./%s/%d.%s" ); //图像文件格式字符数字.拓展名 "../%s/%d.%s"   ../ 表示可执行文件在build中,图像在上一个目录,所以用../
        /*这里的%对应./ color对应%s  下面的符号就是与上面一致对应的 */
        colorImgs.push_back( cv::imread( (fmt%"color"%(i+1)%"png").str() ));
        depthImgs.push_back( cv::imread( (fmt%"depth"%(i+1)%"pgm").str(), -1 )); // 使用-1读取原始图像
        
        /*基于范围的for循环，表示从data数组的第一项开始 循环遍历  auto表示自动根据后面的元素 获得符合要求的类型*/
        //pose每行7个参数，前三个为相机坐标，后四个为四元数，虚数在前，实数在后
        double data[7] = {0};
        for ( auto& d:data ) //auto自动类型转换
            fin>>d; //文件流类型的变量fin将pose.txt中的数据给了d数组
        //四元数 data[6]是实数 但是coeffis输出的是先虚数后实数
        Eigen::Quaterniond q( data[6], data[3], data[4], data[5] );
        //变换矩阵初始化旋转部分，
        Eigen::Isometry3d T(q);
        //变换矩阵初始化平移向量部分
        T.pretranslate( Eigen::Vector3d( data[0], data[1], data[2] ));
        //存储变换矩阵到位姿数组中
        poses.push_back( T );
    }
    
    // 计算点云并拼接
    // 相机内参 
    double cx = 325.5;
    double cy = 253.5;
    double fx = 518.0;
    double fy = 519.0;
    double depthScale = 1000.0;
    
    cout<<"正在将图像转换为点云..."<<endl;
    
    // 定义点云使用的格式：这里用的是XYZRGB
    typedef pcl::PointXYZRGB PointT; 
    typedef pcl::PointCloud<PointT> PointCloud;
    
    // 新建一个点云
    /*Ptr是一个智能指针，返回一个PointCloud<PointT> 其中PointT是pcl::PointXYZRGB类型。它重载了->  返回了指向PointCloud<PointT>的指针
     *Ptr是下面类型 boost::shared_ptr<PointCloud<PointT> > */
    /*pointCloud 是一个智能指针类型的对象 具体可以参考http://blog.csdn.net/worldwindjp/article/details/18843087*/
    PointCloud::Ptr pointCloud( new PointCloud ); 

    /*将5张图片 像素坐标转换到相机坐标 之后转换到世界坐标存储到点云格式的变量中 for循环之后用pcl的相关函数将点云转换到pcl能够显示的格式*/
    for ( int i=0; i<5; i++ )
    {
        cout<<"转换图像中: "<<i+1<<endl; 
        cv::Mat color = colorImgs[i]; 
        cv::Mat depth = depthImgs[i];
        Eigen::Isometry3d T = poses[i];

        /*           插入部分
         * //color.at<cv::Vec3b>(471,537)[0] = 12;//修改图像上的对应像素位置的值
        //color.ptr<cv::Vec3b>(471)[537][0] = 12;//与上面的效果一样

        //测试像素的输出效果,这里无法通过cout<<color.at<cv::Vec3b>(471,537)[0] 这种方式来输出第一个通道的值,因为每个通道的像素占了8位而unsigned char
        // 表示ascii码 所以输出的时候不是正确的数字，可以通过下面的方式强制转化为int类型（或者用自带的类型转换方式进行显示转换），就可以看到内部的值了
        //需要注意的一点是 cout页无法输出char类型的变量的地址，也是需要强制转换成void *类型的指针才能正常输出char类型变量的地址信息。
        if(colorImgs[i].channels() == 3) {
             std::cout << "测试1结果 " << color.ptr<cv::Vec3b>(471)[537] << "正确的结果:  "
                       << (char) color.at<cv::Vec3b>(471, 537)[0] << std::endl;
             std::cout << depth.ptr<unsigned short>(471)[537] << std::endl;
             std::cout << colorImgs[i].at<cv::Vec3b>(471, 537) << std::endl;
        }
                     插入部分结束
         */

        /*对图像像素进行坐标转换，将图像的坐标通过内参矩阵K转换为相机坐标系下的坐标，之后通过外参矩阵T 转化为世界坐标系下的坐标*/
        for ( int v=0; v<color.rows; v++ )
            for ( int u=0; u<color.cols; u++ )
            {
/*通过用Mat中的ptr模板函数 返回一个unsigned short类型的指针。v表示行 根据内部计算返回data头指针 + 偏移量来计算v行的头指针
                 * 图像为单通道的   depth.ptr<unsigned short> ( v ) 来获取行指针*/
                unsigned int d = depth.ptr<unsigned short> ( v )[u]; // 深度值16位存储 一般color图像像素中每一个通道是8位

                 /*             单通道遍历图像的方式总结:
                 *  注意深度图像的像素占16位 与普通图片每个通道的像素为8位不同
                 * 1、同样是用上面的双层for循环，遍历图像 用at方式
                 * for ( int v=0; v<color.rows; v++ )
                 *      for ( int u=0; u<color.cols; u++ )
                 *          unsigned int d = depth.at<unsigned short >(v,u);
                 *
                 *
                 * 2、使用迭代器进行图像的遍历
                 * 不是基于for循环了
                 * cv::MatIterator_<unsigned short > begin,end;
                 * for( begin =depth.begin<unsigned short >(), end = depth.end<unsigned short >(); begin != end; ){}
                 *
                 * 3、使用指针的方式 如本实验的结果
                 * */

                //迭代器的参数是通道数，因为深度图是单通道的，每个像素的值是unsigned short，所以参数是unsigned short
                //begin代表像素的开始地方                
                if ( d==0 ) continue; // 为0表示没有测量到 然后继续进行for循环那么跳过这个像素继续执行 在后面形成点云时需要设置is_dense为假

                Eigen::Vector3d point; 
                point[2] = double(d)/depthScale; //对实际尺度的一个缩放
                point[0] = (u-cx)*point[2]/fx; //根据书上5.5式子---86页
                point[1] = (v-cy)*point[2]/fy; 
                Eigen::Vector3d pointWorld = T*point; //将相机坐标系转换为世界坐标系
                
                PointT p ;
                p.x = pointWorld[0]; //将世界坐标系下的坐标用pcl专门的点云格式存储起来
                p.y = pointWorld[1];
                p.z = pointWorld[2];
            
                /*  color.step 虽然是一个类，但是它内部有一个转换操作符 operator size_t() const;
                 * 此时的color.size编译器就会把它当做size_t类型的变量,这个值的大小是1920 这个是随着图像的读入MAT类中会有自动转换然后存储的buf[]中 */
                p.b = color.data[ v*color.step+u*color.channels() ];
                p.g = color.data[ v*color.step+u*color.channels()+1 ];
                p.r = color.data[ v*color.step+u*color.channels()+2 ];
                /*  -> 是智能指针重载的 然后返回的类型就是输入的类型 可以看上面Ptr的解释说明 */
                pointCloud->points.push_back( p );//存储格式好的点
            }
    }
    std::cout<<"点云的列和行为 ： "<<pointCloud->width<<" "<<pointCloud->height<<std::endl;
    //这里有可能深度图中某些像素没有深度信息，那么就是包含无效的像素，所以先置为假，但是如果设置成true的话 也没有看出来有什么不一样的地方
    pointCloud->is_dense = false;
    std::cout<<"点云的列和行为 ： "<<pointCloud->width<<" "<<pointCloud->height<<std::endl;

    cout<<"点云共有"<<pointCloud->size()<<"个点."<<endl;
    pcl::io::savePCDFileBinary("map.pcd", *pointCloud );//获取pointCloud指向的对象 这个就当做获取普通指针指向的对象来理解，这个对象是在定义的时候new出来的一段内存空间。
    return 0;

}
/*                                            备注： 3通道的图像的遍历方式总结
 * 对于单通道来说 每个像素占8位 3通道则是每个矩阵元素是一个Vec3b 即一个三维的向量 向量内部元素为8位数的unsigned char类型
 * 1、使用at遍历图像
 * for(v)row
 *  for(u)col
 *      image.at<Vec3b>（v,u）[0] 表示第一个通道的像素的值
 *      image.at<Vec3b>(v,u)[1]
 *      image.at<Vec3b>(v,u)[2]
 * 2、使用迭代器方式 (实际上就是一个指针指向了 cv::Mat矩阵元素)
 * cv::MatIterator_<Vec3b>begin,end;
 * for( begin = image.begin<Vec3b>(), end = image.end<Vec3b>() ; begin != end;  )
 *      (*begin)[0] = ...
 *      (*begin)[1] = ...
 *      (*begin)[2] = ...
 *
 * 3、用指针的方式操作
 * for(v)
 *  for(u)
 *      image.ptr<Vec3b>(v)[u][0] 表示第一个通道
 *      image.ptr<Vec3b>(v)[u][0] 表示第二通道
 *              .
 *              .
 *              .
 * */
