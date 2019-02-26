#include <ros/ros.h>
#include <math.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/core/core.hpp> 
#include<opencv2/imgproc/imgproc.hpp>  
#include "iostream"
#include<opencvtest/contours.h>
#include<opencvtest/img_pro_info.h>
#include <typeinfo>
//static const std::string OPENCV_WINDOW = "Image window";
using namespace std;
using namespace cv;
long int t_last = 0,t_now = 0;
long double t_during = 0.0;
int red_offset_add ;
int red_offset_sub ;
int red_offset_line ;
int alpha_slider_add;
int alpha_slider_sub;
int alpha_slider_line;
//按照轮廓周长排序
// static inline bool LinesSortFun(vector<cv::Point> contour1,vector<cv::Point> contour2)  
// {  
//     return (contour1.size() > contour2.size());  
// }  
//按照轮廓面积排序  
static inline bool ContoursSortFun(vector<cv::Point> contour1,vector<cv::Point> contour2)  
{  
    return (cv::contourArea(contour1) > cv::contourArea(contour2));  
}
struct img_pro_info
{
    bool out_flag;
    int width;
    int height;
    int x_pos;
    int y_pos;
};

static inline bool x_order(Point contours1,Point contours2)
{
    return(contours1.x < contours2.x);
}

// void Point_order(const vector<vector<Point> > &contours_ploy)
// {
//     int size = contours_ploy.size();
//     std::sort(contours_ploy[0].begin(),contours_ploy[0].end(),x_order); 
// }
void get_center(vector<vector<Point> > poly,Point &P_center)
{
    int P_center_x = 0;
    int P_center_y = 0;
    float P_in_x,P_in_y,P_out_x,P_out_y;
    for(int i = 0;i < 2;i++)
    {
        P_center_x = P_center_y = 0;
        for(int j = 0;j < poly[i].size();j++)
        {
            P_center_x = poly[i][j].x + P_center_x;
            P_center_y = poly[i][j].y + P_center_y;
        }
        if(i == 0)
        {
            P_out_x = P_center_x*1.0/poly[i].size();
            P_out_y = P_center_y*1.0/poly[i].size();
            //cout<<"P_out_x = "<<P_out_x<<endl;
            //cout<<"P_out_y = "<<P_out_y<<endl;
        }
        else
        {
            //_in_x = P_center_x*1.0/poly[i].size();
            //P_in_y = P_center_y*1.0/poly[i].size();
            //cout<<"P_in_x = "<<P_in_x<<endl;
            //cout<<"P_in_y = "<<P_in_y<<endl;
        }
    }
    P_center.x = round(P_out_x);
    P_center.y = round(P_out_y);
}
class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher contours_pub_ ;


    
public:
    ImageConverter()
    : it_(nh_)
    {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,&ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);
        //contours_pub_ = it_.advertise("/contours_topic", 1);
        contours_pub_ = nh_.advertise<opencvtest::img_pro_info>("/contours_topic", 50);
        //cv::namedWindow(OPENCV_WINDOW);
    }
 
    ~ImageConverter()
    {
        //cv::destroyWindow(OPENCV_WINDOW);
    }
    
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
        }
        // 矩阵访问
        //如果是采用Mat形式存储，想要访问灰度图像的灰度值，可以采用如下方法：
        //int value = img.at<uchar>(i,j)[k];//k表示通道数


        // 画圈
        // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
        // {
        //     cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
        // }

        //载入图片
        //cv::Mat img = imread("momo3.jpeg",1);
        //变量定义
        cv::Mat img_gauss,img_gray,img_hsv,img_binary,img_red,img_red_gauss,img_red_median;
        //白色的点，用于赋值
        // CvScalar white_point;
        // white_point.val[0] = 255;
        // white_point.val[1] = 255;
        // white_point.val[2] = 255;
        cv::Mat img = cv_ptr->image;
        img_red = Mat::zeros(img.size(),CV_8UC1);
        int red_hsv = 120;
	    cv::GaussianBlur(img,img_gauss,cv::Size(3,3),0);
        cvtColor(img_gauss,img_hsv,CV_RGB2HSV);
        cvtColor(img_hsv,img_gray,CV_RGB2GRAY);
        //求取灰度均值
        // cv:Scalar gray_Val = cv::mean(img_gray);
        // float gray_Mean_0 = gray_Val.val[0];
        // std::cout<<"gray mean value = "<<gray_Mean_0<<std::endl;
        //某一点赋值
        // img.at<Vec3b>(200,100)[0] = 120;
        // img.at<Vec3b>(200,100)[1] = 255;
        // img.at<Vec3b>(200,100)[2] = 240;
        // Vec3i hsv_point = img_hsv.at<Vec3b>(122,100);
        // cout<<"hsv0 = "<<hsv_point.val[0]<<endl;
        // cout<<"hsv1 = "<<hsv_point.val[1]<<endl;
        // cout<<"hsv2 = "<<hsv_point.val[2]<<endl;
        //Vec3i rgb_point = img.at<Vec3b>(200,100);
        
        //红色区域筛选
        for (int i = 0; i < img_hsv.rows; i++)
        {
            for (int j = 0; j < img_hsv.cols; j++)
		    {
			    //CvScalar hsv_point = cvGet2D(img_hsv, i, j);//获取像素点为（j, i）点的HSV的值 
                Vec3i hsv_point = img_hsv.at<Vec3b>(i,j);
                if ((hsv_point.val[0]>red_hsv - 7)&&(hsv_point.val[0]<red_hsv + 7))
                {
                    //cvSet2D(img_red, i ,j, white_point);
                    img_red.at<uchar>(i,j) = 255;
                }
            }           
        }
        //二值化处理
        //int thresh = 120;
        //int maxValue = 250;
        //threshold(img_gray,img_binary, thresh, maxValue, THRESH_BINARY);
        Mat canny_img;
	    cv::Canny(img_gray,canny_img,120,250);
        //imshow("CannyImg", canny_img);
        //存储轮廓
        vector<vector<Point> > contours;
	    vector<Vec4i> hierarchy;
        //中值滤波
        cv::medianBlur(img_red,img_red_median,11);
        //高斯滤波
        //cv::GaussianBlur(img_red_gauss,img_red_median,cv::Size(3,3),0);
        //查找轮廓
        findContours(img_red_median,contours,hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE,Point());
	    Mat imageContours = Mat::zeros(img.size(),CV_8UC1);
        Mat poly_image = Mat::zeros(img.size(),CV_8UC1); //凸包图像
	    Mat Contours = Mat::zeros(img.size(),CV_8UC1);  //绘制
        //轮廓排序
        std::sort(contours.begin(),contours.end(),ContoursSortFun); 
        // //定义逼近后的存储容器
        vector<vector<Point> > contours_ploy(contours.size());
        //vector<Rect> rects_ploy(contours.size());
        //vector<RotatedRect> RotatedRect_ploy;//注意：由于下面赋值的过程中有个点数大于5的条件，所以这里没有直接初始化，才有下面pushback的方法添加值。
        //vector<RotatedRect> minRect( contours_ploy.size() );
        // //将结果放到各自的容器中
        for (size_t i = 0; i< contours.size(); i++)
        {
            //将轮廓做多边形逼近
            approxPolyDP(contours[i], contours_ploy[i],15, true);//第三个变量表征逼近的程度
            //printf("%s\n",typeid(contours[0]).name());//类型
            //cout<<"类型为"<<typeid(contours[0]).name()<<endl;

            //查找最小的矩形包围框
            //minRect[i] = minAreaRect( Mat(contours_ploy[i]) );
            //rects_ploy[i] = boundingRect(contours_ploy[i]);    
            // if (contours_ploy[i].size() >5)
            // {
            //     RotatedRect temp1 = minAreaRect(contours_ploy[i]);
            //     RotatedRect_ploy.push_back(temp1);
            //     // 椭圆
            //     // RotatedRect temp2 = fitEllipse(contours_ploy[i]);
            //     // ellipse_ploy.push_back(temp2);
            // }
        }


        //用于判断轮廓是否超出范围
        int out_flag = 0;
        //用于判断是否需要标记中心点
        int circle_flag = 0;
        //是否有可以成功发布的对象
        int pub_flag = 0;
        //通过相似度查找符合条件的轮廓
        //cout<<"第一种相似判断，结果两个轮廓之间的相似度为：  "<<matchShapes(contours_ploy[0],contours_ploy[0],CV_CONTOURS_MATCH_I1, 0)<<endl;
        //cout<<"第二种相似判断，结果两个轮廓之间的相似度为：  "<<matchShapes(contours_ploy[0],contours_ploy[1],CV_CONTOURS_MATCH_I2, 0)<<endl;
        //cout<<"第三种相似判断，结果两个轮廓之间的相似度为：  "<<matchShapes(contours_ploy[0],contours_ploy[1],CV_CONTOURS_MATCH_I3, 0)<<endl;
        //总的中心点
        Point P_center;
        //临时中心点
        Point P_center_0;

        //保存面积最大的轮廓信息
        vector<Point> contours_max(4);
        //轮廓的四个角点
        opencvtest::img_pro_info info_pub;
        //float down_flag = (89.0*42.0*0.7)/(73.0*24.0);
        //float up_flag = (89.0*42.0*1.5)/(73.0*24.0);
        //int similar_flag = 0;
        for(int i = 0;i < contours_ploy.size();i++)
        {
            //出界标志位置0 
            out_flag = 0; 
            if(contours_ploy[i].size() == 4  && cv::contourArea(contours_ploy[i]) > 500 && contours[i].size() > 30)
            //if(contours_ploy[i].size() == 4)
            {
                // for(int m = 0;m < i; m++)
                // {
                //     if(contourArea(contours_ploy[m])/cv::contourArea(contours_ploy[i]) > down_flag && contourArea(contours_ploy[m])/cv::contourArea(contours_ploy[i]) < up_flag)
                //     {
                //         similar_flag = 1;
                //     }
                // }
                // if(similar_flag == 1)
                // {
                //     continue;
                // }
                //std::cout<<"矩形角点 = "<<contours_ploy[i].size()<<endl;
                for(int j = 0;j < contours_ploy[i].size();j++)
                {
                    if( (contours_ploy[i][j].x < 30 || contours_ploy[i][j].x > 610)  || (contours_ploy[i][j].y < 20 || contours_ploy[i][j].y > 460) )
                    {
                        out_flag = 1; 
                    }
                }
                //零阶原点距和一阶原点距
                vector<Moments> mu(1); 
                vector<Point2f> mc(1);
                mu[0] = moments( contours_ploy[i], true ); 
                mc[0] = Point2d( mu[0].m10/mu[0].m00 , mu[0].m01/mu[0].m00 );
                P_center = Point(mc[0].x,mc[0].y);
                if(circle_flag == 0)
                {
                    P_center = Point(mc[0].x,mc[0].y);
                    circle_flag = 1;
                }
                //计算方向
                double a = mu[0].m20 / mu[0].m00 - mc[0].x*mc[0].x;
                double b = mu[0].m11 / mu[0].m00 - mc[0].x*mc[0].y;
                double c = mu[0].m02 / mu[0].m00 - mc[0].y*mc[0].y;
                double theta = fastAtan2(2*b,(a - c))/2;//此为形状的方向
                //cout<<"角度 = "<<theta<<endl;
                info_pub.out_flag = out_flag;
                info_pub.x_pos = P_center.x;
                info_pub.y_pos = P_center.y;
                //在图像上显示
                if(out_flag == 0 && (theta < 20 || theta > 160))
                {
                    //drawContours(imageContours,contours_ploy,i,Scalar(255),1,8,hierarchy);
                    //drawContours(img,contours_ploy,i,Scalar(0,255,0),3,8,hierarchy);
                    //cv::circle(imageContours, P_center, 5, Scalar(255),2);
                    cv::circle(img, P_center, 5, Scalar(0,255,0),2);
                }
                if(out_flag == 1 && (theta < 20 || theta > 160))
                {
                    //drawContours(imageContours,contours_ploy,i,Scalar(255),1,8,hierarchy);
                    //drawContours(img,contours_ploy,i,Scalar(255,0,0),3,8,hierarchy);
                    //cv::circle(imageContours, P_center, 5, Scalar(255),2);
                    cv::circle(img, P_center, 5, Scalar(255,0,0),2);
                }
                //轮廓排序
                std::sort(contours_ploy[0].begin(),contours_ploy[0].end(),x_order);
                // cout<<"x1 = "<<contours_ploy[0][0].x<<endl;
                // cout<<"x2 = "<<contours_ploy[0][1].x<<endl;
                // cout<<"x3 = "<<contours_ploy[0][2].x<<endl;
                // cout<<"x4 = "<<contours_ploy[0][3].x<<endl;
                //cout<<endl;
                if(contours_ploy[0][0].y < contours_ploy[0][1].y)
                {
                    contours_max[0] = Point(contours_ploy[0][0].x,contours_ploy[0][0].y);
                    contours_max[1] = Point(contours_ploy[0][1].x,contours_ploy[0][1].y);
                }
                else
                {
                    contours_max[0] = Point(contours_ploy[0][1].x,contours_ploy[0][1].y);
                    contours_max[1] = Point(contours_ploy[0][0].x,contours_ploy[0][0].y);
                }
                if(contours_ploy[0][2].y < contours_ploy[0][3].y)
                {
                    contours_max[2] = Point(contours_ploy[0][2].x,contours_ploy[0][2].y);
                    contours_max[3] = Point(contours_ploy[0][3].x,contours_ploy[0][3].y);
                }
                else
                {
                    contours_max[2] = Point(contours_ploy[0][3].x,contours_ploy[0][3].y);
                    contours_max[3] = Point(contours_ploy[0][2].x,contours_ploy[0][2].y);
                }
                info_pub.width = floor(0.5*(contours_max[3].x+contours_max[2].x-contours_max[1].x-contours_max[0].x));
                info_pub.height = floor(0.5*(contours_max[3].y+contours_max[1].y-contours_max[2].y-contours_max[0].y));
                //cout<<"width = "<<info_pub.width<<"   height = "<<info_pub.height <<endl;
                // Output modified video stream
                contours_pub_.publish(info_pub);
                pub_flag = 1;
                break;
                //contours[i]代表的是第i个轮廓，contours[i].size()代表的是第i个轮廓上所有的像素点数
                //绘制出点
                //for(int j=0;j<contours[i].size();j++) 
                //{
                    //绘制出contours向量内所有的像素点
                    //Point P=Point(contours[i][j].x,contours[i][j].y);
                    //Contours.at<uchar>(P)=255;
                //}
                //输出hierarchy向量内容
                //char ch[256];
                //sprintf(ch,"%d",i);
                //string str=ch;
                //cout<<"向量hierarchy的第" <<str<<" 个元素内容为："<<endl<<hierarchy[i]<<endl<<endl;
                //绘制轮廓
                //cout<<"size = "<<contours.size()<<endl;
                //绘制凸包的图像
                //cv::circle(poly_image, Point(300,300), 10, Scalar(255),2);
                //drawContours(poly_image,poly,i,Scalar(255),1,8,hierarchy);
            } 
        }
        if(pub_flag == 0 )
        {
            info_pub.out_flag = 1;
            info_pub.x_pos = 0;
            info_pub.y_pos = 0;
            info_pub.width = 0;
            info_pub.height = 0;
        	contours_pub_.publish(info_pub);
        }
        // if(contours_ploy.size() >= 3)
        // {
        //     for(int m = 0;m < contours_ploy[0].size();m++)
        //     {
        //         cout << "x = " << contours_ploy[0][m].x << "   y = " << contours_ploy[0][m].y << endl;
        //     }
        //     cout<<endl;
        // }
        //查找竖直方向上的直线
        // vector<Vec4i> Lines;
        // float x_mean = 0;
        // float y_mean = 0;
        // //Point vertical_center;
        // int num_vertical = 0;
        // cv::HoughLinesP(imageContours, Lines, 1, CV_PI/180, red_offset_line,20,8);
        // vector<Point2d> Lines_vertical(Lines.size()*2);
        // for (int i = 0; i < Lines.size(); i++)
	    // {
        //     if(abs(Lines[i][1] - Lines[i][3]) > 10 && atan(abs(Lines[i][1] - Lines[i][3])*1.0/abs(Lines[i][0] - Lines[i][2])) > M_PI*3/8)
        //     {
        //         Lines_vertical[num_vertical] = Point2d(Lines[i][0],Lines[i][1]);
        //         num_vertical++;
        //         Lines_vertical[num_vertical] = Point2d(Lines[i][2],Lines[i][3]);
        //         num_vertical++;
        //         //line(img, Point(Lines[i][0], Lines[i][1]), Point(Lines[i][2], Lines[i][3]), Scalar(0, 255, 0), 4, 8);
        //     }
	    // }
        //vertical_center = Point2d(floor(x_mean/num_vertical),floor(y_mean/num_vertical));

        //cout<<"直线条数 = "<<Lines.size()<<endl;     
        // //定义最终绘图的图片
        //Mat draw_rotateRect(img.size(), img.type(), Scalar::all(0));
        // //绘图圆形、矩形
        //RNG rng(12345);
        // //绘图椭圆形、旋转矩形
        // Point2f pot[4];
        // for (size_t i = 0; i<RotatedRect_ploy.size(); i++)
        // {
        //     Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));
        //     RotatedRect_ploy[i].points(pot);
        //     for(int j=0; j<4; j++)
        //     {
        //         line(draw_rotateRect, pot[j], pot[(j+1)%4], color);
        //     }
        // }


        //imshow("rotateRect",draw_rotateRect);
        //Update GUI Window
        //imshow("Contours Image",imageContours); //轮廓
        //imshow("gray Image",img_gray);
        //imshow("hsv Image",img_hsv);
        //imshow("red Image",img_red);//hsv提取到的红色区域
        //imshow("red Image_gauss",img_red_median);//高斯滤波后的红色区域
        //imshow("binary Image",img_binary);
        //imshow("poly Image",poly_image); //凸包的图像
        //imshow("draw_rotateRect", draw_rotateRect); 
	    //imshow("Point of Contours",Contours);   //向量contours内保存的所有轮廓点集
        cv::imshow("OPENCV_WINDOW_color", img);
        t_now = cv::getTickCount();
        t_during = (t_now-t_last)/cv::getTickFrequency();
        //std::cout <<"Time of image = "<< t_during<<endl;
        t_last = t_now;
        cv::waitKey(3);

    }
     
};
//调整阈值时使用
void on_trackbar_add( int, void* )
{
    red_offset_add = alpha_slider_add;
}
void on_trackbar_sub( int, void* )
{
    red_offset_sub = alpha_slider_sub;
} 
void on_trackbar_line( int, void* )
{
    red_offset_line = alpha_slider_line;
}    
int main(int argc, char** argv)
{
ros::init(argc, argv, "image_converter");
ImageConverter ic;
int alpha_slider_line_max = 100;
int alpha_slider_max = 50;
// namedWindow("Linear Offset_add", 1);
// namedWindow("Linear Offset_sub", 1);
//namedWindow("Linear Offset_line", 1);
// createTrackbar( "Trackbar", "Linear Offset_add", &alpha_slider_add, alpha_slider_max, on_trackbar_add );
// createTrackbar( "Trackbar", "Linear Offset_sub", &alpha_slider_sub, alpha_slider_max, on_trackbar_sub );
// createTrackbar( "Trackbar", "Linear Offset_line", &alpha_slider_line, alpha_slider_line_max, on_trackbar_line );
ros::spin();
return 0;
}

