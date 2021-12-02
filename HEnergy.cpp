#include<opencv2/opencv.hpp>
#include<iostream>
#include<opencv2/imgproc.hpp>
#include<opencv2/highgui.hpp>
#include <algorithm>
using namespace std;
using namespace cv;
void findContours(Mat&src);
void time(Mat&src);
bool LeastSquaresCircleFitting(vector<Point2d> &m_Points, Point2d &Centroid, double &dRadius);
vector<Point2d> points;
int main()
{
 	VideoCapture cap;
	 cap.open("RH.avi");
    if (!cap.isOpened())
    {
        return -1;
    }
    Mat src;
    while (1)
    {cap >>src;
     if(src.empty())
     break;
     findContours(src);
     time(src);
    namedWindow("大符",WINDOW_NORMAL);
	  imshow("大符",src);
    waitKey(1);
    }
    return 0;
}
void findContours(Mat&src)
{ 
    vector<Mat>channels;
     split(src,channels);
     Mat src2=channels.at(2)-channels.at(0);//通道相减R-B
	Mat thresh;
     threshold(src2,thresh,100,255,THRESH_BINARY);
	Mat tchange;
  Mat kernel=getStructuringElement(MORPH_RECT,Size(7,7));
     morphologyEx(thresh,tchange,MORPH_CLOSE,kernel);
  //namedWindow("二值化",WINDOW_NORMAL);
  //imshow("二值化",tchange);
    vector<vector<Point>>contours; 
    vector<Vec4i>hierarchy;
      Point2i center; //用来存放找到的目标的中心坐标
     findContours(tchange,contours,hierarchy,RETR_TREE,CHAIN_APPROX_NONE,Point());
     int countour[20]={0};
     for (int i = 0; i < contours.size(); i++)
    {
            if (hierarchy[i][3] != -1) //有父轮廓
        {
            countour[hierarchy[i][3]]++; //对该父轮廓进行记录
        }
    }
  for (int j = 0; j < contours.size(); j++)
  {
    if (countour[j] == 1) //有一个子轮廓
     {
       RotatedRect box = minAreaRect(contours[hierarchy[j][2]]); //包含该轮廓
       Point2f vertex[4];
       box.points(vertex);//将左下角，左上角，右上角，右下角存入点集
      center = (vertex[0] + vertex[2]) / 2; //返回中心坐标
      for (int i = 0; i <3; i++)
        {
          line(src, vertex[i], vertex[i+1], Scalar(255, 0, 0), 4, LINE_AA); //画线
        }
        line(src,vertex[3],vertex[0],Scalar(255, 0, 0), 4, LINE_AA);
      putText(src, "target", center, FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 0),1);
     }
     if (int a =countour[j] == 3) 
     {
       
       vector<int>countour3;
       int counterarea[3];
       int temp[3];
       int index = 0;

       for(int k=0;k<contours.size();k++)
       {
         if(hierarchy[k][3]==j)
         {
          countour3.push_back(k);
         }

       }
       for(int l=0;l<3;l++)
       {
         int area = contourArea(contours[countour3[l]]);
         counterarea[l]=area;
         temp[l]=area;
       }
       sort(temp,temp+3);//面积排序
       for(int s=0;s<3;s++)
       {
         if(counterarea[s]==temp[0])
         {
           index = countour3[s];
         }
       }
          int num= index; 
          RotatedRect box= minAreaRect(contours[num]); 
          Point2f vertex[4];
          box.points(vertex);
          Point2f center = (vertex[0] + vertex[2]) / 2;
       for (int i = 0; i < 3; i++)
       { 
          line(src, vertex[i], vertex[i+1], Scalar(255, 0, 0), 4, LINE_AA); //画线
       }
        line(src,vertex[3],vertex[0],Scalar(255, 0, 0), 4, LINE_AA);
        circle(src,center,5,Scalar(0, 0, 255),-1);
        points.push_back(box.center);
		  	Point2d c;//圆心
			  double r = 0;//半径
			 LeastSquaresCircleFitting(points, c, r);//拟合圆
			 circle(src, c, r, Scalar(0, 0, 255), 2);//绘制圆
			 circle(src, c, 6, Scalar(255, 0, 0), -1);//绘制圆心
        putText(src, "destroyed", center, FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 255, 0));
     }
  } 
}
void time(Mat&src)
{
    double fps;
    double t = 0;
    char string[20];  // 用于存放帧率的字符串
    t = (double)getTickCount();
    t = (getTickCount()-t) / getTickFrequency();
    fps = 1/ t;
    sprintf(string, "%.2f", fps);      // 帧率保留两位小数
    std::string fpsString("FPS:");
    fpsString += string;  // 在"FPS:"后加入帧率数值字符串
  putText(src,fpsString,Point(5,50),FONT_HERSHEY_SIMPLEX,1,Scalar(255, 0, 0),1);
}
bool LeastSquaresCircleFitting(std::vector<cv::Point2d> &m_Points, cv::Point2d &Centroid, double &dRadius)
{
	if (!m_Points.empty())
	{
		int iNum = (int)m_Points.size();
		if (iNum < 3)	return 0;
		double X1 = 0.0;
		double Y1 = 0.0;
		double X2 = 0.0;
		double Y2 = 0.0;
		double X3 = 0.0;
		double Y3 = 0.0;
		double X1Y1 = 0.0;
		double X1Y2 = 0.0;
		double X2Y1 = 0.0;
		vector<Point2d>::iterator iter;
		vector<Point2d>::iterator end = m_Points.end();
		for (iter = m_Points.begin(); iter != end; ++iter)
		{
			X1 = X1 + (*iter).x;
			Y1 = Y1 + (*iter).y;
			X2 = X2 + (*iter).x * (*iter).x;
			Y2 = Y2 + (*iter).y * (*iter).y;
			X3 = X3 + (*iter).x * (*iter).x * (*iter).x;
			Y3 = Y3 + (*iter).y * (*iter).y * (*iter).y;
			X1Y1 = X1Y1 + (*iter).x * (*iter).y;
			X1Y2 = X1Y2 + (*iter).x * (*iter).y * (*iter).y;
			X2Y1 = X2Y1 + (*iter).x * (*iter).x * (*iter).y;
		}
		double C = 0.0;
		double D = 0.0;
		double E = 0.0;
		double G = 0.0;
		double H = 0.0;
		double a = 0.0;
		double b = 0.0;
		double c = 0.0;
		C = iNum * X2 - X1 * X1;
		D = iNum * X1Y1 - X1 * Y1;
		E = iNum * X3 + iNum * X1Y2 - (X2 + Y2) * X1;
		G = iNum * Y2 - Y1 * Y1;
		H = iNum * X2Y1 + iNum * Y3 - (X2 + Y2) * Y1;
		a = (H * D - E * G) / (C * G - D * D);
		b = (H * C - E * D) / (D * D - G * C);
		c = -(a * X1 + b * Y1 + X2 + Y2) / iNum;
		double A = 0.0;
		double B = 0.0;
		double R = 0.0;
		A = a / (-2);
		B = b / (-2);
		R = double(sqrt(a * a + b * b - 4 * c) / 2);
		Centroid.x = A;
		Centroid.y = B;
		dRadius = R;
		return 1;
	}
	else
		return 0;
	return 1;
}