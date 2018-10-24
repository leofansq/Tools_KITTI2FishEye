/************************************************************************
*	fisheye calibration
*	author ZYF
*	date 2014/11/22
************************************************************************/
//模型的参考论文附在文件夹中
//release模式运行会导致输出图像背景不为黑色（这些地方按理来说没有填充像素），debug模式无此问题

// FSQ更新部分：修正了UndisImage部分，完成了Label转换函数LabelTrans及其所调用的坐标转换函数LabelPoint
//				实现了图片转换和Label转换的批量处理main

#include "fisheye.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

//批量处理 fsq2018.2.4
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

using cv::Point2f;
using std::asin;
using namespace cv;
using namespace std;

#ifndef PI
#define PI (3.1415926)
#endif
/************************************************************************/
/* PointMap
/* 将一个鱼眼图像上的点用等距模型映射到单位球面
/************************************************************************/
void PointMap(Point2f sp, Point2f &dp, float r)
{
	PointMap(sp.x,sp.y,dp.x,dp.y,r);
}

/************************************************************************/
/* PointMap
/* 将一个鱼眼图像上的点用等距模型映射到单位球面
/* 参数：
/*		x,y: 输入参数，畸变图像上的点的坐标
/*		new_x, new_y : 输出参数，单位球面上点的坐标
		theta_max 鱼眼镜头视场角
		r  制作的鱼眼图像半径
/************************************************************************/
void PointMap(float x, float y, float& new_x, float& new_y,float r)//注释部分可以用于实现图像移位
{
	float l = sqrt(x*x + y*y);   //鱼眼图上某点距中心距离	
	float theta_max = PI/2 ;     //鱼眼镜头半视场角
	
	//float x0 = 0;原图移动像素量
	//float y0 = 0;
	//float theta_change = atan2(y0 , x0);
	//float l_change = sqrt(x0*x0 + y0*y0);

	//极坐标角alpha
	float alpha(0);
	if ( 0 == x) 
		alpha = PI / 2;
	else 
		alpha = atan2( y,x);

	float f = r / theta_max;    //用等距投影的方式计算焦距f
	float theta = l / f;        //鱼眼图映射到单位圆（theta = r/f，单位圆r=1）
	float d = f*tan(theta);     //单位圆映射到原图

	//float tx = d* cos(alpha)-l_change*cos(theta_change);
	//float ty = d* sin(alpha) - l_change*sin(theta_change);
	float tx = d* cos(alpha);
	float ty = d* sin(alpha);

	//new_x = tx;
	//new_y = ty;

	//这一段去掉会导致图像中线处有错位
	if ( x > 0)
		new_x = abs(tx);
	else if (x < 0)
		new_x = -abs(tx);
	else
		new_x = 0;

	if (y > 0)
		new_y = abs(ty);
	else if (y < 0)
		new_y = -abs(ty);
	else
		new_y = 0;
}

/************************************************************************/
/* PointMap2   未使用
/* 将一个矫正图像上的点映射到畸变图像上的一个点，使用纬度不变法映射
/* 参数：
/*		x,y: 输入参数，待矫正图像上的点的坐标
/*		new_x, new_y : 输出参数，畸变图像上点的坐标
/*		r : 输入参数，圆半径
/************************************************************************/
void PointMap2(float x, float y, float& new_x, float& new_y, float r)
{
	float theta_x = x / r;
	float xx = r * sin(theta_x);
	float theta_y = y / r;
	float yy = r * sin(theta_y);

	//迭代更新xx,yy
	float scale = 1.0f; // x,y坐标的缩放比例，默认为1，调整此参数会改变映射结果
	int iters = 0;//
	for (int i = 0; i < iters; ++i) {
		float rr = sqrt(r*r - yy*yy);
		float xx1 = rr * xx / r;
		rr = sqrt( r*r - xx*xx);
		float yy1 = rr * yy / r;
		xx = xx1; yy = yy1;
	}

	if (x == 0)
		new_x = 0;
	else
		new_x = (x > 0 ? 1 : -1) * abs(xx);

	if (y == 0)
		new_y = 0;
	else
		new_y = (y > 0 ? 1 : -1) * abs(yy);
}

/************************************************************************/
/* 生成从原图像到鱼眼图像的坐标的映射矩阵mapx mapy  
参数：
	r ： 圆半径，鱼眼图像半径
/************************************************************************/
void RectifyMap(Mat& mapx, Mat& mapy, float r)
{
	//int width = ceil(PI * r / 2) * 2;//ceil函数：取大于等于表达式的最小整数

	//这里设置了变换矩阵的大小，之后的输出图像和变换矩阵同样大
	//int width = 1000; //映射图像的宽度
	//float s = 480.0f / 720.0f; //图像高和宽的比例


	//尝试改变生成鱼眼图像长宽比2018.7.10  非批量处理main中也有参数改动
	int width = 1224; //映射图像的宽度
	float s = 480.0f / 720.0f; //图像高和宽的比例


	//计算图像的高 & 中心点x、y坐标
	int height = width * s;
	int center_x = width / 2, center_y = height / 2;
	//创建mapx mapy 均为32位浮点
	mapx.create(height,width,CV_32F);
	mapy.create(height,width,CV_32F);

	for (int i = 0; i < height; ++i) 
	{
		//像素确定
		float y = center_y - i;
		float* px = (float*)(mapx.data + i * mapx.step);//第i行第一个像素的地址
		float* py = (float*)(mapy.data + i * mapy.step);
		for (int j = 0; j < width; ++j) 
		{
			float x = j - center_x;
			float nx,ny;
			//鱼眼图像上圆形区域外边不填充图像像素
			//圆形区域内调用PointMap利用等距模型将畸变图像上的点映射到单位球面
			if (sqrt(x*x + y*y) >= r)//实际上这段if无用，可去掉
			{
				nx = -1;
				ny = -1;
			}
			else
			{
				PointMap(x, y, nx, ny,r);//
				//PointMap2(x, y, nx, ny, 300);//
				px[j] = nx;
				py[j] = ny;
			}
		}
	}
}

/************************************************************************/
/* 矫正图像   fsq2018.1.27                                     
/************************************************************************/
void UndisImage(Mat undistort_image, Mat& distort_image, Mat mapx, Mat mapy)
{
	assert(mapx.rows == mapy.rows && mapy.cols == mapy.cols);
	//变换前图像高 宽&中心位置
	int height = undistort_image.rows;
	int width = undistort_image.cols;
	float cx = width / 2;
	float cy = height / 2;
	//cout << width << endl << height << endl;
	//cx = 320; cy = 260;

	//变换后图像高 宽&中心位置
	int distort_height = mapx.rows;
	int distort_width = mapy.cols;
	float center_x = distort_width / 2;
	float center_y = distort_height / 2;

	distort_image.create(distort_height, distort_width, undistort_image.type());
	distort_image.setTo(0);
	int channel = distort_image.channels();
	cv::Mat_<cv::Vec3b> _undistrot_image = undistort_image;
	cv::Mat_<float> _mapx = mapx;
	cv::Mat_<float> _mapy = mapy;

	for (int i = 0; i < distort_height; ++i) 
	{
		uchar* pdata = distort_image.data + i * distort_image.step;
		//float* pmapx = (float*)(mapx.data + i * mapx.step);
		//float* pmapy = (float*)(mapy.data + i * mapy.step);
		for (int j = 0; j < distort_width; ++j) 
		{
// 			if ((i - center_y)*(i - center_y) + (j - center_x)*(j - center_x) > un_width * un_width / 4) {
// 				continue;
// 			}
			//int x = pmapx[j] + cx;
			//int y = cy - pmapy[j];
			int x = _mapx(i,j) + cx;
			int y = cy - _mapy(i,j);
			//鱼眼图圆形外不填充像素
			if (x - cx == -1 && cy - y == -1)
			{
				continue;
			}
			//若圆形区域内某位置，对应原图上超出范围，不填充像素，这是黑边的由来
			if ((x < 0 || x >= width || y < 0 || y >= height)) 
			{
				continue;
			}
			for (int k = 0; k < channel; ++k) 
			{
				pdata[j * channel + k] = _undistrot_image(y,x)[k];
			}
		}
	}
	//cv::resize(distort_image,distort_image,undistort_image.size());
}


/************************************************************************/
/*用于Label转换的坐标变换（PointMap函数的反函数）fsq2018.2.4
/*参数：
/*    x,y : 输入参数，需转换的原图坐标
/*    new_x,new_y : 输出值，转换后的鱼眼图片中相应的坐标
/*    r : 输入参数，鱼眼半径
/************************************************************************/
void LabelPoint(float x, float y, float& new_x, float& new_y, float r)
{
	float theta_max = PI / 2;	//鱼眼镜头半视场角
	float d = sqrt(x*x + y*y);

	float alpha(0);
	if (0 == x)
		alpha = PI / 2;
	else
		alpha = atan2(y, x);

	float f = r / theta_max;
	float l = f * atan2(d, f);
	new_x = l * cos(alpha);
	new_y = l * sin(alpha);
}


/************************************************************************/
/* Label转换 fsq2018.2.4
/* 参数：
/*     x1,x2,y1,y2 : 输入参数，原图Label中2D框左上、右下顶点坐标
/*     bbx1,bbx2,bby1,bby2 : 输出值，鱼眼图片中相应的2D框左上、右下顶点坐标
/************************************************************************/
void LabelTrans(float x1, float x2, float y1, float y2, float& bbx1, float& bbx2, float& bby1, float& bby2)
{
	float nx1, nx2, nx3, nx4, ny1, ny2, ny3, ny4;
	//float cx = 616, cy = 186, center_x = 500, center_y = 333;
	//尝试2018.7.10  该段用来替换上一句，以便在更改生成图片尺寸后相应改变label变换计算
	float cx = 616, cy = 186;	
	int width = 1000; //映射图像的宽度
	float s = 480.0f / 720.0f; //图像高和宽的比例
	int height = width * s;
	float center_x = width / 2, center_y = height / 2;

    //将原图坐标系原点移至图片中心，为对应图片处理部分的Resize，做*2处理
	x1 = 2*(x1 - cx);
	x2 = 2*(x2 - cx);
	y1 = 2*(cy - y1);
	y2 = 2*(cy - y2);
	//将原图2D框四个顶点坐标转换为鱼眼图片的相应坐标
	LabelPoint(x1, y1, nx1, ny1, 200);//左上
	nx1 = center_x + nx1;
	ny1 = center_y - ny1;
	//cout << nx1 << endl << ny1 << endl<<endl;
	
	LabelPoint(x2, y1, nx2, ny2, 200);//右上
	nx2 = center_x + nx2;
	ny2 = center_y - ny2;
	//cout << nx2 << endl << ny2 << endl << endl;

	LabelPoint(x1, y2, nx3, ny3, 200);//左下
	nx3 = center_x + nx3;
	ny3 = center_y - ny3;
	//cout << nx3 << endl << ny3 << endl << endl;	

	LabelPoint(x2, y2, nx4, ny4, 200);//右下
	nx4 = center_x + nx4;
	ny4 = center_y - ny4;
	//cout << nx4 << endl << ny4 << endl << endl;

	// 比较坐标值，重新确定鱼眼图片的2D矩形框
	bbx1 = min(nx1, nx3);
	bby1 = min(ny1, ny2);
	bbx2 = max(nx4, nx2);
	bby2 = max(ny4, ny3);
}

//非批量处理 main

void main()
{
	Mat undistort_image = imread("000145.png");	
	Mat mapx, mapy, distort_image;
	//imshow("origin_image", undistort_image);

	//放大图片，让原图映射到单位半球上的范围更大，进而让单位半球映射到鱼眼图上的面积更大，同样可减少黑边
	resize(undistort_image, undistort_image, Size(undistort_image.cols * 2, undistort_image.rows * 2));
	//imshow("resized_image", undistort_image);

	//修改r来调整鱼眼图像圆半径，主要影响是改变了焦距f，改变了原图映射到单位半球上的范围，进而可调整黑边
	RectifyMap(mapx, mapy, 200);

	//尝试改变生成鱼眼图像长宽比2018.7.10 RectifyMap中也有参数改动
	RectifyMap(mapx,mapy,800);
	//imshow("mapx", mapx);
	//imshow("mapy", mapy);
	UndisImage(undistort_image, distort_image, mapx, mapy);
	imshow("distort_iamge", distort_image);
	imwrite("000145.jpg",distort_image);
	waitKey();

}


//批量处理main fsq2018.2.4
/*
int main()
{
	Mat undistort_image;
	Mat mapx, mapy, distort_image;	
	string fileName,fileName_label;
	//图片列表文档&输入输出地址
	char* filePath = "E:\\FishEye\\dir.txt";//批量处理文件名储存文档地址
	char* dir_in = "F:\\KITTI\\training\\image_2\\";//批量处理输入文件地址
	char* dir_out = "E:\\FishEye_out\\image_2\\";//批量处理文件输出地址
	//Label列表文档&输入输出地址
	char* filePath_label = "E:\\FishEye\\dir_label.txt";
	char* dir_label_in = "F:\\KITTI\\training\\label_2\\";
	char* dir_label_out = "E:\\FishEye_out\\label_2\\";

	//图片转换
	
	RectifyMap(mapx, mapy, 200);//构建图片位置映射矩阵

	int counts = 0;//输出显示计数
	cout << "Pic Trans" << endl;

	ifstream inFile(filePath);
	if (!inFile.is_open())
	{
		cerr << "Failed open the file" << endl;
		return -1;
	}
	while (getline(inFile, fileName)) //按行读取文件名
	{
		string str_in = dir_in + fileName;
		string str_out = dir_out + fileName;
		//待转换图片读入
		undistort_image = imread(str_in, 1);
		//图片转换
		resize(undistort_image, undistort_image, Size(undistort_image.cols * 2, undistort_image.rows * 2));
		UndisImage(undistort_image, distort_image, mapx, mapy);
		//鱼眼图片保存 & 计数显示
		imwrite(str_out, distort_image);
		counts += 1;
		cout << counts<<endl;
	}
	inFile.close();		
	

	//Label转换
	cout << "Label Trans" << endl;
	int counts_label = 0;

	ifstream inFile_label(filePath_label);
	if (!inFile_label.is_open())
	{
		cerr << "Failed open the file" << endl;
		return -1;
	}
	while (getline(inFile_label, fileName_label)) //按行读取文件名
	{
		string str_label_in = dir_label_in + fileName_label;
		string str_label_out = dir_label_out + fileName_label;
		//输入&输出label文件
		ifstream inLabel(str_label_in);
		ofstream outLabel(str_label_out);

		string label;
		string str_type, str_truncated, str_occluded, str_alph, str_bbx1, str_bby1, str_bbx2, str_bby2;
		//逐行读取输入文件数据
		//提取所需数据：类别、2D框左上&右下顶点坐标
		//将坐标转换生成鱼眼图像2D框坐标，生成新label文件
		while (getline(inLabel, label))
		{
			istringstream is(label);//按空格分割数据，str_type类别，str_bbx1 str_bbx2 str_bby1 str_bby2为2D框左上和右下顶点
			is>> str_type >> str_truncated >> str_occluded >> str_alph >> str_bbx1 >> str_bby1 >> str_bbx2 >> str_bby2;

			float x1 = atof(str_bbx1.c_str());//数据类型转换string --> float
			float x2 = atof(str_bbx2.c_str());
			float y1 = atof(str_bby1.c_str());
			float y2 = atof(str_bby2.c_str());
			
			float bbx1, bbx2, bby1, bby2;
			LabelTrans(x1, x2, y1, y2, bbx1, bbx2, bby1, bby2);
			
			outLabel << str_type << " " <<str_truncated<<" "<<str_occluded<<" "<< str_alph<<" "<< bbx1 << " " << bby1 << " " << bbx2 << " " << bby2 << endl;
		}
		inLabel.close();
		outLabel.close();
		counts_label += 1;
		cout << counts_label << endl;
	}
	inFile_label.close();
	waitKey(0);
}
*/