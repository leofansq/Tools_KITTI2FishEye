# FishEye

## 主程序
```
void main()
{
	Mat undistort_image = imread("test3.png");	
	Mat mapx, mapy, distort_image;

	resize(undistort_image, undistort_image, Size(undistort_image.cols * 2, undistort_image.rows * 2));
	imshow("undistort_image", undistort_image);
	
	RectifyMap(mapx,mapy,200);
	
	UndisImage(undistort_image, distort_image, mapx, mapy);
	imshow("distort_iamge", distort_image);
	imwrite("test_image.jpg",distort_image);
	waitKey();
}
```
* 放大图片(长和宽各乘2)，让原图映射到单位半球上的范围更大，进而让单位半球映射到鱼眼图上的面积更大，可减少黑边
```
resize(undistort_image, undistort_image, Size(undistort_image.cols * 2, undistort_image.rows * 2));
```
* 修改r来调整鱼眼图像圆半径至200，主要影响是改变了焦距f，改变了原图映射到单位半球上的范围，进而可调整黑边
```
RectifyMap(mapx,mapy,200);
```

## RectifyMap  计算从待矫正图像到畸变图像坐标的映射矩阵  
```
void RectifyMap(Mat& mapx, Mat& mapy, float r = 600);
```
```
void RectifyMap(Mat& mapx, Mat& mapy, float r)
{
	//设置变换矩阵的大小，之后的输出图像和变换矩阵同样大
	int width = 1000;            //映射图像的宽度
	float s = 480.0f / 720.0f;   //图像高和宽的比例
	
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
		float* px = (float*)(mapx.data + i * mapx.step);
		float* py = (float*)(mapy.data + i * mapy.step);
		for (int j = 0; j < width; ++j)
		{
			float x = j - center_x;
			float nx,ny;
			//图像变换
			if (sqrt(x*x + y*y) >= r)
			{
				nx = -1;
				ny = -1;
			}
			else
			{
				PointMap(x, y, nx, ny,r);
				px[j] = nx;
				py[j] = ny;
			}
		}
	}
}
```
* 用Mat类型的参数data和step确定第i行第一个像素的地址，其中Mat.data是图像第一行第一列像素的地址，Mat.step是图像一行所占空间
```
float* px = (float*)(mapx.data + i * mapx.step);
```
* 鱼眼图像上圆形区域外边不填充图像像素,圆形区域内调用PointMap利用等距模型将畸变图像上的点映射到单位球面。
```
if (sqrt(x*x + y*y) >= r)
{
	nx = -1;
	ny = -1;
}
else
{
	PointMap(x, y, nx, ny,r);
	px[j] = nx;
	py[j] = ny;
}
```
* mapx和mapy中为映射矩阵，其中元素（i，j）存储的为鱼眼图片横纵坐标为i和j的位置  在原图中对应的坐标

## PointMap 将一个畸变图像上的点用等距模型映射到单位球面
```
void PointMap(Point2f sp, Point2f &dp, float r);
```
```
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
```

## UndisImage 校正图像
```
void UndisImage(Mat distort_image, Mat& undistort_image, Mat mapx, Mat mapy);
```
```
void UndisImage(Mat undistort_image, Mat& distort_image, Mat mapx, Mat mapy)
{
	assert(mapx.rows == mapy.rows && mapy.cols == mapy.cols);
	//变换前图像高 宽&中心位置
	int height = undistort_image.rows;
	int width = undistort_image.cols;
	float cx = width / 2;
	float cy = height / 2;

	//变换后图像高 宽&中心位置
	int distort_height = mapx.rows;
	int distort_width = mapy.cols;
	float center_x = distort_width / 2;
	float center_y = distort_height / 2;
	
	//创建新的图像为校正后的图像
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
			//计算鱼眼图片像素点对应原图中的像素点
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
			//对应像素点填充
			for (int k = 0; k < channel; ++k) 
			{
				pdata[j * channel + k] = _undistrot_image(y,x)[k];
			}
		}
	}
	//cv::resize(distort_image,distort_image,undistort_image.size());
}
```
* 通过遍历待生成的鱼眼图片的每个像素点（i，j），利用mapx和mapy两个映射矩阵，得到其在原图中对应像素点。将原图中像素点的深度赋值给鱼眼图片对应像素点，从而实现鱼眼图片的填充，进而完成图片的转换。