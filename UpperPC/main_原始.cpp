#include <opencv2/opencv.hpp>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <vector>
#include <string>
#include <fcntl.h>
#include <cstring>
#include <algorithm> // 包含 std::clamp
#include <cmath>

#define ubuntu (1)
#if ubuntu
#include <termios.h>
#include <unistd.h>
#endif

using namespace cv;
using namespace std;

string frame_path = "/home/dgdz/桌面/Test_Image/yellow.png";
string video_path = "D:/1vscode/930/8.mp4"; // 5 ,6     8
int Tracking(Mat dilated_image);
void Tracking_barrel(Mat dilated_image);
void vertebral_model(Mat &img_);
void Later_vertebral_model();

//////////////////////////////////////////////////////////////////////////////////////////
// 全局变量声明
float Track_width = 120;

float m_left = 0;
float m_right = 0;
float a_right = 0;
float a_left = 0;
float x_left = 0;
float x_right = 0;

float m_mid_blue = 0;
float b_mid_blue = 0;

int8_t Trackingflag = 1;

// 串口
int8_t Head = 0XA5;
int8_t Tail = 0X5A;
int8_t Length = 3;

int My_threshold = 0;
int My_HoughLinesP = 50;

int Min_moderate = 95;	   // 70
int Min_barrel_area = 130; // 130

int brightness_threshold = 20;

int8_t flag_cross = 0; // 斑马线标志位
int8_t Bias_Line;	   // 中线偏差
int8_t R_L_Flag;	   // 左右变向标志位//1为左，2为右
int8_t R_L_Flag_Temp;  // 左右变向标志位//1为左，2为右
int Bias_Line_int = 0; // 中线偏差

cv::Mat img;					// 源图像
cv::Mat img_xiaoxi;				// 带有各种信息的图片
cv::Mat img_ROI;				// 带有各种信息的图片
cv::Mat binaryImage;			// 图像处理后的二值化图片
cv::Mat binaryImage_Finish;		// 最终提取赛道白线的图片
cv::Mat binaryImage12;			// 图像处理后的二值化图片
uint8_t is_First_cross = 0;		// 首次识别人行道标志//增加至1时表示确定找到人行道
uint8_t barrel_find_finish = 0; // 是否继续寻找锥桶标志
uint8_t barrel_find_yes = 0;	// 锥桶识别标志位

uint8_t Stop_Turn_Cross = 0;
uint8_t Find_Turn_Flag = 0;

int Left_barrel_serial_number = 1;
int Mid_barrel_serial_number = 2;
int Right_barrel_serial_number = 3;
////////////////////////////////////////////////////
// 边线模块


std::vector<cv::Point> left_line;
std::vector<cv::Point> right_line;
std::vector<cv::Point> mid;
std::vector<cv::Point> left_line_blue;
std::vector<cv::Point> right_line_blue;
std::vector<cv::Point> mid_blue;
/////////////////////////////////////////////////////

/////////////////////////////////////////////////////

Mat defineTrapezoidROI(Mat src)
{
	// 创建一个黑色的空白遮罩，大小与输入图像相同
	Mat mask = Mat::zeros(src.size(), src.type());

	// 定义感性区域的六个顶点（你之前定义的形状）
	int height = src.rows;
	int width = src.cols;

	// Point top_left(0, 180);	   // 左下角
	// Point top_right(320, 180); // 右下角

	// Point top_middle_left(100, 30);	 // 左中上角
	// Point top_middle_right(220, 30); // 右中上角

	// Point bottom_right(320, 240); // 右底部
	// Point bottom_left(0, 240);	  // 左底部

	Point top_left(0, 210);	   // 左下角
	Point top_right(320, 210); // 右下角

	Point top_middle_left(100, 30);	 // 左中上角
	Point top_middle_right(160, 30); // 右中上角

	Point bottom_right(320, 240); // 右底部
	Point bottom_left(0, 240);	  // 左底部

	// 将六个点存储在一个数组中，作为多边形顶点
	vector<Point> trapezoid = {top_left, top_middle_left, top_middle_right, top_right, bottom_right, bottom_left};

	// 在遮罩上绘制梯形区域
	vector<vector<Point>> contours;
	contours.push_back(trapezoid);

	// 填充多边形区域，白色代表感兴趣区域
	fillPoly(mask, contours, Scalar(255, 255, 255));

	// 将遮罩应用于原图像，只保留梯形区域
	Mat maskedImage;
	bitwise_and(src, mask, maskedImage);

	// 绘制梯形轮廓，方便显示感兴趣区域的边界
	polylines(src, contours, true, Scalar(255, 255, 255), 1); // 绿色边界线，线宽为3

	return maskedImage;
}

Mat ImagePreprocessing_Sobel_LinePro(Mat frame_a)
{
	int width = frame_a.cols;
	int height = frame_a.rows;

	// 空白图像，用于存储最终的处理结果
	binaryImage_Finish = cv::Mat::zeros(height, width, CV_8U);
	cv::Mat binaryImage_Finish3 = cv::Mat::zeros(height, width, CV_8U); // 保底处理图像
	cv::Mat binaryImage_Finish4 = cv::Mat::zeros(height, width, CV_8U); // 处理图像1
	cv::Mat binaryImage_Finish5 = cv::Mat::zeros(height, width, CV_8U); // 处理图像2

	Mat binaryImage12;
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 进行伽马变换
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Mat normalized_image;
	// frame_a.convertTo(normalized_image, CV_32F, 1.0 / 255, 0);  // 归一化
	//// 自适应伽马变换
	// Mat gamma_corrected;
	// double meanBrightness = cv::mean(frame_a)[0];  // 获取图像平均亮度
	// double gamma = 1.0;  // 根据亮度动态调整gamma值
	// pow(normalized_image, gamma, gamma_corrected);
	// gamma_corrected.convertTo(gamma_corrected, CV_8U, 255, 0);  // 将像素值重新缩放回0-255

	Mat originalImage;
	cvtColor(frame_a, originalImage, cv::COLOR_BGR2GRAY); // 转换为灰度图

	// 高斯模糊
	Mat enhancedImage = originalImage;
	GaussianBlur(enhancedImage, enhancedImage, Size(5, 5), 2); // 高斯滤波

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 中值滤波
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 使用中值滤波去除椒盐噪声
	medianBlur(enhancedImage, enhancedImage, 5); // 中值滤波，3x3的窗口大小，尝试不同的窗口大小
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 开运算（形态学操作）
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	Mat kernel1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
	cv::Mat morphedImage;
	morphologyEx(enhancedImage, morphedImage, MORPH_OPEN, kernel1); // 开运算

	Mat kernel2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));

	// 膨胀操作
	cv::dilate(morphedImage, morphedImage, kernel1, cv::Point(-1, -1), 1);

	medianBlur(morphedImage, morphedImage, 5); // 中值滤波，3x3的窗口大小，尝试不同的窗口大小

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Sobel边缘检测
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	Mat sobelx, sobely;
	Sobel(morphedImage, sobelx, CV_64F, 1, 0, 3); // Sobel X方向
	Sobel(morphedImage, sobely, CV_64F, 0, 1, 3); // Sobel Y方向
	convertScaleAbs(sobelx, sobelx);			  // 统一为正值
	convertScaleAbs(sobely, sobely);			  // 统一为正值

	Mat gradientMagnitude;
	addWeighted(sobelx, 0.5, sobely, 0.5, 0, gradientMagnitude); // 合并梯度

	// 二值化操作
	binaryImage = Mat::zeros(morphedImage.size(), CV_8U);
	cv::threshold(gradientMagnitude, binaryImage, My_threshold, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
	kernel2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	cv::dilate(binaryImage, binaryImage, kernel2, cv::Point(-1, -1), 1);

	// 显示图像
	// imshow("Binary Image After Threshold", binaryImage);

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 霍夫变换线段检测
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	vector<Vec4i> lines; // 定义线段变量
	int minLineLength = 20;
	int maxLineGap = 10;

	// 定义感兴趣区域（ROI）
	int x_roi = 1;
	int y_roi = 100;
	int width_roi = 319;
	int height_roi = 140;
	Rect roi(x_roi, y_roi, width_roi, height_roi);
	Mat croppedObject = binaryImage(roi); // 提取ROI区域进行霍夫变换

	// 霍夫变换检测线段
	HoughLinesP(croppedObject, lines, 1, CV_PI / 180, My_HoughLinesP, minLineLength, maxLineGap);

	// 绘制检测到的线段
	for (size_t i = 0; i < lines.size(); i++)
	{
		Vec4i l = lines[i];

		float k = static_cast<float>(l[3] - l[1]) / (l[2] - l[0]);
		float angle_k = std::atan(k) * 180.0 / CV_PI;
		float b = l[1] - k * l[0];
		float x6 = (10 - b) / k;
		float length = sqrt(pow(l[3] - l[1], 2) + pow(l[2] - l[0], 2));

		// 绘制符合条件的线段
		if (abs(angle_k) > 30 && length > 30)
		{
			// if ((l[1] < 8 || l[3] < 8) && (l[1] > 30 || l[3] > 30))
			//{
			l[0] += x_roi;
			l[1] += y_roi;
			l[2] += x_roi;
			l[3] += y_roi;
			line(binaryImage_Finish3, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 2, LINE_AA);
			l[0] -= x_roi;
			l[1] -= y_roi;
			l[2] -= x_roi;
			l[3] -= y_roi;
			/*}*/

			if (abs(angle_k) < 80 && length > 30 && (l[1] > 55 || l[3] > 55) && (l[1] < 15 || l[3] < 15) && ((l[0] + l[2] < 319) ? (k < 0 && abs(angle_k - a_left) < 18 && abs(115 - x_right + x6) < 10) : (k > 0 && abs(angle_k - a_right) < 18 && abs(115 + x_left - x6) < 10)))
			{
				l[0] += x_roi;
				l[1] += y_roi;
				l[2] += x_roi;
				l[3] += y_roi;
				line(binaryImage_Finish4, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 2, LINE_AA);
				l[0] -= x_roi;
				l[1] -= y_roi;
				l[2] -= x_roi;
				l[3] -= y_roi;
			}

			if (abs(angle_k) < 85 && length > 30 && (l[1] > 65 || l[3] > 65) && (l[1] < 8 || l[3] < 8) && ((l[0] < 160 + 20 || l[2] < 160 + 20) ? k < 0 : 1) && ((l[0] > 160 - 20 || l[2] > 160 - 20) ? k > 0 : 1))
			{
				l[0] += x_roi;
				l[1] += y_roi;
				l[2] += x_roi;
				l[3] += y_roi;
				line(binaryImage_Finish5, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 2, LINE_AA);
				l[0] -= x_roi;
				l[1] -= y_roi;
				l[2] -= x_roi;
				l[3] -= y_roi;
			}
		}
	}

	// imshow("111", binaryImage_Finish4);
	// imshow("222", binaryImage_Finish5);
	// imshow("333", binaryImage_Finish3);
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 跟踪处理
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
Tracking_start:
	switch (Trackingflag)
	{
	case 0:
		if (Tracking(binaryImage_Finish4))
		{
			binaryImage_Finish = binaryImage_Finish4;
			return binaryImage_Finish;
		}
		else
		{
			Trackingflag = 1;
			goto Tracking_start; // 跳转到标签
		}
		break;

	case 1:
		if (Tracking(binaryImage_Finish5))
		{
			Trackingflag = 0;
			binaryImage_Finish = binaryImage_Finish5;
			return binaryImage_Finish;
		}
		else
		{
			Trackingflag = 2;
			goto Tracking_start; // 跳转到标签
		}
		break;

	case 2:
		Tracking(binaryImage_Finish3);
		binaryImage_Finish = binaryImage_Finish3;
		Trackingflag = 0;
		return binaryImage_Finish;
		break;
	}

	return binaryImage_Finish;
}

void Sobel_LinePro(Mat frame_a)
{
	int width = frame_a.cols;

	int height = frame_a.rows;

	// 空白图像;用于存储最终的霍夫线段图
	cv::Mat binaryImage_Finish3 = cv::Mat::zeros(height, width, CV_8U); // 保底处理图像

	////霍夫线段检测
	vector<Vec4i> lines; // 定义线段变量
	int minLineLength = 30;
	int maxLineGap = 10;

	// 定义感兴趣区域 (ROI)
	int x_roi = 50;
	int y_roi = 60;
	int width_roi = 200;
	int height_roi = 50;
	Rect roi(x_roi, y_roi, width_roi, height_roi);
	Mat croppedObject = binaryImage(roi); // 对 ROI 进行霍夫线段检测
	// imshow("666", croppedObject);
	// 检测线段
	HoughLinesP(croppedObject, lines, 1, CV_PI / 180, My_HoughLinesP, minLineLength, maxLineGap);

	// 绘制检测到的线段
	for (size_t i = 0; i < lines.size(); i++)
	{
		Vec4i l = lines[i];

		float k = static_cast<float>(l[3] - l[1]) / (l[2] - l[0]);
		float angle_k = std::atan(k) * 180.0 / CV_PI;
		float b = l[1] - k * l[0];
		float x6 = (10 - b) / k;
		float length = sqrt(pow(l[3] - l[1], 2) + pow(l[2] - l[0], 2));

		if (abs(angle_k) > 30 && length > 25)
		{

			if ((l[1] < 4 || l[3] < 4) && (l[1] > 15 || l[3] > 15))
			{
				l[0] += x_roi;
				l[1] += y_roi;
				l[2] += x_roi;
				l[3] += y_roi;
				line(binaryImage_Finish3, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 2, LINE_AA);
				l[0] -= x_roi;
				l[1] -= y_roi;
				l[2] -= x_roi;
				l[3] -= y_roi;
			}
		}
	}

	// imshow("333", binaryImage_Finish3);

	int begin = 160;		 // 图像中间位置
	left_line_blue.clear();	 // 左边线清除
	right_line_blue.clear(); // 右边线清除
	mid_blue.clear();		 // 中线清除

	for (int i = 99; i >= 61; i--)
	{
		int find_l = 0;
		int find_r = 0;
		int to_left = begin;
		int to_right = begin;

		// 查找左边线
		while (to_left > 1)
		{
			if (binaryImage_Finish3.at<uchar>(i, to_left) == 255) // 只需检测当前像素是否为白色
			{
				find_l = 1;
				left_line_blue.emplace_back(to_left, i);
				break;
			}
			to_left--;
		}

		// 查找右边线
		while (to_right < 319)
		{
			if (binaryImage_Finish3.at<uchar>(i, to_right) == 255) // 只需检测当前像素是否为白色
			{
				find_r = 1;
				right_line_blue.emplace_back(to_right, i);
				break;
			}
			to_right++;
		}

		if (!left_line_blue.empty() && !right_line_blue.empty())
		{
			cv::Point midx1 = left_line_blue.back();
			cv::Point midx2 = right_line_blue.back();
			mid_blue.emplace_back((midx1.x + midx2.x) / 2, i);
			begin = (to_right + to_left) / 2;
		}
	}

	cv::Vec4f mid_fit;

	if (left_line_blue.empty() || right_line_blue.empty())
	{
		return; // 或采取其他错误处理方式
	}

	cv::fitLine(mid_blue, mid_fit, cv::DIST_L2, 0, 0.01, 0.01);

	m_mid_blue = mid_fit[1] / mid_fit[0];
	b_mid_blue = mid_fit[3] - m_mid_blue * mid_fit[2];

	return;
}

int crossroad_num = 0;
Rect roi_rect;
int crossroad(Mat frame)
{
	// 初始化标志位
	int height = frame.rows;
	int width = frame.cols;

	// 将图像转换为HSV颜色空间
	Mat hsv;
	cvtColor(frame, hsv, COLOR_BGR2HSV);

	// 定义白色的HSV范围
	Scalar lower_white = Scalar(0, 0, 221);
	Scalar upper_white = Scalar(180, 100, 255);

	// 创建一个白色掩码
	Mat mask1;
	inRange(hsv, lower_white, upper_white, mask1);

	Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
	// morphologyEx(mask1, mask1, MORPH_CLOSE, kernel);  // 闭操作
	morphologyEx(mask1, mask1, MORPH_OPEN, kernel); // 开操作
	// imshow("mask1", mask1);  // 显示处理后的掩码

	if (crossroad_num == 0)
	{
		roi_rect = Rect(1, 100, 319, 60);
	}
	else
	{
		roi_rect = Rect(1, 210, 319, 30);
		flag_cross = 0;
	}

	Mat src(mask1, roi_rect);

	// imshow("ROI with Rectangle", src);  // 显示带有矩形框的原始图像

	// 计数器初始化
	int cout1 = 0, cout2 = 0, flag = 0;
	for (int i = 0; i < src.rows; i++)
	{
		if (cout1 < 10)
		{
			flag = 0;
		}
		cout1 = 0;
		for (int j = 10; j < src.cols - 10; j++)
		{
			// 遍历一行，记录斑马线条数
			if (src.at<char>(i, j - 2) == 0 && src.at<uchar>(i, j) == 0 && src.at<uchar>(i, j - 1) == 0 && src.at<uchar>(i, j + 1) == 255 && src.at<uchar>(i, j + 2) == 255)
			{
				cout1++;
			}
			else if (src.at<uchar>(i, j - 2) == 255 && src.at<uchar>(i, j) == 255 && src.at<uchar>(i, j - 1) == 255 && src.at<uchar>(i, j + 1) == 0 && src.at<uchar>(i, j + 2) == 0)
			{
				cout1++;
			}
			// 斑马线条数大于阈值，计有效一次
			if (cout1 >= 10)
			{
				cout2++;
				flag++;
				if (flag >= 5)
				{
					if (crossroad_num == 0)
					{
						flag_cross = -1;
						// crossroad_num = 1;
					}
					else
					{
						is_First_cross = 1;
						flag_cross = 1;
					}
				}
				break;
			}
		}
		if (flag_cross == 1 || flag_cross == -1)
		{
			// 提前退出循环
			break;
		}
	}
	static int flag_cross_num = 0;
	if (flag_cross == -1)
	{
		flag_cross_num++;
		if (flag_cross_num % 3 == 0)
		{
			crossroad_num = 1;
		}
	}
	return flag_cross; // 返回斑马线检测结果
}

// 全局变量或成员变量//锥桶
#define LEFT 1	 // 左锥桶
#define RIGHT 2	 // 右锥桶
#define MIDDLE 3 // 中锥桶

vector<Point> previousCenters;
size_t maxHistory = 5; // 设定记录的帧数

bool cleverFlag = 1;

int Frame_zhen = 60;
int detectCount = 0;	  // 连续检测到合理轮廓的帧数
int missingCount = 0;	  // 轮廓消失后的帧数
bool barrelFound = false; // 是否检测到合理轮廓的标志
int Found_barrel = 0;	  // 已经找到的锥桶的数量
uint8_t try_patching_line = 0;
int x_image_bottom = 0;
int x_image_top = 0;
int Later_vertebral_Flag = 0;
Point barrel_center_Temp;
float slope_vertebral = 0;
int First_Find_vertebral_Flag = 0;
Point Temp_vertebral_Point;
int offset = 0;
int off = 0;

// 锥桶检测调试统计
static int cone_debug_frame_count = 0;
static int cone_detected_count = 0;
static int cone_missed_count = 0;

// 定义直线结构体

// 定义结构体
struct Line
{
	float slope;		  // 直线的斜率
	cv::Point startPoint; // 直线的起点

	Line(float s, cv::Point start)
		: slope(s), startPoint(start)
	{
	}
} Left_Line(0, cv::Point(0, 0)), Right_Line(0, cv::Point(0, 0)); // 初始化左右两条线

void blue_vertebral_barrel_Judge(Mat frame) // 找到蓝色锥桶的中心
{
	// 矩形高度
	int height_roi = 80;

	if (x_right < 150 || 170 < x_left || x_right - x_left <= 0)
	{
		return;
	}

	Rect roi(x_left, 110 - height_roi, x_right - x_left, height_roi);

	if (roi.area() == 0)
	{
		return;
	}

	Mat roi_image = frame(roi);

	Sobel_LinePro(binaryImage);
	imshow("roi_image", roi_image);

	Mat hsvImage;
	cvtColor(roi_image, hsvImage, COLOR_BGR2HSV);

	Scalar lower_blue = Scalar(97, 160, 60);   // 蓝色的低阈值
	Scalar upper_blue = Scalar(151, 255, 255); // 蓝色的高阈值

	// Scalar lower_blue = Scalar(89, 100, 210);  // 蓝色的低阈值
	// Scalar upper_blue= Scalar(111, 255, 255);  // 蓝色的高阈值

	Scalar lower_white = Scalar(0, 0, 200);	   // 白色的低阈值
	Scalar upper_white = Scalar(180, 30, 255); // 白色的高阈值

	Mat blueMask, whiteMask;
	inRange(hsvImage, lower_blue, upper_blue, blueMask);	// 蓝色部分掩膜
	inRange(hsvImage, lower_white, upper_white, whiteMask); // 白色部分掩膜

	Mat filteredMask;
	bitwise_not(whiteMask, whiteMask);				// 反转白色掩膜
	bitwise_and(blueMask, whiteMask, filteredMask); // 只保留蓝色部分

	imshow("filteredMask", filteredMask);

	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(filteredMask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	// 对轮廓按照面积进行排序，找出最大的轮廓
	sort(contours.begin(), contours.end(), [](const vector<Point> &a, const vector<Point> &b)
		 { return contourArea(a) > contourArea(b); });

	bool foundValidContour = false; // 用于判断是否找到合理轮廓
	int leftWhitePixels = 0;
	int rightWhitePixels = 0;
	if (!contours.empty())
	{
		const vector<Point> &largestContour = contours[0];
		double maxArea = contourArea(largestContour);
		// printf("are:%f,barrelFound:%d\n", maxArea, barrelFound);
		//  如果轮廓面积合理
		if (maxArea >= Min_barrel_area)
		{
			// 找到合理轮廓，设置标志
			foundValidContour = true;
		}
	}

	if (foundValidContour)
	{
		detectCount++;

		if (detectCount >= 5)
		{
			flag_cross = 3;
			missingCount = 0;	// 重置轮廓丢失计数器
			barrelFound = true; // 设置找到轮廓标志位
		}
	}
}

Point blue_vertebral_barrel_find(Mat frame) // 找到蓝色锥桶的中心
{
	Point barrel_center = Point(0, 0);

	Mat hsvImage;
	cvtColor(frame, hsvImage, COLOR_BGR2HSV);

	// Scalar lower_blue = Scalar(89, 100, 210);  // 蓝色的低阈值
	// Scalar upper_blue = Scalar(111, 255, 255); // 蓝色的高阈值
	Scalar lower_blue = Scalar(100, 160, 30);
	Scalar upper_blue = Scalar(151, 255, 255);
	// Scalar lower_blue = Scalar(95, 160, 60);   // 蓝色的低阈值
	// Scalar upper_blue = Scalar(151, 255, 255); // 蓝色的高阈值

	Scalar lower_white = Scalar(0, 0, 200);	   // 白色的低阈值
	Scalar upper_white = Scalar(180, 30, 255); // 白色的高阈值

	Mat blueMask, whiteMask;
	inRange(hsvImage, lower_blue, upper_blue, blueMask);	// 蓝色部分掩膜
	inRange(hsvImage, lower_white, upper_white, whiteMask); // 白色部分掩膜

	Mat filteredMask;
	bitwise_not(whiteMask, whiteMask);				// 反转白色掩膜
	bitwise_and(blueMask, whiteMask, filteredMask); // 只保留蓝色部分

	imshow("filteredMask", filteredMask);
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(filteredMask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	// 锥桶检测调试信息 - 轮廓统计
	cone_debug_frame_count++;
	printf("[CONE DEBUG] 帧%d: 检测到%d个轮廓\n", cone_debug_frame_count, contours.size());

	// 对轮廓按照面积进行排序，找出最大的轮廓
	sort(contours.begin(), contours.end(), [](const vector<Point> &a, const vector<Point> &b)
		 { return contourArea(a) > contourArea(b); });

	bool foundValidContour = false; // 用于判断是否找到合理轮廓

	if (!contours.empty())
	{
		const vector<Point> &largestContour = contours[0];
		double maxArea = contourArea(largestContour);
		
		// 调试输出轮廓面积信息
		printf("[CONE DEBUG] 最大轮廓面积: %.2f, 阈值: %d\n", maxArea, Min_barrel_area);
		
		//  如果轮廓面积合理
		if (maxArea >= Min_barrel_area)
		{
			Moments m = moments(largestContour);
			int cx = int(m.m10 / m.m00);
			int cy = int(m.m01 / m.m00);
			Point currentCenter(cx, cy);

			previousCenters.push_back(currentCenter);
			if (previousCenters.size() > maxHistory)
			{
				previousCenters.erase(previousCenters.begin());
			}

			// 找到合理轮廓，设置标志
			foundValidContour = true;
			printf("[CONE DEBUG] ✓ 找到有效锥桶! 位置: (%d, %d)\n", cx, cy);
			cone_detected_count++;
		}
		else
		{
			printf("[CONE DEBUG] ✗ 轮廓面积不足: %.2f < %d\n", maxArea, Min_barrel_area);
			cone_missed_count++;
		}

		if (maxArea >= Min_moderate && !foundValidContour)
		{
			flag_cross = 3;
			printf("[CONE DEBUG] 设置flag_cross=3 (中等面积锥桶)\n");
		}
		else if (maxArea < Min_moderate && flag_cross == 3 && !foundValidContour && !Later_vertebral_Flag)
		{
			flag_cross = 0;
			printf("[CONE DEBUG] 重置flag_cross=0 (锥桶消失)\n");
		}
	}
	else
	{
		printf("[CONE DEBUG] 未检测到任何轮廓\n");
		cone_missed_count++;
	}

	if (foundValidContour)
	{
		detectCount++;
		printf("[CONE DEBUG] 连续检测计数: %d/5\n", detectCount);

		if (detectCount >= 5)
		{
			missingCount = 0;	// 重置轮廓丢失计数器
			barrelFound = true; // 设置找到轮廓标志位
			printf("[CONE DEBUG] ★ 确认锥桶存在! barrelFound=true\n");
		}
	}
	else if (barrelFound)
	{
		missingCount++;
		printf("[CONE DEBUG] 丢失计数: %d/5\n", missingCount);

		if (missingCount >= 5)
		{
			barrelFound = false; // 重置找到轮廓标志位
			Found_barrel++;
			detectCount = 0; // 重置找到轮廓的帧数计数器
			printf("[CONE DEBUG] ★ 锥桶消失! 已找到锥桶总数: %d\n", Found_barrel);
		}
	}

	if (!previousCenters.empty() && barrelFound)
	{
		for (const Point &center : previousCenters)
		{
			barrel_center.x += center.x;
			barrel_center.y += center.y;
		}
		barrel_center.x /= previousCenters.size();
		barrel_center.y /= previousCenters.size();
		printf("[CONE DEBUG] 平滑后锥桶中心: (%d, %d), 历史帧数: %zu\n", 
		       barrel_center.x, barrel_center.y, previousCenters.size());
	}

	// 每10帧输出一次统计信息
	if (cone_debug_frame_count % 10 == 0)
	{
		float detection_rate = (cone_detected_count * 100.0f) / cone_debug_frame_count;
		printf("[CONE STATS] 帧数: %d, 检测率: %.1f%%, 检测/丢失: %d/%d\n", 
		       cone_debug_frame_count, detection_rate, cone_detected_count, cone_missed_count);
	}

	return barrel_center;
}

float Limit(int now, int last, int limit)
{

	if ((now - last) >= limit)
		return (last + limit);
	else if ((now - last) <= -limit)
		return (last - limit);
	else
		return now;

	return last;
}

void Later_vertebral_model()
{
	static int Num_Zhen = 0;
	Num_Zhen++;
	
	printf("[LATER CONE] 后锥桶处理模式, 帧计数: %d/%d, 当前偏移: %d\n", 
	       Num_Zhen, Frame_zhen, offset);

	if (Num_Zhen >= Frame_zhen)
	{
		if (flag_cross == 2)
		{
			off = (try_patching_line == LEFT) ? Track_width + 20 : -Track_width - 20;
			printf("[LATER CONE] 中间锥桶目标偏移: %d\n", off);
		}
		else
		{
			off = (try_patching_line == LEFT) ? Track_width : -Track_width;
			printf("[LATER CONE] 普通锥桶目标偏移: %d\n", off);
		}

		int old_offset = offset;
		offset = Limit(off, offset, 1);
		printf("[LATER CONE] 偏移量调整: %d -> %d\n", old_offset, offset);
	}

	Point center_offset;

	if (try_patching_line == LEFT)
	{
		slope_vertebral = -m_right;
		center_offset = Point(x_right - offset, 110);

		circle(img, center_offset, 3, Scalar(0, 255, 255), -1); // 绘制质心

		// 左边线信息，为偏移的线
		Left_Line = Line(slope_vertebral, center_offset); // 设置左边线的斜率和起点
		// 右边线，为赛道
		Right_Line = Line(m_right, cv::Point(x_right, 110)); // 设置右边线的斜率和起点
		
		printf("[LATER CONE] LEFT模式: slope_vertebral=%.2f, center_offset=(%d, %d), offset=%d\n", 
		       slope_vertebral, center_offset.x, center_offset.y, offset);
	}
	else if (try_patching_line == RIGHT)
	{
		slope_vertebral = -m_left;
		center_offset = Point(x_left - offset, 110);

		circle(img, center_offset, 3, Scalar(0, 255, 255), -1); // 绘制质心

		// 左边线信息，为赛道
		Left_Line = Line(m_left, cv::Point(x_left, 110)); // 设置左边线的斜率和起点
		// 右边线，为偏移的线
		Right_Line = Line(slope_vertebral, center_offset); // 设置右边线的斜率和起点
		
		printf("[LATER CONE] RIGHT模式: slope_vertebral=%.2f, center_offset=(%d, %d), offset=%d\n", 
		       slope_vertebral, center_offset.x, center_offset.y, offset);
	}

	if (flag_cross == 3)
	{
		if (abs(offset) == Track_width)
		{
			printf("[LATER CONE] ★ 普通锥桶补线完成! 重置所有标志位\n");
			Num_Zhen = 0;
			Later_vertebral_Flag = 0;
			flag_cross = 0;
			First_Find_vertebral_Flag = 0;
			return;
		}
		else
		{
			printf("[LATER CONE] 普通锥桶补线进度: %d/%d\n", abs(offset), Track_width);
		}
	}
	else if (flag_cross == 2)
	{
		if (abs(offset) == Track_width + 20)
		{
			printf("[LATER CONE] ★ 中间锥桶补线完成! 重置所有标志位\n");
			Num_Zhen = 0;
			Later_vertebral_Flag = 0;
			flag_cross = 0;
			First_Find_vertebral_Flag = 0;
			return;
		}
		else
		{
			printf("[LATER CONE] 中间锥桶补线进度: %d/%d\n", abs(offset), Track_width + 20);
		}
	}

	Tracking_barrel(binaryImage_Finish);
}

void vertebral_model(Mat &img_) // 蓝色锥桶补线
{

	Point barrel_center = Point(0, 0);

	img_ROI = defineTrapezoidROI(img_);
	barrel_center = blue_vertebral_barrel_find(img_ROI);

	printf("[CONE MODEL] 进入锥桶模型, 锥桶中心: (%d, %d), Later_Flag: %d\n", 
	       barrel_center.x, barrel_center.y, Later_vertebral_Flag);

	if (Later_vertebral_Flag == 1)
	{
		printf("[CONE MODEL] 进入后锥桶处理模式\n");
		Later_vertebral_model();
		return;
	}

	if (barrel_center == Point(0, 0) || barrel_center.y > 239)
	{
		printf("[CONE MODEL] 未找到有效锥桶或锥桶位置无效\n");
		return; // 未找到锥桶
	}

	printf("[CONE MODEL] 找到锥桶, 位置: (%d, %d), Found_barrel: %d\n", 
	       barrel_center.x, barrel_center.y, Found_barrel);

	if (cleverFlag)
	{
		printf("[CONE MODEL] 使用智能模式\n");
		if (Found_barrel == Mid_barrel_serial_number - 1)
		{
			try_patching_line = RIGHT;
			flag_cross = 2;
			printf("[CONE MODEL] ★ 中间锥桶! 设置补线方向: RIGHT, flag_cross=2\n");
		}
		else if (Found_barrel == Right_barrel_serial_number - 1)
		{
			try_patching_line = RIGHT;
			flag_cross = 3;
			printf("[CONE MODEL] ★ 右边锥桶! 设置补线方向: RIGHT, flag_cross=3\n");
		}
		else if (Found_barrel == Left_barrel_serial_number - 1)
		{
			try_patching_line = LEFT;
			flag_cross = 3;
			printf("[CONE MODEL] ★ 左边锥桶! 设置补线方向: LEFT, flag_cross=3\n");
		}
		else
		{
			printf("[CONE MODEL] 未匹配到已知锥桶序列号\n");
		}
	}
	else
	{
		printf("[CONE MODEL] 使用手动模式\n");
		Sobel_LinePro(binaryImage);
		line(img, Point((99 - b_mid_blue) / m_mid_blue, 99), Point((60 - b_mid_blue) / m_mid_blue, 60), Scalar(255, 255, 0), 1); // 竖直线
	}
	circle(img, Point(barrel_center.x, barrel_center.y), 3, Scalar(0, 255, 255), -1); // 绘制质心

	if (barrel_center.y > 41 && barrel_center.y < 60 && !First_Find_vertebral_Flag)
	{
		printf("[CONE MODEL] 初次发现锥桶! 锥桶Y坐标: %d\n", barrel_center.y);
		
		if (!cleverFlag)
		{
			float s = (barrel_center.y - b_mid_blue) / m_mid_blue - barrel_center.x;
			printf("[CONE MODEL] 离中线距离计算: %.2f\n", s);
			
			if (fabs(s) < 30)
			{
				printf("[CONE MODEL] 距离中线: %.2f\n", s);
				if (fabs(s) > 5)
				{
					if (s < 0)
						try_patching_line = RIGHT;
					else
						try_patching_line = LEFT;

					flag_cross = 3;
					printf("[CONE MODEL] 设置普通锥桶, 补线方向: %s\n", (try_patching_line == LEFT) ? "LEFT" : "RIGHT");
				}
				else
				{
					if (s < 0)
						try_patching_line = RIGHT;
					else
						try_patching_line = LEFT;

					flag_cross = 2;
					printf("[CONE MODEL] 设置中间锥桶, 补线方向: %s\n", (try_patching_line == LEFT) ? "LEFT" : "RIGHT");
				}
			}
		}
		
		if (First_Find_vertebral_Flag != 1)
		{
			offset = (try_patching_line == LEFT) ? Track_width : -Track_width;
			printf("[CONE MODEL] 设置初始偏移量: %d\n", offset);
		}

		First_Find_vertebral_Flag = 1;
		printf("[CONE MODEL] ★ 设置首次发现标志 First_Find_vertebral_Flag=1\n");
	}

	if (barrel_center.y < 180 && First_Find_vertebral_Flag) // 初见锥桶，主要为了得到某一边线的斜率
	{
		printf("[CONE MODEL] 初见锥桶处理, 锥桶Y坐标: %d, flag_cross: %d\n", barrel_center.y, flag_cross);
		
		if (flag_cross == 2)
		{
			off = (try_patching_line == LEFT) ? 10 : -10; // 50
			offset = Limit(off, offset, 1);
			printf("[CONE MODEL] 中间锥桶偏移: %d -> %d\n", off, offset);
		}
		else if (flag_cross == 3)
		{
			off = (try_patching_line == LEFT) ? 110 : -110;
			offset = Limit(off, offset, 2);
			printf("[CONE MODEL] 普通锥桶偏移: %d -> %d\n", off, offset);
		}
		
		Point center_offset;
		if (try_patching_line == LEFT)
		{
			slope_vertebral = -m_right;
			center_offset = Point(x_right - offset, 110);

			circle(img, center_offset, 3, Scalar(0, 255, 255), -1); // 绘制质心

			// 左边线信息，为偏移的线
			Left_Line = Line(slope_vertebral, center_offset); // 设置左边线的斜率和起点
			// 右边线，为赛道
			Right_Line = Line(m_right, cv::Point(x_right, 110)); // 设置右边线的斜率和起点
			
			printf("[CONE MODEL] LEFT模式: slope_vertebral=%.2f, center_offset=(%d, %d)\n", 
			       slope_vertebral, center_offset.x, center_offset.y);
		}
		else if (try_patching_line == RIGHT)
		{
			slope_vertebral = -m_left;
			center_offset = Point(x_left - offset, 110);

			circle(img, center_offset, 3, Scalar(0, 255, 255), -1); // 绘制质心

			// 左边线信息，为赛道
			Left_Line = Line(m_left, cv::Point(x_left, 110)); // 设置左边线的斜率和起点
			// 右边线，为偏移的线
			Right_Line = Line(slope_vertebral, center_offset); // 设置右边线的斜率和起点

			printf("[CONE MODEL] RIGHT模式: slope_vertebral=%.2f, center_offset=(%d, %d)\n", 
			       slope_vertebral, center_offset.x, center_offset.y);
		}

		Tracking_barrel(binaryImage_Finish);
	}
	else if (barrel_center.y > 180 && Later_vertebral_Flag == 0 && First_Find_vertebral_Flag)
	{
		// 置标志位，后锥桶标志位
		Later_vertebral_Flag = 1;
		printf("[CONE MODEL] ★ 锥桶Y坐标>180, 设置后锥桶标志 Later_vertebral_Flag=1\n");
	}
}

// 画中线，得到左右边线以及中线
int Tracking(Mat dilated_image)
{
	int begin = 160; // 图像中间位置

	left_line.clear();	// 左边线清除
	right_line.clear(); // 右边线清除
	mid.clear();		// 中线清除

	for (int i = 239; i >= 101; i--)
	{
		int find_l = 0;
		int find_r = 0;
		int to_left = begin;
		int to_right = begin;

		// 查找左边线
		while (to_left > 1)
		{
			if (dilated_image.at<uchar>(i, to_left) == 255) // 只需检测当前像素是否为白色
			{
				find_l = 1;
				left_line.emplace_back(to_left, i);
				break;
			}
			to_left--;
		}

		// 查找右边线
		while (to_right < 319)
		{
			if (dilated_image.at<uchar>(i, to_right) == 255) // 只需检测当前像素是否为白色
			{
				find_r = 1;
				right_line.emplace_back(to_right, i);
				break;
			}
			to_right++;
		}

		if (!left_line.empty() && !right_line.empty())
		{
			cv::Point midx1 = left_line.back();
			cv::Point midx2 = right_line.back();
			mid.emplace_back((midx1.x + midx2.x) / 2, i);
			begin = (to_right + to_left) / 2;
		}
	}

	cv::Vec4f left_fit, right_fit;
 
	if (left_line.empty() || right_line.empty()) 
		{
		if (left_line.empty())
			Left_Line = Line(std::numeric_limits<double>::infinity(), cv::Point(1, 110)); // 设置左边线的斜率和起点
		else
			Left_Line = Line(m_left, cv::Point(x_left, 110)); // 设置左边线的斜率和起点

		if (right_line.empty())
			Right_Line = Line(std::numeric_limits<double>::infinity(), cv::Point(319, 110)); // 设置右边线的斜率和起点
		else
			Right_Line = Line(m_right, cv::Point(x_right, 110)); // 设置右边线的斜率和起点

		Tracking_barrel(binaryImage_Finish);
		return 0; // 或采取其他错误处理方式
	}

	cv::fitLine(left_line, left_fit, cv::DIST_L2, 0, 0.01, 0.01);
	cv::fitLine(right_line, right_fit, cv::DIST_L2, 0, 0.01, 0.01);

	m_left = left_fit[1] / left_fit[0];
	float b_left = left_fit[3] - m_left * left_fit[2];

	m_right = right_fit[1] / right_fit[0];
	float b_right = right_fit[3] - m_right * right_fit[2];

	a_left = std::atan(m_left) * 180.0 / CV_PI;
	a_right = std::atan(m_right) * 180.0 / CV_PI;

	if (fabs(a_right) > 89.5 || abs(a_right) < 1)
	{
		x_left = (100 + 10 - b_left) / m_left;
		Left_Line = Line(m_left, cv::Point(x_left, 110)); // 设置左边线的斜率和起点
		// 右边线，为偏移的线
		Right_Line = Line(-m_left, cv::Point(x_right, 110)); // 设置右边线的斜率和起点
		Tracking_barrel(binaryImage_Finish);
		return 0;
	}
	if (fabs(a_left) > 89.5 || abs(a_left) < 1)
	{
		x_right = (100 + 10 - b_right) / m_right;
		Left_Line = Line(-m_right, cv::Point(x_left, 110)); // 设置左边线的斜率和起点
		// 右边线，为偏移的线
		Right_Line = Line(m_right, cv::Point(x_right, 110)); // 设置右边线的斜率和起点
		Tracking_barrel(binaryImage_Finish);
		return 0;
	}

	if (Stop_Turn_Cross)
	{
		if (abs(160 - x_left) < 20)
		{
			x_right = 319;
			m_right = std::numeric_limits<double>::infinity();
		}
		else if (abs(160 - x_right) < 20)
		{
			x_left = 1;
			m_left = std::numeric_limits<double>::infinity();
		}
	}

	// 左边线信息，为赛道
	Left_Line = Line(m_left, cv::Point(x_left, 110)); // 设置左边线的斜率和起点
	// 右边线，为偏移的线
	Right_Line = Line(m_right, cv::Point(x_right, 110)); // 设置右边线的斜率和起点
	Tracking_barrel(binaryImage_Finish);

	// printf("L:%f   R:%f\n", m_left, m_right);

	x_left = (100 + 10 - b_left) / m_left;
	x_right = (100 + 10 - b_right) / m_right;

	// printf("X:%f\n", x_right - x_left);

	return 1;
}

void Tracking_barrel(Mat dilated_image)
{

	left_line.clear();	// 左边线清除
	right_line.clear(); // 右边线清除
	mid.clear();		// 中线清除

	for (int i = 239; i >= 101; i--) // 行遍历
	{
		int X_Left = 0;
		X_Left = (int)Left_Line.startPoint.x + (i - Left_Line.startPoint.y) / Left_Line.slope;
		if (X_Left < 1)
			X_Left = 1;

		left_line.emplace_back(cv::Point(X_Left, i));

		int X_Right = 0;
		X_Right = (int)Right_Line.startPoint.x + (i - Right_Line.startPoint.y) / Right_Line.slope;
		if (X_Right > 319)
			X_Right = 319;

		right_line.emplace_back(cv::Point(X_Right, i));

		mid.emplace_back(cv::Point((X_Left + X_Right) / 2, i)); // 得到中线
	}
}

#define To_Left 1
#define To_Right 2
uint8_t BianDao_Num[3] = {0};

void Judge_Turn_Pro(Mat frame)
{

	// 将颜色空间转换为HSV
	Mat hsv;
	cvtColor(frame, hsv, COLOR_BGR2HSV);

	// 定义HSV中的蓝色范围
	Scalar lower_blue(90, 29, 30);
	Scalar upper_blue(130, 255, 255);
//  Scalar lower_blue = Scalar(89, 100, 210); // 蓝色的低阈值
// 	Scalar upper_blue= Scalar(111, 255, 255); // 蓝色的高阈值
	// 创建掩膜
	Mat mask;
	inRange(hsv, lower_blue, upper_blue, mask);

	// 查找蓝色区域的轮廓
	vector<vector<Point>> contours;
	findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	// imshow("222", mask);
	// cout << "轮廓数量: " << contours.size() << endl;
	//  查找最大的蓝色区域
	double max_area = 0;
	vector<Point> largest_contour;
	int height = frame.rows;
	int width = frame.cols;
	int upper_limit = height / 3;
	int left_limit = width / 8;

	for (const auto &contour : contours)
	{
		double area = contourArea(contour);

		Rect boundingRect = cv::boundingRect(contour);

		if (boundingRect.y >= upper_limit &&
			boundingRect.x >= left_limit)
		{
			if (area > max_area)
			{
				max_area = area;
				largest_contour = contour;
			}
		}
	}

	static uint8_t Count = 0;

	if (Find_Turn_Flag == 1)
	{
		Count++;
		if (Count % 10 == 0)
		{
			BianDao_Num[0] = 0;
			BianDao_Num[1] = 0;
			R_L_Flag = 0;
			Stop_Turn_Cross = 1;
		}
		return;
	}

	// 处理最大蓝色区域
	if (!largest_contour.empty())
	{

		Rect rect = boundingRect(largest_contour);

		Point2f src_points[4] = {
			Point2f(rect.x, rect.y),
			Point2f(rect.x + rect.width, rect.y),
			Point2f(rect.x + rect.width, rect.y + rect.height),
			Point2f(rect.x, rect.y + rect.height)};

		Point2f dst_points[4] = {
			Point2f(0, 0),
			Point2f(300, 0),
			Point2f(300, 200),
			Point2f(0, 200)};

		// 计算透视变换矩阵
		Mat matrix = getPerspectiveTransform(src_points, dst_points);

		// 应用透视变换
		Mat warped;
		warpPerspective(frame, warped, matrix, Size(300, 200)); // 进行透视变换

		// 进行后续图像处理（如灰度转换、二值化、寻找轮廓等）
		Mat gray_warped, thresh_warped;
		cvtColor(warped, gray_warped, COLOR_BGR2GRAY);
		threshold(gray_warped, thresh_warped, 127, 255, THRESH_BINARY);

		Mat white_mask;
		inRange(gray_warped, 220, 255, white_mask);
		Mat combined_mask;
		bitwise_and(thresh_warped, white_mask, combined_mask);

		Mat roi = combined_mask(Rect(0, 2 * combined_mask.rows / 3, combined_mask.cols, combined_mask.rows / 3)).clone();

		// 找到轮廓
		vector<vector<Point>> white_contours;
		findContours(roi, white_contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

		if (!white_contours.empty())
		{

			vector<Point> largest_white_contour = *max_element(white_contours.begin(), white_contours.end(),
															   [](const vector<Point> &a, const vector<Point> &b)
															   {
																   return contourArea(a) < contourArea(b);
															   });

			Vec4f fitted_line;
			fitLine(largest_white_contour, fitted_line, DIST_L2, 0, 0.01, 0.01);

			float vx = fitted_line[0], vy = fitted_line[1]; // 方向向量
			float x0 = fitted_line[2], y0 = fitted_line[3]; // 直线上的一点

			// 强制规定直线的大致为从下向上
			if (vy < 0)
			{
				vx = -vx;
				vy = -vy;
			}

			float roi_offset_y = 2 * combined_mask.rows / 3;
			x0 += 0;
			y0 += roi_offset_y;

			Point pt1, pt2;
			pt1.x = x0 - vx * 200;
			pt1.y = y0 - vy * 200;
			pt2.x = x0 + vx * 200;
			pt2.y = y0 + vy * 200;

			// 修改后的代码片段
			Mat roi = combined_mask(Rect(0, 0, combined_mask.cols, combined_mask.rows / 3)).clone();

			Mat distance_matrix(roi.size(), CV_32F);
			for (int y = 0; y < roi.rows; y++)
			{
				for (int x = 0; x < roi.cols; x++)
				{
					float relative_pos = (x - x0) * vy - (y - y0) * vx;
					distance_matrix.at<float>(y, x) = relative_pos;
				}
			}

			Mat left_mask = (distance_matrix < 0) & (roi == 255);
			Mat right_mask = (distance_matrix > 0) & (roi == 255);

			// line(warped, pt1, pt2, Scalar(0, 0, 255), 2, LINE_AA);  // 红色直线
			// imshow("111", warped);
			int left_count = countNonZero(left_mask);
			int right_count = countNonZero(right_mask);
			// printf("left_count:%d,right_count:%d\n", left_count, right_count);

			// 判断转向方向
			if (left_count >= right_count)
			{
				R_L_Flag_Temp = 1; // 左转
			}
			else
			{
				R_L_Flag_Temp = 2; // 右转
			}
		}

		// 更新转向计数
		if (R_L_Flag_Temp == 1)
		{
			BianDao_Num[0]++;
		}
		else if (R_L_Flag_Temp == 2)
		{
			BianDao_Num[1]++;
		}

		// 根据计数来判断最终的转向
		if (BianDao_Num[0] > BianDao_Num[1] && BianDao_Num[0] > 8)
		{
			R_L_Flag = 1;
			Find_Turn_Flag = 1;
		}
		else if (BianDao_Num[0] < BianDao_Num[1] && BianDao_Num[1] > 8)
		{
			R_L_Flag = 2;
			Find_Turn_Flag = 1;
		}
	}
	else
	{
		cout << "没有找到蓝色区域" << endl;
	}
}

int bufferSize = 50;		 // 移动平均窗口大小
std::vector<int> biasBuffer; // 用于存储最近的偏差值
void Calculate_Bias(void)
{
	for (int i = 1; i <= 100; i += 2)
	{
		Bias_Line_int += mid[i].x;
	}
	Bias_Line_int = Bias_Line_int / 50 - 160;

	if (Bias_Line_int > 80)
		Bias_Line_int = 80;
	else if (Bias_Line_int < -80)
		Bias_Line_int = -80;
	Bias_Line = Bias_Line_int;
}

void display_Information(void)
{
	int j = 0;
	for (int i = 239; i >= 101; i--) // 行遍历
	{
		// 在原图上画赛道线, 赛道中线
		circle(img, Point(int(left_line[j].x), i), 0.1, Scalar(0, 0, 0), 2);
		circle(img, Point(int(right_line[j].x), i), 0.1, Scalar(0, 0, 0), 2);
		circle(img, Point(int((left_line[j].x + right_line[j].x) / 2), i), 0.1, Scalar(255, 0, 0), 2);
		j++;
	}

	// 将轨迹绘制在原图上，方便观察
	// 1. 中间线
	line(img, Point(160, 100), Point(160, 239), Scalar(0, 0, 255), 2); // 白色竖直线
	// 2. 找线的感性区域
	rectangle(img, Rect(1, 100, 319, 239), Scalar(0, 255, 0), 1);

	// 文本显示在图片上
	std::ostringstream oss;
	oss << "Bias_Line: " << static_cast<int>(Bias_Line); // 添加说明
	std::string text = oss.str();
	cv::putText(img, text, cv::Point(0, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);

	oss.str(""); // 清空之前的内容
	oss.clear(); // 清空状态标志

	oss << "Track_width: " << static_cast<int>(x_right - x_left); // 添加说明
	text = oss.str();
	cv::putText(img, text, cv::Point(0, 80), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);

	// 显示斑马线
	oss.str(""); // 清空之前的内容
	oss.clear(); // 清空状态标志

	switch (flag_cross)
	{
	case -1:
	{
		oss << "flag_cross: find"; // 添加说明
		text = oss.str();
		cv::putText(img, text, cv::Point(0, 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1);
		break;
	}
	case 0:
	{
		oss << "flag_cross: nofind"; // 添加说明
		text = oss.str();
		cv::putText(img, text, cv::Point(0, 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
		break;
	}
	case 1:
	{
		oss << "flag_cross: stop_car"; // 添加说明
		text = oss.str();
		cv::putText(img, text, cv::Point(0, 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1);
		break;
	}
	case 2:
	{
		oss << "vertebral: middleCone! (" << Found_barrel + 1 << ")"; // 添加说明 中间锥桶
		text = oss.str();
		cv::putText(img, text, cv::Point(0, 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1);
		break;
	}
	case 3:
	{
		oss << "vertebral: ordinary! (" << Found_barrel + 1 << ")"; // 添加说明 普通锥桶
		text = oss.str();
		cv::putText(img, text, cv::Point(0, 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1);
		break;
	}
	case 4:
	{
		oss << "  stop!"; // 添加说明
		text = oss.str();
		cv::putText(img, text, cv::Point(0, 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1);
		break;
	}
	}

	oss.str(""); // 清空之前的内容
	oss.clear(); // 清空状态标志

	switch (R_L_Flag)
	{
	case 0:
	{
		oss << "R_L_Flag: To_zhi"; // 添加说明
		break;
	}
	case 1:
	{
		// 如果向左，绘制向左的箭头
		Point start_point(150, 90);												   // 箭头起点
		Point end_point(100, 90);												   // 箭头终点（向左）
		arrowedLine(img, start_point, end_point, Scalar(0, 255, 0), 2, 8, 0, 0.5); // 绘制绿色箭头

		oss << "R_L_Flag: To_Left"; // 添加说明
		break;
	}
	case 2:
	{
		// 如果向右，绘制向右的箭头
		Point start_point(150, 90);												   // 箭头起点
		Point end_point(200, 90);												   // 箭头终点（向右）
		arrowedLine(img, start_point, end_point, Scalar(0, 255, 0), 2, 8, 0, 0.5); // 绘制绿色箭头

		oss << "R_L_Flag: To_Right"; // 添加说明
		break;
	}
	default:
	{
		// 如果 R_L_Flag 的值不在 0 到 2 之间，可以添加一个默认行为
		oss << "R_L_Flag: Unknown"; // 添加说明
		break;
	}
	}

	text = oss.str();
	cv::putText(img, text, cv::Point(0, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1);
}

Mat hsvImg, outputImg;
int h_min = 90, h_max = 130; // 色调范围
int s_min = 29, s_max = 255; // 饱和度范围
int v_min = 30, v_max = 255; // 亮度范围

// 更新输出图像的回调函数
void updateHSV(int, void *)
{
	// 将HSV图像的值按滑动条设置的范围过滤
	inRange(hsvImg, Scalar(h_min, s_min, v_min), Scalar(h_max, s_max, v_max), outputImg);
	imshow("Output Image", outputImg);
}

#if ubuntu
int fd = -1;
int Serial_Init(void)
{
	const char *port = "/dev/ttyUSB0"; // ttyUSB0  ttyACM0
	int baud_rate = B115200;

	fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1)
	{
		std::cerr << "error_serial " << port << std::endl;
		return 0;
	}

	fcntl(fd, F_SETFL, 0);

	struct termios options;
	tcgetattr(fd, &options);

	cfsetispeed(&options, baud_rate);
	cfsetospeed(&options, baud_rate);

	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;

	options.c_cflag |= (CLOCAL | CREAD);

	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	options.c_iflag &= ~(IXON | IXOFF | IXANY);
	options.c_oflag &= ~OPOST;

	tcsetattr(fd, TCSANOW, &options);
	waitKey(100);
	return 1;
}

int Data_Send(void)
{
	uint8_t buffer[5];
	// 发送包头
	memcpy(buffer, &Head, sizeof(int8_t));
	memcpy(buffer + 1, &Bias_Line, sizeof(int8_t));
	memcpy(buffer + 2, &flag_cross, sizeof(int8_t));
	memcpy(buffer + 3, &R_L_Flag, sizeof(int8_t));
	memcpy(buffer + 4, &Tail, sizeof(int8_t));

	// 调试输出：显示发送的数据内容
	printf("[UART DEBUG] 发送数据包: HEAD=0x%02X, Bias=%d, FlagCross=%d, R_L_Flag=%d, TAIL=0x%02X\n", 
	       Head, Bias_Line, flag_cross, R_L_Flag, Tail);

	int n = write(fd, buffer, sizeof(buffer)); // 发送5字节

	// 调试输出：显示发送结果
	if (n < 0)
	{
		printf("[UART ERROR] 发送失败: %d字节, 错误: %s\n", n, strerror(errno));
		close(fd);
		return 1;
	}
	else
	{
		printf("[UART SUCCESS] 发送成功: %d字节\n", n);
	}
	return 0;
}
#endif

#if ubuntu
std::string getCurrentTimeString()
{
	auto now = std::time(nullptr);
	std::tm *localTime = std::localtime(&now);
	std::ostringstream oss;
	oss << std::put_time(localTime, "%Y-%m-%d_%H-%M-%S");
	return oss.str();
}
#endif

#define Select_Video_Frame (1)
#if ubuntu
int main(int argc, char *argv[])
{
#else
int main()
{
#endif 

#if ubuntu
	if (!Serial_Init())
	{
		return 0;
	}
#endif

#if Select_Video_Frame == 1 // 视频
#if ubuntu
	cv::VideoCapture capture("/dev/video0", cv::CAP_V4L2);

	// 设置分辨率
	cv::Size resolution(320, 240);
	capture.set(cv::CAP_PROP_FRAME_WIDTH, resolution.width);
	capture.set(cv::CAP_PROP_FRAME_HEIGHT, resolution.height);

#else
	VideoCapture capture;
	// 打开视频对象
	capture.open(video_path);
#endif

#if ubuntu
	// 帧率初始化
	auto start = std::chrono::high_resolution_clock::now();
	int frameCount = 0;
	float FPS = 0;
#endif

	cv::VideoWriter writer;
	uint8_t handle = 0;

#if ubuntu

	if (argc > 1)
	{
		std::string param = argv[1];
		std::string outputPath;
		// 检查参数是否等于 "6"
		char key = cv::waitKey(1) & 0xFF;
            switch (key) {
                case 'q':
				param = "6";
                	break;

                case 's':
                param = "1";
                    break;

            }
		if (param == "6")
		{
			// 生成输出文件名
			outputPath = "/home/dgdz/桌面/原图/" + getCurrentTimeString() + ".mp4";
			std::cout << "开始录原图" << std::endl;
			handle = 2;
		}
		else if (param == "1")
		{
			// 生成输出文件名
			outputPath = "/home/dgdz/桌面/处理图/" + getCurrentTimeString() + ".mp4";
			std::cout << "开始录处理图" << std::endl;
			handle = 1;
		}
		else
		{

			switch (argv[1][0])
			{
			case '1':
				Left_barrel_serial_number = 1;
				printf("Left_barrel_serial_number\n");
				break;
			case '2':
				Right_barrel_serial_number = 1;
				printf("Right_barrel_serial_number\n");
				break;
			default:
				Mid_barrel_serial_number = 1;
				printf("Mid_barrel_serial_number\n");
				break;
			}
			switch (argv[1][1])
			{
			case '1':
				Left_barrel_serial_number = 2;
				printf("Left_barrel_serial_number\n");
				break;
			case '2':
				Right_barrel_serial_number = 2;
				printf("Right_barrel_serial_number\n");
				break;
			default:
				Mid_barrel_serial_number = 2;
				printf("Mid_barrel_serial_number\n");
				break;
			}
			switch (argv[1][2])
			{
			case '1':
				Left_barrel_serial_number = 3;
				printf("Left_barrel_serial_number\n");
				break;
			case '2':
				Right_barrel_serial_number = 3;
				printf("Right_barrel_serial_number\n");
				break;
			default:
				Mid_barrel_serial_number = 3;
				printf("Mid_barrel_serial_number\n");
				break;
			}
			cleverFlag = 1;
		}

		if (argv[1][0] == '6' || argv[1][0] == '1')
		{
			int frameWidth = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH));
			int frameHeight = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT));
			// 定义视频编解码器和输出视频文件
			writer.open(outputPath, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 30, cv::Size(frameWidth, frameHeight));

			if (!writer.isOpened())
			{
				std::cerr << "no" << std::endl;
				return -1;
			}
			std::cout << "按 '6' 键停止录制" << std::endl;
		}
	}
#endif

	while (capture.read(img))
	{
#if ubuntu
		frameCount++;
#else
		resize(img, img, Size(320, 240)); // 压缩到320x240
#endif

		// 写入视频文件
		if (handle == 2)
			writer.write(img); // 将帧写入视频文件
		ImagePreprocessing_Sobel_LinePro(img);

		if (is_First_cross != 1 && Stop_Turn_Cross != 1) // 找到斑马线后不再寻找锥桶
		{
			vertebral_model(img);
		}

		if (Stop_Turn_Cross != 1) // 第一次找到斑马线后停止寻找
		{
			crossroad(img);
		}

		if (is_First_cross == 1 && Stop_Turn_Cross != 1)
		{
			img_ROI = defineTrapezoidROI(img);
			Judge_Turn_Pro(img_ROI);
		}

		Calculate_Bias();
		display_Information();
#if ubuntu
		Data_Send();
#endif

#if ubuntu
		auto end = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> elapsed = end - start;
		if (elapsed.count() >= 1.0)
		{ // 计算一次帧率
			FPS = frameCount / elapsed.count();
			frameCount = 0;
			start = std::chrono::high_resolution_clock::now();
		}

		std::ostringstream oss;
		oss << std::fixed << std::setprecision(2) << "FPS: " << FPS;
		std::string text = oss.str();
		cv::putText(img, text, cv::Point(230, 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
#endif

		// 写入视频文件
		if (handle == 1)
			writer.write(img); // 将帧写入视频文件

		resize(img, img, cv::Size(), 2.5, 2.5);
		imshow("img_finall", img);

#if ubuntu
		// 按 '6' 键停止录制
		if (cv::waitKey(1) == '6')
		{
			break;
		}

#else
		if (waitKey(10) >= 0)
		{
			while (1)
			{
				if (waitKey(10) >= 0)
					break;
			}
		}
#endif
	}
	capture.release();
	writer.release();
	cv::destroyAllWindows();
	return 0;
#elif Select_Video_Frame == 2 // 单个图像操作
	/////////////////////////////////////////////////////////////////////////////////////////////
	img = imread(frame_path);

	Judge_Turn_Pro(img);
	imshow("img_finall", img);
	// printf("%d",R_L_Flag);
	waitKey(0);

	return 0;
#elif Select_Video_Frame == 3 // 调节色域
img = imread(frame_path);
cvtColor(img, hsvImg, COLOR_BGR2HSV);

namedWindow("Original Image", WINDOW_AUTOSIZE);
namedWindow("Output Image", WINDOW_AUTOSIZE);

imshow("Original Image", img);

createTrackbar("H Min", "Output Image", &h_min, 179, updateHSV);
createTrackbar("H Max", "Output Image", &h_max, 179, updateHSV);
createTrackbar("S Min", "Output Image", &s_min, 255, updateHSV);
createTrackbar("S Max", "Output Image", &s_max, 255, updateHSV);
createTrackbar("V Min", "Output Image", &v_min, 255, updateHSV);
createTrackbar("V Max", "Output Image", &v_max, 255, updateHSV);

updateHSV(0, 0);

waitKey(0);

return 0;
#else						  // 视频转图像

// 指定保存帧的文件夹路径
string savePath = "D:/IDE/bike_video/video/nk6/";

// 打开视频文件

VideoCapture cap;
// 打开视频对象
cap.open(video_path);

int frameCount = 0;			// 记录帧数
int saveFrameInterval = 10; // 每10帧提取一帧

// 读取视频帧
Mat frame;
while (true)
{
	// 读取下一帧
	bool success = cap.read(frame);
	if (!success)
	{
		cout << "视频播放结束或读取失败" << endl;
		break;
	}

	// 判断是否为需要保存的帧
	if (frameCount % saveFrameInterval == 0)
	{
		// 生成文件名
		stringstream ss;
		ss << savePath << "frame_" << frameCount << ".jpg"; // 保存为指定路径下的图像文件
		string filename = ss.str();

		// 保存图像
		imwrite(filename, frame);
		cout << "保存帧 " << frameCount << " 为文件: " << filename << endl;
	}

	// 增加帧计数
	frameCount++;
}

// 释放资源
cap.release();
cout << "视频处理完成" << endl;

return 0;
#endif
}
