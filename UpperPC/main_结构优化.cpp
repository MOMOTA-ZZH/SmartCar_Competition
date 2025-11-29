/**
 * 室外无人驾驶自行车 - 上位机图像处理代码
 * 
 * 主要模块分组：
 * 1. 头文件和全局定义
 * 2. 全局变量声明（按功能分组）
 * 3. 图像处理函数
 * 4. 赛道跟踪函数
 * 5. 锥桶检测和避障函数
 * 6. 斑马线检测函数
 * 7. 转向标志识别函数
 * 8. 控制参数计算函数
 * 9. 显示和通信函数
 * 10. 主函数
 */

// ============================================================================
// 1. 头文件和全局定义
// ============================================================================

#include <opencv2/opencv.hpp>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <vector>
#include <string>
#include <fcntl.h>
#include <cstring>
#include <algorithm>
#include <cmath>

#define ubuntu (1)
#if ubuntu
#include <termios.h>
#include <unistd.h>
#endif

using namespace cv;
using namespace std;

// ============================================================================
// 2. 全局变量声明（按功能分组）
// ============================================================================

// 2.1 文件路径配置
string frame_path = "/home/dgdz/桌面/Test_Image/yellow.png";
string video_path = "D:/1vscode/930/8.mp4";

// 2.2 赛道跟踪相关参数
float Track_width = 120;
float m_left = 0, m_right = 0;
float a_right = 0, a_left = 0;
float x_left = 0, x_right = 0;
float m_mid_blue = 0, b_mid_blue = 0;
int8_t Trackingflag = 1;

// 2.3 串口通信协议
int8_t Head = 0XA5;
int8_t Tail = 0X5A;
int8_t Length = 3;

// 2.4 图像处理参数
int My_threshold = 0;
int My_HoughLinesP = 50;
int Min_moderate = 95;
int Min_barrel_area = 130;
int brightness_threshold = 20;

// 2.5 控制标志位
int8_t flag_cross = 0;
int8_t Bias_Line;
int8_t R_L_Flag;
int8_t R_L_Flag_Temp;
int Bias_Line_int = 0;

// 2.6 图像矩阵
cv::Mat img;
cv::Mat img_xiaoxi;
cv::Mat img_ROI;
cv::Mat binaryImage;
cv::Mat binaryImage_Finish;
cv::Mat binaryImage12;

// 2.7 状态标志
uint8_t is_First_cross = 0;
uint8_t barrel_find_finish = 0;
uint8_t barrel_find_yes = 0;
uint8_t Stop_Turn_Cross = 0;
uint8_t Find_Turn_Flag = 0;

// 2.8 锥桶序列配置
int Left_barrel_serial_number = 1;
int Mid_barrel_serial_number = 2;
int Right_barrel_serial_number = 3;

// 2.9 边界点存储容器
std::vector<cv::Point> left_line;
std::vector<cv::Point> right_line;
std::vector<cv::Point> mid;
std::vector<cv::Point> left_line_blue;
std::vector<cv::Point> right_line_blue;
std::vector<cv::Point> mid_blue;

// 2.10 锥桶检测相关变量
#define LEFT 1
#define RIGHT 2
#define MIDDLE 3

vector<Point> previousCenters;
size_t maxHistory = 5;
bool cleverFlag = 1;
int Frame_zhen = 60;
int detectCount = 0;
int missingCount = 0;
bool barrelFound = false;
int Found_barrel = 0;
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

// 2.11 锥桶调试统计
static int cone_debug_frame_count = 0;
static int cone_detected_count = 0;
static int cone_missed_count = 0;

// 2.12 转向标志相关
#define To_Left 1
#define To_Right 2
uint8_t BianDao_Num[3] = {0};

// 2.13 直线结构体定义
struct Line
{
	float slope;
	cv::Point startPoint;

	Line(float s, cv::Point start)
		: slope(s), startPoint(start)
	{
	}
} Left_Line(0, cv::Point(0, 0)), Right_Line(0, cv::Point(0, 0));

// 2.14 斑马线检测相关
int crossroad_num = 0;
Rect roi_rect;

// 2.15 其他变量
int bufferSize = 50;
std::vector<int> biasBuffer;
Mat hsvImg, outputImg;
int h_min = 90, h_max = 130;
int s_min = 29, s_max = 255;
int v_min = 30, v_max = 255;

#if ubuntu
int fd = -1;     //串口标志初始值
#endif

// ============================================================================
// 3. 图像处理函数
// ============================================================================

/**
 * @brief 定义梯形感兴趣区域(ROI)
 */
Mat defineTrapezoidROI(Mat src)
{
	Mat mask = Mat::zeros(src.size(), src.type());
	int height = src.rows;
	int width = src.cols;

	Point top_left(0, 210);
	Point top_right(320, 210);
	Point top_middle_left(100, 30);
	Point top_middle_right(160, 30);
	Point bottom_right(320, 240);
	Point bottom_left(0, 240);

	vector<Point> trapezoid = {top_left, top_middle_left, top_middle_right, top_right, bottom_right, bottom_left};
	vector<vector<Point>> contours;
	contours.push_back(trapezoid);

	fillPoly(mask, contours, Scalar(255, 255, 255));
	Mat maskedImage;
	bitwise_and(src, mask, maskedImage);
	polylines(src, contours, true, Scalar(255, 255, 255), 1);

	return maskedImage;
}

/**
 * @brief 主图像预处理函数
 */
Mat ImagePreprocessing_Sobel_LinePro(Mat frame_a)
{
	int width = frame_a.cols;
	int height = frame_a.rows;

	binaryImage_Finish = cv::Mat::zeros(height, width, CV_8U);
	cv::Mat binaryImage_Finish3 = cv::Mat::zeros(height, width, CV_8U);
	cv::Mat binaryImage_Finish4 = cv::Mat::zeros(height, width, CV_8U);
	cv::Mat binaryImage_Finish5 = cv::Mat::zeros(height, width, CV_8U);

	Mat binaryImage12;
	
	// 图像预处理
	Mat originalImage;
	cvtColor(frame_a, originalImage, cv::COLOR_BGR2GRAY);
	Mat enhancedImage = originalImage;
	GaussianBlur(enhancedImage, enhancedImage, Size(5, 5), 2);
	medianBlur(enhancedImage, enhancedImage, 5);

	Mat kernel1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
	cv::Mat morphedImage;
	morphologyEx(enhancedImage, morphedImage, MORPH_OPEN, kernel1);
	cv::dilate(morphedImage, morphedImage, kernel1, cv::Point(-1, -1), 1);
	medianBlur(morphedImage, morphedImage, 5);

	// Sobel边缘检测
	Mat sobelx, sobely;
	Sobel(morphedImage, sobelx, CV_64F, 1, 0, 3);
	Sobel(morphedImage, sobely, CV_64F, 0, 1, 3);
	convertScaleAbs(sobelx, sobelx);
	convertScaleAbs(sobely, sobely);

	Mat gradientMagnitude;
	addWeighted(sobelx, 0.5, sobely, 0.5, 0, gradientMagnitude);

	// 二值化处理
	binaryImage = Mat::zeros(morphedImage.size(), CV_8U);
	cv::threshold(gradientMagnitude, binaryImage, My_threshold, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
	Mat kernel2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	cv::dilate(binaryImage, binaryImage, kernel2, cv::Point(-1, -1), 1);

	// 霍夫线段检测
	vector<Vec4i> lines;
	int minLineLength = 20;
	int maxLineGap = 10;

	int x_roi = 1;
	int y_roi = 100;
	int width_roi = 319;
	int height_roi = 140;
	Rect roi(x_roi, y_roi, width_roi, height_roi);
	Mat croppedObject = binaryImage(roi);

	HoughLinesP(croppedObject, lines, 1, CV_PI / 180, My_HoughLinesP, minLineLength, maxLineGap);

	// 线段过滤和分类
	for (size_t i = 0; i < lines.size(); i++)
	{
		Vec4i l = lines[i];

		float k = static_cast<float>(l[3] - l[1]) / (l[2] - l[0]);
		float angle_k = std::atan(k) * 180.0 / CV_PI;
		float b = l[1] - k * l[0];
		float x6 = (10 - b) / k;
		float length = sqrt(pow(l[3] - l[1], 2) + pow(l[2] - l[0], 2));

		if (abs(angle_k) > 30 && length > 30)
		{
			l[0] += x_roi; l[1] += y_roi; l[2] += x_roi; l[3] += y_roi;
			line(binaryImage_Finish3, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 2, LINE_AA);
			l[0] -= x_roi; l[1] -= y_roi; l[2] -= x_roi; l[3] -= y_roi;

			if (abs(angle_k) < 80 && length > 30 && (l[1] > 55 || l[3] > 55) && (l[1] < 15 || l[3] < 15) && 
				((l[0] + l[2] < 319) ? (k < 0 && abs(angle_k - a_left) < 18 && abs(115 - x_right + x6) < 10) : 
				 (k > 0 && abs(angle_k - a_right) < 18 && abs(115 + x_left - x6) < 10)))
			{
				l[0] += x_roi; l[1] += y_roi; l[2] += x_roi; l[3] += y_roi;
				line(binaryImage_Finish4, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 2, LINE_AA);
				l[0] -= x_roi; l[1] -= y_roi; l[2] -= x_roi; l[3] -= y_roi;
			}

			if (abs(angle_k) < 85 && length > 30 && (l[1] > 65 || l[3] > 65) && (l[1] < 8 || l[3] < 8) && 
				((l[0] < 160 + 20 || l[2] < 160 + 20) ? k < 0 : 1) && 
				((l[0] > 160 - 20 || l[2] > 160 - 20) ? k > 0 : 1))
			{
				l[0] += x_roi; l[1] += y_roi; l[2] += x_roi; l[3] += y_roi;
				line(binaryImage_Finish5, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 2, LINE_AA);
				l[0] -= x_roi; l[1] -= y_roi; l[2] -= x_roi; l[3] -= y_roi;
			}
		}
	}

	// 多策略跟踪选择
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
			goto Tracking_start;
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
			goto Tracking_start;
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

/**
 * @brief 蓝色边界检测函数
 */
void Sobel_LinePro(Mat frame_a)
{
	int width = frame_a.cols;
	int height = frame_a.rows;

	cv::Mat binaryImage_Finish3 = cv::Mat::zeros(height, width, CV_8U);

	vector<Vec4i> lines;
	int minLineLength = 30;
	int maxLineGap = 10;

	int x_roi = 50;
	int y_roi = 60;
	int width_roi = 200;
	int height_roi = 50;
	Rect roi(x_roi, y_roi, width_roi, height_roi);
	Mat croppedObject = binaryImage(roi);

	HoughLinesP(croppedObject, lines, 1, CV_PI / 180, My_HoughLinesP, minLineLength, maxLineGap);

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
				l[0] += x_roi; l[1] += y_roi; l[2] += x_roi; l[3] += y_roi;
				line(binaryImage_Finish3, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 2, LINE_AA);
				l[0] -= x_roi; l[1] -= y_roi; l[2] -= x_roi; l[3] -= y_roi;
			}
		}
	}

	int begin = 160;
	left_line_blue.clear();
	right_line_blue.clear();
	mid_blue.clear();

	for (int i = 99; i >= 61; i--)
	{
		int find_l = 0, find_r = 0;
		int to_left = begin, to_right = begin;

		while (to_left > 1)
		{
			if (binaryImage_Finish3.at<uchar>(i, to_left) == 255)
			{
				find_l = 1;
				left_line_blue.emplace_back(to_left, i);
				break;
			}
			to_left--;
		}

		while (to_right < 319)
		{
			if (binaryImage_Finish3.at<uchar>(i, to_right) == 255)
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
		return;
	}

	cv::fitLine(mid_blue, mid_fit, cv::DIST_L2, 0, 0.01, 0.01);
	m_mid_blue = mid_fit[1] / mid_fit[0];
	b_mid_blue = mid_fit[3] - m_mid_blue * mid_fit[2];
}

// ============================================================================
// 4. 赛道跟踪函数
// ============================================================================

/**
 * @brief 赛道边界跟踪函数
 */
int Tracking(Mat dilated_image)
{
	int begin = 160;
	left_line.clear();
	right_line.clear();
	mid.clear();

	for (int i = 239; i >= 101; i--)
	{
		int find_l = 0, find_r = 0;
		int to_left = begin, to_right = begin;

		while (to_left > 1)
		{
			if (dilated_image.at<uchar>(i, to_left) == 255)
			{
				find_l = 1;
				left_line.emplace_back(to_left, i);
				break;
			}
			to_left--;
		}

		while (to_right < 319)
		{
			if (dilated_image.at<uchar>(i, to_right) == 255)
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
			Left_Line = Line(std::numeric_limits<double>::infinity(), cv::Point(1, 110));
		else
			Left_Line = Line(m_left, cv::Point(x_left, 110));

		if (right_line.empty())
			Right_Line = Line(std::numeric_limits<double>::infinity(), cv::Point(319, 110));
		else
			Right_Line = Line(m_right, cv::Point(x_right, 110));

		Tracking_barrel(binaryImage_Finish);
		return 0;
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
		Left_Line = Line(m_left, cv::Point(x_left, 110));
		Right_Line = Line(-m_left, cv::Point(x_right, 110));
		Tracking_barrel(binaryImage_Finish);
		return 0;
	}
	if (fabs(a_left) > 89.5 || abs(a_left) < 1)
	{
		x_right = (100 + 10 - b_right) / m_right;
		Left_Line = Line(-m_right, cv::Point(x_left, 110));
		Right_Line = Line(m_right, cv::Point(x_right, 110));
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

	Left_Line = Line(m_left, cv::Point(x_left, 110));
	Right_Line = Line(m_right, cv::Point(x_right, 110));
	Tracking_barrel(binaryImage_Finish);

	x_left = (100 + 10 - b_left) / m_left;
	x_right = (100 + 10 - b_right) / m_right;

	return 1;
}

/**
 * @brief 锥桶模式下的边界跟踪
 */
void Tracking_barrel(Mat dilated_image)
{
	left_line.clear();
	right_line.clear();
	mid.clear();

	for (int i = 239; i >= 101; i--)
	{
		int X_Left = 0;
		X_Left = (int)Left_Line.startPoint.x + (i - Left_Line.startPoint.y) / Left_Line.slope;
		if (X_Left < 1) X_Left = 1;

		left_line.emplace_back(cv::Point(X_Left, i));

		int X_Right = 0;
		X_Right = (int)Right_Line.startPoint.x + (i - Right_Line.startPoint.y) / Right_Line.slope;
		if (X_Right > 319) X_Right = 319;

		right_line.emplace_back(cv::Point(X_Right, i));

		mid.emplace_back(cv::Point((X_Left + X_Right) / 2, i));
	}
}

// ============================================================================
// 5. 锥桶检测和避障函数
// ============================================================================

/**
 * @brief 蓝色锥桶检测函数
 */
Point blue_vertebral_barrel_find(Mat frame)
{
	Point barrel_center = Point(0, 0);

	Mat hsvImage;
	cvtColor(frame, hsvImage, COLOR_BGR2HSV);

	Scalar lower_blue = Scalar(100, 160, 30);
	Scalar upper_blue = Scalar(151, 255, 255);
	Scalar lower_white = Scalar(0, 0, 200);
	Scalar upper_white = Scalar(180, 30, 255);

	Mat blueMask, whiteMask;
	inRange(hsvImage, lower_blue, upper_blue, blueMask);
	inRange(hsvImage, lower_white, upper_white, whiteMask);

	Mat filteredMask;
	bitwise_not(whiteMask, whiteMask);
	bitwise_and(blueMask, whiteMask, filteredMask);

	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(filteredMask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	cone_debug_frame_count++;
	printf("[CONE DEBUG] 帧%d: 检测到%d个轮廓\n", cone_debug_frame_count, contours.size());

	sort(contours.begin(), contours.end(), [](const vector<Point> &a, const vector<Point> &b)
		 { return contourArea(a) > contourArea(b); });

	bool foundValidContour = false;

	if (!contours.empty())
	{
		const vector<Point> &largestContour = contours[0];
		double maxArea = contourArea(largestContour);
		
		printf("[CONE DEBUG] 最大轮廓面积: %.2f, 阈值: %d\n", maxArea, Min_barrel_area);
		
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
			missingCount = 0;
			barrelFound = true;
			printf("[CONE DEBUG] ★ 确认锥桶存在! barrelFound=true\n");
		}
	}
	else if (barrelFound)
	{
		missingCount++;
		printf("[CONE DEBUG] 丢失计数: %d/5\n", missingCount);

		if (missingCount >= 5)
		{
			barrelFound = false;
			Found_barrel++;
			detectCount = 0;
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

	if (cone_debug_frame_count % 10 == 0)
	{
		float detection_rate = (cone_detected_count * 100.0f) / cone_debug_frame_count;
		printf("[CONE STATS] 帧数: %d, 检测率: %.1f%%, 检测/丢失: %d/%d\n", 
		       cone_debug_frame_count, detection_rate, cone_detected_count, cone_missed_count);
	}

	return barrel_center;
}

/**
 * @brief 限制数值变化函数
 */
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

/**
 * @brief 后锥桶处理模式
 */
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

		circle(img, center_offset, 3, Scalar(0, 255, 255), -1);

		Left_Line = Line(slope_vertebral, center_offset);
		Right_Line = Line(m_right, cv::Point(x_right, 110));
		
		printf("[LATER CONE] LEFT模式: slope_vertebral=%.2f, center_offset=(%d, %d), offset=%d\n", 
		       slope_vertebral, center_offset.x, center_offset.y, offset);
	}
	else if (try_patching_line == RIGHT)
	{
		slope_vertebral = -m_left;
		center_offset = Point(x_left - offset, 110);

		circle(img, center_offset, 3, Scalar(0, 255, 255), -1);

		Left_Line = Line(m_left, cv::Point(x_left, 110));
		Right_Line = Line(slope_vertebral, center_offset);
		
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

/**
 * @brief 锥桶检测和补线函数
 */
void vertebral_model(Mat &img_)
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
		return;
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
		line(img, Point((99 - b_mid_blue) / m_mid_blue, 99), Point((60 - b_mid_blue) / m_mid_blue, 60), Scalar(255, 255, 0), 1);
	}
	circle(img, Point(barrel_center.x, barrel_center.y), 3, Scalar(0, 255, 255), -1);

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

	if (barrel_center.y < 180 && First_Find_vertebral_Flag)
	{
		printf("[CONE MODEL] 初见锥桶处理, 锥桶Y坐标: %d, flag_cross: %d\n", barrel_center.y, flag_cross);
		
		if (flag_cross == 2)
		{
			off = (try_patching_line == LEFT) ? 10 : -10;
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

			circle(img, center_offset, 3, Scalar(0, 255, 255), -1);

			Left_Line = Line(slope_vertebral, center_offset);
			Right_Line = Line(m_right, cv::Point(x_right, 110));
			
			printf("[CONE MODEL] LEFT模式: slope_vertebral=%.2f, center_offset=(%d, %d)\n", 
			       slope_vertebral, center_offset.x, center_offset.y);
		}
		else if (try_patching_line == RIGHT)
		{
			slope_vertebral = -m_left;
			center_offset = Point(x_left - offset, 110);

			circle(img, center_offset, 3, Scalar(0, 255, 255), -1);

			Left_Line = Line(m_left, cv::Point(x_left, 110));
			Right_Line = Line(slope_vertebral, center_offset);

			printf("[CONE MODEL] RIGHT模式: slope_vertebral=%.2f, center_offset=(%d, %d)\n", 
			       slope_vertebral, center_offset.x, center_offset.y);
		}

		Tracking_barrel(binaryImage_Finish);
	}
	else if (barrel_center.y > 180 && Later_vertebral_Flag == 0 && First_Find_vertebral_Flag)
	{
		Later_vertebral_Flag = 1;
		printf("[CONE MODEL] ★ 锥桶Y坐标>180, 设置后锥桶标志 Later_vertebral_Flag=1\n");
	}
}

// ============================================================================
// 6. 斑马线检测函数
// ============================================================================

/**
 * @brief 斑马线检测函数
 */
int crossroad(Mat frame)
{
	int height = frame.rows;
	int width = frame.cols;

	Mat hsv;
	cvtColor(frame, hsv, COLOR_BGR2HSV);

	Scalar lower_white = Scalar(0, 0, 221);
	Scalar upper_white = Scalar(180, 100, 255);

	Mat mask1;
	inRange(hsv, lower_white, upper_white, mask1);

	Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
	morphologyEx(mask1, mask1, MORPH_OPEN, kernel);

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
			if (src.at<char>(i, j - 2) == 0 && src.at<uchar>(i, j) == 0 && src.at<uchar>(i, j - 1) == 0 && src.at<uchar>(i, j + 1) == 255 && src.at<uchar>(i, j + 2) == 255)
			{
				cout1++;
			}
			else if (src.at<uchar>(i, j - 2) == 255 && src.at<uchar>(i, j) == 255 && src.at<uchar>(i, j - 1) == 255 && src.at<uchar>(i, j + 1) == 0 && src.at<uchar>(i, j + 2) == 0)
			{
				cout1++;
			}
			
			if (cout1 >= 10)
			{
				cout2++;
				flag++;
				if (flag >= 5)
				{
					if (crossroad_num == 0)
					{
						flag_cross = -1;
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
	return flag_cross;
}

// ============================================================================
// 7. 转向标志识别函数
// ============================================================================

/**
 * @brief 转向标志识别函数
 */
void Judge_Turn_Pro(Mat frame)
{
	Mat hsv;
	cvtColor(frame, hsv, COLOR_BGR2HSV);

	Scalar lower_blue(90, 29, 30);
	Scalar upper_blue(130, 255, 255);

	Mat mask;
	inRange(hsv, lower_blue, upper_blue, mask);

	vector<vector<Point>> contours;
	findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

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

		Mat matrix = getPerspectiveTransform(src_points, dst_points);
		Mat warped;
		warpPerspective(frame, warped, matrix, Size(300, 200));

		Mat gray_warped, thresh_warped;
		cvtColor(warped, gray_warped, COLOR_BGR2GRAY);
		threshold(gray_warped, thresh_warped, 127, 255, THRESH_BINARY);

		Mat white_mask;
		inRange(gray_warped, 220, 255, white_mask);
		Mat combined_mask;
		bitwise_and(thresh_warped, white_mask, combined_mask);

		Mat roi = combined_mask(Rect(0, 2 * combined_mask.rows / 3, combined_mask.cols, combined_mask.rows / 3)).clone();

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

			float vx = fitted_line[0], vy = fitted_line[1];
			float x0 = fitted_line[2], y0 = fitted_line[3];

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

			int left_count = countNonZero(left_mask);
			int right_count = countNonZero(right_mask);

			if (left_count >= right_count)
			{
				R_L_Flag_Temp = 1;
			}
			else
			{
				R_L_Flag_Temp = 2;
			}
		}

		if (R_L_Flag_Temp == 1)
		{
			BianDao_Num[0]++;
		}
		else if (R_L_Flag_Temp == 2)
		{
			BianDao_Num[1]++;
		}

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

// ============================================================================
// 8. 控制参数计算函数
// ============================================================================

/**
 * @brief 计算中线偏差
 */
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

// ============================================================================
// 9. 显示和通信函数
// ============================================================================

/**
 * @brief 显示处理信息
 */
void display_Information(void)
{
	int j = 0;
	for (int i = 239; i >= 101; i--)
	{
		circle(img, Point(int(left_line[j].x), i), 0.1, Scalar(0, 0, 0), 2);
		circle(img, Point(int(right_line[j].x), i), 0.1, Scalar(0, 0, 0), 2);
		circle(img, Point(int((left_line[j].x + right_line[j].x) / 2), i), 0.1, Scalar(255, 0, 0), 2);
		j++;
	}

	line(img, Point(160, 100), Point(160, 239), Scalar(0, 0, 255), 2);
	rectangle(img, Rect(1, 100, 319, 239), Scalar(0, 255, 0), 1);

	std::ostringstream oss;
	oss << "Bias_Line: " << static_cast<int>(Bias_Line);
	std::string text = oss.str();
	cv::putText(img, text, cv::Point(0, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);

	oss.str("");
	oss.clear();
	oss << "Track_width: " << static_cast<int>(x_right - x_left);
	text = oss.str();
	cv::putText(img, text, cv::Point(0, 80), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);

	oss.str("");
	oss.clear();
	switch (flag_cross)
	{
	case -1: oss << "flag_cross: find"; break;
	case 0: oss << "flag_cross: nofind"; break;
	case 1: oss << "flag_cross: stop_car"; break;
	case 2: oss << "vertebral: middleCone! (" << Found_barrel + 1 << ")"; break;
	case 3: oss << "vertebral: ordinary! (" << Found_barrel + 1 << ")"; break;
	case 4: oss << "  stop!"; break;
	}
	text = oss.str();
	cv::putText(img, text, cv::Point(0, 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1);

	oss.str("");
	oss.clear();
	switch (R_L_Flag)
	{
	case 0:
		oss << "R_L_Flag: To_zhi";
		break;
	case 1:
		arrowedLine(img, Point(150, 90), Point(100, 90), Scalar(0, 255, 0), 2, 8, 0, 0.5);
		oss << "R_L_Flag: To_Left";
		break;
	case 2:
		arrowedLine(img, Point(150, 90), Point(200, 90), Scalar(0, 255, 0), 2, 8, 0, 0.5);
		oss << "R_L_Flag: To_Right";
		break;
	default:
		oss << "R_L_Flag: Unknown";
		break;
	}
	text = oss.str();
	cv::putText(img, text, cv::Point(0, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1);
}

/**
 * @brief HSV参数调节回调函数
 */
void updateHSV(int, void *)
{
	inRange(hsvImg, Scalar(h_min, s_min, v_min), Scalar(h_max, s_max, v_max), outputImg);
	imshow("Output Image", outputImg);
}

#if ubuntu
/**
 * @brief 串口初始化
 */
int Serial_Init(void)
{
	const char *port = "/dev/ttyUSB0";
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

/**
 * @brief 数据发送函数
 */
int Data_Send(void)
{
	uint8_t buffer[5];
	memcpy(buffer, &Head, sizeof(int8_t));
	memcpy(buffer + 1, &Bias_Line, sizeof(int8_t));
	memcpy(buffer + 2, &flag_cross, sizeof(int8_t));
	memcpy(buffer + 3, &R_L_Flag, sizeof(int8_t));
	memcpy(buffer + 4, &Tail, sizeof(int8_t));

	printf("[UART DEBUG] 发送数据包: HEAD=0x%02X, Bias=%d, FlagCross=%d, R_L_Flag=%d, TAIL=0x%02X\n", 
	       Head, Bias_Line, flag_cross, R_L_Flag, Tail);

	int n = write(fd, buffer, sizeof(buffer));
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

/**
 * @brief 获取当前时间字符串
 */
std::string getCurrentTimeString()
{
	auto now = std::time(nullptr);
	std::tm *localTime = std::localtime(&now);
	std::ostringstream oss;
	oss << std::put_time(localTime, "%Y-%m-%d_%H-%M-%S");
	return oss.str();
}
#endif

// ============================================================================
// 10. 主函数
// ============================================================================

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

#if Select_Video_Frame == 1
#if ubuntu
	cv::VideoCapture capture("/dev/video0", cv::CAP_V4L2);
	cv::Size resolution(320, 240);
	capture.set(cv::CAP_PROP_FRAME_WIDTH, resolution.width);
	capture.set(cv::CAP_PROP_FRAME_HEIGHT, resolution.height);
#else
	VideoCapture capture;
	capture.open(video_path);
#endif

#if ubuntu
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
			outputPath = "/home/dgdz/桌面/原图/" + getCurrentTimeString() + ".mp4";
			std::cout << "开始录原图" << std::endl;
			handle = 2;
		}
		else if (param == "1")
		{
			outputPath = "/home/dgdz/桌面/处理图/" + getCurrentTimeString() + ".mp4";
			std::cout << "开始录处理图" << std::endl;
			handle = 1;
		}
		else
		{
			switch (argv[1][0])
			{
			case '1': Left_barrel_serial_number = 1; printf("Left_barrel_serial_number\n"); break;
			case '2': Right_barrel_serial_number = 1; printf("Right_barrel_serial_number\n"); break;
			default: Mid_barrel_serial_number = 1; printf("Mid_barrel_serial_number\n"); break;
			}
			switch (argv[1][1])
			{
			case '1': Left_barrel_serial_number = 2; printf("Left_barrel_serial_number\n"); break;
			case '2': Right_barrel_serial_number = 2; printf("Right_barrel_serial_number\n"); break;
			default: Mid_barrel_serial_number = 2; printf("Mid_barrel_serial_number\n"); break;
			}
			switch (argv[1][2])
			{
			case '1': Left_barrel_serial_number = 3; printf("Left_barrel_serial_number\n"); break;
			case '2': Right_barrel_serial_number = 3; printf("Right_barrel_serial_number\n"); break;
			default: Mid_barrel_serial_number = 3; printf("Mid_barrel_serial_number\n"); break;
			}
			cleverFlag = 1;
		}

		if (argv[1][0] == '6' || argv[1][0] == '1')
		{
			int frameWidth = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH));
			int frameHeight = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT));
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
		resize(img, img, Size(320, 240));
#endif

		if (handle == 2) writer.write(img);
		ImagePreprocessing_Sobel_LinePro(img);

		if (is_First_cross != 1 && Stop_Turn_Cross != 1)
		{
			vertebral_model(img);
		}

		if (Stop_Turn_Cross != 1)
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
		{
			FPS = frameCount / elapsed.count();
			frameCount = 0;
			start = std::chrono::high_resolution_clock::now();
		}

		std::ostringstream oss;
		oss << std::fixed << std::setprecision(2) << "FPS: " << FPS;
		std::string text = oss.str();
		cv::putText(img, text, cv::Point(230, 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
#endif

		if (handle == 1) writer.write(img);

		resize(img, img, cv::Size(), 2.5, 2.5);
		imshow("img_finall", img);

#if ubuntu
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
#elif Select_Video_Frame == 2
	img = imread(frame_path);
	Judge_Turn_Pro(img);
	imshow("img_finall", img);
	waitKey(0);
	return 0;
#elif Select_Video_Frame == 3
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
#else
string savePath = "D:/IDE/bike_video/video/nk6/";
VideoCapture cap;
cap.open(video_path);
int frameCount = 0;
int saveFrameInterval = 10;
Mat frame;
while (true)
{
	bool success = cap.read(frame);
	if (!success) break;
	if (frameCount % saveFrameInterval == 0)
	{
		stringstream ss;
		ss << savePath << "frame_" << frameCount << ".jpg";
		string filename = ss.str();
		imwrite(filename, frame);
	}
	frameCount++;
}
cap.release();
return 0;
#endif
}
}