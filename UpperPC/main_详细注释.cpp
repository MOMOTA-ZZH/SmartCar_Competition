/**
 * 室外无人驾驶自行车 - 上位机图像处理代码（新版本详细注释版）
 * 
 * 主要功能模块：
 * 1. 赛道边界检测和跟踪
 * 2. 锥桶检测和避障策略
 * 3. 斑马线检测
 * 4. 转向标志识别
 * 5. 串口通信控制
 * 
 * 开发环境：OpenCV 4.x, C++11
 * 硬件平台：工控机 + USB摄像头
 */

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
#include <termios.h>
#include <unistd.h>

// 平台定义：1=Ubuntu/Linux, 0=Windows
#define ubuntu (1)
#if ubuntu
#endif

using namespace cv;
using namespace std;

// ============================================================================
// 文件路径配置
// ============================================================================

string frame_path = "/home/dgdz/桌面/Test_Image/yellow.png";  // 测试图像路径
string video_path = "D:/1vscode/930/8.mp4";                  // 测试视频路径

// 函数声明
int Tracking(Mat dilated_image);              // 赛道边界跟踪
void Tracking_barrel(Mat dilated_image);      // 锥桶模式下的边界跟踪
void vertebral_model(Mat &img_);              // 锥桶检测和补线
void Later_vertebral_model();                 // 后锥桶处理模式

// ============================================================================
// 全局变量声明 - 赛道跟踪相关
// ============================================================================

float Track_width = 120;      // 赛道宽度基准值（像素）

// 赛道边界参数
float m_left = 0;             // 左边界直线斜率
float m_right = 0;            // 右边界直线斜率
float a_right = 0;            // 右边界角度（度）
float a_left = 0;             // 左边界角度（度）
float x_left = 0;             // 左边界在y=110处的x坐标
float x_right = 0;            // 右边界在y=110处的x坐标

// 蓝色边界参数（用于锥桶检测）
float m_mid_blue = 0;         // 蓝色中线斜率
float b_mid_blue = 0;         // 蓝色中线截距

int8_t Trackingflag = 1;      // 跟踪策略标志：0=策略1, 1=策略2, 2=策略3

// ============================================================================
// 串口通信协议定义
// ============================================================================

int8_t Head = 0XA5;           // 数据包头
int8_t Tail = 0X5A;           // 数据包尾
int8_t Length = 3;            // 数据长度

// ============================================================================
// 图像处理参数
// ============================================================================

int My_threshold = 0;          // 边缘检测阈值
int My_HoughLinesP = 50;       // 霍夫变换阈值

int Min_moderate = 95;         // 中等面积阈值（像素）
int Min_barrel_area = 130;     // 最小锥桶面积阈值（像素）

int brightness_threshold = 20; // 亮度阈值

// ============================================================================
// 控制标志位
// ============================================================================

int8_t flag_cross = 0;         // 斑马线标志位：-1找到,0未找到,1停车,2中间锥桶,3普通锥桶,4停止
int8_t Bias_Line;              // 中线偏差（像素）
int8_t R_L_Flag;               // 左右变向标志位：0直行,1左转,2右转
int8_t R_L_Flag_Temp;          // 临时转向标志
int Bias_Line_int = 0;         // 中线偏差整数值

// ============================================================================
// 图像矩阵定义
// ============================================================================

cv::Mat img;                    // 源图像
cv::Mat img_xiaoxi;            // 带有各种信息的图片
cv::Mat img_ROI;               // 感兴趣区域图像
cv::Mat binaryImage;           // 图像处理后的二值化图片
cv::Mat binaryImage_Finish;    // 最终提取赛道白线的图片
cv::Mat binaryImage12;         // 图像处理后的二值化图片

// ============================================================================
// 状态标志
// ============================================================================

uint8_t is_First_cross = 0;     // 首次识别人行道标志：增加至1时表示确定找到人行道
uint8_t barrel_find_finish = 0; // 是否继续寻找锥桶标志
uint8_t barrel_find_yes = 0;    // 锥桶识别标志位
uint8_t Stop_Turn_Cross = 0;    // 停止转向检测标志
uint8_t Find_Turn_Flag = 0;     // 找到转向标志

// ============================================================================
// 锥桶序列配置 - 智能识别机制核心
// ============================================================================

int Left_barrel_serial_number = 1;   // 左锥桶序列号
int Mid_barrel_serial_number = 2;    // 中间锥桶序列号  
int Right_barrel_serial_number = 3;  // 右锥桶序列号

// ============================================================================
// 边界点存储容器
// ============================================================================

std::vector<cv::Point> left_line;      // 左边界点集
std::vector<cv::Point> right_line;     // 右边界点集
std::vector<cv::Point> mid;            // 中线点集
std::vector<cv::Point> left_line_blue; // 蓝色左边界点集
std::vector<cv::Point> right_line_blue;// 蓝色右边界点集
std::vector<cv::Point> mid_blue;       // 蓝色中线点集

// ============================================================================
// 函数实现
// ============================================================================

/**
 * @brief 定义梯形感兴趣区域(ROI)
 * 
 * 创建一个梯形掩膜，只保留摄像头视野中有效的赛道区域，
 * 排除天空、车辆本体等干扰区域
 * 
 * @param src 输入彩色图像
 * @return 梯形ROI掩膜图像，只保留梯形区域，其他区域为黑色
 */
Mat defineTrapezoidROI(Mat src)
{
    // 创建一个黑色的空白遮罩，大小与输入图像相同
    Mat mask = Mat::zeros(src.size(), src.type());

    int height = src.rows;
    int width = src.cols;

    // 梯形顶点定义（根据摄像头视角调整）
    Point top_left(0, 210);           // 左下角
    Point top_right(320, 210);        // 右下角
    Point top_middle_left(100, 30);   // 左中上角
    Point top_middle_right(160, 30);  // 右中上角
    Point bottom_right(320, 240);     // 右底部
    Point bottom_left(0, 240);        // 左底部

    // 将六个点存储在一个数组中，作为多边形顶点
    vector<Point> trapezoid = {top_left, top_middle_left, top_middle_right, 
                              top_right, bottom_right, bottom_left};

    // 在遮罩上绘制梯形区域
    vector<vector<Point>> contours;
    contours.push_back(trapezoid);

    // 填充多边形区域，白色代表感兴趣区域
    fillPoly(mask, contours, Scalar(255, 255, 255));

    // 将遮罩应用于原图像，只保留梯形区域
    Mat maskedImage;
    bitwise_and(src, mask, maskedImage);

    // 绘制梯形轮廓，方便显示感兴趣区域的边界
    polylines(src, contours, true, Scalar(255, 255, 255), 1);

    return maskedImage;
}

/**
 * @brief 主图像预处理函数 - Sobel边缘检测 + 霍夫线段检测
 * 
 * 处理流程：
 * 1. 图像灰度化
 * 2. 高斯模糊降噪
 * 3. 中值滤波去除椒盐噪声
 * 4. 形态学开运算优化边缘
 * 5. Sobel边缘检测
 * 6. 霍夫变换检测线段
 * 7. 多策略跟踪选择
 * 
 * @param frame_a 输入彩色图像
 * @return 处理后的二值图像（包含赛道边界）
 */
Mat ImagePreprocessing_Sobel_LinePro(Mat frame_a)
{
    int width = frame_a.cols;
    int height = frame_a.rows;

    // 创建多个二值图像用于不同处理阶段
    binaryImage_Finish = cv::Mat::zeros(height, width, CV_8U);   // 最终输出图像
    cv::Mat binaryImage_Finish3 = cv::Mat::zeros(height, width, CV_8U); // 保底处理图像
    cv::Mat binaryImage_Finish4 = cv::Mat::zeros(height, width, CV_8U); // 处理图像1
    cv::Mat binaryImage_Finish5 = cv::Mat::zeros(height, width, CV_8U); // 处理图像2

    Mat binaryImage12;
    
    // === 图像预处理 ===
    // 转换为灰度图
    Mat originalImage;
    cvtColor(frame_a, originalImage, cv::COLOR_BGR2GRAY);

    // 高斯模糊降噪
    Mat enhancedImage = originalImage;
    GaussianBlur(enhancedImage, enhancedImage, Size(5, 5), 2);

    // 中值滤波去除椒盐噪声
    medianBlur(enhancedImage, enhancedImage, 5);

    // 形态学开运算优化边缘
    Mat kernel1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
    cv::Mat morphedImage;
    morphologyEx(enhancedImage, morphedImage, MORPH_OPEN, kernel1);

    // 膨胀操作连接断开的边缘
    cv::dilate(morphedImage, morphedImage, kernel1, cv::Point(-1, -1), 1);

    // 再次中值滤波确保噪声去除
    medianBlur(morphedImage, morphedImage, 5);

    // === Sobel边缘检测 ===
    Mat sobelx, sobely;
    Sobel(morphedImage, sobelx, CV_64F, 1, 0, 3);  // x方向梯度
    Sobel(morphedImage, sobely, CV_64F, 0, 1, 3);  // y方向梯度
    convertScaleAbs(sobelx, sobelx);               // 转换为8位无符号整数
    convertScaleAbs(sobely, sobely);

    // 合并梯度幅值
    Mat gradientMagnitude;
    addWeighted(sobelx, 0.5, sobely, 0.5, 0, gradientMagnitude);

    // 二值化处理：使用大津法自动计算阈值
    binaryImage = Mat::zeros(morphedImage.size(), CV_8U);
    cv::threshold(gradientMagnitude, binaryImage, My_threshold, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
    
    // 形态学膨胀连接断开的边缘
    Mat kernel2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::dilate(binaryImage, binaryImage, kernel2, cv::Point(-1, -1), 1);

    // === 霍夫线段检测 ===
    vector<Vec4i> lines;  // 存储检测到的线段
    int minLineLength = 20;  // 最小线段长度
    int maxLineGap = 10;     // 最大线段间隙

    // 定义感兴趣区域(ROI)：只处理图像中部（100-140）的赛道区域
    int x_roi = 1;
    int y_roi = 100;
    int width_roi = 319;
    int height_roi = 140;
    Rect roi(x_roi, y_roi, width_roi, height_roi);
    Mat croppedObject = binaryImage(roi);  // 提取ROI区域进行霍夫变换

    // 霍夫变换检测直线
    HoughLinesP(croppedObject, lines, 1, CV_PI / 180, My_HoughLinesP, minLineLength, maxLineGap);

    // 线段过滤和分类
    for (size_t i = 0; i < lines.size(); i++) {
        Vec4i l = lines[i];
        
        // 计算线段参数
        float k = static_cast<float>(l[3] - l[1]) / (l[2] - l[0]);  // 斜率
        float angle_k = std::atan(k) * 180.0 / CV_PI;               // 角度(度)
        float b = l[1] - k * l[0];                                  // 截距
        float x6 = (10 - b) / k;                                    // 在y=10处的x坐标
        float length = sqrt(pow(l[3] - l[1], 2) + pow(l[2] - l[0], 2));  // 线段长度

        // 根据角度和长度过滤线段
        if (abs(angle_k) > 30 && length > 30) {
            // 第一种分类：基础线段（保底策略）
            l[0] += x_roi; l[1] += y_roi; l[2] += x_roi; l[3] += y_roi;
            line(binaryImage_Finish3, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 2, LINE_AA);
            l[0] -= x_roi; l[1] -= y_roi; l[2] -= x_roi; l[3] -= y_roi;

            // 第二种分类：中等角度的长线段（策略1）
            if (abs(angle_k) < 80 && length > 30 && (l[1] > 55 || l[3] > 55) && 
                (l[1] < 15 || l[3] < 15) && 
                ((l[0] + l[2] < 319) ? (k < 0 && abs(angle_k - a_left) < 18 && abs(115 - x_right + x6) < 10) : 
                 (k > 0 && abs(angle_k - a_right) < 18 && abs(115 + x_left - x6) < 10))) {
                l[0] += x_roi; l[1] += y_roi; l[2] += x_roi; l[3] += y_roi;
                line(binaryImage_Finish4, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 2, LINE_AA);
                l[0] -= x_roi; l[1] -= y_roi; l[2] -= x_roi; l[3] -= y_roi;
            }

            // 第三种分类：大角度的长线段（策略2）
            if (abs(angle_k) < 85 && length > 30 && (l[1] > 65 || l[3] > 65) && 
                (l[1] < 8 || l[3] < 8) && 
                ((l[0] < 160 + 20 || l[2] < 160 + 20) ? k < 0 : 1) && 
                ((l[0] > 160 - 20 || l[2] > 160 - 20) ? k > 0 : 1)) {
                l[0] += x_roi; l[1] += y_roi; l[2] += x_roi; l[3] += y_roi;
                line(binaryImage_Finish5, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 2, LINE_AA);
                l[0] -= x_roi; l[1] -= y_roi; l[2] -= x_roi; l[3] -= y_roi;
            }
        }
    }

    // === 多策略跟踪选择 ===
    // 使用状态机选择最佳跟踪策略，提高系统鲁棒性
Tracking_start:
    switch (Trackingflag) {
    case 0:  // 策略1：使用binaryImage_Finish4
        if (Tracking(binaryImage_Finish4)) {
            binaryImage_Finish = binaryImage_Finish4;
            return binaryImage_Finish;
        } else {
            Trackingflag = 1;  // 策略1失败，切换到策略2
            goto Tracking_start;
        }
        break;

    case 1:  // 策略2：使用binaryImage_Finish5
        if (Tracking(binaryImage_Finish5)) {
            Trackingflag = 0;  // 策略2成功，下次使用策略1
            binaryImage_Finish = binaryImage_Finish5;
            return binaryImage_Finish;
        } else {
            Trackingflag = 2;  // 策略2失败，切换到策略3
            goto Tracking_start;
        }
        break;

    case 2:  // 策略3：使用binaryImage_Finish3（保底策略）
        Tracking(binaryImage_Finish3);
        binaryImage_Finish = binaryImage_Finish3;
        Trackingflag = 0;  // 重置为策略1
        return binaryImage_Finish;
        break;
    }
    
    return binaryImage_Finish;
}

/**
 * @brief 蓝色边界检测函数（用于锥桶检测）
 * 
 * 在锥桶检测区域进行额外的边界检测，用于判断锥桶相对于赛道的位置
 * 
 * @param frame_a 输入图像
 */
void Sobel_LinePro(Mat frame_a)
{
    int width = frame_a.cols;
    int height = frame_a.rows;

    // 创建用于存储霍夫线段的图像
    cv::Mat binaryImage_Finish3 = cv::Mat::zeros(height, width, CV_8U);

    // 霍夫线段检测
    vector<Vec4i> lines;
    int minLineLength = 30;
    int maxLineGap = 10;

    // 定义锥桶检测的ROI区域
    int x_roi = 50;
    int y_roi = 60;
    int width_roi = 200;
    int height_roi = 50;
    Rect roi(x_roi, y_roi, width_roi, height_roi);
    Mat croppedObject = binaryImage(roi);

    // 霍夫变换检测线段
    HoughLinesP(croppedObject, lines, 1, CV_PI / 180, My_HoughLinesP, minLineLength, maxLineGap);

    // 绘制检测到的线段
    for (size_t i = 0; i < lines.size(); i++) {
        Vec4i l = lines[i];
        
        float k = static_cast<float>(l[3] - l[1]) / (l[2] - l[0]);
        float angle_k = std::atan(k) * 180.0 / CV_PI;
        float b = l[1] - k * l[0];
        float x6 = (10 - b) / k;
        float length = sqrt(pow(l[3] - l[1], 2) + pow(l[2] - l[0], 2));

        // 过滤符合条件的线段
        if (abs(angle_k) > 30 && length > 25) {
            if ((l[1] < 4 || l[3] < 4) && (l[1] > 15 || l[3] > 15)) {
                l[0] += x_roi; l[1] += y_roi; l[2] += x_roi; l[3] += y_roi;
                line(binaryImage_Finish3, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 2, LINE_AA);
                l[0] -= x_roi; l[1] -= y_roi; l[2] -= x_roi; l[3] -= y_roi;
            }
        }
    }

    // 边界跟踪
    int begin = 160;
    left_line_blue.clear();
    right_line_blue.clear();
    mid_blue.clear();

    for (int i = 99; i >= 61; i--) {
        int find_l = 0, find_r = 0;
        int to_left = begin, to_right = begin;

        // 查找左边线
        while (to_left > 1) {
            if (binaryImage_Finish3.at<uchar>(i, to_left) == 255) {
                find_l = 1;
                left_line_blue.emplace_back(to_left, i);
                break;
            }
            to_left--;
        }

        // 查找右边线
        while (to_right < 319) {
            if (binaryImage_Finish3.at<uchar>(i, to_right) == 255) {
                find_r = 1;
                right_line_blue.emplace_back(to_right, i);
                break;
            }
            to_right++;
        }

        // 计算中线点
        if (!left_line_blue.empty() && !right_line_blue.empty()) {
            cv::Point midx1 = left_line_blue.back();
            cv::Point midx2 = right_line_blue.back();
            mid_blue.emplace_back((midx1.x + midx2.x) / 2, i);
            begin = (to_right + to_left) / 2;
        }
    }

    // 直线拟合
    cv::Vec4f mid_fit;
    if (left_line_blue.empty() || right_line_blue.empty()) {
        return;
    }

    cv::fitLine(mid_blue, mid_fit, cv::DIST_L2, 0, 0.01, 0.01);
    m_mid_blue = mid_fit[1] / mid_fit[0];
    b_mid_blue = mid_fit[3] - m_mid_blue * mid_fit[2];
}

// 由于代码长度限制，这里只展示了部分函数的详细注释
// 完整的详细注释版本需要继续为剩余函数添加注释

// 注意：由于代码长度限制，这里只展示了部分核心函数的详细注释
// 在实际使用中，需要为所有函数添加类似的详细注释
