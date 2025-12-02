#include <opencv2/opencv.hpp>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <vector>
#include <string>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <algorithm> // 包含 std::clamp
#include <cmath>

using namespace cv;
using namespace std;

// 测试文件路径（开发调试用）
string frame_path = "D:/IDE/bike_video/video/nk6/frame_450.jpg";
string video_path = "D:/IDE/bike_video/video/锥桶11.mp4";

// 函数声明
int Tracking(Mat dilated_image);              // 赛道边界跟踪
void Tracking_barrel(Mat dilated_image);      // 锥桶模式下的边界跟踪

//////////////////////////////////////////////////////////////////////////////////////////
// 全局变量声明
float m_left = 0;      // 左边界直线斜率
float m_right = 0;     // 右边界直线斜率
float a_right = 0;     // 右边界角度（度）
float a_left = 0;      // 左边界角度（度）
float x_left = 0;      // 左边界在y=110处的x坐标
float x_right = 0;     // 右边界在y=110处的x坐标
int8_t Trackingflag = 1; // 跟踪策略标志

// 串口通信协议定义
int8_t Head = 0XA5;    // 数据包头
int8_t Tail = 0X5A;    // 数据包尾
int8_t Length = 3;     // 数据长度

// 图像处理参数
int My_threshold = 0;          // 边缘检测阈值
int My_HoughLinesP = 50;       // 霍夫变换阈值
int Min_barrel_area = 70;      // 最小锥桶面积阈值
int brightness_threshold = 20; // 亮度阈值

// 控制标志位
int8_t flag_cross = 0;         // 斑马线标志位：-1找到，0未找到，1停车，2中间锥桶，3普通锥桶，4停止
int8_t Bias_Line;              // 中线偏差（像素）
int8_t R_L_Flag;               // 左右变向标志位：0直行，1左转，2右转
int8_t R_L_Flag_Temp;          // 临时转向标志
int Bias_Line_int = 0;         // 中线偏差整数值

// 图像矩阵定义
cv::Mat img;                    // 源图像
cv::Mat img_xiaoxi;             // 带有各种信息的图片
cv::Mat img_ROI;                // 感兴趣区域图像
cv::Mat binaryImage;            // 图像处理后的二值化图片
cv::Mat binaryImage_Finish;     // 最终提取赛道白线的图片
cv::Mat binaryImage_Finish2;    // 备用二值图像

// 状态标志
uint8_t is_First_cross = 0;     // 首次识别人行道标志
uint8_t barrel_find_finish = 0; // 是否继续寻找锥桶标志
uint8_t barrel_find_yes = 0;    // 锥桶识别标志位
uint8_t Stop_Turn_Cross = 0;    // 停止转向检测标志
uint8_t Find_Turn_Flag = 0;     // 找到转向标志

////////////////////////////////////////////////////
// 边界点存储容器
std::vector<cv::Point> left_line;    // 左边界点集
std::vector<cv::Point> right_line;   // 右边界点集
std::vector<cv::Point> mid;          // 中线点集
std::vector<cv::Point> left_line_blue;  // 蓝色左边界点集
std::vector<cv::Point> right_line_blue; // 蓝色右边界点集
std::vector<cv::Point> mid_blue;        // 蓝色中线点集
/////////////////////////////////////////////////////

/////////////////////////////////////////////////////
// 蓝色锥桶HSV颜色范围参数
int H_low_Blue = 106;   // 色相下限
int H_high_Blue = 176;  // 色相上限
int S_low_Blue = 69;    // 饱和度下限
int S_high_Blue = 255;  // 饱和度上限
int V_low_Blue = 18;    // 亮度下限
int V_high_Blue = 255;  // 亮度上限

cv::Mat blue_vertebral; // 蓝色锥桶处理图像
/////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief 在二值图像上绘制白色线段
 * @param binaryImage 输入二值图像
 * @param start 线段起点
 * @param end 线段终点
 * @param lineWidth 线段宽度
 * @return 绘制后的图像
 */
cv::Mat drawWhiteLine(Mat binaryImage, cv::Point start, cv::Point end, int lineWidth)
{
    cv::Mat resultImage = binaryImage.clone();

    int x1 = start.x, y1 = start.y;
    int x2 = end.x, y2 = end.y;

    // 垂直线段处理
    if (x1 == x2)
    {
        for (int y = std::min(y1, y2); y <= std::max(y1, y2); ++y)
        {
            for (int i = -lineWidth / 2; i <= lineWidth / 2; ++i)
            {
                resultImage.at<uchar>(cv::Point(x1 + i, y)) = 255;
            }
        }
    }
    else
    {
        // 斜线段处理
        double slope = static_cast<double>(y2 - y1) / (x2 - x1);
        double intercept = y1 - slope * x1;

        for (int x = std::min(x1, x2); x <= std::max(x1, x2); ++x)
        {
            int y = static_cast<int>(slope * x + intercept);
            for (int i = -lineWidth / 2; i <= lineWidth / 2; ++i)
            {
                int newY = std::max(0, std::min(y + i, resultImage.rows - 1));
                resultImage.at<uchar>(cv::Point(x, newY)) = 255;
            }
        }
    }

    return resultImage;
}

/**
 * @brief 定义梯形感兴趣区域(ROI)
 * @param src 输入图像
 * @return 梯形ROI掩膜图像
 */
Mat defineTrapezoidROI(Mat src)
{
    // 创建一个黑色的空白遮罩，大小与输入图像相同
    Mat mask = Mat::zeros(src.size(), src.type());

    // 定义感性区域的六个顶点（梯形区域）
    int height = src.rows;
    int width = src.cols;

    // 梯形顶点定义（根据摄像头视角调整）
    Point top_left(0, 210);        // 左下角
    Point top_right(320, 210);     // 右下角
    Point top_middle_left(110, 30);  // 左中上角
    Point top_middle_right(210, 30); // 右中上角
    Point bottom_right(320, 240);  // 右底部
    Point bottom_left(0, 240);     // 左底部

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

    return maskedImage;
}

/**
 * @brief 计算两点之间的欧几里得距离
 * @param p1 点1
 * @param p2 点2
 * @return 两点距离
 */
float calculateDistance(Point2f p1, Point2f p2)
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

/**
 * @brief 计算点到直线的距离
 * @param x0 点x坐标
 * @param y0 点y坐标
 * @param k 直线斜率
 * @param b 直线截距
 * @return 点到直线距离
 */
float pointToLineDistance(float x0, float y0, float k, float b)
{
    return fabs(k * x0 - y0 + b) / sqrt(k * k + 1);
}

/**
 * @brief 计算直线的斜率
 * @param line 直线端点坐标
 * @return 直线斜率
 */
float calculateSlope(Vec4i line)
{
    return static_cast<float>(line[3] - line[1]) / (line[2] - line[0]);
}

// 赛道宽度检测参数
uint16_t Gap_Heigth_Top = 80;         // 上部检测线y坐标
uint16_t Gap_Heigth_Lower = 220;      // 下部检测线y坐标
uint16_t Gap_Distance_TopMin = 80;    // 上部最小距离
uint16_t Gap_Distance_TopMax = 200;   // 上部最大距离
uint16_t Gap_Distance_Lower = 250;    // 下部距离阈值

/**
 * @brief 计算两条直线在指定高度处的横向距离，用于 判断并筛选 霍夫线段检测后 两条线是否构成合理的赛道边界
 * @param L1 直线1参数（端点、斜率、截距、距离）
 * @param L2 直线2参数
 * @param tapy 高度类型：0上部，1下部
 * @return 两条直线在指定高度处的横向距离
 */
float getLineDistance(tuple<Vec4i, float, float, float> L1, tuple<Vec4i, float, float, float> L2, uint8_t tapy)
{
    uint16_t y = 0;
    if (tapy == 0)
    {
        y = Gap_Heigth_Top;    // 使用上部检测线
    }
    else
    {
        y = Gap_Heigth_Lower;  // 使用下部检测线
    }
    
    // 计算两条直线在高度y处的x坐标
    float x1 = (y - get<2>(L1)) / get<1>(L1);
    float x2 = (y - get<2>(L2)) / get<1>(L2);

    // 返回它们的横坐标差（绝对值）
    return fabs(x1 - x2);
}

/**
 * @brief 限制数值在0-319范围内（图像宽度）
 * @param value 输入值
 * @return 限制后的值
 */
float limit(float value)
{
    return std::max(0.0f, std::min(value, 319.0f));
}

/**
 * @brief 主图像预处理函数 原始图像 → 图像增强（归一化后伽马校正，提高对比度；转灰度图后高斯模糊降噪） 
 * → 边缘检测（sobel算法的梯度图，再经大津法提取边界得到二值化图像） 
 * → 霍夫线段检测（roi裁剪后的二值化图像经霍夫检测后得到 图像中各有效直线线段起点终点的横纵坐标）
 * → 线段分类（用长度和角度筛选后，依据线段可靠度（在图像中的位置、线段长短、线段角度等）分为三类，并在三张原来的底图上画出相应的赛道边线，对应三种不同的赛道边线追踪策略） 
 * → 多策略跟踪、输出边界（根据现有的赛道图像，返回能拥有的最好的带赛道边线的二值化图像）
 * @param frame_a 输入彩色图像
 * @return 处理后的二值图像（包含赛道边界）
 */
Mat ImagePreprocessing_Sobel_LinePro(Mat frame_a)
{
    int width = frame_a.cols;
    int height = frame_a.rows;

    // 创建多个二值图像用于不同处理阶段
    binaryImage_Finish = cv::Mat::zeros(height, width, CV_8U);   // 最终输出图像
    binaryImage_Finish2 = cv::Mat::zeros(height, width, CV_8U);  // 备用图像
    cv::Mat binaryImage_Finish3 = cv::Mat::zeros(height, width, CV_8U); // 保底处理图像
    cv::Mat binaryImage_Finish4 = cv::Mat::zeros(height, width, CV_8U); // 处理图像1
    cv::Mat binaryImage_Finish5 = cv::Mat::zeros(height, width, CV_8U); // 处理图像2

    Mat binaryImage12;
    
    // === 图像增强处理 ===
    // 伽马校正：增强图像对比度，适应不同光照条件
    Mat normalized_image;
    frame_a.convertTo(normalized_image, CV_32F, 1.0 / 255, 0);     // 像素归一化到0-1，使后续的伽马校正变得稳定且精确
    int gamma = 3; 
    Mat gamma_corrected;                
    pow(normalized_image, gamma, gamma_corrected);  // 伽马变换 
    /* 
    当 gamma > 1 时：低亮度像素被拉伸，高亮度像素被压缩，实现增强暗部细节，抑制过亮区域，增强图像对比度；
    当 gamma < 1 时：亮部区域增强，暗部区域压缩；
    暗部：0.1 → 0.1³ = 0.001
    中灰：0.5 → 0.5³ = 0.125  
    亮部：0.9 → 0.9³ = 0.729
    1.适应不同光照：
    - 阴天或阴影下，图像整体偏暗
    - 伽马校正可以增强赛道边界的可见性
    - 抑制过亮区域（如反光）的干扰
    2.突出赛道边缘：
    - 赛道白线与路面形成强烈对比
    - 伽马校正使这种对比更加明显
    */ 
    gamma_corrected.convertTo(gamma_corrected, CV_8U, 255, 0);  // 将归一化图像经伽马校正后 缩回得到校正后的图

    // 转换为灰度图，转换前为BGR彩色图像，也即8位无符号整数的3通道图像（CV_8UC3）
    Mat originalImage;
    cvtColor(gamma_corrected, originalImage, cv::COLOR_BGR2GRAY);

    // 亮度检测：如果平均亮度太低，设置停止标志，防止发生误检测或事故
    double mean_brightness = cv::mean(originalImage)[0];
    if (mean_brightness < brightness_threshold) 
    {
        flag_cross = 4; 
    }

    // === 边缘检测 ===
    Mat enhancedImage = originalImage;
    GaussianBlur(enhancedImage, enhancedImage, Size(5, 5), 1.5);  // 高斯模糊降噪

    // Sobel边缘检测：计算x和y方向的梯度，通过卷积运算检测图像中的边缘，并得出最终梯度图
    Mat sobelx, sobely;
    Sobel(enhancedImage, sobelx, CV_64F, 1, 0, 3);  // x方向梯度 
    Sobel(enhancedImage, sobely, CV_64F, 0, 1, 3);  // y方向梯度
    convertScaleAbs(sobelx, sobelx);  // 转换为8位无符号整数
    convertScaleAbs(sobely, sobely);

    // 合并梯度幅值
    Mat gradientMagnitude;    //最终梯度图
    addWeighted(sobelx, 0.5, sobely, 0.5, 0, gradientMagnitude);

    // 二值化处理：使用大津法计算阈值，阈值与梯度图作比较，也即像素变化率大于阈值（即边缘）的图像部分转为白色，小于的转为黑色
    binaryImage12 = Mat::zeros(enhancedImage.size(), CV_8U);  //创建一个全黑的二值化图像矩阵，我感觉是冗余的
    cv::threshold(gradientMagnitude, binaryImage12, My_threshold, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);

    // 形态学膨胀：连接断开的边缘
    cv::Mat kernel1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(8, 8));
    cv::dilate(binaryImage12, binaryImage, kernel1, cv::Point(-1, -1), 1);

  

    // 定义感兴趣区域(ROI)：只处理图像中部（100-140）的赛道区域
    int x_roi = 1;
    int y_roi = 100;
    int width_roi = 319;
    int height_roi = 140;
    Rect roi(x_roi, y_roi, width_roi, height_roi);
    Mat croppedObject = binaryImage12(roi);

    // === 霍夫线段检测 ===
    vector<Vec4i> lines;  // 存储检测到的线段
    int minLineLength = 30;  // 最小线段长度
    int maxLineGap = 10;     // 最大线段间隙
    // 霍夫变换检测直线，输出 各个线段 起点和终点坐标的集合，vector<Vec4i>是OpenCV中定义的4维整数向量类型，如lines[0] → [x1, y1, x2, y2]
    HoughLinesP(croppedObject, lines, 1, CV_PI / 180, My_HoughLinesP, minLineLength, maxLineGap);
    /*
    croppedObject：输入的二值边缘图像（ROI区域）
    lines：输出线段容器，每个元素是Vec4i(x1,y1,x2,y2)
    1：距离分辨率（像素）
    CV_PI/180：角度分辨率（弧度）
    My_HoughLinesP：累加器阈值（代码中=50）
    minLineLength：最小线段长度
    maxLineGap：最大线段间隙
    */

    // 线段过滤和分类
    for (size_t i = 0; i < lines.size(); i++) {
        Vec4i l = lines[i];           //l是一个线段的起点和终点的坐标，而l[0]代表起点x坐标，l[1]为起点y坐标
        
        // 计算线段参数
        float k = static_cast<float>(l[3] - l[1]) / (l[2] - l[0]);  // 斜率
        float angle_k = std::atan(k) * 180.0 / CV_PI;  // 角度(度)
        float b = l[1] - k * l[0];  // 截距
        float x6 = (10 - b) / k;    // 在y=10处的x坐标
        float length = sqrt(pow(l[3] - l[1], 2) + pow(l[2] - l[0], 2));  // 线段长度

        // 根据角度和长度过滤线段
        if (abs(angle_k) > 30 && length > 50) {
            // 第一种分类：靠近图像顶部的线段
            if ((l[1] < 8 || l[3] < 8) && (l[1] > 30 || l[3] > 30)) {
                l[0] += x_roi; l[1] += y_roi; l[2] += x_roi; l[3] += y_roi;
                line(binaryImage_Finish3, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 2, LINE_AA);
                l[0] -= x_roi; l[1] -= y_roi; l[2] -= x_roi; l[3] -= y_roi;
            }

            // 第二种分类：中等角度的长线段
            if (abs(angle_k) < 80 && length > 60 && (l[1] > 55 || l[3] > 55) && 
                (l[1] < 15 || l[3] < 15) && 
                ((l[0] + l[2] < 319) ? (k < 0 && abs(angle_k - a_left) < 18 && abs(115 - x_right + x6) < 18) : 
                 (k > 0 && abs(angle_k - a_right) < 18 && abs(115 + x_left - x6) < 18))) {
                l[0] += x_roi; l[1] += y_roi; l[2] += x_roi; l[3] += y_roi;
                line(binaryImage_Finish4, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 2, LINE_AA);
                l[0] -= x_roi; l[1] -= y_roi; l[2] -= x_roi; l[3] -= y_roi;
            }

            // 第三种分类：大角度的长线段
            if (abs(angle_k) < 85 && length > 60 && (l[1] > 65 || l[3] > 65) && 
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
    // 使用状态机选择最佳跟踪策略
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

// 其他图像预处理函数（简化版，用于不同场景）
Mat ImagePreprocessing_Sobel2(Mat frame_a) { /* 实现略 */ }
Mat ImagePreprocessing_Sobel(Mat frame_a) { /* 实现略 */ }

// 斑马线检测相关变量
int crossroad_num = 0;  // 斑马线检测次数
Rect roi_rect;          // 斑马线检测区域

/**
 * @brief 斑马线检测函数
 * @param frame 输入图像
 * @return 斑马线检测结果标志
 */
int crossroad(Mat frame)
{
    int height = frame.rows;
    int width = frame.cols;

    // 将图像转换为HSV颜色空间
    Mat hsv;
    cvtColor(frame, hsv, COLOR_BGR2HSV);

    // 定义白色的HSV范围
    Scalar lower_white = Scalar(0, 0, 221);
    Scalar upper_white = Scalar(180, 30, 255);

    // 创建白色掩码
    Mat mask1;
    inRange(hsv, lower_white, upper_white, mask1);

    // 形态学操作去除噪声
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(mask1, mask1, MORPH_OPEN, kernel); // 开操作

    // 根据检测次数调整ROI区域
    if (crossroad_num == 0) {
        roi_rect = Rect(1, 100, 319, 60);  // 第一次检测使用上部区域
    } else {
        roi_rect = Rect(1, 210, 319, 30);  // 后续检测使用下部区域
        flag_cross = 0;
    }

    Mat src(mask1, roi_rect);

    // 斑马线条纹计数
    int cout1 = 0, cout2 = 0, flag = 0;
    for (int i = 0; i < src.rows; i++) {
        if (cout1 < 10) flag = 0;
        cout1 = 0;
        
        for (int j = 10; j < src.cols - 10; j++) {
            // 检测黑白交替模式（斑马线特征）
            if (src.at<char>(i, j - 2) == 0 && src.at<uchar>(i, j) == 0 && 
                src.at<uchar>(i, j - 1) == 0 && src.at<uchar>(i, j + 1) == 255 && 
                src.at<uchar>(i, j + 2) == 255) {
                cout1++;
            } else if (src.at<uchar>(i, j - 2) == 255 && src.at<uchar>(i, j) == 255 && 
                       src.at<uchar>(i, j - 1) == 255 && src.at<uchar>(i, j + 1) == 0 && 
                       src.at<uchar>(i, j + 2) == 0) {
                cout1++;
            }
            
            // 斑马线条数大于阈值，计有效一次
            if (cout1 >= 10) {
                cout2++;
                flag++;
                if (flag >= 5) {  // 连续多行检测到斑马线
                    if (crossroad_num == 0) {
                        flag_cross = -1;  // 找到斑马线
                    } else {
                        is_First_cross = 1;  // 首次确认斑马线
                        flag_cross = 1;      // 停车标志
                    }
                }
                break;
            }
        }
        if (flag_cross == 1 || flag_cross == -1) break;
    }
    
    // 斑马线确认逻辑
    static int flag_cross_num = 0;
    if (flag_cross == -1) {
        flag_cross_num++;
        if (flag_cross_num % 3 == 0) {
            crossroad_num = 1;  // 确认斑马线存在
        }
    }
    
    return flag_cross;
}

// 锥桶检测相关定义
#define LEFT 1     // 左锥桶
#define RIGHT 2    // 右锥桶
#define MIDDLE 3   // 中锥桶

vector<Point> previousCenters;  // 历史锥桶中心点（用于平滑）
size_t maxHistory = 5;          // 最大历史帧数

// 锥桶检测状态变量
int Frame_zhen = 60;              // 锥桶跟踪帧数
int detectCount = 0;              // 连续检测到合理轮廓的帧数
int missingCount = 0;             // 轮廓消失后的帧数
bool barrelFound = false;         // 是否检测到合理轮廓的标志
int Found_barrel = 0;             // 已经找到的锥桶的数量
uint8_t try_patching_line = 0;    // 补线尝试标志
int x_image_bottom = 0;           // 图像底部x坐标
int x_image_top = 0;              // 图像顶部x坐标
int Later_vertebral_Flag = 0;     // 后锥桶标志
Point barrel_center_Temp;         // 临时锥桶中心
float slope_vertebral = 0;        // 锥桶补线斜率
int First_Find_vertebral_Flag = 0;// 首次找到锥桶标志
Point Temp_vertebral_Point;       // 临时锥桶点
int offset = 0;                   // 补线偏移量
int off = 0;                      // 临时偏移量

// 直线结构体定义
struct Line {
    float slope;                  // 直线的斜率
    cv::Point startPoint;         // 直线的起点
    Line(float s, cv::Point start) : slope(s), startPoint(start) {}
} Left_Line(0, cv::Point(0, 0)), Right_Line(0, cv::Point(0, 0)); // 初始化左右两条线

/**
 * @brief 蓝色锥桶检测函数
 * @param frame 输入图像
 * @return 锥桶中心点坐标
 */
Point blue_vertebral_barrel_find(Mat frame) {
    Point barrel_center = Point(0, 0);  // 初始化锥桶中心

    // === HSV颜色空间转换 ===
    Mat hsvImage;
    cvtColor(frame, hsvImage, COLOR_BGR2HSV);

    // 对比度增强：对V通道进行CLAHE处理
    vector<Mat> hsvChannels;
    split(hsvImage, hsvChannels);
    Ptr<CLAHE> clahe = createCLAHE(2.0, Size(8, 8));
    clahe->apply(hsvChannels[2], hsvChannels[2]);  // 增强亮度通道
    merge(hsvChannels, hsvImage);

    // === 颜色阈值分割 ===
    Scalar lower_blue = Scalar(100, 160, 30);   // 蓝色下限
    Scalar upper_blue = Scalar(151, 255, 255);  // 蓝色上限
    Scalar lower_white = Scalar(0, 0, 200);     // 白色下限
    Scalar upper_white = Scalar(180, 30, 255);  // 白色上限

    Mat blueMask, whiteMask;
    inRange(hsvImage, lower_blue, upper_blue, blueMask);    // 蓝色掩膜
    inRange(hsvImage, lower_white, upper_white, whiteMask); // 白色掩膜

    // 调试输出：显示HSV参数和掩膜信息
    std::cout << "=== 锥桶检测调试信息 ===" << std::endl;
    std::cout << "HSV范围: H[" << lower_blue[0] << "-" << upper_blue[0] << "] S[" 
              << lower_blue[1] << "-" << upper_blue[1] << "] V[" 
              << lower_blue[2] << "-" << upper_blue[2] << "]" << std::endl;
    std::cout << "蓝色掩膜像素数: " << cv::countNonZero(blueMask) << std::endl;
    std::cout << "白色掩膜像素数: " << cv::countNonZero(whiteMask) << std::endl;

    // 过滤白色区域：只保留纯蓝色部分
    Mat filteredMask;
    bitwise_not(whiteMask, whiteMask);              // 反转白色掩膜
    bitwise_and(blueMask, whiteMask, filteredMask); // 与操作得到纯蓝色区域
    
    std::cout << "过滤后掩膜像素数: " << cv::countNonZero(filteredMask) << std::endl;

    // === 轮廓分析 ===
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(filteredMask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // 调试输出：轮廓信息
    std::cout << "轮廓数量: " << contours.size() << std::endl;

    // 按面积排序，找到最大轮廓
    sort(contours.begin(), contours.end(), [](const vector<Point> &a, const vector<Point> &b) {
        return contourArea(a) > contourArea(b);
    });

    bool foundValidContour = false;

    if (!contours.empty()) {
        const vector<Point> &largestContour = contours[0];
        double maxArea = contourArea(largestContour);
        
        // 调试输出：最大轮廓面积
        std::cout << "最大轮廓面积: " << maxArea << " (阈值: " << Min_barrel_area << ")" << std::endl;
        
        // 面积阈值过滤
        if (maxArea >= Min_barrel_area) {
            // 计算轮廓质心
            Moments m = moments(largestContour);
            int cx = int(m.m10 / m.m00);
            int cy = int(m.m01 / m.m00);
            Point currentCenter(cx, cy);

            // 调试输出：锥桶中心坐标
            std::cout << "锥桶中心坐标: (" << cx << ", " << cy << ")" << std::endl;

            // 添加到历史记录（多帧平滑）
            previousCenters.push_back(currentCenter);
            if (previousCenters.size() > maxHistory) {
                previousCenters.erase(previousCenters.begin());
            }
            foundValidContour = true;
        } else {
            std::cout << "轮廓面积小于阈值，被过滤" << std::endl;
        }
    } else {
        std::cout << "未找到任何轮廓" << std::endl;
    }

    // === 状态机：多帧确认 ===
    if (foundValidContour) {
        detectCount++;  // 连续检测帧数增加
        if (detectCount >= 5) {
            missingCount = 0;    // 重置丢失计数
            barrelFound = true;  // 确认找到锥桶
        }
    } else if (barrelFound) {
        missingCount++;  // 锥桶丢失计数增加
        if (missingCount >= 5) {
            barrelFound = false;  // 确认锥桶丢失
            Found_barrel++;       // 已找到锥桶计数
            detectCount = 0;      // 重置检测计数
        }
    }

    // 调试输出：多帧确认状态
    std::cout << "多帧确认状态 - 检测计数: " << detectCount << ", 丢失计数: " << missingCount 
              << ", 锥桶确认: " << (barrelFound ? "是" : "否") << std::endl;

    // === 计算平滑后的锥桶中心 ===
    if (!previousCenters.empty() && barrelFound) {
        for (const Point &center : previousCenters) {
            barrel_center.x += center.x;
            barrel_center.y += center.y;
        }
        barrel_center.x /= previousCenters.size();  // 平均x坐标
        barrel_center.y /= previousCenters.size();  // 平均y坐标
        
        // 调试输出：平滑后的锥桶中心
        std::cout << "平滑后锥桶中心: (" << barrel_center.x << ", " << barrel_center.y 
                  << "), 历史帧数: " << previousCenters.size() << std::endl;
    } else {
        std::cout << "未计算平滑中心 - 历史帧: " << previousCenters.size() 
                  << ", 锥桶确认: " << (barrelFound ? "是" : "否") << std::endl;
    }

    return barrel_center;
}

/**
 * @brief 限制数值变化的函数（防止突变）
 * @param now 当前值
 * @param last 上一个值
 * @param limit 最大变化量
 * @return 限制后的值
 */
float Limit(int now, int last, int limit) {
    if ((now - last) >= limit) return (last + limit);
    else if ((now - last) <= -limit) return (last - limit);
    else return now;
    return last;
}

// 后锥桶模式处理函数
void Later_vertebral_model() { /* 实现略 */ }

/**
 * @brief 锥桶模式下的边界补线
 * @param img_ 输入图像
 */
void vertebral_model(Mat &img_) {
    Point barrel_center = blue_vertebral_barrel_find(img_);

    // 后锥桶模式处理
    if (Later_vertebral_Flag == 1) {
        Later_vertebral_model();
        return;
    }

    try_patching_line = 0;

    // 锥桶有效性检查
    if (barrel_center == Point(0, 0) || barrel_center.y > 239) {
        barrel_find_yes = 0;
        return; // 未找到锥桶
    } else {
        barrel_find_yes = 1;
    }

    // 判断锥桶位置（左/右）
    if (barrel_center.x <= 160) {
        try_patching_line = LEFT;   // 左侧锥桶
    } else {
        try_patching_line = RIGHT;  // 右侧锥桶
    }

    // 锥桶初次发现处理
    if (barrel_center.y > 41 && barrel_center.y < 80) {
        if (First_Find_vertebral_Flag != 1)
            offset = (try_patching_line == LEFT) ? 120 : -120;  // 初始偏移量
        First_Find_vertebral_Flag = 1;
    }

    // 锥桶跟踪和补线逻辑
    if (barrel_center.y < 180 && First_Find_vertebral_Flag == 1) {
        // 根据场景调整偏移量
        if (flag_cross == 2) {
            off = (try_patching_line == LEFT) ? 50 : -50;
            offset = Limit(off, offset, 2);
        } else if (flag_cross == 3) {
            off = (try_patching_line == LEFT) ? 80 : -80;
            offset = Limit(off, offset, 3);
        }
        
        // 根据锥桶位置生成补线
        Point center_offset;
        if (try_patching_line == LEFT) {
            slope_vertebral = -m_right;  // 使用对侧边界斜率
            center_offset = Point(x_right - offset, 110);
            Left_Line = Line(slope_vertebral, center_offset);      // 左边补线
            Right_Line = Line(m_right, cv::Point(x_right, 110));   // 右边实际边界
        } else if (try_patching_line == RIGHT) {
            slope_vertebral = -m_left;
            center_offset = Point(x_left - offset, 110);
            Left_Line = Line(m_left, cv::Point(x_left, 110));      // 左边实际边界
            Right_Line = Line(slope_vertebral, center_offset);     // 右边补线
        }

        Tracking_barrel(binaryImage_Finish);  // 锥桶模式下的边界跟踪
    } else if (barrel_center.y > 180 && Later_vertebral_Flag == 0 && First_Find_vertebral_Flag == 1) {
        Later_vertebral_Flag = 1;  // 进入后锥桶模式
    }
}

/**
 * @brief 赛道边界跟踪函数（核心函数）
 * @param dilated_image 膨胀后的二值图像
 * @return 跟踪是否成功（1成功，0失败）
 */
int Tracking(Mat dilated_image) {
    int begin = 160;  // 从图像中心开始搜索
    
    // 清空之前的边界点
    left_line.clear();
    right_line.clear();
    mid.clear();

    // 从图像底部向上逐行扫描（239-101行）
    for (int i = 239; i >= 101; i--) {
        int find_l = 0, find_r = 0;
        int to_left = begin, to_right = begin;

        // 向左搜索左边界
        while (to_left > 1) {
            if (dilated_image.at<uchar>(i, to_left) == 255) {
                find_l = 1;
                left_line.emplace_back(to_left, i);  // 存储左边界点
                break;
            }
            to_left--;
        }
        if (!find_l) left_line.emplace_back(1, i);  // 未找到则使用最左边

        // 向右搜索右边界
        while (to_right < 319) {
            if (dilated_image.at<uchar>(i, to_right) == 255) {
                find_r = 1;
                right_line.emplace_back(to_right, i);  // 存储右边界点
                break;
            }
            to_right++;
        }
        if (!find_r) right_line.emplace_back(319, i);  // 未找到则使用最右边

        // 计算中线点
        if (!left_line.empty() && !right_line.empty()) {
            cv::Point midx1 = left_line.back();
            cv::Point midx2 = right_line.back();
            mid.emplace_back((midx1.x + midx2.x) / 2, i);  // 中线点
            begin = (to_right + to_left) / 2;  // 更新搜索起点
        }
    }

    // === 直线拟合 ===
    cv::Vec4f left_fit, right_fit;
    cv::fitLine(left_line, left_fit, cv::DIST_L2, 0, 0.01, 0.01);   // 拟合左边界直线
    cv::fitLine(right_line, right_fit, cv::DIST_L2, 0, 0.01, 0.01); // 拟合右边界直线

    // 提取直线参数：斜率(m)和截距(b)
    m_left = left_fit[1] / left_fit[0];
    float b_left = left_fit[3] - m_left * left_fit[2];
    
    m_right = right_fit[1] / right_fit[0];
    float b_right = right_fit[3] - m_right * right_fit[2];

    // 计算角度（度）
    a_left = std::atan(m_left) * 180.0 / CV_PI;
    a_right = std::atan(m_right) * 180.0 / CV_PI;

    // 有效性检查：角度不能接近垂直（避免数值不稳定）
    if (abs(a_right) > 89 || abs(a_left) > 89) return 0;

    // 计算在y=110处的x坐标（用于后续控制）
    x_left = (100 + 10 - b_left) / m_left;
    x_right = (100 + 10 - b_right) / m_right;

    return 1;  // 跟踪成功
}

/**
 * @brief 锥桶模式下的边界跟踪函数
 * 
 * 在检测到锥桶后，使用预设的补线策略生成虚拟的赛道边界
 * 与普通Tracking函数不同，此函数不进行实际的图像边界检测，
 * 而是基于预计算的补线参数生成边界点，实现绕锥桶避障
 * 
 * @param dilated_image 输入的二值图像（实际未使用，仅为了保持接口一致）
 *                      - 在锥桶模式下，边界点完全由预定义的直线参数生成
 *                      - 参数保留是为了与普通Tracking函数接口保持一致
 */
void Tracking_barrel(Mat dilated_image) {
    // === 第一步：清空历史边界点 ===
    left_line.clear();   // 清空左边界点容器，准备存储新的补线边界点
    right_line.clear();  // 清空右边界点容器，准备存储新的补线边界点  
    mid.clear();         // 清空中线点容器，准备重新计算中线

    // === 第二步：逐行生成边界点（从图像底部向上扫描） ===
    // 扫描范围：y坐标从239（图像底部）到101（图像中上部）
    // 这种自底向上的扫描方式符合车辆前进的视角逻辑
    for (int i = 239; i >= 101; i--) {
        
        // === 2.1 计算左边界点坐标 ===
        int X_Left = 0;
        // 使用直线方程计算左边界在当前行i的x坐标：
        // 直线方程：y = slope * (x - startPoint.x) + startPoint.y
        // 变换得到：x = startPoint.x + (y - startPoint.y) / slope
        // 其中：y = i（当前行），startPoint是补线的起点坐标
        X_Left = (int)Left_Line.startPoint.x + (i - Left_Line.startPoint.y) / Left_Line.slope;
        
        // 边界保护：确保左边界点不会超出图像左边界（x < 1）
        if (X_Left < 1) X_Left = 1;
        
        // 将计算得到的左边界点添加到左边界点集
        left_line.emplace_back(cv::Point(X_Left, i));

        // === 2.2 计算右边界点坐标 ===
        int X_Right = 0;
        // 使用相同的直线方程计算右边界在当前行i的x坐标
        X_Right = (int)Right_Line.startPoint.x + (i - Right_Line.startPoint.y) / Right_Line.slope;
        
        // 边界保护：确保右边界点不会超出图像右边界（x > 319）
        if (X_Right > 319) X_Right = 319;
        
        // 将计算得到的右边界点添加到右边界点集
        right_line.emplace_back(cv::Point(X_Right, i));

        // === 2.3 计算中线点坐标 ===
        // 中线点 = (左边界点 + 右边界点) / 2
        // 这个中线将用于后续的车辆控制决策
        mid.emplace_back(cv::Point((X_Left + X_Right) / 2, i));
    }
}

// 转向标志检测相关定义
#define To_Left 1    // 向左转向
#define To_Right 2   // 向右转向
uint8_t BianDao_Num[3] = {0};  // 转向计数数组

/**
 * @brief 转向标志识别函数
 * @param frame 输入图像
 */
void Judge_Turn_Pro(Mat frame) {
    // 转换为HSV颜色空间
    Mat hsv;
    cvtColor(frame, hsv, COLOR_BGR2HSV);

    // 定义HSV中的蓝色范围（转向标志颜色）
    Scalar lower_blue(90, 29, 30);
    Scalar upper_blue(130, 255, 255);

    // 创建蓝色掩膜
    Mat mask;
    inRange(hsv, lower_blue, upper_blue, mask);

    // 查找蓝色区域的轮廓
    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // 查找最大的蓝色区域
    double max_area = 0;
    vector<Point> largest_contour;
    int height = frame.rows;
    int width = frame.cols;
    
    // 设置搜索区域限制（避免误检边缘区域）
    int upper_limit = height / 3;
    int lower_limit = height * 99 / 100;
    int left_limit = width / 8;
    int right_limit = width * 7 / 8;

    for (const auto &contour : contours) {
        double area = contourArea(contour);
        Rect boundingRect = cv::boundingRect(contour);
        
        // 区域过滤
        if (boundingRect.y >= upper_limit && (boundingRect.y + boundingRect.height) <= lower_limit &&
            boundingRect.x >= left_limit && (boundingRect.x + boundingRect.width) <= right_limit) {
            if (area > max_area) {
                max_area = area;
                largest_contour = contour;
            }
        }
    }

    // 转向标志消失处理
    static uint8_t Count = 0;
    if (max_area < 200) {
        if (Find_Turn_Flag == 1) {
            Count++;
            if (Count % 10 == 0) {
                BianDao_Num[0] = 0; BianDao_Num[1] = 0;
                R_L_Flag = 0; Stop_Turn_Cross = 1;
            }
        }
        return;
    }

    // 处理最大蓝色区域（转向标志）
    if (!largest_contour.empty()) {
        RotatedRect rotated_rect = minAreaRect(largest_contour);  // 最小外接矩形
        
        Point2f box[4];
        rotated_rect.points(box);  // 获取矩形的四个顶点
        
        Point2f center = rotated_rect.center;  // 中心点
        float angle = rotated_rect.angle;      // 旋转角度
        
        // 角度调整
        if (angle > 0) angle = angle - 90;
        else if (angle == 0) angle = 0;
        
        // 图像旋转校正
        Mat rotated_image;
        Mat M = getRotationMatrix2D(center, angle, 1.0);
        Rect bounding_box = rotated_rect.boundingRect();
        warpAffine(img, rotated_image, M, img.size(), INTER_LINEAR, BORDER_CONSTANT);
        Mat rect = rotated_image(bounding_box);  // 提取转向标志区域

        // 绘制外接矩形
        for (int i = 0; i < 4; i++) {
            line(img, box[i], box[(i + 1) % 4], Scalar(255, 0, 0), 1);
        }

        // 透视变换标准化
        Point2f src_points[4] = {Point2f(0, 0), Point2f(rect.cols, 0), 
                                Point2f(rect.cols, rect.rows), Point2f(0, rect.rows)};
        Point2f dst_points[4] = {Point2f(0, 0), Point2f(300, 0), 
                                Point2f(300, 200), Point2f(0, 200)};
        Mat matrix = getPerspectiveTransform(src_points, dst_points);
        Mat warped;
        warpPerspective(rect, warped, matrix, Size(300, 200));

        // 白色区域检测（箭头部分）
        Mat gray_warped, thresh_warped;
        cvtColor(warped, gray_warped, COLOR_BGR2GRAY);
        threshold(gray_warped, thresh_warped, 127, 255, THRESH_BINARY);
        
        Mat white_mask;
        inRange(gray_warped, 220, 255, white_mask);  // 高亮度区域
        Mat combined_mask;
        bitwise_and(thresh_warped, white_mask, combined_mask);

        // 白色轮廓分析
        vector<vector<Point>> white_contours;
        findContours(combined_mask, white_contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        if (!white_contours.empty()) {
            // 找到最大白色轮廓（箭头）
            vector<Point> largest_white_contour = *max_element(white_contours.begin(), white_contours.end(),
                [](const vector<Point> &a, const vector<Point> &b) {
                    return contourArea(a) < contourArea(b);
                });

            // 计算凸包
            vector<Point> hull;
            convexHull(largest_white_contour, hull);
            vector<vector<Point>> hull_list{hull};
            drawContours(warped, hull_list, -1, Scalar(255, 0, 0), 2);

            // 计算凸包质心（箭头方向判断）
            Moments M = moments(hull);
            if (M.m00 != 0) {
                int cX = static_cast<int>(M.m10 / M.m00);  // 质心X坐标
                int cY = static_cast<int>(M.m01 / M.m00);  // 质心Y坐标

                // 绘制质心点
                circle(warped, Point(cX, cY), 5, Scalar(0, 0, 255), -1);

                // 判断转向方向（基于质心位置）
                if (cX <= 149) {
                    R_L_Flag_Temp = 1;  // 左转
                } else if (cX >= 151) {
                    R_L_Flag_Temp = 2;  // 右转
                }
            }
        }
        
        // 转向计数统计（多帧确认）
        if (R_L_Flag_Temp == 1) {
            BianDao_Num[0]++;  // 左转计数
        } else if (R_L_Flag_Temp == 2) {
            BianDao_Num[1]++;  // 右转计数
        }

        // 最终转向决策
        if (BianDao_Num[0] > BianDao_Num[1] && BianDao_Num[0] > 8) {
            R_L_Flag = 1;        // 确认左转
            Find_Turn_Flag = 1;  // 找到转向标志
        } else if (BianDao_Num[0] < BianDao_Num[1] && BianDao_Num[1] > 8) {
            R_L_Flag = 2;        // 确认右转
            Find_Turn_Flag = 1;  // 找到转向标志
        }
    }
}

// 中线偏差计算相关
int bufferSize = 50;              // 移动平均窗口大小
std::vector<int> biasBuffer;      // 用于存储最近的偏差值

/**
 * @brief 计算中线偏差
 * 在100-239行范围内采样50个点，计算中线与图像中心的平均偏差
 */
void Calculate_Bias(void) {
    Bias_Line_int = 0;
    
    // 在100-239行范围内等间隔采样50个点
    for (int i = 1; i <= 100; i += 2) {
        Bias_Line_int += mid[i].x;  // 累加中线点x坐标
    }
    
    // 计算平均偏差（相对于图像中心160）
    Bias_Line_int = Bias_Line_int / 50 - 160;

    // 限制偏差范围在±80像素内
    if (Bias_Line_int > 80) Bias_Line_int = 80;
    else if (Bias_Line_int < -80) Bias_Line_int = -80;
    
    Bias_Line = Bias_Line_int;  // 赋值给通信变量
}

/**
 * @brief 显示处理信息和边界线
 * 在原图上绘制赛道边界、中线、ROI区域和各种状态信息
 */
void display_Information(void) {
    int j = 0;
    
    // 绘制边界线和中线
    for (int i = 239; i >= 101; i--) {
        circle(img, Point(int(left_line[j].x), i), 0.1, Scalar(0, 0, 0), 2);    // 左边界
        circle(img, Point(int(right_line[j].x), i), 0.1, Scalar(0, 0, 0), 2);   // 右边界
        circle(img, Point(int((left_line[j].x + right_line[j].x) / 2), i), 0.1, Scalar(255, 0, 0), 2); // 中线
        j++;
    }

    // 绘制参考线和ROI区域
    line(img, Point(160, 100), Point(160, 239), Scalar(0, 0, 255), 2);  // 中心竖直线
    rectangle(img, Rect(1, 100, 319, 239), Scalar(0, 255, 0), 1);       // ROI区域边框

    // 显示中线偏差
    std::ostringstream oss;
    oss << "Bias_Line: " << static_cast<int>(Bias_Line);
    std::string text = oss.str();
    cv::putText(img, text, cv::Point(0, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);

    // 显示斑马线状态
    oss.str(""); oss.clear();
    switch (flag_cross) {
    case -1: oss << "flag_cross: find"; break;
    case 0: oss << "flag_cross: nofind"; break;
    case 1: oss << "flag_cross: stop_car"; break;
    case 2: oss << "vertebral: middleCone! (" << Found_barrel + 1 << ")"; break;
    case 3: oss << "vertebral: ordinary! (" << Found_barrel + 1 << ")"; break;
    case 4: oss << "  stop!"; break;
    }
    text = oss.str();
    cv::putText(img, text, cv::Point(0, 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1);

    // 显示转向标志和绘制箭头
    oss.str(""); oss.clear();
    switch (R_L_Flag) {
    case 0:  // 直行
        arrowedLine(img, Point(160, 100), Point(160, 60), Scalar(0, 255, 0), 2, 8, 0, 0.5);
        oss << "R_L_Flag: To_zhi";
        break;
    case 1:  // 左转
        arrowedLine(img, Point(150, 90), Point(100, 90), Scalar(0, 255, 0), 2, 8, 0, 0.5);
        oss << "R_L_Flag: To_Left";
        break;
    case 2:  // 右转
        arrowedLine(img, Point(150, 90), Point(200, 90), Scalar(0, 255, 0), 2, 8, 0, 0.5);
        oss << "R_L_Flag: To_Right";
        break;
    default: oss << "R_L_Flag: Unknown"; break;
    }
    text = oss.str();
    cv::putText(img, text, cv::Point(0, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1);
}

// HSV参数调节相关（开发调试用）
Mat hsvImg, outputImg;
int h_min = 90, h_max = 130;
int s_min = 29, s_max = 255;
int v_min = 30, v_max = 255;

void updateHSV(int, void *) {
    inRange(hsvImg, Scalar(h_min, s_min, v_min), Scalar(h_max, s_max, v_max), outputImg);
    imshow("Output Image", outputImg);
}

// 串口通信相关函数
int fd = -1;  // 串口文件描述符

/**
 * @brief 串口初始化
 * @return 初始化是否成功
 */
int Serial_Init(void) {
    const char *port = "/dev/ttyUSB0";  // 串口设备（Linux）
    int baud_rate = B115200;            // 波特率115200

    fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        std::cerr << "error_serial " << port << std::endl;
        return 0;
    }

    fcntl(fd, F_SETFL, 0);

    // 串口参数配置
    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, baud_rate);
    cfsetospeed(&options, baud_rate);
    options.c_cflag &= ~PARENB;    // 无奇偶校验
    options.c_cflag &= ~CSTOPB;    // 1位停止位
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;        // 8位数据位
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
 * 发送5字节数据包到TC377控制器
 * 格式：0xA5 | 中线偏差 | 标志位 | 转向标志 | 0x5A
 * @return 发送是否成功
 */
int Data_Send(void) {
    uint8_t buffer[5];
    // 数据包组装
    memcpy(buffer, &Head, sizeof(int8_t));        // 包头 0xA5
    memcpy(buffer + 1, &Bias_Line, sizeof(int8_t));   // 中线偏差
    memcpy(buffer + 2, &flag_cross, sizeof(int8_t));  // 标志位
    memcpy(buffer + 3, &R_L_Flag, sizeof(int8_t));    // 转向标志
    memcpy(buffer + 4, &Tail, sizeof(int8_t));        // 包尾 0x5A

    // 串口发送
    int n = write(fd, buffer, sizeof(buffer));
    if (n < 0) {
        printf("error");
        close(fd);
        return 1;
    }
    return 0;
}

/**
 * @brief 获取当前时间字符串（用于录像文件名）
 * @return 时间字符串
 */
std::string getCurrentTimeString() {
    auto now = std::time(nullptr);
    std::tm *localTime = std::localtime(&now);
    std::ostringstream oss;
    oss << std::put_time(localTime, "%Y-%m-%d_%H-%M-%S");
    return oss.str();
}

// 运行模式选择
#define Select_Video_Frame (1)  // 1:摄像头 2:单图像 3:色域调节 4:视频转图像

/**
 * @brief 主函数
 * 图像处理主循环，包含摄像头采集、图像处理、决策输出、串口通信
 */
int main(int argc, char *argv[]) {
    // 串口初始化
    if (!Serial_Init()) return 0;

#if Select_Video_Frame == 1  // 摄像头实时处理模式
    // 打开摄像头
    cv::VideoCapture capture("/dev/video0", cv::CAP_V4L2);
    cv::Size resolution(320, 240);
    capture.set(cv::CAP_PROP_FRAME_WIDTH, resolution.width);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, resolution.height);

    // 帧率计算初始化
    auto start = std::chrono::high_resolution_clock::now();
    int frameCount = 0;
    float FPS = 0;

    // 录像设置
    cv::VideoWriter writer;
    uint8_t handle = 0;
    if (argc > 1) {
        std::string param = argv[1];
        std::string outputPath;
        if (param == "6") {
            outputPath = "/home/dgdz/桌面/原图/" + getCurrentTimeString() + ".mp4";
            handle = 2;  // 录制原图
        } else {
            outputPath = "/home/dgdz/桌面/处理图/" + getCurrentTimeString() + ".mp4";
            handle = 1;  // 录制处理图
        }
        int frameWidth = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH));
        int frameHeight = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT));
        writer.open(outputPath, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 30, cv::Size(frameWidth, frameHeight));
    }

    // 主处理循环
    while (capture.read(img)) {
        frameCount++;

        // 录制原图
        if (handle == 2) writer.write(img);

        // === 核心图像处理流程 ===
        ImagePreprocessing_Sobel_LinePro(img);  // 图像预处理和边界检测

        // 锥桶检测（斑马线未确认时）
        if (is_First_cross != 1 && Stop_Turn_Cross != 1) {
            img_ROI = defineTrapezoidROI(img);
            vertebral_model(img_ROI);  // 锥桶检测和补线
        }

        // 斑马线检测
        if (Stop_Turn_Cross != 1) {
            crossroad(img);
        }

        // 转向标志检测（斑马线确认后）
        if (is_First_cross == 1 && Stop_Turn_Cross != 1) {
            img_ROI = defineTrapezoidROI(img);
            Judge_Turn_Pro(img_ROI);  // 转向标志识别
        }

        // 计算控制参数并显示
        Calculate_Bias();        // 计算中线偏差
        display_Information();   // 显示处理信息
        Data_Send();             // 发送控制数据

        // 帧率计算和显示
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        if (elapsed.count() >= 1.0) {
            FPS = frameCount / elapsed.count();
            frameCount = 0;
            start = std::chrono::high_resolution_clock::now();
        }
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << "FPS: " << FPS;
        std::string text = oss.str();
        cv::putText(img, text, cv::Point(230, 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);

        // 录制处理图
        if (handle == 1) writer.write(img);

        // 显示结果图像
        resize(img, img, cv::Size(), 2.5, 2.5);  // 放大显示
        imshow("img_finall", img);

        // 按'6'键停止
        if (cv::waitKey(1) == '6') break;
    }
    
    // 资源释放
    capture.release();
    writer.release();
    cv::destroyAllWindows();
    return 0;

#elif Select_Video_Frame == 2  // 单图像测试模式
    img = imread(frame_path);
    Judge_Turn_Pro(img);
    imshow("img_finall", img);
    waitKey(0);
    return 0;

#elif Select_Video_Frame == 3  // HSV色域调节模式（调试用）
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

#else  // 视频转图像模式（数据采集用）
    string savePath = "D:/IDE/bike_video/video/nk6/";
    VideoCapture cap;
    cap.open(video_path);
    int frameCount = 0;
    int saveFrameInterval = 10;  // 每10帧保存一帧
    Mat frame;
    while (true) {
        bool success = cap.read(frame);
        if (!success) break;
        if (frameCount % saveFrameInterval == 0) {
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
