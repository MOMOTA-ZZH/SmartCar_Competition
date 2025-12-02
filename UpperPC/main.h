#ifndef MAIN_H
#define MAIN_H

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

// ============================================================================
// 平台与模式定义
// ============================================================================
#define ubuntu (1)
#if ubuntu
#include <termios.h>
#include <unistd.h>
#endif

using namespace cv;
using namespace std;

// ============================================================================
// 常量与宏定义
// ============================================================================
#define LEFT 1
#define RIGHT 2
#define MIDDLE 3
#define To_Left 1
#define To_Right 2

// ============================================================================
// 枚举定义
// ============================================================================

// 车辆任务状态
enum class VehicleState : int8_t
{
    NoTask = 0,             // 无特定任务
    FindingZebraCrossing = -1, // 寻找斑马线
    StoppingAtZebraCrossing = 1, // 停车等待斑马线
    AvoidingMiddleCone = 2,    // 避让中间锥桶
    AvoidingNormalCone = 3,    // 避让普通锥桶
    Stopping = 4,           // 完全停止
    TurningLeft = 5,        // 正在左转
    TurningRight = 6,       // 正在右转
    ReturningFromLaneChange = 7 // 变道后沿引导锥桶返回 (新状态)
};

// ============================================================================
// 结构体定义
// ============================================================================
struct Line
{
    float slope;
    cv::Point startPoint;

    Line(float s, cv::Point start) : slope(s), startPoint(start) {}
};

// ============================================================================
// 全局变量声明 (extern)
// ============================================================================

// --- 文件路径配置 ---
extern string testFramePath;
extern string testVideoPath;

// --- 赛道跟踪相关参数 ---
extern float trackWidthPx; // 赛道宽度基准值（像素）
extern float leftLineSlope, rightLineSlope; // 赛道左右边界直线斜率
extern float rightLineAngleDeg, leftLineAngleDeg; // 赛道左右边界角度（度）
extern float leftLineXAtRefY, rightLineXAtRefY; // 赛道左右边界在参考Y坐标处的X值
extern float blueMidlineSlope, blueMidlineIntercept; // 蓝色边界中线参数
extern int8_t trackingStrategyFlag; // 跟踪策略标志：0=策略1, 1=策略2, 2=策略3

// --- 串口通信协议 ---
extern int8_t uartHeader, uartTail, uartLength; // 数据包头、尾、长度

// --- 图像处理参数 ---
extern int sobelThreshold; // Sobel边缘检测阈值，原 My_threshold
extern int houghLinesPThreshold; // 霍夫变换阈值，原 My_HoughLinesP
extern int minModerateAreaPx; // 中等面积阈值（像素），用于锥桶，原 Min_moderate
extern int minConeAreaPx; // 最小锥桶面积阈值（像素），原 Min_barrel_area
extern int brightnessThreshold; // 亮度阈值，原 brightness_threshold

// --- 控制与状态标志 ---
extern VehicleState vehicleCurrentState; // 车辆当前任务状态，原 flag_cross
extern int8_t controlBiasLine; // 中线偏差（像素），发送给下位机，原 Bias_Line
extern int8_t turnDirectionFlag; // 左右变向标志位：0直行,1左转,2右转，原 R_L_Flag
extern int8_t tempTurnDirectionFlag; // 临时转向标志，原 R_L_Flag_Temp
extern int controlBiasLineInt; // 中线偏差整数值，内部计算使用，原 Bias_Line_int
extern uint8_t isFirstZebraCrossing; // 首次识别人行道标志，原 is_First_cross
extern uint8_t barrelFindFinishFlag; // 是否继续寻找锥桶标志，原 barrel_find_finish
extern uint8_t barrelFindYesFlag; // 锥桶识别标志位，原 barrel_find_yes
extern uint8_t stopTurnDetectFlag; // 停止转向检测标志，原 Stop_Turn_Cross
extern uint8_t findTurnSignFlag; // 找到转向标志，原 Find_Turn_Flag
extern bool isCleverConeMode; // 是否启用智能锥桶模式，原 cleverFlag

// --- 图像矩阵 ---
extern cv::Mat currentImage; // 源图像，原 img
extern cv::Mat displayImage; // 带有各种信息的图片，原 img_xiaoxi
extern cv::Mat roiImage; // 感兴趣区域图像，原 img_ROI
extern cv::Mat binaryOutputImage; // 图像处理后的二值化图片，原 binaryImage
extern cv::Mat finalBinaryLaneImage; // 最终提取赛道白线的图片，原 binaryImage_Finish
extern cv::Mat intermediateBinaryImage1; // 图像处理后的二值化图片，原 binaryImage12
extern Mat hsvImage, outputHsvImage; // HSV图像和输出图像，原 hsvImg, outputImg

// --- 锥桶检测相关变量 ---
extern int leftConeSerialNumber, middleConeSerialNumber, rightConeSerialNumber; // 锥桶序列号，原 Left_barrel_serial_number等
extern vector<Point> previousConeCenters; // 历史锥桶中心点，用于平滑，原 previousCenters
extern size_t maxHistoryCount; // 历史中心点最大存储数量，原 maxHistory
extern int frameCountLimit; // 帧数限制，原 Frame_zhen
extern int detectConsecutiveCount, missingConsecutiveCount; // 连续检测/丢失计数，原 detectCount, missingCount
extern bool isConeFound; // 锥桶是否被确认找到，原 barrelFound
extern int totalConesFound; // 已找到锥桶总数，原 Found_barrel
extern uint8_t patchLineDirection; // 补线方向 (LEFT/RIGHT)，原 try_patching_line
extern int laterConeProcessingFlag; // 后锥桶处理模式标志，原 Later_vertebral_Flag
extern float conePathSlope; // 锥桶补线路径斜率，原 slope_vertebral
extern int firstConeFoundFlag; // 首次发现锥桶标志，原 First_Find_vertebral_Flag
extern int pathOffset; // 路径偏移量，原 offset
extern int targetOffset; // 目标偏移量，原 off
extern Line LeftVirtualLine, RightVirtualLine; // 虚拟左右边界线，原 Left_Line, Right_Line

// --- 边界点存储容器 ---
extern std::vector<cv::Point> leftLanePoints, rightLanePoints, middleLanePoints; // 赛道左右边界点集，中线点集，原 left_line等
extern std::vector<cv::Point> blueLeftLanePoints, blueRightLanePoints, blueMiddleLanePoints; // 蓝色左右边界点集，中线点集，原 left_line_blue等

// --- 斑马线与转向相关 ---
extern int zebraCrossingCount; // 斑马线计数，原 crossroad_num
extern Rect roiRect; // ROI矩形，原 roi_rect
extern uint8_t laneChangeDetectCount[3]; // 变道检测计数，原 BianDao_Num

// --- HSV参数调试 ---
extern int hMin, hMax, sMin, sMax, vMin, vMax; // HSV阈值，原 h_min等

// --- 串口文件描述符 ---
#if ubuntu
extern int fd; // 串口文件描述符，原 fd
#endif

// --- 其他变量 ---
// extern int bufferSize; // 缓冲大小，原 bufferSize
// extern std::vector<int> biasBuffer; // 偏差缓冲区，原 biasBuffer

// ============================================================================
// 函数原型声明
// ============================================================================

// --- 核心图像处理函数 ---
Mat defineTrapezoidROI(Mat src);
Mat preprocessImage(Mat src); // 图像预处理
Mat detectLaneEdges(Mat processedImg); // 边缘检测
vector<Vec4i> findHoughLines(Mat binaryImg, Rect roi); // 霍夫线段检测
void classifyAndDrawLines(const vector<Vec4i>& lines, int x_roi, int y_roi, Mat& outputImage3, Mat& outputImage4, Mat& outputImage5); // 线段分类和绘制
Mat ImagePreprocessing_Sobel_LinePro(Mat frame_a); // 整合函数
void Sobel_LinePro(Mat frame_a);
Point blue_vertebral_barrel_find(Mat frame); // 蓝色锥桶检测函数

// --- 赛道跟踪与避障函数 ---
int Tracking(Mat dilated_image);
void Tracking_barrel(Mat dilated_image);
void vertebral_model(Mat &img_);
void Later_vertebral_model();
float Limit(int now, int last, int limit); // 数值限制函数

// --- 专项任务函数 ---
int crossroad(Mat frame); // 斑马线检测函数
void Judge_Turn_Pro(Mat frame); // 转向标志识别函数

// --- 控制与通信函数 ---
void Calculate_Bias(void); // 计算中线偏差
void display_Information(void); // 显示处理信息

#if ubuntu
int Serial_Init(void); // 串口初始化
int Data_Send(void); // 数据发送
std::string getCurrentTimeString(); // 获取当前时间字符串
#endif

// --- 调试辅助函数 ---
void updateHSV(int, void *); // HSV参数调节回调函数

#endif // MAIN_H
