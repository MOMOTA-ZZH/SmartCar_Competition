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

#include "main.h" // 引入自定义头文件

// ============================================================================
// 全局变量定义 (在 main.h 中声明为 extern)
// ============================================================================

// --- 文件路径配置 ---
string testFramePath = "/home/dgdz/桌面/Test_Image/yellow.png";  // 测试图像路径
string testVideoPath = "D:/1vscode/930/8.mp4";                  // 测试视频路径

// --- 赛道跟踪相关参数 ---
float trackWidthPx = 120;      // 赛道宽度基准值（像素）
float leftLineSlope = 0, rightLineSlope = 0; // 赛道左右边界直线斜率
float rightLineAngleDeg = 0, leftLineAngleDeg = 0; // 赛道左右边界角度（度）
float leftLineXAtRefY = 0, rightLineXAtRefY = 0; // 赛道左右边界在参考Y坐标处的X值
float blueMidlineSlope = 0, blueMidlineIntercept = 0; // 蓝色边界中线参数
int8_t trackingStrategyFlag = 1;      // 跟踪策略标志：0=策略1, 1=策略2, 2=策略3

// --- 串口通信协议 ---
int8_t uartHeader = 0XA5;           // 数据包头
int8_t uartTail = 0X5A;           // 数据包尾
int8_t uartLength = 3;            // 数据长度

// --- 图像处理参数 ---
int sobelThreshold = 0;          // Sobel边缘检测阈值
int houghLinesPThreshold = 50;       // 霍夫变换阈值
int minModerateAreaPx = 95;         // 中等面积阈值（像素），用于锥桶
int minConeAreaPx = 130;     // 最小锥桶面积阈值（像素）
int brightnessThreshold = 20; // 亮度阈值

// --- 控制与状态标志 ---
VehicleState vehicleCurrentState = VehicleState::NoTask; // 车辆当前任务状态
int8_t controlBiasLine = 0;              // 中线偏差（像素），发送给下位机
int8_t turnDirectionFlag = 0;               // 左右变向标志位：0直行,1左转,2右转
int8_t tempTurnDirectionFlag = 0;          // 临时转向标志
int controlBiasLineInt = 0;         // 中线偏差整数值，内部计算使用
uint8_t isFirstZebraCrossing = 0;     // 首次识别人行道标志
uint8_t barrelFindFinishFlag = 0; // 是否继续寻找锥桶标志
uint8_t barrelFindYesFlag = 0;    // 锥桶识别标志位
uint8_t stopTurnDetectFlag = 0;    // 停止转向检测标志
uint8_t findTurnSignFlag = 0;     // 找到转向标志
bool isCleverConeMode = false; // 是否启用智能锥桶模式

// --- 图像矩阵 ---
cv::Mat currentImage;                    // 源图像
cv::Mat displayImage;            // 带有各种信息的图片
cv::Mat roiImage;               // 感兴趣区域图像
cv::Mat binaryOutputImage;           // 图像处理后的二值化图片
cv::Mat finalBinaryLaneImage;    // 最终提取赛道白线的图片
cv::Mat intermediateBinaryImage1;         // 图像处理后的二值化图片

// --- HSV图像与输出 ---
Mat hsvImage, outputHsvImage;

// --- 锥桶检测相关变量 ---
int leftConeSerialNumber = 1;   // 左锥桶序列号
int middleConeSerialNumber = 2;    // 中间锥桶序列号  
int rightConeSerialNumber = 3;  // 右锥桶序列号
vector<Point> previousConeCenters; // 历史锥桶中心点，用于平滑
size_t maxHistoryCount = 5; // 历史中心点最大存储数量
int frameCountLimit = 60; // 帧数限制
int detectConsecutiveCount = 0; // 连续检测计数
int missingConsecutiveCount = 0; // 连续丢失计数
bool isConeFound = false; // 锥桶是否被确认找到
int totalConesFound = 0; // 已找到锥桶总数
uint8_t patchLineDirection = 0; // 补线方向 (LEFT/RIGHT)
int laterConeProcessingFlag = 0; // 后锥桶处理模式标志
float conePathSlope = 0; // 锥桶补线路径斜率
int firstConeFoundFlag = 0; // 首次发现锥桶标志
int pathOffset = 0; // 路径偏移量
int targetOffset = 0; // 目标偏移量
Line LeftVirtualLine(0, cv::Point(0, 0)), RightVirtualLine(0, cv::Point(0, 0)); // 虚拟左右边界线

// --- 边界点存储容器 ---
std::vector<cv::Point> leftLanePoints;      // 左边界点集
std::vector<cv::Point> rightLanePoints;     // 右边界点集
std::vector<cv::Point> middleLanePoints;            // 中线点集
std::vector<cv::Point> blueLeftLanePoints; // 蓝色左边界点集
std::vector<cv::Point> blueRightLanePoints;// 蓝色右边界点集
std::vector<cv::Point> blueMiddleLanePoints;       // 蓝色中线点集

// --- 斑马线与转向相关 ---
int zebraCrossingCount = 0; // 斑马线计数
Rect roiRect; // ROI矩形
uint8_t laneChangeDetectCount[3] = {0}; // 变道检测计数

// --- HSV参数调试 ---
int hMin = 90, hMax = 130;
int sMin = 29, sMax = 255;
int vMin = 30, vMax = 255;

// --- 锥桶调试统计 ---
int coneDebugFrameCount = 0;
int coneDetectedCount = 0;
int coneMissedCount = 0;

#if ubuntu
int fd = -1;     // 串口文件描述符
#endif

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
    finalBinaryLaneImage = cv::Mat::zeros(height, width, CV_8U);   // 最终输出图像，原 binaryImage_Finish
    cv::Mat binaryImage_Finish3 = cv::Mat::zeros(height, width, CV_8U); // 保底处理图像
    cv::Mat binaryImage_Finish4 = cv::Mat::zeros(height, width, CV_8U); // 处理图像1
    cv::Mat binaryImage_Finish5 = cv::Mat::zeros(height, width, CV_8U); // 处理图像2

    // Mat binaryImage12; // 声明已移到全局变量

/**
 * @brief 图像预处理：灰度化、降噪、形态学操作
 * @param src 输入彩色图像
 * @return 预处理后的灰度图像
 */
Mat preprocessImage(Mat src)
{
    Mat grayImage;
    cvtColor(src, grayImage, cv::COLOR_BGR2GRAY);

    Mat blurredImage;
    GaussianBlur(grayImage, blurredImage, Size(5, 5), 2);
    medianBlur(blurredImage, blurredImage, 5);

    Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
    Mat morphedImage;
    morphologyEx(blurredImage, morphedImage, MORPH_OPEN, kernel);
    cv::dilate(morphedImage, morphedImage, kernel, cv::Point(-1, -1), 1);
    medianBlur(morphedImage, morphedImage, 5);

    return morphedImage;
}

/**
 * @brief 边缘检测：Sobel算子和二值化
 * @param processedImg 预处理后的灰度图像
 * @return 边缘二值图像
 */
Mat detectLaneEdges(Mat processedImg)
{
    Mat sobelx, sobely;
    Sobel(processedImg, sobelx, CV_64F, 1, 0, 3);
    Sobel(processedImg, sobely, CV_64F, 0, 1, 3);
    convertScaleAbs(sobelx, sobelx);
    convertScaleAbs(sobely, sobely);

    Mat gradientMagnitude;
    addWeighted(sobelx, 0.5, sobely, 0.5, 0, gradientMagnitude);

    Mat binaryEdges = Mat::zeros(processedImg.size(), CV_8U);
    cv::threshold(gradientMagnitude, binaryEdges, sobelThreshold, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
    
    Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::dilate(binaryEdges, binaryEdges, kernel, cv::Point(-1, -1), 1);

    return binaryEdges;
}

/**
 * @brief 霍夫线段检测
 * @param binaryImg 输入的二值图像
 * @param roi 感兴趣区域
 * @return 检测到的线段向量
 */
vector<Vec4i> findHoughLines(Mat binaryImg, Rect roi)
{
    Mat croppedObject = binaryImg(roi);
    vector<Vec4i> lines;
    HoughLinesP(croppedObject, lines, 1, CV_PI / 180, houghLinesPThreshold, 20, 10); // minLineLength=20, maxLineGap=10
    return lines;
}

/**
 * @brief 线段分类和绘制
 * @param lines 检测到的线段
 * @param x_roi, y_roi ROI偏移量
 * @param outputImage3, outputImage4, outputImage5 输出图像
 */
void classifyAndDrawLines(const vector<Vec4i>& lines, int x_roi, int y_roi, Mat& outputImage3, Mat& outputImage4, Mat& outputImage5)
{
    for (size_t i = 0; i < lines.size(); i++) {
        Vec4i l = lines[i];
        
        float k = static_cast<float>(l[3] - l[1]) / (l[2] - l[0]);
        float angle_k = std::atan(k) * 180.0 / CV_PI;
        float b = l[1] - k * l[0];
        float x6 = (10 - b) / k;
        float length = sqrt(pow(l[3] - l[1], 2) + pow(l[2] - l[0], 2));

        if (abs(angle_k) > 30 && length > 30) {
            Vec4i abs_l = l;
            abs_l[0] += x_roi; abs_l[1] += y_roi; abs_l[2] += x_roi; abs_l[3] += y_roi;
            line(outputImage3, Point(abs_l[0], abs_l[1]), Point(abs_l[2], abs_l[3]), Scalar(255), 2, LINE_AA);

            if (abs(angle_k) < 80 && length > 30 && (l[1] > 55 || l[3] > 55) && 
                (l[1] < 15 || l[3] < 15) && 
                ((l[0] + l[2] < 319) ? (k < 0 && abs(angle_k - leftLineAngleDeg) < 18 && abs(115 - rightLineXAtRefY + x6) < 10) : 
                 (k > 0 && abs(angle_k - rightLineAngleDeg) < 18 && abs(115 + leftLineXAtRefY - x6) < 10))) {
                line(outputImage4, Point(abs_l[0], abs_l[1]), Point(abs_l[2], abs_l[3]), Scalar(255), 2, LINE_AA);
            }

            if (abs(angle_k) < 85 && length > 30 && (l[1] > 65 || l[3] > 65) && 
                (l[1] < 8 || l[3] < 8) && 
                ((l[0] < 160 + 20 || l[2] < 160 + 20) ? k < 0 : 1) && 
                ((l[0] > 160 - 20 || l[2] > 160 - 20) ? k > 0 : 1)) {
                line(outputImage5, Point(abs_l[0], abs_l[1]), Point(abs_l[2], abs_l[3]), Scalar(255), 2, LINE_AA);
            }
        }
    }
}

/**
 * @brief 主图像预处理函数 - Sobel边缘检测 + 霍夫线段检测
 * 
 * 处理流程：
 * 1. 图像预处理 (调用 preprocessImage)
 * 2. Sobel边缘检测 (调用 detectLaneEdges)
 * 3. 霍夫变换检测线段 (调用 findHoughLines)
 * 4. 线段过滤和分类 (调用 classifyAndDrawLines)
 * 5. 多策略跟踪选择
 * 
 * @param frame_a 输入彩色图像
 * @return 处理后的二值图像（包含赛道边界）
 */
Mat ImagePreprocessing_Sobel_LinePro(Mat frame_a)
{
    int width = frame_a.cols;
    int height = frame_a.rows;

    finalBinaryLaneImage = cv::Mat::zeros(height, width, CV_8U);
    cv::Mat binaryImage_Finish3 = cv::Mat::zeros(height, width, CV_8U);
    cv::Mat binaryImage_Finish4 = cv::Mat::zeros(height, width, CV_8U);
    cv::Mat binaryImage_Finish5 = cv::Mat::zeros(height, width, CV_8U);

    Mat preprocessedImg = preprocessImage(frame_a);
    binaryOutputImage = detectLaneEdges(preprocessedImg); // 将结果赋值给全局变量

    int x_roi = 1;
    int y_roi = 100;
    int width_roi = 319;
    int height_roi = 140;
    Rect roi(x_roi, y_roi, width_roi, height_roi);
    vector<Vec4i> lines = findHoughLines(binaryOutputImage, roi);

    classifyAndDrawLines(lines, x_roi, y_roi, binaryImage_Finish3, binaryImage_Finish4, binaryImage_Finish5);

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
    Mat croppedObject = binaryOutputImage(roi);  // 提取ROI区域进行霍夫变换

    // 霍夫变换检测直线
    HoughLinesP(croppedObject, lines, 1, CV_PI / 180, houghLinesPThreshold, minLineLength, maxLineGap);

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
                ((l[0] + l[2] < 319) ? (k < 0 && abs(angle_k - leftLineAngleDeg) < 18 && abs(115 - rightLineXAtRefY + x6) < 10) : 
                 (k > 0 && abs(angle_k - rightLineAngleDeg) < 18 && abs(115 + leftLineXAtRefY - x6) < 10))) {
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
    switch (trackingStrategyFlag) {
    case 0:  // 策略1：使用binaryImage_Finish4
        if (Tracking(binaryImage_Finish4)) {
            finalBinaryLaneImage = binaryImage_Finish4;
            return finalBinaryLaneImage;
        } else {
            trackingStrategyFlag = 1;  // 策略1失败，切换到策略2
            goto Tracking_start;
        }
        break;

    case 1:  // 策略2：使用binaryImage_Finish5
        if (Tracking(binaryImage_Finish5)) {
            trackingStrategyFlag = 0;  // 策略2成功，下次使用策略1
            finalBinaryLaneImage = binaryImage_Finish5;
            return finalBinaryLaneImage;
        } else {
            trackingStrategyFlag = 2;  // 策略2失败，切换到策略3
            goto Tracking_start;
        }
        break;

    case 2:  // 策略3：使用binaryImage_Finish3（保底策略）
        Tracking(binaryImage_Finish3);
        finalBinaryLaneImage = binaryImage_Finish3;
        trackingStrategyFlag = 0;  // 重置为策略1
        return finalBinaryLaneImage;
        break;
    }
    
    return finalBinaryLaneImage;
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
    Mat croppedObject = binaryOutputImage(roi); // 使用规范化后的 binaryOutputImage

    // 霍夫变换检测线段
    HoughLinesP(croppedObject, lines, 1, CV_PI / 180, houghLinesPThreshold, minLineLength, maxLineGap); // 使用规范化后的 houghLinesPThreshold

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
    blueLeftLanePoints.clear(); // 使用规范化后的 blueLeftLanePoints
    blueRightLanePoints.clear(); // 使用规范化后的 blueRightLanePoints
    blueMiddleLanePoints.clear(); // 使用规范化后的 blueMiddleLanePoints

    for (int i = 99; i >= 61; i--) {
        int find_l = 0, find_r = 0;
        int to_left = begin, to_right = begin;

        // 查找左边线
        while (to_left > 1) {
            if (binaryImage_Finish3.at<uchar>(i, to_left) == 255) {
                find_l = 1;
                blueLeftLanePoints.emplace_back(to_left, i);
                break;
            }
            to_left--;
        }

        // 查找右边线
        while (to_right < 319) {
            if (binaryImage_Finish3.at<uchar>(i, to_right) == 255) {
                find_r = 1;
                blueRightLanePoints.emplace_back(to_right, i);
                break;
            }
            to_right++;
        }

        // 计算中线点
        if (!blueLeftLanePoints.empty() && !blueRightLanePoints.empty()) {
            cv::Point midx1 = blueLeftLanePoints.back();
            cv::Point midx2 = blueRightLanePoints.back();
            blueMiddleLanePoints.emplace_back((midx1.x + midx2.x) / 2, i);
            begin = (to_right + to_left) / 2;
        }
    }

    // 直线拟合
    cv::Vec4f mid_fit;
    if (blueLeftLanePoints.empty() || blueRightLanePoints.empty()) {
        return;
    }

    cv::fitLine(blueMiddleLanePoints, mid_fit, cv::DIST_L2, 0, 0.01, 0.01);
    blueMidlineSlope = mid_fit[1] / mid_fit[0]; // 使用规范化后的 blueMidlineSlope
    blueMidlineIntercept = mid_fit[3] - blueMidlineSlope * mid_fit[2]; // 使用规范化后的 blueMidlineIntercept
}

/**
 * @brief 赛道边界跟踪函数
 * @param dilated_image 输入的二值图像
 * @return 1成功，0失败
 */
int Tracking(Mat dilated_image)
{
    int begin = 160;
    leftLanePoints.clear();
    rightLanePoints.clear();
    middleLanePoints.clear();

    for (int i = 239; i >= 101; i--)
    {
        int find_l = 0, find_r = 0;
        int to_left = begin, to_right = begin;

        while (to_left > 1)
        {
            if (dilated_image.at<uchar>(i, to_left) == 255)
            {
                find_l = 1;
                leftLanePoints.emplace_back(to_left, i);
                break;
            }
            to_left--;
        }

        while (to_right < 319)
        {
            if (dilated_image.at<uchar>(i, to_right) == 255)
            {
                find_r = 1;
                rightLanePoints.emplace_back(to_right, i);
                break;
            }
            to_right++;
        }

        if (!leftLanePoints.empty() && !rightLanePoints.empty())
        {
            cv::Point midx1 = leftLanePoints.back();
            cv::Point midx2 = rightLanePoints.back();
            middleLanePoints.emplace_back((midx1.x + midx2.x) / 2, i);
            begin = (to_right + to_left) / 2;
        }
    }

    cv::Vec4f left_fit, right_fit;
 
    if (leftLanePoints.empty() || rightLanePoints.empty()) 
    {
        if (leftLanePoints.empty())
            LeftVirtualLine = Line(std::numeric_limits<double>::infinity(), cv::Point(1, 110));
        else
            LeftVirtualLine = Line(leftLineSlope, cv::Point(leftLineXAtRefY, 110));

        if (rightLanePoints.empty())
            RightVirtualLine = Line(std::numeric_limits<double>::infinity(), cv::Point(319, 110));
        else
            RightVirtualLine = Line(rightLineSlope, cv::Point(rightLineXAtRefY, 110));

        Tracking_barrel(finalBinaryLaneImage); // 使用规范化后的 finalBinaryLaneImage
        return 0;
    }

    cv::fitLine(leftLanePoints, left_fit, cv::DIST_L2, 0, 0.01, 0.01);
    cv::fitLine(rightLanePoints, right_fit, cv::DIST_L2, 0, 0.01, 0.01);

    leftLineSlope = left_fit[1] / left_fit[0]; // 使用规范化后的 leftLineSlope
    float b_left = left_fit[3] - leftLineSlope * left_fit[2];

    rightLineSlope = right_fit[1] / right_fit[0]; // 使用规范化后的 rightLineSlope
    float b_right = right_fit[3] - rightLineSlope * right_fit[2];

    leftLineAngleDeg = std::atan(leftLineSlope) * 180.0 / CV_PI; // 使用规范化后的 leftLineAngleDeg
    rightLineAngleDeg = std::atan(rightLineSlope) * 180.0 / CV_PI; // 使用规范化后的 rightLineAngleDeg

    if (fabs(rightLineAngleDeg) > 89.5 || abs(rightLineAngleDeg) < 1)
    {
        leftLineXAtRefY = (100 + 10 - b_left) / leftLineSlope; // 使用规范化后的 leftLineXAtRefY
        LeftVirtualLine = Line(leftLineSlope, cv::Point(leftLineXAtRefY, 110));
        RightVirtualLine = Line(-leftLineSlope, cv::Point(rightLineXAtRefY, 110)); // 使用规范化后的 rightLineXAtRefY
        Tracking_barrel(finalBinaryLaneImage);
        return 0;
    }
    if (fabs(leftLineAngleDeg) > 89.5 || abs(leftLineAngleDeg) < 1)
    {
        rightLineXAtRefY = (100 + 10 - b_right) / rightLineSlope; // 使用规范化后的 rightLineXAtRefY
        LeftVirtualLine = Line(-rightLineSlope, cv::Point(leftLineXAtRefY, 110));
        RightVirtualLine = Line(rightLineSlope, cv::Point(rightLineXAtRefY, 110));
        Tracking_barrel(finalBinaryLaneImage);
        return 0;
    }

    if (stopTurnDetectFlag) // 使用规范化后的 stopTurnDetectFlag
    {
        if (abs(160 - leftLineXAtRefY) < 20)
        {
            rightLineXAtRefY = 319;
            rightLineSlope = std::numeric_limits<double>::infinity();
        }
        else if (abs(160 - rightLineXAtRefY) < 20)
        {
            leftLineXAtRefY = 1;
            leftLineSlope = std::numeric_limits<double>::infinity();
        }
    }

    LeftVirtualLine = Line(leftLineSlope, cv::Point(leftLineXAtRefY, 110));
    RightVirtualLine = Line(rightLineSlope, cv::Point(rightLineXAtRefY, 110));
    Tracking_barrel(finalBinaryLaneImage);

    leftLineXAtRefY = (100 + 10 - b_left) / leftLineSlope;
    rightLineXAtRefY = (100 + 10 - b_right) / rightLineSlope;

    return 1;
}

/**
 * @brief 锥桶模式下的边界跟踪
 * @param dilated_image 输入的二值图像
 */
void Tracking_barrel(Mat dilated_image)
{
    leftLanePoints.clear();
    rightLanePoints.clear();
    middleLanePoints.clear();

    for (int i = 239; i >= 101; i--)
    {
        int X_Left = 0;
        X_Left = (int)LeftVirtualLine.startPoint.x + (i - LeftVirtualLine.startPoint.y) / LeftVirtualLine.slope;
        if (X_Left < 1) X_Left = 1;

        leftLanePoints.emplace_back(cv::Point(X_Left, i));

        int X_Right = 0;
        X_Right = (int)RightVirtualLine.startPoint.x + (i - RightVirtualLine.startPoint.y) / RightVirtualLine.slope;
        if (X_Right > 319) X_Right = 319;

        rightLanePoints.emplace_back(cv::Point(X_Right, i));

        middleLanePoints.emplace_back(cv::Point((X_Left + X_Right) / 2, i));
    }
}

/**
 * @brief 蓝色锥桶检测函数
 * @param frame 输入彩色图像
 * @return 返回检测到的锥桶中心点
 */
Point blue_vertebral_barrel_find(Mat frame)
{
    Point barrel_center = Point(0, 0);

    cvtColor(frame, hsvImage, COLOR_BGR2HSV); // 使用规范化后的 hsvImage

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

    coneDebugFrameCount++; // 使用规范化后的 coneDebugFrameCount
    printf("[CONE DEBUG] 帧%d: 检测到%d个轮廓\n", coneDebugFrameCount, contours.size());

    sort(contours.begin(), contours.end(), [](const vector<Point> &a, const vector<Point> &b)
         { return contourArea(a) > contourArea(b); });

    bool foundValidContour = false;

    if (!contours.empty())
    {
        const vector<Point> &largestContour = contours[0];
        double maxArea = contourArea(largestContour);
        
        printf("[CONE DEBUG] 最大轮廓面积: %.2f, 阈值: %d\n", maxArea, minConeAreaPx); // 使用规范化后的 minConeAreaPx
        
        if (maxArea >= minConeAreaPx)
        {
            Moments m = moments(largestContour);
            int cx = int(m.m10 / m.m00);
            int cy = int(m.m01 / m.m00);
            Point currentCenter(cx, cy);

            previousConeCenters.push_back(currentCenter); // 使用规范化后的 previousConeCenters
            if (previousConeCenters.size() > maxHistoryCount) // 使用规范化后的 maxHistoryCount
            {
                previousConeCenters.erase(previousConeCenters.begin());
            }

            foundValidContour = true;
            printf("[CONE DEBUG] ✓ 找到有效锥桶! 位置: (%d, %d)\n", cx, cy);
            coneDetectedCount++; // 使用规范化后的 coneDetectedCount
        }
        else
        {
            printf("[CONE DEBUG] ✗ 轮廓面积不足: %.2f < %d\n", maxArea, minConeAreaPx);
            coneMissedCount++; // 使用规范化后的 coneMissedCount
        }

        if (maxArea >= minModerateAreaPx && !foundValidContour)
        {
            // 如果轮廓面积达到中等，且未找到有效锥桶（即锥桶可能开始远离或被遮挡），进入普通避障
            vehicleCurrentState = VehicleState::AvoidingNormalCone; 
            printf("[CONE DEBUG] 设置VehicleState=AvoidingNormalCone (中等面积锥桶)\n");
        }
        else if (maxArea < minModerateAreaPx && vehicleCurrentState == VehicleState::AvoidingNormalCone && !foundValidContour && !laterConeProcessingFlag)
        {
            // 如果中等面积锥桶消失，且不在后锥桶处理模式，则重置状态
            vehicleCurrentState = VehicleState::NoTask; 
            printf("[CONE DEBUG] 重置VehicleState=NoTask (锥桶消失)\n");
        }
    }
    else
    {
        printf("[CONE DEBUG] 未检测到任何轮廓\n");
        coneMissedCount++;
    }

    if (foundValidContour)
    {
        detectConsecutiveCount++; // 使用规范化后的 detectConsecutiveCount
        printf("[CONE DEBUG] 连续检测计数: %d/%zu\n", detectConsecutiveCount, maxHistoryCount);

        if (detectConsecutiveCount >= maxHistoryCount)
        {
            missingConsecutiveCount = 0; // 使用规范化后的 missingConsecutiveCount
            isConeFound = true; // 使用规范化后的 isConeFound
            printf("[CONE DEBUG] ★ 确认锥桶存在! isConeFound=true\n");
        }
    }
    else if (isConeFound)
    {
        missingConsecutiveCount++;
        printf("[CONE DEBUG] 丢失计数: %d/%zu\n", missingConsecutiveCount, maxHistoryCount);

        if (missingConsecutiveCount >= maxHistoryCount)
        {
            isConeFound = false;
            totalConesFound++; // 使用规范化后的 totalConesFound
            detectConsecutiveCount = 0;
            printf("[CONE DEBUG] ★ 锥桶消失! 已找到锥桶总数: %d\n", totalConesFound);
        }
    }

    if (!previousConeCenters.empty() && isConeFound)
    {
        for (const Point &center : previousConeCenters)
        {
            barrel_center.x += center.x;
            barrel_center.y += center.y;
        }
        barrel_center.x /= previousConeCenters.size();
        barrel_center.y /= previousConeCenters.size();
        printf("[CONE DEBUG] 平滑后锥桶中心: (%d, %d), 历史帧数: %zu\n", 
               barrel_center.x, barrel_center.y, previousConeCenters.size());
    }

    if (coneDebugFrameCount % 10 == 0)
    {
        float detection_rate = (coneDetectedCount * 100.0f) / coneDebugFrameCount;
        printf("[CONE STATS] 帧数: %d, 检测率: %.1f%%, 检测/丢失: %d/%d\n", 
               coneDebugFrameCount, detection_rate, coneDetectedCount, coneMissedCount);
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
	static int currentFrameCount = 0;
	currentFrameCount++;
	
	printf("[LATER CONE] 后锥桶处理模式, 帧计数: %d/%d, 当前偏移: %d\n", 
	       currentFrameCount, frameCountLimit, pathOffset); // 使用规范化

	if (currentFrameCount >= frameCountLimit) // 使用规范化
	{
		if (vehicleCurrentState == VehicleState::AvoidingMiddleCone) // 使用枚举
		{
			targetOffset = (patchLineDirection == LEFT) ? trackWidthPx + 20 : -trackWidthPx - 20; // 使用规范化
			printf("[LATER CONE] 中间锥桶目标偏移: %d\n", targetOffset);
		}
		else
		{
			targetOffset = (patchLineDirection == LEFT) ? trackWidthPx : -trackWidthPx; // 使用规范化
			printf("[LATER CONE] 普通锥桶目标偏移: %d\n", targetOffset);
		}

		int old_pathOffset = pathOffset;
		pathOffset = Limit(targetOffset, pathOffset, 1);
		printf("[LATER CONE] 偏移量调整: %d -> %d\n", old_pathOffset, pathOffset);
	}

	Point center_offset;

	if (patchLineDirection == LEFT) // 使用规范化
	{
		conePathSlope = -rightLineSlope; // 使用规范化
		center_offset = Point(rightLineXAtRefY - pathOffset, 110); // 使用规范化

		circle(currentImage, center_offset, 3, Scalar(0, 255, 255), -1); // 使用规范化

		LeftVirtualLine = Line(conePathSlope, center_offset); // 使用规范化
		RightVirtualLine = Line(rightLineSlope, cv::Point(rightLineXAtRefY, 110)); // 使用规范化
		
		printf("[LATER CONE] LEFT模式: conePathSlope=%.2f, center_offset=(%d, %d), pathOffset=%d\n", 
		       conePathSlope, center_offset.x, center_offset.y, pathOffset);
	}
	else if (patchLineDirection == RIGHT) // 使用规范化
	{
		conePathSlope = -leftLineSlope; // 使用规范化
		center_offset = Point(leftLineXAtRefY - pathOffset, 110); // 使用规范化

		circle(currentImage, center_offset, 3, Scalar(0, 255, 255), -1); // 使用规范化

		LeftVirtualLine = Line(leftLineSlope, cv::Point(leftLineXAtRefY, 110)); // 使用规范化
		RightVirtualLine = Line(conePathSlope, center_offset); // 使用规范化

		printf("[LATER CONE] RIGHT模式: conePathSlope=%.2f, center_offset=(%d, %d)\n", 
		       conePathSlope, center_offset.x, center_offset.y);
	}

	if (vehicleCurrentState == VehicleState::AvoidingNormalCone) // 使用枚举
	{
		if (abs(pathOffset) == trackWidthPx) // 使用规范化
		{
			printf("[LATER CONE] ★ 普通锥桶补线完成! 重置所有标志位\n");
			currentFrameCount = 0;
			laterConeProcessingFlag = 0; // 使用规范化
			vehicleCurrentState = VehicleState::NoTask; // 使用枚举
			firstConeFoundFlag = 0; // 使用规范化
			return;
		}
		else
		{
			printf("[LATER CONE] 普通锥桶补线进度: %d/%f\n", abs(pathOffset), trackWidthPx); // 使用规范化
		}
	}
	else if (vehicleCurrentState == VehicleState::AvoidingMiddleCone) // 使用枚举
	{
		if (abs(pathOffset) == trackWidthPx + 20) // 使用规范化
		{
			printf("[LATER CONE] ★ 中间锥桶补线完成! 重置所有标志位\n");
			currentFrameCount = 0;
			laterConeProcessingFlag = 0;
			vehicleCurrentState = VehicleState::NoTask;
			firstConeFoundFlag = 0;
			return;
		}
		else
		{
			printf("[LATER CONE] 中间锥桶补线进度: %d/%f\n", abs(pathOffset), trackWidthPx + 20); // 使用规范化
		}
	}

	Tracking_barrel(finalBinaryLaneImage); // 使用规范化
}

/**
 * @brief 锥桶检测和补线函数
 * @param img_ 输入彩色图像
 */
void vertebral_model(Mat &img_)
{
	Point barrel_center = Point(0, 0);

	roiImage = defineTrapezoidROI(img_); // 使用规范化
	barrel_center = blue_vertebral_barrel_find(roiImage); // 使用规范化

	printf("[CONE MODEL] 进入锥桶模型, 锥桶中心: (%d, %d), Later_Flag: %d\n", 
	       barrel_center.x, barrel_center.y, laterConeProcessingFlag); // 使用规范化

	if (laterConeProcessingFlag == 1) // 使用规范化
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

	printf("[CONE MODEL] 找到锥桶, 位置: (%d, %d), Total_Cones_Found: %d\n", 
	       barrel_center.x, barrel_center.y, totalConesFound); // 使用规范化

	if (isCleverConeMode) // 使用规范化
	{
		printf("[CONE MODEL] 使用智能模式\n");
		if (totalConesFound == middleConeSerialNumber - 1) // 使用规范化
		{
			patchLineDirection = RIGHT; // 使用规范化
			vehicleCurrentState = VehicleState::AvoidingMiddleCone; // 使用枚举
			printf("[CONE MODEL] ★ 中间锥桶! 设置补线方向: RIGHT, VehicleState=AvoidingMiddleCone\n");
		}
		else if (totalConesFound == rightConeSerialNumber - 1) // 使用规范化
		{
			patchLineDirection = RIGHT;
			vehicleCurrentState = VehicleState::AvoidingNormalCone; // 使用枚举
			printf("[CONE MODEL] ★ 右边锥桶! 设置补线方向: RIGHT, VehicleState=AvoidingNormalCone\n");
		}
		else if (totalConesFound == leftConeSerialNumber - 1) // 使用规范化
		{
			patchLineDirection = LEFT;
			vehicleCurrentState = VehicleState::AvoidingNormalCone; // 使用枚举
			printf("[CONE MODEL] ★ 左边锥桶! 设置补线方向: LEFT, VehicleState=AvoidingNormalCone\n");
		}
		else
		{
			printf("[CONE MODEL] 未匹配到已知锥桶序列号\n");
		}
	}
	else
	{
		printf("[CONE MODEL] 使用手动模式\n");
		Sobel_LinePro(binaryOutputImage); // 使用规范化
		line(currentImage, Point((99 - blueMidlineIntercept) / blueMidlineSlope, 99), Point((60 - blueMidlineIntercept) / blueMidlineSlope, 60), Scalar(255, 255, 0), 1); // 使用规范化
	}
	circle(currentImage, Point(barrel_center.x, barrel_center.y), 3, Scalar(0, 255, 255), -1); // 使用规范化

	if (barrel_center.y > 41 && barrel_center.y < 60 && !firstConeFoundFlag) // 使用规范化
	{
		printf("[CONE MODEL] 初次发现锥桶! 锥桶Y坐标: %d\n", barrel_center.y);
		
		if (!isCleverConeMode) // 使用规范化
		{
			float s = (barrel_center.y - blueMidlineIntercept) / blueMidlineSlope - barrel_center.x; // 使用规范化
			printf("[CONE MODEL] 离中线距离计算: %.2f\n", s);
			
			if (fabs(s) < 30)
			{
				printf("[CONE MODEL] 距离中线: %.2f\n", s);
				if (fabs(s) > 5)
				{
					if (s < 0)
						patchLineDirection = RIGHT;
					else
						patchLineDirection = LEFT;

					vehicleCurrentState = VehicleState::AvoidingNormalCone; // 使用枚举
					printf("[CONE MODEL] 设置普通锥桶, 补线方向: %s\n", (patchLineDirection == LEFT) ? "LEFT" : "RIGHT");
				}
				else
				{
					if (s < 0)
						patchLineDirection = RIGHT;
					else
						patchLineDirection = LEFT;

					vehicleCurrentState = VehicleState::AvoidingMiddleCone; // 使用枚举
					printf("[CONE MODEL] 设置中间锥桶, 补线方向: %s\n", (patchLineDirection == LEFT) ? "LEFT" : "RIGHT");
				}
			}
		}
		
		if (firstConeFoundFlag != 1)
		{
			pathOffset = (patchLineDirection == LEFT) ? trackWidthPx : -trackWidthPx; // 使用规范化
			printf("[CONE MODEL] 设置初始偏移量: %d\n", pathOffset);
		}

		firstConeFoundFlag = 1; // 使用规范化
		printf("[CONE MODEL] ★ 设置首次发现标志 firstConeFoundFlag=1\n");
	}

	if (barrel_center.y < 180 && firstConeFoundFlag) // 使用规范化
	{
		printf("[CONE MODEL] 初见锥桶处理, 锥桶Y坐标: %d, vehicleCurrentState: %d\n", barrel_center.y, (int)vehicleCurrentState); // 使用枚举
		
		if (vehicleCurrentState == VehicleState::AvoidingMiddleCone) // 使用枚举
		{
			targetOffset = (patchLineDirection == LEFT) ? 10 : -10; // 使用规范化
			pathOffset = Limit(targetOffset, pathOffset, 1); // 使用规范化
			printf("[CONE MODEL] 中间锥桶偏移: %d -> %d\n", targetOffset, pathOffset);
		}
		else if (vehicleCurrentState == VehicleState::AvoidingNormalCone) // 使用枚举
		{
			targetOffset = (patchLineDirection == LEFT) ? 110 : -110; // 使用规范化
			pathOffset = Limit(targetOffset, pathOffset, 2); // 使用规范化
			printf("[CONE MODEL] 普通锥桶偏移: %d -> %d\n", targetOffset, pathOffset);
		}
		
		Point center_offset;
		if (patchLineDirection == LEFT)
		{
			conePathSlope = -rightLineSlope;
			center_offset = Point(rightLineXAtRefY - pathOffset, 110);

			circle(currentImage, center_offset, 3, Scalar(0, 255, 255), -1);

			LeftVirtualLine = Line(conePathSlope, center_offset);
			RightVirtualLine = Line(rightLineSlope, cv::Point(rightLineXAtRefY, 110));
			
			printf("[CONE MODEL] LEFT模式: conePathSlope=%.2f, center_offset=(%d, %d)\n", 
			       conePathSlope, center_offset.x, center_offset.y);
		}
		else if (patchLineDirection == RIGHT)
		{
			conePathSlope = -leftLineSlope;
			center_offset = Point(leftLineXAtRefY - pathOffset, 110);

			circle(currentImage, center_offset, 3, Scalar(0, 255, 255), -1);

			LeftVirtualLine = Line(leftLineSlope, cv::Point(leftLineXAtRefY, 110));
			RightVirtualLine = Line(conePathSlope, center_offset);

			printf("[CONE MODEL] RIGHT模式: conePathSlope=%.2f, center_offset=(%d, %d)\n", 
			       conePathSlope, center_offset.x, center_offset.y);
		}

		Tracking_barrel(finalBinaryLaneImage);
	}
	else if (barrel_center.y > 180 && laterConeProcessingFlag == 0 && firstConeFoundFlag) // 使用规范化
	{
		laterConeProcessingFlag = 1; // 使用规范化
		printf("[CONE MODEL] ★ 锥桶Y坐标>180, 设置后锥桶标志 laterConeProcessingFlag=1\n");
	}
}

// ============================================================================
// 6. 斑马线检测函数
// ============================================================================

/**
 * @brief 斑马线检测函数
 * @param frame 输入彩色图像
 * @return 斑马线标志位
 */
int crossroad(Mat frame)
{
	int height = frame.rows;
	int width = frame.cols;

	cvtColor(frame, hsvImage, COLOR_BGR2HSV); // 使用规范化后的 hsvImage

	Scalar lower_white = Scalar(0, 0, 221);
	Scalar upper_white = Scalar(180, 100, 255);

	Mat mask1;
	inRange(hsvImage, lower_white, upper_white, mask1);

	Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
	morphologyEx(mask1, mask1, MORPH_OPEN, kernel);

	// 根据斑马线计数调整ROI，第一次寻找在图片中间偏上，第二次在图片底部
	if (zebraCrossingCount == 0) 
	{
		roiRect = Rect(1, 100, 319, 60); 
	}
	else
	{
		roiRect = Rect(1, 210, 319, 30);
		// vehicleCurrentState = VehicleState::NoTask; // 这里不应直接重置，应由主循环逻辑控制
	}

	Mat src(mask1, roiRect);

	int transitionCountRow = 0; // 统计一行中黑白/白黑交替的次数
	int consecutiveTransitionRows = 0; // 统计连续满足条件行的数量
	
	// 遍历ROI区域的每一行
	for (int i = 0; i < src.rows; i++)
	{
		transitionCountRow = 0; // 每行重置
		// 遍历当前行的像素
		for (int j = 10; j < src.cols - 10; j++)
		{
			// 检测黑白-白黑的像素模式，这代表斑马线的边缘
			if (src.at<uchar>(i, j - 2) == 0 && src.at<uchar>(i, j - 1) == 0 && src.at<uchar>(i, j) == 0 &&
                src.at<uchar>(i, j + 1) == 255 && src.at<uchar>(i, j + 2) == 255)
			{
				transitionCountRow++; // 发现一个模式
			}
			else if (src.at<uchar>(i, j - 2) == 255 && src.at<uchar>(i, j - 1) == 255 && src.at<uchar>(i, j) == 255 &&
                     src.at<uchar>(i, j + 1) == 0 && src.at<uchar>(i, j + 2) == 0)
			{
				transitionCountRow++; // 发现一个模式
			}
		}

		if (transitionCountRow >= 10) // 如果一行中检测到足够的模式，认为此行包含斑马线特征
		{
			consecutiveTransitionRows++; // 连续满足条件的行数增加
			if (consecutiveTransitionRows >= 5) // 如果连续5行都满足条件，则判定为斑马线
			{
				if (zebraCrossingCount == 0) // 第一次检测到斑马线
				{
					vehicleCurrentState = VehicleState::FindingZebraCrossing; // 设置寻找斑马线状态
				}
				else // 第二次检测到斑马线（通常在变道后）
				{
					isFirstZebraCrossing = 1; // 设置首次斑马线标志 (注意命名，这可能意味着第二次检测到)
					vehicleCurrentState = VehicleState::StoppingAtZebraCrossing; // 设置停车等待状态
				}
				break; // 找到后立即退出循环
			}
		} else {
            consecutiveTransitionRows = 0; // 连续性中断
        }
	}

    // 斑马线计数逻辑 (原 flag_cross_num)
	static int zebraCrossingInternalCount = 0; // 内部计数器
	if (vehicleCurrentState == VehicleState::FindingZebraCrossing) 
	{
		zebraCrossingInternalCount++;
		if (zebraCrossingInternalCount % 3 == 0) // 每隔3帧更新一次斑马线计数，用于防止抖动
		{
			zebraCrossingCount = 1; // 第一次检测到斑马线后，将其置为1
		}
	} else {
        zebraCrossingInternalCount = 0; // 如果状态不是寻找斑马线，则重置计数
    }
	return (int)vehicleCurrentState; // 返回枚举的底层int值
}

// ============================================================================
// 7. 转向标志识别函数
// ============================================================================

/**
 * @brief 转向标志识别函数
 * @param frame 输入彩色图像
 */
void Judge_Turn_Pro(Mat frame)
{
	cvtColor(frame, hsvImage, COLOR_BGR2HSV); // 使用规范化后的 hsvImage

	Scalar lower_blue(hMin, sMin, vMin); // 使用规范化后的 hMin等
	Scalar upper_blue(hMax, sMax, vMax); // 使用规范化后的 hMax等

	Mat mask;
	inRange(hsvImage, lower_blue, upper_blue, mask);

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

	if (findTurnSignFlag == 1) // 使用规范化
	{
		Count++;
		if (Count % 10 == 0)
		{
			laneChangeDetectCount[0] = 0; // 使用规范化
			laneChangeDetectCount[1] = 0;
			turnDirectionFlag = 0; // 使用规范化
			stopTurnDetectFlag = 1; // 使用规范化
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

			Mat roi_upper = combined_mask(Rect(0, 0, combined_mask.cols, combined_mask.rows / 3)).clone(); // 规范化变量名

			Mat distance_matrix(roi_upper.size(), CV_32F);
			for (int y = 0; y < roi_upper.rows; y++)
			{
				for (int x = 0; x < roi_upper.cols; x++)
				{
					float relative_pos = (x - x0) * vy - (y - y0) * vx;
					distance_matrix.at<float>(y, x) = relative_pos;
				}
			}

			Mat left_mask = (distance_matrix < 0) & (roi_upper == 255);
			Mat right_mask = (distance_matrix > 0) & (roi_upper == 255);

			int left_count = countNonZero(left_mask);
			int right_count = countNonZero(right_mask);

			if (left_count >= right_count)
			{
				tempTurnDirectionFlag = 1; // 使用规范化
			}
			else
			{
				tempTurnDirectionFlag = 2; // 使用规范化
			}
		}

		if (tempTurnDirectionFlag == 1) // 使用规范化
		{
			laneChangeDetectCount[0]++; // 使用规范化
		}
		else if (tempTurnDirectionFlag == 2) // 使用规范化
		{
			laneChangeDetectCount[1]++;
		}

		if (laneChangeDetectCount[0] > laneChangeDetectCount[1] && laneChangeDetectCount[0] > 8) // 使用规范化
		{
			turnDirectionFlag = 1; // 使用规范化
			findTurnSignFlag = 1; // 使用规范化
		}
		else if (laneChangeDetectCount[0] < laneChangeDetectCount[1] && laneChangeDetectCount[1] > 8) // 使用规范化
		{
			turnDirectionFlag = 2; // 使用规范化
			findTurnSignFlag = 1; // 使用规范化
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
		controlBiasLineInt += middleLanePoints[i].x; // 使用规范化
	}
	controlBiasLineInt = controlBiasLineInt / 50 - 160; // 使用规范化

	if (controlBiasLineInt > 80)
		controlBiasLineInt = 80;
	else if (controlBiasLineInt < -80)
		controlBiasLineInt = -80;
	controlBiasLine = controlBiasLineInt; // 使用规范化
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
		circle(currentImage, Point(int(leftLanePoints[j].x), i), 0.1, Scalar(0, 0, 0), 2); // 使用规范化
		circle(currentImage, Point(int(rightLanePoints[j].x), i), 0.1, Scalar(0, 0, 0), 2); // 使用规范化
		circle(currentImage, Point(int((leftLanePoints[j].x + rightLanePoints[j].x) / 2), i), 0.1, Scalar(255, 0, 0), 2); // 使用规范化
		j++;
	}

	line(currentImage, Point(160, 100), Point(160, 239), Scalar(0, 0, 255), 2); // 使用规范化
	rectangle(currentImage, Rect(1, 100, 319, 239), Scalar(0, 255, 0), 1); // 使用规范化

	std::ostringstream oss;
	oss << "Bias_Line: " << static_cast<int>(controlBiasLine); // 使用规范化
	std::string text = oss.str();
	cv::putText(currentImage, text, cv::Point(0, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1); // 使用规范化

	oss.str("");
	oss.clear();
	oss << "Track_width: " << static_cast<int>(rightLineXAtRefY - leftLineXAtRefY); // 使用规范化
	text = oss.str();
	cv::putText(currentImage, text, cv::Point(0, 80), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1); // 使用规范化

	oss.str("");
	oss.clear();
	switch (vehicleCurrentState)
	{
	case VehicleState::FindingZebraCrossing: oss << "State: Finding Zebra"; break;
	case VehicleState::NoTask: oss << "State: Following Lane"; break;
	case VehicleState::StoppingAtZebraCrossing: oss << "State: Stop Car"; break;
	case VehicleState::AvoidingMiddleCone: oss << "State: Avoiding Middle Cone (" << totalConesFound + 1 << ")"; break;
	case VehicleState::AvoidingNormalCone: oss << "State: Avoiding Normal Cone (" << totalConesFound + 1 << ")"; break;
	case VehicleState::Stopping: oss << "State: Stopping"; break;
    case VehicleState::TurningLeft: oss << "State: Turning Left"; break;
    case VehicleState::TurningRight: oss << "State: Turning Right"; break;
    case VehicleState::ReturningFromLaneChange: oss << "State: Returning From Lane Change"; break;
	default: oss << "State: Unknown"; break;
	}
	text = oss.str();
	cv::putText(currentImage, text, cv::Point(0, 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1); // 使用规范化

	oss.str("");
	oss.clear();
	switch (turnDirectionFlag) // 使用规范化
	{
	case 0:
		oss << "Turn_Flag: Straight";
		break;
	case 1:
		arrowedLine(currentImage, Point(150, 90), Point(100, 90), Scalar(0, 255, 0), 2, 8, 0, 0.5); // 使用规范化
		oss << "Turn_Flag: Left";
		break;
	case 2:
		arrowedLine(currentImage, Point(150, 90), Point(200, 90), Scalar(0, 255, 0), 2, 8, 0, 0.5); // 使用规范化
		oss << "Turn_Flag: Right";
		break;
	default:
		oss << "Turn_Flag: Unknown";
		break;
	}
	text = oss.str();
	cv::putText(currentImage, text, cv::Point(0, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1); // 使用规范化
}

/**
 * @brief HSV参数调节回调函数
 */
void updateHSV(int, void *)
{
	inRange(hsvImage, Scalar(hMin, sMin, vMin), Scalar(hMax, sMax, vMax), outputHsvImage); // 使用规范化
	imshow("Output Image", outputHsvImage); // 使用规范化
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
	memcpy(buffer, &uartHeader, sizeof(int8_t)); // 使用规范化
	memcpy(buffer + 1, &controlBiasLine, sizeof(int8_t)); // 使用规范化
	memcpy(buffer + 2, &vehicleCurrentState, sizeof(int8_t)); // 使用枚举
	memcpy(buffer + 3, &turnDirectionFlag, sizeof(int8_t)); // 使用规范化
	memcpy(buffer + 4, &uartTail, sizeof(int8_t)); // 使用规范化

	printf("[UART DEBUG] 发送数据包: HEAD=0x%02X, Bias=%d, FlagCross=%d, R_L_Flag=%d, TAIL=0x%02X\n", 
	       uartHeader, controlBiasLine, (int)vehicleCurrentState, turnDirectionFlag, uartTail); // 使用规范化

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
	auto now = std::chrono::system_clock::now(); // 改用 system_clock
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
	std::ostringstream oss;
	oss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
	return oss.str();
}
#endif

// ============================================================================
// 10. 主函数
// ============================================================================

#if ubuntu
int main(int argc, char *argv[])
{
#else
int main()
{
#endif 
    int mode = 1; // 默认为实时比赛模式
#if ubuntu
    if (argc > 1) {
        mode = atoi(argv[1]);
        if (mode < 1 || mode > 4) { // 检查是否是合法的模式
            // 如果第一个参数不是模式，尝试解析为锥桶序列号或录制参数
            // 这里根据原始逻辑，如果mode无效且有第二个参数，则继续按旧逻辑解析argv[1]为锥桶序列或录制
            // 否则退回到默认模式1
            if (argc > 2) { 
                // mode = 1; // 保持默认模式，后续用argv[2]解析锥桶/录制
            } else {
                 mode = 1; // 如果只有argv[1]且不是模式，则退回默认模式1
            }
        }
    }
#endif

    switch (mode)
    {
        case 1: // 实时比赛模式
        {
#if ubuntu
            if (!Serial_Init())
            {
                return 0;
            }
#endif
#if ubuntu
            cv::VideoCapture capture("/dev/video0", cv::CAP_V4L2);
            cv::Size resolution(320, 240);
            capture.set(cv::CAP_PROP_FRAME_WIDTH, resolution.width);
            capture.set(cv::CAP_PROP_FRAME_HEIGHT, resolution.height);
#else
            VideoCapture capture;
            capture.open(testVideoPath); // 使用规范化
#endif

#if ubuntu
            auto start = std::chrono::high_resolution_clock::now();
            int frameCount = 0;
            float FPS = 0;
#endif

            cv::VideoWriter writer;
            uint8_t handle = 0;

#if ubuntu
            // 处理第二个命令行参数（录制或锥桶序列号）
            if (argc > 1) 
            {
                std::string param = "";
                if (mode >= 1 && mode <=4) { // 如果第一个参数是模式，则param取argv[2]
                    if(argc > 2) param = argv[2];
                } else { // 否则param取argv[1] (这是旧的逻辑，保持兼容性)
                    param = argv[1];
                }

                if (!param.empty()) {
                    std::string outputPath;

                    if (param == "6") // 录制原图
                    {
                        outputPath = "/home/dgdz/桌面/原图/" + getCurrentTimeString() + ".mp4";
                        std::cout << "开始录原图" << std::endl;
                        handle = 2;
                    }
                    else if (param == "1") // 录制处理图
                    {
                        outputPath = "/home/dgdz/桌面/处理图/" + getCurrentTimeString() + ".mp4";
                        std::cout << "开始录处理图" << std::endl;
                        handle = 1;
                    }
                    else // 设置锥桶序列号
                    {
                        if(param.length() >= 3) {
                            leftConeSerialNumber = param[0] - '0'; // 从字符转数字
                            middleConeSerialNumber = param[1] - '0';
                            rightConeSerialNumber = param[2] - '0';
                            printf("Left_barrel_serial_number: %d\n", leftConeSerialNumber);
                            printf("Mid_barrel_serial_number: %d\n", middleConeSerialNumber);
                            printf("Right_barrel_serial_number: %d\n", rightConeSerialNumber);
                        }
                        isCleverConeMode = true; // 启用智能锥桶模式
                    }

                    // 录制视频初始化
                    if (param == "6" || param == "1")
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
            }
#endif

            while (capture.read(currentImage)) // 使用规范化
            {
#if ubuntu
                frameCount++;
#else
                resize(currentImage, currentImage, Size(320, 240)); // 使用规范化
#endif

                if (handle == 2) writer.write(currentImage); // 使用规范化
                ImagePreprocessing_Sobel_LinePro(currentImage); // 使用规范化

                // 根据当前车辆状态执行不同的任务
                if (vehicleCurrentState != VehicleState::StoppingAtZebraCrossing && stopTurnDetectFlag != 1)
                {
                    // 如果不是停车状态且未停止转向检测，进行锥桶或斑马线检测
                    if (isFirstZebraCrossing != 1) // 未首次识别斑马线时
                    {
                        vertebral_model(currentImage);
                        if (vehicleCurrentState == VehicleState::NoTask) { // 如果锥桶模型未改变状态，则检测斑马线
                            crossroad(currentImage);
                        }
                    } else { // 首次识别斑马线后，停止检测锥桶，专注于斑马线和转向
                        crossroad(currentImage);
                    }
                }
                
                // 转向标志检测在斑马线后触发
                if (isFirstZebraCrossing == 1 && stopTurnDetectFlag != 1)
                {
                    roiImage = defineTrapezoidROI(currentImage);
                    Judge_Turn_Pro(roiImage);
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
                cv::putText(currentImage, text, cv::Point(230, 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1); // 使用规范化
#endif

                if (handle == 1) writer.write(currentImage); // 使用规范化

                resize(currentImage, currentImage, cv::Size(), 2.5, 2.5); // 使用规范化
                imshow("img_finall", currentImage); // 使用规范化

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
        }
        case 2: // 单张图片测试模式
        {
            currentImage = imread(testFramePath); // 使用规范化
            Judge_Turn_Pro(currentImage); // 使用规范化
            imshow("img_finall", currentImage); // 使用规范化
            waitKey(0);
            return 0;
        }
        case 3: // HSV参数调试模式
        {
            currentImage = imread(testFramePath); // 使用规范化
            cvtColor(currentImage, hsvImage, COLOR_BGR2HSV); // 使用规范化
            namedWindow("Original Image", WINDOW_AUTOSIZE);
            namedWindow("Output Image", WINDOW_AUTOSIZE);
            imshow("Original Image", currentImage); // 使用规范化
            createTrackbar("H Min", "Output Image", &hMin, 179, updateHSV); // 使用规范化
            createTrackbar("H Max", "Output Image", &hMax, 179, updateHSV); // 使用规范化
            createTrackbar("S Min", "Output Image", &sMin, 255, updateHSV); // 使用规范化
            createTrackbar("S Max", "Output Image", &sMax, 255, updateHSV); // 使用规范化
            createTrackbar("V Min", "Output Image", &vMin, 255, updateHSV); // 使用规范化
            createTrackbar("V Max", "Output Image", &vMax, 255, updateHSV); // 使用规范化
            updateHSV(0, 0);
            waitKey(0);
            return 0;
        }
        case 4: // 视频帧导出模式
        {
            string savePath = "/home/dgdz/桌面/bike_video/video/nk6/"; // 修改为Linux路径
            VideoCapture cap;
            cap.open(testVideoPath); // 使用规范化
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
        }
        default:
            cout << "无效的模式！请使用 1-4 之间的数字。\n"
                 << "使用方法: ./main [模式数字] [模式1可选参数]\n"
                 << "模式1可选参数: \n"
                 << "  '1' 或 '6' 用于录制视频 (处理后/原始)\n"
                 << "  'XYZ' (如 '123') 用于设置锥桶序列号 (Left-Mid-Right)\n"
                 << endl;
            return -1;
    }
}
