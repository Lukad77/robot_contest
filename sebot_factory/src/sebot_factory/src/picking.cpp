/**
 * @file picking.cpp
 * @author Leo (sasu@saishukeji.com)
 * @brief 智能取件场景服务（机械臂取放零件）
 * @version 0.1
 * @date 2024-03-07
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <geometry_msgs/Twist.h>   // 速控发布类
#include <sensor_msgs/LaserScan.h> // 订阅激光雷达数据
#include <chrono>
#include "../include/detection.hpp"
#include "../include/tools.hpp"
#include "../include/arm.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp> // 确保包含了 ArUco 头文件
#include <algorithm>

using namespace cv;
using namespace std;

/**
 * @brief 智能取件场景类
 *
 */
class Picking
{
private:
    ros::NodeHandle rosNode;         // ROS节点类
    shared_ptr<Detection> detection; // AI目标检测类实例化指针
    VideoCapture captureDeep;        // 摄像头实例化类
    VideoCapture captureRgb;         // 摄像头实例化类
    ros::Subscriber subLaser;        // 订阅激光雷达话题
    ros::Publisher pubCmd;           // 发布速控话题
    long counter = 0;                // 计数器
    float orienOffset = 0.0;         // 姿态方向校正误差
    Talon talon;                     // 机械臂控制类
    

    /**
     * @brief 激光雷达类
     *
     */
    class Rplidar
    {
    public:
        float angle = 1.57;           // 激光雷达探测角度（机器人前方/rad）
        float angleIncrement = 0.087; // 间隔取点度数rad(≈5°)
        vector<float> rangesLeft;     // 选取左侧距离数据
        vector<float> rangesRight;    // 选取右侧距离数据
    };
    Rplidar rplidar; // 激光雷达类

    /**
     * @brief 目标点控制中心
     *
     */
    class Center
    {
    public:
        int x = COLSIMAGE; // X轴向
        int y = ROWSIMAGE; // Y轴向
    };

    /**
     * @brief 接收激光雷达数据
     * @note angle_min: -3.12413907051  angle_max: 3.14159274101    angle_increment: 0.0174532923847
     *      time_increment: 1.80087170065e-07   scan_time: 6.46512926323e-05    range_min: 0.193000003695
     *      range_max: 8.0
     * @param msg
     */
    void getLaser(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        rplidar.rangesLeft.clear();
        rplidar.rangesRight.clear();

        if (msg->ranges.size() > 0 && msg->angle_increment > 0)
        {
            if (rplidar.angleIncrement < msg->angle_increment)
                rplidar.angleIncrement = msg->angle_increment;
            int numLaser = rplidar.angle / rplidar.angleIncrement;    // 截取的激光点数 (90)
            int steps = msg->ranges.size();                           // 激光雷达总步长
            int step = rplidar.angleIncrement / msg->angle_increment; // 间隔取点步长

            if (steps > numLaser * step)
            {
                for (int i = 0; i < numLaser / 2; i++)
                {
                    if (msg->ranges[i * step] > 0.2 && msg->ranges[i * step] < 1.0)
                        rplidar.rangesLeft.push_back(msg->ranges[i * step]);
                    if (msg->ranges[steps - i * step - 1] > 0.2 && msg->ranges[steps - i * step - 1] < 1.0)
                        rplidar.rangesRight.push_back(msg->ranges[steps - i * step - 1]);
                }
            }
        }
    }

    /**
     * @brief 获取容器数据中值
     *
     * @param stream
     * @return float
     */
    float getMedian(vector<float> stream)
    {
        sort(stream.begin(), stream.end()); // 排序
        int size = stream.size()/3;
        if (size & 1)                 // 位运算判断是否是奇数
            return stream[size]*1.414; // 奇数
        else
            return (stream[size] + stream[size + 1]) *1.414/ 2; // 偶数个
    }

    /**
     * @brief 获取机械爪夹取零件的间距：cm
     *
     * @param foo
     * @return float
     */
    float getClawPosition(string part)
    {
        return 3.5;
    }

    bool validCenter(const Center &center)
    {
        return center.x > -COLSIMAGE && center.x < COLSIMAGE &&
               center.y > -ROWSIMAGE && center.y < ROWSIMAGE;
    }

    void sleepMs(int durationMs)
    {
        if (durationMs > 0)
            usleep(durationMs * 1000);
    }

    bool waitForRgbFrame(Mat &img, int timeoutMs)
    {
        long start = getSystTime();
        while ((getSystTime() - start) < timeoutMs)
        {
            if (captureRgb.read(img))
                return true;
            usleep(30 * 1000);
        }
        return captureRgb.read(img);
    }

    /**
     * @brief 控制机械爪搜索零件位置
     *
     * @param part
     */
    bool clawfindParts(string part)
    {
        Center center = searchPartRgb(part);

        if (validCenter(center)) // 检测到有效的目标零件
        {
            pidClawX.enable = true;
            pidClawY.enable = true;
        }
        else
        {
            pidClawX.enable = false;
            pidClawY.enable = false;
            pidClawX.out = 0;
            pidClawY.out = 0;
        }
        clawControl(center); // 机械爪运动控制
        return validCenter(center);
    }

public:
    Pid pidPose;           // PID姿态控制器：方向（角度）
    Pid pidDis;            // PID姿态控制器：距离（X轴）
    Pid pidLocal;          // PID姿态控制器：位置（Y轴）
    Pid pidClawX;          // PID姿态控制器：机械爪方向（X轴）
    Pid pidClawY;          // PID姿态控制器：机械爪方向（Y轴）
    bool debug = false;    // 调试使能：AI绘制+显示
    bool findPart = false; // AI搜索到目标零件标志
    float score = 0.4;     // AI置信度
    float disSearch = 0.8; // AI搜索零件距离：m
    float disPick = 0.5;   // 机械臂抓取零件距离：m
    float disClaw = 0.3;   // 机械爪夹取零件距离：m
    int placePart = 1;     // 零件放置位置（工作台）：1/中间，2/左边，3/右边

    // 取件台扫描与识别参数：中心视角先识别，失败时才补扫左右边缘，避免每次都增加总耗时。
    int scanCenterDuration = 800;       // 中间视角停留识别时间：ms
    int scanLeftDuration = 2200;        // 左边缘补扫时间：ms，配合低速横扫覆盖取件台
    int scanRightDuration = 2600;       // 右边缘补扫时间：ms，配合低速横扫覆盖取件台
    int scanReturnDuration = 800;       // 扫到左端后的停留识别时间：ms，之后仍未识别就退出本次搜索
    float scanYawOffset = 0.02;         // 左右补扫时给姿态控制的小角度偏置，过大会导致车身偏得太多
    int scanLateralOffset = 60;         // 左右补扫横向诱导像素，降低横扫速度，避免冲出取件台
    int detectionConfirmFrames = 5;     // 连续检测确认帧数
    int detectionMissResetFrames = 8;   // 目标短暂丢失后重置确认计数的帧数
    int pickupSearchTimeout = 13500;    // 单个零件搜索总超时：ms，覆盖低速左右补扫和右扫后的反向回扫
    int partDetectMaxWidth = 100;       // 放宽取件台目标宽度过滤，避免边缘目标被误滤
    int partDetectMaxHeight = 140;      // 放宽取件台目标高度过滤
    int partDetectMinY = ROWSIMAGE / 2 - 20; // 允许目标略高于下半屏，提升边缘视角容错

    // 动作等待参数：保留等待与超时，但集中为可调参数，方便现场按机械臂速度微调。
    int cameraWarmupMs = 300;
    int cameraSwitchSettleMs = 300;
    int cameraSwitchTimeout = 1200;
    int armExtendTimeout = 7000;
    int armPostExtendSettleMs = 800;
    int clawGrabWaitMs = 3200;
    int putActionTimeout = 10000;
    int armPostPutSettleMs = 800;
    int placePreReleaseWaitMs = 1800;
    int placeReleaseWaitMs = 4500;
    float placeApproachOutMax = 0.05;   // 放置阶段靠近工作台的最大速度，过快会导致物体高处掉落
    float placeDropOffsetY = -0.22;     // 放置前机械爪下探高度，负值越大越靠近桌面，现场谨慎微调

    /**
     * @brief 智能取件任务流程
     *
     */
    enum PickStep
    {
        PICK_STEP_START = 0, // 任务开始
        PICK_STEP_POSE,      // 机器人姿态校正
        PICK_STEP_SEARCH,    // 零件搜索
        PICK_STEP_FORWARD,   // 机器人向前移动
        PICK_STEP_AIM,       // 机械爪瞄准零件
        PICK_STEP_GRAB,      // 零件抓取
        PICK_STEP_LOCAL,     // 机器人重定位
        PICK_STEP_END,       // 智能取件结束
    };
    PickStep pickStep = PickStep::PICK_STEP_START; // 智能取件任务流程

    Picking()
    {
        string pathPkg = ros::package::getPath("sebot_factory");   // 功能包文件根路径
        detection = make_shared<Detection>(pathPkg + "/res/model"); // 初始化NPU及AI模型
        detection->score = 0.4;                                     // AI检测置信度
        ros::param::get("/sebot_factory/camera_warmup_ms", cameraWarmupMs);

        // 摄像头初始化
        setCamera(true);

        // ROS节点类
        subLaser = rosNode.subscribe("/scan", 1, &Picking::getLaser, this); // 订阅激光雷达话题
        pubCmd = rosNode.advertise<geometry_msgs::Twist>("/cmd_vel", 5);    // 速控话题发布器

        // PID控制器初始化
        pidPose.enable = true;
        pidDis.enable = true;
        pidDis.ref = 0.3;       // 机器人距离控制：m（激光雷达盲区20cm+10cm距离）
        pidDis.deadline = 0.05; // 机器人距离控制死区：m
        pidDis.outMax = 0.15;   // 限制输出速度：m/s
        pidLocal.deadline = 15; // 图像控制中心死区
        pidLocal.outMax = 0.2;  // 限制输出速度：m/s
        pidClawX.deadline = 10; // 图像控制中心死区
        pidClawX.outMax = 0.3;  // 限制输出角度:rad
        pidClawY.deadline = 5;  // 图像控制中心死区
        pidClawY.outMax = 0.2;  // 限制输出角度:rad
        pidClawY.enable = false;
        sleepMs(cameraWarmupMs);
    };
    ~Picking()
    {
        robotCtrl(0, 0, 0); // 机器人运动控制
        captureDeep.release();
        captureRgb.release();
        talon.close(); // 释放串口线程
    };

    /**
     * @brief 机械臂使能/失能
     */
    void setJointEnable(bool enable)
    {
        talon.setJointEnable(talon.ArmJoint::ARM_JOINT_ALL, enable);
    }

    /**
     * @brief 选择相机通道
     *
     * @param depth 是否选择深度相机
     */
    void setCamera(bool depth)
    {
        if (depth) // 深度相机
        {
            std::string device;
            if (!getVideoDevice(VideoIndex::ASTRA_RGB, device))
                device = "/dev/deepCamera";
            captureDeep = VideoCapture(device,CAP_V4L2); // 打开摄像头
            if (!captureDeep.isOpened())
                ROS_ERROR_STREAM("can not open deepCamera !!!!");
            captureDeep.set(CAP_PROP_FRAME_WIDTH, COLSIMAGE);  // 设置图像的列
            captureDeep.set(CAP_PROP_FRAME_HEIGHT, ROWSIMAGE); // 设置图像的行
        }
        else // RGB相机
        {
            captureDeep.release();
            sleepMs(cameraSwitchSettleMs);
            std::string device;
            if (!getVideoDevice(VideoIndex::ARM_RGB, device))
                device = "/dev/rgbCamera";
            captureRgb = VideoCapture(device,CAP_V4L2); // RGB摄像头初始化
            if (!captureRgb.isOpened())
                ROS_ERROR_STREAM("can not open rgbCamera !!!!");
            captureRgb.set(CAP_PROP_FRAME_WIDTH, COLSIMAGE);  // 设置图像的列
            captureRgb.set(CAP_PROP_FRAME_HEIGHT, ROWSIMAGE); // 设置图像的行
            captureRgb.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M','J','P','G'));  
        }
    }

    /**
     * @brief  机器人运动控制
     *
     * @param linearX X轴线速度
     * @param linearY Y轴线速度
     * @param angular Z轴角速度
     */
    void robotCtrl(float linearX, float linearY, float angular)
    {
        geometry_msgs::Twist msgCmd; // 速控话题数据
        msgCmd.linear.x = linearX;
        msgCmd.linear.y = linearY;
        msgCmd.angular.z = angular;
        pubCmd.publish(msgCmd); // 发布速控话题
    }

    /**
     * @brief 重置PID运行状态
     *
     * 扫描方向切换时清掉上一个方向的累计输出，避免机器人右扫结束后继续带着旧方向惯性，
     * 导致反向左扫只能回到中间，扫不到左端。
     */
    void resetPidRuntime(Pid &pid)
    {
        pid.out = 0;
        pid.preError = 0;
        pid.preDerror = 0;
        pid.counter = 0;
    }

    void switchScanPhase(int &scanPhase, long &scanPhaseStart, int nextPhase, long now)
    {
        scanPhase = nextPhase;
        scanPhaseStart = now;
        resetPidRuntime(pidLocal);
        resetPidRuntime(pidPose);
        robotCtrl(0, 0, 0);
    }

    bool waitForTalonAction(int action, int timeoutMs, int pollMs = 20)
    {
        long start = getSystTime();
        while (!talon.actionOver[action] && (getSystTime() - start) < timeoutMs)
        {
            robotCtrl(0, 0, 0);
            usleep(pollMs * 1000);
        }
        return talon.actionOver[action];
    }

    /**
     * @brief 机器人姿态控制（距离+方向+位置）
     *
     */
    void poseControl()
    {
        float linearX = 0.0, linearY = 0.0, angular = 0.0;

        if (rplidar.rangesLeft.size() >= 3 && rplidar.rangesRight.size() >= 3)
        {
            //[01] 方向控制
            if (pidPose.enable)
            {
                if (orienOffset > 0.3)
                    orienOffset = 0.3;
                else if (orienOffset < -0.3)
                    orienOffset = -0.3;
                pidPose.feedBack = getMedian(rplidar.rangesLeft) - getMedian(rplidar.rangesRight) - orienOffset; // ±0.3
                pidController(pidPose);                                                                          // 姿态PID控制器
                angular = pidPose.out;
            }

            //[02] 距离控制
            if (pidDis.enable)
            {
                if (pidDis.ref < 0.2) // 激光雷达盲区距离
                    pidDis.ref = 0.2;
                pidDis.feedBack = (getMedian(rplidar.rangesLeft) + getMedian(rplidar.rangesRight)) / 2; // ±0.3
                pidController(pidDis);
                linearX = -pidDis.out;
                // if (linearX < 0) // 限制机器人仅向前运动
                //     linearX = 0;
            }
            // if (debug)
            //     ROS_ERROR_STREAM("[PidDis] error:" + to_string(pidDis.feedBack) + "  out:" + to_string(linearX));
        }

        // 位置控制
        if (pidLocal.enable)
        {
            pidController(pidLocal);
            linearY = pidLocal.out;
            // ROS_ERROR_STREAM("error:" + to_string(pidLocal.feedBack) + "  out:" + to_string(linearY));
        }

        robotCtrl(linearX, linearY, angular); // 机器人运动控制
    }

    /**
     * @brief 机械爪运动控制
     *
     */
    void clawControl(Center center)
    {
        if (pidClawX.enable)
        {
            pidClawX.ref = 0;
            pidClawX.feedBack = center.x;
            pdController(pidClawX);
        }

        if (pidClawY.enable)
        {
            pidClawY.ref = 0;
            pidClawY.feedBack = center.y;
            pdController(pidClawY);
        }

        talon.setClawMotion(-pidClawX.out, pidClawY.out); // 机械爪运动控制
    }

    /**
     * @brief 搜索零件
     *
     * @param part 零件AI标签
     * @return Center 返回控制中心：center
     */
    Center searchPartDepth(string part)
    {
        Center center;
        center.x = COLSIMAGE;
        center.y = ROWSIMAGE;
        Mat img;
        if (!captureDeep.read(img)) // 相机采集数据
            return center;

        // 启动AI推理
        detection->score = score;
        detection->inference(img); // NPU推理

        vector<PredictResult> results;
        for (int i = 0; i < detection->results.size(); i++)
        {
            // 边缘视角下目标框会发生形变，过滤阈值放到参数里，现场可按误检情况收紧。
            if (detection->results[i].label == part &&
                detection->results[i].width < partDetectMaxWidth &&
                detection->results[i].height < partDetectMaxHeight)
                results.push_back(detection->results[i]);
        }
        PredictResult result;
        result.score = 0;
        result.height = 0;
        result.width = 0;
        result.x = 0;
        result.y = 0;
        for (int i = 0; i < results.size(); i++)
        {
            // if (results[i].score >= result.score)//选择分数最优的目标物
            int row = results[i].y + results[i].height / 2;
            if (row > (result.y + result.height / 2) && row > partDetectMinY) // 选择图像靠下的目标物，允许边缘补扫时略高
                result = results[i];
        }

        // 控制中心求取
        if (result.label == part)
        {
            center.x = result.x + result.width / 2 - COLSIMAGE / 2; // 校正相机中值（-30）
            center.y = result.y + result.height / 2 - ROWSIMAGE / 2;
        }

        if (debug) // 调试模式
        {
            // 绘制目标框
            auto score = std::to_string(result.score);
            int pointY = result.y - 20;
            if (pointY < 0)
                pointY = 0;
            cv::Rect rectText(result.x, pointY, result.width, 20);
            cv::rectangle(img, rectText, cv::Scalar(0, 255, 0), -1);
            std::string label_name = result.label + " [" + score.substr(0, score.find(".") + 2) + "]";
            cv::Rect rect(result.x, result.y, result.width, result.height);
            cv::rectangle(img, rect, cv::Scalar(0, 255, 0), 1);
            cv::putText(img, label_name, Point(result.x, result.y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 254), 1);

            imshow("img", img);
            waitKey(1);
        }

        return center;
    }

    /**
     * @brief 搜索零件(RGB相机)
     *
     * @param part 零件AI标签
     * @return Center 返回控制中心：center
     */
    Center searchPartRgb(string part) 
    {
        Center center;
        center.x = COLSIMAGE;
        center.y = ROWSIMAGE;
        Mat img;
        
        if (!captureRgb.read(img)) 
        {
            setCamera(false); // 调用你现有的切换/初始化函数
            return center;
        }
            

        // --- ArUco 检测配置 ---
        vector<int> ids;
        vector<vector<Point2f>> corners;
        Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
        
        // 执行检测
        aruco::detectMarkers(img, dictionary, corners, ids);

        int targetIdx = -1;
        float minDistanceToCenter = 1e10;

        // --- 筛选 ID 为 1 且离图像中心最近的标签 ---
        for (int i = 0; i < ids.size(); i++) {
            if (ids[i] == 1) {
                // 计算当前 ArUco 的几何中心
                Point2f markerCenter(0, 0);
                for (auto& corner : corners[i]) {
                    markerCenter += corner;
                }
                markerCenter.x /= 4.0;
                markerCenter.y /= 4.0;

                // 计算该点到图像中心的距离
                float dx = markerCenter.x - COLSIMAGE / 2.0;
                float dy = markerCenter.y - ROWSIMAGE / 2.0;
                float dist = sqrt(dx * dx + dy * dy);

                if (dist < minDistanceToCenter) {
                    minDistanceToCenter = dist;
                    targetIdx = i;
                }
            }
        }

        // --- 计算并赋值 Center ---
        if (targetIdx != -1) {
            // 计算目标 ArUco 的中心点
            Point2f finalMarkerCenter(0, 0);
            for (auto& corner : corners[targetIdx]) {
                finalMarkerCenter += corner;
            }
            finalMarkerCenter.x /= 4.0;
            finalMarkerCenter.y /= 4.0;

            // 保持原有的校正逻辑
            center.x = (int)finalMarkerCenter.x - COLSIMAGE / 2;
            // 模仿原代码对机械爪的微调：Y轴加上一个偏移量 (ArUco 中心点近似于原 result.y + height*0.5)
            center.y = (int)finalMarkerCenter.y - ROWSIMAGE / 2; 
        }
        else    //未找到 ArUco，搜索蓝色方块
        {
            Mat hsv, mask;
            cvtColor(img, hsv, COLOR_BGR2HSV);
            // 定义蓝色的 HSV 范围（根据环境光线可能需要微调）
            // H: 100-124 是典型蓝色范围
            Scalar low_blue = Scalar(100, 100, 46);
            Scalar high_blue = Scalar(124, 255, 255);
            inRange(hsv, low_blue, high_blue, mask);

            // 形态学处理：去除噪点
            Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
            morphologyEx(mask, mask, MORPH_OPEN, kernel);

            vector<vector<Point>> contours;
            findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

            double maxArea = 0;
            int bestContourIdx = -1;
            float minBlueDist = 1e10;

            for (int i = 0; i < contours.size(); i++) 
            {
                double area = contourArea(contours[i]);
                if (area > 500) 
                {   
                    // 过滤掉太小的干扰点.
                    Rect r = boundingRect(contours[i]);
                    float bX = r.x + r.width / 2.0;
                    float bY = r.y + r.height / 2.0;
                    float d = sqrt(pow(bX - COLSIMAGE/2.0, 2) + pow(bY - ROWSIMAGE/2.0, 2));
                    
                    if (d < minBlueDist)
                    {
                        minBlueDist = d;
                        bestContourIdx = i;
                    }
                }
            }

            if (bestContourIdx != -1) {
                Rect finalRect = boundingRect(contours[bestContourIdx]);
                center.x = (finalRect.x + finalRect.width / 2) - COLSIMAGE / 2;
                center.y = (finalRect.y + finalRect.height / 2) - ROWSIMAGE / 2;
                
                if (debug) 
                {
                    rectangle(img, finalRect, Scalar(0, 0, 255), 2); // 蓝色框标识
                }
            }
        }

        // --- 调试模式 (保持原样) ---
        if (debug) {
            if (targetIdx != -1) {
                // 绘制 ArUco 边框和 ID
                aruco::drawDetectedMarkers(img, corners, ids);
                cv::line(img, Point(COLSIMAGE/2, 0), Point(COLSIMAGE/2, ROWSIMAGE), Scalar(255,0,0), 1);
                cv::line(img, Point(0, ROWSIMAGE/2), Point(COLSIMAGE, ROWSIMAGE/2), Scalar(255,0,0), 1);
            }
            imshow("img", img);
            waitKey(1);
        }
        return center;
    }


    /**
     * @brief 智能取件(零件抓取场景)
     *
     * @param part 目标零件AI标签
     * @return true 返回任务完成状态
     * @return false
     */
    bool pickupSomething(string part)
    {
        static int count = 0; // AI检测计数器
        static int scene = 0; // 图像场计数器
        static int scanPhase = 0; // 0:中心 1:右端补扫 2:从右端扫到左端 3:左端停留
        static long scanPhaseStart = getSystTime();

        switch (pickStep)
        {
        case PickStep::PICK_STEP_START: // 任务开始
        {
            pickStep = PickStep::PICK_STEP_POSE; // 切换智能取件任务流程
            count = 0;                           // AI检测计数器
            scene = 0;                           // 图像场计数器
            scanPhase = 0;
            scanPhaseStart = getSystTime();
            resetPidRuntime(pidLocal);
            resetPidRuntime(pidPose);
            orienOffset = 0;
            findPart = false;
            counter = getSystTime();
            break;
        }

        case PickStep::PICK_STEP_POSE: // 机器人姿态校正
        {
            Center center = searchPartDepth(part);
            if (validCenter(center)) // 检测到有效的目标零件
            {
                pidLocal.enable = true; // 使能位置PID控制
                pidLocal.feedBack = center.x;
                count++;
                scene = 0;
                if (count >= detectionConfirmFrames)
                    findPart = true;
            }
            else
            {
                pidLocal.enable = false;
                if (!findPart && count > 0)
                {
                    scene++;
                    if (scene > detectionMissResetFrames)
                    {
                        scene = 0;
                        count = 0;
                    }
                }
            }
            pidDis.ref = disSearch; // AI搜索零件距离：m
            poseControl();          // 机器人姿态控制（距离+方向+位置）

            // 姿态校正状态检测
            if ((abs(pidDis.feedBack - pidDis.ref) < 0.1 &&
                 abs(pidPose.feedBack) < 0.05) ||
                (getSystTime() - counter) > 3000) // 姿态调制至误差范围or超时
            {
                for (int i = 0; i < 5; i++) // 机器人停止运动
                {
                    robotCtrl(0, 0, 0);
                    talon.setActions(talon.ACTION_EXT);             // 机械臂动作：伸展
                    usleep(30 * 1000);                              // 等待：us
                    talon.setJointPosition(talon.ARM_JOINT_6, 9.0); // 机械爪张开：9cm
                }
                if (abs(pidLocal.feedBack - pidLocal.ref) <= pidLocal.deadline && findPart) // 面对目标零件姿态已校正Ok
                {
                    Mat img;
                    counter = getSystTime(); // 更新计时器
                    robotCtrl(0, 0, 0);   // 机器人运动控制
                    orienOffset = 0;      // 姿态校正误差=0
                    // 先等待机械臂伸展到位，再打开RGB相机，避免机械臂未伸直时相机视角和夹爪位置都不稳定。
                    if (!waitForTalonAction(talon.ACTION_EXT, armExtendTimeout))
                        ROS_ERROR_STREAM("机械臂伸展等待超时，继续尝试取件");
                    sleepMs(armPostExtendSettleMs);                  // 伸展完成后的稳定等待，现场按机械臂速度调参
                    setCamera(false);                               // 切换RGB相机
                    if (!waitForRgbFrame(img, cameraSwitchTimeout)) // 检测到RGB出图就立刻继续
                        ROS_ERROR_STREAM("相机打开失败，正在重新尝试...");
                    talon.setClawInitial();                 // 初始化（标定）机械爪
                    pickStep = PickStep::PICK_STEP_FORWARD; // 切换智能取件任务流程
                    pidPose.outMax = 0.2;                   // 限制角速度：rad/s
                    pidDis.outMax = 0.1;                    // 限制输出速度：m/s
                    pidLocal.enable = false;                // 位置PID控制
                    count = 0;
                    pidDis.ref = disSearch;
                    //pidDis.ref = disPick; // 机械臂抓取零件距离：m
                    ROS_ERROR_STREAM("----------- STEP: " + to_string(pickStep) + " -----------");
                    robotCtrl(0, 0, 0);      // 机器人运动控制
                    counter = getSystTime(); // 更新计时器
                }
                else
                {
                    pickStep = PickStep::PICK_STEP_SEARCH; // 切换智能取件任务流程
                    ROS_ERROR_STREAM("----------- STEP: " + to_string(pickStep) + " -----------");
                    counter = getSystTime();
                    scanPhase = 0;
                    scanPhaseStart = getSystTime();
                    resetPidRuntime(pidLocal);
                    resetPidRuntime(pidPose);
                    orienOffset = 0;
                }

                count = 0;
                scene = 0;
                findPart = false;
            }
        }
        break;

        case PickStep::PICK_STEP_SEARCH: // 零件搜索
        {
            Center center = searchPartDepth(part);
            if (validCenter(center)) // 检测到有效的目标零件
            {
                pidLocal.enable = true; // 使能位置PID控制
                pidLocal.feedBack = center.x;
                count++;
                scene = 0;
                orienOffset = 0;
                if (count >= detectionConfirmFrames)
                    findPart = true;
            }
            else
            {
                if (!findPart && count > 0)
                {
                    scene++;
                    if (scene > detectionMissResetFrames)
                    {
                        scene = 0;
                        count = 0;
                    }
                }
            }

            if (!findPart && !validCenter(center)) // 如果AI未检测到零件，按中心-右端-左端顺序补扫
            {
                pidLocal.enable = true;
                pidPose.enable = true;  //同时保证机器人身体不歪
                long now = getSystTime();
                long phaseElapsed = now - scanPhaseStart;
                int fullCrossDuration = scanRightDuration + scanLeftDuration; // 从右端扫到左端需要穿过中间，时间要覆盖整张取件台。

                if (scanPhase == 0)
                {
                    pidLocal.feedBack = 0;
                    orienOffset = 0;
                    if (phaseElapsed >= scanCenterDuration)
                    {
                        switchScanPhase(scanPhase, scanPhaseStart, 1, now);
                    }
                }
                else if (scanPhase == 1)
                {
                    // 右端补扫：保持当前现场验证过的方向，先扫完右端。
                    pidLocal.feedBack = scanLateralOffset;
                    orienOffset = -scanYawOffset;
                    if (phaseElapsed >= scanRightDuration)
                    {
                        switchScanPhase(scanPhase, scanPhaseStart, 2, now);
                    }
                }
                else if (scanPhase == 2)
                {
                    // 左端补扫：右端扫完后反方向扫过中间，继续覆盖到左端，避免只回到中间就结束。
                    pidLocal.feedBack = -scanLateralOffset;
                    orienOffset = scanYawOffset;
                    if (phaseElapsed >= fullCrossDuration)
                    {
                        switchScanPhase(scanPhase, scanPhaseStart, 3, now);
                    }
                }
                else // 未检测到目标零件
                {
                    // 到达左端后短暂停留识别；仍未识别就结束本次搜索，避免继续横向扫出取件台。
                    pidLocal.feedBack = 0;
                    orienOffset = 0;
                    if (phaseElapsed >= scanReturnDuration)
                    {
                        pickStep = PickStep::PICK_STEP_LOCAL;
                        ROS_ERROR_STREAM("----------- STEP: " + to_string(pickStep) + " -----------");
                        robotCtrl(0, 0, 0);
                        findPart = false;
                    }
                }
            }
            pidDis.ref = disSearch;   // AI搜索零件距离：m
            poseControl();            // 机器人姿态控制（距离+方向+位置）

            if (findPart) // 已检测到目标零件
            {
                orienOffset = 0;                                                                               // 姿态校正误差=0
                if (abs(pidLocal.feedBack - pidLocal.ref) <= pidLocal.deadline && abs(pidPose.feedBack) < 0.05) // 角度+位置调整完成
                {
                    Mat img;
                    counter = getSystTime(); // 更新计时器
                    robotCtrl(0, 0, 0);      // 机器人运动控制
                    // 搜索阶段确认目标后同样先等机械臂伸直，再切RGB相机进入近距离抓取。
                    if (!waitForTalonAction(talon.ACTION_EXT, armExtendTimeout))
                        ROS_ERROR_STREAM("机械臂伸展等待超时，继续尝试取件");
                    sleepMs(armPostExtendSettleMs);
                    setCamera(false);        // 切换RGB相机
                    if (!waitForRgbFrame(img, cameraSwitchTimeout)) // 检测到RGB出图就立刻继续
                        ROS_ERROR_STREAM("相机打开失败，正在重新尝试...");
                    talon.setClawInitial();                 // 初始化（标定）机械爪
                    pickStep = PickStep::PICK_STEP_FORWARD; // 切换智能取件任务流程
                    pidPose.outMax = 0.2;                   // 限制角速度：rad/s
                    pidDis.outMax = 0.06;                   // 限制输出速度：m/s
                    pidLocal.enable = false;                // 位置PID控制
                    count = 0;
                    // pidDis.ref = disPick;                   // 机械臂抓取零件距离：m
                    counter = getSystTime();                // 更新计时器
                    ROS_ERROR_STREAM("----------- STEP: " + to_string(pickStep) + " -----------");
                }
            }

            if (getSystTime() - counter > pickupSearchTimeout) // 搜索超时
            {
                pickStep = PickStep::PICK_STEP_LOCAL; // 任务结束
                ROS_ERROR_STREAM("----------- STEP: " + to_string(pickStep) + " -----------");
                orienOffset = 0;    // 姿态校正误差=0
                robotCtrl(0, 0, 0); // 机器人运动控制
                findPart = false;   // 搜索零件失败
            }
        }
        break;

        case PickStep::PICK_STEP_FORWARD: // 机器人向前移动
        {
            // 机械爪跟随零件控制
            Center center = searchPartRgb(part);
            if (validCenter(center)) // 检测到有效的目标零件
            {
                pidClawX.enable = true;
                count++;
            }
            else
            {
                pidClawX.enable = false;
                pidClawX.out = 0;
                count = 0;
            }

            pidClawY.enable = false;
            pidClawY.out = 0;
            clawControl(center); // 机械爪运动控制

            if (count >= detectionConfirmFrames || (getSystTime() - counter) > 3000)
            {
                count = 0;
                pickStep = PickStep::PICK_STEP_AIM; // 切换智能取件任务流程
                robotCtrl(0, 0, 0);                 // 机器人运动控制
                ROS_ERROR_STREAM("----------- STEP: " + to_string(pickStep) + " -----------");
                counter = getSystTime(); // 更新计时器
            }
        }
        break;

        case PickStep::PICK_STEP_AIM: // 机械爪瞄准零件
        {
            robotCtrl(0, 0, 0);  // 机器人运动控制
            bool targetVisible = clawfindParts(part); // 机械爪跟随零件控制

            if (targetVisible && abs(pidClawX.feedBack) < pidClawX.deadline) // 机械爪初次定位准确
                count++;
            else
                count = 0;

            if (count >= detectionConfirmFrames || (getSystTime() - counter) > 3000) // 爪子对准 or 超时
            {
                pickStep = PickStep::PICK_STEP_GRAB; // 切换智能取件任务流程
                count = 0;
                counter = getSystTime(); // 更新计时器
                pidDis.ref = disClaw;    // 机械爪夹取零件距离：m
                pidDis.outMax = 0.08;    // 限制输出速度：m/s , 机器人缓慢向前进
            }
            break;
        }

        case PickStep::PICK_STEP_GRAB: // 零件抓取
        {
            //[01] 机械爪瞄准零件控制
            clawfindParts(part); // 机械爪跟随零件控制

            poseControl();                                                                    // 机器人姿态控制（距离+方向）
            if (abs(pidDis.feedBack - pidDis.ref) < 0.05 || (getSystTime() - counter) > 8000) // 姿态调制至误差范围or超时
            {
                for (int i = 0; i < 3; i++) // 机器人停止运动
                {
                    robotCtrl(0, 0, 0);
                    usleep(30 * 1000);                                                // 等待：us
                }
                
                talon.setJointPosition(talon.ARM_JOINT_6, getClawPosition(part)); // 机械爪夹取

                sleepMs(clawGrabWaitMs); // 等待机械爪夹取结束

                for (int i = 0; i < 3; i++) // 机械爪运动控制: 往上抬0.3rad
                {
                    usleep(250 * 1000); // 等待：us
                    talon.setClawMotion(-pidClawX.out, 0.15);
                }

                pickStep = PickStep::PICK_STEP_LOCAL; // 切换智能取件任务流程
                ROS_ERROR_STREAM("----------- STEP: " + to_string(pickStep) + " -----------");
                counter = getSystTime();
                findPart = true;     // 搜索零件成功
                pidDis.outMax = 0.2; // 限制输出速度：m/s
            }
            break;
        }

        case PickStep::PICK_STEP_LOCAL: // 机器人重定位
        {
            talon.setClawMotion(-pidClawX.out, 0.15); // 机械爪运动控制: 往上抬0.3rad
            pidDis.ref = disSearch;                  // 机器人后退，导航适合距离
            poseControl();                           // 机器人姿态控制（距离+方向）

            if (abs(pidDis.feedBack - pidDis.ref) < 0.05 || (getSystTime() - counter) > 5500) // 姿态调制至误差范围or超时
            {
                for (int i = 0; i < 3; i++) // 机器人停止运动
                {
                    robotCtrl(0, 0, 0);
                    usleep(30 * 1000);                  // 等待：us
                    talon.setActions(talon.ACTION_CUR); // 机械臂动作：收缩
                }
                pickStep = PickStep::PICK_STEP_END; // 结束
                ROS_ERROR_STREAM("----------- STEP: " + to_string(pickStep) + " -----------");
            }
            break;
        }
        }

        if (pickStep == PickStep::PICK_STEP_END)
            return true;
        else
            return false;
    }

    /**
     * @brief 放置零件
     *
     * @param part 目标零件AI标签
     */
    bool putdownSomething(string part)
    {
        switch (pickStep)
        {
        case PickStep::PICK_STEP_START: // 任务开始
            counter = getSystTime();
            pickStep = PickStep::PICK_STEP_POSE; // 切换智能取件任务流程
            break;

        case PickStep::PICK_STEP_POSE: // 机器人姿态校正
        {
            pidLocal.enable = false;    // 关闭位置PID控制
            pidDis.ref = disPick + 0.6; // 机械臂抓取零件距离：m

            // 姿态校正状态检测
            if ((abs(pidDis.feedBack - pidDis.ref) < 0.1 &&
                 abs(pidPose.feedBack) < 0.1) ||
                (getSystTime() - counter) > 3000) // 姿态调制至误差范围or超时
            {
                pickStep = PickStep::PICK_STEP_GRAB; // 切换智能取件任务流程
                for (int i = 0; i < 5; i++)          // 机器人停止运动
                {
                    robotCtrl(0, 0, 0);
                    usleep(30 * 1000);                  // 等待：us
                    talon.setActions(talon.ACTION_PUT); // 机械臂动作：放置零件
                }
                counter = getSystTime();
                while (!talon.actionOver[talon.ACTION_PUT] && (getSystTime() - counter) < putActionTimeout) // 等待机械臂放置动作完成
                {
                    robotCtrl(0, 0, 0); // 机器人运动控制
                    usleep(50 * 1000);
                }
                sleepMs(armPostPutSettleMs);
                talon.setClawInitial(); // 初始化（标定）机械爪
                counter = getSystTime();
            }
            else
                poseControl(); // 机器人姿态控制（距离+方向+位置）
        }
        break;

        case PickStep::PICK_STEP_GRAB: // 零件放置
        {
            pidDis.outMax = 0.10;
            pidDis.ref = disClaw + 0.2; // 机械臂放置零件距离：m
            poseControl();        // 机器人姿态控制（距离+方向+位置）

            float offsetX = 0;  // 机械爪末端横向偏移
            float offsetY = -0.20;  // 机械爪末端纵向偏移
            if (placePart == 2)
                offsetX = -0.3;// 机械爪运动控制: 左侧放置零件
            else if (placePart == 3)
                offsetX = 0.3; // 机械爪运动控制: 右侧放置零件
            else
                offsetX = 0.0;

            talon.setClawMotion(offsetX, 0); // 机械爪运动控制

            // 姿态校正状态检测
            if ((abs(pidDis.feedBack - pidDis.ref) < 0.1 &&
                 abs(pidPose.feedBack) < 0.1) ||
                (getSystTime() - counter) > 3000) // 姿态调制至误差范围or超时
            {
                
                pickStep = PickStep::PICK_STEP_LOCAL; // 切换智能取件任务流程
                for (int i = 0; i < 5; i++)
                {
                    robotCtrl(0, 0, 0); // 机器人运动控制
                    usleep(100 * 1000);
                }
                talon.setClawMotion(offsetX, offsetY); // 机械爪运动控制
                sleepMs(placePreReleaseWaitMs);
                talon.setJointPosition(talon.ARM_JOINT_6, 9); // 机械爪放置零件
                sleepMs(placeReleaseWaitMs); // 等待零件放置完成
                // if (part != LABEL_AI_SCREW)
                talon.setClawMotion(offsetX, 0.2); // 机械爪运动控制
                pidDis.ref = disPick + 0.5; // 机器人后退，导航适合距离+机械臂收回
                counter = getSystTime();
            }
            break;
        }

        case PickStep::PICK_STEP_LOCAL: // 机器人重定位
        {
            if (abs(pidDis.feedBack - pidDis.ref) < 0.05 || (getSystTime() - counter) > 5000) // 姿态调制至误差范围or超时
            {
                for (int i = 0; i < 10; i++) // 机器人停止运动
                {
                    robotCtrl(0, 0, 0);
                    usleep(200 * 1000);                 // 等待：us
                    talon.setActions(talon.ACTION_CUR); // 机械臂动作：收缩
                }
                pickStep = PickStep::PICK_STEP_END; // 结束
                ROS_ERROR_STREAM("----------- STEP: " + to_string(pickStep) + " -----------");
            }
            else
            {
                poseControl(); // 机器人姿态控制（距离+方向）
            }
            break;
        }
        }

        if (pickStep == PickStep::PICK_STEP_END)
            return true;
        else
            return false;
    }
};
