/**
 * @file factory.cpp
 * @author Leo (sasu@saishukeji.com)
 * @brief 送件到桌场景（基于MultiNavi）
 * @version 0.1
 * @date 2024-03-05
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include "../include/tinyxml.hpp"
#include "../include/tools.hpp"
#include "confirm.cpp"
#include "picking.cpp"
#include "summary.cpp"
#include <signal.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; // moveBase导航通信类

/**
 * @brief 智能工厂数据类
 *
 */
class Factory
{
private:
#pragma region "数据结构声明"

    std::string puttPart = ""; // 用于记录目前抓取的零件
    /**
     * @brief  智能工厂场景流程（状态机标志）
     *
     */
    enum FactoryStep
    {
        FACTORY_STEP_START = 0, // 任务开始
        FACTORY_STEP_INIT,      // 机器人重定位
        FACTORY_STEP_DELIVERY,  // 送件到桌
        FACTORY_STEP_SUMMARY,   // 智能结算
        FACTORY_STEP_END        // 任务结束
    };

    /**
     * @brief 导航目标点类型
     *
     */
    enum StationNavi
    {
        STATION_TABLE = 0, // 工作台
        STATION_SERVING,   // 取件台
        STATION_COUNTER    // 结算区
    };

    /**
     * @brief 工作台场景数据类
     *
     */
    class Table
    {
    public:
        move_base_msgs::MoveBaseGoal position; // 工作台位置信息
        vector<string> orders;                 // 订单信息
        bool ordered = false;                  // 是否启动过[需求确认]任务标志
        bool putting = false;                  // 放置零件
        int partPlace = 1;                     // 工作台累计放置零件数量
    };

    /**
     * @brief 工厂场景定位信息类
     *
     */
    class Location
    {
    public:
        vector<Table> tables;                 // 工作台位置信息（1~6个）
        move_base_msgs::MoveBaseGoal serving; // 取件台位置信息
        move_base_msgs::MoveBaseGoal origin;  // 起始点位置信息
        move_base_msgs::MoveBaseGoal counter; // 结算区位置信息
    };

    /**
     * @brief 导航相关参数
     *
     */
    class Navigation
    {
    public:
        bool arriving = false;                             // 等待导航到达目标点标志
        bool overtime = false;                             // 导航超时查验标志
        int timeout = 10000;                               // 导航超时时间：ms
        int maxRetryCount = 1;                             // 单次导航目标最多重发次数
        int retryCount = 0;                                // 当前导航目标已重发次数
        int serverWaitTimeout = 5;                         // 等待move_base服务上线时间：s
        int goalSettleMs = 150;                            // 发送目标后短暂等待状态刷新：ms
        geometry_msgs::PoseWithCovarianceStamped amclPose; // AMCL重定位坐标
        StationNavi station = StationNavi::STATION_TABLE;  // 导航目标点类型
    };

    ros::NodeHandle rosNode;                           // ROS节点类
    ros::Publisher pubAudio;                           // 语音播报话题发布
    ros::Publisher pubInitPose;                        // 发布机器人重定位话题
    ros::Subscriber subAmclPose;                       // 接收AMCL话题
    MoveBaseClient actionMoveBase;                     // Movebase导航目标通信
    bool enable = true;                                // 场景初始化标志(资源加载成功)
    long timeBegin;                                    // 任务开始计时
    long counter;                                      // 任务计数器
    Location location;                                 // 工厂场景定位信息类
    FactoryStep caterStep = FactoryStep::FACTORY_STEP_START; // 智能工厂场景流程（状态机标志）
    int tableId = 1;                                   // 工作台ID编号
    vector<Order> ordersSummary;                        // 智能结算订单数据
    bool safeWork = true;                               // 工作台导航标志
    // ros::Publisher map_pub_;                           // 代价地图发布
    // ros::ServiceClient clear_costmap_service_;         // 代价地图清除

#pragma endregion

    /**
     * @brief 获取AMCL定位信息（重定位/导航成功后会接收此话题）
     *
     * @param pose
     */
    void getAmclPose(const geometry_msgs::PoseWithCovarianceStamped &pose)
    {
        navi.amclPose = pose;
    }

    /**
     * @brief 验证AMCL定位坐标与导航目标是否一致
     *
     * @param goal
     * @return true
     * @return false
     */
    bool checkGoal(move_base_msgs::MoveBaseGoal goal)
    {
        // 误差可合理设置更大一些，因为降低AMCL的重复定位频率（CPU消耗），AMCL位置更新可能更慢
        if (abs(goal.target_pose.pose.position.x - navi.amclPose.pose.pose.position.x) < 0.5 &&
            abs(goal.target_pose.pose.position.y - navi.amclPose.pose.pose.position.y) < 0.5 &&
            abs(abs(goal.target_pose.pose.orientation.w) - abs(navi.amclPose.pose.pose.orientation.w)) < 0.5) // 校验AMCL定位信息与导航目标点一致
        {
            return true; // 重定位成功
        }
        else
            return false;
    }

    /**
     * @brief 字符串解析（double）
     *
     * @param str
     * @return std::vector<double>
     */
    vector<double> stringTodouble(std::string str)
    {
        std::string token;
        std::istringstream iss(str);
        std::vector<double> doubles;

        while (std::getline(iss, token, ' ')) // 分离空格符
        {
            try
            {
                doubles.push_back(std::stod(token)); // str转double
            }
            catch (const std::invalid_argument &e)
            {
                ROS_ERROR_STREAM("Invalid number format: " + token); // 字符格式错误
            }
            catch (const std::out_of_range &e)
            {
                ROS_ERROR_STREAM("Number out of range: " + token); // 数值超出double范围
            }
        }
        return doubles;
    }

    /**
     * @brief 解析MoveBaseGoal数据类型
     *
     * @param position 坐标
     * @param orientation 方向
     * @return move_base_msgs::MoveBaseGoal
     */
    move_base_msgs::MoveBaseGoal getMoveBaseGoal(std::string position, std::string orientation)
    {
        move_base_msgs::MoveBaseGoal goal;

        std::vector<double> positions = stringTodouble(position);       // 字符串解析（double）
        std::vector<double> orientations = stringTodouble(orientation); // 字符串解析（double）

        if (positions.size() >= 3 && orientations.size() >= 4) // 数据准确性校验
        {
            goal.target_pose.pose.position.x = positions[0];
            goal.target_pose.pose.position.y = positions[1];
            goal.target_pose.pose.position.z = positions[2];
            goal.target_pose.pose.orientation.x = orientations[0];
            goal.target_pose.pose.orientation.y = orientations[1];
            goal.target_pose.pose.orientation.z = orientations[2];
            goal.target_pose.pose.orientation.w = orientations[3];
            goal.target_pose.header.frame_id = "map";
        }
        else
            ROS_ERROR_STREAM("MoveBaseGoal conversion failed!!");

        return goal;
    }

    /**
     * @brief 位置信息加载（读取xml文件）
     *
     * @param xml
     */
    Location locationFromXml()
    {
        string pathXml = ros::package::getPath("sebot_factory") + "/res/location.xml";

        Location location; // 工厂场景定位信息类
        if (pathXml.empty())
        {
            ROS_ERROR_STREAM("Location'xml file is empty!!!");
            enable = false; // 场景初始化标志
            return location;
        }

        TiXmlDocument *xmlFile = new TiXmlDocument();                    // 实例化xml文档类
        if (!xmlFile->LoadFile(pathXml.c_str(), TIXML_ENCODING_UNKNOWN)) // 加载xml文件
        {
            ROS_ERROR_STREAM("Can't load xml file!");
            enable = false; // 场景初始化标志
            return location;
        }

        TiXmlElement *root = xmlFile->RootElement(); // 读取xml根节点
        if (!root)
        {
            ROS_ERROR_STREAM("Failed to get root element!");
            enable = false; // 场景初始化标志
            return location;
        }

        vector<Table> tables;
        for (TiXmlElement *child = root->FirstChildElement(); child; child = child->NextSiblingElement()) // 遍历坐标数据
        {
            const char *nameEle = child->Value();
            move_base_msgs::MoveBaseGoal goal;

            std::string position = child->FirstChildElement("local")->Attribute("position");       // 读取position属性
            std::string orientation = child->FirstChildElement("local")->Attribute("orientation"); // 读取orientation属性
            if (!position.empty() && !orientation.empty())
            {
                if (!std::strcmp(nameEle, "table")) // 工作台定位数据
                {
                    move_base_msgs::MoveBaseGoal goal = getMoveBaseGoal(position, orientation); // 解析MoveBaseGoal数据类型
                    for (int i = 1; i < 7; i++)                                                 // 解析坐标点序号
                    {
                        if (child->Attribute("name") == ("工作台-" + to_string(i)))
                        {
                            goal.target_pose.header.seq = i;
                            break;
                        }
                    }

                    Table table;
                    table.position = goal;
                    tables.push_back(table);
                }

                else if (!std::strcmp(nameEle, "serving"))                     // 取件台定位数据
                    location.serving = getMoveBaseGoal(position, orientation); // 解析MoveBaseGoal数据类型
                else if (!std::strcmp(nameEle, "origin"))                      // 起始点定位数据
                    location.origin = getMoveBaseGoal(position, orientation);  // 解析MoveBaseGoal数据类型
                else if (!std::strcmp(nameEle, "counter"))                     // 结算区定位数据
                    location.counter = getMoveBaseGoal(position, orientation); // 解析MoveBaseGoal数据类型
            }
        }
        for (int i = 0; i < tables.size(); i++)
        {
            for (int j = 0; j < tables.size(); j++)
            {
                if (tables[j].position.target_pose.header.seq == i + 1)
                {
                    location.tables.push_back(tables[i]);
                    break;
                }
            }
        }

        return location;
    }

    /**
     * @brief 机器人重定位（初始化位置校正）
     *
     */
    void initialPose(move_base_msgs::MoveBaseGoal goal)
    {
        if (!enable)
            return;
        geometry_msgs::PoseWithCovarianceStamped msgPose; // 重定位话题msg
        msgPose.header.stamp = ros::Time::now();          // 当前时间戳
        msgPose.header.frame_id = "map";                  // 参考坐标系，通常是地图坐标系
        msgPose.pose.pose.position = goal.target_pose.pose.position;
        msgPose.pose.pose.orientation = goal.target_pose.pose.orientation;
        msgPose.pose.covariance[0] = 0.25; // 协方差矩阵
        msgPose.pose.covariance[6 * 1 + 1] = 0.25;
        msgPose.pose.covariance[6 * 5 + 5] = 0.06853891945200942;

        pubInitPose.publish(msgPose); // 发布机器人重定位话题
    }

    void sleepMs(int durationMs)
    {
        if (durationMs > 0)
            ros::Duration(durationMs / 1000.0).sleep();
    }

    /**
     * @brief 设置智能取件相关配置
     *
     * @param picking
     */
    void setPicking(Picking &picking)
    {
        picking.pidPose.kP = pidConfig.pidPoseKp;
        picking.pidPose.kI = pidConfig.pidPoseKi;
        picking.pidPose.kD = pidConfig.pidPoseKd;
        picking.pidDis.kP = pidConfig.pidDisKp;
        picking.pidDis.kI = pidConfig.pidDisKi;
        picking.pidDis.kD = pidConfig.pidDisKd;
        picking.disClaw = disClaw;
        picking.disPick = disPick;
        picking.disSearch = disSearch;
        picking.pidLocal.kP = pidConfig.pidLocalKp;
        picking.pidLocal.kI = pidConfig.pidLocalKi;
        picking.pidLocal.kD = pidConfig.pidLocalKd;
        picking.pidClawX.kP = pidConfig.pidClawXKp;
        picking.pidClawX.kI = pidConfig.pidClawXKi;
        picking.pidClawX.kD = pidConfig.pidClawXKd;
        picking.pidClawY.kP = pidConfig.pidClawYKp;
        picking.pidClawY.kI = pidConfig.pidClawYKi;
        picking.pidClawY.kD = pidConfig.pidClawYKd;
        picking.debug = debug;
        picking.score = score;
        picking.scanCenterDuration = scanCenterDuration;
        picking.scanLeftDuration = scanLeftDuration;
        picking.scanRightDuration = scanRightDuration;
        picking.scanReturnDuration = scanReturnDuration;
        picking.scanYawOffset = scanYawOffset;
        picking.scanLateralOffset = scanLateralOffset;
        picking.detectionConfirmFrames = detectionConfirmFrames;
        picking.detectionMissResetFrames = detectionMissResetFrames;
        picking.pickupSearchTimeout = pickupSearchTimeout;
        picking.partDetectMaxWidth = partDetectMaxWidth;
        picking.partDetectMaxHeight = partDetectMaxHeight;
        picking.partDetectMinY = partDetectMinY;
        picking.cameraWarmupMs = cameraWarmupMs;
        picking.cameraSwitchSettleMs = cameraSwitchSettleMs;
        picking.cameraSwitchTimeout = cameraSwitchTimeout;
        picking.armExtendTimeout = armExtendTimeout;
        picking.armPostExtendSettleMs = armPostExtendSettleMs;
        picking.clawGrabWaitMs = clawGrabWaitMs;
        picking.putActionTimeout = putActionTimeout;
        picking.armPostPutSettleMs = armPostPutSettleMs;
        picking.placePreReleaseWaitMs = placePreReleaseWaitMs;
        picking.placeReleaseWaitMs = placeReleaseWaitMs;
        picking.placeApproachOutMax = placeApproachOutMax;
        picking.placeDropOffsetY = placeDropOffsetY;
    }

    /**
     * @brief 设置智能取件相关配置
     *
     * @param picking
     */
    void sysExit()
    {
        Talon talon; // 机械臂控制类
        sleep(2);
        talon.setActions(talon.ACTION_RES);
        sleep(2);
        talon.setJointEnable(talon.ArmJoint::ARM_JOINT_ALL, false); // 机械臂失能
        sleep(1);
        talon.setJointEnable(talon.ArmJoint::ARM_JOINT_ALL, false); // 机械臂失能
    }

public:
    Navigation navi;         // 导航相关参数
    bool simulation = false; // STDR仿真模式

    /**
     * @brief 智能取件相关配置
     *
     */
    class PidConfig
    {
    public:
        float pidPoseKp = 12.0;    // 方向PID：比例系数
        float pidPoseKi = 3.3;     // 方向PID：积分系数
        float pidPoseKd = 10.0;    // 方向PID：微分系数
        float pidDisKp = 5.0;      // 距离PID：比例系数
        float pidDisKi = 0.3;      // 距离PID：积分系数
        float pidDisKd = 0.0;      // 距离PID：微分系数
        float pidLocalKp = 0.001;  // 位置PID：比例系数
        float pidLocalKi = 0.0002; // 位置PID：积分系数
        float pidLocalKd = 0.0036; // 位置PID：微分系数

        float pidClawXKp = 0.001;  // 机械爪PID：比例系数
        float pidClawXKi = 0.0002; // 机械爪PID：积分系数
        float pidClawXKd = 0.0036; // 机械爪PID：微分系数
        float pidClawYKp = 0.001;  // 机械爪PID：比例系数
        float pidClawYKi = 0.0002; // 机械爪PID：积分系数
        float pidClawYKd = 0.0036; // 机械爪PID：微分系数
    };
    PidConfig pidConfig;   // 自主取件 + 智能取件PID参数配置
    float disSearch = 0.6; // AI搜索零件距离：m
    float disPick = 0.4;   // 机械臂抓取零件距离：m
    float disClaw = 0.3;   // 机械爪夹取零件距离：m
    float disOrder = 0.75; // 机器人距离阈值：m（需要+激光雷达盲区20cm）
    bool debug = false;    // 调试使能：AI绘制+显示
    float score = 0.4;     // AI置信度

    // 总流程节拍参数：把原来散落的固定等待集中配置，默认值偏保守。
    int initialPoseRetryInterval = 1000; // 初始化阶段重发AMCL初始位姿间隔：ms
    int audioNavigationDelayMs = 0;      // 导航前等待语音播报时间；默认不阻塞导航
    int interStepDelayMs = 100;          // 状态切换短暂停顿：ms
    int postPickRelocalizeDelayMs = 300; // 取件后发布取件台初始位姿前等待：ms

    // 传给Picking的取件台扫描和动作等待参数。
    int scanCenterDuration = 800;
    int scanLeftDuration = 2200;
    int scanRightDuration = 2600;
    int scanReturnDuration = 800;
    float scanYawOffset = 0.02;
    int scanLateralOffset = 60;
    int detectionConfirmFrames = 5;
    int detectionMissResetFrames = 8;
    int pickupSearchTimeout = 13500;
    int partDetectMaxWidth = 100;
    int partDetectMaxHeight = 140;
    int partDetectMinY = ROWSIMAGE / 2 - 20;
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
    float placeApproachOutMax = 0.05;
    float placeDropOffsetY = -0.22;

    Factory() : actionMoveBase("move_base", true)
    {
        pubInitPose = rosNode.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 2); // 发布AMCL重定位话题
        subAmclPose = rosNode.subscribe("/amcl_pose", 2, &Factory::getAmclPose, this);                  // 订阅AMCL定位话题
        pubAudio = rosNode.advertise<std_msgs::String>("/audio", 2);                                  // 语音播报话题发布
        location = locationFromXml();                                                                 // 位置信息加载（读取xml文件）
        timeBegin = getSystTime();                                                                    // 任务开始计时
    };

    // 定义析构函数
    ~Factory() {};
    /**
     * @brief 工作台任务
     *
     */
    void taskTables()
    {
        if (location.tables.size() > 0) // 工作台任务还未执行完毕
        {
            if (!location.tables[0].ordered) // 未确认需求
            {
                publishAudio(pubAudio, "confirm"); // 语音播报：启动需求确认
                if (simulation)                     // 仿真模式
                    sleep(5);
                else
                {
                    Confirm confirm; // 实例化需求确认类
                    // setConfirm(confirm); // 设置需求确认相关配置
                    confirm.pidPose.kP = pidConfig.pidPoseKp; // 方向控制PID
                    confirm.pidPose.kI = pidConfig.pidPoseKi;
                    confirm.pidPose.kD = pidConfig.pidPoseKd;
                    confirm.pidDis.kP = pidConfig.pidDisKp; // 距离控制PID
                    confirm.pidDis.kI = pidConfig.pidDisKi;
                    confirm.pidDis.kD = pidConfig.pidDisKd;
                    confirm.pidDis.ref = disOrder;              // 机器人距离阈值：m（需要+激光雷达盲区20cm）
                    confirm.pidLocal.kP = pidConfig.pidLocalKp; // 距离控制PID
                    confirm.pidLocal.kI = pidConfig.pidLocalKi;
                    confirm.pidLocal.kD = pidConfig.pidLocalKd;
                    confirm.debug = debug;
                    confirm.score = score;

                    bool run = false;
                    while (!run)
                    {
                        run = confirm.getConfirm();
                        ros::spinOnce(); // 回调话题通信
                    }
                    sleepMs(interStepDelayMs);
                    location.tables[0].orders = confirm.orders; // 获取需求信息

                    // 创建智能结算订单
                    if (confirm.orders.size() > 0)
                    {
                        Order order;
                        order.tableId = tableId;
                        order.part = confirm.orders;
                        ordersSummary.push_back(order);
                    }
                }

                location.tables[0].ordered = true;
                switch (location.tables[0].orders.size())
                {
                case 0:                                  // 没有订单
                    publishAudio(pubAudio, "orderNone"); // 语音播报：没有订单
                    break;
                case 1:                                 // 1个零件
                    publishAudio(pubAudio, "orderOne"); // 语音播报：1个零件
                    break;
                case 2:                                // 2个零件
                    publishAudio(pubAudio, "orderTwo"); // 语音播报：2个零件
                    break;
                case 3:                                   // 3个零件
                    publishAudio(pubAudio, "orderThree"); // 语音播报：3个零件
                    break;
                }
            }
            else if (location.tables[0].putting) // 送件（放置零件）
            {
                Picking picking;
                setPicking(picking);
                picking.placePart = location.tables[0].partPlace; // 零件放置在工作台上的位置设置
                publishAudio(pubAudio, "putdown");                // 语音播报：放置零件
                if (simulation)                                   // 仿真模式
                    sleep(5);
                else
                {
                    while (!picking.putdownSomething(puttPart)) // 启动[智能取件]，等待放置零件
                        ros::spinOnce();                        // 回调话题通信
                }
                publishAudio(pubAudio, "putOk"); // 语音播报：放置零件
                location.tables[0].putting = false;
                location.tables[0].partPlace++;
            }
            else if (location.tables[0].orders.size() > 0) // 继续取件
            {
                publishAudio(pubAudio, "gotoServing");           // 语音播报：前往取件台
                sleepMs(audioNavigationDelayMs);                  // 语音不再默认阻塞导航，必要时现场调大
                naviToWorkStation(StationNavi::STATION_SERVING); // 自主导航至取件台
            }
            else if (location.tables.size() <= 1) // 送件结束：导航至结算区
            {
                location.tables.clear();                         // 删除工作台任务
                sleepMs(audioNavigationDelayMs);                  // 保留可调等待，默认不浪费比赛时间
                naviToWorkStation(StationNavi::STATION_COUNTER); // 导航至结算区
                publishAudio(pubAudio, "deliveryOver");          // 语音播报：前往结算区
            }
            else // 下一个工作台
            {
                location.tables.erase(location.tables.begin()); // 更新工作台信息（删除已完成任务工作台）
                naviToWorkStation(StationNavi::STATION_TABLE);  // 自主导航至下一个工作台
                tableId += 1;                                   // 工作台编号+1
            }
        }
    }

    /**
     * @brief 取件台任务
     *
     */
    void taskServing()
    {
        if (location.tables[0].orders.size() <= 0) // 订单异常
            return;
        Picking picking;
        setPicking(picking);
        if (location.tables[0].orders[0] == LABEL_AI_NUT)   // 抓取螺钉
            publishAudio(pubAudio, "pickNut");              // 语音播报：取件
        else if (location.tables[0].orders[0] == LABEL_AI_SCREW) // 抓取螺母
            publishAudio(pubAudio, "pickScrew");                 // 语音播报：取件
        else if (location.tables[0].orders[0] == LABEL_AI_PCB) // 抓取电路板
            publishAudio(pubAudio, "pickPcb");                 // 语音播报：取件
        else if (location.tables[0].orders[0] == LABEL_AI_BLOCK)   // 抓取端子排
            publishAudio(pubAudio, "pickBlock");                   // 语音播报：取件
        else if (location.tables[0].orders[0] == LABEL_AI_TAPE)  // 抓取绝缘胶带
            publishAudio(pubAudio, "pickTape");                  // 语音播报：取件

        puttPart = location.tables[0].orders[0]; // 获取当前抓取的零件
        if (simulation)                          // 仿真模式
        {
            picking.findPart = true;
            sleep(5);
        }
        else
        {
            while (!picking.pickupSomething(location.tables[0].orders[0])) // 启动[智能取件]，抓取零件，等待抓取结束
            {
                ros::spinOnce(); // 回调话题通信
            }

            sleepMs(postPickRelocalizeDelayMs);
            initialPose(location.serving); // 取件后立即恢复取件台位姿，避免固定等待3s
            ros::spinOnce();              // 回调话题通信
        }
        if (picking.findPart) // 零件挑选成功
        {
            publishAudio(pubAudio, "pickOk");  // 语音播报：取件成功!
            location.tables[0].putting = true; // 等待放置零件
            naviToWorkStation(StationNavi::STATION_TABLE);                      // 自主导航：返回工作台
        }
        else
        {
            publishAudio(pubAudio, "pickFailed"); // 语音播报：取件失败!
            location.tables[0].putting = false;
            if(safeWork)
                naviToWorkStation(StationNavi::STATION_TABLE);               
            else
                naviToWorkStation(StationNavi::STATION_SERVING);
        }
        // naviToWorkStation(StationNavi::STATION_TABLE);                       // 自主导航：返回工作台
        location.tables[0].orders.erase(location.tables[0].orders.begin()); // 更新订单信息（删除已完成订单）
    }

    /**
     * @brief 结算区任务
     *
     */
    void taskCounter()
    {
    }

    /**
     * @brief 自主导航至任务点
     *
     */
    void naviToWorkStation(StationNavi stationNext, bool resetRetry = true)
    {
        if (resetRetry)
            navi.retryCount = 0;

        if (!actionMoveBase.waitForServer(ros::Duration(navi.serverWaitTimeout))) // 等待move_base服务器
        {
            ROS_ERROR_STREAM("Can't connected to movebase server!!!");
            publishAudio(pubAudio, "errMovebase"); // 语音播报：movebase服务器连接失败!
            return;
        }

        switch (stationNext)
        {
        case StationNavi::STATION_TABLE: // 工作台
            if (location.tables.size() > 0)
            {
                location.tables[0].position.target_pose.header.stamp = ros::Time::now(); // 获取时间戳
                actionMoveBase.sendGoal(location.tables[0].position);                    // 导航至下一个工作台位置
                navi.station = StationNavi::STATION_TABLE;                               // 导航目标点类型
            }
            break;

        case StationNavi::STATION_SERVING: // 取件台

            location.serving.target_pose.header.stamp = ros::Time::now(); // 获取时间戳
            actionMoveBase.sendGoal(location.serving);                    // 导航至结算区
            navi.station = StationNavi::STATION_SERVING;                  // 导航目标点类型
            break;

        case StationNavi::STATION_COUNTER: // 结算区

            location.counter.target_pose.header.stamp = ros::Time::now(); // 获取时间戳
            actionMoveBase.sendGoal(location.counter);                    // 导航至结算区
            navi.station = StationNavi::STATION_COUNTER;                  // 导航目标点类型
            break;
        }
        navi.arriving = true;  // 等到导航到达目标点标志
        navi.overtime = false; // 导航超时查验标志
        counter = getSystTime();
        sleepMs(navi.goalSettleMs); // 只短暂等待Action状态刷新，避免每次导航固定损失1s
    }

    void retryNavigation(StationNavi stationNext)
    {
        if (navi.retryCount < navi.maxRetryCount)
        {
            navi.retryCount++;
            publishAudio(pubAudio, "naviTimeout"); // 语音播报：导航超时
            naviToWorkStation(stationNext, false); // 超时后只重发有限次数，避免无限重复导航耗尽比赛时间
        }
        else
        {
            ROS_WARN_STREAM("Navigation retry limit reached, keep waiting current goal.");
            counter = getSystTime();
        }
    }

    /**
     * @brief 智能工厂场景-状态机处理
     *
     */
    void caterHandle()
    {
        switch (caterStep)
        {
        case FactoryStep::FACTORY_STEP_START:   // 任务开始
            publishAudio(pubAudio, "none"); // 语音播报：
            sleepMs(interStepDelayMs);
            publishAudio(pubAudio, "caterStart"); // 语音播报：智能工厂任务开始
            caterStep = FactoryStep::FACTORY_STEP_INIT;
            counter = getSystTime() - initialPoseRetryInterval; // 进入INIT后立即发布一次初始位姿
            ROS_INFO_STREAM("factory is starting!!!");
            break;

        case FactoryStep::FACTORY_STEP_INIT:                    // 机器人重定位
            if (!checkGoal(location.origin) && !simulation) // 等待机器人重定位
            {
                if ((getSystTime() - counter) > initialPoseRetryInterval) // 按配置间隔发布重定位话题
                {
                    initialPose(location.origin);
                    counter = getSystTime(); // 更新系统时间戳
                    ROS_INFO_STREAM("Relocation is running...");
                }
                sleepMs(100); // 短周期等待AMCL反馈，避免每轮固定阻塞5s
            }
            else // AMCL重定位成功!!
            {
                publishAudio(pubAudio, "initPose"); // 语音播报：机器人重定位成功!
                caterStep = FactoryStep::FACTORY_STEP_DELIVERY;
                ROS_INFO_STREAM("Relocation success!!!");

                if (location.tables.size() > 0) // 工作台任务
                {
                    naviToWorkStation(StationNavi::STATION_TABLE); // 自主导航至下一个工作台
                }
            }
            break;

        case FactoryStep::FACTORY_STEP_DELIVERY: // 送件到桌
            if (navi.arriving)
            {
                bool finished = actionMoveBase.waitForResult(ros::Duration(navi.timeout / 1000.0)); // 带超时等待，避免导航异常时无限阻塞
                bool moveBaseSucceeded = finished && actionMoveBase.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
                if (moveBaseSucceeded) // 成功到达目标点!
                    ROS_INFO_STREAM("Reached the goal!");
                else if (!finished)
                    ROS_WARN_STREAM("Navigation wait timeout, checking AMCL pose before retry.");
                else // 导航失败!!!(可能已发生碰撞)
                {
                    ROS_ERROR_STREAM("Failed to navigation!!!");
                    // publishAudio(pubAudio, "naviTimeout");               // 语音播报：导航超时
                    // if (navi.station == StationNavi::STATION_TABLE)
                    //     naviToWorkStation(StationNavi::STATION_TABLE);   // 重新发布导航目标点
                    // else if (navi.station == StationNavi::STATION_SERVING)
                    //     naviToWorkStation(StationNavi::STATION_SERVING); // 重新发布导航目标点
                    // else if (navi.station == StationNavi::STATION_COUNTER)
                    //     naviToWorkStation(StationNavi::STATION_COUNTER); // 重新发布导航目标点
                }

                if (!navi.overtime) // 导航超时查验标志
                {
                    navi.overtime = true;
                    if (moveBaseSucceeded)
                        counter = getSystTime(); // move_base已成功时给AMCL一次刷新窗口，避免误重发
                }

                switch (navi.station)
                {
                case StationNavi::STATION_TABLE:                       // 工作台
                    if (checkGoal(location.tables[0].position))        // 等待到达导航点
                    {
                        navi.arriving = false;                         // 等待到达目标点标志
                        navi.retryCount = 0;
                    }
                    else if ((getSystTime() - counter) > navi.timeout) // 导航超时:10s
                    {
                        retryNavigation(StationNavi::STATION_TABLE); // 重新发布导航目标点
                    }
                    break;

                case StationNavi::STATION_SERVING:                     // 取件台
                    if (checkGoal(location.serving))                   // 等待到达导航点
                    {
                        navi.arriving = false;                         // 等待到达目标点标志
                        navi.retryCount = 0;
                    }
                    else if ((getSystTime() - counter) > navi.timeout) // 导航超时:10s
                    {
                        retryNavigation(StationNavi::STATION_SERVING); // 重新发布导航目标点
                    }
                    break;

                case StationNavi::STATION_COUNTER:   // 结算区
                    if (checkGoal(location.counter)) // 等待到达导航点
                    {
                        navi.arriving = false;                 // 等待到达目标点标志
                        navi.retryCount = 0;
                        publishAudio(pubAudio, "summary");      // 语音播报：启动智能结算
                        caterStep = FactoryStep::FACTORY_STEP_SUMMARY; // 启动智能结算

                        long time = getSystTime() - timeBegin;
                        ROS_INFO_STREAM("Total time for factory: " + to_string(long(time / 1000)) + "s");
                    }
                    else if ((getSystTime() - counter) > navi.timeout) // 导航超时:10s
                    {
                        retryNavigation(StationNavi::STATION_COUNTER); // 重新发布导航目标点
                    }
                    break;
                }
            }
            else // Navi到达以下工作站，执行下列任务↓
            {
                switch (navi.station)
                {
                case StationNavi::STATION_TABLE: // 工作台
                    taskTables();                // 执行工作台任务
                    break;

                case StationNavi::STATION_SERVING: // 取件台
                    taskServing();                 // 执行取件任务
                    break;

                case StationNavi::STATION_COUNTER: // 结算区
                    taskCounter();                 // 执行收银任务
                    break;
                }
            }
            break;
        case FactoryStep::FACTORY_STEP_SUMMARY: // 智能结算
        {
            sysExit();     // 机械臂关闭
            Summary summary; // 实例化需求确认类
            bool run = false;
            while (1)
            {
                if (!run)
                    run = summary.showOrder(ordersSummary);
                ros::spinOnce(); // 回调话题通信
            }
            break;
        }
        }
    }
};

/**
 * @brief 系统信号回调函数：系统退出
 *
 * @param signum 信号量
 */
void exitSignal(int signum)
{
    Talon talon; // 机械臂控制类
    sleep(2);
    talon.setActions(talon.ACTION_RES);
    sleep(2);
    talon.setJointEnable(talon.ArmJoint::ARM_JOINT_ALL, false); // 机械臂失能
    sleep(1);
    talon.setJointEnable(talon.ArmJoint::ARM_JOINT_ALL, false); // 机械臂失能
    ROS_INFO_STREAM("------ sebot_factory closeed!!! ------");
    exit(signum);
}

int main(int argc, char **argv)
{
    int delay = 5;                              // 程序延时启动时间：s
    ros::init(argc, argv, "sebot_factory");     // ROS节点初始化
    Factory factory;                            // 智能工厂数据类
    signal(SIGINT, exitSignal);                 // 程序退出信号

    ros::param::get("/sebot_factory/delayStart", delay);                    // 程序延时启动时间：s
    ros::param::get("/sebot_factory/timeoutNavi", factory.navi.timeout);    // 导航超时时间：ms
    ros::param::get("/sebot_factory/navigation_timeout", factory.navi.timeout); // 新参数名：导航超时时间：ms
    ros::param::get("/sebot_factory/max_retry_count", factory.navi.maxRetryCount); // 单个导航目标最多重发次数
    ros::param::get("/sebot_factory/navigation_server_timeout", factory.navi.serverWaitTimeout); // 等待move_base服务时间：s
    ros::param::get("/sebot_factory/navigation_settle_ms", factory.navi.goalSettleMs); // 发送目标后的短等待：ms
    ros::param::get("/sebot_factory/simulation", factory.simulation);       // STDR仿真模式使能

    // 需求确认+智能取件相关配置参数
    ros::param::get("/sebot_factory/pidPoseKp", factory.pidConfig.pidPoseKp);   // 方向控制PID
    ros::param::get("/sebot_factory/pidPoseKi", factory.pidConfig.pidPoseKi);
    ros::param::get("/sebot_factory/pidPoseKd", factory.pidConfig.pidPoseKd);
    ros::param::get("/sebot_factory/pidDisKp", factory.pidConfig.pidDisKp);     // 距离控制PID
    ros::param::get("/sebot_factory/pidDisKi", factory.pidConfig.pidDisKi);
    ros::param::get("/sebot_factory/pidDisKd", factory.pidConfig.pidDisKd);
    ros::param::get("/sebot_factory/disClaw", factory.disClaw);                 // 机械爪夹取零件距离：m（需要+激光雷达盲区20cm）
    ros::param::get("/sebot_factory/disPick", factory.disPick);                 // 机械臂抓取零件距离：m（需要+激光雷达盲区20cm）
    ros::param::get("/sebot_factory/disSearch", factory.disSearch);             // AI搜索零件距离：m（需要+激光雷达盲区20cm）
    ros::param::get("/sebot_factory/pidLocalKp", factory.pidConfig.pidLocalKp); // 位置控制PID
    ros::param::get("/sebot_factory/pidLocalKi", factory.pidConfig.pidLocalKi); 
    ros::param::get("/sebot_factory/pidLocalKd", factory.pidConfig.pidLocalKd); 
    ros::param::get("/sebot_factory/pidClawXKp", factory.pidConfig.pidClawXKp); // 机械爪PID
    ros::param::get("/sebot_factory/pidClawXKi", factory.pidConfig.pidClawXKi); 
    ros::param::get("/sebot_factory/pidClawXKd", factory.pidConfig.pidClawXKd); 
    ros::param::get("/sebot_factory/pidClawYKp", factory.pidConfig.pidClawYKp); // 机械爪PID
    ros::param::get("/sebot_factory/pidClawYKi", factory.pidConfig.pidClawYKi); 
    ros::param::get("/sebot_factory/pidClawYKd", factory.pidConfig.pidClawYKd); 
    ros::param::get("/sebot_factory/disOrder",  factory.disOrder);              // [需求确认] 机器人距离阈值：m（需要+激光雷达盲区20cm）
    ros::param::get("/sebot_factory/debug",     factory.debug);                 // 调试使能：AI绘制+显示
    ros::param::get("/sebot_factory/score",     factory.score);                 // AI检测置信度
    ros::param::get("/sebot_factory/initial_pose_retry_interval", factory.initialPoseRetryInterval);
    ros::param::get("/sebot_factory/audio_navigation_delay_ms", factory.audioNavigationDelayMs);
    ros::param::get("/sebot_factory/inter_step_delay_ms", factory.interStepDelayMs);
    ros::param::get("/sebot_factory/post_pick_relocalize_delay_ms", factory.postPickRelocalizeDelayMs);

    ros::param::get("/sebot_factory/scan_center_duration", factory.scanCenterDuration);
    ros::param::get("/sebot_factory/scan_left_duration", factory.scanLeftDuration);
    ros::param::get("/sebot_factory/scan_right_duration", factory.scanRightDuration);
    ros::param::get("/sebot_factory/scan_return_duration", factory.scanReturnDuration);
    ros::param::get("/sebot_factory/scan_yaw_offset", factory.scanYawOffset);
    ros::param::get("/sebot_factory/scan_lateral_offset", factory.scanLateralOffset);
    ros::param::get("/sebot_factory/detection_confirm_frames", factory.detectionConfirmFrames);
    ros::param::get("/sebot_factory/detection_miss_reset_frames", factory.detectionMissResetFrames);
    ros::param::get("/sebot_factory/pickup_search_timeout", factory.pickupSearchTimeout);
    ros::param::get("/sebot_factory/part_detect_max_width", factory.partDetectMaxWidth);
    ros::param::get("/sebot_factory/part_detect_max_height", factory.partDetectMaxHeight);
    ros::param::get("/sebot_factory/part_detect_min_y", factory.partDetectMinY);
    ros::param::get("/sebot_factory/camera_warmup_ms", factory.cameraWarmupMs);
    ros::param::get("/sebot_factory/camera_switch_settle_ms", factory.cameraSwitchSettleMs);
    ros::param::get("/sebot_factory/camera_switch_timeout", factory.cameraSwitchTimeout);
    ros::param::get("/sebot_factory/arm_extend_timeout", factory.armExtendTimeout);
    ros::param::get("/sebot_factory/arm_post_extend_settle_ms", factory.armPostExtendSettleMs);
    ros::param::get("/sebot_factory/claw_grab_wait_ms", factory.clawGrabWaitMs);
    ros::param::get("/sebot_factory/put_action_timeout", factory.putActionTimeout);
    ros::param::get("/sebot_factory/arm_post_put_settle_ms", factory.armPostPutSettleMs);
    ros::param::get("/sebot_factory/place_pre_release_wait_ms", factory.placePreReleaseWaitMs);
    ros::param::get("/sebot_factory/place_release_wait_ms", factory.placeReleaseWaitMs);
    ros::param::get("/sebot_factory/place_approach_out_max", factory.placeApproachOutMax);
    ros::param::get("/sebot_factory/place_drop_offset_y", factory.placeDropOffsetY);

    sleep(delay);                   // 等待所有任务节点均正常启动
    ros::Rate loop_rate(10);        // 设置ROS循环周期: Hz

    while (ros::ok())
    {
        factory.caterHandle();
        ros::spinOnce();   // 回调话题通信
        loop_rate.sleep(); // ROS线程周期控制
    }

    return 0;
}
