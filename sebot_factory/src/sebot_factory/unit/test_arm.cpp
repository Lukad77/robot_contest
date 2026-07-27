/**
 * @file test_arm.cpp
 * @brief 机械臂测试程序 — 逐步排查问题 + 夹爪测试
 * @note 编译后运行，观察输出信息判断故障位置
 */

#include "../include/arm.hpp"
#include <ros/ros.h>
#include <signal.h>

Talon *g_talon = nullptr;

void exitSignal(int signum)
{
    if (g_talon && g_talon->isOpen)
    {
        ROS_INFO("正在关闭机械臂...");
        g_talon->setActions(g_talon->ACTION_RES);  // 复位
        sleep(3);
        g_talon->setJointEnable(g_talon->ARM_JOINT_ALL, false);  // 失能
    }
    ROS_INFO("测试结束");
    exit(signum);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_arm");
    signal(SIGINT, exitSignal);

    ROS_INFO("===== 机械臂测试开始 =====");

    // ----------------------------------------
    // 步骤1：打开串口
    // ----------------------------------------
    ROS_INFO("[步骤1] 打开串口 /dev/talon ...");
    Talon talon;
    g_talon = &talon;
    sleep(2);

    if (!talon.isOpen)
    {
        ROS_ERROR("❌ 串口打开失败！请检查：");
        ROS_ERROR("   1. 机械臂是否已上电");
        ROS_ERROR("   2. /dev/talon 设备是否存在 (ls -l /dev/talon)");
        ROS_ERROR("   3. 串口是否被其他程序占用");
        return 1;
    }
    ROS_INFO("✅ 串口打开成功！");

    // ----------------------------------------
    // 步骤2：关节使能
    // ----------------------------------------
    ROS_INFO("[步骤2] 机械臂关节使能...");
    talon.setJointEnable(talon.ARM_JOINT_ALL, true);
    sleep(2);
    // 再发一次确保使能
    talon.setJointEnable(talon.ARM_JOINT_ALL, true);
    sleep(1);
    ROS_INFO("✅ 关节使能完成（关节应感觉有阻力/锁住）");

    // ----------------------------------------
    // 步骤3：测试动作库
    // ----------------------------------------
    ROS_INFO("[步骤3] ⚠️ 即将执行伸展动作，请注意安全距离！");
    ROS_INFO("   3秒后开始...");
    sleep(3);

    ROS_INFO("  → 执行：伸展 (ACTION_EXT)");
    talon.setActions(talon.ACTION_EXT);
    sleep(5);

    if (talon.actionOver[talon.ACTION_EXT])
        ROS_INFO("  ✅ 伸展完成");
    else
        ROS_WARN("  ⚠️ 伸展可能未完成（动作超时或下位机未反馈）");

    // ----------------------------------------
    // 步骤4：夹爪测试（关键部分）
    // ----------------------------------------
    ROS_INFO("[步骤4] 夹爪测试 ================");

    // 4.1 标定机械爪
    ROS_INFO("  4.1 机械爪标定...");
    talon.setClawInitial();
    sleep(3);
    ROS_INFO("  ✅ 标定完成");

    // 4.2 机械爪张开到最大
    ROS_INFO("  4.2 机械爪张开到 9.0cm ...");
    talon.setJointPosition(talon.ARM_JOINT_6, 9.0);
    sleep(3);
    ROS_INFO("  ✅ 张开完成");

    // 4.3 机械爪闭合到不同开度，测试夹持力
    float claw_positions[] = {7.0, 5.0, 4.8, 4.0, 3.0, 2.0, 1.0};
    for (int i = 0; i < 7; i++)
    {
        ROS_INFO("  4.3 机械爪闭合到 %.1fcm ...", claw_positions[i]);
        talon.setJointPosition(talon.ARM_JOINT_6, claw_positions[i]);
        sleep(3);
        ROS_INFO("  ✅ 已闭合到 %.1fcm", claw_positions[i]);
    }

    // 4.4 再次张开
    ROS_INFO("  4.4 机械爪重新张开到 9.0cm ...");
    talon.setJointPosition(talon.ARM_JOINT_6, 9.0);
    sleep(3);

    // 4.5 快速夹紧测试（模拟抓取）
    ROS_INFO("  4.5 快速夹紧测试（模拟抓取动作）...");
    for (int i = 0; i < 3; i++)
    {
        ROS_INFO("  第 %d 次夹紧", i + 1);
        talon.setJointPosition(talon.ARM_JOINT_6, 4.0);
        sleep(2);
        talon.setJointPosition(talon.ARM_JOINT_6, 9.0);
        sleep(2);
    }
    ROS_INFO("  ✅ 夹紧测试完成");

    // 4.6 机械爪微调测试（setClawMotion）
    ROS_INFO("  4.6 机械爪XY微调测试...");
    talon.setClawMotion(0.0, 0.0);   // 归中
    sleep(2);
    talon.setClawMotion(0.2, 0.0);   // X方向微调
    sleep(2);
    talon.setClawMotion(-0.2, 0.0);  // X方向反向微调
    sleep(2);
    talon.setClawMotion(0.0, 0.15);  // Y方向抬起
    sleep(2);
    talon.setClawMotion(0.0, 0.0);   // 归中
    sleep(2);
    ROS_INFO("  ✅ 微调测试完成");

    // ----------------------------------------
    // 步骤5：收缩并复位
    // ----------------------------------------
    ROS_INFO("[步骤5] 收缩复位...");
    talon.setActions(talon.ACTION_CUR);
    sleep(4);

    if (talon.actionOver[talon.ACTION_CUR])
        ROS_INFO("  ✅ 收缩完成");
    else
        ROS_WARN("  ⚠️ 收缩可能未完成");

    talon.setActions(talon.ACTION_RES);
    sleep(4);

    // ----------------------------------------
    // 步骤6：失能
    // ----------------------------------------
    ROS_INFO("[步骤6] 机械臂失能...");
    talon.setJointEnable(talon.ARM_JOINT_ALL, false);
    sleep(1);

    ROS_INFO("===== 测试完成 =====");
    return 0;
}
