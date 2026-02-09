#pragma once
#include <linux/can.h>
#include <cstring>
#include <cstdint>
#include <string>
#include <sstream>


// ADCU 控制指令结构体
struct ADCU_CMD{
    double linear_speed;   // 线速度
    double angular_speed;  // 角速度
    uint8_t roll_cnt = 0;  // 滚动计数
    uint8_t fault_level = 0; // 故障等级
};

// BMS 总信息 0 结构体
struct BMS_TotalInfo0 {
    double sum_voltage;    // 总电压 (0.1V)
    double current;        // 电流 (0.1A, 偏移 -30000)
    double soc;            // 剩余电量 (0.1%)
    uint8_t life;          // 寿命 (0-255)
};

// BMS 总信息 1 结构体
struct BMS_TotalInfo1 {
    int32_t power;         // 功率 (1W)
    int32_t total_energy;  // 总能量 (1WH)
    int16_t mos_temp;      // MOS 温度 (1C, 偏移 -40)
    int16_t board_temp;    // 板载温度 (1C, 偏移 -40)
    int16_t heat_temp;     // 加热温度 (1C, 偏移 -40)
    double heat_current;   // 加热电流 (1A)
};

// BMS 统计信息结构体
struct BMS_Stats {
    uint16_t max_v;        // 最高单体电压 (1mV)
    uint8_t max_v_no;      // 最高单体电压编号
    uint16_t min_v;        // 最低单体电压 (1mV)
    uint8_t min_v_no;      // 最低单体电压编号
    uint16_t diff_v;       // 压差 (1mV)
    int16_t max_t;         // 最高温度 (1C, 偏移 -40)
    uint8_t max_t_no;      // 最高温度编号
    int16_t min_t;         // 最低温度 (1C)
    uint8_t min_t_no;      // 最低温度编号
    int16_t diff_t;        // 温差 (1C)
};

// BMS 状态信息 0 结构体
struct BMS_Status0 {
    uint8_t chg_mos_state;  // 充电 MOS 状态
    uint8_t dchg_mos_state; // 放电 MOS 状态
    uint8_t pre_mos_state;  // 预充 MOS 状态
    uint8_t heat_mos_state; // 加热 MOS 状态
    uint8_t fan_mos_state;  // 风扇 MOS 状态
    uint8_t do_state;       // DO 输出状态
    uint8_t di_state;       // DI 输入状态
};

// BMS 状态信息 1 结构体
struct BMS_Status1 {
    uint8_t bat_state;    // 电池状态
    uint8_t chg_detect;   // 充电检测
    uint8_t load_detect;  // 负载检测
};

// BMS 状态信息 2 结构体
struct BMS_Status2 {
    uint8_t cell_number;      // 电池串数
    uint8_t ntc_number;       // 温度传感器个数
    uint32_t remain_capacity; // 剩余容量 (mAH)
    uint16_t cycle_time;      // 循环次数
};

// BMS 时间结构体
struct BMS_Time {
    uint16_t year;   // 年
    uint8_t month;   // 月
    uint8_t day;     // 日
    uint8_t hour;    // 时
    uint8_t minute;  // 分
    uint8_t second;  // 秒
};

// BMS 故障信息结构体 (新增)
struct BMS_FaultData {
    uint8_t page1[8]; // 故障页 1 数据 (原始字节)
    uint8_t page2[8]; // 故障页 2 数据 (原始字节)
    
    BMS_FaultData() {
        std::memset(page1, 0, sizeof(page1));
        std::memset(page2, 0, sizeof(page2));
    }

    // 获取故障描述字符串
    std::string getFaultDescription() const;
};

// BMS 聚合数据结构体
struct BMS_Data {
    BMS_TotalInfo0 info0;
    BMS_TotalInfo1 info1;
    BMS_Stats stats;
    BMS_Status0 status0;
    BMS_Status1 status1;
    BMS_Status2 status2;
    BMS_Time time;
    BMS_FaultData fault_data; // 新增故障数据

    BMS_Data() {
        std::memset(&info0, 0, sizeof(info0));
        std::memset(&info1, 0, sizeof(info1));
        std::memset(&stats, 0, sizeof(stats));
        std::memset(&status0, 0, sizeof(status0));
        std::memset(&status1, 0, sizeof(status1));
        std::memset(&status2, 0, sizeof(status2));
        std::memset(&time, 0, sizeof(time));
        // fault_data has its own constructor
    }
};


class dbc_encoder{
    public:
        // 生成can报文
        dbc_encoder();
        ~dbc_encoder();
        // 生成can报文 (0x20)
        void encode0x20(double linear_speed, double angluar_speed, struct can_frame& frame, struct ADCU_CMD& adcu_cmd);
        
        // 生成 BMS 唤醒报文 (0x0400FF80)
        void encodeBMSControl(struct can_frame& frame);

        // 解码油门/速度信号 (旧协议/电机反馈)
        void parseVehicleSpd(const struct can_frame& frame,double& left_speed, double& right_speed);
        
        // 通用帧解析入口
        void parseFrame(const struct can_frame& frame, BMS_Data& bms_data);
        
        // 辅助解析方法
        void parseCellVoltage(const struct can_frame& frame); // 单体电压
        void parseCellTemp(const struct can_frame& frame);    // 单体温度
        void parseTotalInfo0(const struct can_frame& frame, BMS_TotalInfo0& info); // 总信息0
        void parseTotalInfo1(const struct can_frame& frame, BMS_TotalInfo1& info); // 总信息1
        void parseStats(const struct can_frame& frame, BMS_Stats& stats); // 统计信息 (0x0404, 0x0405)
        void parseStatus0(const struct can_frame& frame, BMS_Status0& status); // 状态0 (0x0406)
        void parseStatus1(const struct can_frame& frame, BMS_Status1& status); // 状态1 (0x0407)
        void parseStatus2(const struct can_frame& frame, BMS_Status2& status); // 状态2 (0x0408)
        void parseTime(const struct can_frame& frame, BMS_Time& time); // 时间 (0x040C)
        void parseFaultInfo(const struct can_frame& frame, BMS_FaultData& fault); // 故障信息 (0x040E)
};
