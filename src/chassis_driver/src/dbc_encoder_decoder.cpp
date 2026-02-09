#include <dbc_encoder_decoder.h>
#include <iostream>
#include <iomanip>
#include <cstring>
#include <cmath>

dbc_encoder::dbc_encoder(){}
dbc_encoder::~dbc_encoder(){}
// 解析速度信号
// void dbc_encoder::parseVehicleSpd(const struct can_frame& frame,double& left_speed, double& right_speed){
//     if (frame.can_id == 0x581 && frame.can_dlc == 8) {
//         if(frame.data[0]== 0x43 && frame.data[1] == 0x6C){
//             // std::cout<<"data: ";
//             // for (int i = 0; i < frame.can_dlc; ++i) {
//             //     std::cout << std::hex <<std::setw(2)<<std::setfill('0')<<static_cast<int>(frame.data[i]) << " ";
//             // }
//             // std::cout << std::endl << "raw_left: ";
//             int16_t raw_left = ((int16_t)frame.data[5] << 8) | frame.data[4];
//             std::cout << std::hex <<std::setw(2)<<std::setfill('0')<<static_cast<int16_t>(raw_left) << std::endl; 
//             left_speed = static_cast<double>(raw_left/10.0);// r/min
//             int16_t raw_right = (int16_t)(frame.data[7] << 8 | frame.data[6]);
//             right_speed = static_cast<double>(-(raw_right/10.0));//r/min
//             // vehicleSpd =  ((left_speed/60*2*M_PI*0.0535)+(right_speed/60*2*M_PI* 0.0535))/2;
//             // std::cout<<"left_speed: "<<left_speed<<" r/min, right_speed: "<<right_speed<<std::endl;
//             // std::cout<<"vehicle_speed: "<<vehicleSpd<<std::endl;
//         }
//     }
// }

void dbc_encoder::parseVehicleSpd(const struct can_frame& frame, double& left_speed, double& right_speed){
    // 1. 严格检查 ID
    if (frame.can_id != 0x581 || frame.can_dlc != 8) {
        return;
    }
    // std::cout<<"data: ";
    // for (int i = 0; i < frame.can_dlc; ++i) {
    //     std::cout << std::hex <<std::setw(2)<<std::setfill('0')<<static_cast<int>(frame.data[i]) << " ";
    // }
    // 2. 提取 CANopen 协议头信息
    uint8_t cmd = frame.data[0];    // 0x43, 0x4B 等
    uint16_t index = (frame.data[2] << 8) | frame.data[1]; // 索引，应为 0x606C
    uint8_t subindex = frame.data[3]; // 子索引，关键判断依据！

    // 3. 仅处理 0x606C (速度反馈)
    if (index == 0x606C) {        
        // ==========================================================
        // 情况 A: 子索引 01 (通常代表左轮单独反馈)
        // ==========================================================
        if (subindex == 0x01) {
            // 数据在 4-7 字节，通常是 32位整数
            int32_t raw_val = (frame.data[7]<<24 | frame.data[6]<<16 | frame.data[5]<<8 | frame.data[4]);
            left_speed = static_cast<double>(raw_val / 10.0);
            // ⚠️ 注意：这里不要修改 right_speed，保持上一帧的值
        }
        else if (subindex == 0x02) {
            // 右轮速度 (注意取反)
            int32_t raw_val = (frame.data[7]<<24 | frame.data[6]<<16 | frame.data[5]<<8 | frame.data[4]);
            right_speed = static_cast<double>(-raw_val / 10.0);
        }
        else if (subindex == 0x03) {
            // 双通道打包数据
            int16_t raw_left = (int16_t)((frame.data[5] << 8) | frame.data[4]);
            int16_t raw_right = (int16_t)((frame.data[7] << 8) | frame.data[6]);
            left_speed = static_cast<double>(raw_left / 10.0);
            right_speed = static_cast<double>(-(raw_right / 10.0));
        }
    }
}

// 编码 0x20 控制帧
void dbc_encoder::encode0x20(double linear_speed, double angluar_speed, struct can_frame& frame, struct ADCU_CMD& adcu_cmd) {
    frame.can_id = 0x20;
    frame.can_dlc = 8;
    std::memset(frame.data, 0, sizeof(frame.data));
    int16_t linear_speed_raw = static_cast<int16_t>(linear_speed/0.001);
    int16_t angular_speed_raw = static_cast<int16_t>((angluar_speed)/0.01);
    frame.data[2] = (linear_speed_raw >> 8)&0xFF;
    frame.data[3] = linear_speed_raw &0xFF;
    frame.data[0] = (angular_speed_raw >> 8)&0xFF;
    frame.data[1] = angular_speed_raw &0xFF;
    frame.data[4] = adcu_cmd.fault_level &0xFF;
}

// 编码 BMS 唤醒/控制帧 (0x0400FF80)
void dbc_encoder::encodeBMSControl(struct can_frame& frame) {
    frame.can_id = 0x0400FF80 | CAN_EFF_FLAG; // 扩展帧
    frame.can_dlc = 8;
    std::memset(frame.data, 0, sizeof(frame.data));
}

// ==========================================================
// 新增 BMS 协议实现
// ==========================================================

void dbc_encoder::parseFrame(const struct can_frame& frame, BMS_Data& bms_data) {
    // 屏蔽标志位获取基础 ID
    uint32_t id = frame.can_id & CAN_EFF_MASK;
    
    // 过滤源地址 (最后 8 位)
    uint32_t pg_id = id & 0xFFFFFF00; 

    switch (pg_id) {
        case 0x04028000:
            parseTotalInfo0(frame, bms_data.info0);
            break;
        case 0x04038000:
            parseTotalInfo1(frame, bms_data.info1);
            break;
        case 0x04048000: // 电压统计
        case 0x04058000: // 温度统计
            parseStats(frame, bms_data.stats);
            break;
        case 0x04068000:
            parseStatus0(frame, bms_data.status0);
            break;
        case 0x04078000:
            parseStatus1(frame, bms_data.status1);
            break;
        case 0x04088000:
            parseStatus2(frame, bms_data.status2);
            break;
        case 0x040C8000:
            parseTime(frame, bms_data.time);
            break;
        case 0x040E8000: // 故障信息 (新增)
            parseFaultInfo(frame, bms_data.fault_data);
            break;
        case 0x04008000: // 单体电压 - 待实现
            parseCellVoltage(frame);
            break;
        case 0x04018000: // 单体温度 - 待实现
            parseCellTemp(frame); 
            break;
        default:
            // std::cout << "未知 ID: " << std::hex << id << std::endl;
            break;
    }
}

void dbc_encoder::parseTotalInfo0(const struct can_frame& frame, BMS_TotalInfo0& info) {
    // Byte 0-1: 总电压 (0.1V)
    uint16_t sum_v = (frame.data[0] << 8) | frame.data[1];
    info.sum_voltage = sum_v * 0.1;
    
    // Byte 2-3: 电流 (0.1A, 偏移 -30000)
    uint16_t curr_raw = (frame.data[2] << 8) | frame.data[3];
    info.current = (curr_raw * 0.1) - 3000.0; 
    
    // Byte 4-5: SOC (0.1%)
    uint16_t soc_raw = (frame.data[4] << 8) | frame.data[5];
    info.soc = soc_raw * 0.1;
    
    // Byte 6: 寿命
    info.life = frame.data[6];
}

void dbc_encoder::parseTotalInfo1(const struct can_frame& frame, BMS_TotalInfo1& info) {
    // Byte 0-1: 功率 (1W)
    info.power = (int16_t)((frame.data[0] << 8) | frame.data[1]);
    
    // Byte 2-3: 总能量 (1WH)
    info.total_energy = (int16_t)((frame.data[2] << 8) | frame.data[3]);
    
    // Byte 4: MOS 温度 (偏移 -40)
    info.mos_temp = (int16_t)frame.data[4] - 40;
    
    // Byte 5: 板载温度
    info.board_temp = (int16_t)frame.data[5] - 40;
    
    // Byte 6: 加热温度
    info.heat_temp = (int16_t)frame.data[6] - 40;
    
    // Byte 7: 加热电流 (1A)
    info.heat_current = frame.data[7];
}

void dbc_encoder::parseStats(const struct can_frame& frame, BMS_Stats& stats) {
    uint32_t id = frame.can_id & 0xFFFFFF00;
    if (id == 0x04048000) { // 电压统计
        stats.max_v = (frame.data[0] << 8) | frame.data[1];
        stats.max_v_no = frame.data[2];
        stats.min_v = (frame.data[3] << 8) | frame.data[4];
        stats.min_v_no = frame.data[5];
        stats.diff_v = (frame.data[6] << 8) | frame.data[7];
    } else if (id == 0x04058000) { // 温度统计
        stats.max_t = (int16_t)frame.data[0] - 40;
        stats.max_t_no = frame.data[1];
        stats.min_t = (int16_t)frame.data[2] - 40;
        stats.min_t_no = frame.data[3];
        stats.diff_t = (int16_t)frame.data[4];
    }
}

void dbc_encoder::parseStatus0(const struct can_frame& frame, BMS_Status0& status) {
    status.chg_mos_state = frame.data[0];
    status.dchg_mos_state = frame.data[1];
    status.pre_mos_state = frame.data[2];
    status.heat_mos_state = frame.data[3];
    status.fan_mos_state = frame.data[4];
    status.do_state = frame.data[5];
    status.di_state = frame.data[6];
}

void dbc_encoder::parseStatus1(const struct can_frame& frame, BMS_Status1& status) {
    status.bat_state = frame.data[0];
    status.chg_detect = frame.data[1];
    status.load_detect = frame.data[2];
}

void dbc_encoder::parseStatus2(const struct can_frame& frame, BMS_Status2& status) {
    status.cell_number = frame.data[0];
    status.ntc_number = frame.data[1];
    // 剩余容量 (可能是 4 字节，起始于 Byte 2)
    status.remain_capacity = (frame.data[2] << 24) | (frame.data[3] << 16) | (frame.data[4] << 8) | frame.data[5]; 
    
    // Byte 6-7: 循环次数
    status.cycle_time = (frame.data[6] << 8) | frame.data[7];
}

void dbc_encoder::parseTime(const struct can_frame& frame, BMS_Time& time) {
    time.year = 2000 + frame.data[0];
    time.month = frame.data[1];
    time.day = frame.data[2];
    time.hour = frame.data[3];
    time.minute = frame.data[4];
    time.second = frame.data[5];
}

void dbc_encoder::parseFaultInfo(const struct can_frame& frame, BMS_FaultData& fault) {
    uint8_t page_no = frame.data[0];
    if (page_no == 1) {
        // 保存第 1 页故障数据
        std::memcpy(fault.page1, frame.data, 8);
    } else if (page_no == 2) {
        std::memcpy(fault.page2, frame.data, 8);
    }
}

std::string BMS_FaultData::getFaultDescription() const {
    std::stringstream ss;
    bool fault_found = false;

    // Helper lambda to append fault string
    auto append = [&](const char* msg) {
        if (fault_found) ss << "; ";
        ss << msg;
        fault_found = true;
    };
    
    // ==========================================
    // Page 1: 告警等级 (0=正常)
    // ==========================================
    
    // Byte 1
    if ((page1[1] & 0x07) > 0) append("充电低温告警");
    if ((page1[1] & 0x38) > 0) append("放电高温告警");
    if (page1[1] & 0x40) append("充电MOS过温");
    if (page1[1] & 0x80) append("充电MOS温度传感器故障");

    // Byte 2
    if ((page1[2] & 0x07) > 0) append("放电低温告警");
    if ((page1[2] & 0x38) > 0) append("压差过大告警");
    if (page1[2] & 0x40) append("放电MOS过温");
    if (page1[2] & 0x80) append("放电MOS温度传感器故障");

    // Byte 3
    if ((page1[3] & 0x07) > 0) append("总压过高告警");
    if ((page1[3] & 0x38) > 0) append("总压过低告警");
    if (page1[3] & 0x40) append("短路保护");
    if (page1[3] & 0x80) append("高压禁止放电");

    // Byte 4
    if ((page1[4] & 0x07) > 0) append("充电过流告警");
    if ((page1[4] & 0x38) > 0) append("放电过流告警");
    if (page1[4] & 0x40) append("低压禁止充电");
    // Bit 6: 并机成功(忽略)
    if (page1[4] & 0x80) append("并机通信失败");

    // Byte 5
    if ((page1[5] & 0x07) > 0) append("SOC过低告警");
    if ((page1[5] & 0x38) > 0) append("SOH过低告警");

    // ==========================================
    // Page 2: 硬件故障标志 (1=故障)
    // ==========================================
    
    // Byte 1: AFE & 采集
    if (page2[1] & 0x01) append("AFE芯片故障");
    if (page2[1] & 0x02) append("AFE通信故障");
    if (page2[1] & 0x04) append("AFE采样故障");
    if (page2[1] & 0x08) append("电压检测故障");
    if (page2[1] & 0x10) append("电压采集线掉线");
    if (page2[1] & 0x20) append("总压检测故障");
    if (page2[1] & 0x40) append("电流检测故障");
    if (page2[1] & 0x80) append("温度检测故障");

    // Byte 2: 硬件与MOS
    if (page2[2] & 0x01) append("温度采集线掉线");
    if (page2[2] & 0x02) append("EEPROM存储故障");
    if (page2[2] & 0x04) append("Flash存储故障");
    if (page2[2] & 0x08) append("RTC时钟故障");
    if (page2[2] & 0x10) append("充电MOS故障");
    if (page2[2] & 0x20) append("放电MOS故障");
    if (page2[2] & 0x40) append("预充MOS故障");
    if (page2[2] & 0x80) append("预充失败");

    if (!fault_found) {
        return "systerm normal";
    }
    return ss.str();
}

void dbc_encoder::parseCellVoltage(const struct can_frame& frame) {
    // 预留接口
}

void dbc_encoder::parseCellTemp(const struct can_frame& frame) {
    // 预留接口
}
