/**
 * @file ethercat_device.h
 * @author mq (maqun@buaa.edu.cm)
 * @brief
 * @version 0.1
 * @date 2021-06-24
 *
 * @copyright Copyright (c) 2021 ROSC
 *
 */

#ifndef MODULES_COMPONENTS_ETHERCAT_COMPONENT_INCLUDE_ETHERCAT_DEVICE_H_
#define MODULES_COMPONENTS_ETHERCAT_COMPONENT_INCLUDE_ETHERCAT_DEVICE_H_

#include <bits/stdint-uintn.h>
#include <ecrt.h>
#include <sys/types.h>
#include <robot_brain/core.hpp>
#include <rtt/Time.hpp>
#include <rtt/os/TimeService.hpp>

#define DC_FILTER_CNT 50
#define SYNC_MASTER_TO_REF 1
#define NSEC_PER_SEC (1000000000L)
#define TIMESPEC2NS(T) ((uint64_t)(T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)

/** Return the sign of a number
 *
 * ie -1 for -ve value, 0 for 0, +1 for +ve value
 *
 * \retval the sign of the value
 */
#define sign(val) ({ ((val > 0) - (val < 0)); })

typedef long long RTIME;
namespace rosc {

/**
 * @brief 电机的品牌，不同电机的参数不一样，如果电机确定的话，值就不变了
 *
 */
typedef struct SlaveConfiguration {
  uint16_t alias;         // 别名
  uint16_t position;      // 总线位置
  uint32_t vendor_id;     // 供应商id
  uint32_t product_code;  // 产品代码
} SlaveConfiguration;

typedef struct Offset {
  uint operation_mode;           // 操作模式
  uint ctrl_word;                // 控制字
  uint status_word;              // 状态字
  uint target_position;          // 目标位置
  uint current_position;         // 当前位置
  uint modes_operation_display;  // 操作字显示
  uint target_velocity;          // 目标速度
  uint target_torque;            // 目标力矩
  uint velocity_actual_value;    // 当前速度
  uint torque_actual_value;      // 当前力矩
  uint error_code;               // 错误代码

  // 后续按需要增加新的字段
  unsigned int digit_out;  // 数字输出
  unsigned int digit_in;   // 数字输入
  unsigned int digit_out_8;  // 16位io输出的高8位
} Offset;

class EthercatDevice {
 public:
  explicit EthercatDevice(uint32_t period);
  ~EthercatDevice();
  bool CreateMaster(uint id);
  bool AddDomain();
  bool ConfigSlaves();
  bool SelectReferenceClock();
  bool DomainRegister();
  bool DomainRegisterList();
  bool MasterActive();
  bool DomainData();
  void MasterDeactive();
  void MasterApplicationTime(uint64_t application_time);
  void WriteEthercatCtlFrame(const EthercatCtlFrame &frame);
  void WriteEthercatCtlFrame();
  void ReadEthercatStatusFrame(EthercatStatusFrame *status_frame);
  void WaitPeriod();
  void SetWaitUpTime(uint64_t time);
  uint64_t SystemTimeNs();

  RTIME System2count(uint64_t time);

 private:
  void CheckDomainState();
  void CheckMasterState();
  void CheckSlaveConfigStates();
  void CheckAndUpdateState();
  void SyncClocks();
  void UpdateMasterClock(void);

 private:
  int slave_num_;
  int dof_;
  SlaveType *slave_list_;

 private:
  // 驱动器相关的pdo信息
  // static ec_pdo_entry_info_t *pdo_entries;
  // static ec_pdo_info_t *pdo_info_;
  // ec_sync_info_t **syncs_info_;
  uint16_t *driver_alias;
  uint16_t *driver_positions;
  uint32_t *driver_vendor_ids;
  uint32_t *driver_product_codes;

  // ec相关内容
  ec_master_t *master_;  // 主站指针
  ec_domain_t *domain_;  // 域指针
  uint8_t *domain_pd_;   // ecrt_domain_data()返回的domain逻辑地址
  // ec_slave_config_t *sc_[kSLAVENUM];  // ecrt_master_slave_config()返回值
  ec_slave_config_t **sc_;

  // SlaveConfiguration slave_config_[kSLAVENUM];  // 每个从站的属性
  SlaveConfiguration *slave_config_;
  // ec_pdo_entry_reg_t domain_regs[kIOPDONUM * kIOSLAVENUM + kIOINSLAVENUM +
  //                                kIOOUTSLAVENUM * kIOPDONUM +
  //                                kPDONUM * (kSLAVENUM - kIOSLAVENUM -
  //                                           kIOINSLAVENUM - kIOOUTSLAVENUM) +
  //                                1];
  ec_pdo_entry_reg_t *domain_regs;
  // Offset offset_[kSLAVENUM];
  Offset *offset_;

  ec_master_state_t master_state_;                         // 主站状态
  ec_domain_state_t domain_state_;                         // 域状态
  // ec_slave_config_state_t slave_config_state_[kSLAVENUM];  // 从站状态
  ec_slave_config_state_t *slave_config_state_;

  uint32_t period_ns_;          // 时钟周期
  bool reference_slave_clock_;  // 从站时钟同步

  // EtherCAT distributed clock variables
  uint64_t dc_start_time_ns_ = 0LL;
  uint64_t dc_time_ns_ = 0;

  uint8_t dc_started_ = 0;
  int32_t dc_diff_ns_ = 0;
  int32_t dc_fail_sleep_time_ = 0;
  bool dc_filter_idx_flag_ = false;
  int32_t prev_dc_diff_ns_ = 0;
  int64_t normalise_dc_diff_total_ns_ = 0LL;
  int64_t dc_diff_total_ns_ = 0LL;
  int64_t dc_diff_total_ns1_ = 0LL;
  int64_t dc_delta_total_ns_ = 0LL;
  int dc_filter_idx_ = 0;
  int64_t dc_adjust_ns_;
  uint32_t dc_max_time_diff_;

  int64_t system_time_base_ = 0LL;
  uint64_t wakeup_time_ = 0LL;
  uint64_t overruns_ = 0LL;
  int update_master_clock_time_ = 0;
};
}  // namespace rosc

#endif  // MODULES_COMPONENTS_ETHERCAT_COMPONENT_INCLUDE_ETHERCAT_DEVICE_H_
