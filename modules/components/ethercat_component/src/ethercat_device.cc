/**
 * @file ethercat_device.cc
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief ethercat设备相关内容
 * @version 0.1
 * @date 2021-10-25
 *
 * @copyright Copyright (c) 2021 ROSC
 *
 */

#include <ethercat_device.h>
#include <math.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/time.h>
#include <bits/types/struct_timespec.h>
#include <bits/stdint-intn.h>
#include <bits/stdint-uintn.h>
#include <ecrt.h>
#include <glog/logging.h>
#include <iostream>
#include <stack>
#include <cstring>
#include <ctime>
#include <memory>
#include <rtt/os/rtt-os-fwd.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>
#include <robot_brain/core.hpp>
#include <eni_slaves.hpp>

namespace rosc {

// ec_sync_info_t **EthercatDevice::syncs_info_ = slave_syncs_all;

/**
 * @brief EthercatDevice构造函数
 * 初始化类内指针、变量，初始化slave_config_, domain_regs等
 * @param period 周期, 单位为纳秒（ns）
 */
EthercatDevice::EthercatDevice(uint32_t period) {
  this->domain_ = nullptr;
  this->master_ = nullptr;
  this->domain_pd_ = nullptr;
  this->master_state_ = {};
  this->domain_state_ = {};
  this->period_ns_ = period;
  rosc::Config config = *rosc::Application::GetContext()->GetConfig();

  reference_slave_clock_ =
      config["bus_config"]["reference_slave_clock"].as<bool>();
  LOG(INFO) << "period_ns_ = " << period_ns_
            << "; reference_slave_clock = " << reference_slave_clock_;

  int arm_tp = config["init_global_config"]["robot_type"].as<int>();
  YAML::Node slave_list;
  switch (arm_tp) {
  case 0:
    this->slave_num_ =
        config["zero_device_config"]["zero_device_config"]["robot"]["slave_num"]
            .as<int>();
    this->dof_ =
        config["zero_device_config"]["zero_device_config"]["robot"]["dof"]
            .as<int>();
    slave_list = config["zero_device_config"]["zero_device_config"]["robot"]
                       ["slave_type_list"];
    break;
  case 1:
    this->slave_num_ = config["scara_device_config"]["scara_device_config"]
                             ["robot"]["slave_num"]
                                 .as<int>();
    this->dof_ =
        config["scara_device_config"]["scara_device_config"]["robot"]["dof"]
            .as<int>();
    slave_list = config["scara_device_config"]["scara_device_config"]["robot"]
                       ["slave_type_list"];
    break;
  default:
    this->slave_num_ = 0;
    break;
  }
  this->slave_list_ = new SlaveType[this->slave_num_]();
  int motor_slave_num = 0;
  int in_slave_num = 0;
  int out_slave_num = 0;
  int io_slave_num = 0;
  for (int i = 0; i < this->slave_num_; ++i) {
    int tp = slave_list[i].as<int>();
    switch (tp) {
    case 0:
      this->slave_list_[i] = MOTOR_SERVO;
      motor_slave_num++;
      break;
    case 1:
      this->slave_list_[i] = IO_SERVO;
      io_slave_num++;
      break;
    case 2:
      this->slave_list_[i] = IO_SERVO_IN;
      in_slave_num++;
      break;
    case 3:
      this->slave_list_[i] = IO_SERVO_OUT;
      out_slave_num++;
      break;
    default:
      break;
    }
  }

  // 对一些指针数组申请空间
  driver_alias = new uint16_t[this->slave_num_]();
  driver_positions = new uint16_t[this->slave_num_]();
  driver_vendor_ids = new uint32_t[this->slave_num_]();
  driver_product_codes = new uint32_t[this->slave_num_]();

  this->slave_config_ = new SlaveConfiguration[this->slave_num_]();

  this->offset_ = new Offset[this->slave_num_]();

  this->slave_config_state_ = new ec_slave_config_state_t[this->slave_num_]();

  this->sc_ = new ec_slave_config_t *[this->slave_num_]();

  if (arm_tp == 0) {  // 6-zero
    memset(driver_alias, 0, sizeof(uint16_t) * this->slave_num_);
    uint16_t slave_pos[] = {0, 1, 2, 3, 4, 5};
    memcpy(driver_positions, slave_pos, sizeof(slave_pos));
    uint32_t ven_id[] = {vendor_id_zero, vendor_id_zero, vendor_id_zero,
                         vendor_id_zero, vendor_id_zero, vendor_id_zero};
    memcpy(driver_vendor_ids, ven_id, sizeof(ven_id));
    uint32_t p_code[] = {product_code_zero, product_code_zero,
                         product_code_zero, product_code_zero,
                         product_code_zero, product_code_zero};
    memcpy(driver_product_codes, p_code, sizeof(p_code));
  } else if (arm_tp == 1) {  // scara
    memset(driver_alias, 0, sizeof(uint16_t) * this->slave_num_);
  }

  // 配置alias，position，VID PID
  for (int i = 0; i < this->slave_num_; ++i) {
    this->slave_config_[i].alias = driver_alias[i];  // 不同的从站可能会不同
    this->slave_config_[i].vendor_id = driver_vendor_ids[i];
    this->slave_config_[i].product_code = driver_product_codes[i];
    this->slave_config_[i].position = driver_positions[i];
  }

  // 生成regs, 这些参数应该从配置文件中获得
  int pdo_num = io_pdo_num * io_slave_num + in_slave_num +
                out_slave_num * io_pdo_num + motor_pdo_num * motor_slave_num;
  this->domain_regs = new ec_pdo_entry_reg_t[pdo_num + 1]();

  int seq = 0;
  for (int i = 0; i < this->slave_num_; ++i) {
    if (this->slave_list_[i] == SlaveType::IO_SERVO) {
      this->domain_regs[seq] = {this->slave_config_[i].alias,
                                this->slave_config_[i].position,
                                this->slave_config_[i].vendor_id,
                                this->slave_config_[i].product_code,
                                0x7080, /*out_gen_bit1*/
                                0x01,
                                &this->offset_[i].digit_out};
      seq++;
      this->domain_regs[seq] = {this->slave_config_[i].alias,
                                this->slave_config_[i].position,
                                this->slave_config_[i].vendor_id,
                                this->slave_config_[i].product_code,
                                0x6000, /*out_gen_bit1*/
                                0x01,
                                &this->offset_[i].digit_in};
      seq++;
      continue;
    }
    if (this->slave_list_[i] == SlaveType::IO_SERVO_IN) {
      this->domain_regs[seq] = {this->slave_config_[i].alias,
                                this->slave_config_[i].position,
                                this->slave_config_[i].vendor_id,
                                this->slave_config_[i].product_code,
                                0x6000, /*input*/
                                0x01,
                                &this->offset_[i].digit_in};
      seq++;
      continue;
    }
    if (this->slave_list_[i] == SlaveType::IO_SERVO_OUT) {
      this->domain_regs[seq] = {this->slave_config_[i].alias,
                                this->slave_config_[i].position,
                                this->slave_config_[i].vendor_id,
                                this->slave_config_[i].product_code,
                                0x7000, /*out_gen_bit1*/
                                0x01,
                                &this->offset_[i].digit_out};
      seq++;
      this->domain_regs[seq] = {this->slave_config_[i].alias,
                                this->slave_config_[i].position,
                                this->slave_config_[i].vendor_id,
                                this->slave_config_[i].product_code,
                                0x7080, /*out_gen_bit1*/
                                0x01,
                                &this->offset_[i].digit_out_8};
      seq++;
      continue;
    }
    this->domain_regs[seq] = {this->slave_config_[i].alias,
                              this->slave_config_[i].position,
                              this->slave_config_[i].vendor_id,
                              this->slave_config_[i].product_code,
                              0x6040, /*ctrl word*/
                              0,
                              &this->offset_[i].ctrl_word};
    seq++;
    // this->domain_regs[seq] = {this->slave_config_[i].alias,
    //                           this->slave_config_[i].position,
    //                           this->slave_config_[i].vendor_id,
    //                           this->slave_config_[i].product_code,
    //                           0x6060, /*operation mode*/
    //                           0,
    //                           &this->offset_[i].operation_mode};
    // seq++;
    this->domain_regs[seq] = {this->slave_config_[i].alias,
                              this->slave_config_[i].position,
                              this->slave_config_[i].vendor_id,
                              this->slave_config_[i].product_code,
                              0x6041, /*status word*/
                              0,
                              &this->offset_[i].status_word};
    seq++;
    this->domain_regs[seq] = {this->slave_config_[i].alias,
                              this->slave_config_[i].position,
                              this->slave_config_[i].vendor_id,
                              this->slave_config_[i].product_code,
                              0x6064, /*current position*/
                              0,
                              &this->offset_[i].current_position};
    seq++;
    this->domain_regs[seq] = {this->slave_config_[i].alias,
                              this->slave_config_[i].position,
                              this->slave_config_[i].vendor_id,
                              this->slave_config_[i].product_code,
                              0x607a, /*target positon*/
                              0,
                              &this->offset_[i].target_position};
    seq++;
    // this->domain_regs[seq] = {this->slave_config_[i].alias,
    //                           this->slave_config_[i].position,
    //                           this->slave_config_[i].vendor_id,
    //                           this->slave_config_[i].product_code,
    //                           0x6061, /*mode operation display*/
    //                           0,
    //                           &this->offset_[i].modes_operation_display};
    // seq++;
    // this->domain_regs[seq] = {this->slave_config_[i].alias,
    //                           this->slave_config_[i].position,
    //                           this->slave_config_[i].vendor_id,
    //                           this->slave_config_[i].product_code,
    //                           0x6071, /*target torque*/
    //                           0,
    //                           &this->offset_[i].target_torque};
    // seq++;
    // this->domain_regs[seq] = {this->slave_config_[i].alias,
    //                           this->slave_config_[i].position,
    //                           this->slave_config_[i].vendor_id,
    //                           this->slave_config_[i].product_code,
    //                           0x60FF, /*target velocity*/
    //                           0,
    //                           &this->offset_[i].target_velocity};
    // seq++;
    // this->domain_regs[seq] = {this->slave_config_[i].alias,
    //                           this->slave_config_[i].position,
    //                           this->slave_config_[i].vendor_id,
    //                           this->slave_config_[i].product_code,
    //                           0x603F, /*error code*/
    //                           0,
    //                           &this->offset_[i].error_code};
    // seq++;
    // this->domain_regs[seq] = {this->slave_config_[i].alias,
    //                           this->slave_config_[i].position,
    //                           this->slave_config_[i].vendor_id,
    //                           this->slave_config_[i].product_code,
    //                           0x606C, /*Velocity actual value*/
    //                           0,
    //                           &this->offset_[i].velocity_actual_value};
    // seq++;
    // this->domain_regs[seq] = {this->slave_config_[i].alias,
    //                           this->slave_config_[i].position,
    //                           this->slave_config_[i].vendor_id,
    //                           this->slave_config_[i].product_code,
    //                           0x6077, /*torque actual value*/
    //                           0,
    //                           &this->offset_[i].torque_actual_value};
    // seq++;
  }
  // 最后一个位置，要用一个空的作为结尾。
  if (pdo_num != seq) {
    LOG(INFO) << "pdo 数量初始化错误";
    return;
  }
  this->domain_regs[pdo_num].index = 0;
  this->domain_regs[pdo_num].subindex = 0;
}  // NOLINT

/**
 * @brief EthercatDevice 析构函数
 * 销毁主站
 * 销毁读写锁，队列锁
 */
EthercatDevice::~EthercatDevice() {
  if (this->master_) {
    this->MasterDeactive();
    this->master_ = nullptr;
    LOG(INFO) << "Master Deactivete.";
  }
  delete[] this->slave_list_;
  delete[] this->domain_regs;
  delete[] this->driver_alias;
  delete[] this->driver_positions;
  delete[] this->driver_product_codes;
  delete[] this->driver_vendor_ids;
  delete[] this->slave_config_;
  delete[] this->slave_config_state_;
  delete[] this->offset_;
  delete[] this->sc_;
}

/**
 * @brief 请求用于实时操作的EtherCAT主站，获取主站的指针ec_master_t*
 *
 * @param id
 * @return true
 * @return false
 */
bool EthercatDevice::CreateMaster(uint id) {
  this->master_ = ecrt_request_master(id);
  if (!this->master_) {
    LOG(ERROR) << "Create Master failed! System Exits!";
    return false;
  } else {
    return true;
  }
}

/**
 * @brief 创建新的过程数据域，获取数据域指针ec_domain_t*
 *
 * @return true
 * @return false
 */
bool EthercatDevice::AddDomain() {
  if (this->master_ == nullptr) {
    LOG(INFO) << "EtherCAT Master initializes failed.";
    return false;
  }
  this->domain_ = ecrt_master_create_domain(this->master_);
  if (!this->domain_) {
    LOG(FATAL) << "Create Domain Failed! System exits!";
    return false;
  } else {
    return true;
  }
}

/**
 * @brief 获取从站配置，获得从站配置指针ec_slave_config_t*
 *        依次调用：ecrt_master_slave_config()
                   ecrt_slave_config_pdos()
                   ecrt_slave_config_dc()
 * @return true
 * @return false
 */
bool EthercatDevice::ConfigSlaves() {
  // 调用ecrt_master_slave_config()
  for (int i = 0; i < this->slave_num_; ++i) {
    this->sc_[i] = ecrt_master_slave_config(
        this->master_, this->slave_config_[i].alias,
        this->slave_config_[i].position, this->slave_config_[i].vendor_id,
        this->slave_config_[i].product_code);
    if (!this->sc_[i]) {
      LOG(INFO) << "Master Slave Config Error! System Exits!";
      return false;
    } else {
      LOG(INFO) << "Config Slave " << i;
    }
  }

  // config sdos
  // 配置sdo
  // 跳过IO从站
  for (int i = 0; i < this->slave_num_; i++) {
    if (this->slave_list_[i] != SlaveType::MOTOR_SERVO) {
      continue;
    }
    int val = 0;
    // 设置为csp模式、设置插补周期为1ms
    // int val = ecrt_slave_config_sdo8(this->sc_[i], 0x6060, 0, 8);
    if (val != 0) {
      LOG(INFO) << "Slave Config pdos Error! System Exits! "
                   "ecrt_slave_config_sdo8 [0x6060] return the value: "
                << val;
      return false;
    }
    // val = ecrt_slave_config_sdo8(this->sc_[i], 0x60C2, 1, 1);
    if (val != 0) {
      LOG(INFO) << "Slave Config pdos Error! System Exits! "
                   "ecrt_slave_config_sdo8 [0x60C2] return the value: "
                << val;
      return false;
    }
    // 设置紧急刹车
    // val = ecrt_slave_config_sdo32(this->sc_[i], 0x6084, 0, 0xEFFF4240);
    if (val != 0) {
      LOG(INFO) << "Slave Config pdos Error! System Exits! "
                   "ecrt_slave_config_sdo8 [0x6084] return the "
                   "value: "
                << val;
      return false;
    }
    // val = ecrt_slave_config_sdo32(this->sc_[i], 0x6085, 0, 0xEFFF4240);
    if (val != 0) {
      LOG(INFO) << "Slave Config pdos Error! System Exits! "
                   "ecrt_slave_config_sdo8 [0x6085] return the "
                   "value: "
                << val;
      return false;
    }
  }

  if (reference_slave_clock_) {
    // 调用ecrt_slave_config_dc() 同步从站的时钟
    // 需要在非实时上下文种调用
    for (int i = 0; i < this->slave_num_; ++i) {  // 开启DC
      if (this->slave_list_[i] == SlaveType::IO_SERVO ||
          this->slave_list_[i] == SlaveType::IO_SERVO_IN ||
          this->slave_list_[i] == SlaveType::IO_SERVO_OUT) {
        ecrt_slave_config_dc(this->sc_[i], 0, this->period_ns_,
                             this->period_ns_ / 10 * 3, 0, 0);
      } else {
        ecrt_slave_config_dc(this->sc_[i], 0x0300, this->period_ns_,
                             this->period_ns_ / 10 * 3, 0, 0);
        LOG(INFO) << "ecrt_slave_config_dc for << " << i
                  << " axis: " << this->period_ns_ << " ns";
      }
    }
  }

  // 调用ecrt_slave_config_pdos()
  for (int i = 0; i < this->slave_num_; ++i) {
    int val;
    switch (slave_list_[i]) {
    case SlaveType::MOTOR_SERVO:
      val = ecrt_slave_config_pdos(this->sc_[i], EC_END, slave_zero_syncs);
      break;
    case SlaveType::IO_SERVO_IN:
      val = ecrt_slave_config_pdos(this->sc_[i], EC_END, slave_io_in_syncs);
      break;
    case SlaveType::IO_SERVO_OUT:
      val = ecrt_slave_config_pdos(this->sc_[i], EC_END, slave_io_out_syncs);
      break;
    case SlaveType::IO_SERVO:
      val = ecrt_slave_config_pdos(this->sc_[i], EC_END, slave_io_syncs);
      break;
    default:
      break;
    }
    // val = ecrt_slave_config_pdos(this->sc_[i], EC_END, this->syncs_info_[i]);
    if (val != 0) {
      LOG(INFO) << "Slave Config pdos Error! System Exits! "
                   "ecrt_slave_config_pdos return the value: "
                << val;
      return false;
    }
  }

  return true;
}

uint64_t EthercatDevice::SystemTimeNs(void) {
  struct timespec time_spc;
  clock_gettime(CLOCK_MONOTONIC, &time_spc);
  RTIME time = TIMESPEC2NS(time_spc);

  if (system_time_base_ > time) {
    LOG(ERROR) << __func__
               << "() error: system_time_base greater than"
                  " system time (system_time_base: "
               << system_time_base_ << ", time: " << time;
    return time;
  } else {
    return time - system_time_base_;
  }
}

/**
 * @brief 设置参考从站时钟
 *
 * @return true
 * @return false
 */
bool EthercatDevice::SelectReferenceClock() {
  int ret = ecrt_master_select_reference_clock(this->master_, NULL);
  if (ret < 0) {
    LOG(ERROR) << "Failed to select reference clock: " << strerror(-ret);
    return false;
  } else {
    LOG(INFO) << "Successed to select reference clock!";
    return true;
  }
}

/**
 * @brief 通过输入从站PDO入口注册信息，为数据域注册PDO入口
 *
 * @return true
 * @return false
 */
// TODO(maqun): domain_regs数据的获取应该更规范一些
bool EthercatDevice::DomainRegisterList() {
  if (ecrt_domain_reg_pdo_entry_list(this->domain_, domain_regs) != 0) {
    LOG(INFO) << "Failed to register PDO entry, return False.";
    return false;
  } else {
    LOG(INFO) << "Success to config PDO entries.";
    for (int i = 0; i < this->slave_num_; i++) {
      LOG(INFO) << "Slave [" << i << "]: "
                << "opration mode: " << offset_[i].operation_mode
                << ", ctrl_word: " << offset_[i].ctrl_word
                << ", status_word: " << offset_[i].status_word;
    }
    return true;
  }
}

/**
 * @brief 激活主站
 *
 * @return true
 * @return false
 */
bool EthercatDevice::MasterActive() {
  if (!reference_slave_clock_) {
    // 调用ecrt_slave_config_dc() 同步从站的时钟
    // 需要在非实时上下文种调用
    for (int i = 0; i < this->slave_num_; ++i) {  // 开启DC
      if (this->slave_list_[i] != SlaveType::MOTOR_SERVO) {
        ecrt_slave_config_dc(this->sc_[i], 0, this->period_ns_, 0, 0, 0);
      } else {
        ecrt_slave_config_dc(this->sc_[i], 0x0300, this->period_ns_, 4400000, 0,
                             0);
        LOG(INFO) << "ecrt_slave_config_dc for << " << i
                  << " axis: " << this->period_ns_ << " ns";
      }
    }
    // TODO(liqingfengzzu@163.com): 校验
    if (ecrt_master_set_send_interval(this->master_, this->period_ns_ / 1000) !=
        0) {
      LOG(FATAL) << "Master set send interval error!";
      return false;
    }
  }

  if (ecrt_master_activate(this->master_) != 0) {
    LOG(FATAL) << "Master Active Error, System Exits!";
    return false;
  }
  return true;
}

/**
 * @brief 吊销主站
 * Deactivates the master.
 * Removes the bus configuration. All objects created by
 * ecrt_master_create_domain(), ecrt_master_slave_config(), ecrt_domain_data()
 * ecrt_slave_config_create_sdo_request() and
 * ecrt_slave_config_create_voe_handler() are freed, so pointers to them become
 * invalid.
 * This method should not be called in realtime context.
 */
void EthercatDevice::MasterDeactive() { ecrt_master_deactivate(this->master_); }

/**
 * @brief 数据域绑定，调用ecrt_domain_data()
 *
 * @return true
 * @return false
 */
bool EthercatDevice::DomainData() {
  if (!(this->domain_pd_ = ecrt_domain_data(this->domain_))) {
    LOG(FATAL) << "Domain Data Error, System Exits!";
    return false;
  } else {
    /* Set the initial master time and select a slave to use as the DC
     * reference clock, otherwise pass NULL to auto select the first capable
     * slave. Note: This can be used whether the master or the ref slave will
     * be used as the systems master DC clock.
     */
    dc_start_time_ns_ = SystemTimeNs();
    dc_time_ns_ = dc_start_time_ns_;

    return true;
  }
}

/**
 * @brief 检查从站的状态
 *
 */
void EthercatDevice::CheckSlaveConfigStates() {
  ec_slave_config_state_t s[kSLAVENUMMAX];
  for (int i = 0; i < this->slave_num_; ++i) {
    ecrt_slave_config_state(this->sc_[i], &s[i]);
    if (s[i].al_state != this->slave_config_state_[i].al_state) {
      // LOG(INFO) << "Slave " << i << " State " << s[i].al_state;
    }
    if (s[i].online != this->slave_config_state_[i].online) {
      // LOG(INFO) << "Slave " << i << ":"
      //          << (s[i].online ? " Online" : " Offline");
    }
    if (s[i].operational != this->slave_config_state_[i].operational) {
      // LOG(INFO) << "Slave " << i << ": " << (s[i].operational ? " " : "Not
      // ");
    }
    this->slave_config_state_[i] = s[i];
  }
}

/**
 * @brief 检查域状态
 *
 */
void EthercatDevice::CheckDomainState() {
  // TODO(liqingfengzzu@163.com): 未处理异常状态
  ec_domain_state_t ds;
  ecrt_domain_state(this->domain_, &ds);
  // 如果值存在变化，记录日志
  if (ds.working_counter != this->domain_state_.working_counter) {
    // LOG(INFO) << "Domain working counter: " << ds.working_counter;
  }
  if (ds.wc_state != this->domain_state_.wc_state) {
    // LOG(INFO) << "Domain State: " << ds.wc_state;
  }
  this->domain_state_ = ds;
}

/**
 * @brief 检查主站状态
 *
 */
void EthercatDevice::CheckMasterState() {
  // TODO(liqingfengzzu@163.com): 未处理异常状态
  ec_master_state_t ms;
  ecrt_master_state(this->master_, &ms);
  if (ms.slaves_responding != this->master_state_.slaves_responding) {
    // LOG(INFO) << ms.slaves_responding << " slave(s).";
  }
  if (ms.al_states != this->master_state_.al_states) {
    // LOG(INFO) << "Al States: " << ms.al_states;
  }
  if (ms.link_up != this->master_state_.link_up) {
    // LOG(INFO) << "Link is: " << (ms.link_up ? "up" : "down");
  }
  this->master_state_ = ms;
}

/**
 * @brief 检查主站、域以及从站信息，并更新记录
 *
 */
void EthercatDevice::CheckAndUpdateState() {
  static uint counter = 0;
  this->CheckDomainState();
  if (counter) {
    counter--;
  } else {
    counter = CHECK_DEVICE_FREQUENCY;

    // check for master & slave states
    this->CheckMasterState();
    if (!this->reference_slave_clock_) {
      this->CheckSlaveConfigStates();
    }
  }
}

/****************************************************************************/

/** Convert system time to RTAI time in counts (via the system_time_base).
 */
RTIME EthercatDevice::System2count(uint64_t time) {
  RTIME ret;

  if ((system_time_base_ < 0) && ((uint64_t)(-system_time_base_) > time)) {
    LOG(ERROR) << __func__
               << "() error: system_time_base less than system time "
                  "(system_time_base: "
               << system_time_base_ << ", time: " << time;
    ret = time;
  } else {
    ret = time + system_time_base_;
  }

  return ret;
}

void EthercatDevice::SetWaitUpTime(uint64_t time) {
  wakeup_time_ = time;
  LOG(INFO) << "set wait up time = " << time;
}

/**
 * Wait for the next period
 */
void EthercatDevice::WaitPeriod(void) {
  while (1) {
    RTIME wakeup_count = System2count(wakeup_time_);
    struct timespec wakeupTime;
    wakeupTime.tv_sec = wakeup_count / NSEC_PER_SEC;
    wakeupTime.tv_nsec = wakeup_count % NSEC_PER_SEC;

    struct timespec time_spc;
    clock_gettime(CLOCK_MONOTONIC, &time_spc);
    RTIME current_count = TIMESPEC2NS(time_spc);

    if ((wakeup_count < current_count) ||
        (wakeup_count > current_count + (50 * period_ns_))) {
      LOG(ERROR) << __func__ << "(): unexpected wake time! system_time_base = "
                 << system_time_base_ << " wakeup_count = " << wakeup_count
                 << " current_count = " << current_count;
    }

    int c_ret =
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeupTime, NULL);

    if (c_ret != 0) {  //避免过度睡眠
      if (c_ret == EINTR) {
        LOG(ERROR) << "Interrupted by signal handler";
        // continue;
      } else {
        LOG(ERROR) << "clock_nanosleep:errno=" << c_ret;
      }
    }
    // done if we got to here
    break;
  }

  // set master time in nano-seconds
  MasterApplicationTime(wakeup_time_);

  // calc next wake time (in sys time)
  wakeup_time_ += period_ns_;
}

/** Update the master time based on ref slaves time diff
 *
 * called after the ethercat frame is sent to avoid time jitter in
 * sync_distributed_clocks()
 */
void EthercatDevice::UpdateMasterClock(void) {
  // only update if primary master
  if (dc_started_) {
    int32_t normalise_dc_diff_ns = dc_diff_ns_;
    // normalise the time diff
    normalise_dc_diff_ns =
        ((normalise_dc_diff_ns + (period_ns_ / 2)) % period_ns_) -
        (period_ns_ / 2);

    if (dc_filter_idx_ >= DC_FILTER_CNT) {
      if (dc_filter_idx_flag_) {
        if (dc_diff_total_ns1_ > dc_diff_total_ns_) {
          dc_adjust_ns_++;
        }
        if (dc_diff_total_ns1_ < dc_diff_total_ns_) {
          dc_adjust_ns_--;
        }
      } else {
        if (normalise_dc_diff_total_ns_ > 0) {
          dc_adjust_ns_++;
        }
        if (normalise_dc_diff_total_ns_ < 0) {
          dc_adjust_ns_--;
        }
      }
      dc_filter_idx_flag_ = !dc_filter_idx_flag_;

      // reset
      normalise_dc_diff_total_ns_ = 0LL;
      dc_diff_total_ns1_ = 0LL;
      dc_diff_total_ns_ = 0LL;
      dc_filter_idx_ = 0;
    } else {
      if (dc_filter_idx_ < (DC_FILTER_CNT / 2)) {
        dc_diff_total_ns_ += dc_diff_ns_;
      } else {
        dc_diff_total_ns1_ += dc_diff_ns_;
      }

      normalise_dc_diff_total_ns_ += normalise_dc_diff_ns;
      dc_filter_idx_++;
    }

    // add cycles adjustment to time base (including a spot adjustment)
    system_time_base_ += dc_adjust_ns_;
    // if (update_master_clock_time_++ == 2000) {
    //  LOG(INFO) << "system_time_base_ =" << system_time_base_
    //            << " ; dc_diff_ns_ = " << dc_diff_ns_;
    //  update_master_clock_time_ = 0;
    // }
  } else {
    dc_started_ = true;
  }
}

/**
 * @brief 同步时钟
 *
 */
void EthercatDevice::SyncClocks() {
  if (!reference_slave_clock_) {
    static int counter = 0;
    // 每隔多少周期，同步一次clock
    if (counter == 0) {
      RTT::os::TimeService::nsecs time =
          RTT::os::TimeService::Instance()->getNSecs();
      ecrt_master_sync_reference_clock_to(this->master_, time);
      counter = SYNC_CLOCKS_FREQUENCY;
    } else {
      counter--;
    }
  } else {
    uint32_t ref_time = 0;
    uint64_t prev_app_time = dc_time_ns_;
    dc_time_ns_ = SystemTimeNs();
    if (dc_fail_sleep_time_ > 0) {
      dc_fail_sleep_time_--;
    }
    if (dc_started_ && dc_fail_sleep_time_ <= 0) {
      int ret = ecrt_master_reference_clock_time(master_, &ref_time);
      if (ret == 0) {
        dc_diff_ns_ = (uint32_t)prev_app_time - ref_time;
      } else {
        dc_diff_ns_ = 0;
        dc_fail_sleep_time_ = 100;
        LOG(ERROR) << "ecrt_master_reference_clock_time failed! ret = " << ret;
      }
    } else {
      dc_diff_ns_ = 0;
    }

    if (dc_started_) {
      dc_max_time_diff_ = ecrt_master_sync_monitor_process(master_);
    }
    ecrt_master_sync_monitor_queue(master_);
  }

  ecrt_master_sync_slave_clocks(this->master_);
}

/**
 * @brief 写应用时间到主站
 *
 * @param application_time 应用时间，目标周期
 */
void EthercatDevice::MasterApplicationTime(uint64_t application_time) {
  // 获取系统的纳秒，并同步时钟周期
  ecrt_master_application_time(this->master_, application_time);
}

/**
 * @brief 写入Ethercat控制帧内容
 *
 * @param frame 控制数据帧具体内容
 */
void EthercatDevice::WriteEthercatCtlFrame(const EthercatCtlFrame &frame) {
  // 写入数据
  for (int i = 0; i < this->slave_num_; ++i) {
    if (this->slave_list_[i] == SlaveType::IO_SERVO) {  // internal io
      uint8_t io_out = frame.digit_io_out >> 16;
      EC_WRITE_U8(this->domain_pd_ + this->offset_[i].digit_out, io_out);
    } else if (this->slave_list_[i] == SlaveType::IO_SERVO_IN) {
      continue;
    } else if (this->slave_list_[i] ==
               SlaveType::IO_SERVO_OUT) {  // external io
      uint8_t io_out = 0x00;
      io_out = frame.digit_io_out & 0xffff;
      EC_WRITE_U8(this->domain_pd_ + this->offset_[i].digit_out, io_out);
      io_out = (frame.digit_io_out >> 8) & 0xffff;
      EC_WRITE_U8(this->domain_pd_ + this->offset_[i].digit_out_8, io_out);
    } else if (this->slave_list_[i] == SlaveType::MOTOR_SERVO) {
      EC_WRITE_U16(this->domain_pd_ + this->offset_[i].ctrl_word,
                   frame.ctrl_word[i]);
      EC_WRITE_S32(this->domain_pd_ + this->offset_[i].target_position,
                   frame.target_position[i]);
      // EC_WRITE_S8(this->domain_pd_ + this->offset_[i].operation_mode,
      //             frame.operation_mode[i]);
    } else {
      continue;
    }
  }

  if (!reference_slave_clock_) {
    // 同步时钟，检查更新状态等。在写入帧之前，做一些预备工作
    // 同步分布式时钟
    this->SyncClocks();
    // 发送数据
    ecrt_domain_queue(this->domain_);
    ecrt_master_send(this->master_);
  } else {
    ecrt_domain_queue(this->domain_);
    // 同步时钟，检查更新状态等。在写入帧之前，做一些预备工作
    // 同步分布式时钟
    this->SyncClocks();
    // 发送数据
    ecrt_master_send(this->master_);
    UpdateMasterClock();
  }
}

/**
 * @brief 写入Ethercat控制帧内容
 */
void EthercatDevice::WriteEthercatCtlFrame() {
  if (!reference_slave_clock_) {
    // 同步时钟，检查更新状态等。在写入帧之前，做一些预备工作
    // 同步分布式时钟
    this->SyncClocks();
    // 发送数据
    ecrt_domain_queue(this->domain_);
    ecrt_master_send(this->master_);
  } else {
    ecrt_domain_queue(this->domain_);
    // 同步时钟，检查更新状态等。在写入帧之前，做一些预备工作
    // 同步分布式时钟
    this->SyncClocks();
    // 发送数据
    ecrt_master_send(this->master_);
    UpdateMasterClock();
  }
}

/**
 * @brief 读取状态帧
 *
 * @param status_frame 读取的状态帧内容
 */
void EthercatDevice::ReadEthercatStatusFrame(
    EthercatStatusFrame *status_frame) {
  // receive process data
  ecrt_master_receive(this->master_);
  ecrt_domain_process(this->domain_);
  this->CheckAndUpdateState();

  status_frame->al_state = this->master_state_.al_states;
  // 读从站数据，保存至readcache中
  for (int i = 0; i < this->slave_num_; ++i) {
    if (this->slave_list_[i] == SlaveType::IO_SERVO) {  // internal io
      uint8_t io_in = EC_READ_U8(this->domain_pd_ + this->offset_[i].digit_in);
      status_frame->digit_io_in &= 0x00FFFF;
      status_frame->digit_io_in |= (io_in << 16);
    } else if (this->slave_list_[i] == SlaveType::IO_SERVO_IN) {  // external io
      uint16_t io_in =
          EC_READ_U16(this->domain_pd_ + this->offset_[i].digit_in);
      status_frame->digit_io_in &= 0xFF0000;
      status_frame->digit_io_in |= io_in;
    } else if (this->slave_list_[i] == SlaveType::MOTOR_SERVO) {
      status_frame->ctrl_word[i] =
          EC_READ_U16(this->domain_pd_ + this->offset_[i].ctrl_word);

      status_frame->status_word[i] =
          EC_READ_U16(this->domain_pd_ + this->offset_[i].status_word);

      // status_frame->operation_mode[i] =
      //     EC_READ_S8(this->domain_pd_ + this->offset_[i].operation_mode);

      status_frame->current_position[i] =
          EC_READ_S32(this->domain_pd_ + this->offset_[i].current_position);

      status_frame->target_position[i] =
          EC_READ_S32(this->domain_pd_ + this->offset_[i].target_position);

      // status_frame->modes_operation_display[i] = EC_READ_S32(
      //     this->domain_pd_ + this->offset_[i].modes_operation_display);

      // status_frame->last_position[i] = EC_READ_S32(
      //     this->domain_pd_ + this->offset_[i].velocity_actual_value);

      // status_frame->current_torque[i] =
      //   EC_READ_S16(this->domain_pd_ + this->offset_[i].torque_actual_value);

      // status_frame->error_code[i] =
      //     EC_READ_U16(this->domain_pd_ + this->offset_[i].error_code);
    }
  }
}

}  // namespace rosc
