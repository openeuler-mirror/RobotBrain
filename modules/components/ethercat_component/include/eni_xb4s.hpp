/**
 * @file eni_xb4s.hpp
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief
 * 关于XB4s机型，相关的pdo等配置参数。该方法中的大部分代码，可以用过ethercat工具中的“ethercat
 * slave”命令获得
 * @version 0.1
 * @date 2021-11-17
 *
 * @copyright Copyright (c) 2021 ROSC
 *
 */
#ifndef MODULES_COMPONENTS_ETHERCAT_COMPONENT_INCLUDE_ENI_XB4S_HPP_
#define MODULES_COMPONENTS_ETHERCAT_COMPONENT_INCLUDE_ENI_XB4S_HPP_
#include <bits/stdint-uintn.h>
#include <ecrt.h>
/* Master 0, Slave 0, "SAFETY MODULE"
 * Vendor ID:       0x00000a39
 * Product code:    0x00000100
 * Revision number: 0x00000001
 */

ec_pdo_entry_info_t slave_0_pdo_entries[] = {
    {0x7000, 0x01, 8},  /* Digital_Inputs1 */
    {0x7010, 0x01, 16}, /* EXDO */
    {0x6000, 0x01, 16}, /* Digital_Outputs1 */
    {0x6010, 0x01, 16}, /* EXDI */
};

ec_pdo_info_t slave_0_pdos[] = {
    {0x1600, 1, slave_0_pdo_entries + 0}, /* Outputs */
    {0x1601, 1, slave_0_pdo_entries + 1}, /* Outputs */
    {0x1a00, 1, slave_0_pdo_entries + 2}, /* Inputs */
    {0x1a01, 1, slave_0_pdo_entries + 3}, /* Inputs */
};

ec_sync_info_t slave_0_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 2, slave_0_pdos + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 2, slave_0_pdos + 2, EC_WD_DISABLE},
    {0xff}};

/* Master 0, Slave 1, "CDHD"
 * Vendor ID:       0x000002e1
 * Product code:    0x000002ec
 * Revision number: 0x00029002
 */

ec_pdo_entry_info_t slave_1_pdo_entries[] = {
    {0x6040, 0x00, 16}, {0x6060, 0x00, 8},  {0x607a, 0x00, 32},
    {0x6081, 0x00, 32}, {0x60ff, 0x00, 32}, {0x6071, 0x00, 16},
    {0x60fe, 0x01, 32}, {0x60b2, 0x00, 16}, {0x6041, 0x00, 16},
    {0x6061, 0x00, 8},  {0x6077, 0x00, 16}, {0x6064, 0x00, 32},
    {0x6074, 0x00, 16}, {0x20f2, 0x00, 16}, {0x60fd, 0x00, 32},
    {0x20b6, 0x00, 32}, {0x60f4, 0x00, 32},
};

ec_pdo_info_t slave_1_pdos[] = {
    {0x1600, 2, slave_1_pdo_entries + 0},
    {0x1601, 2, slave_1_pdo_entries + 2},
    {0x1602, 1, slave_1_pdo_entries + 4},
    {0x1603, 3, slave_1_pdo_entries + 5},
    {0x1a00, 3, slave_1_pdo_entries + 8},
    {0x1a01, 1, slave_1_pdo_entries + 11},
    {0x1a02, 2, slave_1_pdo_entries + 12},
    {0x1a03, 3, slave_1_pdo_entries + 14},
};

ec_sync_info_t slave_1_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 4, slave_1_pdos + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 4, slave_1_pdos + 4, EC_WD_DISABLE},
    {0xff}};

/* Master 0, Slave 2, "CDHD"
 * Vendor ID:       0x000002e1
 * Product code:    0x000002ec
 * Revision number: 0x00029002
 */

ec_pdo_entry_info_t slave_2_pdo_entries[] = {
    {0x6040, 0x00, 16}, {0x6060, 0x00, 8},  {0x607a, 0x00, 32},
    {0x6081, 0x00, 32}, {0x60ff, 0x00, 32}, {0x6071, 0x00, 16},
    {0x60fe, 0x01, 32}, {0x60b2, 0x00, 16}, {0x6041, 0x00, 16},
    {0x6061, 0x00, 8},  {0x6077, 0x00, 16}, {0x6064, 0x00, 32},
    {0x6074, 0x00, 16}, {0x20f2, 0x00, 16}, {0x60fd, 0x00, 32},
    {0x20b6, 0x00, 32}, {0x60f4, 0x00, 32},
};

ec_pdo_info_t slave_2_pdos[] = {
    {0x1600, 2, slave_2_pdo_entries + 0},
    {0x1601, 2, slave_2_pdo_entries + 2},
    {0x1602, 1, slave_2_pdo_entries + 4},
    {0x1603, 3, slave_2_pdo_entries + 5},
    {0x1a00, 3, slave_2_pdo_entries + 8},
    {0x1a01, 1, slave_2_pdo_entries + 11},
    {0x1a02, 2, slave_2_pdo_entries + 12},
    {0x1a03, 3, slave_2_pdo_entries + 14},
};

ec_sync_info_t slave_2_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 4, slave_2_pdos + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 4, slave_2_pdos + 4, EC_WD_DISABLE},
    {0xff}};

/* Master 0, Slave 3, "CDHD"
 * Vendor ID:       0x000002e1
 * Product code:    0x000002ec
 * Revision number: 0x00029002
 */

ec_pdo_entry_info_t slave_3_pdo_entries[] = {
    {0x6040, 0x00, 16}, {0x6060, 0x00, 8},  {0x607a, 0x00, 32},
    {0x6081, 0x00, 32}, {0x60ff, 0x00, 32}, {0x6071, 0x00, 16},
    {0x60fe, 0x01, 32}, {0x60b2, 0x00, 16}, {0x6041, 0x00, 16},
    {0x6061, 0x00, 8},  {0x6077, 0x00, 16}, {0x6064, 0x00, 32},
    {0x6074, 0x00, 16}, {0x20f2, 0x00, 16}, {0x60fd, 0x00, 32},
    {0x20b6, 0x00, 32}, {0x60f4, 0x00, 32},
};

ec_pdo_info_t slave_3_pdos[] = {
    {0x1600, 2, slave_3_pdo_entries + 0},
    {0x1601, 2, slave_3_pdo_entries + 2},
    {0x1602, 1, slave_3_pdo_entries + 4},
    {0x1603, 3, slave_3_pdo_entries + 5},
    {0x1a00, 3, slave_3_pdo_entries + 8},
    {0x1a01, 1, slave_3_pdo_entries + 11},
    {0x1a02, 2, slave_3_pdo_entries + 12},
    {0x1a03, 3, slave_3_pdo_entries + 14},
};

ec_sync_info_t slave_3_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 4, slave_3_pdos + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 4, slave_3_pdos + 4, EC_WD_DISABLE},
    {0xff}};

/* Master 0, Slave 4, "CDHD"
 * Vendor ID:       0x000002e1
 * Product code:    0x000002ec
 * Revision number: 0x00029002
 */

ec_pdo_entry_info_t slave_4_pdo_entries[] = {
    {0x6040, 0x00, 16}, {0x6060, 0x00, 8},  {0x607a, 0x00, 32},
    {0x6081, 0x00, 32}, {0x60ff, 0x00, 32}, {0x6071, 0x00, 16},
    {0x60fe, 0x01, 32}, {0x60b2, 0x00, 16}, {0x6041, 0x00, 16},
    {0x6061, 0x00, 8},  {0x6077, 0x00, 16}, {0x6064, 0x00, 32},
    {0x6074, 0x00, 16}, {0x20f2, 0x00, 16}, {0x60fd, 0x00, 32},
    {0x20b6, 0x00, 32}, {0x60f4, 0x00, 32},
};

ec_pdo_info_t slave_4_pdos[] = {
    {0x1600, 2, slave_4_pdo_entries + 0},
    {0x1601, 2, slave_4_pdo_entries + 2},
    {0x1602, 1, slave_4_pdo_entries + 4},
    {0x1603, 3, slave_4_pdo_entries + 5},
    {0x1a00, 3, slave_4_pdo_entries + 8},
    {0x1a01, 1, slave_4_pdo_entries + 11},
    {0x1a02, 2, slave_4_pdo_entries + 12},
    {0x1a03, 3, slave_4_pdo_entries + 14},
};

ec_sync_info_t slave_4_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 4, slave_4_pdos + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 4, slave_4_pdos + 4, EC_WD_DISABLE},
    {0xff}};

/* Master 0, Slave 5, "CDHD"
 * Vendor ID:       0x000002e1
 * Product code:    0x000002ec
 * Revision number: 0x00029002
 */

ec_pdo_entry_info_t slave_5_pdo_entries[] = {
    {0x6040, 0x00, 16}, {0x6060, 0x00, 8},  {0x607a, 0x00, 32},
    {0x6081, 0x00, 32}, {0x60ff, 0x00, 32}, {0x6071, 0x00, 16},
    {0x60fe, 0x01, 32}, {0x60b2, 0x00, 16}, {0x6041, 0x00, 16},
    {0x6061, 0x00, 8},  {0x6077, 0x00, 16}, {0x6064, 0x00, 32},
    {0x6074, 0x00, 16}, {0x20f2, 0x00, 16}, {0x60fd, 0x00, 32},
    {0x20b6, 0x00, 32}, {0x60f4, 0x00, 32},
};

ec_pdo_info_t slave_5_pdos[] = {
    {0x1600, 2, slave_5_pdo_entries + 0},
    {0x1601, 2, slave_5_pdo_entries + 2},
    {0x1602, 1, slave_5_pdo_entries + 4},
    {0x1603, 3, slave_5_pdo_entries + 5},
    {0x1a00, 3, slave_5_pdo_entries + 8},
    {0x1a01, 1, slave_5_pdo_entries + 11},
    {0x1a02, 2, slave_5_pdo_entries + 12},
    {0x1a03, 3, slave_5_pdo_entries + 14},
};

ec_sync_info_t slave_5_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 4, slave_5_pdos + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 4, slave_5_pdos + 4, EC_WD_DISABLE},
    {0xff}};

/* Master 0, Slave 6, "CDHD"
 * Vendor ID:       0x000002e1
 * Product code:    0x000002ec
 * Revision number: 0x00029002
 */

ec_pdo_entry_info_t slave_6_pdo_entries[] = {
    {0x6040, 0x00, 16}, {0x6060, 0x00, 8},  {0x607a, 0x00, 32},
    {0x6081, 0x00, 32}, {0x60ff, 0x00, 32}, {0x6071, 0x00, 16},
    {0x60fe, 0x01, 32}, {0x60b2, 0x00, 16}, {0x6041, 0x00, 16},
    {0x6061, 0x00, 8},  {0x6077, 0x00, 16}, {0x6064, 0x00, 32},
    {0x6074, 0x00, 16}, {0x20f2, 0x00, 16}, {0x60fd, 0x00, 32},
    {0x20b6, 0x00, 32}, {0x60f4, 0x00, 32},
};

ec_pdo_info_t slave_6_pdos[] = {
    {0x1600, 2, slave_6_pdo_entries + 0},
    {0x1601, 2, slave_6_pdo_entries + 2},
    {0x1602, 1, slave_6_pdo_entries + 4},
    {0x1603, 3, slave_6_pdo_entries + 5},
    {0x1a00, 3, slave_6_pdo_entries + 8},
    {0x1a01, 1, slave_6_pdo_entries + 11},
    {0x1a02, 2, slave_6_pdo_entries + 12},
    {0x1a03, 3, slave_6_pdo_entries + 14},
};

ec_sync_info_t slave_6_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 4, slave_6_pdos + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 4, slave_6_pdos + 4, EC_WD_DISABLE},
    {0xff}};

// TODO(liqingfengzzu@163.com):
// 此处不同从站，需要修改的地方。目前该功能不是从配置文件中读取的。
// 上面的信息，可以直接用
ec_sync_info_t *slave_syncs_all[] = {slave_1_syncs, slave_2_syncs,
                                     slave_3_syncs, slave_4_syncs,
                                     slave_5_syncs, slave_6_syncs};
uint16_t driver_alias[] = {0, 0, 0, 0, 0, 0};      // 别名，对应master
uint16_t driver_positions[] = {1, 2, 3, 4, 5, 6};  // 总线位置
uint32_t driver_vendor_ids[] = {0x000002e1, 0x000002e1, 0x000002e1, 0x000002e1,
                                0x000002e1, 0x000002e1};  // vendor code
uint32_t driver_product_codes[] = {0x000002ec, 0x000002ec,
                                   0x000002ec, 0x000002ec,
                                   0x000002ec, 0x000002ec};  // product code

#endif  // MODULES_COMPONENTS_ETHERCAT_COMPONENT_INCLUDE_ENI_XB4S_HPP_
