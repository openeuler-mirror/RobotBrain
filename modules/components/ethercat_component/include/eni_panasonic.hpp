/**
 * @file eni_panasonic.hpp
 * @author QingfengLi (liqingfengzzu@163.com)
 * @brief 关于松下试验箱的配置参数
 * @version 0.1
 * @date 2021-11-11
 *
 * @copyright Copyright (c) 2021 ROSC
 *
 */
#ifndef MODULES_COMPONENTS_ETHERCAT_COMPONENT_INCLUDE_ENI_PANASONIC_HPP_
#define MODULES_COMPONENTS_ETHERCAT_COMPONENT_INCLUDE_ENI_PANASONIC_HPP_
#include <bits/stdint-uintn.h>
#include <ecrt.h>

namespace rosc {

/* Master 0, Slave 0
 * Vendor ID:       0x00000a19
 * Product code:    0x00000f3d
 * Revision number: 0x00000001
 */
ec_pdo_entry_info_t slave_io_pdo_entries[] = {
    {0x7000, 0x01, 1},  /* OUT_GEN_INT2 */
    {0x7000, 0x02, 1},  /* OUT_GEN_Bit1 */
    {0x7000, 0x03, 1},  /* OUT_GEN_Bit2 */
    {0x7000, 0x04, 1},  /* OUT_GEN_Bit3 */
    {0x7000, 0x05, 1},  /* OUT_GEN_Bit4 */
    {0x7000, 0x06, 1},  /* OUT_GEN_Bit5 */
    {0x7000, 0x07, 1},  /* OUT_GEN_Bit6 */
    {0x7000, 0x08, 1},  /* OUT_GEN_Bit7 */
    {0x7000, 0x09, 1},  /* OUT_GEN_Bit8 */
    {0x7000, 0x0a, 1},  /* OUT_GEN_Bit9 */
    {0x7000, 0x0b, 1},  /* OUT_GEN_Bit10 */
    {0x7000, 0x0c, 1},  /* OUT_GEN_Bit11 */
    {0x7000, 0x0d, 1},  /* OUT_GEN_Bit12 */
    {0x7000, 0x0e, 1},  /* OUT_GEN_Bit13 */
    {0x7000, 0x0f, 1},  /* OUT_GEN_Bit14 */
    {0x7000, 0x10, 1},  /* OUT_GEN_Bit15 */
    {0x7000, 0x11, 16}, /* OUT_GEN_Bit16 */
    {0x7000, 0x12, 16}, /* SubIndex 018 */
    {0x7000, 0x13, 32}, /* SubIndex 019 */
    {0x7100, 0x00, 32}, /* SubIndex 000 */
    {0x7101, 0x00, 32}, /* SubIndex 000 */
    {0x7102, 0x00, 16}, /* SubIndex 000 */
    {0x7103, 0x00, 16}, /* SubIndex 000 */
    {0x7104, 0x00, 32}, /* SubIndex 000 */
    {0x6000, 0x01, 1},  /* IN_GEN_Bit1 */
    {0x6000, 0x02, 1},  /* IN_GEN_Bit2 */
    {0x6000, 0x03, 1},  /* IN_GEN_Bit3 */
    {0x6000, 0x04, 1},  /* IN_GEN_Bit4 */
    {0x6000, 0x05, 1},  /* IN_GEN_Bit5 */
    {0x6000, 0x06, 1},  /* IN_GEN_Bit6 */
    {0x6000, 0x07, 1},  /* IN_GEN_Bit7 */
    {0x6000, 0x08, 1},  /* IN_GEN_Bit8 */
    {0x6000, 0x09, 1},  /* IN_GEN_Bit9 */
    {0x6000, 0x0a, 1},  /* IN_GEN_Bit10 */
    {0x6000, 0x0b, 1},  /* IN_GEN_Bit11 */
    {0x6000, 0x0c, 1},  /* IN_GEN_Bit12 */
    {0x6000, 0x0d, 1},  /* IN_GEN_Bit13 */
    {0x6000, 0x0e, 1},  /* IN_GEN_Bit14 */
    {0x6000, 0x0f, 1},  /* IN_GEN_Bit15 */
    {0x6000, 0x10, 1},  /* IN_GEN_Bit16 */
    {0x6000, 0x11, 32}, /* COUNT_IN_1 */
    {0x6000, 0x12, 32}, /* COUNT_IN_2 */
    {0x6100, 0x00, 32}, {0x6101, 0x00, 32}, {0x6102, 0x00, 16},
    {0x6103, 0x00, 16}, {0x6104, 0x00, 32},
};

ec_pdo_info_t slave_io_pdos[] = {
    {0x1600, 24,
     slave_io_pdo_entries + 0}, /* OUT_GENERIC process data mapping */
    {0x1a00, 23,
     slave_io_pdo_entries + 24}, /* IN_GENERIC process data mapping */
};

ec_sync_info_t slave_io_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_io_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_io_pdos + 1, EC_WD_DISABLE},
    {0xff}};

ec_pdo_entry_info_t pdo_entries[] = {
    /*** PDO映射1 ***/
    // {0x6040, 0x00, 16}, /* Controlword */
    // {0x6060, 0x00, 8},  /* Modes of operation */
    // {0x607a, 0x00, 32}, /* Target position */
    // {0x60b8, 0x00, 16}, /* Touch probe function */
    // {0x603f, 0x00, 16}, /* Error code */
    // {0x6041, 0x00, 16}, /* Statusword */
    // {0x6061, 0x00, 8},  /* Modes of operation display */
    // {0x6064, 0x00, 32}, /* Position actual value */
    // {0x60b9, 0x00, 16}, /* Touch probe status */
    // {0x60ba, 0x00, 32}, /* Touch probe pos1 pos value */
    // {0x60f4, 0x00, 32}, /* Following error actual value */
    // {0x60fd, 0x00, 32}, /* Digital inputs */
    /*** PDO映射2 ***/
    {0x6040, 0x00, 16}, /* Controlword */
    {0x6060, 0x00, 8},  /* Modes of operation */
    {0x6071, 0x00, 16}, /* Target Torque */
    {0x607A, 0x00, 32}, /* Target Position */
    {0x6080, 0x00, 32}, /* Max motor speed */
    {0x60B8, 0x00, 16}, /* Touch probe function */
    {0x60FF, 0x00, 32}, /* Target Velocity */
    {0x603F, 0x00, 16}, /* Error code */
    {0x6041, 0x00, 16}, /* Statusword */
    {0x6061, 0x00, 8},  /* Modes of operation display */
    {0x6064, 0x00, 32}, /* Position actual value */
    {0x606C, 0x00, 32}, /* Velocity actual value */
    {0x6077, 0x00, 16}, /* Torque actual value */
    {0x60B9, 0x00, 16}, /* Touch probe status */
    {0x60BA, 0x00, 32}, /* Touch probe pos1 pos value */
    {0x60FD, 0x00, 32}  /* Digital inputs */
};
ec_pdo_info_t pdos[] = {
    // // RxPdo
    // {0x1600, 4, pdo_entries + 0},
    // // TxPdo
    // {0x1a00, 8, pdo_entries + 4}

    {0x1601, 7, pdo_entries + 0},
    // TxPdo
    // {0x1a00, 8, pdo_entries + 4}
    {0x1a01, 9, pdo_entries + 7}};

ec_sync_info_t syncs[] = {{0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
                          {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
                          {2, EC_DIR_OUTPUT, 1, pdos + 0, EC_WD_ENABLE},
                          {3, EC_DIR_INPUT, 1, pdos + 1, EC_WD_DISABLE},
                          {0xFF}};

ec_sync_info_t *slave_syncs_all[] = {syncs, syncs, slave_io_syncs};
uint16_t driver_alias[] = {0, 0, 0};      // 别名，对应master
uint16_t driver_positions[] = {0, 1, 2};  // 总线位置
uint32_t driver_vendor_ids[] = {0x0000066f, 0x0000066f,
                                0x00000a19};  // vendor code
uint32_t driver_product_codes[] = {0x515050a1, 0x515050a1,
                                   0x00000f3d};  // product code

}  // namespace rosc
#endif  // MODULES_COMPONENTS_ETHERCAT_COMPONENT_INCLUDE_ENI_PANASONIC_HPP_
