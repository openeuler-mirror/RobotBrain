/********************************************************************
 * @file eni_slaves.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-08-02
 *
 * @copyright Copyright (c) 2023 ROSC
 *
 ********************************************************************/
#ifndef MODULES_COMPONENTS_ETHERCAT_COMPONENT_INCLUDE_ENI_SLAVES_HPP_
#define MODULES_COMPONENTS_ETHERCAT_COMPONENT_INCLUDE_ENI_SLAVES_HPP_
#include <bits/stdint-uintn.h>
#include <ecrt.h>

namespace rosc {
/* Master 0, Slave 5, "EL1809"
 * Vendor ID:       0x00000002
 * Product code:    0x07113052
 * Revision number: 0x00120000
 */

ec_pdo_entry_info_t slave_io_in_pdo_entries[] = {
    {0x6000, 0x01, 1}, /* Input */
    {0x6010, 0x01, 1}, /* Input */
    {0x6020, 0x01, 1}, /* Input */
    {0x6030, 0x01, 1}, /* Input */
    {0x6040, 0x01, 1}, /* Input */
    {0x6050, 0x01, 1}, /* Input */
    {0x6060, 0x01, 1}, /* Input */
    {0x6070, 0x01, 1}, /* Input */
    {0x6080, 0x01, 1}, /* Input */
    {0x6090, 0x01, 1}, /* Input */
    {0x60a0, 0x01, 1}, /* Input */
    {0x60b0, 0x01, 1}, /* Input */
    {0x60c0, 0x01, 1}, /* Input */
    {0x60d0, 0x01, 1}, /* Input */
    {0x60e0, 0x01, 1}, /* Input */
    {0x60f0, 0x01, 1}, /* Input */
};

ec_pdo_info_t slave_io_in_pdos[] = {
    {0x1a00, 1, slave_io_in_pdo_entries + 0},  /* Channel 1 */
    {0x1a01, 1, slave_io_in_pdo_entries + 1},  /* Channel 2 */
    {0x1a02, 1, slave_io_in_pdo_entries + 2},  /* Channel 3 */
    {0x1a03, 1, slave_io_in_pdo_entries + 3},  /* Channel 4 */
    {0x1a04, 1, slave_io_in_pdo_entries + 4},  /* Channel 5 */
    {0x1a05, 1, slave_io_in_pdo_entries + 5},  /* Channel 6 */
    {0x1a06, 1, slave_io_in_pdo_entries + 6},  /* Channel 7 */
    {0x1a07, 1, slave_io_in_pdo_entries + 7},  /* Channel 8 */
    {0x1a08, 1, slave_io_in_pdo_entries + 8},  /* Channel 9 */
    {0x1a09, 1, slave_io_in_pdo_entries + 9},  /* Channel 10 */
    {0x1a0a, 1, slave_io_in_pdo_entries + 10}, /* Channel 11 */
    {0x1a0b, 1, slave_io_in_pdo_entries + 11}, /* Channel 12 */
    {0x1a0c, 1, slave_io_in_pdo_entries + 12}, /* Channel 13 */
    {0x1a0d, 1, slave_io_in_pdo_entries + 13}, /* Channel 14 */
    {0x1a0e, 1, slave_io_in_pdo_entries + 14}, /* Channel 15 */
    {0x1a0f, 1, slave_io_in_pdo_entries + 15}, /* Channel 16 */
};

ec_sync_info_t slave_io_in_syncs[] = {
    {0, EC_DIR_INPUT, 16, slave_io_in_pdos + 0, EC_WD_DISABLE}, {0xff}};

/* Master 0, Slave 6, "EL2809"
 * Vendor ID:       0x00000002
 * Product code:    0x0af93052
 * Revision number: 0x00120000
 */

ec_pdo_entry_info_t slave_io_out_pdo_entries[] = {
    {0x7000, 0x01, 1}, /* Output */
    {0x7010, 0x01, 1}, /* Output */
    {0x7020, 0x01, 1}, /* Output */
    {0x7030, 0x01, 1}, /* Output */
    {0x7040, 0x01, 1}, /* Output */
    {0x7050, 0x01, 1}, /* Output */
    {0x7060, 0x01, 1}, /* Output */
    {0x7070, 0x01, 1}, /* Output */
    {0x7080, 0x01, 1}, /* Output */
    {0x7090, 0x01, 1}, /* Output */
    {0x70a0, 0x01, 1}, /* Output */
    {0x70b0, 0x01, 1}, /* Output */
    {0x70c0, 0x01, 1}, /* Output */
    {0x70d0, 0x01, 1}, /* Output */
    {0x70e0, 0x01, 1}, /* Output */
    {0x70f0, 0x01, 1}, /* Output */
};

ec_pdo_info_t slave_io_out_pdos[] = {
    {0x1600, 1, slave_io_out_pdo_entries + 0},  /* Channel 1 */
    {0x1601, 1, slave_io_out_pdo_entries + 1},  /* Channel 2 */
    {0x1602, 1, slave_io_out_pdo_entries + 2},  /* Channel 3 */
    {0x1603, 1, slave_io_out_pdo_entries + 3},  /* Channel 4 */
    {0x1604, 1, slave_io_out_pdo_entries + 4},  /* Channel 5 */
    {0x1605, 1, slave_io_out_pdo_entries + 5},  /* Channel 6 */
    {0x1606, 1, slave_io_out_pdo_entries + 6},  /* Channel 7 */
    {0x1607, 1, slave_io_out_pdo_entries + 7},  /* Channel 8 */
    {0x1608, 1, slave_io_out_pdo_entries + 8},  /* Channel 9 */
    {0x1609, 1, slave_io_out_pdo_entries + 9},  /* Channel 10 */
    {0x160a, 1, slave_io_out_pdo_entries + 10}, /* Channel 11 */
    {0x160b, 1, slave_io_out_pdo_entries + 11}, /* Channel 12 */
    {0x160c, 1, slave_io_out_pdo_entries + 12}, /* Channel 13 */
    {0x160d, 1, slave_io_out_pdo_entries + 13}, /* Channel 14 */
    {0x160e, 1, slave_io_out_pdo_entries + 14}, /* Channel 15 */
    {0x160f, 1, slave_io_out_pdo_entries + 15}, /* Channel 16 */
};

ec_sync_info_t slave_io_out_syncs[] = {
    {0, EC_DIR_OUTPUT, 8, slave_io_out_pdos + 0, EC_WD_ENABLE},
    {1, EC_DIR_OUTPUT, 8, slave_io_out_pdos + 8, EC_WD_ENABLE},
    {0xff}};

/* Master 0, Slave 8, "EL1859"
 * Vendor ID:       0x00000002
 * Product code:    0x07433052
 * Revision number: 0x00120000
 */

ec_pdo_entry_info_t slave_io_pdo_entries[] = {
    {0x7080, 0x01, 1}, /* Output */
    {0x7090, 0x01, 1}, /* Output */
    {0x70a0, 0x01, 1}, /* Output */
    {0x70b0, 0x01, 1}, /* Output */
    {0x70c0, 0x01, 1}, /* Output */
    {0x70d0, 0x01, 1}, /* Output */
    {0x70e0, 0x01, 1}, /* Output */
    {0x70f0, 0x01, 1}, /* Output */
    {0x6000, 0x01, 1}, /* Input */
    {0x6010, 0x01, 1}, /* Input */
    {0x6020, 0x01, 1}, /* Input */
    {0x6030, 0x01, 1}, /* Input */
    {0x6040, 0x01, 1}, /* Input */
    {0x6050, 0x01, 1}, /* Input */
    {0x6060, 0x01, 1}, /* Input */
    {0x6070, 0x01, 1}, /* Input */
};

ec_pdo_info_t slave_io_pdos[] = {
    {0x1608, 1, slave_io_pdo_entries + 0},  /* Channel 9 */
    {0x1609, 1, slave_io_pdo_entries + 1},  /* Channel 10 */
    {0x160a, 1, slave_io_pdo_entries + 2},  /* Channel 11 */
    {0x160b, 1, slave_io_pdo_entries + 3},  /* Channel 12 */
    {0x160c, 1, slave_io_pdo_entries + 4},  /* Channel 13 */
    {0x160d, 1, slave_io_pdo_entries + 5},  /* Channel 14 */
    {0x160e, 1, slave_io_pdo_entries + 6},  /* Channel 15 */
    {0x160f, 1, slave_io_pdo_entries + 7},  /* Channel 16 */
    {0x1a00, 1, slave_io_pdo_entries + 8},  /* Channel 1 */
    {0x1a01, 1, slave_io_pdo_entries + 9},  /* Channel 2 */
    {0x1a02, 1, slave_io_pdo_entries + 10}, /* Channel 3 */
    {0x1a03, 1, slave_io_pdo_entries + 11}, /* Channel 4 */
    {0x1a04, 1, slave_io_pdo_entries + 12}, /* Channel 5 */
    {0x1a05, 1, slave_io_pdo_entries + 13}, /* Channel 6 */
    {0x1a06, 1, slave_io_pdo_entries + 14}, /* Channel 7 */
    {0x1a07, 1, slave_io_pdo_entries + 15}, /* Channel 8 */
};

ec_sync_info_t slave_io_syncs[] = {
    {0, EC_DIR_OUTPUT, 8, slave_io_pdos + 0, EC_WD_ENABLE},
    {1, EC_DIR_INPUT, 8, slave_io_pdos + 8, EC_WD_DISABLE},
    {0xff}};
// 安川电机
ec_pdo_entry_info_t pdo_entries[] = {
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
    // RxPdo
    // {0x1600, 4, pdo_entries + 0},
    {0x1601, 7, pdo_entries + 0},
    // TxPdo
    // {0x1a00, 8, pdo_entries + 4}
    {0x1a01, 9, pdo_entries + 7}};

ec_sync_info_t syncs[] = {{0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
                          {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
                          {2, EC_DIR_OUTPUT, 1, pdos + 0, EC_WD_ENABLE},
                          {3, EC_DIR_INPUT, 1, pdos + 1, EC_WD_DISABLE},
                          {0xFF}};

/* Master 0, Slave 0, "ZeroErr Driver"
 * Vendor ID:       0x5a65726f
 * Product code:    0x00029252
 * Revision number: 0x00000001
 */

ec_pdo_entry_info_t slave_zero_pdo_entries[] = {
    {0x607a, 0x00, 32}, /* Target Position */
    {0x60fe, 0x00, 32}, /* Digital outputs */
    {0x6040, 0x00, 16}, /* Control Word */
    {0x6064, 0x00, 32}, /* Position Actual Value */
    {0x60fd, 0x00, 32}, /* Digital inputs */
    {0x6041, 0x00, 16}, /* Status Word */
};

ec_pdo_info_t slave_zero_pdos[] = {
    {0x1600, 3, slave_zero_pdo_entries + 0}, /* R0PDO */
    {0x1a00, 3, slave_zero_pdo_entries + 3}, /* T0PDO */
};

ec_sync_info_t slave_zero_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_zero_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_zero_pdos + 1, EC_WD_DISABLE},
    {0xff}};
uint32_t vendor_id_zero = 0x5a65726f;
uint32_t product_code_zero = 0x00029252;

uint32_t vendor_id_motor = 0x0000066f;
uint32_t vendor_id_io = 0x00000002;

uint32_t product_code_z = 0x60380005;
uint32_t product_code_axis = 0x60380004;
uint32_t product_code_in = 0x07113052;
uint32_t product_code_out = 0x0af93052;
uint32_t product_code_io = 0x07433052;

int motor_pdo_num = 4;
int io_pdo_num = 0;

}  // namespace rosc
#endif  // MODULES_COMPONENTS_ETHERCAT_COMPONENT_INCLUDE_ENI_SLAVES_HPP_
