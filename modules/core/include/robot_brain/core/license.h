/**
 * @file license.cc
 * @author hongxiaoxiao
 * @brief
 * @version 0.1
 * @date 2023-5-23
 *
 * @copyright Copyright (c) 2023 ROSC
 *
 */
#ifndef MODULES_CORE_INCLUDE_ROBOT_BRAIN_CORE_LICENSE_H_
#define MODULES_CORE_INCLUDE_ROBOT_BRAIN_CORE_LICENSE_H_

#include <cstddef>
#include <string>
#include <vector>

#define LICENSE_MAX_LEN 1024
#define HARDWARE_INFO_LEN 200
#define MAC_INFO_LEN 32

#define WORD(x) (u_int16_t)(*(const u_int16_t *)(x))
#define DWORD(x) (u_int32_t)(*(const u_int32_t *)(x))
#define QWORD(x) (*(const u64 *)(x))

namespace rosc {
struct dmi_header {
  u_int8_t type;
  u_int8_t length;
  u_int16_t handle;
  u_int8_t *data;
};

class License {
 public:
  License();
  ~License();

  bool checkLicense(const char *path);
  bool checkLicense();

 private:
  bool isMatchHardward(const char *hardware_license, u_int64_t file_size);
  int getAllHardwareInfo(char (*hardwareInfo)[HARDWARE_INFO_LEN]);
  int getCpuid(char *cpuid, int cpuid_len);
  unsigned char *memChunk(size_t base, size_t len, const char *devmem);
  int myread(int fd, unsigned char *buf, size_t count, const char *prefix);
  int getAllMac(char (*mac)[MAC_INFO_LEN]);
  char *getMacName(const char *line, int line_len, char *name, int name_len);
  int checksum(const u_int8_t *buf, size_t len);
  void toDmiHeader(struct dmi_header *h, u_int8_t *data);
  void createHashSHA256(const char *data, size_t size, unsigned char *hash);
  std::vector<char> EncryptByPrikeyString(unsigned char *message,
                                          size_t messageLength,
                                          const char *prikey);
  std::string DecryptByPubkeyString(const char *cipher, uint32_t len,
                                    const char *pubkey);
  const char *publicKey =
      "-----BEGIN PUBLIC "
      "KEY-----"
      "\nMIGfMA0GCSqGSIb3DQEBAQUAA4GNADCBiQKBgQDEdObmN4pj6OtR9ftlhEakqJh7\nSBei"
      "Pt"
      "0qZb6+OHxN6/"
      "27C1kzfeiAGYY7tDQytpsRptt+4cnpUuoIMDqPZeeaX1M8\ndnle82jNi+"
      "QM99tnEJ73QVzKZULDcKgeGCkzyZPtuYej2MQoQNJq1qeCCXCE18ZN\ncnAAoX32yb3Jh3DX"
      "Xw"
      "IDAQAB\n-----END PUBLIC KEY-----";
};
}  // namespace rosc

#endif  // MODULES_CORE_INCLUDE_ROBOT_BRAIN_CORE_LICENSE_H_
