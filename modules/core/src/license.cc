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
#include <cstddef>

#include <robot_brain/core/license.h>
#include <robot_brain/core/application.h>
#include <robot_brain/core/context.h>
#include <glog/logging.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <netinet/in.h>
#include <net/if.h>
// #include <robot_brain/core.hpp>

#ifdef USE_MMAP
#include <sys/mman.h>
#ifndef MAP_FAILED
#define MAP_FAILED ((void *)-1)
#endif /* !MAP_FAILED */
#endif /* USE MMAP */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <openssl/rsa.h>
#include <openssl/pem.h>
#include <iostream>
#include <fstream>

namespace rosc {
/**
 * @brief 构造一个对象
 */
License::License() {}

/**
 * @brief 析构一个对象
 *
 */
License::~License() {}

bool License::checkLicense(const char *path) {
  std::fstream fileIn(path, std::ios::in | std::ios::binary);
  if (!fileIn) {
    LOG(ERROR) << "Error opening fileIn";
    char hardwareInfo[16][HARDWARE_INFO_LEN];
    int num = getAllHardwareInfo(hardwareInfo);
    if (num <= 0) {
      return false;
    }
    LOG(ERROR) << "hardware info:\"" << hardwareInfo[0] << "\"";
    return false;
  }
  char sizeCharRead[10];
  fileIn.read(sizeCharRead, sizeof(size_t));
  size_t licenseLen = 0;
  for (size_t i = 0; i < sizeof(size_t); i++) {
    licenseLen |= (sizeCharRead[i] << (i * 8)) & 0xFF;
  }

  char *licenseFile = new char[licenseLen];
  fileIn.read(licenseFile, licenseLen);
  bool isMatch = isMatchHardward(licenseFile, licenseLen);
  delete[] licenseFile;
  return isMatch;
}

bool License::checkLicense() {
  auto config = *rosc::Application::GetContext()->GetConfig();
  std::string licensePath =
      config["init_global_config"]["license_path"].as<std::string>();
  return checkLicense(licensePath.c_str());
}

bool License::isMatchHardward(const char *hardwareLicense,
                              u_int64_t licenseSize) {
  char hardwareInfo[16][HARDWARE_INFO_LEN];
  int num = getAllHardwareInfo(hardwareInfo);
  if (num <= 0) {
    return 0;
  }
  int i = 0;
  unsigned char license[SHA256_DIGEST_LENGTH];
  std::string decryptedHardwareLicense =
      DecryptByPubkeyString(hardwareLicense, licenseSize, publicKey);

  for (i = 0; i < num; i++) {
    memset(license, 0, sizeof(license));
    createHashSHA256(hardwareInfo[i], HARDWARE_INFO_LEN, license);

    if (0 == memcmp(license, decryptedHardwareLicense.c_str(),
                    SHA256_DIGEST_LENGTH)) {
      return true;
    }
  }
  LOG(ERROR) << "hardware info:\"" << hardwareInfo[0] << "\"";
  return false;
}

int License::getAllHardwareInfo(char (*hardwareInfo)[HARDWARE_INFO_LEN]) {
  char cpuid[50];
  memset(cpuid, 0, sizeof(cpuid));
  char mac[16][32];
  memset(mac, 0, sizeof(mac));
  if (1 != getCpuid(cpuid, 50)) {
    snprintf(cpuid, sizeof(cpuid), "%s", "0000000000000000");
    // return -1;
  }
  int num = getAllMac(mac);
  if (0 >= num) {
    return -2;
  }

  int i = 0;
  for (i = 0; i < num; i++) {
    memset(hardwareInfo[i], 0, HARDWARE_INFO_LEN * sizeof(char));
    strcpy(hardwareInfo[i], cpuid);
    strcat(hardwareInfo[i], mac[i]);
  }
  return num;
}

int License::getCpuid(char *cpuid, int cpuid_len) {
  char devmem[10] = "/dev/mem";
  u_int16_t len;
  u_int16_t num;
  size_t fp;
  u_int8_t *buf = NULL, *nbuf = NULL, *data, *p;
  int i = 0;
  if ((buf = memChunk(0xF0000, 0x10000, devmem)) == NULL) {
    free(buf);
    return 0;
  }

  for (fp = 0; fp <= 0xFFF0; fp += 16) {
    if (memcmp(buf + fp, "_SM_", 4) == 0 && fp <= 0xFFE0) {
      len = WORD(buf + fp + 0x16);
      num = WORD(buf + fp + 0x1C);

      if (!checksum(buf + fp, (buf + fp)[0x05]) ||
          memcmp(buf + fp + 0x10, "_DMI_", 5) != 0 ||
          !checksum(buf + fp + 0x10, 0x0F)) {
        free(buf);
        return 0;
      }
      if ((nbuf = memChunk(DWORD(buf + fp + 0x18), len, devmem)) == NULL) {
        fprintf(stderr, "Table is unreachable, sorry.\n");
        free(buf);
        free(nbuf);
        return 0;
      }
      data = nbuf;
      while (i < num && data + 4 <= nbuf + len) {
        u_int8_t *next;
        struct dmi_header h;

        toDmiHeader(&h, data);

        if (h.length < 4) {
          printf("Invalid entry length (%u). DMI table is "
                 "broken! Stop.\n\n",
                 (unsigned int)h.length);
          free(buf);
          free(nbuf);
          return 0;
        }

        next = data + h.length;
        while (next - nbuf + 1 < len && (next[0] != 0 || next[1] != 0))
          next++;
        next += 2;
        if (h.type == 4) {
          p = h.data + 0x08;
          snprintf(cpuid, cpuid_len, "%02X%02X%02X%02X%02X%02X%02X%02X",
                   (unsigned char)p[0], (unsigned char)p[1],
                   (unsigned char)p[2], (unsigned char)p[3],
                   (unsigned char)p[4], (unsigned char)p[5],
                   (unsigned char)p[6], (unsigned char)p[7]);
          free(buf);
          free(nbuf);
          return 1;
        }
        data = next;
        i++;
      }
      fp += 16;
    }
  }
  free(buf);
  free(nbuf);
  return 0;
}

unsigned char *License::memChunk(size_t base, size_t len, const char *devmem) {
  unsigned char *p;
  int fd;
#ifdef USE_MMAP
  size_t mmoffset;
  void *mmp;
#endif

  if ((fd = open(devmem, O_RDONLY)) == -1) {
    perror(devmem);
    return NULL;
  }

  if ((p = (unsigned char *)malloc(len)) == NULL) {
    perror("malloc");
    return NULL;
  }

#ifdef USE_MMAP
#ifdef _SC_PAGESIZE
  mmoffset = base % sysconf(_SC_PAGESIZE);
#else
  mmoffset = base % getpagesize();
#endif /* _SC_PAGESIZE */
  /*
   * Please note that we don't use mmap() for performance reasons here,
   * but to workaround problems many people encountered when trying
   * to read from /dev/mem using regular read() calls.
   */
  mmp = mmap(0, mmoffset + len, PROT_READ, MAP_SHARED, fd, base - mmoffset);
  if (mmp == MAP_FAILED)
    goto try_read;

  memcpy(p, (u_int8_t *)mmp + mmoffset, len);

  if (munmap(mmp, mmoffset + len) == -1) {
    fprintf(stderr, "%s: ", devmem);
    perror("munmap");
  }

  goto out;

#endif /* USE_MMAP */

#ifdef USE_MMAP
try_read:
#endif
  if (lseek(fd, base, SEEK_SET) == -1) {
    fprintf(stderr, "%s: ", devmem);
    perror("lseek");
    free(p);
    return NULL;
  }

  if (myread(fd, p, len, devmem) == -1) {
    free(p);
    return NULL;
  }
#ifdef USE_MMAP
out:
#endif
  if (close(fd) == -1)
    perror(devmem);

  return p;
}

int License::getAllMac(char (*mac)[MAC_INFO_LEN]) {
#define MAXINTERFACES 16
  struct ifreq ifr;

  int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
  if (sock == -1) {
    printf("get_all_mac: socket error\n");
    return -1;
  }
  FILE *in = fopen("/proc/net/dev", "r");
  if (NULL == in) {
    printf("get_all_mac: fopen(\"/proc/net/dev\", \"r\") failure\n");
    return -1;
  }
  char ifrn_name[IFNAMSIZ];
  int count = 0;
  char line[1000];
  memset(line, 0, sizeof(line));
  while (fgets(line, 1000, in)) {
    if (NULL == getMacName(line, 1000, ifrn_name, IFNAMSIZ)) {
      continue;
    }
    memset(line, 0, sizeof(line));
    strncpy(ifr.ifr_name, ifrn_name, IFNAMSIZ);
    if (ioctl(sock, SIOCGIFFLAGS, &ifr) == 0) {
      if (!(ifr.ifr_flags & IFF_LOOPBACK)) {  // don't count loopback
        if (ioctl(sock, SIOCGIFHWADDR, &ifr) == 0) {
          snprintf(mac[count], MAC_INFO_LEN, "%02x%02x%02x%02x%02x%02x",
                   (unsigned char)ifr.ifr_hwaddr.sa_data[0],
                   (unsigned char)ifr.ifr_hwaddr.sa_data[1],
                   (unsigned char)ifr.ifr_hwaddr.sa_data[2],
                   (unsigned char)ifr.ifr_hwaddr.sa_data[3],
                   (unsigned char)ifr.ifr_hwaddr.sa_data[4],
                   (unsigned char)ifr.ifr_hwaddr.sa_data[5]);
          count++;
          if (count >= MAXINTERFACES) {
            LOG(ERROR) << "get_all_mac: mac can't over " << MAXINTERFACES;
            fclose(in);
            return -1;
          }
        }
      }
    } else {
      printf("get_all_mac: get mac info error\n");
      fclose(in);
      return -1;
    }
  }

  fclose(in);
  return count;
}

char *License::getMacName(const char *line, int line_len, char *name,
                          int name_len) {
  memset(name, 0, name_len * sizeof(char));
  int i = 0;
  int j = 0;
  if (NULL == strchr(line, ':')) {
    return NULL;
  }
  for (i = 0; i < line_len; i++) {
    switch (line[i]) {
    case ':':
      return name;
      break;
    case ' ':
      break;
    default:
      if (j >= name_len) {
        return NULL;
      }
      name[j++] = line[i];
      break;
    }
  }
  return NULL;
}

int License::checksum(const u_int8_t *buf, size_t len) {
  u_int8_t sum = 0;
  size_t a;

  for (a = 0; a < len; a++)
    sum += buf[a];
  return (sum == 0);
}

void License::toDmiHeader(struct dmi_header *h, u_int8_t *data) {
  h->type = data[0];
  h->length = data[1];
  h->handle = WORD(data + 2);
  h->data = data;
}

void License::createHashSHA256(const char *data, size_t size,
                               unsigned char *hash) {
  SHA256_CTX sha256;
  SHA256_Init(&sha256);
  SHA256_Update(&sha256, data, size);
  SHA256_Final(hash, &sha256);
}

std::vector<char> License::EncryptByPrikeyString(unsigned char *message,
                                                 size_t messageLength,
                                                 const char *prikey) {
  BIO *in = BIO_new_mem_buf(static_cast<const void *>(prikey), -1);
  if (in == NULL) {
    LOG(ERROR) << "BIO_new_mem_buf failed.";
    return std::vector<char>();
  }

  RSA *rsa = PEM_read_bio_RSAPrivateKey(in, NULL, NULL, NULL);
  BIO_free(in);
  if (rsa == NULL) {
    LOG(INFO) << "PEM_read_bio_RSAPrivateKey failed.";
    return std::vector<char>();
  }

  int size = RSA_size(rsa);
  std::vector<char> encrypt_data;
  encrypt_data.resize(size);
  int ret = RSA_private_encrypt(messageLength, message,
                                (unsigned char *)encrypt_data.data(), rsa,
                                RSA_PKCS1_PADDING);
  RSA_free(rsa);
  if (ret == -1) {
    LOG(ERROR) << "RSA_private_encrypt failed";
    return std::vector<char>();
  }

  return encrypt_data;
}

std::string License::DecryptByPubkeyString(const char *cipher, uint32_t len,
                                           const char *pubkey) {
  BIO *in = BIO_new_mem_buf(static_cast<const void *>(pubkey), -1);
  if (in == NULL) {
    LOG(ERROR) << "BIO_new_mem_buf failed.";
    return "";
  }

  RSA *rsa = PEM_read_bio_RSA_PUBKEY(in, NULL, NULL, NULL);
  BIO_free(in);
  if (rsa == NULL) {
    LOG(ERROR) << "PEM_read_bio_RSA_PUBKEY failed.";
    return "";
  }

  int size = RSA_size(rsa);
  std::vector<char> data;
  data.resize(size);
  int ret =
      RSA_public_decrypt(len, (unsigned char *)cipher,
                         (unsigned char *)data.data(), rsa, RSA_PKCS1_PADDING);
  RSA_free(rsa);
  if (ret == -1) {
    LOG(ERROR) << "RSA_public_decrypt failed.";
    return "";
  }
  std::string decrypt_data(data.begin(), data.end());
  return decrypt_data;
}

int License::myread(int fd, unsigned char *buf, size_t count,
                    const char *prefix) {
  ssize_t r = 1;
  size_t r2 = 0;

  while (r2 != count && r != 0) {
    r = read(fd, buf + r2, count - r2);
    if (r == -1) {
      if (errno != EINTR) {
        close(fd);
        perror(prefix);
        return -1;
      }
    } else {
      r2 += r;
    }
  }

  if (r2 != count) {
    close(fd);
    fprintf(stderr, "%s: Unexpected end of file\n", prefix);
    return -1;
  }

  return 0;
}

}  // namespace rosc
