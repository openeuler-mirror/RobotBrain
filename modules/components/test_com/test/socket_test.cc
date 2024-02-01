/* 使用select函数可以以非阻塞的方式和多个socket通信。程序只是演示select函数的使用，连接数达到最大值后会终止程序。
1. 程序使用了一个数组fd，通信开始后把需要通信的多个socket描述符都放入此数组
2. 首先生成一个叫sock_fd的socket描述符，用于监听端口。
3. 将sock_fd和数组fd中不为0的描述符放入select将检查的集合fdsr。
4.
处理fdsr中可以接收数据的连接。如果是sock_fd，表明有新连接加入，将新加入连接的socket描述符放置到fd。
*/
// select_server.c
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <iostream>
#include <gtest/gtest.h>

using namespace std;

int thread_exit = 0;
#define MAX_REQUEST_NUM 5  //连接队列中的个数

#define BUFFER_SIZE 200

int fd[MAX_REQUEST_NUM];  //连接的fd

char recv_msg_buf_[BUFFER_SIZE];
struct sockaddr_in server_addr_;
int server_sock_fd_;  // 服务器端套接字描述符
int client_sock_fd_;  // 客户端套接字描述符
const char *server_ip_ = nullptr;
unsigned int port_ = 8100;
struct sockaddr_in client_address_;
socklen_t address_len_ = sizeof(client_address_);

void ProcessNewConnectionRequest(fd_set &server_fd_set) {
  /*  处理新的连接请求  */
  if (!FD_ISSET(server_sock_fd_, &server_fd_set)) {
    return;
  }
  int new_client_sock_fd = accept(
      server_sock_fd_, (struct sockaddr *)&client_address_, &address_len_);
  if (new_client_sock_fd <= 0) {
    cout << "error" << endl;
    return;
  }
  if (client_sock_fd_ == 0) {
    cout << "new connection ip = " << inet_ntoa(client_address_.sin_addr)
         << "; port = " << ntohs(client_address_.sin_port) << endl;

    client_sock_fd_ = new_client_sock_fd;
    FD_SET(client_sock_fd_, &server_fd_set);
  } else {
    cout << "max connections arrive, exit\n" << endl;
    send(new_client_sock_fd, "bye", 4, 0);
    close(new_client_sock_fd);
    new_client_sock_fd = 0;
  }
}

void ReceiveProcessClientMessage(fd_set &server_fd_set) {
  /*  处理某个客户端发过来的消息  */
  if ((client_sock_fd_ <= 0) || (!FD_ISSET(client_sock_fd_, &server_fd_set))) {
    cout << "haven't received new message!" << endl;
    return;
  }
  bzero(recv_msg_buf_, BUFFER_SIZE);
  int recvNum = recv(client_sock_fd_, recv_msg_buf_, BUFFER_SIZE, 0);
  // >0,接收消息成功
  if (recvNum > 0) {
    if (recvNum > BUFFER_SIZE) {
      recvNum = BUFFER_SIZE;
    }
    recv_msg_buf_[recvNum] = '\0';
    printf("recv: %s\n", recv_msg_buf_);
  } else if (recvNum < 0) {
    cout << "receive message error!" << endl;
  } else {  // =0,对端连接关闭
    FD_CLR(client_sock_fd_, &server_fd_set);
    client_sock_fd_ = 0;
    cout << "client close the connection!" << endl;
  }
}

// server
int startServer(void) {
  memset(&server_addr_, 0, sizeof(server_addr_));
  server_addr_.sin_family = AF_INET;
  server_addr_.sin_port = htons(port_);
  cout << "start" << endl;
  if (server_ip_ != nullptr) {
    cout << "server_ip_ != nullptr" << endl;
    server_addr_.sin_addr.s_addr =
        inet_addr(server_ip_);  // 限定只接受本地连接请求
  } else {
    cout << "server_ip_ == nullptr" << endl;
    server_addr_.sin_addr.s_addr = INADDR_ANY;
  }

  // server_addr_.sin_addr.s_addr = INADDR_ANY;

  cout << "bind socket ip:" << (server_ip_ == nullptr ? "nullptr" : server_ip_)
       << "; port:" << port_ << endl;

  // 1.创建socket
  server_sock_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (server_sock_fd_ < 0) {
    cout << "create socket error:" << errno << endl;
    return -1;
  }

  // 2.绑定socket和端口号
  if (bind(server_sock_fd_, (struct sockaddr *)&server_addr_,
           sizeof(server_addr_)) < 0) {
    cout << "bind socket error:" << errno << endl;
    return -1;
  }

  // 3.监听listen
  if (listen(server_sock_fd_, MAX_REQUEST_NUM) < 0) {
    printf("listen socket error:%s(errno:%d)\n", strerror(errno), errno);
    return -1;
  }

  fd_set server_fd_set;  // 文件描述符集合
  int max_fd = -1;
  struct timeval tv;

  int selectResult;
  while (!thread_exit) {
    tv.tv_sec = 10;  // 超时时间10s
    tv.tv_usec = 0;

    FD_ZERO(&server_fd_set);
    FD_SET(server_sock_fd_, &server_fd_set);
    max_fd = server_sock_fd_;

    if (client_sock_fd_ > 0) {
      FD_SET(client_sock_fd_, &server_fd_set);
      max_fd = max_fd > client_sock_fd_ ? max_fd : client_sock_fd_;
    }
    selectResult = select(max_fd + 1, &server_fd_set, NULL, NULL, &tv);
    if (selectResult < 0) {
      cout << "select error:" << errno << endl;
    } else if (selectResult == 0) {  // 超时
      cout << "timeout";
      continue;
    }
    ProcessNewConnectionRequest(server_fd_set);
    ReceiveProcessClientMessage(server_fd_set);
  }
  return 0;
}

// client
int startClient(const char *addr) {
  int sockfd, n;
  char recvline[4096], sendline[4096];
  struct sockaddr_in servaddr;

  if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    printf("create socket error: %s(errno: %d)\n", strerror(errno), errno);
    exit(0);
  }

  memset(&servaddr, 0, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_port = htons(8100);
  char ch[200];
  // 地址转换函数
  if (inet_pton(AF_INET, addr, &servaddr.sin_addr) <= 0) {
    printf("inet_pton error for %s\n", addr);
    exit(0);
  }

  if (connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
    printf("connect error: %s(errno: %d)\n", strerror(errno), errno);
    exit(0);
  }

  printf("send msg to server: \n");
  int count = 0;
  while (!thread_exit) {
    usleep(100000);
    snprintf(sendline, sizeof(sendline), "%i\n", count);
    count++;
    if (send(sockfd, sendline, strlen(sendline), 0) < 0) {
      printf("send msg error: %s(errno: %d)\n", strerror(errno), errno);
      exit(0);
    }
    // ch[0] = '\0';
    // int rc = read(sockfd, ch, 200);
    // if (rc != -1 || rc != 0) {
    //   printf("=====%s\n", ch);
    // } else {
    //   printf("rc: %d", rc);
    //   break;
    // }
  }
  close(sockfd);
}

/**
 * @brief 测试socket通信
 *
 */
TEST(SocketTest, testSocket) {
  // make two thread for run server and run client, and join them, kill them
  // after 2 seconds
  thread server_thread(startServer);
  thread client_thread(startClient, "127.0.0.1");
  sleep(1);
  thread_exit = 1;
  server_thread.join();
  client_thread.join();
}
