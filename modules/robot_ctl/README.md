# 简要介绍
robot_ctl，又成为rctl，是一个命令行管理工具，可是使用rctl命令，管理和配置机器人状态等。主要提供给调试人员，快速调试和测试机器人的状态。

参考文件(接口设计规范)[https://pantao.gitbooks.io/api-design-guide/content/errors.html]
## 原理
rctl通过利用grpc的机制，访问robot_server，完成数据的分发和交互。

## 语法
使用以下语法 rctl 从终端窗口运行命令
```
rctl [TYPE] [NAME] [command] [flags]
```

## 实现的命令
### ping
- `rctl ping` 测试服务器是否联通

## 待实现命令
### power
- `rctl power on` 上电
- `rctl power shutdown` 下电

### axios
- `rctl axios list` 获得轴的信息
- `rctl axios <axios_name> set <radio_value>` 设置轴的角度
