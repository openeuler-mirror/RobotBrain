# docker_env
该目录下的资料，主要是为了构建一个基于docker的镜像，帮助gitlab-ci提供一个运行和测试的环境。

## 内容介绍
安装一些基础的编译工具和依赖库，配置cpplint等工具，将部分工作前置到基础环境中，加速CI/CD的速度。

## 镜像构建和发布脚本
```
docker build -t robot_brain_env
docker tag robot_brain_env harbor.acbot.net/rosc/robot_brain_env
docker push harbor.acbot.net/rosc/robot_brain_env
```
