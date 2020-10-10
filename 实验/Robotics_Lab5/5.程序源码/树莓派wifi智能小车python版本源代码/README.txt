一.python代码的运行方式
   python + 程序
   如：
   python TCP_Control.py
二.如果使用的是我们出厂的系统（已配置好了开发中所需要的各种库文件），以下内容仅供树莓派原始系统参考：
   在我们使用树莓派的GPIO编程之前，首先我们需要安装基于树莓派的GPIO库文件：
1.在线安装RPi.GPIO
  sudo apt-get install python-dev
2.离线安装RPi.GPIO
  首先先将树莓派软件与文档目录下的RPi.GPIO-0.6.3.tar.gz通过ssh文件传输
  到树莓派的系统中；
  其次，解压文件：tar -xvf RPi.GPIO-0.6.3.tar.gz
  进入解压之后的目录：
  cd RPi.GPIO-0.6.3
  启动安装：
  sudo python setup.py install








