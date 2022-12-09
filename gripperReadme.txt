1、开发环境：树莓派4b + Thonny Python IDE 

2、控制接口：
      (1) setopenlimit(openmax,openmin) 设置开口限位（最大开口度和最小开口度）（参数：0-1000）

      (2) setid(idnew) 设置ID（参数：1-254）

      (3) movetgt(tgt) 运动目标（参数：0-1000）

      (4) movemax(speed) 运动松开（参数：1-1000）

      (5) movemin(speed,power) 运动抓取（参数：1-1000 和 50-1000）

      (6) moveminhold(speed,power) 运动持续抓取（参数：1-1000 和 50-1000）

      (7) getopenlimit() 读取开口限位

      (8) getcopen() 读取当前开口

      (9) getstate() 读取当前状态

      (10) setestop() 急停

      (11) setparam() 参数固化

      (12) setFrsvd() 清除故障 


3、夹爪使用
    1）将夹爪与电脑连接，查看连接端口名称，
    2）用Python IDE打开inspire_hand.py 文件，把ttyUSB0改成你的端口号
       （端口名称应该会显示ttyUSB0或者是ttyUSB1、ttyUSB2等），
    3）在Python IDE的shell中运行上面的控制接口即可。




     

