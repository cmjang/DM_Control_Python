# 达妙Python库

### 1.引用达妙库

默认文件夹下有两个文件DM_CAN.py为所在的电机库，使用的时候

```python
from DM_CAN import *	
```

电机库相关依赖是serial , numpy , struct这几个库，记得安装相关依赖

### 2.定义控制对象

定义电机对象，有几个电机就定义几个。重要的事情：不要把masterid设为0x00

```python
Motor1=Motor(DM_Motor_Type.DM4310,0x01,0x11)
Motor2=Motor(DM_Motor_Type.DM4310,0x02,0x12)
Motor3=Motor(DM_Motor_Type.DM4310,0x03,0x13)
```

第一个参数为电机类型，第二个是SlaveID即电机的CANID（电机的ID）,第三个参数是MasterID是主机ID，建议MasterID设置的都不一样，比SlaveID整体高一个。例如Motor1的SlaveID是0x01，MasterID是0x11.这样是最好。

python使用serial串口，波特率是921600，串口进行选择。demo是windows所以是'COM8'

```python
serial_device = serial.Serial('COM8', 921600, timeout=0.5)
```

初始化电机控制对象。传入参数是定义的串口对象

```python
MotorControl1=MotorControl(serial_device)
```

### 3.电机控制

**推荐在每帧控制完后延迟2ms或者1ms，usb转can默认有缓冲器没有延迟也可使用，但是推荐加上延迟。**

添加电机是addMotor，然后使能电机是enable

```python
MotorControl1.addMotor(Motor1)
MotorControl1.addMotor(Motor2)
MotorControl1.addMotor(Motor3)
MotorControl1.enable(Motor1)
MotorControl1.enable(Motor2)
MotorControl1.enable(Motor3)
```

#### 3.1MIT模式

使能电机后可以使用MIT模式控制，推荐用MIT模式控制。

```python
MotorControl1.controlMIT(Motor1, 50, 0.3, 0, 0, 0)
```

其中MotorControl电机控制还有附带delay时间的控制。

```python
MotorControl1.controlMIT_delay(Motor1, 50, 0.3, 0, 0, 0，0.001)
```

#### 3.2 位置速度模式

位置速度模式，第一个参数是电机对象，第二个是位置，第三个是转动速度。具体的参数介绍已经写了函数文档，用pycharm等ide就可以看到。

例子如下

```python
q=math.sin(time.time())
MotorControl1.control_Pos_Vel(Motor1,q*10,2)
```

#### 3.3 速度模式

例子如下，第一个是电机对象，第二个是电机速度

```python
q=math.sin(time.time())
MotorControl1.control_Vel(Motor1, q*5)
```

目前达妙的新固件支持切换

#### 3.4力位混合模式

第一个是电机对象，第二个是电机位置，第三个是电机速度范围是0-10000，第四个是电机电流范围为0-10000。具体详细请查看达妙文档

例子如下

```python
MotorControl1.control_pos_force(Motor1, 10, 1000,100)
```

### 4.电机模式更改

达妙电机新固件支持使用can进行电机模式修改，以及修改其他参数等操作。具体请咨询达妙客服。

通过下面的函数可以对电机的控制模式进行修改。支持MIT,POS_VEL,VEL,Torque_Pos。四种控制模式在线修改。下面是修改的demo。

```python
MotorControl1.switchControlMode(Motor1,Control_Type.POS_VEL)
```

**保存参数**

默认电机修改模式等操作后参数不会保存到flash中，需要使用命令如下进行保存至电机的flash中

```python
MotorControl1.save_motor_param(Motor1)
```

