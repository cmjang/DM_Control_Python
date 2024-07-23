# 达妙Python库

### 1.引用达妙库

默认文件夹下有两个文件DM_CAN.py为所在的电机库，使用的时候

```python
from DM_CAN import *	
```

### 2.定义控制对象

定义电机对象，有几个电机就定义几个。

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

添加电机是addMotor，然后使能电机是enable

```python
MotorControl1.addMotor(Motor1)
MotorControl1.addMotor(Motor2)
MotorControl1.addMotor(Motor3)
MotorControl1.enable(Motor1)
MotorControl1.enable(Motor2)
MotorControl1.enable(Motor3)
```

使能电机后可以使用MIT模式控制，推荐用MIT模式控制。

```python
MotorControl1.control(Motor1, 50, 0.3, 0, 0, 0)
```

推荐在每帧控制完后延迟2ms或者1ms，usb转can默认有缓冲器没有延迟也可使用，但是推荐加上延迟。

其中MotorControl电机控制还有附带delay时间的控制。

```python
MotorControl1.control_delay(Motor1, 50, 0.3, 0, 0, 0，0.001)
```

