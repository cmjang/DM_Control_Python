# DaMiao Python Library

Language: [中文](README_zh.md) | [ENGLISH](README.md) 

This library supports **macOS**, **Linux**, and **Windows** platforms.

🎉 **Join the QQ group for DaMiao motor technical discussions**: [677900232].  
🛒 **Visit the DaMiao Store on Taobao**: [DaMiao Intelligent Control Store](https://shop290016675.taobao.com/?spm=pc_detail.29232929/evo365560b447259.shop_block.dshopinfo.59f47dd6w4Z4dX).

---

## Table of Contents
1. [Importing the DaMiao Library](#1-importing-the-DaMiao-library)
2. [Defining Motor Control Objects](#2-defining-motor-control-objects)
3. [Motor States](#3-motor-states)
   - [Adding Motors](#31-adding-motors)
   - [Enabling Motors](#32-enabling-motors)
   - [Setting Zero Position](#33-setting-zero-position)
   - [Disabling Motors](#34-disabling-motors)
   - [Fetching Motor Status](#35-fetching-motor-status)
4. [Motor Control Modes](#4-motor-control-modes)
   - [MIT Mode](#41-mit-mode)
   - [Position-Velocity Mode](#42-position-velocity-mode)
   - [Velocity Mode](#43-velocity-mode)
   - [Force-Position Mixed Mode](#44-force-position-mixed-mode)
5. [Reading Motor States](#5-reading-motor-states)
6. [Modifying Motor Parameters](#6-modifying-motor-parameters)
   - [Changing Control Mode](#61-changing-control-mode)
   - [Saving Parameters](#62-saving-parameters)
   - [Reading Internal Registers](#63-reading-internal-registers)
   - [Writing Internal Registers](#64-writing-internal-registers)

---

## 1. Importing the DaMiao Library

Ensure that `DM_CAN.py` is in your project folder. Import it as follows:

```python
from DM_CAN import *
```
The library depends on the following Python packages: `serial`, `numpy`. Install them using:

```bash
pip install -r requirements.txt
```

* * *

## 2. Defining Motor Control Objects

Create motor objects using:

```python
Motor1 = Motor(DM_Motor_Type.DM4310, 0x01, 0x11)
Motor2 = Motor(DM_Motor_Type.DM4310, 0x02, 0x12)
Motor3 = Motor(DM_Motor_Type.DM4310, 0x03, 0x13)
```

* **First Parameter**: Motor type
* **Second Parameter**: Slave ID (Motor's CAN ID)
* **Third Parameter**: Master ID (Host ID; must be unique and not `0x00`)

Set up the serial device (example for Windows):

```python
serial_device = serial.Serial('COM8', 921600, timeout=0.5)
```

Initialize the motor control object:

```python
MotorControl1 = MotorControl(serial_device)
```

* * *

## 3. Motor States

### 3.1 Adding Motors

```python
MotorControl1.addMotor(Motor1)
MotorControl1.addMotor(Motor2)
MotorControl1.addMotor(Motor3)
```

### 3.2 Enabling Motors

Enable motors (ensure parameters are set before enabling):

```python
MotorControl1.enable(Motor1)
MotorControl1.enable(Motor2)
MotorControl1.enable(Motor3)
```

For older firmware, specify the control mode:

```python
MotorControl1.enable_old(Motor1, Control_Type.MIT)
MotorControl1.enable_old(Motor2, Control_Type.POS_VEL)
MotorControl1.enable_old(Motor3, Control_Type.VEL)
```

### 3.3 Setting Zero Position

Move the motor to the desired zero position (in disabled state) and set it as the new zero position:

```python
MotorControl1.set_zero_position(Motor3)
MotorControl1.set_zero_position(Motor6)
```

### 3.4 Disabling Motors

```python
MotorControl1.disable(Motor3)
MotorControl1.disable(Motor6)
```

### 3.5 Fetching Motor Status

To fetch motor status (torque, position, velocity) without sending control commands:

```python
MotorControl1.refresh_motor_status(Motor1)
print("Motor1:","POS:", Motor1.getPosition(), "VEL:", Motor1.getVelocity(), "TORQUE:", Motor1.getTorque())
```

* * *

## 4. Motor Control Modes

### 4.1 MIT Mode

After enabling the motor, use MIT mode for control:

```python
MotorControl1.controlMIT(Motor1, 50, 0.3, 0, 0, 0)
```

### 4.2 Position-Velocity Mode

Control motor position and velocity:

```python
q = math.sin(time.time())
MotorControl1.control_Pos_Vel(Motor1, q*10, 2)
```

### 4.3 Velocity Mode

Control motor velocity:

```python
q = math.sin(time.time())
MotorControl1.control_Vel(Motor1, q*5)
```

### 4.4 Force-Position Mixed Mode

Control motor position and force:

```python
MotorControl1.control_pos_force(Motor1, 10, 1000, 100)
```

* * *

## 5. Reading Motor States

Motor states are stored in the motor object. You can read the following parameters:

* Velocity
* Position
* Torque

Use the following functions:

```python
vel = Motor1.getVelocity()    # Get motor velocity
pos = Motor1.getPosition()    # Get motor position
tau = Motor1.getTorque()      # Get motor torque
```

After refreshing motor status:

```python
MotorControl1.refresh_motor_status(Motor1)
print("Motor1:", "POS:", Motor1.getPosition(), "VEL:", Motor1.getVelocity(), "TORQUE:", Motor1.getTorque())
```

* * *

## 6. Modifying Motor Parameters

### 6.1 Changing Control Mode

You can change the motor's control mode using the following function:

```python
if MotorControl1.switchControlMode(Motor1, Control_Type.POS_VEL):
    print("Switch to POS_VEL mode success")
if MotorControl1.switchControlMode(Motor2, Control_Type.VEL):
    print("Switch to VEL mode success")
```

Note: The change is only temporary and will be reset after power cycle.

### 6.2 Saving Parameters

To save all changed parameters to the motor's flash memory:

```python
MotorControl1.save_motor_param(Motor1)
```

### 6.3 Reading Internal Registers

You can read internal registers of the motor, such as:

```python
print("PMAX:", MotorControl1.read_motor_param(Motor1, DM_variable.PMAX))
print("MST_ID:", MotorControl1.read_motor_param(Motor1, DM_variable.MST_ID))
print("VMAX:", MotorControl1.read_motor_param(Motor1, DM_variable.VMAX))
print("TMAX:", MotorControl1.read_motor_param(Motor1, DM_variable.TMAX))
```

Use `getParam` to retrieve stored values:

```python
print("PMAX", Motor1.getParam(DM_variable.PMAX))
```

### 6.4 Writing Internal Registers

Some internal register values can be modified. Here's an example:

```python
if MotorControl1.change_motor_param(Motor1, DM_variable.KP_APR, 54):
   print("Write success")
```

Note: Changes will be lost after power cycle unless saved to flash memory.
