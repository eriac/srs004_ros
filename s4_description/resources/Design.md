# SRS004 design

# Software System Overview

```plantuml
node user{
  node Joy
  node View
  node GlobalSetting
}

node GameSystem
node NavigationSystem
node FireControlSystem
node ViewSystem

node LocalizeFunction
node DetectFunction
database tf

Joy -d-> GameSystem: joy
ViewSystem -u-> View: image

GameSystem =d=> NavigationSystem: NavAction
GameSystem =d=> FireControlSystem: GunAction
GameSystem =d=> ViewSystem: ViewAction

LocalizeFunction -u-> tf: self position
tf -u-> NavigationSystem
tf -u-> FireControlSystem
tf -u-> ViewSystem
DetectFunction -u-> NavigationSystem: object
DetectFunction -u-> FireControlSystem: object
DetectFunction -u-> ViewSystem: object

frame actuator {
  node Omni
  node Gun
  node Body
}
NavigationSystem -d-> Omni: cmd_vel
Omni -u-> LocalizeFunction: odom
FireControlSystem -d-> Gun: point, shot, laser
Gun -u-> FireControlSystem: point
Body -u-> GameSystem: HP
FireControlSystem -d-> Body: Light
Body -u-> ViewSystem: HP, Bat
GlobalSetting -d-> Body: ResetHP, Color

frame sensor {
  node HeadCamera
  node FrontLidar
  node IMU
  node Sonor
}
FrontLidar -u-> NavigationSystem: scan
FrontLidar -u-> LocalizeFunction: scan
FrontLidar -u-> DetectFunction: scan
FrontLidar -u-> ViewSystem: scan
HeadCamera -u-> DetectFunction: image
HeadCamera -u-> ViewSystem: image
IMU -u-> LocalizeFunction: imu
Sonor -u-> NavigationSystem: range
```

# Software Detail

## detection

```dot
digraph graph_name {
  graph [
    rankdir=LR;
  ]

  subgraph cluster_name1 {
    label = "device/sensor";
    "ydlidar";
    "image_proc";
  }

  "image_proc" -> "hsv_detector" [label=image_color_proc];
  "image_proc" -> "hsv_detector2" [label=image_color_proc];
  "ydlidar" -> obstacle_exoractor [label=scan]

  subgraph cluster_name2 {
    label = "detection";
    obstacle_exoractor -> obstacle_tracker
    obstacle_tracker -> "object_merger"
    obstacle_tracker -> "object_merger2"
    
    subgraph cluster_name3 {
      label = "red_can";
      "hsv_detector" -> "rect_tracker";
      "rect_tracker" -> "rect_projector";
      "rect_projector" -> "object_merger";
    }
    "object_merger" -> object_mixer
  
    subgraph cluster_name4 {
      label = "green_board";
      "hsv_detector2" -> "rect_tracker2";
      "rect_tracker2" -> "rect_projector2";
      "rect_projector2" -> "object_merger2";
    }
    "object_merger2" -> object_mixer

    object_mixer -> output
  }
}
```
# Hardware

## JoyCommand(Button)

| Button | Function |
|:-:|:-:|
| Triangle (Up) | Laser off |
| Square (Left) | Reset Gun |
| Circle (Right)  | Gun Tracking |
| Cross (Down) | Shot(+R1) |
| L1 | Move SpeedUp |
| L2 | Move Left |
| L3 | None |
| Allow Up | Select |
| Allow Left | None |
| Allow Right | None |
| Allow Down | None |
| L1 | Laser on & Gun SpeedDown |
| L2 | Move Right |
| L3 | None |

## JoyCommand(Axis)
| Axis | Function |
|:-:|:-:|
| RV | Move Foward |
| RH | Move Turn |
| LV | Gun Elevation |
| LH | Gun Train |


## CANLink & Power distribution
```plantuml
salt
{
  {T
    + PowerHab
    ++ WheelModule(ID:1)
    +++ GeerMotor
    +++ Encoder
    ++ WheelModule(ID:2)
    +++ GeerMotor
    +++ Encoder
    ++ WheelModule(ID:3)
    +++ GeerMotor
    +++ Encoder
    ++ BodyModule(ID:4)
    +++ HitSensor0
    +++ HitSensor1
    +++ HitSensor2
    +++ USrange0
    +++ USrange1
    +++ USrange2
    +++ FrontLight
    ++ GunModule(ID:5)
    +++ Servo0
    +++ Servo1
    +++ GunMotor
    +++ GunSW
    +++ Laser
    +++ Magagene
    ++ CANAdapter(ID:Master)
    +++ RaspberryPi
    ++++ Camera
    ++++ IMU
    ++ LIDARPower(ID:None)
  }
}
```

## Part List (Mechanical)

| Group | Name | Quantity | price | shop |
|:-:|:-:|:-:|:-:|:-:|
| Wheel | GA370 | 3 | \1100 | Aliexpress |
|  | 38mmOmni | 3 | \432 | Viston |
|  | OmniHav | 3 | \270 | Viston |
| Gun | MechBox&Barrel | 1 | \5500 | TokyoMarui |
|  | StepperMotor | 1 | \400 | ? |
|  | GreenLaser | 1 | \1000 | Aliexpress |
|  | KRS4031 | 2 | \7000 | KondoKagaku |
| Sensor | WebCamera | 1 | \2300 | Baffalo |
|  | YDLIDAR | 1 | \11000 | Aliexpress |
|  | US-15 | 3 | \500 | Akizuki |
| Power | LiPo | 1 | \3000 | ? |
| System | RaspberryPi3B | 1 | \6000 | ? |
|  | WiFiModule | 1 | \4000 | ? |


||||||
Hit
Power
System
