# Fun4
## Getting Started

### Prerequisites
ตรวจสอบว่าได้มีการติดตั้ง Library ดังต่อไปนี้:
- `numpy`
- `roboticstoolbox`
- `spatialmath`
- `matplotlib`

### Installation
Clone the repository:
```
git clone https://github.com/ch4dum/fun4.git
```
Build the workspace:
```
cd fun4
colcon build && source install/setup.bash
```

## Part 1: Setup Environment
Launch:
```
ros2 launch example_description simple_display.launch.py
```
เมื่อทำการ Launch จะมีหน้าต่างขึ้นมา 3 หน้าต่างคือ 
- Figure 1
- Joint State Publisher GUI
- RViz

Figure 1 จะแสดงถึง **Workspace** การทำงานของตัวหุ่น และจะมี Function ชื่อ checkworkspace ที่เป็นวิธีการตรวจคำตอบอยู่ในไฟล์ controller_node.py ที่บรรทัดที่ 107 ดังภาพด้านล่าง
![Screenshot from 2024-10-04 12-34-12](https://github.com/user-attachments/assets/2f6916cd-44a8-4900-aa98-3b2dd9a84c8f)

การตรวจสอบ **Topic** `/target` และ `/end_effector`:
เปิดหน้าต่าง Terminal ใหม่ และพิมพ์คำสั่ง:
```
ros2 topic list
```
จะเห็น topic ที่มีชื่อว่า `/target` และ `/end_effector` ดังภาพด้านล่าง
![Screenshot from 2024-10-04 12-39-53](https://github.com/user-attachments/assets/ac666add-57e9-4f18-8c67-de896be3db9d)

## Part 2: Controller
!!!ให้ทำการ**ปิดหน้าต่าง** Figure 1 และ Joint State Publisher GUI เพื่อเริ่มการทำงานของ Part 2
ในการเลือกโหมดการทำงานจะสามารถทำได้โดยการใช้ service call
เปิดหน้าต่าง Terminal ใหม่(หรือใช้หน้าต่างเดิมที่ตรวจสอบ Topic) และพิมพ์คำสั่ง:
```
source install/setup.bash
ros2 service call /set_mode fun4_interfaces/srv/SetMode "mode:
  data: 0.0
ipk:
  x: 0.0
  y: 0.0
  z: 0.0"
```
ในการเปลี่ยนโหมดต่าง ๆ ให้เปลี่ยนในส่วนของ `data: 0.0` เป็น `1.0`, `2.1` , `2.2` และ `3.0` เพื่อใช้งานโหมด Inverse Pose Kinematics (IPK), Teleoperation(ที่ Reference กับปลายมือ), Teleoperation(ที่ Reference กับฐานของหุ่นยนต์) และ Auto ตามลำดับ
### Inverse Pose Kinematics (IPK)
ในการใช้งานโหมด Inverse Pose Kinematics (IPK) ให้ใช้คำสั่ง service call ดังต่อไปนี้
```
ros2 service call /set_mode fun4_interfaces/srv/SetMode "mode:
  data: 1.0
ipk:
  x: 0.0
  y: 0.0
  z: 0.0"
```
โดยค่า `x`, `y` และ `z` จะเป็นตําแหน่ง Taskspace ที่ต้องการ

เมื่อพบคำตอบ

![Screenshot from 2024-10-04 13-25-44](https://github.com/user-attachments/assets/46b8bfa0-bff2-4670-b48b-ac10a972038b)

เมื่อไม่พบคําตอบ

![Screenshot from 2024-10-04 13-25-58](https://github.com/user-attachments/assets/777352b3-9b9f-489a-822e-9fa4fdebac59)

### Teleoperation
เปิดหน้าต่าง Terminal ใหม่เพื่อใช้งาน `teleop_twist_keyboard` ในการควบคุมหุ่นยนต์:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```
ปุ่มหลัก ๆ การควบคุมหุ่นยนต์:
- i/I: Increase X position
- J:   Increase Y position
- t:   Increase Z position
- ,/<: Decrease X position
- L:   Decrease Y position
- b:   Decrease Z position

*หมายเหตุ: สามารถเพิ่มลดความเร็วในแต่ละแกนตามการใช้งานปกติของ `teleop_twist_keyboard` แต่ไม่สามารถใช้งาน และเพิ่มลดความเร็วในส่วนของ angular speed ได้

#### Teleoperation ที่ Reference กับปลายมือ
ในการใช้งานโหมด Teleoperation ที่ Reference กับปลายมือให้ใช้คำสั่ง service call ดังต่อไปนี้
```
ros2 service call /set_mode fun4_interfaces/srv/SetMode "mode:
  data: 2.1
ipk:
  x: 0.0
  y: 0.0
  z: 0.0"
```
#### Teleoperation ที่ Reference กับฐานของหุ่นยนต์
ในการใช้งานโหมด Teleoperation ที่ Reference กับฐานของหุ่นยนต์ให้ใช้คำสั่ง service call ดังต่อไปนี้
```
ros2 service call /set_mode fun4_interfaces/srv/SetMode "mode:
  data: 2.2
ipk:
  x: 0.0
  y: 0.0
  z: 0.0"
```
ระหว่างการใช้งานหากตรวจพบว่าหุ่นยนต์เข้าใกล้สภาวะ Singularity หุ่นยนต์จะหยุดการเคลื่อนที่ และจะมีการแจ้งเตือนที่หน้า Terminal ที่ใช้ Launch ดังรูปด้านล่าง

![Screenshot from 2024-10-04 13-15-53](https://github.com/user-attachments/assets/01ce0e3d-e734-4f4e-bf9b-a491d53caafd)

และยังสามาารถใช้ topic echo เพื่อดูการแจ้งเตือนได้ดังนี้:
```
ros2 topic echo /singularity_warning 
```
ผลลัพธ์จะออกมาดังรูปด้านล่าง

![Screenshot from 2024-10-04 13-19-55](https://github.com/user-attachments/assets/346a6df8-7159-4a90-b5c6-1651801b5e05)

### Auto
ในการใช้งานโหมด Auto ให้ใช้คำสั่ง service call ดังต่อไปนี้
```
ros2 service call /set_mode fun4_interfaces/srv/SetMode "mode:
  data: 3.0
ipk:
  x: 0.0
  y: 0.0
  z: 0.0"
```
*หมายเหตุ: 
- หาก `/target` และ `/end_effector` ไม่แสดงบน RViz สามารถกดเพิ่มได้ตามต้องการดังรูปด้านล่าง
![Screenshot from 2024-10-04 13-36-03](https://github.com/user-attachments/assets/4eafdf22-103f-411b-b6c7-b22afa03d18b)
- เนื่องจากไม่ได้แสดงการนับเวลาไว้ จึงสามารถดูได้จาก `[INFO]['time']` ที่แจ้งเตือนเมื่อมีการรับตำแหน่งจาก `random_target_node.py` และเมื่อถึงเป้าหมายจาก `jointstate_script.py` ในแต่ละรอบ ดังรูปด้านล่าง
![Screenshot from 2024-10-04 13-41-31](https://github.com/user-attachments/assets/3c436c47-699c-4770-81c3-a429455eee97)

