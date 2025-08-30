| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-H2 | ESP32-H21 | ESP32-H4 | ESP32-P4 | ESP32-S2 | ESP32-S3 | Linux |
| ----------------- | ----- | -------- | -------- | -------- | -------- | --------- | -------- | --------- | -------- | -------- | -------- | -------- | ----- |

Face recognition program based on esp32s3 (esp-idf version) Core Architecture

基于 esp32s3（使用 esp-idf 版本）的面部识别程序核心架构
1.多任务队列通信架构 
xQueueAIFrame：将摄像头帧传输至人工智能处理任务 
xQueueLCDFrame：将处理后的帧传输至液晶显示任务 
xQueueEvent：传输标识状态事件（注册/识别/删除） 
xQueueResult：传输面部识别结果
2.四个核心任务 
task_process_camera：采集摄像头数据并将其发送至人工智能队列 
task_process_ai：人工智能处理核心（检测 + 识别），处理三种状态：ENROLL（注册新面孔） RECOGNIZE（识别面孔） DELETE（删除已注册面孔） 
task_process_lcd：将处理结果显示在液晶显示屏上 
task_event_ai：事件处理，更新识别状态
3.人工智能处理流程 
使用 MSR01 和 MNP01 双检测器进行面部检测 
基于 FaceRecognition112V1S16 模型进行特征提取和匹配 
支持从闪存分区（“fr”）存储和加载面部数据 在屏幕上显示识别状态和结果
状态管理 现有操作模式通过全局变量 gEvent 控制，并支持三种面部管理操作以及一个用于结果的显示状态机。
