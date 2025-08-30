| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-H2 | ESP32-H21 | ESP32-H4 | ESP32-P4 | ESP32-S2 | ESP32-S3 | Linux |
| ----------------- | ----- | -------- | -------- | -------- | -------- | --------- | -------- | --------- | -------- | -------- | -------- | -------- | ----- |

Face recognition program based on esp32s3 (esp-idf version)
Core Architecture
1. Multi-task Queue Communication Architecture
xQueueAIFrame: Transmits camera frames to the AI processing task 
xQueueLCDFrame: Transmits the processed frames to the LCD display task 
xQueueEvent: Transmission identification status event (registration/identification/deletion) 
xQueueResult: Transmits the facial recognition result 
2. Four core tasks
task_process_camera: Capturing camera data and sending it to the AI queue 
task_process_ai: AI processing core (detection + recognition), handling three states: 
ENROLL: Register new face 
RECOGNIZE: Recognize a face 
DELETE: Delete registered face 
task_process_lcd: LCD displays the processing results 
task_event_ai: Event handling, update recognition status 
3. AI processing flow
Perform face detection using the MSR01 and MNP01 dual detectors 
Feature extraction and matching are carried out based on the FaceRecognition112V1S16 model. 
Support for storing and loading facial data from the flash partition ("fr") 
Display the recognition status and results on the screen 
4. State Management
The current operation mode is controlled through the gEvent global variable, and it supports three types of face management operations and a display status machine for results.
