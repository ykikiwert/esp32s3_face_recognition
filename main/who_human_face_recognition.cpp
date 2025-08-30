#include "who_human_face_recognition.hpp"

#include "esp_log.h"
#include "esp_camera.h"

#include "../components/esp-dl/include/image/dl_image.hpp"
#include  "../components/fb_gfx/include/fb_gfx.h"

#include "../components/esp-dl/include/model_zoo/human_face_detect_msr01.hpp"
#include "../components/esp-dl/include/model_zoo/human_face_detect_mnp01.hpp"
#include "../components/esp-dl/include/model_zoo/face_recognition_tool.hpp"
#include "esp32_s3_szp.h"

#include "../components/esp-dl/include/model_zoo/face_recognition_112_v1_s16.hpp"

#include "who_ai_utils.hpp"

using namespace std;
using namespace dl;



static QueueHandle_t xQueueLCDFrame = NULL; 
static QueueHandle_t xQueueAIFrame = NULL;
QueueHandle_t xQueueEvent = NULL;
static QueueHandle_t xQueueResult = NULL;


static recognizer_state_t gEvent = RECOGNIZE;
static bool gReturnFB = true;
static face_info_t recognize_result;

SemaphoreHandle_t xMutex;

typedef enum
{
    SHOW_STATE_IDLE,
    SHOW_STATE_DELETE,
    SHOW_STATE_RECOGNIZE,
    SHOW_STATE_ENROLL,
} show_state_t;

#define RGB565_MASK_RED 0xF800
#define RGB565_MASK_GREEN 0x07E0
#define RGB565_MASK_BLUE 0x001F
#define FRAME_DELAY_NUM 16

static void rgb_print(camera_fb_t *fb, uint32_t color, const char *str)
{
    fb_gfx_print(fb, (fb->width - (strlen(str) * 14)) / 2, 10, color, str);
}

static int rgb_printf(camera_fb_t *fb, uint32_t color, const char *format, ...)
{
    char loc_buf[64];
    char *temp = loc_buf;
    int len;
    va_list arg;
    va_list copy;
    va_start(arg, format);
    va_copy(copy, arg);
    len = vsnprintf(loc_buf, sizeof(loc_buf), format, arg);
    va_end(copy);
    if (len >= sizeof(loc_buf))
    {
        temp = (char *)malloc(len + 1);
        if (temp == NULL)
        {
            return 0;
        }
    }
    vsnprintf(temp, len + 1, format, arg);
    va_end(arg);
    rgb_print(fb, color, temp);
    if (len > 64)
    {
        free(temp);
    }
    return len;
}

static void task_process_ai(void *arg)
{
    camera_fb_t *frame = NULL;
    HumanFaceDetectMSR01 detector(0.3F, 0.3F, 10, 0.3F);
    HumanFaceDetectMNP01 detector2(0.4F, 0.3F, 10);

    FaceRecognition112V1S16 *recognizer = new FaceRecognition112V1S16();     //面部识别器
    // FaceRecognition112V1S8 *recognizer = new FaceRecognition112V1S8();
    show_state_t frame_show_state = SHOW_STATE_IDLE;
    recognizer_state_t _gEvent;
    recognizer->set_partition(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "fr");//从flash存储人脸数据
    int partition_result = recognizer->set_ids_from_flash();                 //从flsah加载已经注册的人脸            

    while (true)
    {
        // xSemaphoreTake(xMutex, portMAX_DELAY);
        _gEvent = gEvent;
        // gEvent = IDLE;
        // xSemaphoreGive(xMutex);      //无限循环处理AI任务


        if (_gEvent)
        {
            bool is_detected = false;

            if (xQueueReceive(xQueueAIFrame, &frame, portMAX_DELAY))     //从队列获取摄像头数据
            {
                std::list<dl::detect::result_t> &detect_candidates = detector.infer((uint16_t *)frame->buf, {(int)frame->height, (int)frame->width, 3});
                std::list<dl::detect::result_t> &detect_results = detector2.infer((uint16_t *)frame->buf, {(int)frame->height, (int)frame->width, 3}, detect_candidates);

                if (detect_results.size() == 1)
                    is_detected = true;

                if (is_detected)
                {
                    switch (_gEvent)
                    {
                    case ENROLL:                                        //注册新人脸
                        recognizer->enroll_id((uint16_t *)frame->buf, {(int)frame->height, (int)frame->width, 3}, detect_results.front().keypoint, "", true);
                        ESP_LOGW("ENROLL", "ID %d is enrolled", recognizer->get_enrolled_ids().back().id);
                        frame_show_state = SHOW_STATE_ENROLL;
                        break;

                    case RECOGNIZE:                                       //识别人脸
                        recognize_result = recognizer->recognize((uint16_t *)frame->buf, {(int)frame->height, (int)frame->width, 3}, detect_results.front().keypoint);
                        print_detection_result(detect_results);
                        if (recognize_result.id > 0)
                            ESP_LOGI("RECOGNIZE", "Similarity: %f, Match ID: %d", recognize_result.similarity, recognize_result.id);
                        else
                            ESP_LOGE("RECOGNIZE", "Similarity: %f, Match ID: %d", recognize_result.similarity, recognize_result.id);
                        frame_show_state = SHOW_STATE_RECOGNIZE;
                        break;

                    case DELETE:                                          //删除人脸     
                        vTaskDelay(10);
                        recognizer->delete_id(true);
                        ESP_LOGE("DELETE", "% d IDs left", recognizer->get_enrolled_id_num());
                        frame_show_state = SHOW_STATE_DELETE;
                        break;

                    default:
                        break;
                    }
                }

                if (frame_show_state != SHOW_STATE_IDLE)                  //根据不同状态在画面上显示不同文字和颜色
                {
                    static int frame_count = 0;
                    switch (frame_show_state)
                    {
                    case SHOW_STATE_DELETE:
                        rgb_printf(frame, RGB565_MASK_RED, "%d IDs left", recognizer->get_enrolled_id_num());
                        break;

                    case SHOW_STATE_RECOGNIZE:
                        if (recognize_result.id > 0)
                            rgb_printf(frame, RGB565_MASK_GREEN, "ID %d", recognize_result.id);
                        else
                            rgb_print(frame, RGB565_MASK_RED, "who ?");
                        break;

                    case SHOW_STATE_ENROLL:
                        rgb_printf(frame, RGB565_MASK_BLUE, "Enroll: ID %d", recognizer->get_enrolled_ids().back().id);
                        break;

                    default:
                        break;
                    }

                    if (++frame_count > FRAME_DELAY_NUM)
                    {
                        frame_count = 0;
                        frame_show_state = SHOW_STATE_IDLE;
                    }
                }

                if (detect_results.size())
                {
                    draw_detection_result((uint16_t *)frame->buf, frame->height, frame->width, detect_results);
                }
            }

            if (xQueueLCDFrame)         //发送到LCD队列
            {

                xQueueSend(xQueueLCDFrame, &frame, portMAX_DELAY);
            }
            else if (gReturnFB)
            {
                esp_camera_fb_return(frame);
            }
            else
            {
                free(frame);
            }

            if (xQueueResult && is_detected)         //发送人脸识别结果
            {
                xQueueSend(xQueueResult, &recognize_result, portMAX_DELAY);
            }
        }
    }
}

// lcd处理任务
static void task_process_lcd(void *arg)
{
    camera_fb_t *frame = NULL;

    while (true)
    {
        if (xQueueReceive(xQueueLCDFrame, &frame, portMAX_DELAY))
        {
            lcd_draw_bitmap(0, 0, frame->width, frame->height, (uint16_t *)frame->buf);
            esp_camera_fb_return(frame);
        }
    }
}

// 摄像头处理任务
static void task_process_camera(void *arg)
{
    while (true)
    {
        camera_fb_t *frame = esp_camera_fb_get();
        if (frame)
            xQueueSend(xQueueAIFrame, &frame, portMAX_DELAY);
    }
}
static void task_event_ai(void *arg)
{
    recognizer_state_t _gEvent;
    while (true)
    {
        xQueueReceive(xQueueEvent, &(_gEvent), portMAX_DELAY);
        // xSemaphoreTake(xMutex, portMAX_DELAY);
        gEvent = _gEvent;
        // xSemaphoreGive(xMutex);
    }
}
void app_camera_ai_lcd(void)
{
    xQueueLCDFrame = xQueueCreate(10, sizeof(camera_fb_t *));
    xQueueAIFrame = xQueueCreate(2, sizeof(camera_fb_t *));
    xQueueEvent = xQueueCreate(5, sizeof(recognizer_state_t));


    xTaskCreatePinnedToCore(task_process_camera, "task_process_camera",4 * 1024, NULL, 5, NULL, 1);      //创建摄像头任务
    xTaskCreatePinnedToCore(task_process_lcd, "task_process_lcd", 5 * 1024, NULL, 6, NULL, 0);         ///创建液晶屏任务
    xTaskCreatePinnedToCore(task_process_ai, "task_process_ai", 4 * 1024, NULL, 5, NULL, 1);   //创建AI人脸识别任务 
    if(xQueueEvent)
    {
       xTaskCreatePinnedToCore(task_event_ai, "task_event_ai", 4 * 1024, NULL, 5, NULL, 1);   //创建任务分类 
    }
}  
    