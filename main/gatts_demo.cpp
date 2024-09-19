/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/****************************************************************************
*
* This demo showcases BLE GATT server. It can send adv data, be connected by client.
* Run the gatt_client demo, the client demo will automatically connect to the gatt_server demo.
* Client demo will enable gatt_server's notify after connection. The two devices will then exchange
* data.
*
****************************************************************************/
// real code



#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"

#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "driver/adc.h"
#include "driver/rmt_tx.h"
#include "esp_adc_cal.h"


#include "led_strip_encoder.h"

#include "epaper.h"
#include "driver/timer.h"
#include "imagedata.h"

#include "sdkconfig.h"
#include "driver/ledc.h"
#define GATTS_TAG "GATTS_DEMO"
static const char *TAG = "BatteryLevel";
///Declare the static function
extern "C" {
    static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
}

QueueHandle_t batteryQueue;
QueueHandle_t imageQueue;
QueueHandle_t startQueue;

// 배터리 전압 측정 변수들
#define DEFAULT_VREF    1100        // ADC 기준 전압 (mV)
#define SAMPLES         64          // 멀티샘플링을 위한 샘플 수
#define BATTERY_PIN     ADC1_CHANNEL_0 // GPIO36 (VP)

// write 위해 추가
static int n = 0;
static uint16_t read_len;
char *read_v;
char *call_name = "call";
#define COLORED     0
#define UNCOLORED   1
// 모두 uuid로 구분
#define GATTS_SERVICE_UUID_TEST_A   0x00FF
// uuid에 따라 가끔 write가 안될 떄가 있음
#define GATTS_CHAR_UUID_TEST_A      0xFF01
#define GATTS_DESCR_UUID_TEST_A     0x3333
// handle 개수 (service 1개 + char 2개(char 선언 + char 값) + descr 1개 = 3개..?)
// 특성에서 handle이 2개인 경우에도 특성은 하나의 uuid 값을 가짐
#define GATTS_NUM_HANDLE_TEST_A     4

// 최대 문자열 길이를 정의. 예시에서는 20으로 설정했지만 필요에 따라 조정 가능
#define MAX_STR_LEN 20
static char TEST_DEVICE_NAME[MAX_STR_LEN] = "changed_before";
// #define TEST_DEVICE_NAME            "Changed_Before"
#define TEST_MANUFACTURER_DATA_LEN  17  // 안쓰나

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40 //64

// memory 크기
#define PREPARE_BUF_MAX_SIZE 5000

#define LEDC_TIMER          LEDC_TIMER_0    // LEDC를 사용하여 PWM 신호를 생성하기 위한 타이머 설정
#define LEDC_MODE           LEDC_LOW_SPEED_MODE // LEDC의 작동 모드 설정
#define LEDC_OUTPUT_IO      26  // PWM 신호를 출력할 GPIO 번호를 정의
#define LEDC_CHANNEL        LEDC_CHANNEL_0  // 사용할 LEDC의 채널을 정의. esp32는 여러 개의 채널 지원
                                            // 각각 독립적으로 설정하고 사용 가능

#define LEDC_DUTY           4096            // PWM 신호의 듀티 사이클 설정
#define MAX_NOTE            4               // 연주할 노트의 총 개수


// RMT 모듈의 해상도
#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
// GPIO핀
#define RMT_LED_STRIP_GPIO_NUM     static_cast<gpio_num_t>(21)
// LED 개수
#define EXAMPLE_LED_NUMBERS         6
// chase 속도
#define EXAMPLE_CHASE_SPEED_MS      10


//  도   레   미   파   솔   라   시    도 
// 262, 294, 330, 349, 392, 440, 494, 523 (해당 주파수)

const int note[MAX_NOTE] = {262, 330, 392, 523};

// static const char *TAG = "example";
// LED 스트립의 RGB 데이터를 저장할 배열
static uint8_t led_strip_pixels[EXAMPLE_LED_NUMBERS * 3];


// ?
static uint8_t char1_str[] = {0x11,0x22,0x33};
static esp_gatt_char_prop_t a_property = 0;
// static esp_gatt_char_prop_t b_property = 0;

// 현재 E-ink에서 처음으로 부분 이미지 업로드를 하고있는지
bool battery_first = true;
// 전체 이미지가 올라와 있는지
bool all_upload = false;

static int real_len;
uint8_t *get_data_all;
static int i = 0;
static int is_first = 0;
// static int initial_first = 0;
static const size_t MAX_LONG_BUFF = 600;
int final_data_len;
// character 1의 value 설정
static esp_attr_value_t gatts_demo_char1_val =
{   
    // value의 최대 길이?
    .attr_max_len = GATTS_DEMO_CHAR_VAL_LEN_MAX,
    .attr_len     = sizeof(char1_str),
    .attr_value   = char1_str,
};


// advertising data setting이 완료됐는지 나타내는 변수
static uint8_t adv_config_done = 0;

int battery_per;
int battery_temp = 50;


// 정수가 아닌 비트 형식으로 저장한 이유 : 코드의 가독성과 확장성을 정의하려고..?
#define adv_config_flag      (1 << 0)
#define scan_rsp_config_flag (1 << 1)

#ifdef CONFIG_SET_RAW_ADV_DATA
static uint8_t raw_adv_data[] = {
        0x02, 0x01, 0x06,                  // Length 2, Data Type 1 (Flags), Data 1 (LE General Discoverable Mode, BR/EDR Not Supported)
        0x02, 0x0a, 0xeb,                  // Length 2, Data Type 10 (TX power leve), Data 2 (-21)
        0x03, 0x03, 0xab, 0xcd,            // Length 3, Data Type 3 (Complete 16-bit Service UUIDs), Data 3 (UUID)
};
static uint8_t raw_scan_rsp_data[] = {     // Length 15, Data Type 9 (Complete Local Name), Data 1 (ESP_GATTS_DEMO)
        0x0f, 0x09, 0x45, 0x53, 0x50, 0x5f, 0x47, 0x41, 0x54, 0x54, 0x53, 0x5f, 0x44,
        0x45, 0x4d, 0x4f
};
#else

// 서비스 uuid (16-bit -> 128-bit)
// xxxxxxx - 0000 - 1000 - 8000 - 00805F9B34FB
static uint8_t adv_service_uuid128[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFE, 0x00, 0x00, 0x00,
    //second uuid, 32bit, [12], [13], [14], [15] is the value
};

// service가 a,b일 때
// static uint8_t adv_service_uuid128[32] = {
//     /* LSB <--------------------------------------------------------------------------------> MSB */
//     //first uuid, 16bit, [12],[13] is the value
//     0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xEE, 0x00, 0x00, 0x00,
//     //second uuid, 32bit, [12], [13], [14], [15] is the value
//     0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
// };

// The length of adv data must be less than 31 bytes
//static uint8_t test_manufacturer[TEST_MANUFACTURER_DATA_LEN] =  {0x12, 0x23, 0x45, 0x56};
//adv data
// 이 구조체를 사용해 광고 데이터 패킷 정의
// 주변 디바이스에게 제공하는 정보의 유형과 양 제어
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,  // response로 쓸건지?? - 안쓰면 그냥 페어링을 위한 광고 패킷?
    .include_name = true,   // 광고 데이터에 디바이스 이름 포함시킬건지
    .include_txpower = false,   
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    // min_interval이 작을 수록 더 자주 송출, 배터리 소모 높임
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    // max interval이 클수록 덜 자주 송출됨 + 연결 설정 시간이 길어짐
    // 이 간격을 0.625ms 단위로 설정
    .appearance = 0x00,     // 0x00이면 외관 설정 안함. ex) 0x0080 - 일반 컴퓨터, 0x0340 - 심박수 센서
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,  // service 데이터 길이와 데이터 자체 정의
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),    // service uuid 길이
    .p_service_uuid = adv_service_uuid128,              // service uuid
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),   // 표준 LE 광고 플래그 설정
    // ESP_BLE_ADV_FLAG_GEN_DISC는 일반 검색 모드 ( 이 모드에서 BLE 디바이스는 스캔하는 모든 디바이스에게 검색 가능)
    // 새로운 연결을 수락하거나 정보를 교환하기 위해 다른 디바이스와의 상호작용을 원할 때 사용
    // ESP_BLE_ADV_FLAG_BREDR_NOT_SPT는 BLE디바이스가 BR/EDR을 지원하지 않음 (Bluetooth 통신 방식)
    // -> 순수 ble 디바이스 임을 명시(듀얼 모드 디바이스가 아닐 때)

};





// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    //.min_interval = 0x0006,
    //.max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

#endif /* CONFIG_SET_RAW_ADV_DATA */
// 광고 매개변수 정의
// adv_int_min 과 min_interval 차이
// : min_inteval은 client 기준으로 얼마나 자주 보일지
// adv_int_min은 실제로 디바이스가 얼마나 자주 광고 패킷을 전송할건지
static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,       
    .adv_type           = ADV_TYPE_IND,     // 광고 타입 설정.
    // ADV_TYPE_IND : ble 디바이스를 누구나 스캔하고 연결을 시도할 수 있음
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC, // 공개 or 랜덤
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL, // 사용할 광고 채널 
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    // 광고 필터 정책 설정. 어떤 유형의 디바이스가 광고에 응답할 수 있는지 결정.
    // ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY : 모든 디바이스 허용
};

// #define PROFILE_NUM 2
#define PROFILE_NUM 1
#define PROFILE_A_APP_ID 0

// gatt profile 생성, 다양한 gatt 이벤트에 대한 처리 방식 정의
struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;    // gatt 서버 이벤트에 대한 콜백 함수(연결 상태 변경, 읽기/쓰기 요청 같은 다양한 이벤트에 대해 호출됨)
    uint16_t gatts_if;          // GATT interface 번호 - ESP GATT server와 통신을 위해 사용
    uint16_t app_id;            // application ID - 특정 GATT 프로필을 고유하게 식별
    uint16_t conn_id;           // 특정 BLE 연결을 식별
    uint16_t service_handle;    // 특정 GATT 서비스를 참조하는 데 사용
    esp_gatt_srvc_id_t service_id; // Gatt id, include uuid and instance + service가 private인지..
    uint16_t char_handle;       // Character handle - 특정 Gatt character 참조할 때
    esp_bt_uuid_t char_uuid;   // uuid16,32,128
    esp_gatt_perm_t perm;       // permission 정의
    esp_gatt_char_prop_t property;  
    uint16_t descr_handle; // gatt descriptor handle
    esp_bt_uuid_t descr_uuid;
};

struct e_paper_list {
    int width;
    int height;
};

// static e_paper_list epaper = {
//     .width = 400,
//     .height = 300,
// };

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
// profile 설정
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gatts_cb = gatts_profile_a_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,      // if 없음 /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

static prepare_type_env_t a_prepare_write_env;
// static prepare_type_env_t b_prepare_write_env;

extern "C" void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
extern "C" void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);

// E-ink 부분 업로드 코드
extern "C" void E_ink_upload(void *pvParameter){
    
    Epd epd;
    int wa=epd.width * epd.height / 8;
    unsigned char* all_image = (unsigned char*)malloc(wa);
    // unsigned char* image_temp = (unsigned char*)malloc(wa);
    // memset(image_temp, 1, wa);
    memset(all_image, 1, wa);
    while (1){

        // unsigned char* partial_image = (unsigned char*)malloc(100);
        
        // ESP_LOGI("EPD", "e-Paper");
        
        // 문제점 : 배터리 업로드하는 조건문이 충족되어도 코드가 작동 X, 전체 이미지 업로드하는 코드보다
        // 앞에 뒀을 때는 잘 실행이 됨
        // 원인 : xQueueReceive 함수가 두 번 호출될 때, 'batteryQueue'에 데이터가 도착하더라도
        // 첫 번째 'xQueueReceive'에서 'imageQueue'에 대한 대기를 무한정하게 된다.
        // 이로 인해 두 번째 : xQueueReceive는 실행할 기회를 얻지 못한다.
        // 수정 : 두 큐를 동시에 처리
        // ESP_LOGI("EPD", "1");
        // 전체 이미지를 업로드할 때
        if (xQueueReceive(imageQueue, &all_image, 100) == pdPASS) {
            vTaskDelay(pdMS_TO_TICKS(2000));
            ESP_LOGI("EPD", "IMAGE GET");
            //  같으면 0
            // if( memcmp(image_temp, all_image, wa) != 0 )
            // {
            //    memcpy(image_temp, all_image, wa);
            // }
            
            unsigned char* frame_ = (unsigned char*)malloc(epd.width * epd.height / 8);

            Paint paint_(frame_, epd.width, epd.height);
            paint_.Clear(UNCOLORED);
            vTaskDelay(pdMS_TO_TICKS(2000));

            ESP_LOGI("EPD", "e-Paper init and clear");
            vTaskDelay(pdMS_TO_TICKS(3000));

            // reset에 delay를 둬야 무한 딜레이가 안걸림
            epd.Init();
            ESP_LOGI("EPD", "e-Paper init");
            epd.Clear();
            ESP_LOGI("EPD", "Clear");

            vTaskDelay(pdMS_TO_TICKS(2000));

            epd.Display(all_image);
            ESP_LOGI("EPD", "Upload ALL_image");
            
            vTaskDelay(pdMS_TO_TICKS(3000));
            if (battery_temp==5){
            epd.Display_Partial(gImage_battery_1, 355, 12, 395, 32);  
        }
        
            else if (battery_temp==25){
                epd.Display_Partial(gImage_battery_2, 355, 12, 395, 32);  
        }
            else if (battery_temp==50){
                epd.Display_Partial(gImage_battery_3, 355, 12, 395, 32);  
        }
            else if (battery_temp==75){
                epd.Display_Partial(gImage_battery_4, 355, 12, 395, 32);  
        }
            else {
                epd.Display_Partial(gImage_battery_5, 355, 12, 395, 32);  
        }
        ESP_LOGI("EPD", "battery upload");
        // epd.Display_Partial(gImage_battery_2, 355, 12, 395, 32);
        vTaskDelay(pdMS_TO_TICKS(2000));
        epd.Sleep();
        free(frame_);
        vTaskDelay(pdMS_TO_TICKS(2000));


        }
    // 배터리 업로드해야할 때
        else if (xQueueReceive(batteryQueue, &battery_per, 100) == pdPASS){
            
            ESP_LOGI("EPD", "2");

            // init 함수에서 ReadBusy함수에 있는 while문 때문에 타임독 문제가 생김.
            // 그래서 10ms의 딜레이를 주어 해결함
        
            if (battery_per != battery_temp){
                vTaskDelay(pdMS_TO_TICKS(1000));

                unsigned char* frame_ = (unsigned char*)malloc(epd.width * epd.height / 8);
                Paint paint_(frame_, epd.width, epd.height);
                paint_.Clear(UNCOLORED);

                ESP_LOGI("EPD", "battery upload");
                vTaskDelay(pdMS_TO_TICKS(2000));

                epd.Init();
                epd.Clear();

                vTaskDelay(pdMS_TO_TICKS(2000));
                epd.Display(all_image);
                battery_temp = battery_per;
                vTaskDelay(pdMS_TO_TICKS(2000));

                if (battery_temp==5){
                epd.Display_Partial(gImage_battery_1, 355, 12, 395, 32);  
                }   
                
                else if (battery_temp==25){
                    epd.Display_Partial(gImage_battery_2, 355, 12, 395, 32);  
                }
                else if (battery_temp==50){
                    epd.Display_Partial(gImage_battery_3, 355, 12, 395, 32);  
                }
                else if (battery_temp==75){
                    epd.Display_Partial(gImage_battery_4, 355, 12, 395, 32);  
                }
                else {
                    epd.Display_Partial(gImage_battery_5, 355, 12, 395, 32);  
                }
            // epd.Display_Partial(gImage_battery_2, 355, 12, 395, 32);
                vTaskDelay(pdMS_TO_TICKS(2000));
                epd.Sleep();
                vTaskDelay(pdMS_TO_TICKS(2000));
                free(frame_);
                ESP_LOGI("EPD", "UPLOAD PARTIAL END");
            }
        }
        
        // ESP_LOGI("EPD", "3");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
}
extern "C" int calculateBatteryPercentage(int voltage) {
    ESP_LOGI(TAG, "calculateBatteryPercentage : %dV", voltage);
    // ADC 읽기 값을 실제 전압으로 변환
    if (voltage <= 3.500) {
        return 5;   // 5%
    } else if (voltage <= 3.730) {
        return 25;  // 25%
    } else if (voltage <= 3.850) {
        return 50;  // 50%
    } else if (voltage <= 3.950) {
        return 75;  // 75%
    } else if (voltage <= 4.200) {
        return 100; // 100%
    } else {
        return 100; // 4.2V 이상일 경우 100%로 가정
    }
}


extern "C" void checkBatteryLevel(void *pvParameter){
    // esp_adc_cal_characteristics_t *adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characteristics_t *adc_chars = static_cast<esp_adc_cal_characteristics_t*>(calloc(1, sizeof(esp_adc_cal_characteristics_t)));
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(BATTERY_PIN, ADC_ATTEN_DB_11);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);

    while (1) {
    uint32_t adc_reading = 0;
    // SAMPLES 수 만큼 ADC를 읽고 평균값 계산
    for (int i = 0; i < SAMPLES; i++) {
        adc_reading += adc1_get_raw((adc1_channel_t)BATTERY_PIN);
    }
    adc_reading /= SAMPLES;

    float voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    ESP_LOGI(TAG, "Measured Voltage : %.1fV", voltage/1000);
    const float R1 = 330.0;
    const float R2 = 1000.0;
    float real_voltage = (voltage * ((R1 + R2) / R2)) / 1000;


    // 배터리 잔량 계산
    int battery_percentage = calculateBatteryPercentage((int)real_voltage);
    ESP_LOGI(TAG, "Real Battery Voltage : %.1f V, Battery Level : %d%%", real_voltage, battery_percentage);


    if (xQueueSend(batteryQueue, &battery_percentage, portMAX_DELAY) == pdPASS) {
        ESP_LOGW(TAG, "Battery good send");
        }
    

      vTaskDelay(1000*60*30); // 30분 대기
    }


}

// LED 코드
extern "C" void play_note(void *pvParameter){
    uint8_t get_start;
    ledc_timer_config_t ledc_timer = {
    // PWM 신호 생성에 사용될 타이머 설정 정의
    .speed_mode = LEDC_MODE,    // PWM 신호 생성 시 사용할 속도 모드. 여기서는 저속 모드 사용
    .duty_resolution = LEDC_TIMER_13_BIT,   // 듀티 사이클의 해상도를 지정 0~8191까지 가능
    .timer_num = LEDC_TIMER,    // 제공하는 여러 타이머 중 첫번째 타이머 설정
    .freq_hz = 262,     // 초기 주파수 설정, 이후 변경됨
    .clk_cfg = LEDC_AUTO_CLK,   // 클락 소스 설정 - 자동 클락 구성을 의미
    };
    ledc_timer_config(&ledc_timer); // 위에서 정의한 타이머 설정으로 LEDC 타이머를 구성


    ledc_channel_config_t ledc_channel = {
    .gpio_num = LEDC_OUTPUT_IO,
    .speed_mode = LEDC_MODE,
    .channel = LEDC_CHANNEL,
    .timer_sel = LEDC_TIMER,
    .duty = LEDC_DUTY,
    };
    ledc_channel_config(&ledc_channel);



    int lastFrequency = 0;  // 이전에 연주된 음의 주파수 저장할 변수
    while (true){

        if (xQueueReceive(startQueue, &get_start, portMAX_DELAY) == pdPASS) {
            ESP_LOGE("EPD", "연주시작");
            for (int i=0; i < MAX_NOTE; i++){
                if (note[i] == lastFrequency){  // 현재 음계가 이전 음계와 같은 경우
                    ledc_stop(LEDC_MODE, LEDC_CHANNEL, 0);  // 현재 채널의 PWM 출력을 중지
                    vTaskDelay(50 / portTICK_PERIOD_MS);    // 50ms 대기 같은 음계 사이의 구분을 위해
                }
                ledc_set_freq(LEDC_MODE, LEDC_TIMER, note[i]);  // LEDC 타이머의 주파수를 현재 음계로 설정
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);  // 현재 채널의 듀티 사이클을 설정
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);      // 변경된 듀티 사이클 설정을 적용

                vTaskDelay(500 / portTICK_PERIOD_MS);   // 음계와 음계 사이 500ms 대기
                lastFrequency = note[i];                // lastFrequency를 현재 음계로 업데이트
            }
        }

    ledc_stop(LEDC_MODE, LEDC_CHANNEL, 0);      // 연구가 끝난 후 정지

    }
}


extern "C" void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b)
{
    h %= 360; // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
    case 0:
        *r = rgb_max;
        *g = rgb_min + rgb_adj;
        *b = rgb_min;
        break;
    case 1:
        *r = rgb_max - rgb_adj;
        *g = rgb_max;
        *b = rgb_min;
        break;
    case 2:
        *r = rgb_min;
        *g = rgb_max;
        *b = rgb_min + rgb_adj;
        break;
    case 3:
        *r = rgb_min;
        *g = rgb_max - rgb_adj;
        *b = rgb_max;
        break;
    case 4:
        *r = rgb_min + rgb_adj;
        *g = rgb_min;
        *b = rgb_max;
        break;
    default:
        *r = rgb_max;
        *g = rgb_min;
        *b = rgb_max - rgb_adj;
        break;
    }
}


// esp_gap_ble_cb_event_t : 개개개개개많은 event가 들어있음 모든 경우 다 들어있는듯 ?ㅋ
extern "C"{
    static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
    {
        switch (event) {
    #ifdef CONFIG_SET_RAW_ADV_DATA
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~adv_config_flag);
            if (adv_config_done==0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~scan_rsp_config_flag);
            if (adv_config_done==0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
    #else
        // advertising data setting이 다 됐을 때
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~adv_config_flag);
            if (adv_config_done == 0){
                // advertising 시작
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        // scan response data sentting이 다 됐을 때 
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~scan_rsp_config_flag);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
    #endif
        // 광고 시작이 완료됐을 떄 광고가 성공적으로 수행됐는지 표시
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            //advertising start complete event to indicate advertising start successfully or failed
            // adv_start_cmpl : advertising이 성공적으로 수행됐는지 나타냄
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TAG, "Advertising start failed\n");
            }
            break;
        // 광고 시작을 멈췄을 떄 광고가 성공적으로 수행됐는지 표시
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TAG, "Advertising stop failed\n");
            } else {
                ESP_LOGI(GATTS_TAG, "Stop adv successfully\n");
            }
            break;
        // connection parameter 업데이트 완료 됐을 때
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(GATTS_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                    param->update_conn_params.status, // 업데이트 완료 됐는지 상태
                    param->update_conn_params.min_int,    // 최소 연결 간격
                    param->update_conn_params.max_int,    // 최대 연결 간격
                    param->update_conn_params.conn_int,   // 현재 연결 간격
                    param->update_conn_params.latency,    // 연결 이벤트 수에 대한 slave 지연 시간
                    param->update_conn_params.timeout);
            break;
        default:
            break;
        }
    }
}
// 긴 데이터를 보낼 때는 prepare write response를 사용하여 여러 청크로 나눈 후 executive write request를
// 사용하여 전체 쓰기 요청을 확인하거나 취소함
// prepare일 때의 데이터는 최종적으로 보내지않고 임시 저장됨
// excute -> 모든 데이터 조각이 전송되고 준비 완료되면,
//  ESP_GATT_PREP_WRITE_EXEC 명령을 보내어 준비된 모든 쓰기 작업을 실행하도록 요청
// -> 모든 임시 데이터를 실제 특성에 씀
extern "C" 
{
    void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
        esp_gatt_status_t status = ESP_GATT_OK;
        // write.need_rsp : 클라이언트가 데이터를 쓰고 서버로부터의 확인 응답을 받기를 원하는 경우 (성공적으로 쓰기 요청이 완료됐는지)
        if (param->write.need_rsp){
            // 쓰기 요청의 성공 여부를 판단
            if (param->write.is_prep){
                // ? write가 쓰려는 prepare 메모리가 비어있을 때?
                if (prepare_write_env->prepare_buf == NULL) {
                    // 메모리 만들기
                    prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE*sizeof(uint8_t));
                    // 아직 write한게 없으므로 길이가 0?
                    prepare_write_env->prepare_len = 0;
                    // 할당할 수 있는 메모리가 gatt_server에 남아있지 않다?
                    if (prepare_write_env->prepare_buf == NULL) {
                        ESP_LOGE(GATTS_TAG, "Gatt_server prep no mem\n");
                        // ESP_GATT_NO_RESOURCES = 메모리부족, 리소스 한계 도달, 동시 작업 한계
                        status = ESP_GATT_NO_RESOURCES;
                    }
                } else {
                    // write.offset => 주어진 버퍼에서 어느 시작점을 가지고 있는지?
                    // offset : 데이터 write를 시작할 버퍼 내의 위치
                    // write response가 버퍼의 허용된 최대 크기를 벗어나고 있는지
                    // (write의 작업 시작 지점 자체가 유효한 범위 내에 있는지)
                    if(param->write.offset > PREPARE_BUF_MAX_SIZE) {
                        status = ESP_GATT_INVALID_OFFSET;
                    // (작업 전체가 버퍼를 넘지 않는지)
                    } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
                        status = ESP_GATT_INVALID_ATTR_LEN;
                    }
                }

                // Write req 값 atribute에 저장중
                esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
                gatt_rsp->attr_value.len = param->write.len;
                gatt_rsp->attr_value.handle = param->write.handle;
                gatt_rsp->attr_value.offset = param->write.offset;
                // gatt 인증 request(ESP_GATT_AUTH_REQ_NONE : 인증 요구 x - 추가적인 보안 절차나
                // 인증 과정 없이 수행 가능)
                gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
                memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
                // write request에 대한 response를 client에게 전송
                // write하려고 보낸 값 다시 응답 받음??
                esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
                // 전송 x
                if (response_err != ESP_OK){
                ESP_LOGE(GATTS_TAG, "Send response error\n");
                }
                // response 메모리 비우기
                free(gatt_rsp);
                if (status != ESP_GATT_OK){
                    return;
                }
                // 메모리 복사 함수
                // prepare_write_env->prepare_buf : 데이터가 저장될 buffer
                // param->write.offset : 버퍼 내에서 데이터 write를 시작할 위치
                // param->write.value에서 시작하는 param->write.len 바이트의 데이터를
                // prepare_write_env->prepare_buf 버퍼의 param->write.offset 위치부터 시작하는 곳에 복사
                memcpy(prepare_write_env->prepare_buf + param->write.offset,
                    param->write.value,
                    param->write.len);

                prepare_write_env->prepare_len += param->write.len;
    // 재요청
            }else{
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
            }
        }
    }
}
// execute write request
// 이전에 수행된 쓰기 프로시저를 확인하거나 취소하는데 사용
extern "C" 
{
    void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
        // preapare가 끝났는지 (모든 데이터 조각이 전송됐는지)
        if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC){
            // write 값 16진수로 출력

            ESP_LOGI(GATTS_TAG, "length : %d",prepare_write_env->prepare_len);
            // esp_log_buffer_hex(GATTS_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);

            if (is_first==0){
                real_len = 15000;
                get_data_all = (uint8_t*) malloc(real_len);
                memset(get_data_all, 0, real_len);
                // memcpy 쓸 때 특정 값부터 가져오는게 아니라면 &붙이지 않고 사용하기!!
                // &get_data_all[0]는 get_data_all[0]의 주소
                memcpy(&get_data_all[0], prepare_write_env->prepare_buf, MAX_LONG_BUFF); 
                is_first++;
                i++;
                ESP_LOGW(GATTS_TAG, " First piece");
                esp_log_buffer_hex(GATTS_TAG, prepare_write_env->prepare_buf, 600);
            }
            else{
                // memset(get_data_all, 0, real_len);
                ESP_LOGW(GATTS_TAG,"%d start point",MAX_LONG_BUFF*i);
                if (i == 24){
                    
                    unsigned char* get_image = (unsigned char*)malloc(real_len);
                    memcpy(&get_data_all[MAX_LONG_BUFF*i], prepare_write_env->prepare_buf, MAX_LONG_BUFF);
                    memcpy(get_image, get_data_all, real_len);
                    ESP_LOGW(GATTS_TAG, " Final length %d", sizeof(get_data_all));
                    ESP_LOGW(GATTS_TAG, "마지막으로 저장된거");
                    vTaskDelay(pdMS_TO_TICKS(3000));
                    if (xQueueSend(imageQueue, &get_image, portMAX_DELAY) == pdPASS){
                        ESP_LOGW(GATTS_TAG, "Queue 이미지 전송 완료");
                    }

                    i = 0;
                    is_first = 0;
                    free(get_data_all);
                                

                    ESP_LOGI(GATTS_TAG,"here");
                    free(get_image);
                }
                else{
                    memcpy(&get_data_all[(MAX_LONG_BUFF*i)], prepare_write_env->prepare_buf, MAX_LONG_BUFF);   
    
                    ESP_LOGW(GATTS_TAG, " next%d piece",MAX_LONG_BUFF*i);
                    i++;
                }
                

            }

        }else{
            // 쓰기가 취소되고 모든 데이터 삭제
            ESP_LOGI(GATTS_TAG,"ESP_GATT_PREP_WRITE_CANCEL");
        }
        // 메모리에 데이터가 있다면 메모리 해제
        // write 작업 끝
        if (prepare_write_env->prepare_buf) {
            free(prepare_write_env->prepare_buf);
            prepare_write_env->prepare_buf = NULL;
        }
        prepare_write_env->prepare_len = 0;
    }
}
// application profile 저장
extern "C" 
{
    static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
        switch (event) {
        // application id가 등록되었을 때
        case ESP_GATTS_REG_EVT:{
            ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
            // service_id는 primary
            gl_profile_tab[PROFILE_A_APP_ID].service_id.is_primary = true;
            // Gatt id, include uuid and instance
            // profile에 하나의 service만 있고 service의 uuid가 달라서 service inst_id를 0으로 설정 가능??
            gl_profile_tab[PROFILE_A_APP_ID].service_id.id.inst_id = 0x00;
            gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
            // 16-bit uuid
            gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST_A;

            // ! 디바이스 이름 정하는 구간
            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(TEST_DEVICE_NAME);
            // 실패했을 때 = 1
            if (set_dev_name_ret){
                ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
            }
    #ifdef CONFIG_SET_RAW_ADV_DATA
            esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
            if (raw_adv_ret){
                ESP_LOGE(GATTS_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
            }
            adv_config_done |= adv_config_flag;
            esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
            if (raw_scan_ret){
                ESP_LOGE(GATTS_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
            }
            adv_config_done |= scan_rsp_config_flag;
    #else
            //config adv data
            // ble 장치가 주변 장치에게 자신을 알리는 광고 패킷 내용 설정
            esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
            // 광고 패킷 내용 설정 실패시
            if (ret){
                ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
            }
            // 00 | 01 = 01
            adv_config_done |= adv_config_flag;
            //config scan response data
            ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
            if (ret){
                ESP_LOGE(GATTS_TAG, "config scan response data failed, error code = %x", ret);
            }
            // 01 | 10 = 11
            adv_config_done |= scan_rsp_config_flag;

    #endif
            // service 만듦 -> 만들어진 후 ESP_GATTS_CREATE_EVT 호출됨 -> status 및 service를 profile에 보고
            esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_A_APP_ID].service_id, GATTS_NUM_HANDLE_TEST_A);
            break;
            }
        // client가 read req할 때
        case ESP_GATTS_READ_EVT: {
            // conn_id : 현재 연결의 식별자(어떤 client와 연결되어있는지)
            // -> 정수형.
            // -> ble 장치가 여러 기기와 동시에 연결되어 있을 떄, 각 연결은 고유한 conn_id 가짐
            // -> 특정 연결 세션 식별(connect가 끊기면 conn_id는 더 이상 유효 x)
            // -> 새로운 연결이 형성될 때마다 새로운 conn_id가 할당됨
            // trans_id : 거래 식별자??
            // -> 한 연결 내에서 특정 요청-응답 쌍을 식별하는 데 사용
            // -> client가 server에 요청을 보낼 때마다, 각 요청은 고유한 trans_id 가짐
            // (client는 같은 trans_id를 사용하여 resp를 원래의 req과 연결)
            // handle : clinet가 read req를 요청한 char 또는 desc의 handle. 사용자가 어떤 데이터를 요청했는지
            ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, conn_id %d, trans_id %" PRIu32 ", handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
            esp_gatt_rsp_t rsp;
            // gatt atttibute value, handle reset
            // 0으로 채운
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            rsp.attr_value.handle = param->read.handle;
            // rsp.attr_value.len = 4;
            // 보낼 데이터 저장
            // rsp.attr_value.value[0] = 0xde;
            // rsp.attr_value.value[1] = 0xed;
            // rsp.attr_value.value[2] = 0xbe;
            // rsp.attr_value.value[3] = 0xef;

            if (n==0){
                rsp.attr_value.len = 1;
                rsp.attr_value.value[0] = 0x01;
                n++;
            }
            else{
                rsp.attr_value.len = read_len;
                memcpy(rsp.attr_value.value, read_v, read_len);
                ESP_LOGI(GATTS_TAG, "한숨을 수지 마세소");

                esp_log_buffer_hex(GATTS_TAG, rsp.attr_value.value, read_len);
            }
            // client의 read 요청에 response 전송
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                        ESP_GATT_OK, &rsp);
            break;
        }
        case ESP_GATTS_WRITE_EVT: {
            // trans_id : client가 서버에서 특정 characteristic을 read하거나 write 요청을 보낼 때
            ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %" PRIu32 ", handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
            // 긴 write 작업이 아닐 경우(mtu보다 작을 때)
            if (!param->write.is_prep){
                ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, value len %d, value :", param->write.len);
                // ESP_LOGI(GATTS_TAG,"%s", param->write.value);
                // client로부터 받은 데이터 출력
                esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
                ESP_LOGE(GATTS_TAG, "짧은 write 시도");

                if (read_v != NULL){
                    ESP_LOGE(GATTS_TAG, "read_v -> NULL");
                    free(read_v);   // 이전에 할당된 메모리를 해제
                    read_v = NULL;  // 해제 후 NULL로 설정
                }

                // write한 값을 다시 read할 때 할당(ESP_GATTS_READ_EVT, else문)
                // 클라이언트로부터 받은 값을 read_v에 동적 할당 및 복사
                read_v = (char *)malloc(param->write.len);
                
                 ESP_LOGE(GATTS_TAG, "%d", param->write.len);
                //   ESP_LOGE(GATTS_TAG, "%d", strlen(read_v));
                  ESP_LOGE(GATTS_TAG, "%d", sizeof(read_v));
                if (read_v != NULL){
                    ESP_LOGE(GATTS_TAG, "read_v에 이름 넣는중");
                    memcpy((char *)read_v, param->write.value, param->write.len);
                    read_len = param->write.len; // 쓰기 요청의 길이를 read_len에 저장
                } else{
                    ESP_LOGE(GATTS_TAG, "Failed to allocate memory for read_v");
                }
                
                if (read_v != NULL){
                    // 받은 값이 TEST_DEVICE_NAME 이랑 같을 경우(호출)
                    ESP_LOGW(GATTS_TAG,"read_v: %s", read_v);
                    ESP_LOGW(GATTS_TAG,"call_name: %s", call_name);

                    for (int i = 0; i < strlen(read_v); i++) {
                        ESP_LOGW(GATTS_TAG,"read_v[%d]: %c, call_name[%d]: %c", i, read_v[i], i, call_name[i]);
                    }

                    if( strncmp(read_v, "call", 4) == 0 )
                    {
                        ESP_LOGE(GATTS_TAG, "호출!");
                        uint8_t start = 1;
                        // 부저 재생
                        // play_note();
                        if (xQueueSend(startQueue, &start, portMAX_DELAY) == pdPASS) {
                            ESP_LOGW(TAG, "play music");
                            start = 0;
                        }

                        // LED 제어
                        int time_set = 0;

                        // LED 설정
                        uint32_t red = 0;
                        uint32_t green = 0;
                        uint32_t blue = 0;
                        uint16_t hue = 0;
                        uint16_t start_rgb = 0;

                        ESP_LOGI(TAG, "Create RMT TX channel");
                        rmt_channel_handle_t led_chan = NULL;

                        // RMT TX 채널을 생성하고 초기화.
                        rmt_tx_channel_config_t tx_chan_config = {
                            .gpio_num = RMT_LED_STRIP_GPIO_NUM,
                            .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
                            .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
                            .mem_block_symbols = 64, // increase the block size can make the LED less flickering
                            .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
                        };
                        
                        ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));

                        ESP_LOGI(TAG, "Install led strip encoder");

                        rmt_encoder_handle_t led_encoder = NULL;
                        led_strip_encoder_config_t encoder_config = {
                            .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
                        };

                        ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder));

                        ESP_LOGI(TAG, "Enable RMT TX channel");
                        ESP_ERROR_CHECK(rmt_enable(led_chan));

                        ESP_LOGI(TAG, "Start LED rainbow chase");
                        rmt_transmit_config_t tx_config = {
                            .loop_count = 0, // no transfer loop
                        };


                    
                        while (1) {
                            ESP_LOGE(GATTS_TAG, "LED 제어");
                            for (int i = 0; i < 3; i++) {
                                for (int j = i; j < EXAMPLE_LED_NUMBERS; j += 3) {
                                    // Build RGB pixels
                                    hue = j * 360 / EXAMPLE_LED_NUMBERS + start_rgb;
                                    led_strip_hsv2rgb(hue, 100, 100, &red, &green, &blue);
                                    led_strip_pixels[j * 3 + 0] = green;
                                    led_strip_pixels[j * 3 + 1] = blue;
                                    led_strip_pixels[j * 3 + 2] = red;
                                }
                                // Flush RGB values to LEDs
                                ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
                                ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
                                vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
                                memset(led_strip_pixels, 0, sizeof(led_strip_pixels));
                                ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
                                ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
                                vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
                            }
                            time_set++;
                            start_rgb += 60;
                            if (time_set == 10*10){
                                break;
                            }
                        }

                    }
                else{
                    // 기존 이름 제거
                    memset(TEST_DEVICE_NAME, '\0', MAX_STR_LEN);
                    // 기기 이름 사용자 정의
                    for (int i = 0; i < read_len; i++){
                        TEST_DEVICE_NAME[i] = read_v[i];    // ASCII 코드 값을 문자로 변환하여 저장
                    }
                    ESP_LOGI(GATTS_TAG, "RENAME value : %s", TEST_DEVICE_NAME);
                    // 기기 이름 변경
                    esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(TEST_DEVICE_NAME);
                    ESP_LOGI(GATTS_TAG, "Set name");
                    if (set_dev_name_ret){
                        ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
                    }
                    esp_err_t ret;
                    ESP_LOGI(GATTS_TAG, "Stop Advertising");
                    ret = esp_ble_gap_stop_advertising();
                    if (ret != ESP_OK){
                        ESP_LOGE(GATTS_TAG, "Stopping advertising failed : %s", esp_err_to_name(ret));
                    }
                    ret = esp_ble_gap_config_adv_data(&adv_data);
                    if (ret != ESP_OK) {
                        ESP_LOGE(GATTS_TAG, "Configuring advertising data failed: %s", esp_err_to_name(ret));
                    }
                    ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
                    if (ret != ESP_OK) {
                        ESP_LOGE(GATTS_TAG, "Configuring scan response data failed: %s", esp_err_to_name(ret));
                    }
                    // 광고를 재시작합니다.
                    ESP_LOGI(GATTS_TAG, "Start Advertising");
                    ret = esp_ble_gap_start_advertising(&adv_params);
                    if (ret != ESP_OK) {
                        ESP_LOGE(GATTS_TAG, "Starting advertising failed: %s", esp_err_to_name(ret));
                    }          
                }
                
                // descr handle이랑 write handle이랑 같을 때(cccd 설정하려고 하는지!!!)
                // + value 길이가 2일 때?? (와 대박 cccd는 길이가 2바이트야)
                // 내가 보기에 이건 cccd를 의미하는 것 같다(너가 맞았어 멋지다 크카카카카)
                if (gl_profile_tab[PROFILE_A_APP_ID].descr_handle == param->write.handle && param->write.len == 2)
                {
                    // client 보고 확인해보자(계산)
                    uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                    if (descr_value == 0x0001){
                        if (a_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY){
                            ESP_LOGI(GATTS_TAG, "notify enable");
                            // ! 보내려는 데이터
                            uint8_t notify_data[15];
                            for (int i = 0; i < sizeof(notify_data); ++i)
                            {
                                notify_data[i] = i%0xff;
                            }
                            //the size of notify_data[] need less than MTU size
                            esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                    sizeof(notify_data), notify_data, false);
                        }
                    }else if (descr_value == 0x0002){
                        if (a_property & ESP_GATT_CHAR_PROP_BIT_INDICATE){
                            ESP_LOGI(GATTS_TAG, "indicate enable");
                            uint8_t indicate_data[15];
                            for (int i = 0; i < sizeof(indicate_data); ++i)
                            {
                                indicate_data[i] = i%0xff;
                            }
                            //the size of indicate_data[] need less than MTU size
                            esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                    sizeof(indicate_data), indicate_data, true);
                        }
                    }
                    else if (descr_value == 0x0000){
                        ESP_LOGI(GATTS_TAG, "notify/indicate disable ");
                    }else{
                        ESP_LOGE(GATTS_TAG, "unknown descr value");
                        esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
                    }
                }
                }
            }
            // MTU보다 큰 데이터를 보낼 때
            example_write_event_env(gatts_if, &a_prepare_write_env, param);
                    // 기기 이름 변경 후 광고 재시작
            

            break;
        }

        // 실행 쓰기(Executed Write) 요청을 받았을 때, 
        case ESP_GATTS_EXEC_WRITE_EVT:
            ESP_LOGI(GATTS_TAG,"ESP_GATTS_EXEC_WRITE_EVT");
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
            example_exec_write_event_env(&a_prepare_write_env, param);
            break;
        // mtu 설정이 완료됐을 때?
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(GATTS_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            break;
        // 등록되지않은 application id일 때
        case ESP_GATTS_UNREG_EVT:
            break;
        // service 생성할 때
        case ESP_GATTS_CREATE_EVT:{
            // status -> 괜찮은지 오류났는지 (ex.esp_ok)
            ESP_LOGI(GATTS_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);
            gl_profile_tab[PROFILE_A_APP_ID].service_handle = param->create.service_handle;
            gl_profile_tab[PROFILE_A_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
            gl_profile_tab[PROFILE_A_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TEST_A;

            esp_ble_gatts_start_service(gl_profile_tab[PROFILE_A_APP_ID].service_handle);
            // property 설정 -> read, write, notify
            a_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
            // 서비스에 특성 추가하는 함수 (포함된 서비스가 추가될 서비스 핸들, cha- uuid, Characteristic value declaration attribute permission,
            // Characteristic Properties, Characteristic value, attribute response control byte ) 
            esp_err_t add_char_ret = esp_ble_gatts_add_char(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].char_uuid,
                                                            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                            a_property,
                                                            &gatts_demo_char1_val, NULL);
            // service에 char 추가 안됐을 때
            if (add_char_ret){
                ESP_LOGE(GATTS_TAG, "add char failed, error code =%x",add_char_ret);
            }
            break;
        }
        // service 포함이 완료됐을 때
        case ESP_GATTS_ADD_INCL_SRVC_EVT:
            break;
        // char 추가 완료됐을 때
        // 추가된 char에 대해 stack에서 생성된 handle을 반환하는 이벤트 트리거
        case ESP_GATTS_ADD_CHAR_EVT: {
            uint16_t length = 0;
            // char desc의 초기값은 NULL pointer가 될 수 있으며 자동 응답 매개변수도 NULL로 설정
            // -> response가 필요한 resq에 수동으로 응답해야 함을 의미?
            const uint8_t *prf_char;

            ESP_LOGI(GATTS_TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d\n",
                    param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
            // char의 handle 설정 및 descr uuid 설정
            gl_profile_tab[PROFILE_A_APP_ID].char_handle = param->add_char.attr_handle;
            gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
            gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
            // GATT 서버의 특정 attribute에 저장된 value를 검색
            // (값을 검색하고자 하는 char의 handle, 검색된 char value가 저장될 메모리 위치를 가리키는 포인터,
            // char 값의 길이를 저장할 변수의 주소)
            // ?? 이걸 왜 하지??
            esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(param->add_char.attr_handle,  &length, &prf_char);
            if (get_attr_ret == ESP_FAIL){
                ESP_LOGE(GATTS_TAG, "ILLEGAL HANDLE");
            }
            
            // value 값 출력
            ESP_LOGI(GATTS_TAG, "the gatts demo char length = %x\n", length);
            for(int i = 0; i < length; i++){
                ESP_LOGI(GATTS_TAG, "prf_char[%x] =%x\n",i,prf_char[i]);
            }
            // char에 descr value 추가하는 곳 ??
            // (des를 추가할 서비스 handle, 추가할 des의 uuid, des에 대한 접근 permission,data or length)
            esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].descr_uuid,
                                                                    ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
            if (add_descr_ret){
                ESP_LOGE(GATTS_TAG, "add char descr failed, error code =%x", add_descr_ret);
            }
            break;
        }

        // descriptor가 추가됐을 때
        case ESP_GATTS_ADD_CHAR_DESCR_EVT:
            // descr의 handle 설정
            gl_profile_tab[PROFILE_A_APP_ID].descr_handle = param->add_char_descr.attr_handle;
            ESP_LOGI(GATTS_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n",
                    param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
            break;

        // service 삭제할 때  
        case ESP_GATTS_DELETE_EVT:
            break;
        // service 시작했을 때
        case ESP_GATTS_START_EVT:
            ESP_LOGI(GATTS_TAG, "SERVICE_START_EVT, status %d, service_handle %d\n",
                    param->start.status, param->start.service_handle);
            break;
        // 서비스 중단
        case ESP_GATTS_STOP_EVT:
            break;

        // client와 connect됐을 때
        case ESP_GATTS_CONNECT_EVT: {
            esp_ble_conn_update_params_t conn_params = {0};
            // 원격 bluetooth 맥주소
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
            ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
                    param->connect.conn_id,
                    param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                    param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
            gl_profile_tab[PROFILE_A_APP_ID].conn_id = param->connect.conn_id;
            //start sent the update connection parameters to the peer device.
            esp_ble_gap_update_conn_params(&conn_params);
            break;
        }
        // 연결이 끊겼을 때
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(GATTS_TAG, "ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x", param->disconnect.reason);
            // 다시 advertising 시작
            esp_ble_gap_start_advertising(&adv_params);
            break;
        
        // confirm을 받았을 때 (보통 indication에 대한 respon??)
        case ESP_GATTS_CONF_EVT:
            ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONF_EVT, status %d attr_handle %d", param->conf.status, param->conf.handle);
            if (param->conf.status != ESP_GATT_OK){
                esp_log_buffer_hex(GATTS_TAG, param->conf.value, param->conf.len);
            }
            break;
        case ESP_GATTS_OPEN_EVT:
        case ESP_GATTS_CANCEL_OPEN_EVT:
        case ESP_GATTS_CLOSE_EVT:
        case ESP_GATTS_LISTEN_EVT:
        case ESP_GATTS_CONGEST_EVT:
        default:
            break;
        }
    }
}

extern "C" 
{
    static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
    {
        /* If event is register event, store the gatts_if for each profile */
        if (event == ESP_GATTS_REG_EVT) {
            if (param->reg.status == ESP_GATT_OK) {
                gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
            } else {
                ESP_LOGI(GATTS_TAG, "Reg app failed, app_id %04x, status %d\n",
                        param->reg.app_id,
                        param->reg.status);
                return;
            }
        }

        /* If the gatts_if equal to profile A, call profile A cb handler,
        * so here call each profile's callback */
        do {
            int idx;
            for (idx = 0; idx < PROFILE_NUM; idx++) {
                // 아직 어떤 디바이스와도 연결이 되지않았을 때
                if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                        gatts_if == gl_profile_tab[idx].gatts_if) {
                    if (gl_profile_tab[idx].gatts_cb) {
                        gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                    }
                }
            }
        } while (0);
    }
}

// 응용 프로그램이 시작될 때 자동으로 호출됨(c언어의 main과 같음)
// 이벤트 루프를 생성
extern "C" void app_main() 
{
    esp_err_t ret;

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        return;
    }
    // ret = esp_ble_gatts_app_register(PROFILE_B_APP_ID);
    // if (ret){
    //     ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
    //     return;
    // }
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    batteryQueue = xQueueCreate(10, sizeof(int));
    imageQueue = xQueueCreate(10, sizeof(int));
    startQueue = xQueueCreate(1, sizeof(uint8_t));
    if (batteryQueue == NULL) {
        ESP_LOGE(TAG, "Failed to create queue");
        return;
    }

    if (xTaskCreate(checkBatteryLevel, "checkBatteryLevel", 2048, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create checkBatteryLevel task");
        return;
    }
    if (xTaskCreate(E_ink_upload, "E_ink_upload", 4096, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create E_ink_upload task");
        return;
    }
    if (xTaskCreate(play_note, "play_note", 4096, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create E_ink_upload task");
        return;
    }
}
