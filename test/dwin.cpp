#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>

// Định nghĩa UART và PIN (thay thế bằng cấu hình của bạn)
#define DWIN_UART_NUM UART_NUM_1
#define DWIN_UART_TXD_PIN (GPIO_NUM_17)
#define DWIN_UART_RXD_PIN (GPIO_NUM_16)
#define DWIN_UART_BUF_SIZE (1024)
#define DWIN_TASK_STACK_SIZE (2048 * 2)

// Header của khung truyền DWIN
#define DWIN_FRAME_HEADER0 0x5A
#define DWIN_FRAME_HEADER1 0xA5

// Mã lệnh DWIN
#define DWIN_CMD_WRITE 0x82
#define DWIN_CMD_READ 0x83

// Tag cho logging
static const char *TAG = "DWIN_LIB";

// --- Cấu trúc dữ liệu ---
// Dùng để lưu trữ thông tin về một khung truyền DWIN
typedef struct
{
    uint8_t header[2];
    uint8_t len; // Độ dài dữ liệu từ byte CMD trở đi (có thể bao gồm cả CRC tùy theo định nghĩa của DWIN)
    uint8_t command;
    uint8_t vp_addr_high; // Thường là 2 byte đầu của data payload cho read/write
    uint8_t vp_addr_low;
    uint8_t payload[250]; // Dữ liệu thực sự (giá trị đọc/ghi, thông tin chạm)
    uint8_t payload_len;
    uint16_t crc_val; // Giá trị CRC nhận được (nếu có)
    bool crc_ok;
} dwin_processed_frame_t;

// Dùng để quản lý các yêu cầu đọc đang chờ phản hồi
typedef struct
{
    uint16_t vp_address;
    QueueHandle_t response_queue; // Queue để task yêu cầu chờ phản hồi
    TaskHandle_t requester_task;  // Task đã gửi yêu cầu
    bool active;                  // Đánh dấu yêu cầu còn hoạt động
} pending_read_request_t;

// --- Biến toàn cục ---
static QueueHandle_t uart_queue; // Queue cho sự kiện UART từ driver
static QueueHandle_t g_dwin_touch_event_queue = NULL;
static QueueHandle_t g_dwin_write_response_queue = NULL; // Tùy chọn

#define MAX_PENDING_READS 5
static pending_read_request_t g_pending_read_requests[MAX_PENDING_READS];
static SemaphoreHandle_t g_pending_reads_mutex = NULL;

// --- Khai báo hàm nội bộ ---
static void dwin_uart_init(void);
static uint16_t dwin_calculate_crc16(const uint8_t *data, size_t length); // Cần tự triển khai
static void dwin_send_raw_frame(const uint8_t *frame_buffer, size_t length);
static esp_err_t parse_dwin_frame(const uint8_t *buffer, int len, dwin_processed_frame_t *parsed_frame);
static void dwin_uart_event_task(void *pvParameters);
static void dwin_touch_handler_task(void *pvParameters); // Ví dụ

// --- Triển khai hàm ---

static void dwin_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200, // Hoặc baudrate màn hình DWIN của bạn
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    ESP_ERROR_CHECK(uart_driver_install(DWIN_UART_NUM, DWIN_UART_BUF_SIZE * 2, DWIN_UART_BUF_SIZE * 2, 20, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(DWIN_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(DWIN_UART_NUM, DWIN_UART_TXD_PIN, DWIN_UART_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "DWIN UART driver installed and configured.");
}

// Hàm tính CRC-16 (ví dụ Modbus - cần kiểm tra lại với DWIN)
static uint16_t dwin_calculate_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++)
    {
        crc ^= (uint16_t)data[i];
        for (int j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc = (crc >> 1) ^ 0xA001;
            }
            else
            {
                crc = crc >> 1;
            }
        }
    }
    // DWIN có thể yêu cầu byte thấp trước, byte cao sau hoặc ngược lại.
    // return crc; // Hoặc ((crc << 8) & 0xFF00) | ((crc >> 8) & 0x00FF);
    return crc;
}

// Gửi một khung truyền thô qua UART
static void dwin_send_raw_frame(const uint8_t *frame_buffer, size_t length)
{
    uart_write_bytes(DWIN_UART_NUM, (const char *)frame_buffer, length);
}

// Phân tích một khung truyền DWIN nhận được
static esp_err_t parse_dwin_frame(const uint8_t *buffer, int len, dwin_processed_frame_t *parsed_frame)
{
    if (len < 3)
    { // Tối thiểu 5AA5 + LEN
        ESP_LOGE(TAG, "Frame too short (len %d)", len);
        return ESP_FAIL;
    }
    if (buffer[0] != DWIN_FRAME_HEADER0 || buffer[1] != DWIN_FRAME_HEADER1)
    {
        ESP_LOGE(TAG, "Invalid frame header: 0x%02X 0x%02X", buffer[0], buffer[1]);
        return ESP_FAIL;
    }

    parsed_frame->header[0] = buffer[0];
    parsed_frame->header[1] = buffer[1];
    parsed_frame->len = buffer[2]; // Byte LEN của DWIN

    // Kiểm tra độ dài tổng thể có hợp lệ với byte LEN không
    // Độ dài mong đợi = 2 (header) + 1 (len_byte) + parsed_frame->len (data + cmd + có thể cả crc)
    int expected_total_length = 3 + parsed_frame->len;
    if (len < expected_total_length)
    {
        ESP_LOGE(TAG, "Frame actual length (%d) less than indicated by LEN byte (expected %d for data part, total %d)",
                 len, parsed_frame->len, expected_total_length);
        // return ESP_FAIL; // Có thể vẫn xử lý nếu là frame ngắn hơn nhưng hợp lệ
    }
    // Giới hạn việc đọc để tránh tràn bộ đệm
    int process_len = (len < expected_total_length) ? len : expected_total_length;

    if (process_len < 4)
    { // Header + Len + CMD
        ESP_LOGE(TAG, "Frame too short for CMD (len %d)", process_len);
        return ESP_FAIL;
    }
    parsed_frame->command = buffer[3];

    // Logic xác định CRC và payload dựa trên bảng bạn cung cấp và CMD
    // Đây là phần phức tạp và cần sự chính xác dựa trên tài liệu DWIN
    // Ví dụ: Rx: 5AA5 06 83 000F 01 1410 -> parsed_frame->len = 0x06. Data payload: 000F 01 1410
    // Ví dụ: Rx: 5AA5 08 83 000F 01 1410 43F0 -> parsed_frame->len = 0x08. Data payload: 000F 01 1410. CRC: 43F0
    // Payload bắt đầu từ buffer[4]

    int data_field_len = parsed_frame->len - 1; // Trừ byte CMD
    if (data_field_len < 0)
    {
        ESP_LOGE(TAG, "Invalid data field length calculation.");
        return ESP_FAIL;
    }

    bool frame_has_crc_field = false;
    int crc_data_len = 0; // Độ dài dữ liệu dùng để tính CRC

    // Xác định có CRC không dựa trên cấu trúc bạn cung cấp
    if (parsed_frame->command == DWIN_CMD_READ)
    { // 0x83
        // 0x83 Read Response (No CRC Check)
        // Rx: 5AA5 06 83 000F 01 1410 (len=0x06, data = 000F 01 1410)
        // 0x83 Read Response (CRC Check)
        // Rx: 5AA5 08 83 000F 01 1410 43F0 (len=0x08, data = 000F 01 1410, CRC = 43F0)
        // 0x83 Touch Upload (No CRC Check)
        // Rx: 5AA5 06 83 1001 01 005A (len=0x06, data = 1001 01 005A)
        // 0x83 Touch Upload (CRC Check)
        // Rx: 5AA5 08 83 1001 01 005A 0E2C (len=0x08, data = 1001 01 005A, CRC = 0E2C)
        if (parsed_frame->len == 0x08 && process_len >= (3 + 0x08))
        { // Có CRC
            frame_has_crc_field = true;
            parsed_frame->payload_len = data_field_len - 2; // Trừ 2 byte CRC
            parsed_frame->crc_val = (buffer[3 + parsed_frame->len - 2] << 8) | buffer[3 + parsed_frame->len - 1];
            crc_data_len = 3 + parsed_frame->len - 2; // Header + LenByte + CMD + Payload (không gồm CRC)
        }
        else if (parsed_frame->len == 0x06 && process_len >= (3 + 0x06))
        { // Không có CRC
            frame_has_crc_field = false;
            parsed_frame->payload_len = data_field_len;
        }
        else
        {
            ESP_LOGW(TAG, "0x83 frame with unexpected len byte: 0x%02X", parsed_frame->len);
            parsed_frame->payload_len = data_field_len > 0 ? data_field_len : 0; // Best effort
        }
    }
    else if (parsed_frame->command == DWIN_CMD_WRITE)
    { // 0x82
        // 0x82 Write Response (No CRC Check)
        // Rx: 5AA5 03 82 4F4B (len=0x03, data=4F4B)
        // 0x82 Write Response (CRC Check)
        // Rx: 5AA5 05 82 4F4B A5EF (len=0x05, data=4F4B, CRC=A5EF)
        if (parsed_frame->len == 0x05 && process_len >= (3 + 0x05))
        { // Có CRC
            frame_has_crc_field = true;
            parsed_frame->payload_len = data_field_len - 2;
            parsed_frame->crc_val = (buffer[3 + parsed_frame->len - 2] << 8) | buffer[3 + parsed_frame->len - 1];
            crc_data_len = 3 + parsed_frame->len - 2;
        }
        else if (parsed_frame->len == 0x03 && process_len >= (3 + 0x03))
        { // Không có CRC
            frame_has_crc_field = false;
            parsed_frame->payload_len = data_field_len;
        }
        else
        {
            ESP_LOGW(TAG, "0x82 frame with unexpected len byte: 0x%02X", parsed_frame->len);
            parsed_frame->payload_len = data_field_len > 0 ? data_field_len : 0;
        }
    }
    else
    {
        ESP_LOGW(TAG, "Unknown command: 0x%02X", parsed_frame->command);
        parsed_frame->payload_len = data_field_len > 0 ? data_field_len : 0; // Best effort
    }

    if (parsed_frame->payload_len < 0)
        parsed_frame->payload_len = 0;
    if (parsed_frame->payload_len > sizeof(parsed_frame->payload))
    {
        ESP_LOGE(TAG, "Payload too large for buffer: %d", parsed_frame->payload_len);
        parsed_frame->payload_len = sizeof(parsed_frame->payload); // Truncate
    }

    if (parsed_frame->payload_len >= 2)
    { // VP address is usually the first 2 bytes of payload
        parsed_frame->vp_addr_high = buffer[4];
        parsed_frame->vp_addr_low = buffer[5];
        if (parsed_frame->payload_len > 2)
        {
            memcpy(parsed_frame->payload, &buffer[6], parsed_frame->payload_len - 2);
        }
    }
    else if (parsed_frame->payload_len > 0)
    { // For write responses like 4F4B
        memcpy(parsed_frame->payload, &buffer[4], parsed_frame->payload_len);
        parsed_frame->vp_addr_high = 0; // No VP in 4F4B response
        parsed_frame->vp_addr_low = 0;
    }

    parsed_frame->crc_ok = true; // Mặc định là OK nếu không có CRC field
    if (frame_has_crc_field)
    {
        uint16_t calculated_crc = dwin_calculate_crc16(buffer, crc_data_len);
        if (calculated_crc != parsed_frame->crc_val)
        {
            ESP_LOGE(TAG, "CRC Mismatch! Calculated: 0x%04X, Received: 0x%04X", calculated_crc, parsed_frame->crc_val);
            parsed_frame->crc_ok = false;
            // return ESP_FAIL; // Quyết định có bỏ qua frame lỗi CRC không
        }
        else
        {
            ESP_LOGD(TAG, "CRC OK: 0x%04X", parsed_frame->crc_val);
        }
    }
    return ESP_OK;
}

static void dwin_uart_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t *data_buffer = (uint8_t *)malloc(DWIN_UART_BUF_SIZE);
    if (data_buffer == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate UART data buffer");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "DWIN UART Event Task started.");

    for (;;)
    {
        if (xQueueReceive(uart_queue, (void *)&event, (TickType_t)portMAX_DELAY))
        {
            switch (event.type)
            {
            case UART_DATA:
            {
                int len = uart_read_bytes(DWIN_UART_NUM, data_buffer, event.size, pdMS_TO_TICKS(100));
                if (len > 0)
                {
                    ESP_LOGD(TAG, "UART received %d bytes:", len);
                    ESP_LOG_BUFFER_HEXDUMP(TAG, data_buffer, len, ESP_LOG_DEBUG);

                    dwin_processed_frame_t parsed_frame;
                    if (parse_dwin_frame(data_buffer, len, &parsed_frame) == ESP_OK)
                    {
                        if (!parsed_frame.crc_ok && (parsed_frame.len == 0x08 || parsed_frame.len == 0x05))
                        { // Chỉ check CRC nếu frame có trường CRC
                            ESP_LOGE(TAG, "Dropping frame due to CRC error.");
                            continue;
                        }

                        uint16_t vp_from_response = (parsed_frame.vp_addr_high << 8) | parsed_frame.vp_addr_low;

                        if (parsed_frame.command == DWIN_CMD_READ)
                        { // 0x83
                            bool is_read_response = false;
                            if (xSemaphoreTake(g_pending_reads_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
                            {
                                for (int i = 0; i < MAX_PENDING_READS; i++)
                                {
                                    if (g_pending_read_requests[i].active && g_pending_read_requests[i].vp_address == vp_from_response)
                                    {
                                        ESP_LOGI(TAG, "Matched Read Response for VP 0x%04X", vp_from_response);
                                        if (g_pending_read_requests[i].response_queue != NULL)
                                        {
                                            // Gửi chỉ phần payload thực sự (sau VP)
                                            if (xQueueSend(g_pending_read_requests[i].response_queue, &parsed_frame, pdMS_TO_TICKS(10)) != pdPASS)
                                            {
                                                ESP_LOGE(TAG, "Failed to send read response to queue for VP 0x%04X", vp_from_response);
                                            }
                                        }
                                        g_pending_read_requests[i].active = false; // Vô hiệu hóa yêu cầu này
                                        is_read_response = true;
                                        break;
                                    }
                                }
                                xSemaphoreGive(g_pending_reads_mutex);
                            }
                            else
                            {
                                ESP_LOGE(TAG, "Could not take pending_reads_mutex to check 0x83 response.");
                            }

                            if (!is_read_response)
                            {
                                ESP_LOGI(TAG, "Received 0x83, not a pending read response. Assuming Touch Event for VP/Key 0x%04X.", vp_from_response);
                                if (g_dwin_touch_event_queue != NULL)
                                {
                                    if (xQueueSend(g_dwin_touch_event_queue, &parsed_frame, pdMS_TO_TICKS(10)) != pdPASS)
                                    {
                                        ESP_LOGE(TAG, "Failed to send touch event to queue.");
                                    }
                                }
                            }
                        }
                        else if (parsed_frame.command == DWIN_CMD_WRITE)
                        { // 0x82
                            ESP_LOGI(TAG, "Write Response (0x82) received.");
                            if (parsed_frame.payload_len >= 2 && parsed_frame.payload[0] == 0x4F && parsed_frame.payload[1] == 0x4B)
                            {
                                ESP_LOGI(TAG, "Write OK (0x4F4B)");
                            }
                            else
                            {
                                ESP_LOGW(TAG, "Write response not OK or unknown. Payload: %02X %02X",
                                         parsed_frame.payload_len > 0 ? parsed_frame.payload[0] : 0,
                                         parsed_frame.payload_len > 1 ? parsed_frame.payload[1] : 0);
                            }
                            if (g_dwin_write_response_queue != NULL)
                            {
                                xQueueSend(g_dwin_write_response_queue, &parsed_frame, pdMS_TO_TICKS(10));
                            }
                        }
                    }
                    else
                    {
                        ESP_LOGE(TAG, "Failed to parse DWIN frame.");
                    }
                }
                break;
            }
            case UART_FIFO_OVF:
                ESP_LOGW(TAG, "UART FIFO Overflow");
                uart_flush_input(DWIN_UART_NUM);
                xQueueReset(uart_queue);
                break;
            case UART_BUFFER_FULL:
                ESP_LOGW(TAG, "UART Ring Buffer Full");
                uart_flush_input(DWIN_UART_NUM);
                xQueueReset(uart_queue);
                break;
            // Xử lý các sự kiện khác nếu cần
            default:
                ESP_LOGI(TAG, "UART event type: %d", event.type);
                break;
            }
        }
    }
    free(data_buffer);
    vTaskDelete(NULL);
}

// Task ví dụ để xử lý sự kiện chạm
static void dwin_touch_handler_task(void *pvParameters)
{
    dwin_processed_frame_t touch_data;
    ESP_LOGI(TAG, "DWIN Touch Handler Task started.");
    for (;;)
    {
        if (xQueueReceive(g_dwin_touch_event_queue, &touch_data, portMAX_DELAY))
        {
            uint16_t key_code = (touch_data.vp_addr_high << 8) | touch_data.vp_addr_low;
            ESP_LOGI("TOUCH_HANDLER", "Touch Event! Key Code: 0x%04X, Data:", key_code);
            if (touch_data.payload_len > 0)
            {
                ESP_LOG_BUFFER_HEX("TOUCH_PAYLOAD", touch_data.payload, touch_data.payload_len);
            }
            // Xử lý logic chạm dựa trên key_code và touch_data.payload
            // Ví dụ: if (key_code == 0x1001 && touch_data.payload[0] == 0x01) { /* Nút nhấn */ }
        }
    }
}

// --- API Công khai cho Thư viện ---

/**
 * @brief Khởi tạo thư viện DWIN.
 */
void dwin_lib_init(void)
{
    // Khởi tạo mutex cho danh sách yêu cầu đọc
    g_pending_reads_mutex = xSemaphoreCreateMutex();
    if (g_pending_reads_mutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create pending_reads_mutex");
        return;
    }
    for (int i = 0; i < MAX_PENDING_READS; i++)
    {
        g_pending_read_requests[i].active = false;
        g_pending_read_requests[i].response_queue = NULL;
    }

    // Khởi tạo queue cho sự kiện chạm
    g_dwin_touch_event_queue = xQueueCreate(10, sizeof(dwin_processed_frame_t));
    if (g_dwin_touch_event_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create touch event queue");
        // return; // Có thể vẫn tiếp tục nếu không cần xử lý chạm
    }

    // Khởi tạo queue cho phản hồi lệnh ghi (tùy chọn)
    g_dwin_write_response_queue = xQueueCreate(5, sizeof(dwin_processed_frame_t));
    if (g_dwin_write_response_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create write response queue");
    }

    dwin_uart_init(); // Khởi tạo UART
    xTaskCreate(dwin_uart_event_task, "dwin_uart_event_task", DWIN_TASK_STACK_SIZE, NULL, 12, NULL);
    if (g_dwin_touch_event_queue)
    { // Chỉ tạo task nếu queue được tạo thành công
        xTaskCreate(dwin_touch_handler_task, "dwin_touch_handler_task", 2048, NULL, 10, NULL);
    }
    ESP_LOGI(TAG, "DWIN Library Initialized.");
}

/**
 * @brief Đọc dữ liệu từ một địa chỉ VP của DWIN.
 *
 * @param vp_address Địa chỉ VP cần đọc (ví dụ: 0x000F).
 * @param num_bytes_to_read Số byte dữ liệu mong muốn đọc từ VP (ví dụ: 2 cho 1 word).
 * LƯU Ý: DWIN thường đọc theo WORD (2 byte). Byte "NumToRead" trong lệnh
 * Tx thường là số WORD. Bạn cần điều chỉnh logic này.
 * @param out_buffer Buffer để lưu dữ liệu đọc được (chỉ phần data, không bao gồm VP).
 * @param buffer_size Kích thước của out_buffer.
 * @param timeout_ms Thời gian chờ tối đa cho phản hồi (ms).
 * @return Số byte thực sự đọc được và lưu vào out_buffer, hoặc -1 nếu lỗi/timeout.
 */
int dwin_read_vp(uint16_t vp_address, uint8_t num_words_to_read, uint8_t *out_buffer, size_t buffer_size, uint32_t timeout_ms)
{
    if (out_buffer == NULL || buffer_size == 0)
    {
        ESP_LOGE(TAG, "Output buffer invalid for dwin_read_vp.");
        return -1;
    }

    QueueHandle_t temp_response_queue = xQueueCreate(1, sizeof(dwin_processed_frame_t)); 
    if (temp_response_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create temp response queue for VP 0x%04X", vp_address);
        return -1;
    }

    int request_slot = -1;
    if (xSemaphoreTake(g_pending_reads_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        for (int i = 0; i < MAX_PENDING_READS; i++)
        {
            if (!g_pending_read_requests[i].active)
            {
                g_pending_read_requests[i].vp_address = vp_address;
                g_pending_read_requests[i].response_queue = temp_response_queue;
                g_pending_read_requests[i].active = true;
                request_slot = i;
                break;
            }
        }
        xSemaphoreGive(g_pending_reads_mutex);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to take mutex for adding read request VP 0x%04X", vp_address);
        vQueueDelete(temp_response_queue);
        return -1;
    }

    if (request_slot == -1)
    {
        ESP_LOGE(TAG, "No available slot for pending read request VP 0x%04X", vp_address);
        vQueueDelete(temp_response_queue);
        return -1;
    }

    // Tạo khung lệnh đọc (ví dụ: 0x83 Read Instruction - No CRC Check)
    // Tx: 5AA5 04 83 VP_H VP_L NumWordsToRead
    // Nếu có CRC: Tx: 5AA5 06 83 VP_H VP_L NumWordsToRead CRC_H CRC_L
    uint8_t frame_tx[9]; // Kích thước tối đa cho lệnh đọc với CRC
    frame_tx[0] = DWIN_FRAME_HEADER0;
    frame_tx[1] = DWIN_FRAME_HEADER1;
    // frame_tx[2] = LEN byte
    frame_tx[3] = DWIN_CMD_READ;
    frame_tx[4] = (vp_address >> 8) & 0xFF; // VP High
    frame_tx[5] = vp_address & 0xFF;        // VP Low
    frame_tx[6] = num_words_to_read;        // Số WORD cần đọc (DWIN thường là word)

    bool use_crc_for_read_cmd = true; // Quyết định có gửi CRC cho lệnh đọc không (dựa vào bảng của bạn)
                                      // Ví dụ: Tx: 5AA5 04 83 000F 01 (No CRC)
                                      // Tx: 5AA5 06 83 000F 01 ED90 (With CRC)

    size_t tx_len;
    if (use_crc_for_read_cmd)
    {
        frame_tx[2] = 0x06;                                   // LEN = CMD(1) + VP(2) + NumRead(1) + CRC(2)
        uint16_t crc_val = dwin_calculate_crc16(frame_tx, 7); // CRC tính trên 5A A5 LEN CMD VP NumRead
        frame_tx[7] = (crc_val >> 8) & 0xFF;
        frame_tx[8] = crc_val & 0xFF;
        tx_len = 9;
    }
    else
    {
        frame_tx[2] = 0x04; // LEN = CMD(1) + VP(2) + NumRead(1)
        tx_len = 7;
    }

    ESP_LOGD(TAG, "Sending Read command for VP 0x%04X, NumWords: %d. Frame:", vp_address, num_words_to_read);
    ESP_LOG_BUFFER_HEXDUMP(TAG, frame_tx, tx_len, ESP_LOG_DEBUG);
    dwin_send_raw_frame(frame_tx, tx_len);

    dwin_processed_frame_t received_response;
    int bytes_copied = -1;

    if (xQueueReceive(temp_response_queue, &received_response, pdMS_TO_TICKS(timeout_ms)))
    {
        ESP_LOGI(TAG, "Read response received for VP 0x%04X. Payload len: %d", vp_address, received_response.payload_len);
        // Payload của received_response đã bỏ qua VP, chỉ chứa data.
        // payload_len ở đây là số byte dữ liệu sau VP.
        if (received_response.payload_len > 0)
        {
            if (received_response.payload_len <= buffer_size)
            {
                memcpy(out_buffer, received_response.payload, received_response.payload_len);
                bytes_copied = received_response.payload_len;
                ESP_LOGD(TAG, "Data copied to user buffer:");
                ESP_LOG_BUFFER_HEXDUMP(TAG, out_buffer, bytes_copied, ESP_LOG_DEBUG);
            }
            else
            {
                ESP_LOGE(TAG, "Read buffer too small for VP 0x%04X. Need %d, have %d",
                         vp_address, received_response.payload_len, buffer_size);
                bytes_copied = -2; // Lỗi buffer quá nhỏ
            }
        }
        else
        {
            ESP_LOGW(TAG, "Read response for VP 0x%04X has no data payload.", vp_address);
            bytes_copied = 0;
        }
    }
    else
    {
        ESP_LOGW(TAG, "Timeout waiting for read response for VP 0x%04X", vp_address);
        bytes_copied = -3; // Lỗi timeout
        // Dọn dẹp yêu cầu nếu timeout
        if (xSemaphoreTake(g_pending_reads_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            if (g_pending_read_requests[request_slot].active && g_pending_read_requests[request_slot].vp_address == vp_address)
            {
                g_pending_read_requests[request_slot].active = false;
            }
            xSemaphoreGive(g_pending_reads_mutex);
        }
    }

    vQueueDelete(temp_response_queue);
    return bytes_copied;
}

/**
 * @brief Ghi dữ liệu vào một địa chỉ VP của DWIN.
 *
 * @param vp_address Địa chỉ VP cần ghi (ví dụ: 0x1000).
 * @param data_to_write Con trỏ tới buffer chứa dữ liệu cần ghi.
 * @param data_length Độ dài của dữ liệu cần ghi (bytes).
 * @param wait_for_ack true nếu muốn chờ phản hồi "OK" (0x4F4B), false nếu không.
 * @param timeout_ms Thời gian chờ tối đa cho ACK (nếu wait_for_ack là true).
 * @return 0 nếu thành công (hoặc không chờ ACK), -1 nếu lỗi gửi, -2 nếu không nhận được ACK hoặc ACK lỗi.
 */
int dwin_write_vp(uint16_t vp_address, const uint8_t *data_to_write, size_t data_length, bool wait_for_ack, uint32_t timeout_ms)
{
    if (data_to_write == NULL && data_length > 0)
    {
        ESP_LOGE(TAG, "Data to write is NULL for dwin_write_vp.");
        return -1;
    }

    // Tạo khung lệnh ghi
    // Tx: 5AA5 LEN 82 VP_H VP_L Data... (CRC)
    // Ví dụ: Tx: 5AA5 05 82 1000 3132 (No CRC) -> LEN = 0x05 = 1(CMD) + 2(VP) + 2(Data)
    // Ví dụ: Tx: 5AA5 07 82 1000 3132 CC9B (With CRC) -> LEN = 0x07 = 1(CMD) + 2(VP) + 2(Data) + 2(CRC)
    uint8_t frame_tx[6 + data_length + 2]; // Kích thước tối đa: H(2)+L(1)+CMD(1)+VP(2)+Data(N)+CRC(2)
    frame_tx[0] = DWIN_FRAME_HEADER0;
    frame_tx[1] = DWIN_FRAME_HEADER1;
    // frame_tx[2] = LEN byte
    frame_tx[3] = DWIN_CMD_WRITE;
    frame_tx[4] = (vp_address >> 8) & 0xFF; // VP High
    frame_tx[5] = vp_address & 0xFF;        // VP Low
    if (data_length > 0)
    {
        memcpy(&frame_tx[6], data_to_write, data_length);
    }

    bool use_crc_for_write_cmd = true; // Theo ví dụ của bạn: Tx: 5AA5 07 82 1000 3132 CC9B (có CRC)
    size_t tx_len;

    if (use_crc_for_write_cmd)
    {
        frame_tx[2] = 1 + 2 + data_length + 2;                                  // LEN = CMD(1) + VP(2) + Data(N) + CRC(2)
        uint16_t crc_val = dwin_calculate_crc16(frame_tx, 3 + frame_tx[2] - 2); // CRC từ Header đến hết Data
        frame_tx[3 + frame_tx[2] - 2] = (crc_val >> 8) & 0xFF;
        frame_tx[3 + frame_tx[2] - 1] = crc_val & 0xFF;
        tx_len = 3 + frame_tx[2];
    }
    else
    {
        frame_tx[2] = 1 + 2 + data_length; // LEN = CMD(1) + VP(2) + Data(N)
        tx_len = 3 + frame_tx[2];
    }

    ESP_LOGD(TAG, "Sending Write command for VP 0x%04X. Frame:", vp_address);
    ESP_LOG_BUFFER_HEXDUMP(TAG, frame_tx, tx_len, ESP_LOG_DEBUG);
    dwin_send_raw_frame(frame_tx, tx_len);

    if (wait_for_ack && g_dwin_write_response_queue != NULL)
    {
        dwin_processed_frame_t write_ack;
        if (xQueueReceive(g_dwin_write_response_queue, &write_ack, pdMS_TO_TICKS(timeout_ms)))
        {
            if (write_ack.command == DWIN_CMD_WRITE && write_ack.payload_len >= 2 &&
                write_ack.payload[0] == 0x4F && write_ack.payload[1] == 0x4B)
            {
                ESP_LOGI(TAG, "Write ACK OK for VP 0x%04X", vp_address);
                return 0; // Thành công
            }
            else
            {
                ESP_LOGW(TAG, "Write ACK not OK or unexpected for VP 0x%04X", vp_address);
                return -2; // Lỗi ACK
            }
        }
        else
        {
            ESP_LOGW(TAG, "Timeout waiting for write ACK for VP 0x%04X", vp_address);
            return -2; // Lỗi timeout ACK
        }
    }
    return 0; // Thành công nếu không chờ ACK
}

// --- Ví dụ sử dụng trong app_main ---
void app_main(void)
{
    esp_log_level_set(TAG, ESP_LOG_DEBUG); // Bật log debug cho thư viện DWIN
    esp_log_level_set("TOUCH_HANDLER", ESP_LOG_INFO);

    dwin_lib_init(); // Khởi tạo thư viện

    vTaskDelay(pdMS_TO_TICKS(2000)); // Chờ thư viện khởi động xong

    // Ví dụ Ghi dữ liệu: ghi 2 byte 0x31, 0x32 (ASCII '1', '2') vào VP 0x1000
    uint8_t data_to_write[] = {0x31, 0x32};
    ESP_LOGI("APP_MAIN", "Attempting to write to VP 0x1000...");
    int write_status = dwin_write_vp(0x1000, data_to_write, sizeof(data_to_write), true, 1000);
    if (write_status == 0)
    {
        ESP_LOGI("APP_MAIN", "Write to VP 0x1000 successful.");
    }
    else
    {
        ESP_LOGE("APP_MAIN", "Write to VP 0x1000 failed with code: %d", write_status);
    }

    vTaskDelay(pdMS_TO_TICKS(1000));

    // Ví dụ Đọc dữ liệu: đọc 1 word (2 byte) từ VP 0x000F
    uint8_t read_buf[10];
    ESP_LOGI("APP_MAIN", "Attempting to read 1 word from VP 0x000F...");
    // `num_words_to_read` là 1 (nghĩa là 2 bytes)
    int bytes_read = dwin_read_vp(0x000F, 1, read_buf, sizeof(read_buf), 2000);
    if (bytes_read > 0)
    {
        ESP_LOGI("APP_MAIN", "Read %d bytes from VP 0x000F:", bytes_read);
        ESP_LOG_BUFFER_HEX("APP_MAIN_READ_DATA", read_buf, bytes_read);
        if (bytes_read == 2)
        {
            uint16_t val = (read_buf[0] << 8) | read_buf[1];
            ESP_LOGI("APP_MAIN", "Value as uint16_t: 0x%04X (%u)", val, val);
        }
    }
    else
    {
        ESP_LOGE("APP_MAIN", "Failed to read from VP 0x000F or timeout. Code: %d", bytes_read);
    }

    // Các tác vụ khác của ứng dụng...
    // Màn hình DWIN sẽ gửi sự kiện chạm, và dwin_touch_handler_task sẽ xử lý chúng.
}