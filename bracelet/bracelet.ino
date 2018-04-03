#include "MAX30100.h"

/* Принцип работы:

   Моменту удара пульса соответсвует пик гармонического сигнала датчика (ток ИК диода)
   Задача: вычислить время (delta) между двумя соседними пиками, и по формуле 60[sec]/delta[sec]
   получить среднее количество ударов в минуту
   Исходный сигнал заменяется на функцию среднего арифметического - сигнал смещается по фазе
   но становится гладким
   Для отсекания пиков параллельно вычисляется функция среднего арифметического от среднего арифметического,
   она отстает по фазе от первой. Когда первая функция находится над второй, наступает область максимума.
   Для его определения фиксируется момент, когда нарастание первой функции сменяется спадом

   Если браслет снят, ток красного диода резко падает до значения близкого к нулю
   
*/

#define CR_NL                                   0x0A0D

#define ESP_TIMEOUT_MS_DEFAULT                  2000
#define ESP_TIMEOUT_MS_WIFI_CONNECTION          15000
#define ESP_TIMEOUT_MS_WIFI_GETTING_IP          10000
#define ESP_TIMEOUT_MS_TCP_SERVER_CONNECTION    5000
#define ESP_TIMEOUT_MS_DEEP_SLEEP_WAKE_UP       6000
#define MAX_POLL_PERIOD_US                      1E06 / 100 /* 30 мкс */

#define CIRC_BUF_LEN                            30
#define BRACELET_DROP_TRESHOLD                  1000

#define MS_IN_MIN                               60000.0f

/* Имя и пароль WiFi сети: AT+CWJAP_DEF="<SSID>","<passwrd>" */
const char *wifi_connection_str = "AT+CWJAP_DEF=\"Columbia 3.0\",\"allisinvain\"\r\n";
/* Номер браслета: *BRACELET_<number>& */
const char *bracelet_num = "*BRACELET_1&\r\n";
/* MAC адрес браслета: AT+CIPSTAMAC_DEF="<mac address>" */    
const char *bracelet_mac = "AT+CIPSTAMAC_DEF=\"aa:aa:aa:aa:aa:aa\"\r\n";
/* Тип соединения, IP сервера, порт, [keep-alive]: AT+CIPSTART="<connection type>","<server IP>",<port>,[<keep-alive>]" */     
const char *tcp_server_connection_str = "AT+CIPSTART=\"TCP\",\"192.168.1.40\",1100,1\r\n";
/* Команда входа ESP8266 в режим DEEP SLEEP: AT+GSLP=<msec> */
const char *deep_sleep_enter_cmd = "AT+GSLP=5000\r\n";

enum esp_state_typedef {
    ESP_STATE_ENTER_DEEP_SLEEP,
    ESP_STATE_WAIT_WAKE_UP,
    ESP_STATE_TCP_RECONNECT,
    ESP_STATE_SEND_DATA
};

enum max_state_typedef {
    MAX_STATE_IDLE,
    MAX_STATE_MEASURE
};

enum esp_wait_for_str_status {
    PENDING,
    STR_IS_RECEIVED,
    TIMEOUT_EXPIRED
};

typedef unsigned long time_t;

esp_state_typedef esp_state = ESP_STATE_ENTER_DEEP_SLEEP;
max_state_typedef max_state = MAX_STATE_IDLE;
MAX30100 sensor;
uint8_t uart_byte_cntr = 0;
time_t esp_last_timestamp = 0;
time_t max_last_poll_timestamp = 0;
time_t max_last_peak_timestamp = 0;
/* Интервал времени между соседними пиками сигнала ИК диода */
time_t max_peak_delta = 0;
bool max_peak_is_detected = false;
bool bracelet_is_dropped = false;

class circ_buf {
    private:
        float buf[CIRC_BUF_LEN];
        uint8_t idx;
    public:
        void add(float val);
        float avg(); /* Среднее арифметическое */
};

void circ_buf::add(float val) {
    buf[idx] = val;
    idx = (idx < (CIRC_BUF_LEN - 1)) ? (idx + 1) : 0;
}

float circ_buf::avg() {
    float sum = 0;
    for(uint8_t i = 0; i < CIRC_BUF_LEN; i++) {
      sum += buf[i];
    }
    return sum / CIRC_BUF_LEN;
}

/* Буфер значений тока ИК диода */
circ_buf  ired_buf;
/* Буфер средних значений тока ИК диода */
circ_buf  ired_avg_buf;
/* Буфер значений пульса */
circ_buf  pulse_buf;



void atmega_pwr_on() {
    DDRC &= ~(1 << PORTC2);         // PC2 на вход для чтения PWR_GET_DI
    if(!(PORTC & (1 << PORTC2))) {  // Если 0 на PC2
        DDRC  |= (1 << PORTC1);     // PC1 на выход для подачи на него 1
        PORTC |= (1 << PORTC1);
    }
}



void uart_init() {
    /* 115200 бод, 16 МГц, см. http://wormfood.net/avrbaudcalc.php */
    uint16_t baud_rate = 0x0008;
    UBRR0H = (baud_rate & 0x0F00) >> 8;
    UBRR0L = (baud_rate & 0x00FF);
    /* 8-bit data frame, 1 stop bit, no parity mode */
    UCSR0C = (0 << USBS0) | (3 << UCSZ00);
    /* Enable receiver and transmitter */
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
}

void uart_send_byte(uint8_t data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

bool uart_byte_is_received() {
    return ((UCSR0A & (1 << RXC0)) != 0);
}

uint8_t uart_get_received_byte() {
    return UDR0;
}



/* Отправка в ESP8266 строки по uart и возврат времени отправки */
time_t esp_send_str(const char *str) {
    while (*((uint16_t *)(str - sizeof(uint16_t))) != CR_NL) {
        uart_send_byte(*str++);
    }
    return millis();
}

/* Неблокирующая функция, возвращающая текущий статус ожидания строки str в течение timeout_ms от момента времени timestamp_ms */
esp_wait_for_str_status esp_wait_for_str(const char *str, time_t timestamp_ms, time_t timeout_ms) {
    if((millis() - timestamp_ms) < timeout_ms) {
        if(uart_byte_is_received()) {
            uart_byte_cntr = (str[uart_byte_cntr] == (char)uart_get_received_byte()) ? (uart_byte_cntr + 1) : 0;
        }
        if(uart_byte_cntr == strlen(str)) {
            uart_byte_cntr = 0;
            return STR_IS_RECEIVED;  
        }
        else {
            return PENDING;
        }
    }
    else {
        uart_byte_cntr = 0;
        return TIMEOUT_EXPIRED;
    }
}


/* Блокирующая функция ожидания строки str (в ответ на команду cmd) в течение timeout */
bool esp_str_is_received(const char *str, time_t timeout, const char *cmd = nullptr) {
    esp_wait_for_str_status result;
    time_t timestamp = (cmd != nullptr) ? esp_send_str(cmd) : millis();
    do {
        result = esp_wait_for_str(str, timestamp, timeout);
    } while(result == PENDING);
    return (result == STR_IS_RECEIVED);
}


void establish_wifi_connection() {
    /* До тех пор пока соедниение с точкой доступа WiFi в течение TIMEOUT_MS_WIFI_CONNECTION не установлено */
    while(!esp_str_is_received("OK\r\n", ESP_TIMEOUT_MS_WIFI_CONNECTION, wifi_connection_str)) {
        /* Ввести ESP866 в режим deep sleep на 5 сек и ожидать возврата в течение ESP_TIMEOUT_MS_DEEP_SLEEP_WAKE_UP */
        while(!esp_str_is_received("ready\r\n", ESP_TIMEOUT_MS_DEEP_SLEEP_WAKE_UP, deep_sleep_enter_cmd));     
    }
}


void establish_tcp_server_connection() {
    /* До тех пор пока TCP соедниение c сервером в течение TIMEOUT_MS_TCP_SERVER_CONNECTION не установлено */
    while(!esp_str_is_received("OK\r\n", ESP_TIMEOUT_MS_TCP_SERVER_CONNECTION, tcp_server_connection_str)) {
        /* Ввести ESP866 в режим deep sleep на 5 сек и ожидать пока ESP8266 получит IP */
        while(!esp_str_is_received("WIFI GOT IP\r\n", ESP_TIMEOUT_MS_DEEP_SLEEP_WAKE_UP + ESP_TIMEOUT_MS_WIFI_GETTING_IP, deep_sleep_enter_cmd));
    }
}


void esp_station_init() {
  
    /* ESP8266 Instruction Set
       https://www.espressif.com/sites/default/files/documentation/4a-esp8266_at_instruction_set_en.pdf */
       
    while(1) {
        /* Сброс */
        if(!esp_str_is_received("ready\r\n", ESP_TIMEOUT_MS_DEFAULT, "AT+RST\r\n")) {
            continue;      
        }
        
        /* Установить режим Station */
        if(!esp_str_is_received("OK\r\n", ESP_TIMEOUT_MS_DEFAULT, "AT+CWMODE_DEF=1\r\n")) {
            continue;      
        }
      
        /* Установить MAC адрес */
        if(!esp_str_is_received("OK\r\n", ESP_TIMEOUT_MS_DEFAULT, bracelet_mac)) {
            continue;      
        }
        
        /* Отключиться от текущей точки доступа */
        if(!esp_str_is_received("OK\r\n", ESP_TIMEOUT_MS_DEFAULT, "AT+CWQAP\r\n")) {
            continue;      
        }
        
        /* Подключиться к точке доступа WiFi */
        establish_wifi_connection();
        
        /* Получить IP для ESP8266 Station */
        if(!esp_str_is_received("OK\r\n", ESP_TIMEOUT_MS_DEFAULT, "AT+CWDHCP_DEF=1,1\r\n")) {
            continue;      
        }
        
        /* Установить соединение с TCP сервером */
        establish_tcp_server_connection();
        
        /* Ожидание запроса имени браслета от сервера */
        if(!esp_str_is_received("*WHOAREYOU&", ESP_TIMEOUT_MS_DEFAULT)) {
            continue;      
        }
        
        /* Подготовить передачу данных в ESP8266 */
        if(!esp_str_is_received("OK\r\n", ESP_TIMEOUT_MS_DEFAULT, "AT+CIPSEND=12\r\n")) {
            continue;     
        }
        
        /* Ответить на запрос WHOAREYOU */
        if(!esp_str_is_received("*OK&", ESP_TIMEOUT_MS_DEFAULT, bracelet_num)) {
            continue;      
        }

        /* SUCCESS */
        break;
        
    }
}


void max_init() {
    while(!sensor.begin());
    sensor.setMode(MAX30100_MODE_SPO2_HR);
    sensor.setLedsCurrent(/* IRED */ MAX30100_LED_CURR_4_4MA, /* RED */ MAX30100_LED_CURR_4_4MA);
    sensor.setLedsPulseWidth(MAX30100_SPC_PW_1600US_16BITS);
    sensor.setSamplingRate(MAX30100_SAMPRATE_600HZ);
    sensor.setHighresModeEnabled(true);
}


void adc_init() {
    analogReference(INTERNAL);
}


void esp_send_lost() {
    if(!esp_str_is_received("OK\r\n", ESP_TIMEOUT_MS_DEFAULT, "AT+CIPSEND=14\r\n")) {
        return;      
    }
    
    esp_str_is_received("SEND OK\r\n", ESP_TIMEOUT_MS_DEFAULT, "*S=$C=LOST$P=&\r\n");
}


bool esp_send_pulse() {
    uint8_t pulse = (uint8_t)(pulse_buf.avg());
    char pulse_cmd[19] = {0};
    sprintf(pulse_cmd, "*S=$C=PULS$P=%d&\r\n", pulse);
    char at_cipsend_cmd[15] = {0};
    sprintf(at_cipsend_cmd, "AT+CIPSEND=%d\r\n", ((pulse >= 100) ? 17 : ((pulse > 9) ? 16 : 15)) );

    if(!esp_str_is_received("OK\r\n", ESP_TIMEOUT_MS_DEFAULT, at_cipsend_cmd)) {
        return;
    }
    
    esp_str_is_received("SEND OK\r\n", ESP_TIMEOUT_MS_DEFAULT, pulse_cmd);
}


void esp_send_sbatt() {
    uint16_t  adc_val_raw = 0;
    uint8_t   adc_val_percent = 0;
    
    #define ADC_SBATT_RAW_VAL_MIN 503
    #define ADC_SBATT_RAW_VAL_MAX 704
    
    #define SBATT_PERCENTAGE_MIN  0
    #define SBATT_PERCENTAGE_MAX  100
    
    adc_val_raw = analogRead(A0);
    adc_val_percent = map(adc_val_raw, 
                          ADC_SBATT_RAW_VAL_MIN, ADC_SBATT_RAW_VAL_MAX, 
                          SBATT_PERCENTAGE_MIN, SBATT_PERCENTAGE_MAX);
                          
    char sbatt_cmd[20] = {0};
    sprintf(sbatt_cmd, "*S=$C=SBATT$P=%d&\r\n", adc_val_percent);
    
    char at_cipsend_cmd[15] = {0};
    sprintf(at_cipsend_cmd, "AT+CIPSEND=%d\r\n", (adc_val_percent == 100) ? 18 : ((adc_val_percent > 9) ? 17 : 16));
    
    if(!esp_str_is_received("OK\r\n", ESP_TIMEOUT_MS_DEFAULT, at_cipsend_cmd)) {
        return;      
    }
    
    esp_str_is_received("SEND OK\r\n", ESP_TIMEOUT_MS_DEFAULT, sbatt_cmd);    
}


void esp_send_stop() {
    if(!esp_str_is_received("OK\r\n", ESP_TIMEOUT_MS_DEFAULT, "AT+CIPSEND=14\r\n")) {
        return;      
    }
    
    esp_str_is_received("SEND OK\r\n", ESP_TIMEOUT_MS_DEFAULT, "*S=$C=STOP$P=&\r\n");
}


/* Конечный автомат ESP8266 */
void esp_process() {
    switch(esp_state) {
    case ESP_STATE_ENTER_DEEP_SLEEP:
        esp_last_timestamp = esp_send_str(deep_sleep_enter_cmd);
        esp_state = ESP_STATE_WAIT_WAKE_UP;                    
        break;
    case ESP_STATE_WAIT_WAKE_UP:
        switch(esp_wait_for_str("WIFI GOT IP\r\n", esp_last_timestamp, ESP_TIMEOUT_MS_DEEP_SLEEP_WAKE_UP + ESP_TIMEOUT_MS_WIFI_GETTING_IP)) {
        case STR_IS_RECEIVED:
            esp_last_timestamp = esp_send_str(tcp_server_connection_str);
            esp_state = ESP_STATE_TCP_RECONNECT;
            break;           
        case TIMEOUT_EXPIRED:
            esp_state = ESP_STATE_ENTER_DEEP_SLEEP;
            break;
        default:  
            break;
        }    
        break;
    case ESP_STATE_TCP_RECONNECT:
        switch(esp_wait_for_str("OK\r\n", esp_last_timestamp, ESP_TIMEOUT_MS_TCP_SERVER_CONNECTION)) {
        case STR_IS_RECEIVED: {
             /* Ожидание запроса имени браслета от сервера */
            if(!esp_str_is_received("*WHOAREYOU&", ESP_TIMEOUT_MS_DEFAULT)) {
                esp_state = ESP_STATE_ENTER_DEEP_SLEEP;
                break;      
            }
            /* Подготовить передачу данных в ESP8266 */
            if(!esp_str_is_received("OK\r\n", ESP_TIMEOUT_MS_DEFAULT, "AT+CIPSEND=12\r\n")) {
                esp_state = ESP_STATE_ENTER_DEEP_SLEEP;
                break;      
            }
            /* Ответить на запрос WHOAREYOU */
            if(!esp_str_is_received("*OK&", ESP_TIMEOUT_MS_DEFAULT, bracelet_num)) {
                esp_state = ESP_STATE_ENTER_DEEP_SLEEP;
                break;      
            }       
            esp_state = ESP_STATE_SEND_DATA;
            break;
        }           
        case TIMEOUT_EXPIRED:
            esp_state = ESP_STATE_ENTER_DEEP_SLEEP;
            break;
        default:  
            break;
        }
        break;
    case ESP_STATE_SEND_DATA:
        if(bracelet_is_dropped) {
            esp_send_lost();      
        }
        else {
            esp_send_pulse();
            esp_send_sbatt();
        }
        esp_send_stop();
        esp_state = ESP_STATE_ENTER_DEEP_SLEEP;
        break;
    default:
        break;
    }   
}

/* Конечный автомат MAX30100 */
void max_process() {
    switch(max_state) {
    case MAX_STATE_IDLE:
        if((micros() - max_last_poll_timestamp) > MAX_POLL_PERIOD_US) {
            max_state = MAX_STATE_MEASURE;  
        }
        break;
    case MAX_STATE_MEASURE: {
          float prev_ired_avg = ired_buf.avg();
          sensor.update();
          if(sensor.rawRedValue < BRACELET_DROP_TRESHOLD) {
              bracelet_is_dropped = true;
          } 
          else {
              bracelet_is_dropped = false;
              
              float ired_val = (float)sensor.rawIRValue;
              ired_buf.add(ired_val);
              
              float ired_avg = ired_buf.avg();
              ired_avg_buf.add(ired_avg);
              
              float ired_double_avg = ired_avg_buf.avg();
  
              if(ired_avg > ired_double_avg) {
                  if(prev_ired_avg > ired_avg) {
                      if(!max_peak_is_detected) {
                          max_peak_delta = millis() - max_last_peak_timestamp;
                          max_last_peak_timestamp = millis();
                          pulse_buf.add(MS_IN_MIN / (float)max_peak_delta);
                          max_peak_is_detected = true;
                      }
                  }
              }
              else {
                  max_peak_is_detected = false;      
              }
              /* DEBUG */
              //Serial.print(ired_avg);
              //Serial.print('\t');
              //Serial.println(ired_double_avg);
          }
          max_state = MAX_STATE_IDLE;
          max_last_poll_timestamp = micros();
          break;
        }
    default:
        break;
    }
}


void setup() {
    atmega_pwr_on();
    uart_init();
    esp_station_init();
    max_init();
}

/* Запуск двух конечных автоматов, которые будут выполняться параллельно */
void loop() {
   esp_process();
   max_process();
}

