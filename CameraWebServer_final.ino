// ============================================================
// camerawebserver_v3_final.ino
// ESP32-CAM 최종 코드
//
// 역할:
// 1. ESP32-CAM 영상을 PC로 MJPEG 스트리밍합니다.
// 2. PC에서 보낸 손 좌표(P,x,y)를 UDP로 수신합니다.
// 3. 수신한 좌표를 Pico 2W로 UART 전달합니다.
// 4. PC에서 보낸 96x96 RAW 이미지를 UDP 청크로 수신합니다.
// 5. RAW 데이터를 재조립한 뒤 Pico 2W로 UART 전송합니다.
//
// 전체 데이터 흐름:
// PC(MediaPipe) → UDP → ESP32-CAM → UART → Pico 2W
// ============================================================


// ------------------------------------------------------------
// ESP32-CAM 카메라 제어 라이브러리
// ------------------------------------------------------------
#include "esp_camera.h"

// ------------------------------------------------------------
// ESP32 내장 HTTP 서버 라이브러리
// PC가 http://ESP32_IP:81/stream 으로 접속할 때 사용합니다.
// ------------------------------------------------------------
#include "esp_http_server.h"

// ------------------------------------------------------------
// ESP32 타이머 관련 라이브러리
// ------------------------------------------------------------
#include "esp_timer.h"

// ------------------------------------------------------------
// Wi-Fi 연결용 라이브러리
// ------------------------------------------------------------
#include <WiFi.h>

// ------------------------------------------------------------
// UDP 통신용 라이브러리
// PC에서 보내는 좌표와 RAW 데이터를 받을 때 사용합니다.
// ------------------------------------------------------------
#include <WiFiUdp.h>


// ------------------------------------------------------------
// 사용하는 카메라 보드 모델 지정
// AI-Thinker ESP32-CAM 핀맵을 사용합니다.
// ------------------------------------------------------------
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"


// ============================================================
// Wi-Fi 설정
// 본인의 공유기 이름과 비밀번호로 수정해야 합니다.
// ============================================================
const char *ssid = "U+NetE5";
const char *password = "DA9969";


// ============================================================
// 통신 포트 설정
// ============================================================

// PC에서 ESP32-CAM으로 UDP 데이터를 보낼 포트입니다.
const int UDP_PORT = 4210;

// ESP32-CAM 영상 스트리밍 서버 포트입니다.
// 최종 스트림 주소는 http://ESP32_IP:81/stream 형태입니다.
const int STREAM_PORT = 81;


// ============================================================
// ESP32-CAM → Pico UART 설정
// ============================================================

// Pico와 통신할 UART 속도입니다.
// Pico 쪽 UART 속도와 반드시 같아야 합니다.
const uint32_t UART_BAUD = 115200;

// ESP32-CAM의 UART2 TX 핀입니다.
// 이 핀은 Pico RX 핀으로 연결됩니다.
const int UART_TX_PIN = 14;

// ESP32-CAM의 UART2 RX 핀입니다.
// 현재는 주로 송신 위주지만, UART2 초기화에 사용됩니다.
const int UART_RX_PIN = 15;


// ============================================================
// RAW 이미지 설정
// ============================================================

// Edge Impulse 모델 입력 크기입니다.
#define RAW_W 96
#define RAW_H 96

// 96 x 96 grayscale 이미지는 9216바이트입니다.
// grayscale uint8이므로 픽셀 1개가 1바이트입니다.
#define RAW_SIZE 9216

// RAW 버퍼 최대 크기입니다.
// 현재는 정확히 9216바이트만 받도록 설정합니다.
#define RAW_MAX_SIZE 9216


// ============================================================
// MJPEG 스트리밍 boundary 설정
// ============================================================

// HTTP multipart MJPEG 스트리밍에서 각 JPEG 프레임을 구분하는 문자열입니다.
#define PART_BOUNDARY "123456789000000000000987654321"

// 브라우저나 OpenCV가 연속 JPEG 스트림으로 인식하도록 하는 Content-Type입니다.
static const char* _STREAM_CONTENT_TYPE =
  "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;

// 각 JPEG 프레임 앞에 붙는 boundary입니다.
static const char* _STREAM_BOUNDARY =
  "\r\n--" PART_BOUNDARY "\r\n";

// 각 JPEG 프레임의 헤더입니다.
// Content-Length에 JPEG 데이터 크기가 들어갑니다.
static const char* _STREAM_PART =
  "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";


// ============================================================
// 전역 객체 및 버퍼
// ============================================================

// UDP 수신 객체입니다.
WiFiUDP udp;

// Pico와 통신할 UART2 객체입니다.
HardwareSerial picoSerial(2);

// HTTP 스트리밍 서버 핸들입니다.
httpd_handle_t stream_httpd = NULL;

// UDP 패킷을 임시로 담는 버퍼입니다.
// PC에서 오는 S, D, E, P, N 메시지를 먼저 여기에 받습니다.
uint8_t packetBuffer[1024];

// PC에서 나누어 보낸 RAW 데이터를 재조립할 버퍼입니다.
uint8_t rawAssembly[RAW_MAX_SIZE];

// 현재까지 재조립된 RAW 데이터 위치입니다.
size_t rawAssemblyPos = 0;

// PC가 Start 패킷에서 알려준 예상 RAW 크기입니다.
size_t rawExpectedSize = 0;

// RAW 수신이 진행 중인지 표시하는 플래그입니다.
bool rawInProgress = false;

// RAW 수신 시작 시간입니다.
// 일정 시간 안에 수신이 끝나지 않으면 timeout 처리합니다.
unsigned long rawStartedAt = 0;


// ============================================================
// MJPEG 스트리밍 처리 함수
// PC가 /stream 주소로 접속하면 계속 JPEG 프레임을 전송합니다.
// ============================================================
static esp_err_t stream_handler(httpd_req_t *req) {

  // 카메라 프레임 버퍼 포인터입니다.
  camera_fb_t* fb = NULL;

  // HTTP 응답 상태값입니다.
  esp_err_t res = ESP_OK;

  // JPEG 데이터 길이입니다.
  size_t jpg_len = 0;

  // JPEG 데이터 포인터입니다.
  uint8_t* jpg_buf = NULL;

  // 각 프레임의 HTTP 헤더 문자열을 담는 버퍼입니다.
  char part_buf[64];

  // 응답 타입을 MJPEG 스트림으로 설정합니다.
  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);

  // Content-Type 설정 실패 시 종료합니다.
  if (res != ESP_OK) return res;

  // 클라이언트가 연결되어 있는 동안 계속 프레임을 보냅니다.
  while (true) {

    // 카메라에서 한 프레임을 가져옵니다.
    fb = esp_camera_fb_get();

    // 프레임 획득 실패 시 에러 처리합니다.
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } else {

      // 프레임이 JPEG 형식이 아니면 JPEG로 변환합니다.
      if (fb->format != PIXFORMAT_JPEG) {

        // frame2jpg는 비 JPEG 프레임을 JPEG로 압축합니다.
        bool jpeg_converted = frame2jpg(fb, 80, &jpg_buf, &jpg_len);

        // 원본 프레임 버퍼를 반환합니다.
        esp_camera_fb_return(fb);
        fb = NULL;

        // JPEG 변환 실패 시 에러 처리합니다.
        if (!jpeg_converted) {
          Serial.println("JPEG compression failed");
          res = ESP_FAIL;
        }

      } else {

        // 이미 JPEG 형식이면 그대로 사용합니다.
        jpg_len = fb->len;
        jpg_buf = fb->buf;
      }
    }

    // MJPEG boundary 전송
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }

    // JPEG 프레임 헤더 전송
    if (res == ESP_OK) {
      size_t hlen = snprintf(part_buf, sizeof(part_buf), _STREAM_PART, jpg_len);
      res = httpd_resp_send_chunk(req, part_buf, hlen);
    }

    // 실제 JPEG 이미지 데이터 전송
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char*)jpg_buf, jpg_len);
    }

    // 사용한 카메라 프레임 버퍼 반환
    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      jpg_buf = NULL;
    }

    // frame2jpg로 새로 만든 JPEG 버퍼는 free 해줍니다.
    else if (jpg_buf) {
      free(jpg_buf);
      jpg_buf = NULL;
    }

    // 전송 중 에러가 발생하면 스트리밍 루프 종료
    if (res != ESP_OK) break;
  }

  return res;
}


// ============================================================
// 스트리밍 서버 시작 함수
// /stream 주소를 등록하여 PC에서 영상을 받을 수 있게 합니다.
// ============================================================
void startStreamServer() {

  // 기본 HTTP 서버 설정을 가져옵니다.
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();

  // HTTP 서버 포트를 81번으로 설정합니다.
  config.server_port = STREAM_PORT;

  // 제어 포트는 스트림 포트 + 1로 설정합니다.
  config.ctrl_port = STREAM_PORT + 1;

  // /stream 주소로 들어오는 요청을 stream_handler가 처리하게 설정합니다.
  httpd_uri_t stream_uri = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL
  };

  // HTTP 서버 시작
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {

    // /stream 핸들러 등록
    httpd_register_uri_handler(stream_httpd, &stream_uri);

    // 서버 시작 메시지 출력
    Serial.printf("Stream server started: %d\n", STREAM_PORT);

  } else {

    // 서버 시작 실패 메시지 출력
    Serial.println("Stream server failed");
  }
}


// ============================================================
// RAW 데이터를 Pico로 UART 전송하는 함수
//
// ESP32-CAM -> Pico 프로토콜:
// 시작마커 : 0xAA 0x55
// 크기     : size_lo size_hi
// 데이터   : RAW 9216 bytes
// 종료마커 : 0x55 0xAA
// ============================================================
void sendRawToPico(const uint8_t* raw_data, size_t raw_size) {

  // RAW 크기가 9216바이트가 아니면 전송하지 않습니다.
  if (raw_size != RAW_SIZE) {
    Serial.printf("[RAW] size error: %d\n", raw_size);
    return;
  }

  // 디버그용 checksum 계산 변수입니다.
  uint32_t checksum = 0;

  // RAW 이미지의 최소 밝기값입니다.
  uint8_t minv = 255;

  // RAW 이미지의 최대 밝기값입니다.
  uint8_t maxv = 0;

  // RAW 전체 픽셀을 검사하여 checksum, min, max를 계산합니다.
  for (size_t i = 0; i < raw_size; i++) {
    uint8_t v = raw_data[i];
    checksum += v;
    if (v < minv) minv = v;
    if (v > maxv) maxv = v;
  }

  // UART 전송 시작 로그
  Serial.println("[UART] RAW send start");

  // 전송될 RAW 데이터 상태 출력
  Serial.printf("[UART] size=%d checksum=%lu min=%d max=%d avg=%lu\n",
                raw_size, checksum, minv, maxv, checksum / raw_size);

  // Pico UART 수신 안정성을 위해 작은 단위로 나누어 전송합니다.
  const size_t CHUNK_SIZE = 64;

  // 각 청크 사이의 지연 시간입니다.
  const int CHUNK_DELAY_US = 2000;

  // 시작 마커 전송
  picoSerial.write(0xAA);
  picoSerial.write(0x55);

  // RAW 크기를 little-endian 방식으로 전송합니다.
  picoSerial.write((uint8_t)(raw_size & 0xFF));
  picoSerial.write((uint8_t)((raw_size >> 8) & 0xFF));

  // UART 버퍼를 비웁니다.
  picoSerial.flush();

  // Pico가 헤더를 처리할 시간을 줍니다.
  delayMicroseconds(CHUNK_DELAY_US);

  // 현재까지 전송한 바이트 수입니다.
  size_t sent = 0;

  // RAW 데이터를 64바이트씩 나누어 전송합니다.
  while (sent < raw_size) {

    // 남은 데이터 크기를 계산합니다.
    size_t chunk = raw_size - sent;

    // 한 번에 최대 64바이트까지만 보냅니다.
    if (chunk > CHUNK_SIZE) chunk = CHUNK_SIZE;

    // 현재 청크 전송
    picoSerial.write(raw_data + sent, chunk);

    // UART 버퍼 비우기
    picoSerial.flush();

    // 전송 위치 갱신
    sent += chunk;

    // Pico 수신 안정성을 위한 짧은 대기
    delayMicroseconds(CHUNK_DELAY_US);
  }

  // 종료 마커 전송
  picoSerial.write(0x55);
  picoSerial.write(0xAA);
  picoSerial.flush();

  // 전송 완료 로그
  Serial.printf("[UART] RAW send done: %d + header 6\n", raw_size);
}


// ============================================================
// 카메라 초기화 함수
// AI-Thinker ESP32-CAM 핀맵과 QVGA JPEG 스트림을 설정합니다.
// ============================================================
bool setupCamera() {

  // 카메라 설정 구조체입니다.
  camera_config_t config;

  // LEDC는 XCLK 생성에 사용됩니다.
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;

  // 카메라 데이터 핀 설정
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;

  // 카메라 동기화 및 클럭 핀 설정
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;

  // SCCB, 즉 카메라 설정용 I2C 핀입니다.
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;

  // 전원 제어 및 리셋 핀 설정
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  // 카메라 XCLK 주파수 설정
  config.xclk_freq_hz = 20000000;

  // 스트리밍용으로 JPEG 포맷 사용
  config.pixel_format = PIXFORMAT_JPEG;

  // QVGA는 320x240 해상도입니다.
  config.frame_size = FRAMESIZE_QVGA;

  // JPEG 품질 설정
  // 숫자가 낮을수록 품질은 좋고 데이터는 커집니다.
  config.jpeg_quality = 12;

  // 프레임 버퍼 개수 기본값
  config.fb_count = 1;

  // 기본 프레임 획득 방식
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

  // 기본은 DRAM 사용
  config.fb_location = CAMERA_FB_IN_DRAM;

  // PSRAM이 있으면 스트리밍 안정성을 높이기 위해 PSRAM 사용
  if (psramFound()) {
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
    config.jpeg_quality = 10;
    Serial.println("PSRAM found");
  } else {
    Serial.println("PSRAM not found");
  }

  // 카메라 초기화 실행
  esp_err_t err = esp_camera_init(&config);

  // 초기화 실패 시 false 반환
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }

  // 카메라 센서 객체 가져오기
  sensor_t* s = esp_camera_sensor_get();

  // OV3660 센서일 경우 영상 방향과 색감을 보정합니다.
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }

  // 최종 해상도를 QVGA로 설정합니다.
  s->set_framesize(s, FRAMESIZE_QVGA);

  // 카메라 초기화 성공
  return true;
}


// ============================================================
// setup()
// ESP32-CAM 부팅 시 한 번만 실행됩니다.
// 카메라, Wi-Fi, 스트리밍 서버, UDP, UART를 초기화합니다.
// ============================================================
void setup() {

  // USB 시리얼 모니터 시작
  Serial.begin(115200);

  // 부팅 직후 안정화를 위한 대기
  delay(1000);

  Serial.println();
  Serial.println("=== ESP32-CAM RAW Reassembly Firmware Final ===");

  // 카메라 초기화
  if (!setupCamera()) {
    Serial.println("Camera setup failed");
    return;
  }

  // Wi-Fi 연결 시작
  WiFi.begin(ssid, password);

  // Wi-Fi 절전 모드 해제
  // 실시간 스트리밍과 UDP 수신 안정성을 위해 필요합니다.
  WiFi.setSleep(false);

  Serial.print("WiFi connecting");

  // Wi-Fi가 연결될 때까지 대기
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // 연결된 IP 출력
  Serial.println();
  Serial.print("WiFi IP: ");
  Serial.println(WiFi.localIP());

  // MJPEG 스트리밍 서버 시작
  startStreamServer();

  // UDP 수신 시작
  udp.begin(UDP_PORT);
  Serial.printf("UDP listening: %d\n", UDP_PORT);

  // Pico와 UART 통신 시작
  picoSerial.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  // UART 설정 정보 출력
  Serial.printf("UART to Pico: %d bps, TX=%d RX=%d\n",
                UART_BAUD, UART_TX_PIN, UART_RX_PIN);

  // PC에서 사용할 스트림 주소 출력
  Serial.print("Stream URL: http://");
  Serial.print(WiFi.localIP());
  Serial.println(":81/stream");
}


// ============================================================
// loop()
// 계속 반복 실행됩니다.
//
// 처리하는 UDP 메시지:
// P,x,y : 손 좌표
// N     : 손 없음
// S     : RAW 시작
// D     : RAW 데이터 청크
// E     : RAW 끝
// ============================================================
void loop() {

  // RAW 수신 중인데 2초 이상 완료되지 않으면 timeout 처리합니다.
  if (rawInProgress && millis() - rawStartedAt > 2000) {
    Serial.printf("[UDP] RAW timeout: pos=%d expected=%d\n",
                  rawAssemblyPos, rawExpectedSize);

    // RAW 수신 상태 초기화
    rawInProgress = false;
    rawAssemblyPos = 0;
    rawExpectedSize = 0;
  }

  // UDP 패킷이 들어왔는지 확인합니다.
  int packetSize = udp.parsePacket();

  // 들어온 패킷이 없으면 잠깐 쉬고 loop를 다시 시작합니다.
  if (packetSize <= 0) {
    delay(1);
    return;
  }

  // UDP 패킷 내용을 packetBuffer로 읽습니다.
  int len = udp.read(packetBuffer, sizeof(packetBuffer));

  // 읽은 데이터가 없으면 종료
  if (len <= 0) return;

  // 첫 번째 바이트로 메시지 종류를 판단합니다.
  char msgType = (char)packetBuffer[0];

  // ----------------------------------------------------------
  // 1. 손 좌표 또는 손 없음 메시지 처리
  // P,x,y : 손 좌표
  // N     : 손 없음
  // ----------------------------------------------------------
  if (msgType == 'P' || msgType == 'N') {

    // 문자열 끝 표시 추가
    packetBuffer[len] = '\0';

    // 수신한 좌표 메시지 출력
    Serial.printf("[COORD] %s\n", packetBuffer);

    // Pico로 좌표 메시지 전달
    picoSerial.print((char*)packetBuffer);
    picoSerial.println();

    return;
  }

  // ----------------------------------------------------------
  // 2. RAW 시작 패킷 처리
  // 형식: S + size_lo + size_hi
  // ----------------------------------------------------------
  if (msgType == 'S' && len >= 3) {

    // little-endian 방식으로 RAW 크기 복원
    rawExpectedSize =
      (uint8_t)packetBuffer[1] |
      ((uint8_t)packetBuffer[2] << 8);

    Serial.printf("[UDP] RAW Start: expected=%d\n", rawExpectedSize);

    // 기대한 크기가 9216이 아니면 잘못된 패킷으로 판단
    if (rawExpectedSize != RAW_SIZE) {
      Serial.printf("[UDP] RAW size invalid: %d\n", rawExpectedSize);

      // 수신 상태 초기화
      rawInProgress = false;
      rawAssemblyPos = 0;
      rawExpectedSize = 0;

      return;
    }

    // RAW 재조립 시작
    rawAssemblyPos = 0;
    rawInProgress = true;
    rawStartedAt = millis();

    return;
  }

  // ----------------------------------------------------------
  // 3. RAW 데이터 청크 처리
  // 형식: D + index + raw_payload
  //
  // 주의:
  // 현재 코드는 index 값은 검사하지 않고,
  // 도착한 순서대로 payload를 이어 붙입니다.
  // ----------------------------------------------------------
  if (msgType == 'D' && rawInProgress && len > 2) {

    // 실제 RAW payload 길이입니다.
    // 첫 바이트 D, 두 번째 바이트 index를 제외합니다.
    size_t payload_len = len - 2;

    // 버퍼 범위를 넘지 않으면 RAW 조립 버퍼에 복사합니다.
    if (rawAssemblyPos + payload_len <= RAW_MAX_SIZE) {

      // packetBuffer + 2 위치부터 실제 RAW 데이터입니다.
      memcpy(rawAssembly + rawAssemblyPos, packetBuffer + 2, payload_len);

      // 현재까지 조립된 위치 갱신
      rawAssemblyPos += payload_len;

      Serial.printf("[UDP] D chunk: +%d -> %d/%d\n",
                    payload_len, rawAssemblyPos, rawExpectedSize);

    } else {

      // 버퍼 초과 발생 시 수신을 취소합니다.
      Serial.printf("[UDP] overflow: pos=%d + len=%d\n",
                    rawAssemblyPos, payload_len);

      rawInProgress = false;
      rawAssemblyPos = 0;
      rawExpectedSize = 0;
    }

    return;
  }

  // ----------------------------------------------------------
  // 4. RAW 종료 패킷 처리
  // 형식: E
  // ----------------------------------------------------------
  if (msgType == 'E' && rawInProgress) {

    Serial.printf("[UDP] RAW End: %d/%d\n",
                  rawAssemblyPos, rawExpectedSize);

    // 정확히 9216바이트가 모두 모였는지 확인합니다.
    if (rawAssemblyPos == RAW_SIZE && rawExpectedSize == RAW_SIZE) {

      // Pico로 RAW 데이터 전송
      sendRawToPico(rawAssembly, RAW_SIZE);

    } else {

      // 데이터가 부족하면 Pico로 보내지 않습니다.
      Serial.println("[UDP] RAW incomplete -> cancel UART send");
    }

    // RAW 수신 상태 초기화
    rawInProgress = false;
    rawAssemblyPos = 0;
    rawExpectedSize = 0;

    return;
  }

  // ----------------------------------------------------------
  // 5. 알 수 없는 패킷 처리
  // ----------------------------------------------------------
  Serial.printf("[UDP] unknown packet: 0x%02X len=%d\n", msgType, len);
}
