#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Defina o nome da sua rede WiFi e senha
const char* ssid = "Carazzato";
const char* password = "Juliana@1977";

#define BUFFER_SIZE 1024

// Configurações de pinos para a ESP32-CAM (AI Thinker Model)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// Variáveis de sincronização e controle
camera_fb_t *fb = NULL;  // Frame buffer para armazenar a última imagem capturada
SemaphoreHandle_t xSemaphore = NULL;  // Semáforo para sincronizar o acesso ao buffer

// Função de captura de imagens (Thread de captura de imagem)
void captureImageTask(void *pvParameters) {
  while (true) {
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
      if (fb != NULL) {
        esp_camera_fb_return(fb);  // Libera o frame anterior para capturar um novo
      }
      fb = esp_camera_fb_get();  // Captura uma nova imagem da câmera
      if (!fb) {
        Serial.println("Falha ao capturar o frame");
      } else {
        Serial.println("Imagem capturada e armazenada no buffer");
      }
      xSemaphoreGive(xSemaphore);  // Libera o semáforo
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);  // Aguarda 200ms antes de capturar a próxima imagem
  }
}

// Função de captura da deteccao (Thread de captura de deteccao)
void captureDetectionTask(void *pvParameters) {
  while (true) {
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
      uint8_t buffer[BUFFER_SIZE];
      int bytesRead = Serial.readBytes(buffer, sizeof(buffer));

      if (bytesRead > 0) {
      // Aqui você pode processar os dados recebidos
      Serial.println("Dados recebidos:");
      Serial.write(buffer, bytesRead);  // Envia os dados de volta para o monitor serial para verificar
      }
      xSemaphoreGive(xSemaphore);  // Libera o semáforo
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);  // Aguarda 200ms antes de capturar a próxima deteccao
  }
}

// Função de envio de imagem (quando uma requisição HTTP é recebida)
esp_err_t sendImageHandler(httpd_req_t *req) {
  esp_err_t res = ESP_OK;
  size_t jpg_buf_len = 0;
  uint8_t *jpg_buf = NULL;

  if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
    if (fb != NULL) {
      jpg_buf_len = fb->len;
      jpg_buf = fb->buf;

      // Define o tipo de conteúdo da resposta como imagem JPEG
      res = httpd_resp_set_type(req, "image/jpeg");
      if (res == ESP_OK) {
        // Envia a imagem para o cliente (Python)
        res = httpd_resp_send(req, (const char *)jpg_buf, jpg_buf_len);
        Serial.println("Imagem enviada para o cliente");
      } else {
        Serial.println("Falha ao definir o tipo de resposta");
      }
    } else {
      // Envia um erro 500 se não houver imagem no buffer
      httpd_resp_send_500(req);
      Serial.println("Nenhuma imagem disponível no buffer");
    }
    xSemaphoreGive(xSemaphore);  // Libera o semáforo
  }

  return res;
}

// Inicializa o servidor HTTP no ESP32
void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();

  // Cria um manipulador de URI para o endpoint "/capture"
  httpd_uri_t uri_handler = {
    .uri       = "/capture",    // Endpoint para capturar a imagem
    .method    = HTTP_GET,      // Método HTTP: GET
    .handler   = sendImageHandler,  // Função que envia a imagem capturada
    .user_ctx  = NULL
  };

  // Inicializa o servidor HTTP
  static httpd_handle_t camera_httpd = NULL;
  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &uri_handler);  // Registra o manipulador de URI
    Serial.println("Servidor HTTP inicializado");
  } else {
    Serial.println("Falha ao iniciar o servidor HTTP");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  // Conectar ao Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Conectado ao WiFi");
  Serial.print("Endereço IP: ");
  Serial.println(WiFi.localIP());

  // Inicializar a câmera
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Falha ao inicializar a câmera: 0x%x", err);
    return;
  }

  // Cria o semáforo para sincronização
  xSemaphore = xSemaphoreCreateMutex();

  // Cria a thread de captura de imagem
  xTaskCreatePinnedToCore(captureImageTask, "CapturaImagem", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(captureDetectionTask, "CapturaDeteccao", 2048, NULL, 2, NULL, 0);

  // Inicia o servidor HTTP para lidar com a requisição de envio da imagem
  startCameraServer();
}

void loop() {
  // O FreeRTOS gerencia as threads, não há necessidade de código na loop
}
