#include <WiFi.h>

// Параметры для подключения к Wi-Fi
char ssid[] = "Deus wifi 5";  // Изменено на char[]
char password[] = "winix666";  // Изменено на char[]

// URLs для API запросов
const char* serverName = "rms-dev.deusrobotics.com";
const char* authPath = "/identity/api/v1/Authenticate/signin";
const char* apiPathStatus = "/operation/api/v1/Operation/planner/status";
const char* apiPathEnable = "/operation/api/v1/Operation/planner/disable";
const char* apiPathDisable = "/operation/api/v1/Operation/planner/disable";

// Учетные данные пользователя
const char* login = "devs0";
const char* userPassword = "devs0";

// Пины
const int BUTTON_PIN = PA30;
const int LED_PIN = PA13;

// Переменные для кнопки
int buttonState = HIGH;
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

// Переменная для хранения токена доступа
String accessToken;

// Переменная для отслеживания состояния приложения
bool isAppRunning = true;

// Переменные для работы с токеном
unsigned long tokenExpirationTime = 0;
const unsigned long TOKEN_REFRESH_INTERVAL = 15 * 60 * 1000; // 15 минут в миллисекундах

WiFiSSLClient client;

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  // Подключение к Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Подключение к WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nПодключено к Wi-Fi");
  Serial.print("IP адрес: ");
  Serial.println(WiFi.localIP());

  // Получение начального токена доступа
  getAccessToken();

  // Получение начального состояния приложения
  checkAppStatus();
}


void loop() {
  // Проверяем, не истек ли токен
  if (millis() > tokenExpirationTime) {
    getAccessToken();
  }

  int reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) {
        Serial.println("Кнопка нажата");
        toggleAppState();
      }
    }
  }

  lastButtonState = reading;
}

void getAccessToken() {
  Serial.println("Получение токена доступа");
  
  String postData = "{\"login\":\"" + String(login) + "\",\"password\":\"" + String(userPassword) + "\"}";
  
  String response = sendRequest(authPath, postData);
  if (response.length() > 0) {
    // Простой парсинг JSON ответа
    int tokenStart = response.indexOf("\"accessToken\":\"") + 15;
    int tokenEnd = response.indexOf("\"", tokenStart);
    if (tokenStart > 14 && tokenEnd > tokenStart) {
      accessToken = response.substring(tokenStart, tokenEnd);
      Serial.println("Токен получен: " + accessToken);
      tokenExpirationTime = millis() + TOKEN_REFRESH_INTERVAL;
    } else {
      Serial.println("Не удалось извлечь токен из ответа");
    }
  } else {
    Serial.println("Не удалось подключиться к серверу аутентификации");
  }
}

void toggleAppState() {
  const char* path = isAppRunning ? apiPathDisable : apiPathEnable;
  Serial.println(isAppRunning ? "Попытка остановить приложение" : "Попытка запустить приложение");
  
  String postData = "{}";  // Пустой JSON объект
  String response = sendRequest(path, postData);
  if (response.length() > 0) {
    // Простая проверка успешности операции
    if (response.indexOf("\"success\":true") != -1) {
      Serial.println(isAppRunning ? "Приложение успешно остановлено" : "Приложение успешно запущено");
      isAppRunning = !isAppRunning;
    } else {
      Serial.println("Не удалось изменить состояние приложения");
    }
  } else {
    Serial.println("Не удалось изменить состояние приложения");
  }
  
  delay(1000);
  checkAppStatus();
}

void checkAppStatus() {
  Serial.println("Проверка статуса приложения");
  String response = sendRequest(apiPathStatus, "{}");
  if (response.length() > 0) {
    // Простой парсинг JSON ответа
    int stateStart = response.indexOf("\"state\":") + 8;
    int stateEnd = response.indexOf("}", stateStart);
    if (stateStart > 7 && stateEnd > stateStart) {
      int state = response.substring(stateStart, stateEnd).toInt();
      isAppRunning = (state == 3);
      Serial.println("Текущее состояние приложения: " + String(isAppRunning ? "Запущено" : "Остановлено"));
      digitalWrite(LED_PIN, isAppRunning ? HIGH : LOW);
    } else {
      Serial.println("Не удалось найти информацию о состоянии в ответе");
    }
  } else {
    Serial.println("Не удалось проверить статус приложения");
  }
}

String sendRequest(const char* path, String postData) {
  String response = "";
  if (client.connect(serverName, 443)) {
    Serial.println("Отправка POST запроса: " + String(path));
    
    String request = "POST " + String(path) + " HTTP/1.1\r\n";
    request += "Host: " + String(serverName) + "\r\n";
    request += "Authorization: Bearer " + accessToken + "\r\n";
    request += "Content-Type: application/json\r\n";
    request += "Accept: application/json\r\n";
    request += "Content-Length: " + String(postData.length()) + "\r\n";
    request += "Connection: close\r\n\r\n";
    request += postData;

    Serial.println("Отправляемый запрос:");
    Serial.println(request);
    
    client.print(request);
    
    response = readResponse();
  } else {
    Serial.println("Не удалось подключиться к API серверу");
  }
  return response;
}

String readResponse() {
  String response = "";
  bool headersParsed = false;
  unsigned long timeout = millis();
  while (client.connected() && millis() - timeout < 10000) {
    String line = client.readStringUntil('\n');
    if (line == "\r") {
      headersParsed = true;
    } else if (headersParsed) {
      response += line;
    }
    if (line.length() == 0 && headersParsed) {
      break;
    }
  }
  client.stop();
  Serial.println("Ответ сервера:");
  Serial.println(response);
  return response;
}