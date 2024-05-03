#include <Arduino.h>
#include <HardwareSerial.h>


#define RXPIN 16
#define TXPIN 17
HardwareSerial PM25Serial(1);

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  PM25Serial.begin(9600, SERIAL_8N1, RXPIN, TXPIN);
}

void loop()
{
  uint8_t sensorData[32] = {0};
  uint8_t sensorDataLength = PM25Serial.available();
  if (sensorDataLength >= 32)
  {
    Serial.println("Data length = " + String(sensorDataLength));
    for (int i = 0; i < sensorDataLength; i++)
    {
      sensorData[i] = PM25Serial.read();
    }

    for (int i = 0; i < sensorDataLength; i++)
    {
      Serial.print(sensorData[i]);
      Serial.print(" ");
    }
    Serial.println();
  }
}


// void setup() {
//   Serial.begin(115200);
//   pinMode(5, OUTPUT);
//   pinMode(4, OUTPUT);
//   digitalWrite(5, HIGH);
//   digitalWrite(4, HIGH);
//   const uart_port_t uart_num = UART_NUM_0;
//   uart_config_t uart_config = {
//       .baud_rate = 9600,
//       .data_bits = UART_DATA_8_BITS,
//       .parity = UART_PARITY_DISABLE,
//       .stop_bits = UART_STOP_BITS_1,
//       .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
//       .rx_flow_ctrl_thresh = 0,
//   };
//   // Configure UART parameters
//   ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
//   // Set UART pins(TX: IO4, RX: IO5, RTS: IO18, CTS: IO19)
//   ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, 17, 16, 0, 2));
//   // Setup UART buffered IO with event queue
//   const int uart_buffer_size = (1024 * 2);
//   QueueHandle_t uart_queue;
//   // Install UART driver using an event queue here
//   ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, uart_buffer_size,
//                                       uart_buffer_size, 10, &uart_queue, 0));
// }

// void loop() {
//   const uart_port_t uart_num = UART_NUM_0;
//   uint8_t data[128];
//   int length = 0;
//   ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t *)&length));
//   length = uart_read_bytes(uart_num, data, length, 100);
//   if (length > 0) {
//     for (int i = 0; i < length; i++) {
//       Serial.print(data[i]);
//     }
//     Serial.println();
//   }
//   delay(1000);
// }

// void setup()
// {
//   // put your setup code here, to run once:
//   Serial.begin(9600);
// }

// void loop()
// {
//   // put your main code here, to run repeatedly:
//   Serial.println("Hello World!");
//   delay(500);
// }