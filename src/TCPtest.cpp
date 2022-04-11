#include <Arduino.h>
#include <WiFi.h>

const char* ssid = "ご注文はWIFIですか?";
const char* password = "111111111";

WiFiServer server;  //声明服务器对象

void setup() {
    Serial.begin(115200);  //串口初始化
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);  //关闭STA模式下wifi休眠，提高响应速度
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("Connected");
    Serial.print("IP Address:");
    Serial.println(WiFi.localIP());
    server.begin(80);  //服务器启动，端口号2333
}

void loop() {
    WiFiClient client = server.available();  //尝试建立客户对象
    if (client)                              //如果当前客户可用
    {
        Serial.println("[存在客户端连接]");
        String readBuff;            //读取信息暂存
        while (client.connected())  //如果客户端处于连接状态
        {
            if (client.available())  //如果有可读数据
            {
                char c = client.read();  //读取一个字节
                                         //也可以用readLine()等其他方法
                readBuff += c;
                if (c == '\r')  //接收到回车符
                {
                    Serial.println("已收到: " + readBuff);  //从串口打印
                    delay(100);
                    readBuff = "";
                    //break;  //跳出循环
                }
            }
        }
        // client.stop(); //结束当前连接:
        // Serial.println("[无客户端连接]");
    }
}
