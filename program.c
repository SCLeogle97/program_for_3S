#include <dht11.h>
#include <TimerOne.h>
#include <HttpPacket.h>
#include <ArduinoJson.h>
#include <LiquidCrystal.h> 
LiquidCrystal lcd(3,4,7,8,11,12,13); 
dht11 DHT11;
HttpPacketHead packet;

#define DHT11PIN 2
#define DebugSerial Serial
#define ESP8266Serail Serial3
#define Success 1U
#define Failure 0U

int Echo = A5;  
int Trig =A4;  
int Distance = 0;
int Left_motor_back=9;     
int Left_motor_go=5;     
int Right_motor_go=6;    
int Right_motor_back=10;    
int key=A0;
int beep=A1;
long  Time_Cont = 0;       
int esp8266RxBufferLength = 600;
char esp8266RxBuffer[esp8266RxBufferLength];
int ii = 0;
char OneNetServer[] = "api.heclouds.com";       
char ssid[] = "Mercury_623(2)";     
char password[] = "houzi2017"; 
char device_id[] = "4656200";    
char API_KEY[] = "51ohPAODAcvhazA00YLBI7Mvd3c=";    
char sensor_id1[] = "TEMP";
char sensor_id2[] = "HUMI";

void setup()
{
  ESP8266Serail.begin(115200);
  Timer1.initialize(1000);
  imer1.attachInterrupt(Timer1_handler);
  initEsp8266();
  Serial.begin(9600);     
  Serial.println("DHT11 TEST PROGRAM ");
  Serial.print("LIBRARY VERSION: ");
  Serial.println(DHT11LIB_VERSION);
  Serial.println();
  pinMode(Left_motor_go,OUTPUT); 
  pinMode(Left_motor_back,OUTPUT); 
  pinMode(Right_motor_go,OUTPUT);
  pinMode(Right_motor_back,OUTPUT);
  pinMode(key,INPUT);
  pinMode(beep,OUTPUT);
  pinMode(Echo, INPUT);    
  pinMode(Trig, OUTPUT);   
  lcd.begin(16,2);                              
}


void run()     // 前进
{
  digitalWrite(Right_motor_go,HIGH);  
  digitalWrite(Right_motor_back,LOW);     
  analogWrite(Right_motor_go,100);
  analogWrite(Right_motor_back,0);
  digitalWrite(Left_motor_go,HIGH); 
  digitalWrite(Left_motor_back,LOW);
  analogWrite(Left_motor_go,100);
  analogWrite(Left_motor_back,0);
  delay(time * 100);   
}

void brake(int time)  //刹车
{
  digitalWrite(Right_motor_go,LOW);
  digitalWrite(Right_motor_back,LOW);
  digitalWrite(Left_motor_go,LOW);
  digitalWrite(Left_motor_back,LOW);
  delay(time * 100); 
}

void spin_left(int time)   //左转
{
  digitalWrite(Right_motor_go,HIGH);  
  digitalWrite(Right_motor_back,LOW);
  analogWrite(Right_motor_go,100); 
  analogWrite(Right_motor_back,0);
  digitalWrite(Left_motor_go,LOW);  
  digitalWrite(Left_motor_back,HIGH);
  analogWrite(Left_motor_go,0); 
  analogWrite(Left_motor_back,100);
  delay(time * 100);    
}

void spin_right(int time)        //右转
{
  digitalWrite(Right_motor_go,LOW);   
  digitalWrite(Right_motor_back,HIGH);
  analogWrite(Right_motor_go,0); 
  analogWrite(Right_motor_back,100);
  digitalWrite(Left_motor_go,HIGH);
  digitalWrite(Left_motor_back,LOW);
  analogWrite(Left_motor_go,100); 
  analogWrite(Left_motor_back,0);
  delay(time * 100);  
}

void back(int time)          //后退
{
  digitalWrite(Right_motor_go,LOW); 
  digitalWrite(Right_motor_back,HIGH);
  analogWrite(Right_motor_go,0);
  analogWrite(Right_motor_back,100);
  digitalWrite(Left_motor_go,LOW);  
  digitalWrite(Left_motor_back,HIGH);
  analogWrite(Left_motor_go,0);
  analogWrite(Left_motor_back,150);
  delay(time * 100);     
}



double dewPoint(double celsius, double humidity)//转换湿度
{
        double A0= 373.15/(273.15 + celsius);
        double SUM = -7.90298 * (A0-1);
        SUM += 5.02808 * log10(A0);
        SUM += -1.3816e-7 * (pow(10, (11.344*(1-1/A0)))-1) ;
        SUM += 8.1328e-3 * (pow(10,(-3.49149*(A0-1)))-1) ;
        SUM += log10(1013.246);
        double VP = pow(10, SUM-3) * humidity;
        double T = log(VP/0.61078);  
        return (241.88 * T) / (17.558-T);
}

double Fahrenheit(double celsius) //转换温度
{
        return 1.8 * celsius + 32;
}    


void keyscan()//按键扫描
{
  int val;
  val=digitalRead(key);
  while(!digitalRead(key))
  {
    val=digitalRead(key);
  }
  while(digitalRead(key))
  {
    delay(10);  
    val=digitalRead(key);
    if(val==HIGH) 
    {
      digitalWrite(beep,HIGH);    
      while(!digitalRead(key)) 
        digitalWrite(beep,LOW);  
    }
    else
      digitalWrite(beep,LOW);          
  }
}

void Distance_test()   // 量出前方距离 
{
  digitalWrite(Trig, LOW);   
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);  
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);   
  float Fdistance = pulseIn(Echo, HIGH);  
  Fdistance= Fdistance/58;       
 
  Serial.print("Distance:");      
  Serial.println(Fdistance);         
  Distance = Fdistance;
}  


void Distance_display()//显示距离
{
   if((2<Distance)&(Distance<400))
  {
    lcd.home();         
    lcd.print("    Distance: ");      
    lcd.print(Distance);        
    lcd.print("cm");            
  }
  else
  {
    lcd.home();         
    lcd.print("!!! Out of range");       
  }


  delay(250);
  lcd.clear();
}

void react() //数据测试
{
  Serial.println("\n");

  int chk = DHT11.read(DHT11PIN);

  Serial.print("Read sensor: ");
  switch (chk)
  {
    case DHTLIB_OK: 
                Serial.println("OK"); 
                break;
    case DHTLIB_ERROR_CHECKSUM: 
                Serial.println("Checksum error"); 
                break;
    case DHTLIB_ERROR_TIMEOUT: 
                Serial.println("Time out error"); 
                break;
    default: 
                Serial.println("Unknown error"); 
                break;
  }


  
  postDataToOneNet(API_KEY,device_id,sensor_id1,DHT11.temperature);
  delay(100);       
  postDataToOneNet(API_KEY,device_id,sensor_id2,DHT11.humidity);
  delay(500);

  
  }


void postDataToOneNet(char* API_VALUE_temp,char* device_id_temp,char* sensor_id_temp,double thisData)//上传数据
{
        
    StaticJsonBuffer<200> jsonBuffer;

    JsonObject& value = jsonBuffer.createObject();
    value["value"] = thisData;

    JsonObject& id_datapoints = jsonBuffer.createObject();
    id_datapoints["id"] = sensor_id_temp;
    JsonArray& datapoints = id_datapoints.createNestedArray("datapoints");
    datapoints.add(value);

    JsonObject& myJson = jsonBuffer.createObject();
    JsonArray& datastreams = myJson.createNestedArray("datastreams");
    datastreams.add(id_datapoints);

    char p[200];
    int num = myJson.printTo(p, sizeof(p));

	packet.setHostAddress(OneNetServer);
    packet.setDevId(device_id_temp);  
    packet.setAccessKey(API_VALUE_temp); 
	
    packet.createCmdPacket(POST, TYPE_DATAPOINT, p);

    int httpLength = strlen(packet.content) + num;

    char cmd[400];
    memset(cmd, 0, 400);    
    strcpy(cmd, "AT+CIPSTART=\"TCP\",\"");
    strcat(cmd, OneNetServer);
    strcat(cmd, "\",80\r\n");
    if (sendCommand(cmd, "CONNECT", 7, 10000, 5) == Success);
    else ESP8266_ERROR(1);

   
    memset(cmd, 0, 400);   
    sprintf(cmd, "AT+CIPSEND=%d\r\n", httpLength);
    if (sendCommand(cmd, ">", 1, 3000, 1) == Success);
    else ESP8266_ERROR(2);

    memset(cmd, 0, 400);    
    strcpy(cmd, packet.content);
    strcat(cmd, p);
    if (sendCommand(cmd, "\"succ\"}", 7, 3000, 3) == Success);
    else ESP8266_ERROR(3);

    if (sendCommand("AT+CIPCLOSE\r\n", "CLOSED", 6, 3000, 1) == Success);
    else ESP8266_ERROR(4);
}

void initEsp8266() //启动esp8266
{
    if (sendCommand("AT\r\n", "OK", 2, 3000, 10) == Success);
    else ESP8266_ERROR(5);

    if (sendCommand("AT+RST\r\n", "ready", 5, 10000, 10) == Success);
    else ESP8266_ERROR(6);

    if (sendCommand("AT+CWMODE=1\r\n", "OK", 2, 3000, 10) == Success);
    else ESP8266_ERROR(7);

    char cmd[50];
    strcpy(cmd, "AT+CWJAP=\"");
    strcat(cmd, ssid);
    strcat(cmd, "\",\"");
    strcat(cmd, password);
    strcat(cmd, "\"\r\n");

    if (sendCommand(cmd, "OK", 2, 20000, 10) == Success);
    else ESP8266_ERROR(8);

    if (sendCommand("AT+CIPMUX=0\r\n", "OK", 2, 3000, 10) == Success);
    else ESP8266_ERROR(9);

    if (sendCommand("AT+CIFSR\r\n", "OK", 2, 20000, 10) == Success);
    else ESP8266_ERROR(10);
}

void(* resetFunc) (void) = 0; //制造重启命令 

void ESP8266_ERROR(int num) //判断
{
    Serial.print("ERROR");
    Serial.println(num);
    while (1)
    {
        if (sendCommand("AT\r\n", "OK", 2, 100, 10) == Success)
        {
            Serial.print("\r\nRESET!!!!!!\r\n");
            resetFunc();
        }
    }
}

int sendCommand(char *Command, char *Response, unsigned int Res_Length, unsigned long Timeout, unsigned char Retry) //发送命令
{
    clrEsp8266RxBuffer();
    for (unsigned char n = 0; n < Retry; n++)
    {
        Serial.print("\r\nsend AT Command:\r\n----------\r\n");
        Serial.write(Command);

        ESP8266Serail.write(Command);

        Time_Cont = 0;
        while (Time_Cont < Timeout)
        {
            esp8266ReadBuffer();
            if ((mystrstr(esp8266RxBuffer, Response, ii, Res_Length)) != NULL)
            {
                Serial.print("\r\nreceive AT Command:\r\n==========\r\n");
                Serial.print(esp8266RxBuffer); 
                clrEsp8266RxBuffer();
                return Success;
            }
        }
        Time_Cont = 0;
    }
    Serial.print("\r\nreceive AT Command:\r\n==========\r\n");
    Serial.print(esp8266RxBuffer);
    clrEsp8266RxBuffer();
    return Failure;
}

char mystrstr(char *s, char *t, unsigned int Length_s, unsigned int Length_t) //值度量
{   char x = 0; char *p; p = t;
    int i = 0, j = 0;
    for (; i < Length_s; s++, i++)
    {
        while (*t == *s)
        {   s++; t++; i++; j++;
            if (j >= Length_t) return 1;
        }
        s -= j;
        t = p; j = 0;
    }
    return NULL;
}


void Timer1_handler(void) //计时
{
    Time_Cont++;
}



void esp8266ReadBuffer() //读取值
{
    while (ESP8266Serail.available())
    {
        esp8266RxBuffer[ii++] = ESP8266Serail.read();
        if (ii == esp8266RxBufferLength)clrEsp8266RxBuffer();
    }
}

void clrEsp8266RxBuffer(void) //清空
{
    memset(esp8266RxBuffer, 0, esp8266RxBufferLength);      //清空
    ii = 0;
}

void loop()
{
  keyscan();     //调用按键扫描函数
  react();
  while(1)
  {
    Distance_test();//测量前方距离
    Distance_display();//液晶屏显示距离
    if(Distance < 60)//数值为碰到障碍物的距离，可以按实际情况设置
      while(Distance < 60)//再次判断是否有障碍物，若有则转动方向后，继续判断
      {
        right(1);//右转
        brake(1);//停车
        Distance_test();//测量前方距离
        Distance_display();//液晶屏显示距离
      }
    else
      run();//无障碍物，直行
  }
}









