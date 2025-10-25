
# ESP32 S3 Power Monitor & Overload Protection with NETPIE 2020

โปรเจกต์นี้คือระบบตรวจสอบการใช้พลังงานไฟฟ้าด้วย ESP32 ร่วมกับโมดูล PZEM-004T v3.0 และแสดงผลผ่านจอ LCD I2C 16x2 โดยระบบจะเชื่อมต่อกับแพลตฟอร์ม NETPIE 2020 ผ่านโปรโตคอล MQTT เพื่อส่งข้อมูลการใช้พลังงาน (Volt, Amp, Power, Energy) และรับคำสั่งเพื่อควบคุมรีเลย์
จุดเด่นของโปรเจกต์นี้คือการตั้งค่า WiFi ที่ง่ายดายผ่าน **WiFiManager** (ไม่ต้องฮาร์ดโค้ด SSID/Password) และระบบ **Overload Protection** ที่สามารถตั้งค่ากระแสสูงสุดได้จากระยะไกลผ่าน NETPIE เมื่อกระแสเกินกำหนด รีเลย์จะตัดการทำงานอัตโนมัติ

## ✨ คุณสมบัติ (Features)

-   **วัดค่าพลังงานไฟฟ้าแบบเรียลไทม์** (Volt, Amp, Power, Energy) จาก PZEM-004T v3.0
    
-   **เชื่อมต่อแพลตฟอร์ม NETPIE 2020** ผ่าน MQTT
    
-   **ควบคุมรีเลย์ (เปิด/ปิด)** จากระยะไกลผ่าน NETPIE
    
-   **ระบบป้องกันกระแสไฟเกิน (Overload Protection)** พร้อมตัดไฟอัตโนมัติ
    
-   **ตั้งค่ากระแส Overload ได้จากระยะไกล** (ค่าจะถูกบันทึกใน NETPIE Shadow)
    
-   **ใช้ WiFiManager** สำหรับการตั้งค่า WiFi ครั้งแรกผ่าน Web Portal (ไม่ต้องแก้โค้ด)
    
-   **มีระบบล้างค่า WiFi** โดยการกดปุ่ม BOOT (GPIO 0) ค้างไว้ขณะเปิดเครื่อง
    
-   **แสดงผลสถานะ** และค่าที่วัดได้บนจอ 16x2 I2C LCD
    
-   **ซิงค์สถานะล่าสุด** (Relay, Overload) กับ NETPIE Device Shadow เมื่อเริ่มทำงาน
    

## 🛠️ อุปกรณ์ที่ต้องใช้ (Hardware Required)

1.  **ESP32 S3** 
    
2.  **PZEM-004T v3.0** (โมดูลวัดพลังงาน) พร้อม CT Coil
    
3.  **Relay Module** (โมดูลรีเลย์ 1 ช่อง)
    
4.  **16x2 I2C LCD Display** (จอแสดงผล)
    
5.  สายไฟ Jumper
    

## 📚 ไลบรารีที่จำเป็น (Libraries Required)

คุณต้องติดตั้งไลบรารีต่อไปนี้ผ่าน Arduino IDE Library Manager:

-   `PubSubClient` (by Nick O'Leary)
    
-   `ArduinoJson` (by Benoit Blanchon)
    
-   `LiquidCrystal_I2C` (by Frank de Brabander)
    
-   `PZEM-004T-v30` (by mandulaj)
    
-   `WiFiManager` (by tzapu)
    

## 🔌 การต่อวงจร (Hardware Setup / Wiring)

-   **I2C LCD Display:**
    
    -   `SDA` -> `GPIO 21` (ESP32)
        
    -   `SCL` -> `GPIO 22` (ESP32)
        
    -   `VCC` -> `5V`
        
    -   `GND` -> `GND`
        
-   **PZEM-004T v3.0 (UART):**
    
    -   `TX` -> `GPIO 17` (ESP32 RX2)
        
    -   `RX` -> `GPIO 18` (ESP32 TX2)
        
    -   `5V` -> `5V`
        
    -   `GND` -> `GND`
        
-   **Relay Module:**
    
    -   `IN` -> `GPIO 38` (ESP32)
        
    -   `VCC` -> `5V`
        
    -   `GND` -> `GND`
        
-   **ปุ่ม Reset WiFi:**
    
    -   ใช้ปุ่ม **BOOT** ที่อยู่บนบอร์ด (`GPIO 0`)
        

**ข้อควรระวัง:** `GPIO 38` ไม่มีในบอร์ด ESP32-WROOM-32 รุ่นมาตรฐานส่วนใหญ่ หากบอร์ดของคุณไม่มี Pin นี้ ให้เปลี่ยนค่า `int relay = 38;` เป็น GPIO อื่นที่ว่างอยู่

## ⚙️ การตั้งค่า (Configuration)

### 1. ตั้งค่า NETPIE 2020

1.  ไปที่ [NETPIE 2020](https://netpie.io/ "null") และสร้าง Device ใหม่
    
2.  จด **Device ID**, **Token** และ **Secret** จากหน้า Device
    
3.  นำค่าทั้ง 3 ไปใส่ในโค้ดส่วน `NETPIE2020` ในไฟล์ `.ino`:
    
    ```
    // -------- NETPIE2020 --------
    const char* mqttServer = "broker.netpie.io";
    const int mqttPort = 1883;
    const char* mqttClientID = "your Device ID";    // <-- ใส่ Device ID ของคุณ
    const char* mqttUsername = "your Token";      // <-- ใส่ Token ของคุณ
    const char* mqttPassword = "your Secret";     // <-- ใส่ Secret ของคุณ
    
    ```
    

### 2. อัปโหลดโค้ด และตั้งค่า WiFi

1.  อัปโหลดโค้ดนี้ไปยัง ESP32
    
2.  เมื่อเปิดเครื่องครั้งแรก ESP32 จะสร้าง Access Point (AP) ชื่อ **"ESP32-PZEM-Setup"**
    
3.  ใช้โทรศัพท์มือถือหรือคอมพิวเตอร์ เชื่อมต่อ WiFi นี้
    
4.  หน้า Captive Portal (หน้าตั้งค่า) จะเด้งขึ้นมาอัตโนมัติ (หากไม่ขึ้น ให้เปิด Browser แล้วไปที่ `192.168.4.1`)
    
5.  เลือก "Configure WiFi" จากนั้นเลือกชื่อ WiFi บ้านของคุณ และใส่รหัสผ่าน
    
6.  ESP32 จะบันทึกข้อมูลและเชื่อมต่อ WiFi ของคุณโดยอัตโนมัติ และจะเชื่อมต่อ NETPIE 2020 ต่อไป
    

### 3. การรีเซ็ตการตั้งค่า WiFi

หากต้องการเปลี่ยน WiFi หรือลบค่าที่บันทึกไว้:

1.  ถอดปลั๊ก ESP32
    
2.  กดปุ่ม **BOOT** (GPIO 0) ค้างไว้
    
3.  เสียบปลั๊ก ESP32 (โดยที่ยังกดปุ่ม BOOT ค้างไว้)
    
4.  รอ 3 วินาที (ตามที่โค้ดตั้งไว้) แล้วจึงปล่อยมือ
    
5.  อุปกรณ์จะลบค่า WiFi และหยุดทำงาน ให้ถอดปลั๊กแล้วเสียบใหม่เพื่อเริ่มการตั้งค่า WiFiManager อีกครั้ง
    

## 🚀 การทำงานกับ NETPIE 2020 (MQTT Topics)

### 1. Topics ที่ ESP32 ส่งข้อมูลไป (Publish)

-   `@shadow/data/update`
    
    -   **ข้อมูลเซ็นเซอร์ (ทุก 5 วินาที):**
        
        ```
        {"data":{"volt":220.50,"amp":1.234,"power":270.00,"e":10.500}}
        
        ```
        
    -   **อัปเดตสถานะ (เมื่อมีการเปลี่ยนแปลง):**
        
        -   เมื่อตั้งค่า Overload: `{"data":{"overload":5}}`
            
        -   เมื่อสั่งเปิด/ปิด Relay: `{"data":{"relay":1}}` หรือ `{"data":{"relay":0}}`
            
        -   เมื่อเกิด Overload: `{"data":{"relay":0,"StateOverload":1}}`
            
        -   เมื่อสั่งเปิด Relay (ล้างสถานะ Overload): `{"data":{"relay":1,"StateOverload":0}}`
            
-   `@shadow/data/get`
    
    -   **ESP32 ส่ง:** ส่ง Payload `{}` ว่างๆ ไปยัง Topic นี้ 1 ครั้งเมื่อเชื่อมต่อสำเร็จ เพื่อขอข้อมูลล่าสุดที่เก็บไว้ใน Shadow
        

### 2. Topics ที่ ESP32 รอรับคำสั่ง (Subscribe)

-   `@private/shadow/data/get/response`
    
    -   **ESP32 รับ:** รับข้อมูล Shadow (สถานะ `relay` และ `overload` ล่าสุด) หลังจากส่ง `@shadow/data/get` เพื่อให้สถานะของอุปกรณ์ตรงกันกับบนเซิร์ฟเวอร์
        
-   `@msg/relay`
    
    -   **ESP32 รับ:** รับคำสั่งควบคุมรีเลย์
        
    -   **Payload "1"**: เปิดรีเลย์ (และล้างสถานะ `StateOverload`)
        
    -   **Payload "0"**: ปิดรีเลย์
        
-   `@msg/overload`
    
    -   **ESP32 รับ:** รับค่าสำหรับตั้งค่ากระแสไฟสูงสุด (Overload)
        
    -   **Payload (ตัวอย่าง "5")**: ตั้งค่า Overload ที่ 5 แอมป์
        

### 3. โครงสร้างข้อมูลใน Shadow (Shadow Data Structure)

ข้อมูลที่ถูกเก็บไว้ใน NETPIE Device Shadow จะมีโครงสร้างดังนี้:

```
{
  "data": {
    "volt": 220.50,
    "amp": 1.234,
    "power": 270.00,
    "e": 10.500,
    "relay": 1,
    "overload": 5,
    "StateOverload": 0
  }
}

```

-   `relay`: สถานะรีเลย์ (1=ON, 0=OFF)
    
-   `overload`: ค่ากระแสสูงสุดที่ตั้งไว้ (เช่น 5 Amp)
    
-   `StateOverload`: สถานะ Overload (1=เกิด Overload, 0=ปกติ)
