# miniProject-pzem_netpie
ESP32 Power Monitor & Overload Protection with NETPIE 2020
โปรเจกต์นี้ใช้ ESP32 ร่วมกับโมดูลวัดพลังงาน PZEM-004T v3.0 เพื่อวัดค่าแรงดัน (Volt), กระแส (Amp), กำลังไฟฟ้า (Power) และพลังงาน (Energy) จากนั้นส่งข้อมูลไปยังแพลตฟอร์ม NETPIE 2020 ผ่านโปรโตคอล MQTT

นอกจากนี้ยังสามารถควบคุมการเปิด-ปิดรีเลย์ได้จากระยะไกล และมีระบบป้องกันกระแสไฟเกิน (Overload Protection) โดยอัตโนมัติ โดยค่า Overload สามารถตั้งค่าได้ผ่าน NETPIE

✨ คุณสมบัติ (Features)
วัดค่าพลังงานแบบเรียลไทม์: อ่านค่า Voltage, Current, Power, และ Energy

เชื่อมต่อ NETPIE 2020: ส่งข้อมูลขึ้น Dashboard และรับคำสั่งผ่าน MQTT

ควบคุมรีเลย์: สั่งเปิด-ปิดเครื่องใช้ไฟฟ้าผ่าน NETPIE

ป้องกันกระแสเกิน (Overload Protection): ตัดการทำงานของรีเลย์อัตโนมัติ เมื่อกระแสไฟฟ้าเกินค่าที่กำหนด

ตั้งค่า Overload ผ่าน NETPIE: สามารถกำหนดค่ากระแสสูงสุด (Amp) ที่อนุญาตได้จากระยะไกล

ซิงค์สถานะกับ Shadow: ใช้ NETPIE Device Shadow เพื่อจดจำสถานะล่าสุดของ relay และ overload

ตั้งค่าง่ายด้วย WiFiManager: ตั้งค่าการเชื่อมต่อ WiFi ผ่าน Captive Portal (Web Browser) โดยไม่ต้องแก้ไขโค้ด

แสดงผลบนจอ LCD: แสดงสถานะการเชื่อมต่อ, ค่า Volt, Amp และสถานะ Overload

รีเซ็ต WiFi: ลบการตั้งค่า WiFi ที่บันทึกไว้ โดยการกดปุ่ม BOOT (GPIO 0) ค้างไว้ขณะเปิดเครื่อง

🛠️ อุปกรณ์ที่ต้องใช้ (Hardware Required)
ESP32 (รุ่นใดก็ได้ที่มี GPIO 38 เช่น ESP32-WROVER)

PZEM-004T v3.0 (โมดูลวัดพลังงาน) พร้อม CT Coil

Relay Module (โมดูลรีเลย์ 1 ช่อง)

16x2 I2C LCD Display (จอแสดงผล)

สายไฟ Jumper และ Breadboard (หากจำเป็น)

📚 ไลบรารีที่จำเป็น (Libraries Required)
คุณต้องติดตั้งไลบรารีต่อไปนี้ผ่าน Arduino IDE Library Manager:

PubSubClient (by Nick O'Leary)

ArduinoJson (by Benoit Blanchon)

LiquidCrystal_I2C (by Frank de Brabander)

PZEM-004T-v30 (by mandulaj)

WiFiManager (by tzapu)

🔌 การต่อวงจร (Hardware Setup / Wiring)
I2C LCD Display:

SDA -> GPIO 21 (ESP32)

SCL -> GPIO 22 (ESP32)

VCC -> 5V

GND -> GND

PZEM-004T v3.0 (UART):

TX -> GPIO 17 (ESP32 RX2)

RX -> GPIO 18 (ESP32 TX2)

5V -> 5V

GND -> GND

Relay Module:

IN -> GPIO 38 (ESP32)

VCC -> 5V

GND -> GND

Reset Button:

ใช้ปุ่ม BOOT ที่อยู่บนบอร์ด (GPIO 0)

ข้อควรระวัง: GPIO 38 อาจไม่มีในบอร์ด ESP32 บางรุ่น (เช่น ESP32-WROOM-32 รุ่นมาตรฐาน) โปรดตรวจสอบ Pinout ของบอร์ดที่คุณใช้ หากไม่มี สามารถเปลี่ยนขา relay ในโค้ดเป็นขาอื่นที่ว่างได้

⚙️ การตั้งค่า (Configuration)
1. ตั้งค่า NETPIE 2020
ไปที่ NETPIE 2020 และสร้าง Device ใหม่

จด Device ID, Token และ Secret จากหน้า Device

นำค่าทั้ง 3 ไปใส่ในโค้ดส่วน NETPIE2020:

C++

const char* mqttClientID = "your Device ID";    // ใส่ Device ID ของคุณ
const char* mqttUsername = "your Token";      // ใส่ Token ของคุณ
const char* mqttPassword = "your Secret";     // ใส่ Secret ของคุณ
2. ตั้งค่า WiFi (ผ่าน WiFiManager)
อัปโหลดโค้ดนี้ไปยัง ESP32

เมื่อเปิดเครื่องครั้งแรก ESP32 จะสร้าง Access Point (AP) ชื่อ "ESP32-PZEM-Setup"

ใช้โทรศัพท์มือถือหรือคอมพิวเตอร์ เชื่อมต่อ WiFi นี้

หน้า Captive Portal (หน้าตั้งค่า) จะเด้งขึ้นมาอัตโนมัติ (หากไม่ขึ้น ให้เปิด Browser แล้วไปที่ 192.168.4.1)

เลือก "Configure WiFi" จากนั้นเลือกชื่อ WiFi บ้านของคุณ และใส่รหัสผ่าน

ESP32 จะบันทึกข้อมูลและเชื่อมต่อ WiFi ของคุณโดยอัตโนมัติ

3. การรีเซ็ตการตั้งค่า WiFi
หากต้องการเปลี่ยน WiFi หรือลบค่าที่บันทึกไว้:

ถอดปลั๊ก ESP32

กดปุ่ม BOOT (GPIO 0) ค้างไว้

เสียบปลั๊ก ESP32 (โดยที่ยังกดปุ่ม BOOT ค้างไว้)

รอ 3 วินาที (ตามที่โค้ดตั้งไว้) แล้วจึงปล่อยมือ

Serial Monitor จะแสดงข้อความ "Resetting WiFi settings..."

อุปกรณ์จะลบค่า WiFi และหยุดทำงาน ให้ถอดปลั๊กแล้วเสียบใหม่เพื่อเริ่มการตั้งค่า WiFiManager อีกครั้ง

🚀 การทำงาน (How it Works)
MQTT Topics
โปรเจกต์นี้ใช้ MQTT Topics ของ NETPIE 2020 ดังนี้:

การ Publish (ส่งข้อมูลจาก ESP32 ไปยัง NETPIE)
@shadow/data/update

ESP32 ส่ง: ข้อมูลเซ็นเซอร์ (volt, amp, power, e) ทุกๆ 5 วินาที

ESP32 ส่ง: อัปเดตสถานะ relay, overload และ StateOverload เมื่อมีการเปลี่ยนแปลง

ตัวอย่าง Payload (Sensor):

JSON

{"data":{"volt":220.50,"amp":1.234,"power":270.00,"e":10.500}}
ตัวอย่าง Payload (Overload):

JSON

{"data" : {"relay" : 0, "StateOverload" : 1}}
@shadow/data/get

ESP32 ส่ง: ส่ง Payload {} ว่างๆ ไปยัง Topic นี้ 1 ครั้งเมื่อเชื่อมต่อสำเร็จ เพื่อขอข้อมูลล่าสุดที่เก็บไว้ใน Shadow

การ Subscribe (รับข้อมูลจาก NETPIE มายัง ESP32)
@private/shadow/data/get/response

ESP32 รับ: รับข้อมูล Shadow (สถานะ relay และ overload ล่าสุด) หลังจากส่ง @shadow/data/get เพื่อให้สถานะของอุปกรณ์ตรงกันกับบนเซิร์ฟเวอร์

@msg/relay

ESP32 รับ: รับคำสั่งควบคุมรีเลย์

Payload "1": เปิดรีเลย์

Payload "0": ปิดรีเลย์

@msg/overload

ESP32 รับ: รับค่าสำหรับตั้งค่ากระแสไฟสูงสุด (Overload)

Payload (ตัวอย่าง "5"): ตั้งค่า Overload ที่ 5 แอมป์

โครงสร้างข้อมูลใน Shadow (Shadow Data Structure)
ข้อมูลที่ถูกเก็บไว้ใน NETPIE Device Shadow จะมีโครงสร้างดังนี้:

JSON

{
  "data": {
    "volt": 220.50,
    "amp": 1.234,
    "power": 270.00,
    "e": 10.500,
    "relay": 1,         // สถานะรีเลย์ (1=ON, 0=OFF)
    "overload": 5,        // ค่ากระแสสูงสุดที่ตั้งไว้ (เช่น 5 Amp)
    "StateOverload": 0  // สถานะ Overload (1=เกิด Overload, 0=ปกติ)
  }
}
