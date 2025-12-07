<p align="center">
  <img src="Header.jpeg" alt="Header.jpeg" width="80%">
</p>

<h1 align="center">🚧 Implementasi Sistem Palang Parkir Pintar Berbasis ROS dengan Kendali Arduino dan Motor Servo</h1>
<p align="center">
  <i>Project MK Robotika Medis</i>
</p>

---
## 👥 Anggota Kelompok
| Nama                       | NIM       |
|----------------------------|-----------|
| Aldrey Diriyah             | 122430054 |
| Affan Alfarabi             | 122430077 |
| Fadhlurrohman Arif         | 122430144 |

---
# 📘 Pendahuluan
Proyek ini merupakan implementasi sistem palang parkir pintar yang mengintegrasikan Robot Operating System (ROS) dengan Arduino Uno sebagai pengendali aktuator motor servo. Sistem ini dirancang untuk mendemonstrasikan bagaimana ROS dapat digunakan sebagai middleware untuk mengontrol perangkat fisik secara real-time melalui komunikasi serial.

Tujuan utama proyek ini adalah membangun gate parkir otomatis yang dapat dikendalikan menggunakan node ROS, baik secara manual (melalui terminal atau GUI), maupun secara otomatis pada tahap pengembangan lanjutan dengan menambahkan sensor pendeteksi kendaraan.

---

# 🚀 Fitur Utama

- Integrasi ROS dengan Arduino Uno menggunakan **rosserial**.
- Kendali motor **servo** untuk membuka dan menutup palang parkir.
- Arsitektur **publisher–subscriber** pada ROS untuk mengirim perintah gerak.
- Monitoring status palang (open/close) melalui **ROS topic**.
- Mendukung pengembangan otomatis berbasis sensor seperti **RFID, ultrasonic, kamera**, dan lainnya.

---
# 🚧 ROS2 Parking Barrier Project — Arduino + Servo + Push Button
## 1️⃣--⚙️Hardware Requirements
| No | Komponen                     | Jumlah      |
|----|------------------------------|-------------|
| 1  | Arduino Uno / Mega / Nano    | 1           |
| 2  | Servo SG90 / MG996R          | 1           |
| 3  | Push Button (Normally Open)  | 1           |
| 4  | Kabel Jumper                 | Beberapa    |
| 5  | Resistor 10 kΩ (pull-down)   | 1           |
| 6  | Kabel USB                    | 1           |

---
## 2️⃣-- Wiring Diagram
### 🔘 Push Button
Push Button:
- Kaki 1 → Pin D2 (Arduino)
- Kaki 2 → GND

### ⚙️ Servo
Servo:
- Merah  → 5V
- Coklat → GND
- Kuning → Pin D9

---
## 3️⃣ -- Arduino Code
```cpp
#include <Servo.h>

Servo gateServo;
const int servoPin = 9;
const int buttonPin = 2;

int buttonState = 0;
bool gateOpen = false;

void setup() {
  Serial.begin(115200);
  gateServo.attach(servoPin);
  pinMode(buttonPin, INPUT_PULLUP);

  gateServo.write(0);  // posisi palang menutup
  delay(500);
}

void loop() {
  buttonState = digitalRead(buttonPin);

  if (buttonState == LOW) {
    gateOpen = !gateOpen;
    if (gateOpen) {
      gateServo.write(90);   // buka palang
      Serial.println("GATE_OPEN");
    } else {
      gateServo.write(0);    // tutup palang
      Serial.println("GATE_CLOSED");
    }
    delay(500); // debounce
  }

  delay(10);
}

