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
<p align="center">
  <img src="Rangkaian Servo.jpeg" alt="Rangkaian Servo" width="50%">
</p>
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
```
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
```
---
## 4️⃣ -- Install ROS2 + rosserial
### 1. Install rosserial
```
sudo apt install ros-$ROS_DISTRO-rosserial-arduino
sudo apt install ros-$ROS_DISTRO-rosserial-python
```
### 2. Generate Arduino Library
```
cd ~/Arduino/libraries
rosrun rosserial_arduino make_libraries.py .
```
---
## 5️⃣ -- ROS2 Node — smart_barrier Package
Buat workspace:
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```
Buat package:
```
ros2 pkg create smart_barrier --build-type ament_cmake --dependencies rclcpp std_msgs
```
---
## 6️⃣ -- Isi File C++ Node
```
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <string>

class BarrierNode : public rclcpp::Node {
public:
    BarrierNode() : Node("barrier_node") {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "gate_status", 10,
            std::bind(&BarrierNode::topic_callback, this, std::placeholders::_1)
        );
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Status Palang: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BarrierNode>());
    rclcpp::shutdown();
    return 0;
}
```
---
## 7️⃣ -- Edit CMakeLists.txt
```
cmake_minimum_required(VERSION 3.5)
project(smart_barrier)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(barrier_node src/barrier_node.cpp)
ament_target_dependencies(barrier_node rclcpp std_msgs)

install(TARGETS
  barrier_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```
---
## 8️⃣ -- Build Workspace
```
cd ~/ros2_ws
colcon build
source install/setup.bash
```
---
## 9️⃣ -- Menjalankan rosserial
Sambungkan Arduino → USB
Lalu:
```
ros2 run rosserial_python serial_node.py /dev/ttyACM0 _baud:=115200
```
Arduino akan mengirim:
```
GATE_OPEN
GATE_CLOSED
```
---
## 🔟 -- Forward Serial ke ROS Topic
Di terminal lain:
```
ros2 topic echo /gate_status
```
Output:
```
data: GATE_OPEN
data: GATE_CLOSED
```
---
## PUBLISHER — Arduino
```
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class BarrierNode(Node):
    def __init__(self):
        super().__init__('barrier_node')
        
        # Create subscription to gate_status topic
        self.subscription = self.create_subscription(
            String,
            'gate_status',
            self.topic_callback,
            10
        )
        
        self.subscription  # Prevent unused variable warning
        self.get_logger().info('Barrier Node initialized and listening to /gate_status')
    
    def topic_callback(self, msg):
        """Callback function to handle incoming messages"""
        self.get_logger().info(f'Status Palang: "{msg.data}"')
        
        # Handle different states
        if msg.data == 'GATE_OPEN':
            self.handle_gate_open()
        elif msg.data == 'GATE_CLOSED':
            self.handle_gate_closed()
    
    def handle_gate_open(self):
        """Actions when gate is opened"""
        self.get_logger().warn('⚠️ GATE OPENED - Vehicle can pass')
        # Add your logic here (beep, light, etc)
    
    def handle_gate_closed(self):
        """Actions when gate is closed"""
        self.get_logger().info('✓ GATE CLOSED - Barrier is locked')
        # Add your logic here

def main(args=None):
    rclpy.init(args=args)
    barrier_node = BarrierNode()
    
    try:
        rclpy.spin(barrier_node)
    except KeyboardInterrupt:
        pass
    finally:
        barrier_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':

    main()
```
## SUBSCRIBER — ROS2 Node
```
import rospy
import serial
from std_msgs.msg import String
import time

class ServoPublisher:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('servo_publisher', anonymous=True)
        
        # Create publisher for gate status
        self.pub = rospy.Publisher('/gate/status', String, queue_size=10)
        
        # Serial port configuration
        self.serial_port = '/dev/ttyUSB0'  # Ubah sesuai port Arduino (Windows: COM3, COM4, etc)
        self.baud_rate = 115200
        self.ser = None
        
        # Connect to Arduino
        self.connect_serial()
        
    def connect_serial(self):
        """Connect to Arduino via serial port"""
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            rospy.loginfo(f"Connected to {self.serial_port} at {self.baud_rate} baud")
            time.sleep(2)  # Wait for Arduino to reset
        except serial.SerialException as e:
            rospy.logerr(f"Failed to connect to serial port: {e}")
            rospy.signal_shutdown("Serial connection failed")
    
    def run(self):
        """Main loop to read from serial and publish"""
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            try:
                if self.ser and self.ser.in_waiting > 0:
                    # Read data from Arduino
                    data = self.ser.readline().decode('utf-8').strip()
                    
                    # Check for valid messages
                    if data in ['GATE_OPEN', 'GATE_CLOSED']:
                        rospy.loginfo(f"Received: {data}")
                        self.pub.publish(data)
                    
            except UnicodeDecodeError:
                rospy.logwarn("Failed to decode serial data")
            except Exception as e:
                rospy.logerr(f"Error: {e}")
            
            rate.sleep()
    
    def shutdown(self):
        """Cleanup on shutdown"""
        if self.ser and self.ser.is_open:
            self.ser.close()
        rospy.loginfo("Serial connection closed")

if __name__ == '__main__':
    try:
        publisher = ServoPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        publisher.shutdown()
```
---
## 🔥 11. Launch File (opsional)
```
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='smart_barrier',
            executable='barrier_node',
            name='barrier_node'
        )
    ])
```
---
## 🧪 12. Test Tanpa Error
1. Upload Arduino code
2. colcon build
3. source install/setup.bash
4. Jalankan rosserial
5. Jalankan ROS node
6. Tekan push button → servo bergerak + ROS log muncul
---
## 🛠️ Troubleshooting Pasti Berhasil
| Masalah                  | Penyebab                        | Solusi                                              |
| ------------------------ | ------------------------------- | --------------------------------------------------- |
| `No executable found`    | Lupa build atau file nama salah | `colcon build` + cek nama node                      |
| Tidak ada output ROS     | Serial node belum jalan         | Jalankan `ros2 run rosserial_python serial_node.py` |
| Servo tidak bergerak     | Ground tidak terhubung          | Satukan GND Servo & Arduino                         |
| Push button tidak respon | Wiring salah atau tanpa pull-up | Pastikan pakai `INPUT_PULLUP`                       |
