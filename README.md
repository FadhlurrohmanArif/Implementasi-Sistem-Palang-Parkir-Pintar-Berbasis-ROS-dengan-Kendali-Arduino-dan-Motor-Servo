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
| Aldrey Diriyah             | 122430135 |
| Affan Alfarabi             | 122430125 |
| Fadhlurrohman Arif         | 122430137 |

---
##🛠️ **Pendahuluan**
Proyek ini merupakan implementasi sistem palang parkir pintar yang mengintegrasikan Robot Operating System (ROS) dengan Arduino Uno sebagai pengendali aktuator motor servo. Sistem ini dirancang untuk mendemonstrasikan bagaimana ROS dapat digunakan sebagai middleware untuk mengontrol perangkat fisik secara real-time melalui komunikasi serial.

Tujuan utama proyek ini adalah membangun gate parkir otomatis yang dapat dikendalikan menggunakan node ROS, baik secara manual (melalui terminal atau GUI), maupun secara otomatis pada tahap pengembangan lanjutan dengan menambahkan sensor pendeteksi kendaraan.

---

## 🚀 Fitur Utama

- Integrasi ROS dengan Arduino Uno menggunakan **rosserial**.
- Kendali motor **servo** untuk membuka dan menutup palang parkir.
- Arsitektur **publisher–subscriber** pada ROS untuk mengirim perintah gerak.
- Monitoring status palang (open/close) melalui **ROS topic**.
- Mendukung pengembangan otomatis berbasis sensor seperti **RFID, ultrasonic, kamera**, dan lainnya.

