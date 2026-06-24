# ✈️ RC Plane Arduino Controller

Proyek pesawat kontrol jarak jauh (RC Plane) berbasis Arduino Nano dengan stabilisasi otomatis menggunakan sensor IMU MPU6050 dan kontrol via Bluetooth.

---

## 🌐 Panduan Lengkap

👉 **https://mirzakerenn.github.io/rc-plane**

---

## 📦 Komponen yang Dibutuhkan

| Komponen | Spesifikasi |
|---|---|
| Microcontroller | Arduino Nano v3 (CH340) |
| Sensor IMU | MPU6050 |
| Motor | Brushless A2212 |
| ESC | 30A |
| Servo | SG90 x2 (Aileron & Elevator) |
| Bluetooth | HC-05 / HC-06 |

---

## 📥 Download Kode Arduino

1. Klik file **`rc_plane_controller.ino`** di atas
2. Klik tombol **Raw**
3. Tekan **Ctrl+S** untuk simpan

---

## 📚 Library yang Dibutuhkan

Install via Arduino IDE → Tools → Manage Libraries:
- `MPU6050_tockn` by tockn
- `Wire` (built-in)
- `Servo` (built-in)

---

## ⚙️ Konfigurasi Pin

| Komponen | Pin Arduino |
|---|---|
| MPU6050 SDA | A4 |
| MPU6050 SCL | A5 |
| ESC / Motor | D9 |
| Servo Aileron | D3 |
| Servo Elevator | D5 |
| Bluetooth TX/RX | D0/D1 |

---

## 📱 Kontrol Bluetooth

Gunakan aplikasi **Arduino Bluetooth Controller** (Android) mode Terminal.

| Tombol | Fungsi |
|---|---|
| W | Throttle naik |
| S | Throttle turun |
| A | Roll kiri |
| D | Roll kanan |
| U | Pitch naik |
| J | Pitch turun |
| X | ARM / DISARM motor |
| R | Reset PID |
