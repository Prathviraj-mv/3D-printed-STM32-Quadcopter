**🛠 STM32 Black Pill Drone Connections (MPU6050, BMP180, ESCs)**

### **1️⃣ MPU6050 (IMU) → STM32 (I2C)**
| MPU6050 Pin | STM32 Pin |
|------------|----------|
| VCC        | 3.3V     |
| GND        | GND      |
| SDA        | PB7      |
| SCL        | PB6      |

---

### **2️⃣ BMP180 (Barometer) → STM32 (I2C)**
| BMP180 Pin | STM32 Pin |
|------------|----------|
| VCC        | 3.3V     |
| GND        | GND      |
| SDA        | PB7      |
| SCL        | PB6      |

> 📝 **Note:** Both MPU6050 and BMP180 share the **same I2C bus (PB7, PB6)**.

---

### **3️⃣ ESCs (Motor Control) → STM32 (PWM)**
| ESC (Motor) | STM32 Pin  | Timer Channel |
|------------|-----------|---------------|
| **ESC1 (Front-Left)**  | PA8  | TIM1_CH1 |
| **ESC2 (Front-Right)** | PA9  | TIM1_CH2 |
| **ESC3 (Back-Left)**   | PA10 | TIM1_CH3 |
| **ESC4 (Back-Right)**  | PA11 | TIM1_CH4 |

> ⚡ **ESCs need external 12V-16V battery power** (not from STM32!). 
> 🔹 Signal wires from ESCs connect to **STM32 PWM pins**.

