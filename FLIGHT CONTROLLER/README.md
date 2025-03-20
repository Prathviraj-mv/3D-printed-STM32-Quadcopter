# STM32 BLACKPILL
![image](https://github.com/user-attachments/assets/b69bff74-bfac-4742-854b-530741030c5f)





### 2. **MPU6050 Connections**
| MPU6050 Pin | Connect To STM32 |
|-------------|------------------|
| VCC         | 3.3V (not 5V)    |
| GND         | GND              |
| SCL         | PB8 (I2C1_SCL)   |
| SDA         | PB9 (I2C1_SDA)   |

---

### 3. **BMP180 Connections**
| BMP180 Pin  | Connect To STM32 |
|--------------|------------------|
| VCC          | 3.3V             |
| GND          | GND              |
| SCL          | PB8 (shared with MPU6050 SCL) |
| SDA          | PB9 (shared with MPU6050 SDA) |

---
