
#ifndef IMU_H
#define IMU_H

#include <Arduino.h> 
#include <Wire.h>    


typedef struct {
    float ax, ay, az; 
    float gx, gy, gz; 
    float temp;     
} ImuData_t;

// --- AYAR LİSTELERİ (ENUMS) ---
// İvmeölçer hassasiyet ayarı
typedef enum {
    ACCEL_2G = 0,  
    ACCEL_4G,     
    ACCEL_8G,      
    ACCEL_16G     
} AccelRange_t;

// Jiroskop hassasiyet ayarı
typedef enum {
    GYRO_250DPS = 0,  
    GYRO_500DPS,     
    GYRO_1000DPS,     
    GYRO_2000DPS      
} GyroRange_t;

class IMU_Driver {
public:
    // Kurulum fonksiyonu
    // "TwoWire" Arduino'nun Wire kütüphanesinin tipidir.
    bool init(TwoWire *wirePort, AccelRange_t aRange, GyroRange_t gRange);
    
    // Veri okuma fonksiyonu
    void readData(ImuData_t *data);

private:
    TwoWire *_wire;   // Hangi I2C portunu kullanacağız? (Wire, Wire1 vs.)
    float _aScale;    // Ham veriyi m/s2'ye çeviren çarpan
    float _gScale;    // Ham veriyi rad/s'ye çeviren çarpan
    
    // Yardımcı fonksiyonlar (Private - Dışarıdan çağrılmaz)
    void writeReg(uint8_t reg, uint8_t val);
    void readRegs(uint8_t reg, uint8_t *buffer, uint8_t len);
};

#endif
