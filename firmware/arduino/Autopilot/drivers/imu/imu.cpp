#include "imu.h"

// MPU9250 Ana I2C Adresi
#define MPU9250_ADDR 0x68 


#define DEG_TO_RAD 0.01745329251f//  Dereceyi Radyana çevirmek için (PI / 180)
#define GRAVITY_MSS 9.80665f

bool IMU_Driver::init(TwoWire *wirePort, AccelRange_t aRange, GyroRange_t gRange) {
    _wire = wirePort;
     // I2C başlatılmadıysa burada başlatabiliriz (Opsiyonel)
    // _wire->begin(); 

    // 1. Sensör Bağlantı Kontrolü (WHO_AM_I Register: 0x75)
    uint8_t whoami;
    readRegs(0x75, &whoami, 1);
    
    // MPU9250 için WHO_AM_I değeri 0x71 olmalıdır. 
    if(whoami != 0x71) {
        return false; 
    }

    // 2. Sensörü Uyandır (PWR_MGMT_1 Register: 0x6B)
    // Bitleri 0 yaparsak sensör "Sleep" modundan çıkar.
    writeReg(0x6B, 0x00); 
    
     // 2. Sensörü Uyandır (PWR_MGMT_1 Register: 0x6B)
    // Bitleri 0 yaparsak sensör "Sleep" modundan çıkar.
    writeReg(0x1C, aRange << 3); 
    
    // 4. Jiroskop Ayarı (0x1B)
    writeReg(0x1B, gRange << 3); 

    // 5. Ölçekleme Faktörlerini Hesapla (16-bit veriyi fiziksel birime çevirmek için)
    switch(aRange) {
        case ACCEL_2G:  _aScale = 2.0f / 32768.0f; break;
        case ACCEL_4G:  _aScale = 4.0f / 32768.0f; break;
        case ACCEL_8G:  _aScale = 8.0f / 32768.0f; break;
        case ACCEL_16G: _aScale = 16.0f / 32768.0f; break;
    }// g'yi m/s2'ye çevirmek için
    _aScale *= GRAVITY_MSS; 

    switch(gRange) {
        case GYRO_250DPS:  _gScale = 250.0f / 32768.0f; break;
        case GYRO_500DPS:  _gScale = 500.0f / 32768.0f; break;
        case GYRO_1000DPS: _gScale = 1000.0f / 32768.0f; break;
        case GYRO_2000DPS: _gScale = 2000.0f / 32768.0f; break;
    }
    // Dereceyi Radyana
    _gScale *= DEG_TO_RAD;

    // --- MPU9250 EKSTRA: Manyetometreye Erişim İzni ---
    // MPU9250 içindeki pusulaya ulaşmak için "Bypass Mode" açılmalıdır.
    writeReg(0x37, 0x02); 

    return true; 
}

void IMU_Driver::readData(ImuData_t *data) {
    uint8_t rawData[14];
    
    // 0x3B adresinden başlayarak 14 byte oku (İvme, Sıcaklık, Jiroskop)
    readRegs(0x3B, rawData, 14); 
    
    // Byte birleştirme (High Byte << 8 | Low Byte)
    int16_t r_ax = (int16_t)(rawData[0] << 8 | rawData[1]);
    int16_t r_ay = (int16_t)(rawData[2] << 8 | rawData[3]);
    int16_t r_az = (int16_t)(rawData[4] << 8 | rawData[5]);
    int16_t r_temp = (int16_t)(rawData[6] << 8 | rawData[7]);
    int16_t r_gx = (int16_t)(rawData[8] << 8 | rawData[9]);
    int16_t r_gy = (int16_t)(rawData[10] << 8 | rawData[11]);
    int16_t r_gz = (int16_t)(rawData[12] << 8 | rawData[13]);
    
     // Gerçek fiziksel birimlere dönüştürme
    data->ax = (float)r_ax * _aScale;
    data->ay = (float)r_ay * _aScale;
    data->az = (float)r_az * _aScale;
    
    data->gx = (float)r_gx * _gScale;
    data->gy = (float)r_gy * _gScale;
    data->gz = (float)r_gz * _gScale;

    // datasheetten sıcaklık Formülü: ((Raw - RoomTempOffset) / TempSensitivity) + 21
    data->temp = ((float)r_temp - 0.0f) / 333.87f + 21.0f;
}

// --- ARDUINO I2C YARDIMCI FONKSİYONLARI ---

void IMU_Driver::writeReg(uint8_t reg, uint8_t val) {
    _wire->beginTransmission(MPU9250_ADDR);
    _wire->write(reg);
    _wire->write(val);
    _wire->endTransmission();
}

void IMU_Driver::readRegs(uint8_t reg, uint8_t *buffer, uint8_t len) {
    _wire->beginTransmission(MPU9250_ADDR);
    _wire->write(reg);
    _wire->endTransmission(false);
    
    _wire->requestFrom((uint8_t)MPU9250_ADDR, len);
    for(uint8_t i=0; i<len; i++) {
        if(_wire->available()) buffer[i] = _wire->read();
    }
}
