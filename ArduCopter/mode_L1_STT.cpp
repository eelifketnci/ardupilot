#include "Copter.h"
#include <math.h>

bool ModeSTT::init(bool ignore_checks) {
    // Mod başladığında hedefleri ve başlangıç değişkenlerini sıfırla
    return true;
}

void ModeSTT::run() {
    // 1. Aracın Anlık Konumunu Al
    Vector3f pos = inertial_nav.get_position_neu_cm();
    
    // (L1 Açı ve İvme Hesaplamaları Buraya Gelecek)
    // float V = 1500.0f; // 15 m/s
    float yaw_rate_hedef_radyan = 0.5f; // Şimdilik örnek bir dönüş hızı
    
    // 2. Drone'un Alt Seviye Kontrolcüsünü (Attitude Controller) Ezme
    float roll_cd = 0.0f;                           // YATIŞ SIFIR! (Yana yatmak yasak)
    float pitch_cd = -1000.0f;                      // İleri hızlanmak için burnu 10 derece aşağı ez
    float yaw_rate_cds = yaw_rate_hedef_radyan * 100.0f; // Santiderece/saniye cinsine çevir
    
    // 3. Otopilota bu katı STT kurallarını bas
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(roll_cd, pitch_cd, yaw_rate_cds);
}