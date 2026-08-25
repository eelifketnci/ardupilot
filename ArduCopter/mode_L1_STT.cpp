#include "Copter.h"

#if MODE_L1_STT_ENABLED

#include <math.h>

bool ModeSTT::init(bool ignore_checks) {
    _has_target = false;
    return true;
}

void ModeSTT::run() {
    // 1. Drone'un Anlık Konumunu Al (NED Metre)
    Vector3f current_pos = pos_control->get_pos_estimate_NED_m().tofloat();
    
    // Eğer henüz Python'dan bir hedef gelmediyse güvenli şekilde bekle
    if (!_has_target) {
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(0.0f, 0.0f, 0.0f);
        return;
    }

    // 2. Hedef ile Gövde Arasındaki Göreceli Vektör ve Kerteriz (LOS Angle)
    float dx = _target_pos.x - current_pos.x; // Kuzey farkı
    float dy = _target_pos.y - current_pos.y; // Doğu farkı
    float aktif_mesafe = sqrtf(dx * dx + dy * dy);
    
    // Görüş Hattı Açısı (Line-of-Sight Bearing)
    float kerteriz = atan2f(dy, dx);
    
    // 3. Açı Hatası Hesabı (Eta: Kerteriz - Mevcut Baş Açısı)
    float current_heading = ahrs.get_yaw();
    float eta = kerteriz - current_heading;
    
    // Açıyı -pi ile +pi arasına sarmala (Wrap to [-pi, pi])
    while (eta > M_PI) eta -= 2.0f * M_PI;
    while (eta < -M_PI) eta += 2.0f * M_PI;

    // 4. L1 Güdüm Yasası (Skid-to-Turn)
    float V = 15.0f;                             // İleri hız referansı (m/s)
    float L1_dist = MAX(aktif_mesafe, 10.0f);    // Minimum L1 mesafesi
    
    // Talep Edilen Yanal İvme (Lateral Acceleration)
    float lat_acc = 2.0f * (V * V) / L1_dist * sinf(eta);
    
    // Yanal ivmeyi doğrudan Yaw Dönüş Hızına çeviriyoruz (rad/s)
    float yaw_rate_dem = lat_acc / V; 
    
    // 5. STT Alt Seviye Sürüş Komutları
    float roll_cd = 0.0f;                                  // YATIŞ KESİNLİKLE SIFIR (STT Kuralı)
    float pitch_cd = -1200.0f;                             // İleri itki için burnu 12 derece aşağı bas
    float yaw_rate_cds = yaw_rate_dem * 57.2958f * 100.0f; // rad/s -> centi-deg/s

    // Kontrolcüye komutu bas
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(roll_cd, pitch_cd, yaw_rate_cds);
}

#endif