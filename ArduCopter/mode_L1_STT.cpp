#include "Copter.h"

#if MODE_L1_STT_ENABLED
#include <math.h>

bool ModeSTT::init(bool ignore_checks) {
    _has_target = false;
    return true;
}

void ModeSTT::run() {
    Vector3f current_pos = pos_control->get_pos_estimate_NED_m().tofloat(); //droneun anlik konumunu cekiyoruz (metre)
    Vector2f velocity_xy = ahrs.groundspeed_vector(); // droneun anlik ground speed vektorunu cekiyoruz

    // Hedef yoksa asılı kal (Hover)
    if (!_has_target) { //target yoksa
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(0.0f, 0.0f, 0.0f); // roll, pitch, yaw_rate komutlarını sıfırla
        attitude_control->set_throttle_out(motors->get_throttle_hover(), true, 0.0f);// hover için gerekli throttle değerini ayarla
        return;
    }

    // 1. KİNEMATİK VERİLER
    float groundSpeed = velocity_xy.length(); // vector ground speed -> skaler ground speed 
    if (groundSpeed < 0.1f) groundSpeed = 0.1f; // zero division olmamasi icin minimum 0.1 m/s olarak ayarliyoruz

    Vector2f ucak_hedef_vektoru(_target_pos.x - current_pos.x, _target_pos.y - current_pos.y); // target ve drone arasi vector
    float gercek_mesafe = ucak_hedef_vektoru.length(); // target ile drone arasi mesafe (m)

    // 2. AÇI HESABI (Nu)
    float target_bearing = atan2f(ucak_hedef_vektoru.y, ucak_hedef_vektoru.x); // hedefin acisi (radyan)
    float ucak_bearing = ahrs.get_yaw(); // droneun anlik yaw acisi (radyan)
    float Nu = wrap_PI(target_bearing - ucak_bearing);// hedef ile drone arasi aci farki (radyan)
    Nu = constrain_float(Nu, -1.5708f, +1.5708f); // aci farkini -90 ile +90 derece arasinda sinirla

    // -----------------------------------------------------------------
    // 3. KLASİK L1 ALGORİTMASI (Sürekli Yay Çizimi)
    // -----------------------------------------------------------------
    float _L1_damping = 0.75f;// L1 damping katsayisi
    float _L1_period = 4.0f; // Saniye cinsinden referans donus periyodu
    
    // L1 Mesafesi Hesabı
    float L1_dist = 0.3183099f * _L1_damping * _L1_period * groundSpeed;
    
    // Hedefe yaklaştıkça dönüş balonunu daralt
    L1_dist = MAX(L1_dist, 5.0f); // L1 mesafesi aradaki mesafeden buyukse L1_dist aradaki mesafeye eşitlenir (minimum 5 metre)
    if (gercek_mesafe < L1_dist) {
        L1_dist = MAX(gercek_mesafe, 5.0f);
    }

    // Yanal ivme ve yaw komutu hesabı
    float K_L1 = 4.0f * _L1_damping * _L1_damping;
    float latAccDem = K_L1 * (groundSpeed * groundSpeed) / L1_dist * sinf(Nu);
    float yaw_rate_dem = latAccDem / groundSpeed;

    // -----------------------------------------------------------------
    // 4. İLERİ HAREKET KOMUTLARI
    // -----------------------------------------------------------------
    float pitch_cd = -3000.0f; // STT kuralı gereği ileri hareket için sabit pitch komutu (-30 derece)
    float roll_cd = 0.0f;      // STT kuralı gereği yatış sıfır
    float yaw_rate_cds = yaw_rate_dem * 57.2958f * 100.0f;

    // -----------------------------------------------------------------
    // 5. BAĞIMSIZ FİZİKSEL İRTİFA KOMPANZASYONU 
    // -----------------------------------------------------------------
    float current_alt = -current_pos.z; 
    float target_alt = -_target_pos.z; // hedefin  irtifası python tarafindan NED koordinat sistemine çevrilmiş olarak geliyor
    float alt_error = target_alt - current_alt;
    
    float Kp_z = 0.05f; // 1 metre hata için %5 gaz artışı (0.05) 
    float thrust_z_hedef = motors->get_throttle_hover() + (alt_error * Kp_z); // hedef irtifaya göre gaz komutu (0 ~ 1 arası)
    
    // Pitch açısından kaynaklı dikey itki kaybını kosinüs ile toparla
    float pitch_rad = fabsf(pitch_cd * 0.01f * DEG_TO_RAD); // pitch komutunu radyana çevir
    float thrust_basilacak = thrust_z_hedef / cosf(pitch_rad);  // pitch açısına göre basılacak gazı hesapla
    thrust_basilacak = constrain_float(thrust_basilacak, 0.1f, 0.9f); 

    // 6. SİSTEME GÖNDER
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(roll_cd, pitch_cd, yaw_rate_cds);
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    attitude_control->set_throttle_out(thrust_basilacak, true, 0.0f);
}
#endif