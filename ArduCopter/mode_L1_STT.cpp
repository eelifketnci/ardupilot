#include "Copter.h"

#if MODE_L1_STT_ENABLED
#include <math.h>

bool ModeSTT::init(bool ignore_checks) {
    _has_target = false;
    return true;
}

void ModeSTT::run() {
    Vector3f current_pos = pos_control->get_pos_estimate_NED_m().tofloat();
    Vector2f velocity_xy = ahrs.groundspeed_vector();

    // Hedef yoksa asılı kal (Hover)
    if (!_has_target) {
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(0.0f, 0.0f, 0.0f);
        attitude_control->set_throttle_out(motors->get_throttle_hover(), true, 0.0f);
        return;
    }

    // 1. KİNEMATİK VERİLER
    float groundSpeed = velocity_xy.length();
    if (groundSpeed < 0.1f) groundSpeed = 0.1f; 

    Vector2f ucak_hedef_vektoru(_target_pos.x - current_pos.x, _target_pos.y - current_pos.y);
    float gercek_mesafe = ucak_hedef_vektoru.length();

    // 2. AÇI HESABI (Nu)
    float target_bearing = atan2f(ucak_hedef_vektoru.y, ucak_hedef_vektoru.x);
    float ucak_bearing = ahrs.get_yaw();
    float Nu = wrap_PI(target_bearing - ucak_bearing);
    Nu = constrain_float(Nu, -1.5708f, +1.5708f);

    // -----------------------------------------------------------------
    // 3. KLASİK L1 ALGORİTMASI (Sürekli Yay Çizimi)
    // -----------------------------------------------------------------
    float _L1_damping = 0.75f;
    float _L1_period = 5.0f; // Saniye cinsinden referans dönüş periyodu
    
    // L1 Mesafesi Hesabı
    float L1_dist = 0.3183099f * _L1_damping * _L1_period * groundSpeed;
    
    // Hedefe yaklaştıkça dönüş balonunu daralt
    L1_dist = MAX(L1_dist, 5.0f);
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
    float pitch_cd = -1500.0f; // Sabit 15 derece ileri atılım
    float roll_cd = 0.0f;      // STT kuralı gereği yatış sıfır
    float yaw_rate_cds = yaw_rate_dem * 57.2958f * 100.0f;

    // -----------------------------------------------------------------
    // 5. BAĞIMSIZ FİZİKSEL İRTİFA KOMPANZASYONU (API Hatası Önleyici)
    // -----------------------------------------------------------------
    float current_alt = -current_pos.z; 
    float target_alt = -_target_pos.z;
    float alt_error = target_alt - current_alt;
    
    float Kp_z = 0.05f; 
    float thrust_z_hedef = motors->get_throttle_hover() + (alt_error * Kp_z);
    
    // Pitch açısından kaynaklı dikey itki kaybını kosinüs ile toparla
    float pitch_rad = fabsf(pitch_cd * 0.01f * DEG_TO_RAD);
    float thrust_basilacak = thrust_z_hedef / cosf(pitch_rad);
    thrust_basilacak = constrain_float(thrust_basilacak, 0.1f, 0.9f);

    // 6. SİSTEME GÖNDER
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(roll_cd, pitch_cd, yaw_rate_cds);
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    attitude_control->set_throttle_out(thrust_basilacak, true, 0.0f);
}
#endif