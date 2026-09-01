#include <AP_HAL/AP_HAL.h>
#include "AP_L1_Control.h"

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_L1_Control::var_info[] = {
    // @Param: PERIOD
    // @DisplayName: L1 control period
    // @Description: Period in seconds of L1 tracking loop. This parameter is the primary control for aggressiveness of turns in auto mode. This needs to be larger for less responsive airframes. The default is quite conservative, but for most RC aircraft will lead to reasonable flight. For smaller more agile aircraft a value closer to 15 is appropriate, or even as low as 10 for some very agile aircraft. When tuning, change this value in small increments, as a value that is much too small (say 5 or 10 below the right value) can lead to very radical turns, and a risk of stalling.
    // @Units: s
    // @Range: 1 60
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("PERIOD",    0, AP_L1_Control, _L1_period, 17),

    // @Param: DAMPING
    // @DisplayName: L1 control damping ratio
    // @Description: Damping ratio for L1 control. Increase this in increments of 0.05 if you are getting overshoot in path tracking. You should not need a value below 0.7 or above 0.85.
    // @Range: 0.6 1.0
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("DAMPING",   1, AP_L1_Control, _L1_damping, 0.75f),

    // @Param: XTRACK_I
    // @DisplayName: L1 control crosstrack integrator gain
    // @Description: Crosstrack error integrator gain. This gain is applied to the crosstrack error to ensure it converges to zero. Set to zero to disable. Smaller values converge slower, higher values will cause crosstrack error oscillation.
    // @Range: 0 0.1
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("XTRACK_I",   2, AP_L1_Control, _L1_xtrack_i_gain, 0.02),

    // @Param: LIM_BANK
    // @DisplayName: Loiter Radius Bank Angle Limit
    // @Description: The sealevel bank angle limit for a continuous loiter. (Used to calculate airframe loading limits at higher altitudes). Setting to 0, will instead just scale the loiter radius directly
    // @Units: deg
    // @Range: 0 89
    // @User: Advanced
    AP_GROUPINFO("LIM_BANK",   3, AP_L1_Control, _loiter_bank_limit, 0.0f),

    AP_GROUPEND
};

// Bank angle command based on angle between aircraft velocity vector and reference vector to path.
// S. Park, J. Deyst, and J. P. How, "A New Nonlinear Guidance Logic for Trajectory Tracking,"
// Proceedings of the AIAA Guidance, Navigation and Control
// Conference, Aug 2004. AIAA-2004-4900.
// Modified to use PD control for circle tracking to enable loiter radius less than L1 length
// Modified to enable period and damping of guidance loop to be set explicitly
// Modified to provide explicit control over capture angle


/*
  Wrap AHRS yaw if in reverse - radians
 */
float AP_L1_Control::get_yaw() const
{
    if (_reverse) {
        return wrap_PI(M_PI + _ahrs.get_yaw_rad());
    }
    return _ahrs.get_yaw_rad();
}

/*
  Wrap AHRS yaw sensor if in reverse - centi-degress
 */
int32_t AP_L1_Control::get_yaw_sensor() const
{
    if (_reverse) {
        return wrap_180_cd(18000 + _ahrs.yaw_sensor);
    }
    return _ahrs.yaw_sensor;
}

/*
  return the bank angle needed to achieve tracking from the last
  update_*() operation
 */
int32_t AP_L1_Control::nav_roll_cd(void) const
{
    float ret;
	/*
		formula can be obtained through equations of balanced spiral:
		liftForce * cos(roll) = gravityForce * cos(pitch);
		liftForce * sin(roll) = gravityForce * lateralAcceleration / gravityAcceleration; // as mass = gravityForce/gravityAcceleration
		see issue 24319 [https://github.com/ArduPilot/ardupilot/issues/24319]
		Multiplier 100.0f is for converting degrees to centidegrees
		Made changes to avoid zero division as proposed by Andrew Tridgell: https://github.com/ArduPilot/ardupilot/pull/24331#discussion_r1267798397		 
	*/
	float pitchLimL1 = radians(60); // Suggestion: constraint may be modified to pitch limits if their absolute values are less than 90 degree and more than 60 degrees.
	float pitchL1 = constrain_float(_ahrs.get_pitch_rad(),-pitchLimL1,pitchLimL1);
    ret = degrees(atanf(_latAccDem * (1.0f/(GRAVITY_MSS * cosf(pitchL1))))) * 100.0f;
    ret = constrain_float(ret, -9000, 9000);
    return ret;
}

/*
  return the lateral acceleration needed to achieve tracking from the last
  update_*() operation
 */
float AP_L1_Control::lateral_acceleration(void) const
{
    return _latAccDem;
}

int32_t AP_L1_Control::nav_bearing_cd(void) const
{
    return wrap_180_cd(rad_to_cd(_nav_bearing));
}

int32_t AP_L1_Control::bearing_error_cd(void) const
{
    return rad_to_cd(_bearing_error);
}

int32_t AP_L1_Control::target_bearing_cd(void) const
{
    return wrap_180_cd(_target_bearing_cd);
}

/*
  this is the turn distance assuming a 90 degree turn
 */
float AP_L1_Control::turn_distance(float wp_radius) const
{
    wp_radius *= sq(_ahrs.get_EAS2TAS());
    return MIN(wp_radius, _L1_dist);
}

/*
  this approximates the turn distance for a given turn angle. If the
  turn_angle is > 90 then a 90 degree turn distance is used, otherwise
  the turn distance is reduced linearly.
  This function allows straight ahead mission legs to avoid thinking
  they have reached the waypoint early, which makes things like camera
  trigger and ball drop at exact positions under mission control much easier
 */
float AP_L1_Control::turn_distance(float wp_radius, float turn_angle) const
{
    float distance_90 = turn_distance(wp_radius);
    turn_angle = fabsf(turn_angle);
    if (turn_angle >= 90) {
        return distance_90;
    }
    return distance_90 * turn_angle / 90.0f;
}

float AP_L1_Control::loiter_radius(const float radius) const
{
    // prevent an insane loiter bank limit
    float sanitized_bank_limit = constrain_float(_loiter_bank_limit, 0.0f, 89.0f);
    float lateral_accel_sea_level = tanf(radians(sanitized_bank_limit)) * GRAVITY_MSS;

    float nominal_velocity_sea_level = 0.0f;
    if(_tecs != nullptr) {
        nominal_velocity_sea_level =  _tecs->get_target_airspeed();
    }

    float eas2tas_sq = sq(_ahrs.get_EAS2TAS());

    if (is_zero(sanitized_bank_limit) || is_zero(nominal_velocity_sea_level) ||
        is_zero(lateral_accel_sea_level)) {
        // Missing a sane input for calculating the limit, or the user has
        // requested a straight scaling with altitude. This will always vary
        // with the current altitude, but will at least protect the airframe
        return radius * eas2tas_sq;
    } else {
        float sea_level_radius = sq(nominal_velocity_sea_level) / lateral_accel_sea_level;
        if (sea_level_radius > radius) {
            // If we've told the plane that its sea level radius is unachievable fallback to
            // straight altitude scaling
            return radius * eas2tas_sq;
        } else {
            // select the requested radius, or the required altitude scale, whichever is safer
            return MAX(sea_level_radius * eas2tas_sq, radius);
        }
    }
}

bool AP_L1_Control::reached_loiter_target(void)
{
    return _WPcircle;
}

/**
   prevent indecision in our turning by using our previous turn
   decision if we are in a narrow angle band pointing away from the
   target and the turn angle has changed sign
 */
void AP_L1_Control::_prevent_indecision(float &Nu)
{
    const float Nu_limit = 0.9f*M_PI;
    if (fabsf(Nu) > Nu_limit &&
        fabsf(_last_Nu) > Nu_limit &&
        labs(wrap_180_cd(_target_bearing_cd - get_yaw_sensor())) > 12000 &&
        Nu * _last_Nu < 0.0f) {
        // we are moving away from the target waypoint and pointing
        // away from the waypoint (not flying backwards). The sign
        // of Nu has also changed, which means we are
        // oscillating in our decision about which way to go
        Nu = _last_Nu;
    }
}

// update L1 control for waypoint navigation
void AP_L1_Control::update_waypoint(const Location &prev_WP, const Location &next_WP, float dist_min)
{
    Location _current_loc;
    float Nu;

    uint32_t now = AP_HAL::micros();
    float dt = (now - _last_update_waypoint_us) * 1.0e-6f;
    if (dt > 1) {
        // kontrolcu uzun sure cagrilmadiysa bastan baslatiyorum
        _L1_xtrack_i = 0.0f;
    }
    if (dt > 0.1) {
        dt = 0.1;
    }
    _last_update_waypoint_us = now;

    // ucak yalpalamasin diye L1 kazancini (K_L1) ayarliyorum
    float K_L1 = 4.0f * _L1_damping * _L1_damping;

    // ucagin anlik konumunu aliyorum, alamazsam cikiyorum
    if (_ahrs.get_location(_current_loc) == false) {
        _data_is_stale = true;
        return;
    }

    Vector2f _groundspeed_vector = _ahrs.groundspeed_vector();
    _target_bearing_cd = _current_loc.get_bearing_to(next_WP);
    float groundSpeed = _groundspeed_vector.length();
    
    const bool moving_forwards = fabsf(wrap_PI(_groundspeed_vector.angle() - get_yaw())) < M_PI_2;

    if (groundSpeed < 0.1f || !moving_forwards) {
        groundSpeed = 0.1f;
        _groundspeed_vector = Vector2f(cosf(get_yaw()), sinf(get_yaw())) * groundSpeed;
    }

    // ardupilotun kendi ileri bakis (L1) mesafesini hesapliyorum
    _L1_dist = MAX(0.3183099f * _L1_damping * _L1_period * groundSpeed, dist_min);

    // --- benim saf takip (pure pursuit) mantigim basliyor ---
   
    // 1. ucaktan hedefe dogrudan bir vektor ciziyorum
    Vector2f ucak_hedef_vektoru = _current_loc.get_distance_NE(next_WP);
    
    // 2. hedefle aramdaki gercek mesafeyi olcuyorum
    float gercek_mesafe = ucak_hedef_vektoru.length();

    // 3. dinamik L1 kontrolu: eger hedef L1 mesafesinden daha yakindaysa
    // ardupilotun L1 balonunu ezip direkt gercek mesafeye esitliyorum (min 5 metre)
    if (gercek_mesafe < _L1_dist) {
        _L1_dist = MAX(gercek_mesafe, 5.0f); 
    }

    // 4. hedefin ucağa gore acisini buluyorum
    float target_bearing = atan2f(ucak_hedef_vektoru.y, ucak_hedef_vektoru.x);

    // 5. ucagin burnunun baktigi aciyi buluyorum
    float ucak_bearing = atan2f(_groundspeed_vector.y, _groundspeed_vector.x);

    // 6. aradaki aci farkini hesaplayip -pi ile +pi arasina sikistiriyorum
    Nu = wrap_PI(target_bearing - ucak_bearing);

    // loglarda duzgun gorunsun diye navigasyon acisini hedefe kilitliyorum
    _nav_bearing = target_bearing; 

    // biz cizgi degil hareketli hedef takip ettigimiz icin capraz hatayi (crosstrack error) sifirliyorum
    _crosstrack_error = 0.0f;

    // --- saf takip mantigi bitti ---

    _prevent_indecision(Nu);
    _last_Nu = Nu;

    // ucak sacmalamasin diye hata acisini +- 90 derece ile sinirliyorum
    Nu = constrain_float(Nu, -1.5708f, +1.5708f);
    
    // yanal ivme (yatis) emrini hesaplayip ucaga gonderiyorum
    _latAccDem = K_L1 * groundSpeed * groundSpeed / _L1_dist * sinf(Nu);
    // 2. L1 algoritmasının ürettiği yanal ivme (m/s^2)
    float lat_accel = _latAccDem; 
    // 3. Sıfıra bölünme hatasını önlemek için minimum 1 m/s hız sınırı koy
    float ground_speed = MAX(_ahrs.groundspeed(), 1.0f); 
    // 4. Senin denklemin: Yaw Rate (Radyan/saniye cinsinden)
    float yaw_rate_rads = lat_accel / ground_speed;
    // 5. ArduPlane kontrolcüsü için radyanı derece/saniyeye (veya centidegree) çevir
    float yaw_rate_degs = degrees(yaw_rate_rads);
    _nav_yaw_rate_cd = (int32_t)(yaw_rate_degs * 100.0f); // centidegree cinsinden

    _WPcircle = false;
    _last_loiter.reached_loiter_target_ms = 0;
    _bearing_error = Nu; 
    _data_is_stale = false; 
}

// update L1 control for loitering
void AP_L1_Control::update_loiter(const Location &center_WP, float radius, int8_t loiter_direction)
{
    Location _current_loc;

    // konumu bulamazsak cikis yapiyorum
    if (_ahrs.get_location(_current_loc) == false) {
        _data_is_stale = true;
        return;
    }

    Vector2f _groundspeed_vector = _ahrs.groundspeed_vector();
    float groundSpeed = MAX(_groundspeed_vector.length(), 1.0f);
    
    // kazanc ve L1 mesafesini tekrar hesapliyorum
    float K_L1 = 4.0f * _L1_damping * _L1_damping;
    _L1_dist = MAX(0.3183099f * _L1_damping * _L1_period * groundSpeed, 1.0f);

    // ardupilotun cember cizme mantigini tamamen sildim, yerine direkt ustune ucmasini (saf takip) yazdim
    
    // 1. ucaktan hedefe vektorumu ciziyorum
    Vector2f ucak_hedef_vektoru = _current_loc.get_distance_NE(center_WP);

    // 2. acilari hesapliyorum
    float target_bearing = atan2f(ucak_hedef_vektoru.y, ucak_hedef_vektoru.x);
    float ucak_bearing = atan2f(_groundspeed_vector.y, _groundspeed_vector.x);

    // 3. aradaki aci farki
    float Nu = wrap_PI(target_bearing - ucak_bearing);
    
    _nav_bearing = target_bearing;
    _crosstrack_error = 0.0f;

    Nu = constrain_float(Nu, -1.5708f, +1.5708f);
    
    // 4. dogrudan hedefe donmesi icin yatis ivmesini basiyorum
    _latAccDem = K_L1 * groundSpeed * groundSpeed / _L1_dist * sinf(Nu);
    // 2. L1 algoritmasının ürettiği yanal ivme (m/s^2)
    float lat_accel = _latAccDem; 
    // 3. Sıfıra bölünme hatasını önlemek için minimum 1 m/s hız sınırı koy
    float ground_speed = MAX(_ahrs.groundspeed(), 1.0f); 
    // 4. Senin denklemin: Yaw Rate (Radyan/saniye cinsinden)
    float yaw_rate_rads = lat_accel / ground_speed;
    // 5. ArduPlane kontrolcüsü için radyanı derece/saniyeye (veya centidegree) çevir
    float yaw_rate_degs = degrees(yaw_rate_rads);
    _nav_yaw_rate_cd = (int32_t)(yaw_rate_degs * 100.0f); // centidegree cinsinden

    // 5. ardupilotu daire cizmedigime inandirmak icin wbpcircle bayragini false yapiyorum :)
    _WPcircle = false; 
    _bearing_error = Nu; 
    _data_is_stale = false; 
}


// update L1 control for heading hold navigation
void AP_L1_Control::update_heading_hold(int32_t navigation_heading_cd)
{
    // Calculate normalised frequency for tracking loop
    const float omegaA = 4.4428f/_L1_period; // sqrt(2)*pi/period
    // Calculate additional damping gain

    int32_t Nu_cd;
    float Nu;

    // copy to _target_bearing_cd and _nav_bearing
    _target_bearing_cd = wrap_180_cd(navigation_heading_cd);
    _nav_bearing = cd_to_rad(navigation_heading_cd);

    Nu_cd = _target_bearing_cd - wrap_180_cd(_ahrs.yaw_sensor);
    Nu_cd = wrap_180_cd(Nu_cd);
    Nu = cd_to_rad(Nu_cd);

    Vector2f _groundspeed_vector = _ahrs.groundspeed_vector();

    // Calculate groundspeed
    float groundSpeed = _groundspeed_vector.length();

    // Calculate time varying control parameters
    _L1_dist = groundSpeed / omegaA; // L1 distance is adjusted to maintain a constant tracking loop frequency
    float VomegaA = groundSpeed * omegaA;

    // Waypoint capture status is always false during heading hold
    _WPcircle = false;
    _last_loiter.reached_loiter_target_ms = 0;

    _crosstrack_error = 0;

    _bearing_error = Nu; // bearing error angle (radians), +ve to left of track

    // Limit Nu to +-pi
    Nu = constrain_float(Nu, -M_PI_2, M_PI_2);
    _latAccDem = 2.0f*sinf(Nu)*VomegaA;

    _data_is_stale = false; // status are correctly updated with current waypoint data
}

// update L1 control for level flight on current heading
void AP_L1_Control::update_level_flight(void)
{
    // copy to _target_bearing_cd and _nav_bearing
    _target_bearing_cd = _ahrs.yaw_sensor;
    _nav_bearing = _ahrs.get_yaw_rad();
    _bearing_error = 0;
    _crosstrack_error = 0;

    // Waypoint capture status is always false during heading hold
    _WPcircle = false;
    _last_loiter.reached_loiter_target_ms = 0;

    _latAccDem = 0;

    _data_is_stale = false; // status are correctly updated with current waypoint data
}
