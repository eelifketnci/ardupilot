from pymavlink import mavutil
import time

# Mission Planner'a veri göndermek (push etmek) için udpout kullanılır.
# Not: Eğer Mission Planner farklı bir portta çalışıyorsa port numarasını güncelleyebilirsiniz.
master = mavutil.mavlink_connection('udpout:127.0.0.1:14550', source_system=2, source_component=1)

print("Hedef araç MAVLink üzerinden yayinlaniyor...")

while True:
    # 1. Hayati Kısım: Mission Planner bu sinyali almadan aracı/hedefi haritada göstermez!
    master.mav.heartbeat_send(
        type=mavutil.mavlink.MAV_TYPE_GENERIC,
        autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        base_mode=mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
        custom_mode=0,
        system_status=mavutil.mavlink.MAV_STATE_ACTIVE
    )

    # 2. Hedefin Anlık Konum Bilgileri (Örnek koordinatlar)
    lat = int(41.0082 * 1e7) 
    lon = int(28.9784 * 1e7)
    alt = 1000  # Milimetre cinsinden yükseklik (100 metre gibi)

    # 3. Konum Mesajını Gönderme
    master.mav.global_position_int_send(
        time_boot_ms=int(time.time() * 1000) % 4294967296,
        lat=lat,
        lon=lon,
        alt=alt,
        relative_alt=alt,
        vx=0, vy=0, vz=0,
        hdg=0
    )
    
    # 1 saniyede bir gönderim yapalım
    time.sleep(1)