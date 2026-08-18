import time
import math
import subprocess
from pymavlink import mavutil

def hesapla_mesafe(lat1, lon1, lat2, lon2):
    R = 6371000 
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))

def latlon_to_meters(lat, lon, lat_ref, lon_ref):
    dlat = lat - lat_ref
    dlon = lon - lon_ref
    y = dlat * 111132.0 
    x = dlon * 111132.0 * math.cos(math.radians(lat_ref))
    return x, y

print("Avci Ucakla Baglanti Kuruluyor...")
master = mavutil.mavlink_connection('udpin:localhost:14550')
master.wait_heartbeat()
print("Baglanti Basarili! Konum aliniyor...")
start_time = time.time()

ref_lat = -35.3632620
ref_lon = 149.1652370

msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
target_lat = (msg.lat / 1e7) + 0.0020000 
target_lon = (msg.lon / 1e7) + 0.0020000 
target_alt = 100.0 

print("Aktif")

while True:
    target_lat += 0.0000100
    target_lon += 0.0000100
    current_time = time.time()
    
    # ARDUPLANE İÇİN KESİN ÇALIŞAN KOMUT: DO_REPOSITION
    master.mav.command_int_send(
        master.target_system, 
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        mavutil.mavlink.MAV_CMD_DO_REPOSITION,
        0, 0,
        -1.0, 
        mavutil.mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE, 
        0.0, 0.0, 
        int(target_lat * 1e7), 
        int(target_lon * 1e7), 
        target_alt
    )
    
    # Gazebo'daki X3 modelini hareket ettiren komut
    x_pos, y_pos = latlon_to_meters(target_lat, target_lon, ref_lat, ref_lon)
    pose_cmd = f"gz service -s /world/runway/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --req 'name: \"x3\", position: {{x: {x_pos:.2f}, y: {y_pos:.2f}, z: 100.0}}'"
    subprocess.Popen(pose_cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if msg:
        ucak_lat = msg.lat / 1e7
        ucak_lon = msg.lon / 1e7
        mesafe = hesapla_mesafe(ucak_lat, ucak_lon, target_lat, target_lon)
        elapsed_time = current_time - start_time 
        
        if mesafe < 5 :
            print(f"Avci ile Arasindaki Mesafe: {mesafe:.1f} Metre | Takip süresi: {elapsed_time:.2f} saniye | YAKALADI")
        else:
            print(f"Avci ile Arasindaki Mesafe: {mesafe:.1f} Metre | Takip süresi: {elapsed_time:.2f} saniye")