import time
import math
import random
from pymavlink import mavutil

# --- 1. BAĞLANTIYI KUR ---
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
print("Baglanti bekleniyor...")
master.wait_heartbeat()
print("Avci ucaga baglanildi!")

# --- 2. MESAFE HESAPLAMA FONKSİYONU ---
def calculate_distance(lat1, lon1, lat2, lon2):
    R = 6371000  # Dünya yarıçapı (metre)
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

# --- 3. RASTGELE HEDEFLERİ OLUŞTUR ---
hedef_sayisi = 8
hedefler = []

# Avcının başlangıç konumunu al (Hedefleri onun etrafında spawn etmek için)
print("Avcinin baslangic konumu aliniyor...")
msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
baslangic_lat = msg.lat / 1e7
baslangic_lon = msg.lon / 1e7

# Etrafa 8 tane rastgele hedef saç (Yaklaşık 100-300 metre çapında)
for i in range(hedef_sayisi):
    hedef_lat = baslangic_lat + random.uniform(-0.003, 0.003)
    hedef_lon = baslangic_lon + random.uniform(-0.003, 0.003)
    hedefler.append({
        'id': i + 1,
        'lat': hedef_lat,
        'lon': hedef_lon,
        'alt': 100.0,
        'yokedildi': False
    })

print(f"{hedef_sayisi} adet hedef basariyla uretildi. Av basliyor!")
start_time = time.time()

# --- 4. ANA AVLANMA DÖNGÜSÜ ---
try:
    while True:
        # Avcının anlık konumunu çek
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if not msg:
            continue
            
        avci_lat = msg.lat / 1e7
        avci_lon = msg.lon / 1e7
        
        # Hayatta kalan hedefleri bul
        kalan_hedefler = [h for h in hedefler if not h['yokedildi']]
        
        if not kalan_hedefler:
            print(f"\nTEBRIKLER! Tum hedefler {time.time() - start_time:.1f} saniyede yok edildi!")
            break # Görev bitti, döngüden çık
        
                
        # En yakın hedefi bul (Algoritmanın kalbi)
        en_yakin_hedef = None
        min_mesafe = float('inf')
        
        for hedef in kalan_hedefler:
            mesafe = calculate_distance(avci_lat, avci_lon, hedef['lat'], hedef['lon'])
            if mesafe < min_mesafe:
                min_mesafe = mesafe
                en_yakin_hedef = hedef
                
        # Eğer hedef 10 metreden (vurma yarıçapı) yakındaysa yok et!
        VURMA_YARICAPI = 10.0 # Metre
        if min_mesafe < VURMA_YARICAPI:
            print(f"*** HEDEF {en_yakin_hedef['id']} YOK EDILDI! (Mesafe: {min_mesafe:.1f}m) ***")
            en_yakin_hedef['yokedildi'] = True
            continue # Vurulduğu için listeyi güncellemeye başa dön
            
        # Ekran Çıktısı
        gecen_sure = time.time() - start_time
        print(f"Aktif Hedef: {en_yakin_hedef['id']} | Mesafe: {min_mesafe:.1f} Metre | Kalan Hedef: {len(kalan_hedefler)} | Sure: {gecen_sure:.1f} sn")
        
        # ArduPlane'e bu hedefe gitmesi için emir gönder (DO_REPOSITION)
        master.mav.command_int_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            0, 0, -1.0,
            mavutil.mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,
            0.0, 0.0,
            int(en_yakin_hedef['lat'] * 1e7),
            int(en_yakin_hedef['lon'] * 1e7),
            en_yakin_hedef['alt']
        )
        
        #time.sleep(0.5) # Döngüyü çok hızlı çalıştırıp sistemi yormamak için

except KeyboardInterrupt:
    print("\nGorev iptal edildi.")