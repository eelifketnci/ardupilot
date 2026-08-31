import time
import math
import random
from pymavlink import mavutil
import matplotlib.pyplot as plt

# ArduPilot'a MAVLink üzerinden bağlan
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
print("Baglanti bekleniyor...")
master.wait_heartbeat()
print("Ucaga baglanildi!")

# ---------------- RESETLEME BLOĞU ----------------
print("Önceki ucustan kalan C++ hafizasi temizleniyor...")
# Uçağı zorla GUIDED (Mod 4) moduna alarak Mod 29'dan çıkmasını sağlıyoruz.
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    4, 0, 0, 0, 0, 0
)
time.sleep(1.5) #kapatması için süre tanı
print("Hafiza temizlendi. Yeni goreve hazir!")
# --------------------------------------------------------

# 1. Özel L1_STT Moduna (29) Geçiş Fonksiyonu
def set_stt_mode(master):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        29, 0, 0, 0, 0, 0
    )
    print("Mod 29 (L1_STT) aktif edildi!")

# 2. ArduPilot C++ ModeSTT'ye Dinamik Hedef Konumunu Basma
def send_target_to_drone(master, target_n_m, target_e_m, target_d_m):
    """
    ArduPilot'taki ModeSTT'ye hedef koordinatlarını LOCAL NED (Metre) cinsinden iletir.
    """
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111111000, # Sadece X, Y, Z konumlarını dinle
        target_n_m, target_e_m, target_d_m,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )

# Mesafe Hesaplama (Haversine - Metre)
def calculate_distance(lat1, lon1, lat2, lon2):
    R = 6371000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi / 2.0)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

# GPS Koordinatını Başlangıç Noktasına Göre Yerel Metreye (NED) Çevirme
def gps_to_ned_m(lat, lon, origin_lat, origin_lon):
    R = 6378137.0 # Dünya yarıçapı
    dLat = math.radians(lat - origin_lat)
    dLon = math.radians(lon - origin_lon)
    north = dLat * R
    east = dLon * (R * math.cos(math.radians(origin_lat)))
    return north, east

# Görev ve Hedef Tanımları
hedef_sayisi = 4
hedefler = []

print("Baslangic konumu ve otopilot orijini (NED) esitleniyor...")

# İçerideki eski mesajları temizle
while master.recv_match(type='GLOBAL_POSITION_INT', blocking=False): pass
while master.recv_match(type='LOCAL_POSITION_NED', blocking=False): pass

# Hem GPS hem de kalkışa göre anlık Metre (NED) konumunu al
msg_gps = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
msg_ned = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)

# Uçağın script yeniden başlatıldığı andaki GPS konumu
script_start_lat = msg_gps.lat / 1e7
script_start_lon = msg_gps.lon / 1e7
home_lat, home_lon = script_start_lat, script_start_lon

#Uçağın C++ (SITL) orijinine göre havada bulunduğu anki ofseti (metre)
script_start_n = msg_ned.x
script_start_e = msg_ned.y

for i in range(hedef_sayisi):
    hedef_lat = script_start_lat + random.uniform(-0.0015, 0.0015)
    hedef_lon = script_start_lon + random.uniform(-0.0015, 0.0015)
    hedefler.append({
        'id': i + 1,
        'lat': hedef_lat,
        'lon': hedef_lon,
        'alt': 30.0,
        'yokedildi': False
    })

print(f"{hedef_sayisi} adet hedef olusturuldu")
set_stt_mode(master)
start_time = time.time()

# Grafikler için geçmiş kayıtları
avci_lat_gecmisi, avci_lon_gecmisi = [], []
zaman_gecmisi, mesafe_gecmisi = [], []
vurus_noktalari = []
hedef_gecmisleri = {i+1: {'lat': [], 'lon': []} for i in range(hedef_sayisi)}
# Canlı Grafik Ekranı (2x2)
plt.ion()
fig, ((ax1, ax2)) = plt.subplots(2, 1, figsize=(10, 10))
fig.canvas.manager.set_window_title('L1-STT Ekranı')
plt.tight_layout(pad=3.5)
plt.show(block=False)
son_cizim_zamani = time.time()
son_print_zamani = time.time()
secili_hedef = None


try:
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        att_msg = master.recv_match(type='ATTITUDE', blocking=False)
        
        if not msg:
            time.sleep(0.01)
            continue

        su_an = time.time()
        avci_lat = msg.lat / 1e7
        avci_lon = msg.lon / 1e7
        gecen_sure = su_an - start_time
        

        avci_lat_gecmisi.append(avci_lat)
        avci_lon_gecmisi.append(avci_lon)
        zaman_gecmisi.append(gecen_sure)


        kalan_hedefler = [h for h in hedefler if not h['yokedildi']]

        if not kalan_hedefler:
            print(f"\nTEBRIKLER! Tum hedefler {gecen_sure:.1f} saniyede L1-STT ile vuruldu!")
            plt.ioff()
            plt.show()
            break

        # Hedeflerin Hareketi (Dinamik Kaçış)
        for hedef in kalan_hedefler:
            hedef['lat'] += 0.000009
            hedef['lon'] += 0.000009
            hedef_gecmisleri[hedef['id']]['lat'].append(hedef['lat'])
            hedef_gecmisleri[hedef['id']]['lon'].append(hedef['lon'])

        # En Yakın ve Açısal Olarak En Uygun Hedefi Seç (Maliyet Fonksiyonu)
        if secili_hedef is None or secili_hedef['yokedildi']:
            
            # Uçağın anlık yönelimini (Yaw) al (Radyan cinsinden, Kuzey = 0)
            ucak_yaw = att_msg.yaw if att_msg else 0.0
            
            en_iyi_hedef = None
            en_dusuk_maliyet = float('inf')
            
            for hedef in kalan_hedefler:
                # 1. Mesafe Hesabı (Metre)
                mesafe = calculate_distance(avci_lat, avci_lon, hedef['lat'], hedef['lon'])
                
                # 2. Açısal Fark (Heading Error) Hesabı
                # Drone'un şu anki konumunu merkez alarak hedefin yönünü buluyoruz
                hedef_n, hedef_e = gps_to_ned_m(hedef['lat'], hedef['lon'], avci_lat, avci_lon)
                hedef_acisi = math.atan2(hedef_e, hedef_n)
                
                # Açıyı -pi ile +pi arasına sıkıştırıyoruz ki 359 derece ile 1 derece arasındaki fark devasa çıkmasın
                aci_farki = abs(hedef_acisi - ucak_yaw)
                aci_farki = math.atan2(math.sin(aci_farki), math.cos(aci_farki)) 
                aci_farki = abs(aci_farki) # Sağa veya sola dönmek bizim için aynı maliyette
                
                # 3. MALIYET FONKSIYONU
                # 1 Radyanlık (57 derece) ters açı = 40 metrelik ceza puanı
                maliyet = mesafe + (aci_farki * 40.0) 
                
                if maliyet < en_dusuk_maliyet:
                    en_dusuk_maliyet = maliyet
                    en_iyi_hedef = hedef
            
            secili_hedef = en_iyi_hedef

        aktif_mesafe = calculate_distance(avci_lat, avci_lon, secili_hedef['lat'], secili_hedef['lon'])
        mesafe_gecmisi.append(aktif_mesafe)

        # Hedef Vurulma Kontrolü (Hit Radius: 5m)
        VURMA_YARICAPI = 3.0
        if aktif_mesafe < VURMA_YARICAPI:
            print(f"*** HEDEF {secili_hedef['id']} YOK EDILDI! (Mesafe: {aktif_mesafe:.1f}m) ***")
            vurus_noktalari.append((avci_lon, avci_lat, secili_hedef['id']))
            secili_hedef['yokedildi'] = True
            secili_hedef = None
            continue

        # Hedef Koordinatlarını NED Metreye Çevir ve C++ Otopilotuna Gönder
        # 1. Hedefin Python'ın başladığı noktaya göre metre farkı
        target_dn, target_de = gps_to_ned_m(secili_hedef['lat'], secili_hedef['lon'], script_start_lat, script_start_lon)
        
        # 2. Bu farkı C++'ın gerçek orijinine (kalkış noktasına) ekleyerek senkronize et
        target_n = script_start_n + target_dn
        target_e = script_start_e + target_de
        
        send_target_to_drone(master, target_n, target_e, -secili_hedef['alt'])

        if su_an - son_print_zamani > 0.3:
            print(f"Hedef-{secili_hedef['id']} | Mesafe: {aktif_mesafe:.1f}m | Hedef sayisi: {len(kalan_hedefler)} | Zaman: {gecen_sure:.1f}s")
            son_print_zamani = su_an

        # Grafik Çizimi (150 ms aralıkla)
        if su_an - son_cizim_zamani > 0.15:
            ax1.clear(); ax2.clear()

            # 1. Canlı Yörünge
            ax1.scatter(home_lon, home_lat, color='cyan', marker='o', s=80, label='Başlangıç Noktası', zorder=5)
            ax1.plot(avci_lon_gecmisi, avci_lat_gecmisi, label='Drone', color='blue', linewidth=2)
            renkler = ['red', 'orange', 'purple', 'brown', 'magenta', 'yellow', 'black', 'gray']
            for id_num, hist in hedef_gecmisleri.items():
                if hist['lon']:
                    ax1.scatter(hist['lon'][-1], hist['lat'][-1], color=renkler[(id_num-1)%len(renkler)], marker='X', s=70, label=f'Target-{id_num} (Son Konum)', zorder=5)
                    ax1.plot(hist['lon'], hist['lat'], color=renkler[(id_num-1)%len(renkler)], linestyle='--')
                    ax1.text(hist['lon'][-1], hist['lat'][-1], f" Target-{id_num}", fontsize=8)
            for vn in vurus_noktalari:
                ax1.scatter(vn[0], vn[1], color='green', marker='*', s=150, zorder=5)
            ax1.set_title('1. Görev Yörüngesi (Canlı Konumlar)')
            ax1.grid(True); ax1.axis('equal'); ax1.legend()

            # 2. Kilitlenme Mesafesi
            ax2.plot(zaman_gecmisi[-len(mesafe_gecmisi):], mesafe_gecmisi, color='purple', linewidth=2)
            ax2.axhline(y=VURMA_YARICAPI, color='red', linestyle='--', label=f'İmha Sınırı ({VURMA_YARICAPI}m)')
            ax2.set_title('2. Hedefe Kalan Mesafe (m)')
            ax2.grid(True); ax2.legend()
            
            plt.pause(0.001)
            son_cizim_zamani = su_an

        time.sleep(0.02)

except KeyboardInterrupt:
    print("\nTest durduruldu.")