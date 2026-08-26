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
hedef_sayisi = 8
hedefler = []

print("Baslangic konumu aliniyor...")
msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
baslangic_lat = msg.lat / 1e7
baslangic_lon = msg.lon / 1e7
home_lat, home_lon = baslangic_lat, baslangic_lon

for i in range(hedef_sayisi):
    hedef_lat = baslangic_lat + random.uniform(-0.0015, 0.0015)
    hedef_lon = baslangic_lon + random.uniform(-0.0015, 0.0015)
    hedefler.append({
        'id': i + 1,
        'lat': hedef_lat,
        'lon': hedef_lon,
        'alt': 15.0,
        'yokedildi': False
    })

print(f"{hedef_sayisi} adet hedef olusturuldu. STT Moduna geciliyor...")
set_stt_mode(master)
start_time = time.time()

# Grafikler için geçmiş kayıtları
avci_lat_gecmisi, avci_lon_gecmisi = [], []
zaman_gecmisi, mesafe_gecmisi = [], []
roll_gecmisi = []
vurus_noktalari = []
hedef_gecmisleri = {i+1: {'lat': [], 'lon': []} for i in range(hedef_sayisi)}

# Canlı Grafik Ekranı (2x2)
plt.ion()
fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 9))
fig.canvas.manager.set_window_title('C++ Onboard L1-STT Ekranı')
plt.tight_layout(pad=3.5)
plt.show(block=False)
son_cizim_zamani = time.time()
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
        
        anlik_roll = math.degrees(att_msg.roll) if att_msg else 0.0

        avci_lat_gecmisi.append(avci_lat)
        avci_lon_gecmisi.append(avci_lon)
        zaman_gecmisi.append(gecen_sure)
        roll_gecmisi.append(anlik_roll)

        kalan_hedefler = [h for h in hedefler if not h['yokedildi']]

        if not kalan_hedefler:
            print(f"\nTEBRIKLER! Tum hedefler {gecen_sure:.1f} saniyede C++ L1-STT ile vuruldu!")
            plt.ioff()
            plt.show()
            break

        # Hedeflerin Hareketi (Dinamik Kaçış)
        for hedef in kalan_hedefler:
            hedef['lat'] += 0.000008
            hedef['lon'] += 0.000008
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
                
                # 3. MALIYET (COST) FONKSIYONU
                # 1 Radyanlık (57 derece) ters açı = 40 metrelik ceza puanı (Bunu uçuşa göre artırıp azaltabilirsin)
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
        target_n, target_e = gps_to_ned_m(secili_hedef['lat'], secili_hedef['lon'], home_lat, home_lon)
        send_target_to_drone(master, target_n, target_e, -secili_hedef['alt'])

        if gecen_sure % 1.0 < 0.1:
            print(f"Hedef-{secili_hedef['id']} | Mesafe: {aktif_mesafe:.1f}m | C++ Roll (Yatis): {anlik_roll:.2f}° (0° Hedef)")

        # Grafik Çizimi (150 ms aralıkla)
        if su_an - son_cizim_zamani > 0.15:
            ax1.clear(); ax2.clear(); ax3.clear(); ax4.clear()

            # 1. Canlı Yörünge
            ax1.scatter(home_lon, home_lat, color='cyan', marker='o', s=80, label='Başlangıç Noktası', zorder=5)
            ax1.plot(avci_lon_gecmisi, avci_lat_gecmisi, label='Avci (C++ STT)', color='blue', linewidth=2)
            renkler = ['red', 'orange', 'purple', 'brown', 'magenta', 'yellow', 'black', 'gray']
            for id_num, hist in hedef_gecmisleri.items():
                if hist['lon']:
                    ax1.scatter(hist['lon'][-1], hist['lat'][-1], color=renkler[(id_num-1)%len(renkler)], marker='X', s=150, label=f'Target-{id_num} (Son Konum)', zorder=5)
                    ax1.plot(hist['lon'], hist['lat'], color=renkler[(id_num-1)%len(renkler)], linestyle='--')
                    ax1.text(hist['lon'][-1], hist['lat'][-1], f" Target-{id_num}", fontsize=8)
            for vn in vurus_noktalari:
                ax1.scatter(vn[0], vn[1], color='green', marker='*', s=250, zorder=5)
            ax1.set_title('1. Görev Yörüngesi ve Önleme Noktaları')
            ax1.grid(True); ax1.axis('equal'); ax1.legend()

            # 2. Kilitlenme Mesafesi
            ax2.plot(zaman_gecmisi[-len(mesafe_gecmisi):], mesafe_gecmisi, color='purple', linewidth=2)
            ax2.axhline(y=VURMA_YARICAPI, color='red', linestyle='--', label=f'İmha Sınırı ({VURMA_YARICAPI}m)')
            ax2.set_title('2. Hedefe Kalan Mesafe (m)')
            ax2.grid(True); ax2.legend()

            # 3. Sıfır Yatış (STT Kanıtı)
            ax3.plot(zaman_gecmisi[-len(roll_gecmisi):], roll_gecmisi, color='crimson', linewidth=2)
            ax3.axhline(y=0.0, color='green', linestyle='--', linewidth=2, label='STT Referans (0°)')
            ax3.set_title('3. Gövde Yatış Açısı (Roll °) - STT Doğrulaması')
            ax3.set_ylabel('Roll (Derece)'); ax3.set_xlabel('Zaman (s)')
            ax3.set_ylim([-10, 10])
            ax3.grid(True); ax3.legend()

            # 4. Hedeften Kaçış / Yaklaşma Profili
            ax4.plot(zaman_gecmisi[-len(mesafe_gecmisi):], mesafe_gecmisi, color='darkblue', linewidth=2)
            ax4.set_title('4. Yaklaşma Eğrisi')
            ax4.set_xlabel('Zaman (s)'); ax4.set_ylabel('Mesafe (m)')
            ax4.grid(True)

            plt.pause(0.001)
            son_cizim_zamani = su_an

        time.sleep(0.02)

except KeyboardInterrupt:
    print("\nTest durduruldu.")