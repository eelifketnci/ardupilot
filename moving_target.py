import time
import math
import random
from pymavlink import mavutil
import matplotlib.pyplot as plt

# ardupilota mavlink uzerinden baglaniyorum
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
print("Baglanti bekleniyor...")
master.wait_heartbeat()
print("ucaga baglanildi!")

# YENİ: Drone'a yatışsız (Skid-to-Turn) MAVLink komutu basan fonksiyon
def send_stt_velocity_command(master, velocity_x, yaw_rate):
    # MAV_FRAME_BODY_NED kullanıyoruz: X ekseni daima aracın burnudur.
    master.mav.set_position_target_local_ned_send(
        0,                                             
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,            
        0b010111000111, # Sadece V_x ve Yaw_rate dinle, gerisini yok say
        0, 0, 0,                                       
        velocity_x, 0, 0,                              
        0, 0, 0,                                       
        0, yaw_rate                                    
    )

# matematiksel hesaplamalar icin yardimci fonksiyonlar
def calculate_distance(lat1, lon1, lat2, lon2):
    R = 6371000 
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

def hesapla_kerteriz(lat1, lon1, lat2, lon2):
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlambda = math.radians(lon2 - lon1)
    y = math.sin(dlambda) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
    kerteriz = math.degrees(math.atan2(y, x))
    return (kerteriz + 360) % 360

hedef_sayisi = 3
hedefler = []

print("Uçağın baslangic konumu aliniyor...")
msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
baslangic_lat = msg.lat / 1e7
baslangic_lon = msg.lon / 1e7

for i in range(hedef_sayisi):
    hedef_lat = baslangic_lat + random.uniform(-0.002, 0.002)
    hedef_lon = baslangic_lon + random.uniform(-0.002, 0.002)
    hedefler.append({
        'id': i + 1,
        'lat': hedef_lat,
        'lon': hedef_lon,
        'alt': 15.0, # Copter için test irtifasını düşürdüm (15m)
        'yokedildi': False
    })

print(f"{hedef_sayisi} adet hedef basariyla uretildi. Görev basliyor!")
start_time = time.time()

# Grafikler için geçmiş listeleri
avci_lat_gecmisi, avci_lon_gecmisi = [], []
zaman_gecmisi, mesafe_gecmisi = [], []
aci_farki_gecmisi, rmin_gecmisi = [], []      
vurus_noktalari = []
hedef_gecmisleri = {i+1: {'lat': [], 'lon': []} for i in range(hedef_sayisi)}

# İnteraktif Modu Aç (CANLI GRAFİK İÇİN 2x2 EKRAN)
plt.ion() 
fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(18, 10)) 
fig.canvas.manager.set_window_title('Otonom Önleme Ekranı (STT Modeli)')
plt.tight_layout(pad=4.0)
plt.show(block=False) 
son_cizim_zamani = time.time()
secili_hedef = None  

try:
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if not msg:
            continue
            
        su_an = time.time()
        avci_lat = msg.lat / 1e7
        avci_lon = msg.lon / 1e7
        avci_heading = msg.hdg / 100.0 
        gecen_sure = su_an - start_time
        
        avci_lat_gecmisi.append(avci_lat)
        avci_lon_gecmisi.append(avci_lon)
        zaman_gecmisi.append(gecen_sure)
        
        kalan_hedefler = [h for h in hedefler if not h['yokedildi']]
        
        if not kalan_hedefler:
            print(f"\nTEBRIKLER! Tum hedefler {gecen_sure:.1f} saniyede yok edildi!")
            # Son komut olarak drone'u durdur
            send_stt_velocity_command(master, 0.0, 0.0) 
            plt.ioff() 
            plt.show()
            break 
            
        for hedef in kalan_hedefler:
            hedef['lat'] += 0.000009
            hedef['lon'] += 0.000009
            hedef_gecmisleri[hedef['id']]['lat'].append(hedef['lat'])
            hedef_gecmisleri[hedef['id']]['lon'].append(hedef['lon'])
            master.mav.adsb_vehicle_send(
                hedef['id'], int(hedef['lat'] * 1e7), int(hedef['lon'] * 1e7), 0,
                int(hedef['alt'] * 1000), 0, 0, 0,
                f"target-{hedef['id']}".encode('ascii'), 0, 1, 3, 0
            )

        if secili_hedef is None or secili_hedef['yokedildi']:
            min_maliyet = float('inf')
            for hedef in kalan_hedefler:
                mesafe = calculate_distance(avci_lat, avci_lon, hedef['lat'], hedef['lon'])
                hedef_kerteriz = hesapla_kerteriz(avci_lat, avci_lon, hedef['lat'], hedef['lon'])
                aci_farki = abs((hedef_kerteriz - avci_heading + 180) % 360 - 180)
                maliyet = mesafe + (aci_farki * 20.0)
                if maliyet < min_maliyet:
                    min_maliyet = maliyet
                    secili_hedef = hedef
                
        if 'son_zaman' in secili_hedef:
            dt = su_an - secili_hedef['son_zaman']
            if dt > 0:
                ds = calculate_distance(secili_hedef['son_enlem'], secili_hedef['son_boylam'], secili_hedef['lat'], secili_hedef['lon'])
                hedef_hizi_ms = ds / dt
            else:
                hedef_hizi_ms = 0.0
        else:
            hedef_hizi_ms = 0.0
            
        secili_hedef['son_zaman'] = su_an
        secili_hedef['son_enlem'] = secili_hedef['lat']
        secili_hedef['son_boylam'] = secili_hedef['lon']

        aktif_mesafe = calculate_distance(avci_lat, avci_lon, secili_hedef['lat'], secili_hedef['lon'])
        mesafe_gecmisi.append(aktif_mesafe)
                
        VURMA_YARICAPI = 5.0 
        if aktif_mesafe < VURMA_YARICAPI:
            print(f"*** HEDEF {secili_hedef['id']} YOK EDILDI! (Mesafe: {aktif_mesafe:.1f}m) ***")
            vurus_noktalari.append((avci_lon, avci_lat, secili_hedef['id']))
            secili_hedef['yokedildi'] = True
            secili_hedef = None  
            continue 

        # Drone Hızı (Füze Seyir Hızı Sabiti)
        V_sabit = 15.0 
        
        g = 9.81
        maks_yatis_acisi = math.radians(45.0)
        r_min = (V_sabit**2) / (g * math.tan(maks_yatis_acisi))
        
        aktif_kerteriz = hesapla_kerteriz(avci_lat, avci_lon, secili_hedef['lat'], secili_hedef['lon'])
        
        # Matematik için (sağ/sol) dönüş yönünü belirten işaretli hata açısı (eta)
        isaretli_aci_farki_derece = (aktif_kerteriz - avci_heading + 180) % 360 - 180
        isaretli_aci_farki_radyan = math.radians(isaretli_aci_farki_derece)
        
        # Grafikler için mutlak değer
        aktif_aci_farki = abs(isaretli_aci_farki_derece)
        
        aci_farki_gecmisi.append(aktif_aci_farki)
        rmin_gecmisi.append(r_min)

        if aktif_mesafe < (r_min * 1.5) and aktif_aci_farki > 90.0:
            print(f"!!! target-{secili_hedef['id']} ISKALANDI / PAS GECILIYOR! (Ters Açı: {aktif_aci_farki:.1f}°) !!!")
            secili_hedef = None 
            continue

        # --- YENİ: L1 Skid-to-Turn (STT) Güdüm Algoritması ---
        # L1 referans mesafesi (en az 10 metre kilitlenme toleransı)
        L1_dist = max(aktif_mesafe, 10.0) 
        
        # Yanal ivme (lat_acc) ve hedef yaw hızı
        lat_acc = 2 * (V_sabit**2) / L1_dist * math.sin(isaretli_aci_farki_radyan)
        yaw_rate_dem = lat_acc / V_sabit
        
        # Drone'a yatışsız hız ve dönüş komutunu bas
        send_stt_velocity_command(master, V_sabit, yaw_rate_dem)
        # -----------------------------------------------------

        # YENİ: Drone'un anlık yatış (Roll) açısını çekiyoruz
        att_msg = master.recv_match(type='ATTITUDE', blocking=False)
        anlik_roll = 0.0
        if att_msg:
            anlik_roll = math.degrees(att_msg.roll)

        if gecen_sure % 1.0 < 0.1: # Saniyede bir terminale yazdır
            print(f"Hedef-{secili_hedef['id']} | STT L1 | Yaw Hızı: {math.degrees(yaw_rate_dem):.1f}°/s | Anlık YATIŞ (Roll): {anlik_roll:.1f}°")
        
        if su_an - son_cizim_zamani > 0.15:
            ax1.clear(); ax2.clear(); ax3.clear(); ax4.clear()

            ax1.plot(avci_lon_gecmisi, avci_lat_gecmisi, label='Avci Yorungesi (STT)', color='blue', linewidth=2)
            renkler = ['red', 'orange', 'purple', 'brown', 'black', 'gray', 'olive', 'cyan']
            for id_num, hist in hedef_gecmisleri.items():
                if hist['lon']: 
                    ax1.plot(hist['lon'], hist['lat'], color=renkler[(id_num-1)%len(renkler)], linestyle='--')
                    ax1.text(hist['lon'][-1], hist['lat'][-1], f" target-{id_num}", fontsize=8, color=renkler[(id_num-1)%len(renkler)])

            for vn in vurus_noktalari:
                ax1.scatter(vn[0], vn[1], color='green', marker='*', s=300, zorder=5)

            if avci_lon_gecmisi:
                ax1.scatter(avci_lon_gecmisi[0], avci_lat_gecmisi[0], color='cyan', marker='o', s=100, label='Baslangic')

            ax1.set_title('1. Canlı Ekran: Yörünge ve Hedefler')
            ax1.grid(True); ax1.axis('equal') 

            ax2.plot(zaman_gecmisi[-len(mesafe_gecmisi):], mesafe_gecmisi, color='purple', linewidth=2.5, label='Mesafe')
            ax2.axhline(y=VURMA_YARICAPI, color='red', linestyle='--', linewidth=1.5, label=f'Tolerans ({VURMA_YARICAPI}m)')
            ax2.set_title('2. Kilit Mesafesi (m)')
            ax2.grid(True); ax2.legend()

            if aci_farki_gecmisi:
                ax3.plot(zaman_gecmisi[-len(aci_farki_gecmisi):], aci_farki_gecmisi, color='orange', linewidth=2)
                ax3.axhline(y=60.0, color='red', linestyle='--', linewidth=1, label='Kritik Kör Açı (60°)')
            ax3.set_title('3. Hedef Açı Hatası (Derece)')
            ax3.set_ylabel('Açı Farkı'); ax3.set_xlabel('Zaman (s)')
            ax3.grid(True); ax3.legend()

            if mesafe_gecmisi and rmin_gecmisi:
                ax4.plot(zaman_gecmisi[-len(mesafe_gecmisi):], mesafe_gecmisi, color='purple', linewidth=2, label='Hedef Mesafesi')
                ax4.plot(zaman_gecmisi[-len(rmin_gecmisi):], [r * 1.5 for r in rmin_gecmisi], color='red', linestyle='-.', linewidth=2, label='Pas Geçme Sınırı')
                ax4.fill_between(zaman_gecmisi[-len(rmin_gecmisi):], 0, [r * 1.5 for r in rmin_gecmisi], color='red', alpha=0.1)
            ax4.set_title('4. Kinematik Sınır ve Breakaway')
            ax4.set_ylabel('Mesafe (m)'); ax4.set_xlabel('Zaman (s)')
            ax4.grid(True); ax4.legend()

            plt.pause(0.001) 
            son_cizim_zamani = su_an

        time.sleep(0.02)

except KeyboardInterrupt:
    print("\nGorev iptal edildi.")
    send_stt_velocity_command(master, 0.0, 0.0)