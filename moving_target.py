import time
import math
import random
from pymavlink import mavutil
import matplotlib.pyplot as plt

# --- 1. BAĞLANTIYI KUR ---
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
print("Baglanti bekleniyor...")
master.wait_heartbeat()
print("Avci ucaga baglanildi!")

# --- 2. MATEMATİKSEL FONKSİYONLAR ---
def calculate_distance(lat1, lon1, lat2, lon2):
    R = 6371000 
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
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

# --- 3. RASTGELE HEDEFLERİ OLUŞTUR ---
hedef_sayisi = 7
hedefler = []

print("Avcinin baslangic konumu aliniyor...")
msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
baslangic_lat = msg.lat / 1e7
baslangic_lon = msg.lon / 1e7

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

# --- GRAFİK GEÇMİŞ LİSTELERİ ---
avci_lat_gecmisi = []
avci_lon_gecmisi = []
zaman_gecmisi = []
mesafe_gecmisi = []
vurus_noktalari = []

# Hedeflerin yörüngelerini çizmek için ayrı bir sözlük tutuyoruz
hedef_gecmisleri = {i+1: {'lat': [], 'lon': []} for i in range(hedef_sayisi)}

print(f"{hedef_sayisi} adet hedef basariyla uretildi. Av basliyor!")
start_time = time.time()

# --- GRAFİK GEÇMİŞ LİSTELERİ VE CANLI ÇİZİM HAZIRLIĞI ---
avci_lat_gecmisi = []
avci_lon_gecmisi = []
zaman_gecmisi = []
mesafe_gecmisi = []
vurus_noktalari = []
hedef_gecmisleri = {i+1: {'lat': [], 'lon': []} for i in range(hedef_sayisi)}

# 1. İnteraktif Modu Aç (CANLI GRAFİK İÇİN)
plt.ion() 
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 7))
plt.show(block=False) # Kodu durdurmadan pencereyi aç
son_cizim_zamani = time.time()

# --- 4. ANA AVLANMA DÖNGÜSÜ ---
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
            plt.ioff() # Görev bitince grafik açık kalsın diye interaktif modu kapat
            plt.show()
            break 
            
        # Hedefleri Hareket Ettir
        for hedef in kalan_hedefler:
            hedef['lat'] += 0.000001 
            hedef['lon'] += 0.000001
            hedef_gecmisleri[hedef['id']]['lat'].append(hedef['lat'])
            hedef_gecmisleri[hedef['id']]['lon'].append(hedef['lon'])
            master.mav.adsb_vehicle_send(
                hedef['id'], int(hedef['lat'] * 1e7), int(hedef['lon'] * 1e7), 0,
                int(hedef['alt'] * 1000), 0, 0, 0,
                f"TGT-{hedef['id']}".encode('ascii'), 0, 1, 3, 0
            )

        # Maliyet ve Kilitlenme Hesabı
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
                
        # --- HIZ KESTİRİMİ VE HİBRİT GÜDÜM (SAYISAL TÜREV) ---
        if 'son_zaman' in secili_hedef:
            dt = su_an - secili_hedef['son_zaman']
            if dt > 0:
                ds = calculate_distance(secili_hedef['son_enlem'], secili_hedef['son_boylam'], secili_hedef['lat'], secili_hedef['lon'])
                hedef_hizi_ms = ds / dt
                hedef_v_lat = (secili_hedef['lat'] - secili_hedef['son_enlem']) / dt
                hedef_v_lon = (secili_hedef['lon'] - secili_hedef['son_boylam']) / dt
            else:
                hedef_hizi_ms, hedef_v_lat, hedef_v_lon = 0.0, 0.0, 0.0
        else:
            hedef_hizi_ms, hedef_v_lat, hedef_v_lon = 0.0, 0.0, 0.0
            
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

        # --- PAS GEÇME (BREAKAWAY / ABORT) KONTROLÜ ---
        # 1. Uçağın anlık minimum dönüş yarıçapını (R_min) hesapla
        v_x = msg.vx / 100.0
        v_y = msg.vy / 100.0
        anlik_ucak_hizi = math.sqrt(v_x**2 + v_y**2)
        if anlik_ucak_hizi < 5.0: anlik_ucak_hizi = 5.0 
        
        g = 9.81
        maks_yatis_acisi = math.radians(45.0)
        r_min = (anlik_ucak_hizi**2) / (g * math.tan(maks_yatis_acisi))
        
        # 2. Kilitli olduğumuz hedefin şu anki kerterizini ve açı farkını tekrar ölç
        aktif_kerteriz = hesapla_kerteriz(avci_lat, avci_lon, secili_hedef['lat'], secili_hedef['lon'])
        aktif_aci_farki = abs((aktif_kerteriz - avci_heading + 180) % 360 - 180)
        
        # 3. Eğer hedefi sıyırıp geçtiysek (hedef dönüş dairemizin içindeyse) VE arkamızda kaldıysa:
        if aktif_mesafe < (r_min * 1.5) and aktif_aci_farki > 60.0:
            print(f"!!! TGT-{secili_hedef['id']} ISKALANDI / PAS GECILIYOR! (Ters Açı: {aktif_aci_farki:.1f}°) !!!")
            secili_hedef = None # KİLİDİ KIR! Uçak bir sonraki döngüde maliyet fonksiyonunu tekrar çalıştıracak.
            continue

        HIZ_ESIGI = 3.0 
        if hedef_hizi_ms > HIZ_ESIGI:
            t_go = aktif_mesafe / anlik_ucak_hizi 
            hedef_komut_lat = secili_hedef['lat'] + (hedef_v_lat * t_go)
            hedef_komut_lon = secili_hedef['lon'] + (hedef_v_lon * t_go)
            gudum_modu = "ÖNLEME"
        else:
            hedef_komut_lat = secili_hedef['lat']
            hedef_komut_lon = secili_hedef['lon']
            gudum_modu = "SAF TAKİP"

        # Terminale Canlı Log Bas
        print(f"TGT-{secili_hedef['id']} | Mod: {gudum_modu} | Hız: {hedef_hizi_ms:.1f}m/s | Mesafe: {aktif_mesafe:.1f}m")
        
        master.mav.command_int_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            0, 0, -1.0, mavutil.mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,
            0.0, 0.0, int(hedef_komut_lat * 1e7), int(hedef_komut_lon * 1e7), secili_hedef['alt']
        )
        
        # --- CANLI GRAFİK EKRANINI GÜNCELLEME (HER 0.5 SANİYEDE BİR) ---
        if su_an - son_cizim_zamani > 0.15:
            ax1.clear()
            ax2.clear()

            # HARİTA (ax1)
            ax1.plot(avci_lon_gecmisi, avci_lat_gecmisi, label='Avci Yorungesi', color='blue', linewidth=2)
            renkler = ['red', 'orange', 'purple', 'brown', 'pink', 'gray', 'olive', 'cyan']
            for id_num, hist in hedef_gecmisleri.items():
                if hist['lon']: 
                    ax1.plot(hist['lon'], hist['lat'], color=renkler[(id_num-1)%len(renkler)], linestyle='--')
                    ax1.text(hist['lon'][-1], hist['lat'][-1], f" TGT-{id_num}", fontsize=8, color=renkler[(id_num-1)%len(renkler)])

            for vn in vurus_noktalari:
                ax1.scatter(vn[0], vn[1], color='green', marker='*', s=300, zorder=5)

            if avci_lon_gecmisi:
                ax1.scatter(avci_lon_gecmisi[0], avci_lat_gecmisi[0], color='cyan', marker='o', s=100)

            ax1.set_title('Canlı Taktik Ekran: Yörünge')
            ax1.grid(True)
            ax1.axis('equal') # Haritanın yamulmasını engeller

            # MESAFE (ax2)
            ax2.plot(zaman_gecmisi, mesafe_gecmisi, color='purple', linewidth=2.5, label='Mesafe')
            ax2.axhline(y=VURMA_YARICAPI, color='red', linestyle='--', linewidth=1.5, label=f'Tolerans ({VURMA_YARICAPI}m)')
            ax2.set_title('Canlı Taktik Ekran: Kilit Mesafesi')
            ax2.grid(True)
            ax2.legend()

            # Değişiklikleri Ekrana Yansıt
            plt.pause(0.001) 
            son_cizim_zamani = su_an

        # MAVLink haberleşmesinin boğulmaması için kısa uyku
        time.sleep(0.02)

except KeyboardInterrupt:
    print("\nGorev iptal edildi.")