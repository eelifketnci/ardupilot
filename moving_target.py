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

# matematiksel hesaplamalar icin yardimci fonksiyonlar
def calculate_distance(lat1, lon1, lat2, lon2):
    # iki nokta arasindaki mesafeyi haversine formuluyle metre cinsinden buluyorum
    R = 6371000 
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

def hesapla_kerteriz(lat1, lon1, lat2, lon2):
    # hedefin ucaga gore hangi acida durdugunu hesapliyorum
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlambda = math.radians(lon2 - lon1)
    y = math.sin(dlambda) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
    kerteriz = math.degrees(math.atan2(y, x))
    return (kerteriz + 360) % 360

# rastgele sahte hedef olusturuyorum
hedef_sayisi = 1
hedefler = []

print("Uçağın baslangic konumu aliniyor...")
msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
baslangic_lat = msg.lat / 1e7
baslangic_lon = msg.lon / 1e7

for i in range(hedef_sayisi):
    # hedefleri ucagin etrafinda rastgele konumlara atiyorum
    hedef_lat = baslangic_lat + 0.004
    hedef_lon = baslangic_lon + 0.004
    hedefler.append({
        'id': i + 1,
        'lat': hedef_lat,
        'lon': hedef_lon,
        'alt': 100.0,
        'yokedildi': False
    })

print(f"{hedef_sayisi} adet hedef basariyla uretildi. Görev basliyor!")
start_time = time.time()

# grafikte cizdirmek icin yorunge ve mesafe gecmislerini burada tutucam
avci_lat_gecmisi = []
avci_lon_gecmisi = []
zaman_gecmisi = []
mesafe_gecmisi = []
aci_farki_gecmisi = [] # YENİ: Uçağın hedefe olan açı hatası
rmin_gecmisi = []      # YENİ: Anlık minimum dönüş yarıçapı
yaw_cmd_gecmisi = []   # DAHA YENİ: Hedef Yaw Rate
yaw_act_gecmisi = []   # DAHA YENİ: Gerçekleşen Yaw Rate
son_gercek_yaw_rate = 0.0 # Döngüde Attitude mesajı kaçarsa diye tutucu
vurus_noktalari = []
hedef_gecmisleri = {i+1: {'lat': [], 'lon': []} for i in range(hedef_sayisi)}

# 1. İnteraktif Modu Aç (CANLI GRAFİK İÇİN 2x2 EKRAN)
plt.ion() 
fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(18, 10)) # 4 Grafiklik Dashboard
fig.canvas.manager.set_window_title('Otonom Önleme Ekranı')
plt.tight_layout(pad=4.0)
plt.show(block=False) 
son_cizim_zamani = time.time()
# ana görev dongusune basliyoruz
secili_hedef = None  
son_zaman_yaw = time.time()
son_heading_yaw = None

try:
    while True:
        # ucagin anlik konumunu cekiyorum
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if not msg:
            continue
            
        su_an = time.time()
        avci_lat = msg.lat / 1e7
        avci_lon = msg.lon / 1e7
        avci_heading = msg.hdg / 100.0 
        gecen_sure = su_an - start_time
        
        # grafik icin listelere ekliyorum
        avci_lat_gecmisi.append(avci_lat)
        avci_lon_gecmisi.append(avci_lon)
        zaman_gecmisi.append(gecen_sure)
        
        # vurulmamis hedefleri filtreliyorum
        kalan_hedefler = [h for h in hedefler if not h['yokedildi']]
        
        if not kalan_hedefler:
            print(f"\nTEBRIKLER! Tum hedefler {gecen_sure:.1f} saniyede yok edildi!")
            plt.ioff() # isimiz bitti, ekran kapanmasin diye kapatiyorum
            plt.show()
            break 
            
        # hedefleri haritada ufak ufak kaydirip adsb uzerinden basiyorum
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

        # hedef secimi: kararsizlik yapmamasi icin mevcut hedef bitene kadar kilitli kaliyorum
        if secili_hedef is None or secili_hedef['yokedildi']:
            min_maliyet = float('inf')
            # hem bana en yakin hem de burnuma en uygun acida olani buluyorum
            for hedef in kalan_hedefler:
                mesafe = calculate_distance(avci_lat, avci_lon, hedef['lat'], hedef['lon'])
                hedef_kerteriz = hesapla_kerteriz(avci_lat, avci_lon, hedef['lat'], hedef['lon'])
                aci_farki = abs((hedef_kerteriz - avci_heading + 180) % 360 - 180)
                # aci farkina ceza puani verip en dusuk maliyetliyi seciyorum
                maliyet = mesafe + (aci_farki * 20.0)
                if maliyet < min_maliyet:
                    min_maliyet = maliyet
                    secili_hedef = hedef
                
        # hedefin anlik hizini turev alir gibi onceki konumuyla kiyaslayip buluyorum
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

        # kilitlendigim hedefle aramdaki anlik mesafeyi olcuyorum
        aktif_mesafe = calculate_distance(avci_lat, avci_lon, secili_hedef['lat'], secili_hedef['lon'])
        mesafe_gecmisi.append(aktif_mesafe)
                
        VURMA_YARICAPI = 5.0 
        # eger 5 metre icine girdiysek vuruldu sayip listeden dusuruyorum
        if aktif_mesafe < VURMA_YARICAPI:
            print(f"*** HEDEF {secili_hedef['id']} YOK EDILDI! (Mesafe: {aktif_mesafe:.1f}m) ***")
            vurus_noktalari.append((avci_lon, avci_lat, secili_hedef['id']))
            secili_hedef['yokedildi'] = True
            secili_hedef = None  
            continue 

        # chattering ve loiter tuzagini onlemek icin pas gecme mantigi kurdum
        v_x = msg.vx / 100.0
        v_y = msg.vy / 100.0
        anlik_ucak_hizi = math.sqrt(v_x**2 + v_y**2)
        if anlik_ucak_hizi < 5.0: anlik_ucak_hizi = 5.0 
        
        # ucak o hizda en fazla ne kadar dar donebilir onu hesapliyorum (r_min)
        g = 9.81
        maks_yatis_acisi = math.radians(45.0)
        r_min = (anlik_ucak_hizi**2) / (g * math.tan(maks_yatis_acisi))
        
        #-----------------hedef benim burnuma gore tam nerede kaldi
        aktif_kerteriz = hesapla_kerteriz(avci_lat, avci_lon, secili_hedef['lat'], secili_hedef['lon'])
        aktif_aci_farki = abs((aktif_kerteriz - avci_heading + 180) % 360 - 180)
        
        # --- YENİ EKLENEN VERİ KAYITLARI ---
        aci_farki_gecmisi.append(aktif_aci_farki)
        rmin_gecmisi.append(r_min)
        # -----------------------------------
        # --- GERÇEK YAW RATE (PUSULA TÜREVİ) ---
        dt_yaw = su_an - son_zaman_yaw
        if son_heading_yaw is not None and dt_yaw > 0:
            # İki ölçüm arasındaki açı farkını bul (-180 ile +180 arasına sıkıştır)
            delta_hdg = (avci_heading - son_heading_yaw + 180) % 360 - 180
            son_gercek_yaw_rate = delta_hdg / dt_yaw
        else:
            son_gercek_yaw_rate = 0.0

        son_heading_yaw = avci_heading
        son_zaman_yaw = su_an

        # --- YAW RATE MATEMATİKSEL MODELİ (Hedef Yaw Rate İçin) ---
        # 1. İşaretli açı hatasını bul (Sağ pozitif, Sol negatif)
        aci_hatasi_isaretli = (aktif_kerteriz - avci_heading + 180) % 360 - 180
        aci_hatasi_rad = math.radians(aci_hatasi_isaretli)
        
        # 2. C++ L1_dist hesabı: 0.3183 * damping(0.75) * period(17) = ~4.058
        L1_dist = 4.058 * anlik_ucak_hizi
        if aktif_mesafe < L1_dist:
            L1_dist = max(aktif_mesafe, 5.0) # Senin yazdığın pure pursuit ezmesi
            
        # 3. Yanal ivme ve Yaw Rate denklemlerinin çıkarılması
        # K_L1 = 4 * (0.75^2) = 2.25
        lat_accel = 2.25 * (anlik_ucak_hizi**2) / L1_dist * math.sin(aci_hatasi_rad)
        hedef_yaw_rate = math.degrees(lat_accel / anlik_ucak_hizi)
        
        # 4. Grafikler için listelere ekle
        yaw_cmd_gecmisi.append(hedef_yaw_rate)
        yaw_act_gecmisi.append(son_gercek_yaw_rate)
        # ----------------------------------------------------------

        # eger hedef dibime girip arkamda kaldiysa bosuna donmeye calisma, kilidi kirip pas gec
        if aktif_mesafe < (r_min) and aktif_aci_farki > 90.0:
            print(f"!!! target-{secili_hedef['id']} ISKALANDI / PAS GECILIYOR! (Ters Açı: {aktif_aci_farki:.1f}°) !!!")
            secili_hedef = None 
            continue
            
    
        # eger hedef hizliysa onune dogru (onleme), yavas ise tam ustune (saf takip) ucuyorum
        HIZ_ESIGI = 3.0 
        if hedef_hizi_ms > HIZ_ESIGI:
            t_go = aktif_mesafe / anlik_ucak_hizi 
            hedef_komut_lat = secili_hedef['lat'] + (hedef_v_lat * t_go)
            hedef_komut_lon = secili_hedef['lon'] + (hedef_v_lon * t_go)
            gudum_modu = "önlemeli takip"
        else:
            hedef_komut_lat = secili_hedef['lat']
            hedef_komut_lon = secili_hedef['lon']
            gudum_modu = "SAF TAKİP"
            

        print(f"target-{secili_hedef['id']} | Mod: {gudum_modu} | Hız: {hedef_hizi_ms:.1f}m/s | Mesafe: {aktif_mesafe:.1f}m")
        
    
        master.mav.command_int_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            0, 0, -1.0, mavutil.mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,
            0.0, 0.0, int(hedef_komut_lat * 1e7), int(hedef_komut_lon * 1e7), secili_hedef['alt']
        )
    
        # sistemi yormamak icin ekrani sadece belli araliklarla yeniliyorum
        
        if su_an - son_cizim_zamani > 0.15:
            ax1.clear()
            ax2.clear()
            ax3.clear()
            ax4.clear()

            # --- 1. HARİTA VE HEDEFLER (ax1) ---
            ax1.plot(avci_lon_gecmisi, avci_lat_gecmisi, label='Avci Yorungesi', color='blue', linewidth=2)
            
            # Hedeflerin geçmiş yörüngelerini ve ID'lerini çiz
            renkler = ['red', 'orange', 'purple', 'brown', 'black', 'gray', 'olive', 'cyan']
            for id_num, hist in hedef_gecmisleri.items():
                if hist['lon']: 
                    ax1.plot(hist['lon'], hist['lat'], color=renkler[(id_num-1)%len(renkler)], linestyle='--')
                    ax1.text(hist['lon'][-1], hist['lat'][-1], f" target-{id_num}", fontsize=8, color=renkler[(id_num-1)%len(renkler)])

            # Vurulan hedefleri yeşil yıldız ile işaretle
            for vn in vurus_noktalari:
                ax1.scatter(vn[0], vn[1], color='green', marker='*', s=300, zorder=5)

            # Avcının başlangıç noktası
            if avci_lon_gecmisi:
                ax1.scatter(avci_lon_gecmisi[0], avci_lat_gecmisi[0], color='cyan', marker='o', s=100, label='Baslangic')

            ax1.set_title('1. Canlı Ekran: Yörünge ve Hedefler')
            ax1.grid(True)
            ax1.axis('equal') 

            # --- 2. MESAFE (ax2) ---
            ax2.plot(zaman_gecmisi[-len(mesafe_gecmisi):], mesafe_gecmisi, color='purple', linewidth=2.5, label='Mesafe')
            ax2.axhline(y=VURMA_YARICAPI, color='red', linestyle='--', linewidth=1.5, label=f'Tolerans ({VURMA_YARICAPI}m)')
            ax2.set_title('2. Kilit Mesafesi (m)')
            ax2.grid(True)
            ax2.legend()

            # --- 3. AÇI FARKı (ax3) ---
            if aci_farki_gecmisi:
                ax3.plot(zaman_gecmisi[-len(aci_farki_gecmisi):], aci_farki_gecmisi, color='orange', linewidth=2)
                ax3.axhline(y=60.0, color='red', linestyle='--', linewidth=1, label='Kritik Kör Açı (60°)')
            ax3.set_title('3. Hedef Açı Hatası (Derece)')
            ax3.set_ylabel('Açı Farkı')
            ax3.set_xlabel('Zaman (s)')
            ax3.grid(True)
            ax3.legend()

            # --- 4. YAW RATE TAKİBİ (ax4) ---
            if yaw_cmd_gecmisi and yaw_act_gecmisi:
                # Hedef ve Gerçekleşen sinyalleri çizdir
                ax4.plot(zaman_gecmisi[-len(yaw_cmd_gecmisi):], yaw_cmd_gecmisi, color='blue', linewidth=2, linestyle='--', label='Hedef (CMD)')
                ax4.plot(zaman_gecmisi[-len(yaw_act_gecmisi):], yaw_act_gecmisi, color='green', linewidth=2.5, label='Gerçek (ACT)')
                
                # Aradaki hatayı (Error) görselleştirmek için gölgelendirme (Shading)
                ax4.fill_between(zaman_gecmisi[-len(yaw_cmd_gecmisi):], yaw_cmd_gecmisi, yaw_act_gecmisi, color='red', alpha=0.15, label='Hata (Error)')
                
            ax4.set_title('4. L1 Kapalı Çevrim Yaw Rate Takibi')
            ax4.set_ylabel('Yaw Rate (deg/s)')
            ax4.set_xlabel('Zaman (s)')
            ax4.grid(True)
            ax4.legend(loc='upper right')

            plt.pause(0.001) 
            son_cizim_zamani = su_an

        # mavlink hatti sismesin diye kucuk bir delay atiyorum
        time.sleep(0.02)

except KeyboardInterrupt:
    print("\nGorev iptal edildi.")