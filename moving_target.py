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
hedef_sayisi = 4
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

# --- 4. ANA AVLANMA DÖNGÜSÜ ---
secili_hedef = None  # Hedef kilidi

try:
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if not msg:
            continue
            
        avci_lat = msg.lat / 1e7
        avci_lon = msg.lon / 1e7
        avci_heading = msg.hdg / 100.0 
        gecen_sure = time.time() - start_time
        
        avci_lat_gecmisi.append(avci_lat)
        avci_lon_gecmisi.append(avci_lon)
        zaman_gecmisi.append(gecen_sure)
        
        kalan_hedefler = [h for h in hedefler if not h['yokedildi']]
        
        if not kalan_hedefler:
            print(f"\nTEBRIKLER! Tum hedefler {gecen_sure:.1f} saniyede yok edildi!")
            break 
            
        # --- HEDEFLERİ HAREKET ETTİR ---
        for hedef in kalan_hedefler:
            hedef['lat'] += 0.000005  # Hedeflerin kaçış hızı
            hedef['lon'] += 0.000005
            # Çizim için hedeflerin o anki konumunu kaydet
            hedef_gecmisleri[hedef['id']]['lat'].append(hedef['lat'])
            hedef_gecmisleri[hedef['id']]['lon'].append(hedef['lon'])
            
            # (Opsiyonel) Haritada/Terminalde canlı görmek istersen ADSB sinyali yayar
            master.mav.adsb_vehicle_send(
                hedef['id'], int(hedef['lat'] * 1e7), int(hedef['lon'] * 1e7), 0,
                int(hedef['alt'] * 1000), 0, 0, 0,
                f"TGT-{hedef['id']}".encode('ascii'), 0, 1, 3, 0
            )

        # --- KİLİTLENME VE MALİYET HESABI ---
        if secili_hedef is None or secili_hedef['yokedildi']:
            min_maliyet = float('inf')
            
            for hedef in kalan_hedefler:
                mesafe = calculate_distance(avci_lat, avci_lon, hedef['lat'], hedef['lon'])
                hedef_kerteriz = hesapla_kerteriz(avci_lat, avci_lon, hedef['lat'], hedef['lon'])
                aci_farki = abs((hedef_kerteriz - avci_heading + 180) % 360 - 180)
                
                # Açı Ceza Katsayısı: 4.0
                maliyet = mesafe + (aci_farki * 4.0)
                
                if maliyet < min_maliyet:
                    min_maliyet = maliyet
                    secili_hedef = hedef
                
        # Aktif hedef sürekli hareket ettiği için mesafeyi güncel koordinatlarla ölçüyoruz
        aktif_mesafe = calculate_distance(avci_lat, avci_lon, secili_hedef['lat'], secili_hedef['lon'])
        mesafe_gecmisi.append(aktif_mesafe)
                
        VURMA_YARICAPI = 10.0 
        if aktif_mesafe < VURMA_YARICAPI:
            print(f"*** HEDEF {secili_hedef['id']} YOK EDILDI! (Mesafe: {aktif_mesafe:.1f}m) ***")
            vurus_noktalari.append((avci_lon, avci_lat, secili_hedef['id']))
            secili_hedef['yokedildi'] = True
            secili_hedef = None  
            continue 
            
        print(f"Kilit: TGT-{secili_hedef['id']} | Mesafe: {aktif_mesafe:.1f}m | Kalan: {len(kalan_hedefler)} | Sure: {gecen_sure:.1f}s")
        
        # Uçağa hareketli hedefin GÜNCEL konumunu sürekli gönderiyoruz (Saf Takip - Pure Pursuit)
        master.mav.command_int_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            0, 0, -1.0, mavutil.mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,
            0.0, 0.0, int(secili_hedef['lat'] * 1e7), int(secili_hedef['lon'] * 1e7), secili_hedef['alt']
        )
        
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nGorev iptal edildi.")

# --- 5. GRAFİKLERİ ÇİZDİRME ---
print("Veriler isleniyor, Analiz Paneli hazirlaniyor...")

min_len = min(len(zaman_gecmisi), len(mesafe_gecmisi))
zaman_gecmisi = zaman_gecmisi[:min_len]
mesafe_gecmisi = mesafe_gecmisi[:min_len]

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 7))

# HARİTA GRAFİĞİ
ax1.plot(avci_lon_gecmisi, avci_lat_gecmisi, label='Avci Yorungesi', color='blue', linewidth=2, zorder=2)

# Hareketli hedeflerin yörüngelerini çizdir
renkler = ['red', 'orange', 'purple', 'brown', 'pink', 'gray', 'olive', 'cyan']
for id_num, hist in hedef_gecmisleri.items():
    if hist['lon']: # Hedefin konumu kaydedilmişse
        ax1.plot(hist['lon'], hist['lat'], label=f'TGT-{id_num} Rotasi', color=renkler[(id_num-1)%len(renkler)], linestyle='--', zorder=1)
        # Hedefin son vurulduğu (veya simülasyon bittiği) yere ismini yaz
        ax1.text(hist['lon'][-1], hist['lat'][-1], f" TGT-{id_num}", fontsize=9, color=renkler[(id_num-1)%len(renkler)])

for vn in vurus_noktalari:
    ax1.scatter(vn[0], vn[1], color='green', marker='*', s=300, zorder=5, label=f'Vurus TGT-{vn[2]}')

if avci_lon_gecmisi:
    ax1.scatter(avci_lon_gecmisi[0], avci_lat_gecmisi[0], color='cyan', marker='o', s=100, label='Kalkis Noktasi', zorder=5)

ax1.set_title('8 Hareketli Hedefli Akilli Avci Yorungesi')
ax1.set_xlabel('Boylam (Longitude)')
ax1.set_ylabel('Enlem (Latitude)')

# Tekrarlayan legend'leri temizleme
handles, labels = ax1.get_legend_handles_labels()
by_label = dict(zip(labels, handles))
# Haritada legend çok kalabalık olmasın diye sadece önemli olanları seçebiliriz ama şimdilik hepsini koyuyoruz
ax1.legend(by_label.values(), by_label.keys(), fontsize=8, loc='best')
ax1.grid(True)
ax1.axis('equal')

# MESAFE GRAFİĞİ
ax2.plot(zaman_gecmisi, mesafe_gecmisi, color='purple', linewidth=2.5, label='Aktif Hedefe Olan Mesafe')
ax2.axhline(y=VURMA_YARICAPI, color='red', linestyle='--', linewidth=1.5, label='Vurus Toleransi (10m)')

ax2.set_title('Av Serisi: Zamana Bagli Mesafe')
ax2.set_xlabel('Zaman (Saniye)')
ax2.set_ylabel('Mesafe (Metre)')
ax2.legend()
ax2.grid(True)

plt.tight_layout()
plt.show()