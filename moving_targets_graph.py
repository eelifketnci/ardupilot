import time
import math
import random
from pymavlink import mavutil
import matplotlib.pyplot as plt

# --- MATEMATİKSEL FONKSİYONLAR ---
def hesapla_mesafe(lat1, lon1, lat2, lon2):
    R = 6371000 
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))

def hesapla_kerteriz(lat1, lon1, lat2, lon2):
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlambda = math.radians(lon2 - lon1)
    y = math.sin(dlambda) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
    kerteriz = math.degrees(math.atan2(y, x))
    return (kerteriz + 360) % 360

# --- BAĞLANTI ---
print("Avci Ucakla Baglanti Kuruluyor...")
master = mavutil.mavlink_connection('udpin:127.0.0.1:14550')
master.wait_heartbeat()
print("Baglanti Basarili!")

# --- HEDEFLERİ OLUŞTURMA ---
msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
baslangic_lat = msg.lat / 1e7
baslangic_lon = msg.lon / 1e7

hedef_sayisi = 4
hedefler = []
for i in range(hedef_sayisi):
    hedefler.append({
        'id': i + 1,
        'lat': baslangic_lat + random.uniform(-0.005, 0.005),
        'lon': baslangic_lon + random.uniform(-0.005, 0.005),
        'alt': 100.0,
        'yokedildi': False
    })

print(f"{hedef_sayisi} Adet Hareketli Hedef Uretildi. Av Basliyor! (Durdurmak icin Ctrl+C)")
start_time = time.time()

# --- GRAFİK GEÇMİŞ LİSTELERİ ---
avci_lat_gecmisi = []
avci_lon_gecmisi = []
zaman_gecmisi = []
mesafe_gecmisi = []
vurus_noktalari = [] # (lon, lat, id) olarak kaydedilecek

# Her hedefin yörüngesini ayrı ayrı tutmak için sözlük yapısı
hedef_gecmisleri = {i+1: {'lat': [], 'lon': []} for i in range(hedef_sayisi)}

# --- ANA DÖNGÜ ---
try:
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if not msg: continue
        
        avci_lat = msg.lat / 1e7
        avci_lon = msg.lon / 1e7
        avci_heading = msg.hdg / 100.0
        gecen_sure = time.time() - start_time
        
        # Uçağın ve zamanın anlık kaydı
        avci_lat_gecmisi.append(avci_lat)
        avci_lon_gecmisi.append(avci_lon)
        zaman_gecmisi.append(gecen_sure)
        
        kalan_hedefler = [h for h in hedefler if not h['yokedildi']]
        
        # Tüm hedefleri hareket ettir ve geçmişlerini kaydet
        for hedef in kalan_hedefler:
            hedef['lat'] += 0.000015
            hedef['lon'] += 0.000015
            hedef_gecmisleri[hedef['id']]['lat'].append(hedef['lat'])
            hedef_gecmisleri[hedef['id']]['lon'].append(hedef['lon'])
            # MAVProxy'de göstermek için ADSB sinyali
            master.mav.adsb_vehicle_send(
                hedef['id'], int(hedef['lat'] * 1e7), int(hedef['lon'] * 1e7), 0,
                int(hedef['alt'] * 1000), 0, 0, 0,
                f"TGT-{hedef['id']}".encode('ascii'), 0, 1, 3, 0
            )

        if not kalan_hedefler:
            print("\n*** GOREV TAMAMLANDI! TUM HEDEFLER YOK EDILDI! ***")
            break

        # KİNEMATİK MALİYET FONKSİYONU İLE EN VURULABİLİR HEDEFİ SEÇ
        en_yakin_hedef = None
        min_maliyet = float('inf')
        gercek_vurus_mesafesi = 0.0

        for hedef in kalan_hedefler:
            # Önce mesafe ve kerteriz hesapla
            mesafe = hesapla_mesafe(avci_lat, avci_lon, hedef['lat'], hedef['lon'])
            hedef_kerteriz = hesapla_kerteriz(avci_lat, avci_lon, hedef['lat'], hedef['lon'])
            
            # Uçağın burnu ile hedef arasındaki açı farkı (0 ile 180 derece arası)
            aci_farki = abs((hedef_kerteriz - avci_heading + 180) % 360 - 180)
            
            # MALİYET HESABI: Her 1 derecelik ters açı, hedefe 5 metre "sanal" uzaklık ekler (Ceza katsayısı: 5.0)
            maliyet = mesafe + (aci_farki * 5.0)
            
            if maliyet < min_maliyet:
                min_maliyet = maliyet
                en_yakin_hedef = hedef
                gercek_vurus_mesafesi = mesafe # Vuruş kontrolü için gerçek mesafeyi sakla
                secilen_aci_farki = aci_farki  # Pas geçme algoritması için açıyı sakla
                
        # Mesafeyi listeye ekle (Grafik için gerçek mesafe kullanılır)
        mesafe_gecmisi.append(gercek_vurus_mesafesi)
                
        # 4. Vurus Kontrolu (Proximity Fuse)
        if gercek_vurus_mesafesi < 5.0:
            print(f"\n---> TGT-{en_yakin_hedef['id']} VURULDU! (Mesafe: {gercek_vurus_mesafesi:.1f}m) <---")
            vurus_noktalari.append((avci_lon, avci_lat, en_yakin_hedef['id']))
            en_yakin_hedef['yokedildi'] = True
            continue
            
        # Dinamik R_min ve Senin Guncelledigin 1.7 Katsayisi!
        v_x = msg.vx / 100.0
        v_y = msg.vy / 100.0
        anlik_hiz = math.sqrt(v_x**2 + v_y**2)
        if anlik_hiz < 5.0: anlik_hiz = 5.0
            
        g = 9.81
        r_min = (anlik_hiz**2) / (g * math.tan(math.radians(45.0)))
        guvenli_alan = r_min * 1.7  # <--- HASSAS AYAR BURADA
        
        hedef_kerteriz = hesapla_kerteriz(avci_lat, avci_lon, en_yakin_hedef['lat'], en_yakin_hedef['lon'])
        aci_farki = (hedef_kerteriz - avci_heading + 180) % 360 - 180  

        if gercek_vurus_mesafesi < guvenli_alan and secilen_aci_farki > 20.0:
            kacis_mesafesi = 2000.0 
            radyan_heading = math.radians(avci_heading)
            delta_lat = (kacis_mesafesi * math.cos(radyan_heading)) / 111320.0
            delta_lon = (kacis_mesafesi * math.sin(radyan_heading)) / (111320.0 * math.cos(math.radians(avci_lat)))
            komut_lat = avci_lat + delta_lat
            komut_lon = avci_lon + delta_lon
            durum_notu = f"PAS GECILIYOR (R_min: {r_min:.1f}m)"
        else:
            komut_lat = en_yakin_hedef['lat']
            komut_lon = en_yakin_hedef['lon']
            durum_notu = f"KILITLENDI"

        

        master.mav.command_int_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            0, 0, -1.0, mavutil.mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,
            0.0, 0.0, int(komut_lat * 1e7), int(komut_lon * 1e7), en_yakin_hedef['alt']
        )
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nUcus manuel olarak durduruldu.")

# --- GRAFİKLERİ ÇİZDİRME AŞAMASI ---
print("Veriler isleniyor, Analiz Paneli hazirlaniyor...")
# Grafikleri çizdirmeden hemen önce listenin boyunu diğerine uydur
if len(zaman_gecmisi) > len(mesafe_gecmisi):
    zaman_gecmisi = zaman_gecmisi[:len(mesafe_gecmisi)]
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 7))

# 1. GRAFİK: UZAYSAL YÖRÜNGE (HARİTA)
ax1.plot(avci_lon_gecmisi, avci_lat_gecmisi, label='Avci Yorungesi', color='blue', linewidth=2, zorder=2)

renkler = ['red', 'orange', 'purple', 'brown']
for id_num, hist in hedef_gecmisleri.items():
    if hist['lon']: # Eğer hedef hiç hareket etmediyse (anında vurulduysa) boş olmasın
        ax1.plot(hist['lon'], hist['lat'], label=f'Hedef {id_num}', color=renkler[(id_num-1)%len(renkler)], linestyle='--', zorder=1)

# Vuruş noktalarına patlama (yıldız) işareti koy
for vn in vurus_noktalari:
    ax1.scatter(vn[0], vn[1], color='green', marker='*', s=300, zorder=5, label=f'Vurus TGT-{vn[2]}')

if avci_lon_gecmisi:
    ax1.scatter(avci_lon_gecmisi[0], avci_lat_gecmisi[0], color='cyan', marker='o', s=100, label='Kalkis Noktasi', zorder=5)

ax1.set_title(f'Coklu Hedef Avci Yorungesi (Katsayi: 1.7)')
ax1.set_xlabel('Boylam (Longitude)')
ax1.set_ylabel('Enlem (Latitude)')
# Tekrarlayan legend etiketlerini temizle
handles, labels = ax1.get_legend_handles_labels()
by_label = dict(zip(labels, handles))
ax1.legend(by_label.values(), by_label.keys())
ax1.grid(True)
ax1.axis('equal')

# 2. GRAFİK: ZAMANA BAĞLI HEDEF UZAKLIĞI
# Mesafe grafiği hedefler değiştikçe testere dişi gibi inip çıkacak
ax2.plot(zaman_gecmisi, mesafe_gecmisi, color='purple', linewidth=2.5, label='Aktif Hedefe Olan Mesafe')
ax2.axhline(y=5, color='red', linestyle='--', linewidth=1.5, label='Vurus Toleransi (5m)')

ax2.set_title('Av Serisi: Zamana Bagli Mesafe (d(t))')
ax2.set_xlabel('Zaman (Saniye)')
ax2.set_ylabel('Mesafe (Metre)')
ax2.legend()
ax2.grid(True)

plt.tight_layout()
plt.show()