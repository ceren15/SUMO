# 🚑 Yeşil Dalga — SUMO Ambulans Önceliklendirme

## 📌 Proje Özeti  
Bu proje, **SUMO (Simulation of Urban Mobility)** üzerinde acil durum araçlarının (ambulans) geçişlerini kolaylaştırmak için trafik ışıklarını **gerçek zamanlı** olarak optimize eder.  
- **GPS verileri** → A* algoritması ile rota hesaplama  
- **Karar Ağaçları** → trafik yoğunluğu, kavşak yapısı ve aciliyet parametrelerine göre ışık optimizasyonu  
- Ambulans kavşağa **150 m yaklaştığında** acil mod aktif olur, geçiş sonrası sistem normale döner.  

---

## 🛠 Teknolojiler  
- **Simülasyon:** SUMO  
- **Algoritmalar:** A*, Karar Ağaçları, ANFIS  
- **Donanım (prototip):** ESP32, GPS modülü, LED trafik ışığı simülasyonu  
- **Dil/Kütüphaneler:** Python, MQTT (iletişim), SUMO API  

---

## 🚀 Kullanım Yönergeleri  
1. **SUMO kurulumu yapın:**  
   👉 [SUMO Install Docs](https://sumo.dlr.de/docs/Installing.html)  

2. **Python bağımlılıklarını yükleyin:**
   pip install -r requirements.txt
   
Simülasyonu başlatın:

python src/main.py --config config/simulation.sumocfg --anfis-model data/anfis.pt --gui

Çalışma prensibi:
Ambulans kavşağa yaklaşınca geliş yönüne yeşil ışık, diğer yönlere kırmızı verilir.
Geçiş tamamlandığında sistem otomatik olarak normal faza döner.

🎯 Beklenen Çıktılar
Ambulansların geçiş süresi azalır
Trafik akışı daha güvenli ve verimli hale gelir
Sistem çoklu ambulans senaryolarını da destekler

👥 Ekip
Takım Adı: Blue Code

Proje: Yeşil Dalga — TEKNOFEST 2025 Akıllı Ulaşım
