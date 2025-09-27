# Windows Kurulum Rehberi - Detaylı

## 🎯 Gerekli Yazılımlar

1. **Python 3.8+**
2. **SUMO 1.15.0+**
3. **Git** (opsiyonel - zaten kurulu)

## 📦 Adım 1: Python Kurulumu (5 dakika)

### İndirme
1. [Python.org](https://www.python.org/downloads/) sitesine gidin
2. **"Download Python 3.11.x"** butonuna tıklayın
3. Dosya indirildikten sonra çalıştırın

### Kurulum
1. **ÖNEMLİ:** ✅ **"Add Python to PATH"** kutucuğunu işaretleyin
2. **"Install Now"** tıklayın
3. Kurulum tamamlandıktan sonra **"Close"**

### Doğrulama
CMD veya PowerShell açın:
```cmd
python --version
# Çıktı: Python 3.11.x olmalı
```

## 🚗 Adım 2: SUMO Kurulumu (10 dakika)

### İndirme
1. [SUMO Downloads](https://eclipse.dev/sumo/resources/) sayfasına gidin
2. Windows sekmesine geçin
3. **"sumo-win64-1.15.0.msi"** (veya en son sürüm) indirin

### Kurulum
1. İndirilen `.msi` dosyasını çalıştırın
2. **"Next"** ile devam edin
3. Kurulum yolu: `C:\Program Files (x86)\Eclipse\Sumo` (değiştirmeyin)
4. **"Install"** tıklayın
5. Tamamlandıktan sonra **"Finish"**

### Çevre Değişkenlerini Ayarlama

#### Otomatik Yöntem:
1. Proje klasöründe `setup_environment.bat` dosyasını çalıştırın
2. Administrator olarak çalıştırın (sağ tık → "Run as administrator")

#### Manuel Yöntem:
1. **Windows + R** → `sysdm.cpl` → Enter
2. **"Advanced"** sekmesi → **"Environment Variables"**
3. **System Variables** bölümünde:
   - **"New"** → Variable name: `SUMO_HOME` → Value: `C:\Program Files (x86)\Eclipse\Sumo`
   - **"Path"** değişkenini seçin → **"Edit"** → **"New"** → `%SUMO_HOME%\bin` ekleyin
4. **"OK"** ile kaydedin
5. **Bilgisayarı yeniden başlatın**

### Doğrulama
CMD açın:
```cmd
sumo --version
# Çıktı: Eclipse SUMO sumo Version 1.15.0 olmalı
```

## 🐍 Adım 3: Python Bağımlılıkları (3 dakika)

### Terminal açın
1. **Windows + R** → `cmd` → Enter
2. Proje klasörüne gidin:
```cmd
cd C:\Users\Mustafa\Desktop\sumo
```

### Bağımlılıkları yükleyin
```cmd
pip install -r requirements.txt
```

Eğer `pip` bulunamazsa:
```cmd
python -m pip install -r requirements.txt
```

## ✅ Kurulum Testi

Terminal'de aşağıdaki komutları çalıştırın:

```cmd
# Python testi
python --version

# SUMO testi  
sumo --version

# SUMO GUI testi
sumo-gui

# Python bağımlılık testi
python -c "import traci; print('TraCI başarıyla yüklendi!')"
```

## 🚨 Yaygın Sorunlar ve Çözümler

### Problem 1: "python is not recognized"
**Çözüm:**
1. Python kurulumunu tekrar yapın
2. **"Add Python to PATH"** kutucuğunu işaretlemeyi unutmayın
3. Bilgisayarı yeniden başlatın

### Problem 2: "sumo is not recognized"
**Çözüm:**
1. SUMO_HOME çevre değişkenini kontrol edin
2. PATH'e `%SUMO_HOME%\bin` eklendiğinden emin olun
3. Bilgisayarı yeniden başlatın

### Problem 3: "Permission denied" hatası
**Çözüm:**
1. CMD'yi administrator olarak çalıştırın
2. Antivirus yazılımını geçici olarak kapatın

### Problem 4: pip bağımlılık hatası
**Çözüm:**
```cmd
# Pip'i güncelleyin
python -m pip install --upgrade pip

# Bağımlılıkları tek tek yükleyin
pip install traci
pip install sumolib
pip install numpy
```

## 🎉 Kurulum Tamamlandı!

Tüm testler başarılı olduysa, artık projeyi çalıştırmaya hazırsınız!

Sonraki adım: **Basit test simülasyonunu çalıştırma**

```cmd
# Test simülasyonu
python src/main.py --test-mode
``` 