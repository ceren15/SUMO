# SUMO Kurulum Rehberi

## Windows İçin SUMO Kurulumu

### 1. SUMO İndirme ve Kurulum

1. **SUMO'yu İndirin:**
   - [SUMO resmi web sitesinden](https://eclipse.dev/sumo/) Windows sürümünü indirin
   - En son kararlı sürümü (1.15.0+) tercih edin

2. **Kurulum:**
   ```bash
   # SUMO kurulum dosyasını çalıştırın
   # Önerilen kurulum yolu: C:\Program Files (x86)\Eclipse\Sumo
   ```

3. **Sistem Değişkenlerini Ayarlayın:**
   ```bash
   # SUMO_HOME çevre değişkenini ayarlayın
   # Örnek: SUMO_HOME=C:\Program Files (x86)\Eclipse\Sumo
   
   # PATH değişkenine SUMO bin klasörünü ekleyin
   # Örnek: %SUMO_HOME%\bin
   ```

### 2. Python Bağımlılıkları

```bash
pip install -r requirements.txt
```

### 3. Kurulum Doğrulama

```bash
sumo --version
python -m src.main --help
```

## Linux/macOS İçin Kurulum

### Ubuntu/Debian:
```bash
sudo apt-get update
sudo apt-get install sumo sumo-tools sumo-doc
```

### macOS (Homebrew):
```bash
brew install sumo
```

## OpenStreetMap Araçları

Harita verilerini işlemek için:

```bash
pip install osmium
pip install geopy
```

## Troubleshooting

### Yaygın Sorunlar:

1. **"sumo: command not found" hatası:**
   - SUMO_HOME ve PATH değişkenlerini kontrol edin
   - Bilgisayarı yeniden başlatın

2. **TraCI bağlantı hatası:**
   - SUMO'nun doğru çalıştığından emin olun
   - Port 8813'ün kullanılabilir olduğunu kontrol edin

3. **Harita yükleme hatası:**
   - OpenStreetMap verilerinin doğru formatda olduğunu kontrol edin
   - netconvert aracının doğru çalıştığından emin olun

## Geliştirme Ortamı

Önerilen IDE'ler:
- PyCharm
- Visual Studio Code
- Sublime Text

Önerilen eklentiler:
- Python extension
- XML/SUMO syntax highlighting 