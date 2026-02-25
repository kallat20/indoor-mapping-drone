diff --git a//home/bugra/indoor-mapping-drone/docs/RUNBOOK.md b//home/bugra/indoor-mapping-drone/docs/RUNBOOK.md
@@ -0,0 +1,203 @@
+## Indoor Mapping Drone – Çalıştırma Runbook’u
+
+Bu runbook, “uçan sistem”i katman katman ayağa kaldırmak için kısa komut seti verir.
+
+---
+
+## 1. Ortamı hazırlama
+
+Terminalde (her yeni shell için):
+
+```bash
+cd ~/indoor-mapping-drone
+source tools/activate.sh
+```
+
+Beklenen çıktı (örnek):
+
+- `[activate] OK: ROS_DISTRO=humble | WS=/home/bugra/indoor-mapping-drone/ros2_ws`
+
+---
+
+## 2. Sadece sim + sensör + RViz (mevcut workflow)
+
+Tek komutla tmux workflow:
+
+```bash
+cd ~/indoor-mapping-drone
+tools/run_sim.sh
+```
+
+Tmux layout:
+
+- Sol panel: Gazebo + bridge + TF + RViz (launch: `drone_bringup/main.launch.xml`)
+- Sağ üst: `tools/check_health.sh` loop (her 2 saniyede bir sağlık raporu)
+- Sağ alt: `ros2_ws` içinde etkileşimli shell
+
+Tmux içindeyken:
+
+- Paneller arası geçiş: `Ctrl+b` ardından ok tuşları
+- Tmux’tan çıkmak için:
+  - Sadece detach: `Ctrl+b d`
+  - Tamamen kapatmak için: panellerde çalışan prosesleri `Ctrl+C` ile sonlandır.
+
+---
+
+## 3. Health check nasıl okunur?
+
+`tools/check_health.sh` çıktısından beklenenler:
+
+- `== NODES ==` altında:
+  - `rviz2`, `ros_gz_bridge` node’ları ve `robot_state_publisher`/`tf2` ile ilgili node’lar.
+- `== TOPICS ==` altında:
+  - `/clock`
+  - `/scan`
+  - `/pi_camera/image_raw`
+  - `/pi_camera/camera_info`
+  - `/tf`, `/tf_static`
+- `== CLOCK (once) ==`:
+  - Tek seferlik `/clock` mesajı (sim zamanı artıyor olmalı).
+- `== SCAN (once) ==` ve `== CAMERA INFO (once) ==`:
+  - Her ikisinde de `OK` görmelisin.
+- `== IMAGE HZ (short) ==`:
+  - `/pi_camera/image_raw` için mantıklı bir Hz değeri (çok düşük veya 0 olmamalı).
+- `== TF base_link -> lidar_240_link (short) ==`:
+  - TF echo’nun birkaç satır üretmesi, hata atmaması.
+
+Bu seviyede her şey yeşil ise: sim + sensör + TF + RViz altyapısı sağlıklı.
+
+---
+
+## 4. PX4 SITL entegrasyonu – hedef ve çalışma şekli
+
+### 4.1. Hedef seviye
+
+Bu kilometre taşında hedef:
+
+- PX4 SITL süreç olarak çalışıyor.
+- PX4 tarafında prearm check’leri geçmiş (sensörler/parametreler OK).
+- Araç **disarmed** durumda, **arm komutunu bekliyor**.
+- Bu durumu:
+  - PX4’in kendi CLI’sinden **veya**
+  - ROS2 üzerinden `vehicle_status` benzeri bir topic’ten gözlemleyebilmek.
+
+Tam uçuş kontrolü / offboard vb. sonraki aşama.
+
+### 4.2. PX4 SITL nasıl koşturulacak? (dış dünya varsayımı)
+
+Bu repo, PX4 kaynak kodunu içermiyor. Varsayım:
+
+- PX4 firmware ayrı bir klasörde:
+  - Örnek: `~/PX4-Autopilot`
+- PX4 SITL build’i hazır:
+  - Örnek komutlar (sadece referans, gerçek komut senin PX4 versiyonuna göre değişebilir):
+
+```bash
+cd ~/PX4-Autopilot
+make px4_sitl gz_x500     # veya benzeri bir hedef
+```
+
+Bu repo tarafında, PX4’i doğrudan build etmiyoruz; sadece “çalıştırma” için bir wrapper script sağlayacağız.
+
+### 4.3. `tools/run_px4_sitl.sh` ile çalışma (planlanan)
+
+PX4’i tek komutla başlatmak için şu pattern kullanılacak:
+
+1. Ortam değişkeni ile PX4 komutunu tanımla:
+
+```bash
+export PX4_SITL_CMD='cd ~/PX4-Autopilot && make px4_sitl gz_x500'
+```
+
+2. Bu repo kökünde:
+
+```bash
+cd ~/indoor-mapping-drone
+tools/run_px4_sitl.sh
+```
+
+Beklenen davranış:
+
+- Script, `PX4_SITL_CMD` ortam değişkenini okur ve aynen çalıştırır.
+- Böylece:
+  - Farklı PX4 versiyonları / branch’leri için
+  - Farklı hedefler (`gz_x500`, `gz_iris` vs.)
+  - Farklı çalışma klasörleri
+  bu repo kodunu değiştirmeden yönetilebilir.
+
+Eğer `PX4_SITL_CMD` set edilmemişse script:
+
+- Kullanıcıya kısa bir hata mesajı ve örnek kullanım basar,
+- Exit code != 0 ile çıkar.
+
+---
+
+## 5. PX4 + sim birlikte nasıl kullanılacak? (ilk iterasyon)
+
+İlk aşamada hedef:
+
+- PX4 SITL kendi dünyasında / kendi launch’ı ile çalışabilir.
+- Bu repo’daki sim ve sensör katmanı bağımsız da çalışabilir.
+- Ortak nokta:
+  - Zaman (sim time)
+  - Sensör topic’leri
+  - Mümkünse ROS2 üzerinden PX4 `vehicle_status` takibi
+
+Önerilen workflow (ilk basit versiyon):
+
+1. Terminal 1 – PX4 SITL:
+
+```bash
+cd ~/indoor-mapping-drone
+export PX4_SITL_CMD='cd ~/PX4-Autopilot && make px4_sitl gz_x500'
+tools/run_px4_sitl.sh
+```
+
+2. Terminal 2 – Sim + sensör + RViz:
+
+```bash
+cd ~/indoor-mapping-drone
+tools/run_sim.sh
+```
+
+3. Terminal 3 – Health / PX4 gözlemi:
+
+- Genel sim health:
+
+```bash
+cd ~/indoor-mapping-drone
+tools/check_health.sh
+```
+
+- PX4 tarafı (varsayım: PX4 ROS2 bridge / `px4_ros_com` kurulmuş ve topic’ler açık):
+
+```bash
+ros2 topic list | grep /fmu/out/vehicle_status || echo 'vehicle_status topic yok'
+ros2 topic echo /fmu/out/vehicle_status --once
+```
+
+`vehicle_status` mesajında:
+
+- `arming_state` ve ilgili flag’lerden “prearm OK, disarmed” durumu okunabilir.
+- İleride, bu runbook’a bu alanların decode’u daha detaylı eklenebilir.
+
+---
+
+## 6. Sorun giderme (PX4 entegrasyonu)
+
+İlk PX4 entegrasyon iterasyonunda beklenebilecek tipik sorunlar:
+
+- `tools/run_px4_sitl.sh`:
+  - Hata: `PX4_SITL_CMD env var not set`
+    - Çözüm: Ortam değişkenini doğru komutla export et (bkz. 4.3).
+- PX4 build hataları:
+  - Bu repo’nun kapsamı dışında; PX4 dokümantasyonuna bakmak gerekiyor.
+- ROS2 tarafında `vehicle_status` topic’inin görünmemesi:
+  - `ros2 topic list` ile hiç `/fmu/out/...` topic’i yoksa:
+    - Muhtemelen `px4_ros_com` / uXRCE-DDS agent tarafı kurulmamış veya çalışmıyor.
+  - Bu seviyede, runbook sadece “varsa oku” desteği veriyor; kurulumu ayrıca ele almak gerekecek.
+
+Bu aşamada amaç:
+
+- Bu repo tarafında PX4 için net hook noktaları oluşturmak (script + health + runbook),
+- PX4 spesifik kurulum / tuning’i ayrı bir adımda ele almak.
