# 3D Mapping dengan Drone (CoppeliaSim + Python)

UTS Sistem Robotika (SRO) Fall Season  
**Nama:** Oktavian Rifki Danendra  
**NRP:** 5022231107

Quadcopter di CoppeliaSim terbang otomatis mengelilingi arena berisi kotak, silinder, dan kerucut sambil
memindai lingkungan dengan sensor **fast3DLaserScanner**. Hasil scan diubah menjadi point cloud 3D,
dikurangi menjadi voxel, lalu divisualisasikan dengan pewarnaan berdasarkan ketinggian.

## Video dan hasil

Video proses mapping dan hasil 3D mapping ada di Google Drive:

**[Buka folder Google Drive](https://drive.google.com/drive/folders/1Ik7VPm2pZWIu-oK68S5vS6aB16m1mVCD?usp=drive_link)**

## Fitur

| Tingkat | Ketentuan tugas | Implementasi |
|---|---|---|
| C | Mengontrol drone dan menampilkan jalur yang dilewati | Quadcopter mengikuti dummy `target` sepanjang waypoint (angka 8 atau lawnmower). Jalur ditampilkan sebagai garis biru pada peta |
| BC | Melakukan scan 2D | Peta 2D tampak atas (`occupancy_2d`) dari semua titik di atas lantai, disimpan di `hasil_2d_map.png` |
| B | Melakukan scan 3D | Frame depth dari sensor diubah menjadi point cloud 3D dalam koordinat dunia (`capture`, `frame_to_world`) |
| AB | Mengurangi titik menjadi voxel dengan post process | `voxel_downsample` (satu titik per voxel 0,1 m) dan `remove_isolated` (membuang voxel terpencil/noise) |
| A | Color mapping ketinggian dengan post process | Tanah satu warna tetap, objek diwarnai menurut tinggi (colormap `plasma`) lengkap dengan colorbar |

Fitur tambahan:

- **Dua pola terbang:** angka 8 berulang (default 5 putaran) atau lawnmower yang menyapu seluruh area.
- **Obstacle avoidance:** drone menghindari semua shape di scene (kecuali lantai) memakai potential field
  dan `sim.checkDistance`. Target dikembalikan ke garis lajur setelah menghindar.
- **Deteksi macet:** waypoint yang tidak bisa dicapai dilewati otomatis.
- **Pemilihan sumbu kamera otomatis:** mencegah peta tercermin (objek terduplikasi atau memanjang).
- **Preflight check:** skrip menolak berjalan kalau scanner menumpuk di badan drone.

## Sensor

Sensor yang dipakai adalah model **3D laser scanner Fast** bawaan CoppeliaSim (`fast3DLaserScanner`).
Pada scene ini terbaca sebagai satu vision sensor 64x64 piksel, FOV 45°, jangkauan 0,05 sampai 5 m
(nilai pastinya dicetak di terminal saat skrip dijalankan).

Posisi dan orientasi sensor diatur di scene: scanner menempel sebagai anak `Quadcopter`, diletakkan sedikit
di bawah badan drone, dan menghadap ke bawah. Karena drone melintas dari berbagai arah, lantai (ground),
puncak objek, dan sisi objek yang terlihat dari sudut pandang drone ikut terekam.

## Alur kerja

1. Skrip membuat daftar waypoint (angka 8 atau lawnmower).
2. Drone mengikuti `target`. Tiap beberapa langkah simulasi, satu frame depth diambil dari sensor.
3. Tiap pixel depth diubah menjadi titik 3D di koordinat sensor, lalu ditransformasi ke koordinat dunia
   dengan pose sensor saat frame diambil. Titik terlalu dekat (`MIN_RANGE`, badan drone) dan terlalu jauh
   (`MAX_RANGE`) dibuang.
4. Setelah terbang, skrip memilih konvensi sumbu kamera yang membuat objek dari banyak sudut pandang
   menumpuk di tempat yang sama.
5. Semua titik dikurangi menjadi voxel, voxel terpencil dibuang, lalu hasilnya diplot.

## Kebutuhan

- CoppeliaSim (diuji di CoppeliaSim Edu 4.10) dengan Remote API (ZMQ) aktif
- Python 3.10 atau lebih baru

```bash
pip install coppeliasim-zmqremoteapi-client numpy matplotlib
```

## Struktur scene

Skrip mengharapkan objek berikut ada di scene (path bisa diubah di bagian konfigurasi skrip):

| Objek | Keterangan |
|---|---|
| `/Quadcopter` | Model quadcopter bawaan CoppeliaSim |
| `/target` | Dummy yang diikuti drone |
| `/Quadcopter/fast3DLaserScanner` | Scanner, **harus menjadi anak Quadcopter** |
| Cuboid, Cylinder, Cone, dinding | Penghalang yang dipetakan |

Penting: scanner harus diletakkan **di bawah badan drone**, tidak tepat di pusatnya. Kalau posisinya
sama dengan drone, badan keduanya bertabrakan dan drone terlempar saat simulasi mulai. Skrip akan
berhenti dengan pesan error kalau jaraknya kurang dari 5 cm. Simpan scene setelah mengatur posisi scanner.

## Cara menjalankan

1. Buka scene `QuadETS.ttt` di CoppeliaSim. Pastikan hanya satu tab scene yang terbuka dan simulasi belum berjalan.
2. Jalankan skrip dari terminal:

   ```bash
   python drone_3d_mapping.py
   ```

3. Skrip memulai simulasi, menerbangkan drone, menghentikan simulasi, lalu menampilkan peta 3D dan peta 2D.

### Output

| File | Isi |
|---|---|
| `hasil_3d_map.png` | Peta 3D voxel berwarna menurut ketinggian dengan jalur drone |
| `hasil_2d_map.png` | Peta 2D tampak atas dengan jalur drone |
| `pointcloud_raw.npy` | Point cloud mentah (N x 3, meter) |
| `pointcloud_voxel.npy` | Point cloud setelah voxel dan filter noise |

## Konfigurasi

Semua pengaturan ada di bagian atas skrip. Yang paling sering diubah:

| Parameter | Default | Fungsi |
|---|---|---|
| `PATH_MODE` | `"figure8"` | `"figure8"` atau `"lawnmower"` |
| `FIG8_LAPS` | 5 | Jumlah putaran angka 8 |
| `FIG8_A`, `FIG8_B` | 2,0 / 1,2 | Setengah panjang dan lebar angka 8 (m) |
| `ARENA_HALF` | 2,0 | Setengah lebar area sapuan lawnmower (m) |
| `LANE_SPACING` | 1,0 | Jarak antar lajur lawnmower (m). Pilih nilai yang membagi 2 x `ARENA_HALF` dengan pas, misalnya 0,5 |
| `ALTITUDE` | 1,5 | Ketinggian terbang (m) |
| `SAFE_DIST` | 0,9 | Jarak mulai menghindar dari penghalang (m). Turunkan ke sekitar 0,5 kalau jalur terlalu bergelombang |
| `VOXEL_SIZE` | 0,10 | Ukuran voxel (m) |
| `SCAN_EVERY` | 6 | Ambil data sensor tiap N langkah simulasi |
| `MIN_NEIGHBORS` | 2 | Voxel dengan tetangga lebih sedikit dari ini dibuang. 0 untuk mematikan |
| `GROUND_Z` | 0,15 | Voxel dengan z di bawah ini dianggap tanah |

Cakupan scan dari ketinggian h kira-kira 0,83 x h lebar (FOV 45°), jadi `LANE_SPACING` sebaiknya
tidak lebih besar dari itu.

## Pemecahan masalah

| Gejala | Penyebab dan solusi |
|---|---|
| `Exception ... object does not exist` | Nama objek di scene berbeda. Sesuaikan `TARGET_PATH`, `DRONE_PATH`, atau `SCANNER_PATHS` |
| Error "belum menjadi anak /Quadcopter" | Seret `fast3DLaserScanner` ke `Quadcopter` di Scene Hierarchy, lalu simpan scene |
| Error "Scanner menumpuk di pusat drone" | Geser scanner ke bawah badan drone (posisi relatif z sekitar -0,15) dan simpan scene |
| Drone terlempar saat mulai | Scanner menumpuk di badan drone (lihat di atas), atau `target` terlalu jauh dari drone |
| Drone berhenti di dekat dinding atau kotak | Waypoint berada di dalam zona `SAFE_DIST`. Turunkan `SAFE_DIST`, kecilkan `ARENA_HALF`, atau naikkan `ALTITUDE` |
| Objek di peta tampak memanjang atau terduplikasi | Sumbu kamera tertukar. Atur `CAMERA_CONVENTION` ke `"flip_x"` atau `"flip_y"` |
| Titik tersebar jauh di bawah lantai | Nilai depth tidak sesuai satuan. Coba `DEPTH_IN_METERS = False` |
| Muncul titik melayang | Noise sensor atau bagian drone. Naikkan `MIN_RANGE` atau `MIN_NEIGHBORS` |
| Simulasi diam setelah skrip berhenti paksa | Mode stepping tertinggal. Hentikan simulasi lalu mulai ulang CoppeliaSim |

## Struktur repositori

```
.
├── drone_3d_mapping.py   # skrip utama
├── QuadETS.ttt           # scene CoppeliaSim
└── README.md
```
