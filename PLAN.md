# SuSFS Log Sorunları Düzeltme Planı

Analizde tespit edilen 6 sorunun her biri için çözüm:

---

## Sorun 1: PID Format Tutarsızlığı
**Dosyalar:** `fs/susfs.c` satır 27-28, `fs/sus_su.c` satır 12-13
**Sorun:** `fs/susfs.c` PID için `%d` (signed) kullanırken, `fs/sus_su.c` `%u` (unsigned) kullanıyor.
**Çözüm:** `pid_t` Linux kernel'de `int` (signed) olarak tanımlıdır. `fs/sus_su.c`'deki `%u` formatlarını `%d` olarak değiştir.
- `sus_su.c` satır 12: SUSFS_LOGI makrosunda `[%u]` → `[%d]` (ikinci %u, pid olanı)
- `sus_su.c` satır 13: SUSFS_LOGE makrosunda `[%u]` → `[%d]` (ikinci %u, pid olanı)

---

## Sorun 2: Yanlış Log Seviyesi (satır 759)
**Dosya:** `fs/susfs.c` satır 759
**Sorun:** `failed setting fake_cmdline_or_bootconfig` mesajı `SUSFS_LOGI` (INFO) ile loglanıyor, başarısızlık olduğu için `SUSFS_LOGE` (ERROR) olmalı.
**Çözüm:** `SUSFS_LOGI` → `SUSFS_LOGE` olarak değiştir.

---

## Sorun 3: LOGE Makrosunda Boşluk Eksikliği
**Dosyalar:** `fs/susfs.c` satır 28, `fs/sus_su.c` satır 13
**Sorun:** LOGE makrolarında `[%s]` ile `fmt` arasında boşluk yok:
- susfs.c: `"susfs:[%u][%d][%s]" fmt` → boşluk yok
- sus_su.c: `"susfs_sus_su:[%u][%u][%s]" fmt` → boşluk yok
**Karşılaştırma:** LOGI makrolarında boşluk var: `"susfs:[%u][%d][%s] " fmt`
**Çözüm:** Her iki dosyada LOGE makrosuna `]` ile `"` arasına boşluk ekle:
- `[%s]"` → `[%s] "`

---

## Sorun 4: Gizleme Loglarında Performans Riski
**Dosya:** `fs/susfs.c` satır 373, 391, 403, 413, 423
**Sorun:** `SUSFS_LOGI("hiding path...")` her dosya erişiminde tetiklenir. Loglama açıkken dmesg'i doldurur.
**Çözüm:** Bu logları `pr_debug` seviyesine düşür (SUSFS_LOGI yerine). `pr_debug` default olarak derlenmez (dynamic debug ile isteğe bağlı açılabilir). Alternatif olarak bu satırları tamamen kaldırmak da bir seçenek.
**Önerilen yaklaşım:** Bu 5 yüksek frekanslı logu `pr_debug`'a çevirmek yerine tamamen kaldırmak en temizi olur — zaten runtime'da `susfs_is_log_enabled=false` yapıldığında susturulurlar. Ama bu davranış değişikliği yaratır. En güvenli yaklaşım: `SUSFS_LOGI` → `pr_debug` ile sarmalayarak sadece explicit debug modunda görünür kılmak.

---

## Sorun 5: Deprecated Komutların Gereksiz Loglanması
**Dosya:** `drivers/kernelsu/supercalls.c` satır 968, 991, 1025
**Sorun:** 3 deprecated komut (ADD_SUS_MOUNT, ADD_TRY_UMOUNT, SUS_SU) hâlâ `pr_info` ile loglanıyor ama hiçbir işlem yapmıyor.
**Çözüm:** Bu 3 satırdaki `pr_info` çağrılarını `pr_debug` ile değiştir. Deprecated komutlar artık sessizce yoksayılır ama dynamic debug ile izlenebilir kalır.

---

## Sorun 6: supercalls.c Loglarının Koşulsuz Olması
**Dosya:** `drivers/kernelsu/supercalls.c` satır 949-1042
**Sorun:** Tüm SUSFS komut logları doğrudan `pr_info` kullanıyor, `CONFIG_KSU_SUSFS_ENABLE_LOG` ve `susfs_is_log_enabled` kontrolü yok.
**Çözüm:** `supercalls.c`'nin başına (mevcut include'ların ardına) aşağıdaki makro tanımını ekle:

```c
#ifdef CONFIG_KSU_SUSFS_ENABLE_LOG
extern bool susfs_is_log_enabled __read_mostly;
#define SUSFS_PR_INFO(fmt, ...) do { if (susfs_is_log_enabled) pr_info(fmt, ##__VA_ARGS__); } while (0)
#else
#define SUSFS_PR_INFO(fmt, ...)
#endif
```

Ardından satır 949-1042 arasındaki tüm `pr_info("susfs: CMD_SUSFS_...)` çağrılarını `SUSFS_PR_INFO(...)` ile değiştir.

**İstisnalar** (koşulsuz kalması gerekenler):
- satır 441: `KSU_MARK_REFRESH: do nothing` — bu SUSFS değil KSU komutu
- satır 1091: `reboot kprobe registered` — başlangıç bilgisi
- satır 1103: `susfs: do nothing` (exit) — tek seferlik

---

## Uygulama Sırası
1. Sorun 3 (LOGE boşluk) — en basit, 2 satır değişikliği
2. Sorun 1 (PID format) — 2 satır değişikliği
3. Sorun 2 (log seviyesi) — 1 satır değişikliği
4. Sorun 4 (performans) — 5 satır değişikliği
5. Sorun 5 (deprecated) — 3 satır değişikliği
6. Sorun 6 (koşulsuz loglar) — makro ekleme + ~15 satır değişikliği
