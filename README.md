# ÆSIR Kernel | Divine Power, Silent Dominion. | KernelSU Next v3.0.1 + SuSFS v2.0.0 for OnePlus 9 Pro (lemonadep) & OnePlus 9 (lemonade)

<p align="center">
  <img src="https://i.imgur.com/lfo6Z8h.png?raw=true" alt="ÆSIR Header Image"/>
</p>

ÆSIR Kernel is a custom kernel source for the **OnePlus 9 Pro (lemonadep)** and the **OnePlus 9 (lemonade)**. It's [upstreamed](#upstreamed-crdroid-android-repos) from [crDroid Project](https://crdroid.net) with **KernelSU Next v3.0.1** + **SuSFS v2.0.0** integrated directly into the kernel source.

Follow this link to join the [Telegram channel](https://t.me/aesirkernel)

**I know what I'm doing. Take me to the** [installation](#installation)

# **AI Awareness Notice:**
⚠️ **Disclaimer**: This code has been *enhanced* with **Claude Code**.

While AI did the heavy lifting, human oversight remained mandatory. Every commit has been **manually reviewed**, and the entire codebase is **fully tested** and **verified to work correctly**. No robots were allowed to merge code unsupervised.

## Warning & Disclaimer

> - **NO WARRANTY.** This software is provided "as-is" without any warranty of any kind, express or implied. Use at your own risk. You've been **WARNED!!!**
> - **Device brick risk**: Flashing custom kernels and ROMs can permanently brick your device. You are solely responsible for any damage to *your device*, *data loss*, *eye twitching*, *mental blackouts*, or *thermonuclear war*.
> - **Prebuilt ZIP compatibility**: The prebuilt ROM+kernel zip available in the [releases](https://github.com/bcrtvkcs/aesir_kernel_oneplus_sm8350/releases) section is for **OnePlus 9 Pro (lemonadep)** and **OnePlus 9 (lemonade)**. The prebuilt ZIP files are **NOT** compatible with each other. Flashing it on an incompatible device **WILL** brick it.
> - **For the other devices**; This repository is **NOT** compatible to build for any other Snapdragon 888 (SM8350/Lahaina) devices.

### Although everything has been tested, this project is still a work in progress and may contain bugs. If you encounter a bug or have a feature request, [please let me know](https://github.com/bcrtvkcs/aesir_kernel_oneplus_sm8350/issues).

## Problem

Stock crDroid kernel does not include the newest KernelSU Next or the newest root hiding capabilities (SuSFS). Traditional root solutions are increasingly detectable by banking apps, Play Integrity API checks (SafetyNet successor), and other tamper detection mechanisms. Users who need root for legitimate purposes (ad blocking, customization, call recording, etc.) are locked out of these apps.

## Solution

This fork integrates [KernelSU Next](https://github.com/KernelSU-Next/KernelSU-Next) v3.0.1 and [SuSFS](https://gitlab.com/simonpunk/susfs4ksu/-/tree/abf5866ea052c4109e1f1c001655773d5d1ac298) v2.0.0 directly into the kernel source tree, shipped as a single flashable crDroid 12.7 (Android 16) ROM (latest) zip. Instead of kprobes, root is implemented through **9 inline syscall hooks** hand-placed in kernel source files - making detection significantly harder. SuSFS hides all root artifacts (paths, mounts, maps, kernel symbols) while SELinux remains Enforcing and Play Integrity passes at DEVICE level.

## Installation

### Instructions for **OnePlus 9 Pro (lemonadep)**

1. Follow the official [crDroid installation guide for OnePlus 9 Pro](https://crdroid.net/lemonadep/12/install).
2. At the **zip flashing step**, flash the zip you downloaded from this repository's [releases](https://github.com/bcrtvkcs/aesir_kernel_oneplus_sm8350/releases) *instead of the stock crDroid zip*.
3. (Optional GAaps) When asked to sideload [GApps](https://nikgapps.com/crdroid-official), choose 'Yes' to reboot to recovery or 'No' if you don't want gapps and want to reboot to system. Now if you choosed to install GApps, simply sideload *GApps.zip* the same way you installed *crDroid.zip* then reboot to system.
4. After booting up, install the latest version of [KernelSU Next manager](https://github.com/KernelSU-Next/KernelSU-Next/releases) on your device. Alternatively, you can use the [nightly manager](https://t.me/ksunext_ci).
5. (Optional) Open KernelSU Next manager and install the a meta-module for Magick Mount module management alongside OverlayFS. *Hybrid Mount* recommended.
6. Install the [BRENE module](https://github.com/rrr333nnn333/BRENE) by rrr333nnn333 from within the manager to control SuSFS features.

> In step 5, you can also use the [susfs4ksu module](https://github.com/sidex15/susfs4ksu-module/actions/workflows/build.yml) nightly builds from sidex15, but [BRENE module](https://github.com/rrr333nnn333/BRENE) **hides better**.

> If you are using [Bindhosts](https://github.com/bindhosts/bindhosts), select Mode 0 (default) in the module settings. Otherwise, any application will be able to **SEE** your modified hosts file.

> The prebuilt zip in Releases contains the full crDroid 12.7 ROM (latest) + ÆSIR Kernel. The installation process is identical to a standard crDroid installation. The build script retrieves the sources directly from the [crDroid upstream repositories](#upstreamed-crdroid-android-repos). Whenever an update is released on the official crDroid website, I rebuild the ROM and the kernel and post them in the Releases section. Alternatively, you can [build the ROM with my custom kernel repository yourself](#building-from-source).

### Instructions for **OnePlus 9 (lemonade)**

1. Follow the official [crDroid installation guide for OnePlus 9](https://crdroid.net/lemonade/12/install).
2. At the **zip flashing step**, flash the zip you downloaded from this repository's [releases](https://github.com/bcrtvkcs/aesir_kernel_oneplus_sm8350/releases) *instead of the stock crDroid zip*.
3. (Optional GAaps) When asked to sideload [GApps](https://nikgapps.com/crdroid-official), choose 'Yes' to reboot to recovery or 'No' if you don't want gapps and want to reboot to system. Now if you choosed to install GApps, simply sideload *GApps.zip* the same way you installed *crDroid.zip* then reboot to system.
4. After booting up, install the latest version of [KernelSU Next manager](https://github.com/KernelSU-Next/KernelSU-Next/releases) on your device. Alternatively, you can use the [nightly manager](https://t.me/ksunext_ci).
5. (Optional) Open KernelSU Next manager and install the a meta-module for Magick Mount module management alongside OverlayFS. *Hybrid Mount* recommended.
6. Install the [BRENE module](https://github.com/rrr333nnn333/BRENE) by rrr333nnn333 from within the manager to control SuSFS features.

> In step 5, you can also use the [susfs4ksu module](https://github.com/sidex15/susfs4ksu-module/actions/workflows/build.yml) nightly builds from sidex15, but [BRENE module](https://github.com/rrr333nnn333/BRENE) **hides better**.

> If you are using [Bindhosts](https://github.com/bindhosts/bindhosts), select Mode 0 (default) in the module settings. Otherwise, any application will be able to **SEE** your modified hosts file.

> I can't test the OnePlus 9 (lemonade)'s prebuilt zip because I don't have the device. It probably works fine, but there might be something I don't know about. Just in case. If you would like to test it and share your results, I would be appriciate it. If you encounter a bug or want to have a feature request, [please let me know](https://github.com/bcrtvkcs/aesir_kernel_oneplus_sm8350/issues).

> The prebuilt zip in Releases contains the full crDroid 12.7 ROM (latest) + ÆSIR Kernel. The installation process is identical to a standard crDroid installation. The build script retrieves the sources directly from the [crDroid upstream repositories](#upstreamed-crdroid-android-repos). Whenever an update is released on the official crDroid website, I rebuild the ROM and the kernel and post them in the Releases section. Alternatively, you can [build the ROM with my custom kernel repository yourself](#building-from-source).

### If you'd like to support the project, feel free to leave a star ⭐

## Features

| Component | Details |
|-----------|---------|
| **Kernel** | 5.4.302 (aarch64) |
| **ROM** | crDroid 12.7 (Android 16) (latest) |
| **Device** | OnePlus 9 Pro (lemonadep) or OnePlus 9 (lemonade) |
| **SoC** | Qualcomm Snapdragon 888 (SM8350/Lahaina) |
| **KernelSU Next** | v3.0.1 (version code 33006) |
| **SuSFS** | v2.0.0 |
| **SELinux** | Enforcing |
| **Hook Mode** | GKI1 - Inline (manual) syscall hooks |

### SuSFS v2.0.0 Features

| Feature | Status |
|---------|--------|
| SUS Path Support | Enabled |
| SUS Maps Support | Enabled |
| SUS Mount Support | Enabled |
| SUS Kstat Support | Enabled |
| Spoof Uname Support | Enabled |
| Spoof Cmdline/Bootconfig | Enabled |
| Open Redirect Support | Enabled |
| Logging Support | Enabled |
| Hide KSU SuSFS Symbols | Enabled |
| AVC Log Spoofing | Enabled |
| ~~Try Umount Support~~ | Deprecated |
| ~~Auto Default Mount~~ | Deprecated |
| ~~Auto Bind Mount~~ | Deprecated |
| ~~Auto Try Umount Bind~~ | Deprecated |
| ~~Magic Mount Support~~ | Deprecated |
| ~~OverlayFS Auto Kstat Support~~ | Deprecated |

> **Deprecated features:** SuSFS v2.0.0 intentionally removed the legacy per-mount management features (try_umount, auto mounts, magic mount, overlayfs auto kstat). These have been replaced by a single unified **SUS Mount Support** mechanism (`hide_sus_mnts_for_non_su_procs`) that hides all suspicious mounts from non-root processes at once - simpler configuration, smaller attack surface, same result.

### Inline Syscall Hooks

Instead of using kprobes (which can be detected), ÆSIR Kernel uses **9 inline hooks** placed directly in kernel source files:

| File | Hook |
|------|------|
| `fs/exec.c` | execve handler |
| `fs/read_write.c` | vfs_read handler |
| `fs/stat.c` | stat handlers (x2) |
| `fs/open.c` | faccessat handler |
| `drivers/input/input.c` | input event handler |
| `kernel/sys.c` | setresuid handler (manager fd injection) |
| `kernel/reboot.c` | reboot handler (IOCTL communication) |
| `kernel/seccomp.c` | seccomp bypass for reboot supercall |

## Performance Optimizations

ÆSIR Kernel includes a comprehensive set of performance and battery life optimizations applied directly at the source level - no Magisk modules or post-boot scripts required. All tweaks are active from the moment the kernel boots.

### Network & I/O

| Optimization | Details |
|-------------|---------|
| **BBR TCP Congestion Control** | Google's BBR algorithm as default - better throughput and lower latency than CUBIC, especially on lossy mobile networks |
| **FQ/FQ_CODEL Qdisc** | Fair Queue and CoDel packet scheduling for reduced bufferbloat |
| **TCP Timestamps Disabled** | Saves 12 bytes per TCP packet header; reduces overhead on mobile connections |
| **BFQ I/O Scheduler** | Budget Fair Queueing as default - optimized for interactive workloads and flash storage |
| **Block I/O Stats Disabled** | Per-I/O accounting overhead removed from all block devices by default |
| **MMC SPI CRC Disabled** | Eliminates unnecessary CRC checks on storage transfers (modern UFS/eMMC have hardware ECC) |

### CPU & Scheduler

| Optimization | Details |
|-------------|---------|
| **Schedutil Governor** | Frequency scaling driven directly by the scheduler's utilization signals |
| **Tunable Scaling: None** | Prevents auto-scaling of scheduler granularity values by CPU count - our tuned values are used as-is |
| **Min Granularity: 1ms** | CFS minimum timeslice increased from 750us - fewer context switches, better throughput |
| **Wakeup Granularity: 1.5ms** | Wakeup preemption threshold raised from 1ms - reduces unnecessary task preemption |
| **Migration Cost: 200us** | Reduced from 500us - scheduler migrates tasks across CPUs faster for lower latency while avoiding excessive cross-cluster thrashing |
| **Child Runs First** | Forked child processes run before parent - reduces Copy-on-Write page faults on fork+exec |
| **Colocation Threshold: 20** | Top-app tasks receive sched boost at lower utilization than stock (35), balancing responsiveness with thermal efficiency |
| **Perf CPU Overhead: 10%** | Perf sampling CPU time limit reduced from 25% to 10% |

### Memory & VM

| Optimization | Details |
|-------------|---------|
| **ZRAM with LZ4** | Compressed RAM swap using the fast LZ4 algorithm instead of lzo-rle |
| **Page Cluster: 0** | Swap read-ahead disabled - unnecessary with ZRAM since data is already in memory |
| **VFS Cache Pressure: 50** | Dentry/inode caches retained longer, reducing filesystem metadata I/O |
| **Dirty Ratio: 30%** | Allows more dirty pages in RAM before forcing writeback - balances burst write performance with Dynamic Fsync screen-on flush safety |
| **VM Stat Interval: 30s** | Per-CPU vmstat counter flush reduced from every 1s to every 30s - less jitter |

### Power Management

| Optimization | Details |
|-------------|---------|
| **Power-Efficient Workqueues** | Enabled by default - workqueue tasks prefer idle CPUs |
| **Timer Migration Disabled** | Timers stay on their CPU instead of migrating to busy cores - idle CPUs reach deeper sleep states |
| **Dynamic Fsync** | Fsync calls are skipped while the screen is off; all pending data is flushed when the screen turns on (`/sys/kernel/dyn_fsync/Fsync_enabled`) |

### GPU

| Optimization | Details |
|-------------|---------|
| **Adrenoboost** | GPU frequency boost on heavy workloads - three levels configurable via `/sys/class/kgsl/kgsl-3d0/devfreq/adrenoboost` (0=off, 1=light, 2=medium, 3=aggressive) |

### Debug Overhead Removed

Disabled at compile time: `PROFILING`, `SCHEDSTATS`, `DEBUG_INFO`, `DEBUG_STACK_USAGE`, `DEBUG_MEMORY_INIT`, `FUNCTION_ERROR_INJECTION` - reduces kernel image size and eliminates runtime tracing overhead.

> **Note:** All runtime-tuneable parameters (scheduler, VM, network) can still be adjusted via `sysctl` or `sysfs` if you want to override the defaults.

## Building from Source

### crDroid 12.7 (Android 16) - OnePlus 9 Pro (lemonadep) or OnePlus 9 (lemonade)  Build Guide

ÆSIR Kernel is built as part of the full crDroid ROM - there is no need to compile the kernel separately. The `brunch` build system handles everything automatically.

### 1. Requirements

- Ubuntu 20.04+ (or 22.04/24.04) (Linux Mint recommended)
- Minimum 400 GB free disk space
- Minimum 16 GB RAM (32 GB recommended)
- Good internet connection (source code is ~100 GB)
- Depending on your system, the **first build** takes 4-10 hours.
> Once you have configured `ccache`, which is step 6, the **second compilation** will only take 30-60 minutes.

### 2. Install Dependencies

```bash
sudo apt install bc bison build-essential ccache curl flex g++-multilib \
  gcc-multilib git git-lfs gnupg gperf imagemagick lib32ncurses-dev \
  lib32readline-dev lib32z1-dev liblz4-tool libncurses6 libncurses-dev \
  libsdl1.2-dev libssl-dev libwxgtk3.2-dev libxml2 libxml2-utils lzop \
  pngcrush rsync schedtool squashfs-tools xsltproc zip zlib1g-dev
```

### 3. Install the Repo Tool

```bash
mkdir -p ~/bin
curl https://storage.googleapis.com/git-repo-downloads/repo > ~/bin/repo
chmod a+x ~/bin/repo
export PATH=~/bin:$PATH  # also add this to your .bashrc
```

### 4. Download the crDroid Source

```bash
mkdir -p ~/crDroid && cd ~/crDroid
repo init -u https://github.com/crdroidandroid/android.git -b 16.0 --git-lfs --no-clone-bundle
repo sync -c --no-clone-bundle --no-tags -j$(nproc)
```

This will take several hours on the first run.

### 5. Integrate The Kernel (Critical Step)

The crDroid build system uses **local manifests** to override the default kernel source with your own fork. Create (or edir) the file `.repo/local_manifests/roomservice.xml` in your crDroid source tree.

Replace the kernel entry with your fork's repository.

`roomservice.xml` for **OnePlus 9 Pro (lemonadep)**:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<manifest>
  <project path="device/oneplus/lemonadep" remote="crdroid" name="crdroidandroid/android_device_oneplus_lemonadep" revision="16.0" />
  <project path="device/oneplus/sm8350-common" remote="crdroid" name="crdroidandroid/android_device_oneplus_sm8350-common" revision="16.0" />
  <project path="vendor/oneplus/lemonadep" remote="crdroid-gitlab" name="crdroidandroid/proprietary_vendor_oneplus-lemonadep" revision="16.0" />
  <project path="kernel/oneplus/sm8350" remote="github" name="bcrtvkcs/aesir_kernel_oneplus_sm8350" revision="16.0" />
  <project path="hardware/oplus" remote="crdroid" name="crdroidandroid/android_hardware_oplus" revision="16.0" />
  <project path="vendor/oneplus/sm8350-common" remote="crdroid-gitlab" name="crdroidandroid/proprietary_vendor_oneplus_sm8350-common" revision="16.0" />
</manifest>
```

`roomservice.xml` for **OnePlus 9 (lemonade)**:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<manifest>
  <project path="device/oneplus/lemonade" remote="crdroid" name="crdroidandroid/android_device_oneplus_lemonade" revision="16.0" />
  <project path="device/oneplus/sm8350-common" remote="crdroid" name="crdroidandroid/android_device_oneplus_sm8350-common" revision="16.0" />
  <project path="vendor/oneplus/lemonade" remote="crdroid-gitlab" name="crdroidandroid/proprietary_vendor_oneplus_lemonade" revision="16.0" />
  <project path="kernel/oneplus/sm8350" remote="github" name="bcrtvkcs/aesir_kernel_oneplus_sm8350" revision="16.0" />
  <project path="hardware/oplus" remote="crdroid" name="crdroidandroid/android_hardware_oplus" revision="16.0" />
  <project path="vendor/oneplus/sm8350-common" remote="crdroid-gitlab" name="crdroidandroid/proprietary_vendor_oneplus_sm8350-common" revision="16.0" />
</manifest>
```

Sync kernel repo.

```bash
repo sync kernel/oneplus/sm8350 --force-sync
```

### 6. Set Up ccache

```bash
export USE_CCACHE=1
export CCACHE_EXEC=$(which ccache)
ccache -M 50G
```

### 7. Build

for OnePlus 9 Pro (lemonadep):

```bash
cd ~/crDroid
. build/envsetup.sh
brunch lemonadep
```

for OnePlus 9 (lemonade):

```bash
cd ~/crDroid
. build/envsetup.sh
brunch lemonade
```

The build system will automatically:

- Use the `vendor/lahaina-qgki_defconfig` defconfig to compile the kernel
- Enable KernelSU + SUSFS (via Kconfig defaults)
- Package the entire ROM

### 8. Output

After a successful build, the ROM zip will be at:

```
~/crDroid/out/target/product/lemonadep/crDroidAndroid-16.0-*-lemonadep-*.zip
```
or
```
~/crDroid/out/target/product/lemonade/crDroidAndroid-16.0-*-lemonade-*.zip
```

#### Summary Flow

```
repo sync → edit local_manifests to use your kernel fork → repo sync → brunch lemonadep or brunch lemonade
```

There is no need to compile the kernel separately - `brunch` handles everything.

## Upstreamed crDroid Android Repos

### OnePlus 9 Pro (lemonadep) repos:
https://github.com/crdroidandroid/android_device_oneplus_lemonadep
https://github.com/crdroidandroid/android_device_oneplus_sm8350-common
https://gitlab.com/crdroidandroid/proprietary_vendor_oneplus-lemonadep
https://github.com/crdroidandroid/android_hardware_oplus
https://gitlab.com/crdroidandroid/proprietary_vendor_oneplus_sm8350-common

### OnePlus 9 (lemonade) repos:
https://github.com/crdroidandroid/android_device_oneplus_lemonade
https://github.com/crdroidandroid/android_device_oneplus_sm8350-common
https://gitlab.com/crdroidandroid/proprietary_vendor_oneplus_lemonade
https://github.com/crdroidandroid/android_hardware_oplus
https://gitlab.com/crdroidandroid/proprietary_vendor_oneplus_sm8350-common

## Credits

#### Core
- [KernelSU-Next](https://github.com/KernelSU-Next/KernelSU-Next) - Kernel-based root solution
- [SuSFS](https://gitlab.com/simonpunk/susfs4ksu/-/tree/abf5866ea052c4109e1f1c001655773d5d1ac298) - Root hiding subsystem by simonpunk
- [crDroid](https://crdroid.net/) - Custom Android ROM

#### Performance Optimizations
- [NotZeetaa / YAKT](https://github.com/NotZeetaa/YAKT) - Yet Another Kernel Tweaker; runtime tuning defaults ported to source
- [kdrag0n](https://github.com/kdrag0n) - VM stat interval optimization
- [tytydraco](https://github.com/tytydraco) - Colocation threshold tweak
- [flar2 / Aaron Segaert](https://github.com/flar2) - Dynamic Fsync and Adrenoboost implementations (blu_spark / ElementalX)
- [arter97](https://github.com/arter97) - BBR TCP, LZ4 ZRAM, schedutil defaults (arter97-kernel)
- [engstk / McQuaid](https://github.com/engstk) - Power-efficient workqueues, debug debloat (blu_spark)
- [Nathan Chancellor / Eva Kernel](https://github.com/nathanchance) - Power-efficient workqueues reference

#### Tooling
- [Claude Code](https://claude.ai/code) - AI-assisted kernel integration

---

# Original `Android Common Kernel README`

# How do I submit patches to Android Common Kernels

1. BEST: Make all of your changes to upstream Linux. If appropriate, backport to the stable releases.
   These patches will be merged automatically in the corresponding common kernels. If the patch is already
   in upstream Linux, post a backport of the patch that conforms to the patch requirements below.

2. LESS GOOD: Develop your patches out-of-tree (from an upstream Linux point-of-view). Unless these are
   fixing an Android-specific bug, these are very unlikely to be accepted unless they have been
   coordinated with kernel-team@android.com. If you want to proceed, post a patch that conforms to the
   patch requirements below.

# Common Kernel patch requirements

- All patches must conform to the Linux kernel coding standards and pass `script/checkpatch.pl`
- Patches shall not break gki_defconfig or allmodconfig builds for arm, arm64, x86, x86_64 architectures
(see  https://source.android.com/setup/build/building-kernels)
- If the patch is not merged from an upstream branch, the subject must be tagged with the type of patch:
`UPSTREAM:`, `BACKPORT:`, `FROMGIT:`, `FROMLIST:`, or `ANDROID:`.
- All patches must have a `Change-Id:` tag (see https://gerrit-review.googlesource.com/Documentation/user-changeid.html)
- If an Android bug has been assigned, there must be a `Bug:` tag.
- All patches must have a `Signed-off-by:` tag by the author and the submitter

Additional requirements are listed below based on patch type

## Requirements for backports from mainline Linux: `UPSTREAM:`, `BACKPORT:`

- If the patch is a cherry-pick from Linux mainline with no changes at all
    - tag the patch subject with `UPSTREAM:`.
    - add upstream commit information with a `(cherry-picked from ...)` line
    - Example:
        - if the upstream commit message is
```
        important patch from upstream

        This is the detailed description of the important patch

        Signed-off-by: Fred Jones <fred.jones@foo.org>
```
        - then Joe Smith would upload the patch for the common kernel as
```
        UPSTREAM: important patch from upstream

        This is the detailed description of the important patch

        Signed-off-by: Fred Jones <fred.jones@foo.org>

        Bug: 135791357
        Change-Id: I4caaaa566ea080fa148c5e768bb1a0b6f7201c01
        (cherry-picked from c31e73121f4c1ec41143423ac6ce3ce6dafdcec1)
        Signed-off-by: Joe Smith <joe.smith@foo.org>
```

- If the patch requires any changes from the upstream version, tag the patch with `BACKPORT:`
instead of `UPSTREAM:`.
    - use the same tags as `UPSTREAM:`
    - add comments about the changes under the `(cherry-picked from ...)` line
    - Example:
```
        BACKPORT: important patch from upstream

        This is the detailed description of the important patch

        Signed-off-by: Fred Jones <fred.jones@foo.org>

        Bug: 135791357
        Change-Id: I4caaaa566ea080fa148c5e768bb1a0b6f7201c01
        (cherry-picked from c31e73121f4c1ec41143423ac6ce3ce6dafdcec1)
        [ Resolved minor conflict in drivers/foo/bar.c ]
        Signed-off-by: Joe Smith <joe.smith@foo.org>
```

## Requirements for other backports: `FROMGIT:`, `FROMLIST:`,

- If the patch has been merged into an upstream maintainer tree, but has not yet
been merged into Linux mainline
    - tag the patch subject with `FROMGIT:`
    - add info on where the patch came from as `(cherry picked from commit <sha1> <repo> <branch>)`. This
must be a stable maintainer branch (not rebased, so don't use `linux-next` for example).
    - if changes were required, use `BACKPORT: FROMGIT:`
    - Example:
        - if the commit message in the maintainer tree is
```
        important patch from upstream

        This is the detailed description of the important patch

        Signed-off-by: Fred Jones <fred.jones@foo.org>
```
        - then Joe Smith would upload the patch for the common kernel as
```
        FROMGIT: important patch from upstream

        This is the detailed description of the important patch

        Signed-off-by: Fred Jones <fred.jones@foo.org>

        Bug: 135791357
        (cherry picked from commit 878a2fd9de10b03d11d2f622250285c7e63deace
         https://git.kernel.org/pub/scm/linux/kernel/git/foo/bar.git test-branch)
        Change-Id: I4caaaa566ea080fa148c5e768bb1a0b6f7201c01
        Signed-off-by: Joe Smith <joe.smith@foo.org>
```


- If the patch has been submitted to LKML, but not accepted into any maintainer tree
    - tag the patch subject with `FROMLIST:`
    - add a `Link:` tag with a link to the submittal on lore.kernel.org
    - if changes were required, use `BACKPORT: FROMLIST:`
    - Example:
```
        FROMLIST: important patch from upstream

        This is the detailed description of the important patch

        Signed-off-by: Fred Jones <fred.jones@foo.org>

        Bug: 135791357
        Link: https://lore.kernel.org/lkml/20190619171517.GA17557@someone.com/
        Change-Id: I4caaaa566ea080fa148c5e768bb1a0b6f7201c01
        Signed-off-by: Joe Smith <joe.smith@foo.org>
```

## Requirements for Android-specific patches: `ANDROID:`

- If the patch is fixing a bug to Android-specific code
    - tag the patch subject with `ANDROID:`
    - add a `Fixes:` tag that cites the patch with the bug
    - Example:
```
        ANDROID: fix android-specific bug in foobar.c

        This is the detailed description of the important fix

        Fixes: 1234abcd2468 ("foobar: add cool feature")
        Change-Id: I4caaaa566ea080fa148c5e768bb1a0b6f7201c01
        Signed-off-by: Joe Smith <joe.smith@foo.org>
```

- If the patch is a new feature
    - tag the patch subject with `ANDROID:`
    - add a `Bug:` tag with the Android bug (required for android-specific features)

# Vibrator driver for HHG device
## How to merge the driver into kernel source tree

 1. Copy \${this_project}/drivers/hid/hid-aksys.c into \${your_kernel_root}/drivers/hid/

 2. Compare and merge \${this_project}/drivers/hid/hid-ids.h into \${your_kernel_root}/drivers/hid/hid-ids.h :
 Add the following code before the last line of this file

    ```c
		#define USB_VENDER_ID_QUALCOMM  0x0a12
		#define USB_VENDER_ID_TEMP_HHG_AKSY 0x1234
		#define USB_PRODUCT_ID_AKSYS_HHG  0x1000
    ```

 3. Merge \${this_project}/drivers/hid/Kconfig into \${your_kernel_root}/drivers/hid/Kconfig :
Add the following code before the last line of this file

		config HID_AKSYS_QRD
    		tristate "AKSys gamepad USB adapter support"
    		depends on HID
    		---help---
    		Support for AKSys gamepad USB adapter

    	config AKSYS_QRD_FF
    		bool "AKSys gamepad USB adapter force feedback support"
    		depends on HID_AKSYS_QRD
    		select INPUT_FF_MEMLESS
    		---help---
    		Say Y here if you have a AKSys gamepad USB adapter and want to
    		enable force feedback support for it.

 4. Merge \${this_project}/drivers/hid/Makefile into \${your_kernel_root}/drivers/hid/Makefile :
 Add the following code at the end of this file

		obj-$(CONFIG_HID_AKSYS_QRD)	+= hid-aksys.o

 5. Modify your kernel's default build configuration file. Add the following two lines:

        CONFIG_HID_AKSYS_QRD=m
        CONFIG_AKSYS_QRD_FF=y
