#include <linux/compiler.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/task_work.h>
#include <linux/thread_info.h>
#include <linux/seccomp.h>
#include <linux/printk.h>
#include <linux/sched.h>
#include <linux/sched/signal.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/uidgid.h>
#ifdef CONFIG_KSU_SUSFS
#include <linux/susfs_def.h>
#include <linux/susfs.h>
#include <linux/namei.h>
#include <linux/fs.h>
#include <linux/stat.h>
#include <linux/workqueue.h>
#include <linux/vmalloc.h>
#endif // #ifdef CONFIG_KSU_SUSFS

#include "allowlist.h"
#include "app_profile.h"
#include "setuid_hook.h"
#include "klog.h" // IWYU pragma: keep
#include "ksud.h"
#include "manager.h"
#include "selinux/selinux.h"
#include "seccomp_cache.h"
#include "supercalls.h"
#ifndef CONFIG_KSU_SUSFS
#include "syscall_hook_manager.h"
#endif // #ifndef CONFIG_KSU_SUSFS
#include "kernel_umount.h"

#ifdef CONFIG_KSU_SUSFS
bool susfs_is_allow_su(void)
{
	if (is_manager()) {
		return true;
	}
	return ksu_is_allow_uid_for_current(current_uid().val);
}

bool susfs_is_boot_completed_triggered = false;

extern u32 susfs_zygote_sid;

#ifdef CONFIG_KSU_SUSFS_SUS_PATH
extern void susfs_run_sus_path_loop(uid_t uid);
#endif // #ifdef CONFIG_KSU_SUSFS_SUS_PATH

#ifdef CONFIG_KSU_SUSFS_ENABLE_LOG
extern bool susfs_is_log_enabled __read_mostly;
#endif // #ifdef CONFIG_KSU_SUSFS_ENABLE_LOG

#ifdef CONFIG_KSU_SUSFS_SUS_MOUNT
extern bool susfs_hide_sus_mnts_for_non_su_procs;
extern void susfs_reorder_mnt_id(void);
#endif // #ifdef CONFIG_KSU_SUSFS_SUS_MOUNT

#ifdef CONFIG_KSU_SUSFS_OPEN_REDIRECT
static bool susfs_sepolicy_redirect_done = false;

static void susfs_try_setup_sepolicy_redirect(const char *caller)
{
	static const char * const sepolicy_paths[] = {
		"/system_ext/etc/selinux/system_ext_sepolicy.cil",
		NULL
	};
	const char *src_path;
	struct file *filp;
	struct kstat kst;
	struct path p;
	char *buf, *scan;
	loff_t pos = 0;
	ssize_t nread;
	int err, i;

	if (susfs_sepolicy_redirect_done)
		return;

	for (i = 0; sepolicy_paths[i]; i++) {
		src_path = sepolicy_paths[i];

		err = kern_path(src_path, LOOKUP_FOLLOW, &p);
		if (err) {
			pr_info("susfs: %s: sepolicy '%s' not found (err=%d)\n",
				caller, src_path, err);
			continue;
		}

		err = vfs_getattr(&p, &kst, STATX_SIZE, AT_STATX_SYNC_AS_STAT);
		path_put(&p);
		if (err || kst.size == 0 || kst.size > (16 * 1024 * 1024)) {
			pr_info("susfs: %s: sepolicy '%s' size check failed (err=%d, size=%lld)\n",
				caller, src_path, err, kst.size);
			continue;
		}

		filp = filp_open(src_path, O_RDONLY, 0);
		if (IS_ERR(filp)) {
			pr_warn("susfs: %s: failed to open '%s': %ld\n",
				caller, src_path, PTR_ERR(filp));
			continue;
		}

		buf = vmalloc(kst.size);
		if (!buf) {
			filp_close(filp, NULL);
			pr_warn("susfs: %s: vmalloc(%lld) failed for sepolicy\n",
				caller, kst.size);
			continue;
		}

		nread = kernel_read(filp, buf, kst.size, &pos);
		filp_close(filp, NULL);

		if (nread != kst.size) {
			pr_warn("susfs: %s: read %zd != %lld for '%s'\n",
				caller, nread, kst.size, src_path);
			vfree(buf);
			continue;
		}

		/* Replace "lineage" -> "org_ext" (same 7-char length, preserves file size) */
		for (scan = buf; scan <= buf + nread - 7; scan++) {
			if (scan[0] == 'l' && !memcmp(scan, "lineage", 7))
				memcpy(scan, "org_ext", 7);
			else if (scan[0] == 'L' && !memcmp(scan, "Lineage", 7))
				memcpy(scan, "Org_ext", 7);
		}

		err = susfs_create_file_with_content(
			"/data/adb/.susfs/system_ext_sepolicy_clean.cil",
			buf, nread);
		vfree(buf);

		if (err) {
			pr_warn("susfs: %s: failed to write clean sepolicy: %d\n",
				caller, err);
			continue;
		}

		susfs_auto_add_open_redirect_internal(
			src_path,
			"/data/adb/.susfs/system_ext_sepolicy_clean.cil");

		pr_info("susfs: %s: sepolicy redirect set for '%s'\n",
			caller, src_path);
		susfs_sepolicy_redirect_done = true;
		break;
	}
}
#endif /* CONFIG_KSU_SUSFS_OPEN_REDIRECT */

void susfs_on_post_fs_data(void)
{
	pr_info("susfs: post_fs_data triggered (v2.0.0 + auto-init)\n");

#ifdef CONFIG_KSU_SUSFS_SUS_PATH
	/* Auto-hide /system/addon.d - not present on stock Android */
	susfs_auto_add_sus_path_internal("/system/addon.d");
#endif

#ifdef CONFIG_KSU_SUSFS_OPEN_REDIRECT
	susfs_try_setup_sepolicy_redirect("post_fs_data");
#endif

#ifdef CONFIG_KSU_SUSFS_SUS_MAP
	/* Auto-hide vendor lineage binaries/libs from proc maps */
	susfs_auto_add_sus_map_internal("/vendor/bin/hw/vendor.lineage.health-service.default");
	susfs_auto_add_sus_map_internal("/vendor/bin/hw/vendor.lineage.livedisplay-service.oplus");
	susfs_auto_add_sus_map_internal("/vendor/bin/hw/vendor.lineage.powershare-service.oplus");
	susfs_auto_add_sus_map_internal("/vendor/bin/hw/vendor.lineage.touch-service.oplus");
	susfs_auto_add_sus_map_internal("/vendor/lib64/vendor.lineage.health-V2-ndk.so");
	susfs_auto_add_sus_map_internal("/vendor/lib64/vendor.lineage.livedisplay-V1-ndk.so");
	susfs_auto_add_sus_map_internal("/vendor/lib64/vendor.lineage.powershare-V1-ndk.so");
	susfs_auto_add_sus_map_internal("/vendor/lib64/vendor.lineage.touch-V1-ndk.so");
#endif
}

static const char susfs_clean_hosts_content[] =
	"127.0.0.1       localhost\n"
	"::1             ip6-localhost\n";

static struct delayed_work susfs_hosts_delayed_work;
static bool susfs_hosts_hide_done = false;

static void susfs_try_setup_hosts_hide(const char *caller)
{
	struct path p;
	struct kstat kst;
	int err;

	if (susfs_hosts_hide_done)
		return;

	/* Check if hosts file is abnormally large (> 1KB means adblock list) */
	err = kern_path("/system/etc/hosts", LOOKUP_FOLLOW, &p);
	if (err) {
		pr_info("susfs: %s: hosts path not found (err=%d)\n", caller, err);
		return;
	}

	err = vfs_getattr(&p, &kst, STATX_SIZE, AT_STATX_SYNC_AS_STAT);
	path_put(&p);
	if (err) {
		pr_warn("susfs: %s: vfs_getattr failed (err=%d)\n", caller, err);
		return;
	}

	if (kst.size <= 1024) {
		pr_info("susfs: %s: hosts file is %lld bytes, no hiding needed\n",
			caller, kst.size);
		return;
	}

	pr_info("susfs: %s: hosts file is %lld bytes, setting up auto-hide\n",
		caller, kst.size);

	/* 1. Create a clean hosts file for redirection */
	err = susfs_create_file_with_content(
		"/data/adb/.susfs/hosts_clean",
		susfs_clean_hosts_content,
		sizeof(susfs_clean_hosts_content) - 1);
	if (err) {
		pr_warn("susfs: failed to create clean hosts file: %d\n", err);
		return;
	}

#ifdef CONFIG_KSU_SUSFS_SUS_KSTAT
	/* 2. Spoof stat to show small file size */
	susfs_auto_add_sus_kstat_internal(
		"/system/etc/hosts",
		(long long)(sizeof(susfs_clean_hosts_content) - 1),
		8);
#endif

#ifdef CONFIG_KSU_SUSFS_OPEN_REDIRECT
	/* 3. Redirect file reads to clean hosts */
	susfs_auto_add_open_redirect_internal(
		"/system/etc/hosts",
		"/data/adb/.susfs/hosts_clean");
#endif

	susfs_hosts_hide_done = true;
}

static void susfs_hosts_check_work_fn(struct work_struct *work)
{
	susfs_try_setup_hosts_hide("delayed_check");
}

void susfs_schedule_hosts_check(void)
{
	if (susfs_hosts_hide_done)
		return;
	INIT_DELAYED_WORK(&susfs_hosts_delayed_work, susfs_hosts_check_work_fn);
	schedule_delayed_work(&susfs_hosts_delayed_work, msecs_to_jiffies(30000));
	pr_info("susfs: scheduled delayed hosts check (30s)\n");
}

void susfs_on_module_mounted(void)
{
	pr_info("susfs: on_module_mounted auto-init\n");

	susfs_try_setup_hosts_hide("on_module_mounted");
}

static inline bool is_zygote_isolated_service_uid(uid_t uid)
{
	uid %= 100000;
	return (uid >= 99000 && uid < 100000);
}

static inline bool is_zygote_normal_app_uid(uid_t uid)
{
	uid %= 100000;
	return (uid >= 10000 && uid < 19999);
}
#endif // #ifdef CONFIG_KSU_SUSFS

static void ksu_install_manager_fd_tw_func(struct callback_head *cb)
{
    ksu_install_fd();
    kfree(cb);
}

#ifndef CONFIG_KSU_SUSFS
int ksu_handle_setresuid(uid_t ruid, uid_t euid, uid_t suid)
{
    // we rely on the fact that zygote always call setresuid(3) with same uids
    uid_t new_uid = ruid;
    uid_t old_uid = current_uid().val;

    pr_info("handle_setresuid from %d to %d\n", old_uid, new_uid);

    if (likely(ksu_is_manager_appid_valid()) &&
        unlikely(ksu_get_manager_appid() == new_uid % PER_USER_RANGE)) {
        spin_lock_irq(&current->sighand->siglock);
        ksu_seccomp_allow_cache(current->seccomp.filter, __NR_reboot);
        ksu_set_task_tracepoint_flag(current);
        spin_unlock_irq(&current->sighand->siglock);

        pr_info("install fd for manager: %d\n", new_uid);
        struct callback_head *cb = kzalloc(sizeof(*cb), GFP_ATOMIC);
        if (!cb)
            return 0;
        cb->func = ksu_install_manager_fd_tw_func;
        if (task_work_add(current, cb, TWA_RESUME)) {
            kfree(cb);
            pr_warn("install manager fd add task_work failed\n");
        }
        return 0;
    }

    if (ksu_is_allow_uid_for_current(new_uid)) {
        if (current->seccomp.mode == SECCOMP_MODE_FILTER &&
            current->seccomp.filter) {
            spin_lock_irq(&current->sighand->siglock);
            ksu_seccomp_allow_cache(current->seccomp.filter, __NR_reboot);
            spin_unlock_irq(&current->sighand->siglock);
        }
        ksu_set_task_tracepoint_flag(current);
    } else {
        ksu_clear_task_tracepoint_flag_if_needed(current);
    }

    // Handle kernel umount
    ksu_handle_umount(old_uid, new_uid);

    return 0;
}
#else
int ksu_handle_setresuid(uid_t ruid, uid_t euid, uid_t suid){
    // we rely on the fact that zygote always call setresuid(3) with same uids
    uid_t new_uid = ruid;
    uid_t old_uid = current_uid().val;

    // We only interest in process spawned by zygote
    // Skip check if susfs_zygote_sid not yet initialized (0)
    if (susfs_zygote_sid && !susfs_is_sid_equal(current_cred(), susfs_zygote_sid)) {
        return 0;
    }

#ifdef CONFIG_KSU_SUSFS_SUS_MOUNT
    // Check if spawned process is isolated service first, and force to do umount if so
    if (is_zygote_isolated_service_uid(new_uid)) {
        goto do_umount;
    }
#endif // #ifdef CONFIG_KSU_SUSFS_SUS_MOUNT

    // - Since ksu maanger app uid is excluded in allow_list_arr, so ksu_uid_should_umount(manager_uid)
    //   will always return true, that's why we need to explicitly check if new_uid belongs to
    //   ksu manager
    if (ksu_get_manager_appid() == new_uid % PER_USER_RANGE) {
        spin_lock_irq(&current->sighand->siglock);
        ksu_seccomp_allow_cache(current->seccomp.filter, __NR_reboot);
        spin_unlock_irq(&current->sighand->siglock);

        pr_info("install fd for manager: %d\n", new_uid);
        struct callback_head *cb = kzalloc(sizeof(*cb), GFP_ATOMIC);
        if (!cb)
            return 0;
        cb->func = ksu_install_manager_fd_tw_func;
        if (task_work_add(current, cb, TWA_RESUME)) {
            kfree(cb);
            pr_warn("install manager fd add task_work failed\n");
        }
        return 0;
    }

    // Check if spawned process is normal user app and needs to be umounted
    if (likely(is_zygote_normal_app_uid(new_uid) && ksu_uid_should_umount(new_uid))) {
        goto do_umount;
    }

    if (ksu_is_allow_uid_for_current(new_uid)) {
        if (current->seccomp.mode == SECCOMP_MODE_FILTER &&
            current->seccomp.filter) {
            spin_lock_irq(&current->sighand->siglock);
            ksu_seccomp_allow_cache(current->seccomp.filter, __NR_reboot);
            spin_unlock_irq(&current->sighand->siglock);
        }
    }

    return 0;

do_umount:
    // Handle kernel umount
    ksu_handle_umount(old_uid, new_uid);

#ifdef CONFIG_KSU_SUSFS_SUS_MOUNT
    // We can reorder the mnt_id now after all sus mounts are umounted
    susfs_reorder_mnt_id();
#endif // #ifdef CONFIG_KSU_SUSFS_SUS_MOUNT

#ifdef CONFIG_KSU_SUSFS_SUS_PATH
    susfs_run_sus_path_loop(new_uid);
#endif // #ifdef CONFIG_KSU_SUSFS_SUS_PATH

    susfs_set_current_proc_umounted();

    return 0;
}
#endif // #ifndef CONFIG_KSU_SUSFS

int ksu_setuid_hook_init(void)
{
    ksu_kernel_umount_init();
    return 0;
}

void ksu_setuid_hook_exit(void)
{
    pr_info("ksu_core_exit\n");
    ksu_kernel_umount_exit();
}
