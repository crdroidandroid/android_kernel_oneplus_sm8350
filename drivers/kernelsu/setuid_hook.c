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

void susfs_on_post_fs_data(void)
{
	pr_info("susfs: post_fs_data triggered (v2.0.0 + auto-init)\n");

#ifdef CONFIG_KSU_SUSFS_SUS_PATH
	/* Auto-hide /system/addon.d - not present on stock Android */
	susfs_auto_add_sus_path_internal("/system/addon.d");
#endif
}

static const char susfs_clean_hosts_content[] =
	"127.0.0.1       localhost\n"
	"::1             ip6-localhost\n";

void susfs_on_module_mounted(void)
{
	struct path p;
	struct kstat kst;
	int err;

	pr_info("susfs: on_module_mounted auto-init\n");

	/* Check if hosts file is abnormally large (> 1KB means adblock list) */
	err = kern_path("/system/etc/hosts", LOOKUP_FOLLOW, &p);
	if (err)
		return;

	err = vfs_getattr(&p, &kst, STATX_SIZE, AT_STATX_SYNC_AS_STAT);
	path_put(&p);
	if (err)
		return;

	if (kst.size > 1024) {
		pr_info("susfs: hosts file is %lld bytes, setting up auto-hide\n", kst.size);

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
	}
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

void ksu_setuid_hook_init(void)
{
    ksu_kernel_umount_init();
}

void ksu_setuid_hook_exit(void)
{
    pr_info("ksu_core_exit\n");
    ksu_kernel_umount_exit();
}
