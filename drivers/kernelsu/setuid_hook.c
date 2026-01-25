#include <linux/compiler.h>
#include <linux/sched/signal.h>
#include <linux/slab.h>
#include <linux/task_work.h>
#include <linux/thread_info.h>
#include <linux/seccomp.h>
#include <linux/bpf.h>
#include <linux/capability.h>
#include <linux/cred.h>
#include <linux/dcache.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/init_task.h>
#include <linux/kernel.h>
#include <linux/kprobes.h>
#include <linux/mm.h>
#include <linux/mount.h>
#include <linux/namei.h>
#include <linux/nsproxy.h>
#include <linux/path.h>
#include <linux/printk.h>
#include <linux/sched.h>
#include <linux/stddef.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/uidgid.h>
#include <linux/version.h>

#ifdef CONFIG_KSU_SUSFS
#include <linux/susfs.h>
#endif

#include "allowlist.h"
#include "setuid_hook.h"
#include "feature.h"
#include "klog.h" // IWYU pragma: keep
#include "manager.h"
#include "selinux/selinux.h"
#include "seccomp_cache.h"
#include "supercalls.h"
#include "syscall_hook_manager.h"
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
extern bool susfs_is_mnt_devname_ksu(struct path *path);

#ifdef CONFIG_KSU_SUSFS_SUS_PATH
extern void susfs_run_sus_path_loop(uid_t uid);
#endif

#ifdef CONFIG_KSU_SUSFS_ENABLE_LOG
extern bool susfs_is_log_enabled __read_mostly;
#endif

#ifdef CONFIG_KSU_SUSFS_SUS_MOUNT
static bool susfs_is_umount_for_zygote_system_process_enabled = false;
static bool susfs_is_umount_for_zygote_iso_service_enabled = false;
extern bool susfs_hide_sus_mnts_for_all_procs;
extern void susfs_reorder_mnt_id(void);
#endif

#ifdef CONFIG_KSU_SUSFS_AUTO_ADD_SUS_BIND_MOUNT
extern bool susfs_is_auto_add_sus_bind_mount_enabled;
#endif

#ifdef CONFIG_KSU_SUSFS_AUTO_ADD_SUS_KSU_DEFAULT_MOUNT
extern bool susfs_is_auto_add_sus_ksu_default_mount_enabled;
#endif

#ifdef CONFIG_KSU_SUSFS_AUTO_ADD_TRY_UMOUNT_FOR_BIND_MOUNT
extern bool susfs_is_auto_add_try_umount_for_bind_mount_enabled;
#endif

#ifdef CONFIG_KSU_SUSFS_SUS_SU
extern bool susfs_is_sus_su_ready;
extern int susfs_sus_su_working_mode;
extern bool susfs_is_sus_su_hooks_enabled __read_mostly;
extern bool ksu_devpts_hook;
#endif

static inline bool is_some_system_uid(uid_t uid)
{
	return (uid >= 1000 && uid < 10000);
}

static inline bool is_zygote_isolated_service_uid(uid_t uid)
{
	return ((uid >= 90000 && uid < 100000) || (uid >= 1090000 && uid < 1100000));
}

static inline bool is_zygote_normal_app_uid(uid_t uid)
{
	return ((uid >= 10000 && uid < 19999) || (uid >= 1010000 && uid < 1019999));
}

void susfs_on_post_fs_data(void)
{
	struct path path;
#ifdef CONFIG_KSU_SUSFS_SUS_MOUNT
	if (!kern_path(DATA_ADB_UMOUNT_FOR_ZYGOTE_SYSTEM_PROCESS, 0, &path)) {
		susfs_is_umount_for_zygote_system_process_enabled = true;
		path_put(&path);
	}
	pr_info("susfs_is_umount_for_zygote_system_process_enabled: %d\n", susfs_is_umount_for_zygote_system_process_enabled);
#endif
#ifdef CONFIG_KSU_SUSFS_AUTO_ADD_SUS_BIND_MOUNT
	if (!kern_path(DATA_ADB_NO_AUTO_ADD_SUS_BIND_MOUNT, 0, &path)) {
		susfs_is_auto_add_sus_bind_mount_enabled = false;
		path_put(&path);
	}
	pr_info("susfs_is_auto_add_sus_bind_mount_enabled: %d\n", susfs_is_auto_add_sus_bind_mount_enabled);
#endif
#ifdef CONFIG_KSU_SUSFS_AUTO_ADD_SUS_KSU_DEFAULT_MOUNT
	if (!kern_path(DATA_ADB_NO_AUTO_ADD_SUS_KSU_DEFAULT_MOUNT, 0, &path)) {
		susfs_is_auto_add_sus_ksu_default_mount_enabled = false;
		path_put(&path);
	}
	pr_info("susfs_is_auto_add_sus_ksu_default_mount_enabled: %d\n", susfs_is_auto_add_sus_ksu_default_mount_enabled);
#endif
#ifdef CONFIG_KSU_SUSFS_AUTO_ADD_TRY_UMOUNT_FOR_BIND_MOUNT
	if (!kern_path(DATA_ADB_NO_AUTO_ADD_TRY_UMOUNT_FOR_BIND_MOUNT, 0, &path)) {
		susfs_is_auto_add_try_umount_for_bind_mount_enabled = false;
		path_put(&path);
	}
	pr_info("susfs_is_auto_add_try_umount_for_bind_mount_enabled: %d\n", susfs_is_auto_add_try_umount_for_bind_mount_enabled);
#endif
}
#endif // CONFIG_KSU_SUSFS

static bool ksu_enhanced_security_enabled = false;

static int enhanced_security_feature_get(u64 *value)
{
	*value = ksu_enhanced_security_enabled ? 1 : 0;
	return 0;
}

static int enhanced_security_feature_set(u64 value)
{
	bool enable = value != 0;
	ksu_enhanced_security_enabled = enable;
	pr_info("enhanced_security: set to %d\n", enable);
	return 0;
}

static const struct ksu_feature_handler enhanced_security_handler = {
	.feature_id = KSU_FEATURE_ENHANCED_SECURITY,
	.name = "enhanced_security",
	.get_handler = enhanced_security_feature_get,
	.set_handler = enhanced_security_feature_set,
};

static inline bool is_allow_su(void)
{
	if (is_manager()) {
		return true;
	}
	return ksu_is_allow_uid_for_current(current_uid().val);
}

int ksu_handle_setresuid(uid_t ruid, uid_t euid, uid_t suid)
{
	uid_t new_uid = ruid;
	uid_t old_uid = current_uid().val;

#ifdef CONFIG_KSU_DEBUG
	pr_info("handle_setresuid from %d to %d\n", old_uid, new_uid);
#endif

	// Enhanced security checks
	if (old_uid != 0 && ksu_enhanced_security_enabled) {
		if (unlikely(euid == 0)) {
			if (!is_ksu_domain()) {
				pr_warn("find suspicious EoP: %d %s, from %d to %d\n",
					current->pid, current->comm, old_uid, new_uid);
				force_sig(SIGKILL);
				return 0;
			}
		}
		if (is_appuid(old_uid)) {
			if (euid < current_euid().val && !ksu_is_allow_uid_for_current(old_uid)) {
				pr_warn("find suspicious EoP: %d %s, from %d to %d\n",
					current->pid, current->comm, old_uid, new_uid);
				force_sig(SIGKILL);
				return 0;
			}
		}
		return 0;
	}

	// Handle private space manager
	if (new_uid > PER_USER_RANGE && new_uid % PER_USER_RANGE == ksu_get_manager_uid()) {
		ksu_set_manager_uid(new_uid);
	}

	// Manager handling
	if (ksu_get_manager_uid() == new_uid) {
		pr_info("install fd for manager: %d\n", new_uid);
		ksu_install_fd();
		spin_lock_irq(&current->sighand->siglock);
		ksu_seccomp_allow_cache(current->seccomp.filter, __NR_reboot);
		ksu_set_task_tracepoint_flag(current);
		spin_unlock_irq(&current->sighand->siglock);
		return 0;
	}

	// Allowed UID handling
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

#ifdef CONFIG_KSU_SUSFS
	// SUSFS umount handling
	if (!ksu_module_mounted) {
		return 0;
	}

	// Only handle transition from root (zygote)
	if (old_uid != 0) {
		return 0;
	}

	// Check zygote SID
	if (!susfs_is_sid_equal(get_current_cred()->security, susfs_zygote_sid)) {
		return 0;
	}

	// Check isolated service
	if (is_zygote_isolated_service_uid(new_uid) && susfs_is_umount_for_zygote_iso_service_enabled) {
		goto do_susfs_umount;
	}

	// Check manager
	if (ksu_is_manager_uid_valid() &&
	    (new_uid % 1000000 == ksu_get_manager_uid())) {
		return 0;
	}

	// Check normal app
	if (likely(is_zygote_normal_app_uid(new_uid) && ksu_uid_should_umount(new_uid))) {
		goto do_susfs_umount;
	}

	// Check system process
	if (unlikely(is_some_system_uid(new_uid) && susfs_is_umount_for_zygote_system_process_enabled)) {
		goto do_susfs_umount;
	}

	return 0;

do_susfs_umount:
#ifdef CONFIG_KSU_SUSFS_TRY_UMOUNT
	susfs_try_umount(new_uid);
#endif
	// Also do standard ksu umount
	ksu_handle_umount(old_uid, new_uid);

	get_task_struct(current);

#ifdef CONFIG_KSU_SUSFS_SUS_MOUNT
	susfs_reorder_mnt_id();
#endif

	susfs_set_current_proc_umounted();

	put_task_struct(current);

#ifdef CONFIG_KSU_SUSFS_SUS_PATH
	susfs_run_sus_path_loop(new_uid);
#endif
	return 0;
#else
	// Standard KSU umount
	ksu_handle_umount(old_uid, new_uid);
	return 0;
#endif
}

void ksu_setuid_hook_init(void)
{
	ksu_kernel_umount_init();
	if (ksu_register_feature_handler(&enhanced_security_handler)) {
		pr_err("Failed to register enhanced security feature handler\n");
	}
}

void ksu_setuid_hook_exit(void)
{
	pr_info("ksu_setuid_hook_exit\n");
	ksu_kernel_umount_exit();
	ksu_unregister_feature_handler(KSU_FEATURE_ENHANCED_SECURITY);
}
