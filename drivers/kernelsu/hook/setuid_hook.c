#include <linux/compiler.h>
#include <linux/version.h>
#include <linux/sched/signal.h>
#include <linux/slab.h>
#include <linux/task_work.h>
#include <linux/thread_info.h>
#include <linux/seccomp.h>
#include <linux/printk.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/uidgid.h>

#include "policy/allowlist.h"
#include "hook/setuid_hook.h"
#include "klog.h" // IWYU pragma: keep
#include "manager/manager_identity.h"
#include "infra/seccomp_cache.h"
#include "supercall/supercall.h"
#include "hook/hook_manager.h"
#include "feature/kernel_umount.h"
#include "compat/kernel_compat.h"
#ifdef CONFIG_KSU_SUSFS
#include <linux/susfs_def.h>
#include <linux/workqueue.h>
#include "selinux/selinux.h"
#endif // #ifdef CONFIG_KSU_SUSFS

extern void disable_seccomp(struct task_struct *tsk);

#ifdef CONFIG_KSU_SUSFS
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

extern u32 susfs_zygote_sid;
extern struct work_struct susfs_extra_works;

#ifdef CONFIG_KSU_SUSFS_TRY_UMOUNT
extern void susfs_try_umount(uid_t uid);
#endif // #ifdef CONFIG_KSU_SUSFS_TRY_UMOUNT

/*
 * Re-run the sus_path loop after a zygote spawned process changed its uid.
 * SUSFS v2.3.0 implements this through the susfs_extra_works workqueue.
 */
static inline void ksu_handle_extra_susfs_work(void)
{
    if (work_pending(&susfs_extra_works))
        return;

    schedule_work(&susfs_extra_works);
}
#endif // #ifdef CONFIG_KSU_SUSFS

int ksu_handle_setresuid(uid_t old_uid, uid_t new_uid)
{
    // we rely on the fact that zygote always call setresuid(3) with same uids

#ifdef CONFIG_KSU_SUSFS
    // We only interest in process spawned by zygote
    if (!susfs_is_sid_equal(current_cred(), susfs_zygote_sid)) {
        return 0;
    }

#ifdef CONFIG_KSU_SUSFS_SUS_MOUNT
    // Check if spawned process is isolated service first, and force to do umount if so
    if (is_zygote_isolated_service_uid(new_uid)) {
        goto do_umount;
    }
#endif // #ifdef CONFIG_KSU_SUSFS_SUS_MOUNT
#endif // #ifdef CONFIG_KSU_SUSFS

    pr_info("handle_setresuid from %d to %d\n", old_uid, new_uid);

    if (unlikely(is_uid_manager(new_uid))) {

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
        if (current->seccomp.mode == SECCOMP_MODE_FILTER && current->seccomp.filter) {
            ksu_seccomp_allow_cache(current->seccomp.filter, __NR_reboot);
        }
#else
		disable_seccomp(current);
#endif

#ifdef KSU_KPROBES_HOOK
        ksu_set_task_tracepoint_flag(current);
#endif

        pr_info("install fd for manager: %d\n", new_uid);
        ksu_install_fd();
        return 0;
    }

#ifdef CONFIG_KSU_SUSFS
    // Check if spawned process is normal user app and needs to be umounted
    if (likely(is_zygote_normal_app_uid(new_uid) && ksu_uid_should_umount(new_uid))) {
        goto do_umount;
    }
#endif // #ifdef CONFIG_KSU_SUSFS

    if (ksu_is_allow_uid_for_current(new_uid)) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
        if (current->seccomp.mode == SECCOMP_MODE_FILTER && current->seccomp.filter) {
            ksu_seccomp_allow_cache(current->seccomp.filter, __NR_reboot);
        }
#else
		disable_seccomp(current);
#endif

#ifdef KSU_KPROBES_HOOK
		ksu_set_task_tracepoint_flag(current);
#endif
	} else {
#ifdef KSU_KPROBES_HOOK
		ksu_clear_task_tracepoint_flag_if_needed(current);
#endif
    }

#ifdef CONFIG_KSU_SUSFS
    return 0;

do_umount:
    // Handle kernel umount
#ifndef CONFIG_KSU_SUSFS_TRY_UMOUNT
    ksu_handle_umount(old_uid, new_uid);
#else
    susfs_try_umount(new_uid);
#endif // #ifndef CONFIG_KSU_SUSFS_TRY_UMOUNT

    // Handle extra susfs work
    ksu_handle_extra_susfs_work();

    susfs_set_current_proc_umounted();

    return 0;
#else
    // Handle kernel umount
    ksu_handle_umount(old_uid, new_uid);

    return 0;
#endif // #ifdef CONFIG_KSU_SUSFS
}

void __init ksu_setuid_hook_init(void)
{
	ksu_kernel_umount_init();
}

void __exit ksu_setuid_hook_exit(void)
{
	pr_info("ksu_core_exit\n");
	ksu_kernel_umount_exit();
}