#include <linux/export.h>
#include <linux/fs.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#ifdef CONFIG_KSU_SUSFS
#include <linux/susfs.h>
#endif // #ifdef CONFIG_KSU_SUSFS

#include "allowlist.h"
#include "feature.h"
#include "klog.h" // IWYU pragma: keep
#include "throne_tracker.h"
#ifndef CONFIG_KSU_SUSFS
#include "syscall_hook_manager.h"
#else
#include "setuid_hook.h"
#include "sucompat.h"
#endif // #ifndef CONFIG_KSU_SUSFS
#include "ksud.h"
#include "supercalls.h"
#include "ksu.h"
#include "file_wrapper.h"

#ifdef CONFIG_KSU_SUSFS
extern int ksu_avc_spoof_init(void);
extern void ksu_avc_spoof_exit(void);
#endif

struct cred *ksu_cred;

int __init kernelsu_init(void)
{
	int ret;

#ifdef CONFIG_KSU_DEBUG
	pr_alert("*************************************************************");
	pr_alert("**     NOTICE NOTICE NOTICE NOTICE NOTICE NOTICE NOTICE    **");
	pr_alert("**                                                         **");
	pr_alert("**         You are running KernelSU in DEBUG mode          **");
	pr_alert("**                                                         **");
	pr_alert("**     NOTICE NOTICE NOTICE NOTICE NOTICE NOTICE NOTICE    **");
	pr_alert("*************************************************************");
#endif

	ksu_cred = prepare_creds();
	if (!ksu_cred) {
		pr_err("prepare cred failed!\n");
		return -ENOMEM;
	}

	ret = ksu_feature_init();
	if (ret)
		goto err_cred;

	ret = ksu_supercalls_init();
	if (ret)
		goto err_feature;

#ifndef CONFIG_KSU_SUSFS
	ret = ksu_syscall_hook_manager_init();
	if (ret)
		goto err_supercalls;
#else
	ret = ksu_setuid_hook_init();
	if (ret)
		goto err_supercalls;

	ret = ksu_sucompat_init();
	if (ret)
		goto err_setuid_hook;

	ret = ksu_avc_spoof_init();
	if (ret)
		goto err_sucompat;
#endif // #ifndef CONFIG_KSU_SUSFS

	ret = ksu_allowlist_init();
	if (ret)
		goto err_hooks;

	ret = ksu_throne_tracker_init();
	if (ret)
		goto err_allowlist;

#ifdef CONFIG_KSU_SUSFS
	ret = susfs_init();
	if (ret)
		goto err_throne;

	ret = susfs_start_sdcard_monitor_fn();
	if (ret)
		goto err_throne;
#endif // #ifdef CONFIG_KSU_SUSFS

#ifndef CONFIG_KSU_SUSFS
	ret = ksu_ksud_init();
	if (ret)
		goto err_throne;
#endif // #ifndef CONFIG_KSU_SUSFS

	ret = ksu_file_wrapper_init();
	if (ret)
		goto err_ksud;

#ifdef MODULE
#ifndef CONFIG_KSU_DEBUG
	kobject_del(&THIS_MODULE->mkobj.kobj);
#endif
#endif
	return 0;

err_ksud:
#ifndef CONFIG_KSU_SUSFS
	ksu_ksud_exit();
#endif
err_throne:
	ksu_throne_tracker_exit();
err_allowlist:
	ksu_allowlist_exit();
err_hooks:
#ifndef CONFIG_KSU_SUSFS
	ksu_syscall_hook_manager_exit();
#else
	ksu_avc_spoof_exit();
err_sucompat:
	ksu_sucompat_exit();
err_setuid_hook:
	ksu_setuid_hook_exit();
#endif // #ifndef CONFIG_KSU_SUSFS
err_supercalls:
	ksu_supercalls_exit();
err_feature:
	ksu_feature_exit();
err_cred:
	put_cred(ksu_cred);
	ksu_cred = NULL;
	return ret;
}

extern void ksu_observer_exit(void);
void kernelsu_exit(void)
{
	ksu_allowlist_exit();

	ksu_throne_tracker_exit();

	ksu_observer_exit();

#ifndef CONFIG_KSU_SUSFS
	ksu_ksud_exit();

	ksu_syscall_hook_manager_exit();
#else
	ksu_avc_spoof_exit();
#endif // #ifndef CONFIG_KSU_SUSFS

	ksu_supercalls_exit();

	ksu_feature_exit();

	if (ksu_cred) {
		put_cred(ksu_cred);
	}
}

module_init(kernelsu_init);
module_exit(kernelsu_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("weishu");
MODULE_DESCRIPTION("Android KernelSU");
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 13, 0)
MODULE_IMPORT_NS("VFS_internal_I_am_really_a_filesystem_and_am_NOT_a_driver");
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(5, 0, 0)
MODULE_IMPORT_NS(VFS_internal_I_am_really_a_filesystem_and_am_NOT_a_driver);
#endif
