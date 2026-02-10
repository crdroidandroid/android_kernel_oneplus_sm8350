#include <linux/version.h>
#include <linux/fs.h>
#include <linux/nsproxy.h>
#include <linux/sched/task.h>
#include <linux/uaccess.h>
#include <linux/filter.h>
#include <linux/seccomp.h>
#include <linux/uidgid.h>
#include <asm/unistd.h>
#include "klog.h" // IWYU pragma: keep
#include "seccomp_cache.h"
#include "manager.h"

/*
 * Seccomp action cache (SECCOMP_ARCH_NATIVE_NR) was added in kernel 5.15+.
 * For older kernels, these functions are no-ops since the cache doesn't exist.
 */
#ifdef SECCOMP_ARCH_NATIVE_NR

struct action_cache {
	DECLARE_BITMAP(allow_native, SECCOMP_ARCH_NATIVE_NR);
#ifdef SECCOMP_ARCH_COMPAT
	DECLARE_BITMAP(allow_compat, SECCOMP_ARCH_COMPAT_NR);
#endif
};

struct seccomp_filter {
	refcount_t refs;
	refcount_t users;
	bool log;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
	bool wait_killable_recv;
#endif
	struct action_cache cache;
	struct seccomp_filter *prev;
	struct bpf_prog *prog;
	struct notification *notif;
	struct mutex notify_lock;
	wait_queue_head_t wqh;
};

void ksu_seccomp_clear_cache(struct seccomp_filter *filter, int nr)
{
    if (!filter) {
        return;
    }

    if (nr >= 0 && nr < SECCOMP_ARCH_NATIVE_NR) {
        clear_bit(nr, filter->cache.allow_native);
    }

#ifdef SECCOMP_ARCH_COMPAT
    if (nr >= 0 && nr < SECCOMP_ARCH_COMPAT_NR) {
        clear_bit(nr, filter->cache.allow_compat);
    }
#endif
}

void ksu_seccomp_allow_cache(struct seccomp_filter *filter, int nr)
{
    if (!filter) {
        return;
    }

    if (nr >= 0 && nr < SECCOMP_ARCH_NATIVE_NR) {
        set_bit(nr, filter->cache.allow_native);
    }

#ifdef SECCOMP_ARCH_COMPAT
    if (nr >= 0 && nr < SECCOMP_ARCH_COMPAT_NR) {
        set_bit(nr, filter->cache.allow_compat);
    }
#endif
}

#else /* !SECCOMP_ARCH_NATIVE_NR - kernel 5.4 and older */

void ksu_seccomp_clear_cache(struct seccomp_filter *filter, int nr)
{
    /* no-op: seccomp action cache not available in this kernel */
}

void ksu_seccomp_allow_cache(struct seccomp_filter *filter, int nr)
{
    /* no-op: seccomp action cache not available in this kernel */
}

#endif /* SECCOMP_ARCH_NATIVE_NR */

/*
 * Seccomp bypass for __NR_reboot on kernel 5.4.
 *
 * KernelSU uses reboot(0xDEADBEEF, 0xCAFEBABE) as a supercall to install
 * the anonymous inode fd. On kernel 5.15+, ksu_seccomp_allow_cache() modifies
 * the seccomp bitmap to allow __NR_reboot. On kernel 5.4, the bitmap doesn't
 * exist and the cache functions are no-ops, so Android's seccomp BPF filter
 * kills the process with SIGSYS before the reboot handler can intercept it.
 *
 * This function is called from __seccomp_filter() in kernel/seccomp.c to
 * allow __NR_reboot through for KSU Manager and root-granted processes.
 */
bool ksu_seccomp_check_reboot_syscall(int this_syscall)
{
	if (this_syscall != __NR_reboot)
		return false;

	if (current_uid().val == 0)
		return true;

	if (is_manager())
		return true;

	return ksu_is_allow_uid_for_current(current_uid().val);
}
