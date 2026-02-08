/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_TASK_WORK_H
#define _LINUX_TASK_WORK_H

#include <linux/list.h>
#include <linux/sched.h>

/*
 * TWA_RESUME compatibility for kernel 5.4
 * In kernel 5.9+, task_work_add uses enum task_work_notify_mode
 * In kernel 5.4, it uses a bool parameter (true = notify)
 * TWA_RESUME means "run when returning to userspace" = true
 */
#ifndef TWA_RESUME
#define TWA_RESUME true
#endif
#ifndef TWA_SIGNAL
#define TWA_SIGNAL true
#endif

typedef void (*task_work_func_t)(struct callback_head *);

static inline void
init_task_work(struct callback_head *twork, task_work_func_t func)
{
	twork->func = func;
}

int task_work_add(struct task_struct *task, struct callback_head *twork, bool);
struct callback_head *task_work_cancel(struct task_struct *, task_work_func_t);
void task_work_run(void);

static inline void exit_task_work(struct task_struct *task)
{
	task_work_run();
}

#endif	/* _LINUX_TASK_WORK_H */
