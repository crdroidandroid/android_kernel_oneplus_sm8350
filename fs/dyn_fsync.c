// SPDX-License-Identifier: GPL-2.0
/*
 * Dynamic Fsync - battery optimization
 * Based on flar2's implementation for ElementalX
 *
 * When screen is off, fsync calls are skipped.
 * When screen turns on, all pending dirty data is flushed.
 */

#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/notifier.h>
#include <linux/syscalls.h>
#include <linux/msm_drm_notify.h>

bool dyn_fsync_active __read_mostly = true;
static int screen_on __read_mostly = 1;

bool dyn_fsync_fsync_enabled(void)
{
	if (dyn_fsync_active && !READ_ONCE(screen_on))
		return false;
	return true;
}

static int dyn_fsync_fb_notifier_callback(struct notifier_block *self,
					  unsigned long event, void *data)
{
	struct msm_drm_notifier *evdata = data;
	int *blank;

	if (event != MSM_DRM_EVENT_BLANK)
		return NOTIFY_OK;

	if (!evdata || !evdata->data)
		return NOTIFY_OK;

	blank = evdata->data;

	if (*blank == MSM_DRM_BLANK_UNBLANK) {
		WRITE_ONCE(screen_on, 1);
		if (dyn_fsync_active)
			ksys_sync();
	} else if (*blank == MSM_DRM_BLANK_POWERDOWN) {
		WRITE_ONCE(screen_on, 0);
	}

	return NOTIFY_OK;
}

static struct notifier_block dyn_fsync_notifier_block = {
	.notifier_call = dyn_fsync_fb_notifier_callback,
};

static ssize_t Fsync_enabled_show(struct kobject *kobj,
				  struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n", dyn_fsync_active ? 1 : 0);
}

static ssize_t Fsync_enabled_store(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;

	dyn_fsync_active = !!val;
	return count;
}

static struct kobj_attribute dyn_fsync_active_attr =
	__ATTR(Fsync_enabled, 0644, Fsync_enabled_show, Fsync_enabled_store);

static struct attribute *dyn_fsync_attrs[] = {
	&dyn_fsync_active_attr.attr,
	NULL,
};

static const struct attribute_group dyn_fsync_attr_group = {
	.attrs = dyn_fsync_attrs,
};

static struct kobject *dyn_fsync_kobj;

static int __init dyn_fsync_init(void)
{
	int ret;

	dyn_fsync_kobj = kobject_create_and_add("dyn_fsync", kernel_kobj);
	if (!dyn_fsync_kobj)
		return -ENOMEM;

	ret = sysfs_create_group(dyn_fsync_kobj, &dyn_fsync_attr_group);
	if (ret) {
		kobject_put(dyn_fsync_kobj);
		return ret;
	}

	ret = msm_drm_register_client(&dyn_fsync_notifier_block);
	if (ret) {
		sysfs_remove_group(dyn_fsync_kobj, &dyn_fsync_attr_group);
		kobject_put(dyn_fsync_kobj);
		return ret;
	}

	pr_info("dynamic fsync: enabled\n");
	return 0;
}

late_initcall(dyn_fsync_init);
