#include <linux/version.h>
#include <linux/cred.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/seq_file.h>
#include <linux/printk.h>
#include <linux/namei.h>
#include <linux/list.h>
#include <linux/init_task.h>
#include <linux/spinlock.h>
#include <linux/stat.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/fdtable.h>
#include <linux/statfs.h>
#include <linux/random.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/fsnotify_backend.h>
#include <linux/susfs.h>
#include "fuse/fuse_i.h"
#include "mount.h"

extern bool susfs_is_current_ksu_domain(void);

#ifdef CONFIG_KSU_SUSFS_ENABLE_LOG
bool susfs_is_log_enabled __read_mostly = true;
#define SUSFS_LOGI(fmt, ...) if (susfs_is_log_enabled) pr_info("susfs:[%u][%d][%s] " fmt, current_uid().val, current->pid, __func__, ##__VA_ARGS__)
#define SUSFS_LOGE(fmt, ...) if (susfs_is_log_enabled) pr_err("susfs:[%u][%d][%s] " fmt, current_uid().val, current->pid, __func__, ##__VA_ARGS__)
#else
#define SUSFS_LOGI(fmt, ...)
#define SUSFS_LOGE(fmt, ...)
#endif

bool susfs_starts_with(const char *str, const char *prefix) {
    while (*prefix) {
        if (*str++ != *prefix++)
            return false;
    }
    return true;
}

/* sus_path */
#ifdef CONFIG_KSU_SUSFS_SUS_PATH
static DEFINE_SPINLOCK(susfs_spin_lock_sus_path);
static LIST_HEAD(LH_SUS_PATH_LOOP);
static LIST_HEAD(LH_SUS_PATH_ANDROID_DATA);
static LIST_HEAD(LH_SUS_PATH_SDCARD);
static struct st_external_dir android_data_path = {0};
static struct st_external_dir sdcard_path = {0};
#ifndef FUSE_SUPER_MAGIC
#define FUSE_SUPER_MAGIC 0x65735546
#endif
const struct qstr susfs_fake_qstr_name = QSTR_INIT("..5.u.S", 7);

void susfs_set_i_state_on_external_dir(void __user **user_info) {
	struct st_external_dir info;
	struct path path;
	int err = 0;
	struct inode *inode = NULL;

	if (copy_from_user(&info, *user_info, sizeof(info))) {
		SUSFS_LOGE("failed copying from userspace\n");
		info.err = -EFAULT;
		goto out_copy_to_user;
	}

	err = kern_path(info.target_pathname, LOOKUP_FOLLOW, &path);
	if (err) {
		SUSFS_LOGE("Failed opening file '%s'\n", info.target_pathname);
		info.err = err;
		goto out_copy_to_user;
	}

	inode = d_inode(path.dentry);
	if (!inode) {
		info.err = -EINVAL;
		path_put(&path);
		goto out_copy_to_user;
	}

	if (info.cmd == CMD_SUSFS_SET_ANDROID_DATA_ROOT_PATH) {
		spin_lock(&inode->i_lock);
		set_bit(AS_FLAGS_ANDROID_DATA_ROOT_DIR, &inode->i_mapping->flags);
		spin_unlock(&inode->i_lock);
		strscpy(android_data_path.target_pathname, info.target_pathname, SUSFS_MAX_LEN_PATHNAME);
		android_data_path.is_inited = true;
		SUSFS_LOGI("Set android data root dir: '%s'\n", android_data_path.target_pathname);
	} else if (info.cmd == CMD_SUSFS_SET_SDCARD_ROOT_PATH) {
		spin_lock(&inode->i_lock);
		set_bit(AS_FLAGS_SDCARD_ROOT_DIR, &inode->i_mapping->flags);
		spin_unlock(&inode->i_lock);
		strscpy(sdcard_path.target_pathname, info.target_pathname, SUSFS_MAX_LEN_PATHNAME);
		sdcard_path.is_inited = true;
		SUSFS_LOGI("Set sdcard root dir: '%s'\n", sdcard_path.target_pathname);
	} else {
		info.err = -EINVAL;
	}

	path_put(&path);
out_copy_to_user:
	if (copy_to_user(*user_info, &info, sizeof(info)))
		SUSFS_LOGE("copy_to_user() failed\n");
}

void susfs_add_sus_path(void __user **user_info) {
	struct st_susfs_sus_path_list *cursor = NULL, *temp = NULL;
	struct st_susfs_sus_path_list *new_list = NULL;
	struct st_susfs_sus_path info;
	struct path path;
	struct inode *inode = NULL;
	char *resolved_pathname = NULL, *tmp_buf = NULL;
	int err = 0;

	if (copy_from_user(&info, *user_info, sizeof(info))) {
		SUSFS_LOGE("failed copying from userspace\n");
		info.err = -EFAULT;
		goto out_copy_to_user;
	}

	err = kern_path(info.target_pathname, 0, &path);
	if (err) {
		SUSFS_LOGE("Failed opening file '%s'\n", info.target_pathname);
		info.err = err;
		goto out_copy_to_user;
	}

	if (!path.dentry->d_inode) {
		info.err = -EINVAL;
		goto out_path_put_path;
	}
	inode = d_inode(path.dentry);

	tmp_buf = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!tmp_buf) {
		info.err = -ENOMEM;
		goto out_path_put_path;
	}

	resolved_pathname = d_path(&path, tmp_buf, PAGE_SIZE);
	if (IS_ERR_OR_NULL(resolved_pathname)) {
		info.err = resolved_pathname ? PTR_ERR(resolved_pathname) : -ENOMEM;
		goto out_kfree_tmp_buf;
	}

	if (strstr(resolved_pathname, android_data_path.target_pathname)) {
		if (!android_data_path.is_inited) {
			info.err = -EINVAL;
			SUSFS_LOGE("android_data_path is not configured yet\n");
			goto out_kfree_tmp_buf;
		}
		spin_lock(&susfs_spin_lock_sus_path);
		list_for_each_entry_safe(cursor, temp, &LH_SUS_PATH_ANDROID_DATA, list) {
			if (unlikely(!strcmp(cursor->info.target_pathname, path.dentry->d_name.name))) {
				cursor->info.target_ino = info.target_ino;
				strscpy(cursor->info.target_pathname, path.dentry->d_name.name, SUSFS_MAX_LEN_PATHNAME);
				strscpy(cursor->target_pathname, resolved_pathname, SUSFS_MAX_LEN_PATHNAME);
				cursor->info.i_uid = info.i_uid;
				cursor->path_len = strlen(cursor->info.target_pathname);
				spin_unlock(&susfs_spin_lock_sus_path);
				SUSFS_LOGI("target_ino: '%lu', target_pathname: '%s', i_uid: '%u', is successfully updated to LH_SUS_PATH_ANDROID_DATA\n",
							cursor->info.target_ino, cursor->target_pathname, cursor->info.i_uid);
				goto out_kfree_tmp_buf;
			}
		}
		spin_unlock(&susfs_spin_lock_sus_path);
		new_list = kmalloc(sizeof(struct st_susfs_sus_path_list), GFP_KERNEL);
		if (!new_list) {
			info.err = -ENOMEM;
			goto out_kfree_tmp_buf;
		}
		new_list->info.target_ino = info.target_ino;
		strscpy(new_list->info.target_pathname, path.dentry->d_name.name, SUSFS_MAX_LEN_PATHNAME);
		strscpy(new_list->target_pathname, resolved_pathname, SUSFS_MAX_LEN_PATHNAME);
		new_list->info.i_uid = info.i_uid;
		new_list->path_len = strlen(new_list->info.target_pathname);
		INIT_LIST_HEAD(&new_list->list);
		spin_lock(&susfs_spin_lock_sus_path);
		list_add_tail(&new_list->list, &LH_SUS_PATH_ANDROID_DATA);
		SUSFS_LOGI("target_ino: '%lu', target_pathname: '%s', i_uid: '%u', is successfully added to LH_SUS_PATH_ANDROID_DATA\n",
					new_list->info.target_ino, new_list->target_pathname, new_list->info.i_uid);
		spin_unlock(&susfs_spin_lock_sus_path);
		goto out_kfree_tmp_buf;
	} else if (strstr(resolved_pathname, sdcard_path.target_pathname)) {
		if (!sdcard_path.is_inited) {
			info.err = -EINVAL;
			SUSFS_LOGE("sdcard_path is not configured yet\n");
			goto out_kfree_tmp_buf;
		}
		spin_lock(&susfs_spin_lock_sus_path);
		list_for_each_entry_safe(cursor, temp, &LH_SUS_PATH_SDCARD, list) {
			if (unlikely(!strcmp(cursor->info.target_pathname, path.dentry->d_name.name))) {
				cursor->info.target_ino = info.target_ino;
				strscpy(cursor->info.target_pathname, path.dentry->d_name.name, SUSFS_MAX_LEN_PATHNAME);
				strscpy(cursor->target_pathname, resolved_pathname, SUSFS_MAX_LEN_PATHNAME);
				cursor->info.i_uid = info.i_uid;
				cursor->path_len = strlen(cursor->info.target_pathname);
				spin_unlock(&susfs_spin_lock_sus_path);
				SUSFS_LOGI("target_ino: '%lu', target_pathname: '%s', i_uid: '%u', is successfully updated to LH_SUS_PATH_SDCARD\n",
							cursor->info.target_ino, cursor->target_pathname, cursor->info.i_uid);
				goto out_kfree_tmp_buf;
			}
		}
		spin_unlock(&susfs_spin_lock_sus_path);
		new_list = kmalloc(sizeof(struct st_susfs_sus_path_list), GFP_KERNEL);
		if (!new_list) {
			info.err = -ENOMEM;
			goto out_kfree_tmp_buf;
		}
		new_list->info.target_ino = info.target_ino;
		strscpy(new_list->info.target_pathname, path.dentry->d_name.name, SUSFS_MAX_LEN_PATHNAME);
		strscpy(new_list->target_pathname, resolved_pathname, SUSFS_MAX_LEN_PATHNAME);
		new_list->info.i_uid = info.i_uid;
		new_list->path_len = strlen(new_list->info.target_pathname);
		INIT_LIST_HEAD(&new_list->list);
		spin_lock(&susfs_spin_lock_sus_path);
		list_add_tail(&new_list->list, &LH_SUS_PATH_SDCARD);
		SUSFS_LOGI("target_ino: '%lu', target_pathname: '%s', i_uid: '%u', is successfully added to LH_SUS_PATH_SDCARD\n",
					new_list->info.target_ino, new_list->target_pathname, new_list->info.i_uid);
		spin_unlock(&susfs_spin_lock_sus_path);
		goto out_kfree_tmp_buf;
	}

	if (inode->i_sb->s_magic == FUSE_SUPER_MAGIC) {
		struct fuse_inode *fi = get_fuse_inode(inode);
		if (fi) {
			set_bit(AS_FLAGS_SUS_PATH, &fi->inode.i_mapping->flags);
			SUSFS_LOGI("flagged AS_FLAGS_SUS_PATH on FUSE pathname: '%s', fi->nodeid: %llu\n",
						resolved_pathname, fi->nodeid);
		}
	} else {
		spin_lock(&inode->i_lock);
		set_bit(AS_FLAGS_SUS_PATH, &inode->i_mapping->flags);
		spin_unlock(&inode->i_lock);
		SUSFS_LOGI("pathname: '%s', ino: '%lu', is flagged as AS_FLAGS_SUS_PATH\n", resolved_pathname, info.target_ino);
	}
out_kfree_tmp_buf:
	kfree(tmp_buf);
out_path_put_path:
	path_put(&path);
out_copy_to_user:
	if (copy_to_user(*user_info, &info, sizeof(info)))
		SUSFS_LOGE("copy_to_user() failed\n");
}

void susfs_add_sus_path_loop(void __user **user_info) {
	struct st_susfs_sus_path_list *cursor = NULL, *temp = NULL;
	struct st_susfs_sus_path_list *new_list = NULL;
	struct st_susfs_sus_path info;
	struct path path;
	struct inode *inode = NULL;
	char *resolved_pathname = NULL, *tmp_buf = NULL;
	int err = 0;

	if (copy_from_user(&info, *user_info, sizeof(info))) {
		SUSFS_LOGE("failed copying from userspace\n");
		info.err = -EFAULT;
		goto out_copy_to_user;
	}

	err = kern_path(info.target_pathname, 0, &path);
	if (err) {
		SUSFS_LOGE("Failed opening file '%s'\n", info.target_pathname);
		info.err = err;
		goto out_copy_to_user;
	}

	if (!path.dentry->d_inode) {
		info.err = -EINVAL;
		goto out_path_put_path;
	}
	inode = d_inode(path.dentry);

	tmp_buf = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!tmp_buf) {
		info.err = -ENOMEM;
		goto out_path_put_path;
	}

	resolved_pathname = d_path(&path, tmp_buf, PAGE_SIZE);
	if (IS_ERR_OR_NULL(resolved_pathname)) {
		info.err = resolved_pathname ? PTR_ERR(resolved_pathname) : -ENOMEM;
		goto out_kfree_tmp_buf;
	}
	SUSFS_LOGI("resolved_pathname: %s\n", resolved_pathname);

	if (susfs_starts_with(resolved_pathname, "/storage/")) {
		info.err = -EINVAL;
		SUSFS_LOGE("path starts with /storage and /sdcard cannot be added by add_sus_path_loop\n");
		goto out_kfree_tmp_buf;
	}

	spin_lock(&susfs_spin_lock_sus_path);
	list_for_each_entry_safe(cursor, temp, &LH_SUS_PATH_LOOP, list) {
		if (unlikely(!strcmp(cursor->info.target_pathname, resolved_pathname))) {
			cursor->info.target_ino = info.target_ino;
			strscpy(cursor->info.target_pathname, resolved_pathname, SUSFS_MAX_LEN_PATHNAME);
			strscpy(cursor->target_pathname, resolved_pathname, SUSFS_MAX_LEN_PATHNAME);
			cursor->info.i_uid = info.i_uid;
			cursor->path_len = strlen(cursor->info.target_pathname);
			spin_unlock(&susfs_spin_lock_sus_path);
			SUSFS_LOGI("target_ino: '%lu', target_pathname: '%s', i_uid: '%u', is successfully updated to LH_SUS_PATH_LOOP\n",
						cursor->info.target_ino, cursor->target_pathname, cursor->info.i_uid);
			goto out_set_sus_path;
		}
	}
	spin_unlock(&susfs_spin_lock_sus_path);
	new_list = kmalloc(sizeof(struct st_susfs_sus_path_list), GFP_KERNEL);
	if (!new_list) {
		info.err = -ENOMEM;
		goto out_kfree_tmp_buf;
	}
	new_list->info.target_ino = info.target_ino;
	strscpy(new_list->info.target_pathname, resolved_pathname, SUSFS_MAX_LEN_PATHNAME);
	strscpy(new_list->target_pathname, resolved_pathname, SUSFS_MAX_LEN_PATHNAME);
	new_list->info.i_uid = info.i_uid;
	new_list->path_len = strlen(new_list->info.target_pathname);
	INIT_LIST_HEAD(&new_list->list);
	spin_lock(&susfs_spin_lock_sus_path);
	list_add_tail(&new_list->list, &LH_SUS_PATH_LOOP);
	SUSFS_LOGI("target_ino: '%lu', target_pathname: '%s', i_uid: '%u', is successfully added to LH_SUS_PATH_LOOP\n",
				new_list->info.target_ino, new_list->target_pathname, new_list->info.i_uid);
	spin_unlock(&susfs_spin_lock_sus_path);
out_set_sus_path:
	spin_lock(&inode->i_lock);
	set_bit(AS_FLAGS_SUS_PATH, &inode->i_mapping->flags);
	SUSFS_LOGI("pathname: '%s', ino: '%lu', is flagged as AS_FLAGS_SUS_PATH\n", resolved_pathname, info.target_ino);
	spin_unlock(&inode->i_lock);
out_kfree_tmp_buf:
	kfree(tmp_buf);
out_path_put_path:
	path_put(&path);
out_copy_to_user:
	if (copy_to_user(*user_info, &info, sizeof(info)))
		SUSFS_LOGE("copy_to_user() failed\n");
}

void susfs_run_sus_path_loop(uid_t uid) {
	struct st_susfs_sus_path_list *cursor = NULL;
	struct path path;
	struct inode *inode;

	rcu_read_lock();
	list_for_each_entry_rcu(cursor, &LH_SUS_PATH_LOOP, list) {
		if (!kern_path(cursor->target_pathname, 0, &path)) {
			inode = d_backing_inode(path.dentry);
			if (!inode || !inode->i_mapping) {
				path_put(&path);
				continue;
			}
			if (inode->i_sb->s_magic == FUSE_SUPER_MAGIC) {
				struct fuse_inode *fi = get_fuse_inode(inode);
				if (fi)
					set_bit(AS_FLAGS_SUS_PATH, &fi->inode.i_mapping->flags);
			} else {
				set_bit(AS_FLAGS_SUS_PATH, &inode->i_mapping->flags);
			}
			path_put(&path);
			SUSFS_LOGI("re-flag AS_FLAGS_SUS_PATH on path '%s' for uid: %u\n", cursor->target_pathname, uid);
		}
	}
	rcu_read_unlock();
}

int susfs_auto_add_sus_path_internal(const char *pathname) {
	struct st_susfs_sus_path_list *cursor = NULL, *temp = NULL;
	struct st_susfs_sus_path_list *new_list = NULL;
	struct path path;
	struct inode *inode = NULL;
	char *resolved_pathname = NULL, *tmp_buf = NULL;
	int err = 0;

	err = kern_path(pathname, 0, &path);
	if (err) {
		SUSFS_LOGI("auto_add_sus_path: path '%s' not found, skipping\n", pathname);
		return err;
	}

	if (!path.dentry->d_inode) {
		err = -EINVAL;
		goto out_path_put;
	}
	inode = d_inode(path.dentry);

	tmp_buf = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!tmp_buf) {
		err = -ENOMEM;
		goto out_path_put;
	}

	resolved_pathname = d_path(&path, tmp_buf, PAGE_SIZE);
	if (IS_ERR_OR_NULL(resolved_pathname)) {
		err = -ENOMEM;
		goto out_kfree;
	}

	/* Check if already in the loop list */
	list_for_each_entry_safe(cursor, temp, &LH_SUS_PATH_LOOP, list) {
		if (unlikely(!strcmp(cursor->info.target_pathname, resolved_pathname))) {
			SUSFS_LOGI("auto_add_sus_path: '%s' already in LH_SUS_PATH_LOOP\n", resolved_pathname);
			goto out_set_flag;
		}
	}

	new_list = kmalloc(sizeof(struct st_susfs_sus_path_list), GFP_KERNEL);
	if (!new_list) {
		err = -ENOMEM;
		goto out_kfree;
	}
	new_list->info.target_ino = inode->i_ino;
	strscpy(new_list->info.target_pathname, resolved_pathname, SUSFS_MAX_LEN_PATHNAME);
	strscpy(new_list->target_pathname, resolved_pathname, SUSFS_MAX_LEN_PATHNAME);
	new_list->info.i_uid = 0;
	new_list->path_len = strlen(new_list->info.target_pathname);
	INIT_LIST_HEAD(&new_list->list);
	spin_lock(&susfs_spin_lock_sus_path);
	list_add_tail(&new_list->list, &LH_SUS_PATH_LOOP);
	spin_unlock(&susfs_spin_lock_sus_path);
	SUSFS_LOGI("auto_add_sus_path: '%s' added to LH_SUS_PATH_LOOP\n", resolved_pathname);

out_set_flag:
	spin_lock(&inode->i_lock);
	set_bit(AS_FLAGS_SUS_PATH, &inode->i_mapping->flags);
	spin_unlock(&inode->i_lock);
	SUSFS_LOGI("auto_add_sus_path: '%s' flagged as AS_FLAGS_SUS_PATH\n", resolved_pathname);
out_kfree:
	kfree(tmp_buf);
out_path_put:
	path_put(&path);
	return err;
}

static inline bool is_i_uid_in_android_data_not_allowed(uid_t i_uid) {
	return (likely(susfs_is_current_proc_umounted()) &&
		unlikely(current_uid().val != i_uid));
}

static inline bool is_i_uid_in_sdcard_not_allowed(void) {
	return (likely(susfs_is_current_proc_umounted()));
}

static inline bool is_i_uid_not_allowed(uid_t i_uid) {
	return (likely(susfs_is_current_proc_umounted()) &&
		unlikely(current_uid().val != i_uid));
}

bool susfs_is_base_dentry_android_data_dir(struct dentry* base) {
	return (base && !IS_ERR(base) && base->d_inode && test_bit(AS_FLAGS_ANDROID_DATA_ROOT_DIR, &base->d_inode->i_mapping->flags));
}

bool susfs_is_base_dentry_sdcard_dir(struct dentry* base) {
	return (base && !IS_ERR(base) && base->d_inode && test_bit(AS_FLAGS_SDCARD_ROOT_DIR, &base->d_inode->i_mapping->flags));
}

bool susfs_is_sus_android_data_d_name_found(const char *d_name) {
	struct st_susfs_sus_path_list *cursor = NULL, *temp = NULL;

	if (d_name[0] == '\0') {
		return false;
	}

	list_for_each_entry_safe(cursor, temp, &LH_SUS_PATH_ANDROID_DATA, list) {
		if (!strncmp(d_name, cursor->info.target_pathname, cursor->path_len) &&
		    (d_name[cursor->path_len] == '\0' || d_name[cursor->path_len] == '/') &&
			is_i_uid_in_android_data_not_allowed(cursor->info.i_uid))
		{
			pr_debug("susfs: hiding path '%s'\n", cursor->target_pathname);
			return true;
		}
	}
	return false;
}

bool susfs_is_sus_sdcard_d_name_found(const char *d_name) {
	struct st_susfs_sus_path_list *cursor = NULL, *temp = NULL;

	if (d_name[0] == '\0') {
		return false;
	}
	list_for_each_entry_safe(cursor, temp, &LH_SUS_PATH_SDCARD, list) {
		if (!strncmp(d_name, cursor->info.target_pathname, cursor->path_len) &&
		    (d_name[cursor->path_len] == '\0' || d_name[cursor->path_len] == '/') &&
			is_i_uid_in_sdcard_not_allowed())
		{
			pr_debug("susfs: hiding path '%s'\n", cursor->target_pathname);
			return true;
		}
	}
	return false;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0)
bool susfs_is_inode_sus_path(struct mnt_idmap* idmap, struct inode *inode) {
	struct fuse_inode *fi = NULL;
	if (current_uid().val < 10000 || !susfs_is_current_proc_umounted())
		return false;
	if (inode->i_sb->s_magic == FUSE_SUPER_MAGIC) {
		fi = get_fuse_inode(inode);
		if (!fi)
			return false;
		if (unlikely(test_bit(AS_FLAGS_SUS_PATH, &fi->inode.i_mapping->flags) &&
			is_i_uid_not_allowed(i_uid_into_vfsuid(idmap, &fi->inode).val)))
		{
			SUSFS_LOGI("hiding FUSE path with ino '%lu'\n", inode->i_ino);
			return true;
		}
		return false;
	}
	if (unlikely(test_bit(AS_FLAGS_SUS_PATH, &inode->i_mapping->flags) &&
		is_i_uid_not_allowed(i_uid_into_vfsuid(idmap, inode).val)))
	{
		pr_debug("susfs: hiding path with ino '%lu'\n", inode->i_ino);
		return true;
	}
	return false;
}
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
bool susfs_is_inode_sus_path(struct inode *inode) {
	struct fuse_inode *fi = NULL;
	if (current_uid().val < 10000 || !susfs_is_current_proc_umounted())
		return false;
	if (inode->i_sb->s_magic == FUSE_SUPER_MAGIC) {
		fi = get_fuse_inode(inode);
		if (!fi)
			return false;
		if (unlikely(test_bit(AS_FLAGS_SUS_PATH, &fi->inode.i_mapping->flags) &&
			is_i_uid_not_allowed(i_uid_into_mnt(i_user_ns(&fi->inode), &fi->inode).val)))
		{
			SUSFS_LOGI("hiding FUSE path with ino '%lu'\n", inode->i_ino);
			return true;
		}
		return false;
	}
	if (unlikely(test_bit(AS_FLAGS_SUS_PATH, &inode->i_mapping->flags) &&
		is_i_uid_not_allowed(i_uid_into_mnt(i_user_ns(inode), inode).val)))
	{
		pr_debug("susfs: hiding path with ino '%lu'\n", inode->i_ino);
		return true;
	}
	return false;
}
#else
bool susfs_is_inode_sus_path(struct inode *inode) {
	struct fuse_inode *fi = NULL;
	if (current_uid().val < 10000 || !susfs_is_current_proc_umounted())
		return false;
	if (inode->i_sb->s_magic == FUSE_SUPER_MAGIC) {
		fi = get_fuse_inode(inode);
		if (!fi)
			return false;
		if (unlikely(test_bit(AS_FLAGS_SUS_PATH, &fi->inode.i_mapping->flags) &&
			is_i_uid_not_allowed(fi->inode.i_uid.val)))
		{
			SUSFS_LOGI("hiding FUSE path with ino '%lu'\n", inode->i_ino);
			return true;
		}
		return false;
	}
	if (unlikely(test_bit(AS_FLAGS_SUS_PATH, &inode->i_mapping->flags) &&
		is_i_uid_not_allowed(inode->i_uid.val)))
	{
		pr_debug("susfs: hiding path with ino '%lu'\n", inode->i_ino);
		return true;
	}
	return false;
}
#endif

#endif // #ifdef CONFIG_KSU_SUSFS_SUS_PATH

/* sus_mount */
#ifdef CONFIG_KSU_SUSFS_SUS_MOUNT
static DEFINE_SPINLOCK(susfs_spin_lock_sus_mount);
bool susfs_hide_sus_mnts_for_non_su_procs = false;

void susfs_set_hide_sus_mnts_for_non_su_procs(void __user **user_info) {
	struct st_susfs_hide_sus_mnts_for_non_su_procs info;

	if (copy_from_user(&info, *user_info, sizeof(info))) {
		SUSFS_LOGE("failed copying from userspace\n");
		info.err = -EFAULT;
		goto out_copy_to_user;
	}

	spin_lock(&susfs_spin_lock_sus_mount);
	susfs_hide_sus_mnts_for_non_su_procs = info.enabled;
	spin_unlock(&susfs_spin_lock_sus_mount);
	SUSFS_LOGI("susfs_hide_sus_mnts_for_non_su_procs: %d\n", info.enabled);
	info.err = 0;

out_copy_to_user:
	if (copy_to_user(*user_info, &info, sizeof(info)))
		SUSFS_LOGE("copy_to_user() failed\n");
}
#endif // #ifdef CONFIG_KSU_SUSFS_SUS_MOUNT

/* sus_kstat */
#ifdef CONFIG_KSU_SUSFS_SUS_KSTAT
static DEFINE_SPINLOCK(susfs_spin_lock_sus_kstat);
static DEFINE_HASHTABLE(SUS_KSTAT_HLIST, 10);
static int susfs_update_sus_kstat_inode(char *target_pathname) {
	struct path p;
	struct inode *inode = NULL;
	int err = 0;

	err = kern_path(target_pathname, 0, &p);
	if (err) {
		SUSFS_LOGE("Failed opening file '%s'\n", target_pathname);
		return 1;
	}

	inode = d_inode(p.dentry);
	if (!inode) {
		path_put(&p);
		SUSFS_LOGE("inode is NULL\n");
		return 1;
	}

	if (!test_bit(AS_FLAGS_SUS_KSTAT, &inode->i_mapping->flags)) {
		spin_lock(&inode->i_lock);
		set_bit(AS_FLAGS_SUS_KSTAT, &inode->i_mapping->flags);
		spin_unlock(&inode->i_lock);
	}
	path_put(&p);
	return 0;
}

void susfs_add_sus_kstat(void __user **user_info) {
	struct st_susfs_sus_kstat info;
	struct st_susfs_sus_kstat_hlist *new_entry, *tmp_entry;
	struct hlist_node *tmp_node;
	int bkt;
	bool update_hlist = false;

	if (copy_from_user(&info, *user_info, sizeof(info))) {
		SUSFS_LOGE("failed copying from userspace\n");
		info.err = -EFAULT;
		goto out_copy_to_user;
	}

	if (strlen(info.target_pathname) == 0) {
		SUSFS_LOGE("target_pathname is an empty string\n");
		info.err = -EINVAL;
		goto out_copy_to_user;
	}

	spin_lock(&susfs_spin_lock_sus_kstat);
	hash_for_each_safe(SUS_KSTAT_HLIST, bkt, tmp_node, tmp_entry, node) {
		if (!strcmp(tmp_entry->info.target_pathname, info.target_pathname)) {
			hash_del(&tmp_entry->node);
			kfree(tmp_entry);
			update_hlist = true;
			break;
		}
	}
	spin_unlock(&susfs_spin_lock_sus_kstat);

	new_entry = kmalloc(sizeof(struct st_susfs_sus_kstat_hlist), GFP_KERNEL);
	if (!new_entry) {
		SUSFS_LOGE("no enough memory\n");
		info.err = -ENOMEM;
		goto out_copy_to_user;
	}

#if defined(__ARCH_WANT_STAT64) || defined(__ARCH_WANT_COMPAT_STAT64)
#ifdef CONFIG_MIPS
	info.spoofed_dev = new_decode_dev(info.spoofed_dev);
#else
	info.spoofed_dev = huge_decode_dev(info.spoofed_dev);
#endif /* CONFIG_MIPS */
#else
	info.spoofed_dev = old_decode_dev(info.spoofed_dev);
#endif /* defined(__ARCH_WANT_STAT64) || defined(__ARCH_WANT_COMPAT_STAT64) */

	new_entry->target_ino = info.target_ino;
	memcpy(&new_entry->info, &info, sizeof(info));

	if (susfs_update_sus_kstat_inode(new_entry->info.target_pathname)) {
		kfree(new_entry);
		info.err = -EINVAL;
		goto out_copy_to_user;
	}

	spin_lock(&susfs_spin_lock_sus_kstat);
	hash_add(SUS_KSTAT_HLIST, &new_entry->node, info.target_ino);
	if (update_hlist) {
		SUSFS_LOGI("target_ino: '%lu', target_pathname: '%s', is successfully updated to SUS_KSTAT_HLIST\n",
				new_entry->info.target_ino, new_entry->info.target_pathname);
	} else {
		SUSFS_LOGI("target_ino: '%lu', target_pathname: '%s', is successfully added to SUS_KSTAT_HLIST\n",
				new_entry->info.target_ino, new_entry->info.target_pathname);
	}
	spin_unlock(&susfs_spin_lock_sus_kstat);
	info.err = 0;

out_copy_to_user:
	if (copy_to_user(*user_info, &info, sizeof(info)))
		SUSFS_LOGE("copy_to_user() failed\n");
}

void susfs_update_sus_kstat(void __user **user_info) {
	struct st_susfs_sus_kstat info;
	struct st_susfs_sus_kstat_hlist *new_entry = NULL, *tmp_entry;
	struct st_susfs_sus_kstat saved_info;
	struct hlist_node *tmp_node;
	int bkt;
	int err = 0;
	bool found = false;

	if (copy_from_user(&info, *user_info, sizeof(info))) {
		SUSFS_LOGE("failed copying from userspace\n");
		info.err = -EFAULT;
		goto out_copy_to_user;
	}

	/* First pass: find the matching entry under lock and save its data */
	spin_lock(&susfs_spin_lock_sus_kstat);
	hash_for_each_safe(SUS_KSTAT_HLIST, bkt, tmp_node, tmp_entry, node) {
		if (!strcmp(tmp_entry->info.target_pathname, info.target_pathname)) {
			memcpy(&saved_info, &tmp_entry->info, sizeof(tmp_entry->info));
			found = true;
			break;
		}
	}
	spin_unlock(&susfs_spin_lock_sus_kstat);

	if (!found)
		goto out_set_err;

	/* Sleeping operations outside spinlock */
	if (susfs_update_sus_kstat_inode(saved_info.target_pathname)) {
		err = 1;
		goto out_set_err;
	}
	new_entry = kmalloc(sizeof(struct st_susfs_sus_kstat_hlist), GFP_KERNEL);
	if (!new_entry) {
		SUSFS_LOGE("no enough memory\n");
		err = 1;
		goto out_set_err;
	}

	memcpy(&new_entry->info, &saved_info, sizeof(saved_info));
	SUSFS_LOGI("updating target_ino from '%lu' to '%lu' for pathname: '%s' in SUS_KSTAT_HLIST\n",
					new_entry->info.target_ino, info.target_ino, info.target_pathname);
	new_entry->target_ino = info.target_ino;
	new_entry->info.target_ino = info.target_ino;
	if (info.spoofed_size > 0) {
		new_entry->info.spoofed_size = info.spoofed_size;
	}
	if (info.spoofed_blocks > 0) {
		new_entry->info.spoofed_blocks = info.spoofed_blocks;
	}

	/* Second pass: remove old entry and add new one under lock */
	spin_lock(&susfs_spin_lock_sus_kstat);
	hash_for_each_safe(SUS_KSTAT_HLIST, bkt, tmp_node, tmp_entry, node) {
		if (!strcmp(tmp_entry->info.target_pathname, info.target_pathname)) {
			hash_del(&tmp_entry->node);
			kfree(tmp_entry);
			break;
		}
	}
	hash_add(SUS_KSTAT_HLIST, &new_entry->node, info.target_ino);
	spin_unlock(&susfs_spin_lock_sus_kstat);

out_set_err:
	info.err = err;

out_copy_to_user:
	if (copy_to_user(*user_info, &info, sizeof(info)))
		SUSFS_LOGE("copy_to_user() failed\n");
}

void susfs_sus_ino_for_generic_fillattr(unsigned long ino, struct kstat *stat) {
	struct st_susfs_sus_kstat_hlist *entry;

	hash_for_each_possible(SUS_KSTAT_HLIST, entry, node, ino) {
		if (entry->target_ino == ino) {
			stat->dev = entry->info.spoofed_dev;
			stat->ino = entry->info.spoofed_ino;
			stat->nlink = entry->info.spoofed_nlink;
			stat->size = entry->info.spoofed_size;
			stat->atime.tv_sec = entry->info.spoofed_atime_tv_sec;
			stat->atime.tv_nsec = entry->info.spoofed_atime_tv_nsec;
			stat->mtime.tv_sec = entry->info.spoofed_mtime_tv_sec;
			stat->mtime.tv_nsec = entry->info.spoofed_mtime_tv_nsec;
			stat->ctime.tv_sec = entry->info.spoofed_ctime_tv_sec;
			stat->ctime.tv_nsec = entry->info.spoofed_ctime_tv_nsec;
			stat->blocks = entry->info.spoofed_blocks;
			stat->blksize = entry->info.spoofed_blksize;
			return;
		}
	}
}

void susfs_sus_ino_for_show_map_vma(unsigned long ino, dev_t *out_dev, unsigned long *out_ino) {
	struct st_susfs_sus_kstat_hlist *entry;

	hash_for_each_possible(SUS_KSTAT_HLIST, entry, node, ino) {
		if (entry->target_ino == ino) {
			*out_dev = entry->info.spoofed_dev;
			*out_ino = entry->info.spoofed_ino;
			return;
		}
	}
}

int susfs_auto_add_sus_kstat_internal(const char *pathname, long long spoofed_size, unsigned long long spoofed_blocks) {
	struct st_susfs_sus_kstat_hlist *new_entry = NULL;
	struct path p;
	struct inode *inode = NULL;
	struct kstat real_stat;
	int err = 0;

	err = kern_path(pathname, LOOKUP_FOLLOW, &p);
	if (err) {
		SUSFS_LOGI("auto_add_sus_kstat: path '%s' not found, skipping\n", pathname);
		return err;
	}

	inode = d_inode(p.dentry);
	if (!inode) {
		err = -EINVAL;
		goto out_path_put;
	}

	/* Get real stat values */
	err = vfs_getattr(&p, &real_stat, STATX_BASIC_STATS, AT_STATX_SYNC_AS_STAT);
	if (err) {
		SUSFS_LOGE("auto_add_sus_kstat: vfs_getattr failed for '%s'\n", pathname);
		goto out_path_put;
	}

	new_entry = kzalloc(sizeof(*new_entry), GFP_KERNEL);
	if (!new_entry) {
		err = -ENOMEM;
		goto out_path_put;
	}

	/* Fill with real values, override size and blocks */
	new_entry->target_ino = inode->i_ino;
	new_entry->info.target_ino = inode->i_ino;
	strscpy(new_entry->info.target_pathname, pathname, SUSFS_MAX_LEN_PATHNAME);
	new_entry->info.spoofed_ino = real_stat.ino;
	new_entry->info.spoofed_dev = real_stat.dev;
	new_entry->info.spoofed_nlink = real_stat.nlink;
	new_entry->info.spoofed_size = spoofed_size;
	new_entry->info.spoofed_atime_tv_sec = real_stat.atime.tv_sec;
	new_entry->info.spoofed_atime_tv_nsec = real_stat.atime.tv_nsec;
	new_entry->info.spoofed_mtime_tv_sec = real_stat.mtime.tv_sec;
	new_entry->info.spoofed_mtime_tv_nsec = real_stat.mtime.tv_nsec;
	new_entry->info.spoofed_ctime_tv_sec = real_stat.ctime.tv_sec;
	new_entry->info.spoofed_ctime_tv_nsec = real_stat.ctime.tv_nsec;
	new_entry->info.spoofed_blksize = real_stat.blksize;
	new_entry->info.spoofed_blocks = spoofed_blocks;

	/* Set inode flag */
	if (!test_bit(AS_FLAGS_SUS_KSTAT, &inode->i_mapping->flags)) {
		spin_lock(&inode->i_lock);
		set_bit(AS_FLAGS_SUS_KSTAT, &inode->i_mapping->flags);
		spin_unlock(&inode->i_lock);
	}

	/* Add to hash table */
	spin_lock(&susfs_spin_lock_sus_kstat);
	hash_add(SUS_KSTAT_HLIST, &new_entry->node, inode->i_ino);
	spin_unlock(&susfs_spin_lock_sus_kstat);
	SUSFS_LOGI("auto_add_sus_kstat: '%s' (ino=%lu) spoofed size=%lld blocks=%llu\n",
		pathname, inode->i_ino, spoofed_size, spoofed_blocks);

out_path_put:
	path_put(&p);
	return err;
}
#endif // #ifdef CONFIG_KSU_SUSFS_SUS_KSTAT

/* spoof_uname */
#ifdef CONFIG_KSU_SUSFS_SPOOF_UNAME
static DEFINE_SPINLOCK(susfs_spin_lock_set_uname);
static struct st_susfs_uname my_uname;
static void susfs_my_uname_init(void) {
	memset(&my_uname, 0, sizeof(my_uname));
}

void susfs_set_uname(void __user **user_info) {
	struct st_susfs_uname info;

	if (copy_from_user(&info, *user_info, sizeof(info))) {
		SUSFS_LOGE("failed copying from userspace.\n");
		info.err = -EFAULT;
		goto out_copy_to_user;
	}

	spin_lock(&susfs_spin_lock_set_uname);
	if (!strcmp(info.release, "default")) {
		strscpy(my_uname.release, utsname()->release, sizeof(my_uname.release));
	} else {
		strscpy(my_uname.release, info.release, sizeof(my_uname.release));
	}
	if (!strcmp(info.version, "default")) {
		strscpy(my_uname.version, utsname()->version, sizeof(my_uname.version));
	} else {
		strscpy(my_uname.version, info.version, sizeof(my_uname.version));
	}
	spin_unlock(&susfs_spin_lock_set_uname);
	SUSFS_LOGI("setting spoofed release: '%s', version: '%s'\n",
				my_uname.release, my_uname.version);
	info.err = 0;

out_copy_to_user:
	if (copy_to_user(*user_info, &info, sizeof(info)))
		SUSFS_LOGE("copy_to_user() failed\n");
}

void susfs_spoof_uname(struct new_utsname* tmp) {
	if (unlikely(my_uname.release[0] == '\0'))
		return;
	if (!spin_trylock(&susfs_spin_lock_set_uname))
		return;
	strscpy(tmp->release, my_uname.release, sizeof(tmp->release));
	strscpy(tmp->version, my_uname.version, sizeof(tmp->version));
	spin_unlock(&susfs_spin_lock_set_uname);
}
#endif // #ifdef CONFIG_KSU_SUSFS_SPOOF_UNAME

/* enable_log */
#ifdef CONFIG_KSU_SUSFS_ENABLE_LOG
void susfs_enable_log(void __user **user_info) {
	struct st_susfs_log info;

	if (copy_from_user(&info, *user_info, sizeof(info))) {
		SUSFS_LOGE("failed copying from userspace\n");
		info.err = -EFAULT;
		goto out_copy_to_user;
	}

	susfs_is_log_enabled = info.enabled;
	if (susfs_is_log_enabled) {
		pr_info("susfs: enable logging to kernel\n");
	} else {
		pr_info("susfs: disable logging to kernel\n");
	}
	info.err = 0;

out_copy_to_user:
	if (copy_to_user(*user_info, &info, sizeof(info)))
		pr_err("susfs: copy_to_user() failed\n");
}
#endif // #ifdef CONFIG_KSU_SUSFS_ENABLE_LOG

/* spoof_cmdline_or_bootconfig */
#ifdef CONFIG_KSU_SUSFS_SPOOF_CMDLINE_OR_BOOTCONFIG
static DEFINE_SPINLOCK(susfs_spin_lock_set_cmdline_or_bootconfig);
static char *fake_cmdline_or_bootconfig = NULL;
static bool susfs_is_fake_cmdline_or_bootconfig_set = false;

void susfs_set_cmdline_or_bootconfig(void __user **user_info) {
	struct st_susfs_spoof_cmdline_or_bootconfig *info = NULL;
	int res;

	info = kmalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		return;
	}

	if (copy_from_user(info, *user_info, sizeof(*info))) {
		SUSFS_LOGE("failed copying from userspace\n");
		info->err = -EFAULT;
		goto out_copy_to_user;
	}

	if (!fake_cmdline_or_bootconfig) {
		fake_cmdline_or_bootconfig = kmalloc(SUSFS_FAKE_CMDLINE_OR_BOOTCONFIG_SIZE, GFP_KERNEL);
		if (!fake_cmdline_or_bootconfig) {
			SUSFS_LOGE("no enough memory\n");
			info->err = -ENOMEM;
			goto out_copy_to_user;
		}
	}

	spin_lock(&susfs_spin_lock_set_cmdline_or_bootconfig);
	memset(fake_cmdline_or_bootconfig, 0, SUSFS_FAKE_CMDLINE_OR_BOOTCONFIG_SIZE);
	res = strlen(info->fake_cmdline_or_bootconfig);
	if (res > 0 && res < SUSFS_FAKE_CMDLINE_OR_BOOTCONFIG_SIZE) {
		strscpy(fake_cmdline_or_bootconfig, info->fake_cmdline_or_bootconfig, SUSFS_FAKE_CMDLINE_OR_BOOTCONFIG_SIZE);
		susfs_is_fake_cmdline_or_bootconfig_set = true;
		SUSFS_LOGI("fake_cmdline_or_bootconfig is set, length of string: %d\n", res);
		info->err = 0;
	} else {
		SUSFS_LOGE("failed setting fake_cmdline_or_bootconfig\n");
		info->err = -EINVAL;
	}
	spin_unlock(&susfs_spin_lock_set_cmdline_or_bootconfig);

out_copy_to_user:
	if (copy_to_user(*user_info, info, sizeof(*info)))
		SUSFS_LOGE("copy_to_user() failed\n");
	kfree(info);
}

int susfs_spoof_cmdline_or_bootconfig(struct seq_file *m) {
	if (susfs_is_fake_cmdline_or_bootconfig_set && fake_cmdline_or_bootconfig != NULL) {
		seq_puts(m, fake_cmdline_or_bootconfig);
		return 0;
	}
	return 1;
}
#endif

/* open_redirect */
#ifdef CONFIG_KSU_SUSFS_OPEN_REDIRECT
static DEFINE_SPINLOCK(susfs_spin_lock_open_redirect);
static DEFINE_HASHTABLE(OPEN_REDIRECT_HLIST, 10);
static int susfs_update_open_redirect_inode(struct st_susfs_open_redirect_hlist *new_entry) {
	struct path path_target;
	struct inode *inode_target;
	int err = 0;

	err = kern_path(new_entry->target_pathname, LOOKUP_FOLLOW, &path_target);
	if (err) {
		SUSFS_LOGE("Failed opening file '%s'\n", new_entry->target_pathname);
		return err;
	}

	inode_target = d_inode(path_target.dentry);
	if (!inode_target) {
		SUSFS_LOGE("inode_target is NULL\n");
		err = 1;
		goto out_path_put_target;
	}

	spin_lock(&inode_target->i_lock);
	set_bit(AS_FLAGS_OPEN_REDIRECT, &inode_target->i_mapping->flags);
	spin_unlock(&inode_target->i_lock);

out_path_put_target:
	path_put(&path_target);
	return err;
}

void susfs_add_open_redirect(void __user **user_info) {
	struct st_susfs_open_redirect info;
	struct st_susfs_open_redirect_hlist *new_entry, *tmp_entry;
	struct hlist_node *tmp_node;
	int bkt;
	bool update_hlist = false;

	if (copy_from_user(&info, *user_info, sizeof(info))) {
		SUSFS_LOGE("failed copying from userspace\n");
		info.err = -EFAULT;
		goto out_copy_to_user;
	}

	spin_lock(&susfs_spin_lock_open_redirect);
	hash_for_each_safe(OPEN_REDIRECT_HLIST, bkt, tmp_node, tmp_entry, node) {
		if (!strcmp(tmp_entry->target_pathname, info.target_pathname)) {
			hash_del(&tmp_entry->node);
			kfree(tmp_entry);
			update_hlist = true;
			break;
		}
	}
	spin_unlock(&susfs_spin_lock_open_redirect);

	new_entry = kmalloc(sizeof(struct st_susfs_open_redirect_hlist), GFP_KERNEL);
	if (!new_entry) {
		SUSFS_LOGE("no enough memory\n");
		info.err = -ENOMEM;
		goto out_copy_to_user;
	}

	new_entry->target_ino = info.target_ino;
	strscpy(new_entry->target_pathname, info.target_pathname, SUSFS_MAX_LEN_PATHNAME);
	strscpy(new_entry->redirected_pathname, info.redirected_pathname, SUSFS_MAX_LEN_PATHNAME);
	if (susfs_update_open_redirect_inode(new_entry)) {
		SUSFS_LOGE("failed adding path '%s' to OPEN_REDIRECT_HLIST\n", new_entry->target_pathname);
		kfree(new_entry);
		info.err = -EINVAL;
		goto out_copy_to_user;
	}

	spin_lock(&susfs_spin_lock_open_redirect);
	hash_add(OPEN_REDIRECT_HLIST, &new_entry->node, info.target_ino);
	if (update_hlist) {
		SUSFS_LOGI("target_ino: '%lu', target_pathname: '%s', redirected_pathname: '%s', is successfully updated to OPEN_REDIRECT_HLIST\n",
				new_entry->target_ino, new_entry->target_pathname, new_entry->redirected_pathname);
	} else {
		SUSFS_LOGI("target_ino: '%lu', target_pathname: '%s' redirected_pathname: '%s', is successfully added to OPEN_REDIRECT_HLIST\n",
				new_entry->target_ino, new_entry->target_pathname, new_entry->redirected_pathname);
	}
	spin_unlock(&susfs_spin_lock_open_redirect);
	info.err = 0;

out_copy_to_user:
	if (copy_to_user(*user_info, &info, sizeof(info)))
		SUSFS_LOGE("copy_to_user() failed\n");
}

struct filename* susfs_get_redirected_path(unsigned long ino) {
	struct st_susfs_open_redirect_hlist *entry;

	hash_for_each_possible(OPEN_REDIRECT_HLIST, entry, node, ino) {
		if (entry->target_ino == ino) {
			SUSFS_LOGI("Redirect for ino: %lu\n", ino);
			return getname_kernel(entry->redirected_pathname);
		}
	}
	return ERR_PTR(-ENOENT);
}

int susfs_auto_add_open_redirect_internal(const char *target, const char *redirect) {
	struct st_susfs_open_redirect_hlist *new_entry = NULL;
	struct path path_target;
	struct inode *inode_target = NULL;
	int err = 0;

	err = kern_path(target, LOOKUP_FOLLOW, &path_target);
	if (err) {
		SUSFS_LOGI("auto_add_open_redirect: target '%s' not found, skipping\n", target);
		return err;
	}

	inode_target = d_inode(path_target.dentry);
	if (!inode_target) {
		err = -EINVAL;
		goto out_path_put;
	}

	new_entry = kzalloc(sizeof(*new_entry), GFP_KERNEL);
	if (!new_entry) {
		err = -ENOMEM;
		goto out_path_put;
	}

	new_entry->target_ino = inode_target->i_ino;
	strscpy(new_entry->target_pathname, target, SUSFS_MAX_LEN_PATHNAME);
	strscpy(new_entry->redirected_pathname, redirect, SUSFS_MAX_LEN_PATHNAME);

	/* Set inode flag */
	spin_lock(&inode_target->i_lock);
	set_bit(AS_FLAGS_OPEN_REDIRECT, &inode_target->i_mapping->flags);
	spin_unlock(&inode_target->i_lock);

	/* Add to hash table */
	spin_lock(&susfs_spin_lock_open_redirect);
	hash_add(OPEN_REDIRECT_HLIST, &new_entry->node, inode_target->i_ino);
	spin_unlock(&susfs_spin_lock_open_redirect);
	SUSFS_LOGI("auto_add_open_redirect: '%s' -> '%s' (ino=%lu)\n",
		target, redirect, inode_target->i_ino);

out_path_put:
	path_put(&path_target);
	return err;
}
#endif // #ifdef CONFIG_KSU_SUSFS_OPEN_REDIRECT

/* sus_map */
#ifdef CONFIG_KSU_SUSFS_SUS_MAP
void susfs_add_sus_map(void __user **user_info) {
	struct st_susfs_sus_map info;
	struct path path;
	struct inode *inode = NULL;
	int err = 0;

	if (copy_from_user(&info, *user_info, sizeof(info))) {
		SUSFS_LOGE("failed copying from userspace\n");
		info.err = -EFAULT;
		goto out_copy_to_user;
	}

	err = kern_path(info.target_pathname, LOOKUP_FOLLOW, &path);
	if (err) {
		SUSFS_LOGE("Failed opening file '%s'\n", info.target_pathname);
		info.err = err;
		goto out_copy_to_user;
	}

	if (!path.dentry->d_inode) {
		info.err = -EINVAL;
		goto out_path_put_path;
	}
	inode = d_inode(path.dentry);
	spin_lock(&inode->i_lock);
	set_bit(AS_FLAGS_SUS_MAP, &inode->i_mapping->flags);
	SUSFS_LOGI("pathname: '%s', is flagged as AS_FLAGS_SUS_MAP\n", info.target_pathname);
	spin_unlock(&inode->i_lock);
	info.err = 0;
out_path_put_path:
	path_put(&path);
out_copy_to_user:
	if (copy_to_user(*user_info, &info, sizeof(info)))
		SUSFS_LOGE("copy_to_user() failed\n");
}
#endif // #ifdef CONFIG_KSU_SUSFS_SUS_MAP

/* hide_resetprop_traces */
#ifdef CONFIG_KSU_SUSFS_HIDE_RESETPROP_TRACES
/*
 * Android property area format (from bionic):
 *
 * prop_area header (128 bytes at file offset 0):
 *   uint32_t bytes_used    (offset 0)
 *   uint32_t serial        (offset 4)  - global serial counter
 *   uint32_t magic         (offset 8)  - 0x504f5250 ("PROP")
 *   uint32_t version       (offset 12) - 0xfc6ed0ab
 *   uint32_t reserved[28]  (offset 16-127)
 *
 * Data section starts at offset 128.
 * All offsets within prop_bt/prop_info are relative to data start.
 *
 * prop_bt (binary trie node):
 *   uint32_t namelen
 *   uint32_t prop       - offset to prop_info (0 = no property here)
 *   uint32_t left       - offset to left child
 *   uint32_t right      - offset to right child
 *   uint32_t children   - offset to children
 *   char     name[namelen]
 *
 * prop_info:
 *   uint32_t serial     - bits [31:1] count, bit [0] dirty flag
 *   char     value[92]  - PROP_VALUE_MAX
 *   char     name[]     - full property name (null-terminated)
 */

#define PROP_AREA_MAGIC       0x504f5250
#define PROP_AREA_HEADER_SIZE 128

struct susfs_prop_area_header {
	uint32_t bytes_used;
	uint32_t serial;
	uint32_t magic;
	uint32_t version;
	uint32_t reserved[28];
};

struct susfs_prop_bt {
	uint32_t namelen;
	uint32_t prop;
	uint32_t left;
	uint32_t right;
	uint32_t children;
	/* char name[namelen] follows */
};

#define SUSFS_PROP_BT_FIXED_SIZE (sizeof(uint32_t) * 5)

/* Stack-based iterative tree walk to sanitize prop_info serial counters */
static int susfs_sanitize_prop_tree(char *data, uint32_t bytes_used)
{
	uint32_t stack[256];
	int sp = 0, prop_count = 0;
	struct susfs_prop_bt *node;
	uint32_t *serial_ptr;
	uint32_t offset;

	if (bytes_used <= SUSFS_PROP_BT_FIXED_SIZE)
		return 0;

	/* Push root node at offset 0 in data section */
	stack[sp++] = 0;

	while (sp > 0) {
		offset = stack[--sp];

		/* Validate offset is within bounds */
		if (offset + SUSFS_PROP_BT_FIXED_SIZE > bytes_used)
			continue;

		node = (struct susfs_prop_bt *)(data + offset);

		/* Push child nodes if valid (check bounds and avoid re-visiting offset 0
		 * which is the root we already processed) */
		if (node->children != 0 && node->children + SUSFS_PROP_BT_FIXED_SIZE <= bytes_used && sp < 255)
			stack[sp++] = node->children;
		if (node->right != 0 && node->right + SUSFS_PROP_BT_FIXED_SIZE <= bytes_used && sp < 255)
			stack[sp++] = node->right;
		if (node->left != 0 && node->left + SUSFS_PROP_BT_FIXED_SIZE <= bytes_used && sp < 255)
			stack[sp++] = node->left;

		/* If this node has a property, sanitize its serial.
		 *
		 * Bionic prop_info serial format:
		 *   bits 24-31: value length (SERIAL_VALUE_LEN)
		 *   bits  1-23: update counter (increments by 2 per update)
		 *   bit      0: dirty flag
		 *
		 * For properties loaded from files at boot (ro.*), counter = 0.
		 * Preserve the length bits, reset counter and dirty to 0.
		 */
		if (node->prop != 0 && node->prop + sizeof(uint32_t) <= bytes_used) {
			serial_ptr = (uint32_t *)(data + node->prop);
			*serial_ptr = *serial_ptr & 0xFF000000;
			prop_count++;
		}
	}

	return prop_count;
}

struct susfs_prop_dir_ctx {
	struct dir_context ctx;
	char *dir_path;
	int total_files;
	int total_props;
	int errors;
};

static int susfs_prop_dir_filldir(struct dir_context *ctx, const char *name,
				  int namlen, loff_t offset, u64 ino,
				  unsigned int d_type)
{
	struct susfs_prop_dir_ctx *pctx =
		container_of(ctx, struct susfs_prop_dir_ctx, ctx);
	char filepath[SUSFS_MAX_LEN_PATHNAME];
	struct file *fp;
	struct susfs_prop_area_header header;
	char *buf = NULL;
	loff_t pos;
	ssize_t ret;
	loff_t file_size;
	int prop_count;

	/* Skip . and .. */
	if (name[0] == '.' && (namlen == 1 || (namlen == 2 && name[1] == '.')))
		return 0;

	/* Only process regular files */
	if (d_type != DT_REG && d_type != DT_UNKNOWN)
		return 0;

	/* Build full path */
	ret = snprintf(filepath, sizeof(filepath), "%s/%.*s",
		       pctx->dir_path, namlen, name);
	if (ret < 0 || ret >= sizeof(filepath)) {
		pctx->errors++;
		return 0;
	}

	fp = filp_open(filepath, O_RDWR, 0);
	if (IS_ERR(fp)) {
		SUSFS_LOGE("Failed to open property file: '%s'\n", filepath);
		pctx->errors++;
		return 0;
	}

	/* Read header to validate */
	pos = 0;
	ret = kernel_read(fp, &header, sizeof(header), &pos);
	if (ret != sizeof(header)) {
		SUSFS_LOGE("Failed to read header from '%s'\n", filepath);
		pctx->errors++;
		goto out_close;
	}

	/* Validate magic */
	if (header.magic != PROP_AREA_MAGIC) {
		/* Not a property area file, skip silently */
		goto out_close;
	}

	/* Validate bytes_used is reasonable */
	file_size = i_size_read(file_inode(fp));
	if (file_size < PROP_AREA_HEADER_SIZE ||
	    header.bytes_used > (file_size - PROP_AREA_HEADER_SIZE)) {
		SUSFS_LOGE("Invalid bytes_used in '%s'\n", filepath);
		pctx->errors++;
		goto out_close;
	}

	if (header.bytes_used == 0) {
		/* Empty property area, skip */
		goto out_close;
	}

	/* Allocate buffer for data section */
	buf = kmalloc(header.bytes_used, GFP_KERNEL);
	if (!buf) {
		SUSFS_LOGE("No memory for property data from '%s'\n", filepath);
		pctx->errors++;
		goto out_close;
	}

	/* Read data section */
	pos = PROP_AREA_HEADER_SIZE;
	ret = kernel_read(fp, buf, header.bytes_used, &pos);
	if (ret != header.bytes_used) {
		SUSFS_LOGE("Failed to read data from '%s' (got %zd, expected %u)\n",
			   filepath, ret, header.bytes_used);
		pctx->errors++;
		goto out_free;
	}

	/* Sanitize the property tree */
	prop_count = susfs_sanitize_prop_tree(buf, header.bytes_used);

	if (prop_count > 0) {
		/* Write sanitized data back */
		pos = PROP_AREA_HEADER_SIZE;
		ret = kernel_write(fp, buf, header.bytes_used, &pos);
		if (ret != header.bytes_used) {
			SUSFS_LOGE("Failed to write data to '%s'\n", filepath);
			pctx->errors++;
			goto out_free;
		}

		/* Reset global serial to prop_count (each prop set increments by 1) */
		header.serial = prop_count;
		pos = 0;
		ret = kernel_write(fp, &header, sizeof(header), &pos);
		if (ret != sizeof(header)) {
			SUSFS_LOGE("Failed to write header to '%s'\n", filepath);
			pctx->errors++;
			goto out_free;
		}

		pctx->total_props += prop_count;
		pctx->total_files++;
		SUSFS_LOGI("Sanitized '%s': %d properties\n", filepath, prop_count);
	}

out_free:
	kfree(buf);
out_close:
	filp_close(fp, NULL);
	return 0;
}

void susfs_hide_resetprop_traces(void __user **user_info)
{
	struct st_susfs_hide_resetprop_traces info;
	struct file *dir;
	struct susfs_prop_dir_ctx pctx = {
		.ctx.actor = susfs_prop_dir_filldir,
		.total_files = 0,
		.total_props = 0,
		.errors = 0,
	};

	if (copy_from_user(&info, *user_info, sizeof(info))) {
		SUSFS_LOGE("failed copying from userspace\n");
		info.err = -EFAULT;
		goto out_copy_to_user;
	}

	/* Ensure null termination */
	info.prop_dir[SUSFS_MAX_LEN_PATHNAME - 1] = '\0';

	SUSFS_LOGI("Sanitizing property areas in: '%s'\n", info.prop_dir);

	dir = filp_open(info.prop_dir, O_RDONLY | O_DIRECTORY, 0);
	if (IS_ERR(dir)) {
		SUSFS_LOGE("Failed to open directory: '%s'\n", info.prop_dir);
		info.err = PTR_ERR(dir);
		goto out_copy_to_user;
	}

	pctx.dir_path = info.prop_dir;
	/* Remove trailing slash if present */
	{
		size_t len = strlen(info.prop_dir);
		if (len > 1 && info.prop_dir[len - 1] == '/')
			info.prop_dir[len - 1] = '\0';
	}

	iterate_dir(dir, &pctx.ctx);

	filp_close(dir, NULL);

	SUSFS_LOGI("Done: %d files, %d properties sanitized, %d errors\n",
		   pctx.total_files, pctx.total_props, pctx.errors);

	info.err = pctx.errors ? -EIO : 0;

out_copy_to_user:
	if (copy_to_user(*user_info, &info, sizeof(info)))
		SUSFS_LOGE("copy_to_user() failed\n");
}
#endif // #ifdef CONFIG_KSU_SUSFS_HIDE_RESETPROP_TRACES

/* susfs avc log spoofing */
extern bool susfs_is_avc_log_spoofing_enabled;
void susfs_set_avc_log_spoofing(void __user **user_info) {
	struct st_susfs_avc_log_spoofing info;

	if (copy_from_user(&info, *user_info, sizeof(info))) {
		SUSFS_LOGE("failed copying from userspace\n");
		info.err = -EFAULT;
		goto out_copy_to_user;
	}

	susfs_is_avc_log_spoofing_enabled = info.enabled;
	SUSFS_LOGI("enabled: %d\n", info.enabled);
	info.err = 0;

out_copy_to_user:
	if (copy_to_user(*user_info, &info, sizeof(info)))
		SUSFS_LOGE("copy_to_user() failed\n");
}

/* enabled features */
static int copy_config_to_buf(const char *config_string, char *buf_ptr, size_t *copied_size, size_t bufsize) {
	size_t tmp_size = strlen(config_string);

	*copied_size += tmp_size;
	if (*copied_size >= bufsize) {
		SUSFS_LOGE("bufsize is not big enough to hold the string.\n");
		return -EINVAL;
	}
	memcpy(buf_ptr, config_string, tmp_size);
	return 0;
}

void susfs_get_enabled_features(void __user **user_info) {
	struct st_susfs_enabled_features *info = NULL;
	char *buf_ptr = NULL;
	size_t copied_size = 0;
	int err = 0;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		return;
	}

	buf_ptr = info->enabled_features;
#ifdef CONFIG_KSU_SUSFS_SUS_PATH
	err = copy_config_to_buf("CONFIG_KSU_SUSFS_SUS_PATH\n", buf_ptr, &copied_size, SUSFS_ENABLED_FEATURES_SIZE);
	if (err) goto out;
	buf_ptr = info->enabled_features + copied_size;
#endif
#ifdef CONFIG_KSU_SUSFS_SUS_MOUNT
	err = copy_config_to_buf("CONFIG_KSU_SUSFS_SUS_MOUNT\n", buf_ptr, &copied_size, SUSFS_ENABLED_FEATURES_SIZE);
	if (err) goto out;
	buf_ptr = info->enabled_features + copied_size;
#endif
#ifdef CONFIG_KSU_SUSFS_SUS_KSTAT
	err = copy_config_to_buf("CONFIG_KSU_SUSFS_SUS_KSTAT\n", buf_ptr, &copied_size, SUSFS_ENABLED_FEATURES_SIZE);
	if (err) goto out;
	buf_ptr = info->enabled_features + copied_size;
#endif
#ifdef CONFIG_KSU_SUSFS_SPOOF_UNAME
	err = copy_config_to_buf("CONFIG_KSU_SUSFS_SPOOF_UNAME\n", buf_ptr, &copied_size, SUSFS_ENABLED_FEATURES_SIZE);
	if (err) goto out;
	buf_ptr = info->enabled_features + copied_size;
#endif
#ifdef CONFIG_KSU_SUSFS_ENABLE_LOG
	err = copy_config_to_buf("CONFIG_KSU_SUSFS_ENABLE_LOG\n", buf_ptr, &copied_size, SUSFS_ENABLED_FEATURES_SIZE);
	if (err) goto out;
	buf_ptr = info->enabled_features + copied_size;
#endif
#ifdef CONFIG_KSU_SUSFS_HIDE_KSU_SUSFS_SYMBOLS
	err = copy_config_to_buf("CONFIG_KSU_SUSFS_HIDE_KSU_SUSFS_SYMBOLS\n", buf_ptr, &copied_size, SUSFS_ENABLED_FEATURES_SIZE);
	if (err) goto out;
	buf_ptr = info->enabled_features + copied_size;
#endif
#ifdef CONFIG_KSU_SUSFS_SPOOF_CMDLINE_OR_BOOTCONFIG
	err = copy_config_to_buf("CONFIG_KSU_SUSFS_SPOOF_CMDLINE_OR_BOOTCONFIG\n", buf_ptr, &copied_size, SUSFS_ENABLED_FEATURES_SIZE);
	if (err) goto out;
	buf_ptr = info->enabled_features + copied_size;
#endif
#ifdef CONFIG_KSU_SUSFS_OPEN_REDIRECT
	err = copy_config_to_buf("CONFIG_KSU_SUSFS_OPEN_REDIRECT\n", buf_ptr, &copied_size, SUSFS_ENABLED_FEATURES_SIZE);
	if (err) goto out;
	buf_ptr = info->enabled_features + copied_size;
#endif
#ifdef CONFIG_KSU_SUSFS_SUS_MAP
	err = copy_config_to_buf("CONFIG_KSU_SUSFS_SUS_MAP\n", buf_ptr, &copied_size, SUSFS_ENABLED_FEATURES_SIZE);
	if (err) goto out;
	buf_ptr = info->enabled_features + copied_size;
#endif
#ifdef CONFIG_KSU_SUSFS_HIDE_RESETPROP_TRACES
	err = copy_config_to_buf("CONFIG_KSU_SUSFS_HIDE_RESETPROP_TRACES\n", buf_ptr, &copied_size, SUSFS_ENABLED_FEATURES_SIZE);
	if (err) goto out;
	buf_ptr = info->enabled_features + copied_size;
#endif
	info->err = 0;
out:
	if (err) info->err = err;
	if (copy_to_user(*user_info, info, sizeof(*info)))
		SUSFS_LOGE("copy_to_user() failed\n");
	kfree(info);
}

/* show variant */
void susfs_show_variant(void __user **user_info) {
	struct st_susfs_variant info;
	memset(&info, 0, sizeof(info));
	strscpy(info.susfs_variant, SUSFS_VARIANT, sizeof(info.susfs_variant));
	info.err = 0;
	if (copy_to_user(*user_info, &info, sizeof(info)))
		SUSFS_LOGE("copy_to_user() failed\n");
}

/* show version */
void susfs_show_version(void __user **user_info) {
	struct st_susfs_version info;
	memset(&info, 0, sizeof(info));
	strscpy(info.susfs_version, SUSFS_VERSION, sizeof(info.susfs_version));
	info.err = 0;
	if (copy_to_user(*user_info, &info, sizeof(info)))
		SUSFS_LOGE("copy_to_user() failed\n");
}

/* sdcard monitor via fsnotify + deferred workqueue cleanup */
#define SDCARD_ANDROID_PATH "/data/media/0/Android"
extern void setup_selinux(const char *domain, struct cred *cred);
extern bool susfs_is_current_ksu_domain(void);
bool susfs_is_sdcard_android_data_decrypted __read_mostly = false;

struct watch_dir {
	const char *path;
	u32 mask;
	struct path kpath;
	struct inode *inode;
	struct fsnotify_mark *mark;
};

static struct fsnotify_group *sdcard_fsnotify_group;

static struct watch_dir sdcard_watch = {
	.path = "/data/media/0",
	.mask = (FS_CREATE | FS_MOVED_TO | FS_EVENT_ON_CHILD | FS_ISDIR),
};

static int sdcard_add_mark_on_inode(struct inode *inode, u32 mask,
				    struct fsnotify_mark **out);

static unsigned long sdcard_cleanup_scheduled;
static struct delayed_work sdcard_cleanup_dwork;

static void susfs_sdcard_cleanup_fn(struct work_struct *work)
{
	struct fsnotify_group *grp;
	struct inode *inode;

	SUSFS_LOGI("set susfs_is_sdcard_android_data_decrypted to true\n");
	WRITE_ONCE(susfs_is_sdcard_android_data_decrypted, true);

	SUSFS_LOGI("cleaning up fsnotify sdcard watch\n");

	grp = xchg(&sdcard_fsnotify_group, NULL);
	if (grp)
		fsnotify_destroy_group(grp);

	inode = xchg(&sdcard_watch.inode, NULL);
	if (inode)
		iput(inode);

	if (sdcard_watch.kpath.mnt) {
		path_put(&sdcard_watch.kpath);
		memset(&sdcard_watch.kpath, 0, sizeof(sdcard_watch.kpath));
	}
}

/*
 * fsnotify handler - runs inside an SRCU read section held by fsnotify().
 * Must not block or call fsnotify_destroy_group() (which internally calls
 * synchronize_srcu on the same SRCU struct, causing a permanent deadlock).
 * Cleanup is deferred to a delayed_work that runs outside the SRCU context.
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 9, 0)
static int susfs_handle_sdcard_inode_event(struct fsnotify_mark *mark, u32 mask,
					   struct inode *inode, struct inode *dir,
					   const struct qstr *file_name, u32 cookie)
{
	if (!file_name || file_name->len != 7 ||
	    memcmp(file_name->name, "Android", 7))
		return 0;

	if (test_and_set_bit(0, &sdcard_cleanup_scheduled))
		return 0;

	SUSFS_LOGI("'%s' detected, mask: 0x%x\n", SDCARD_ANDROID_PATH, mask);
	SUSFS_LOGI("deferring cleanup for 5 seconds\n");
	queue_delayed_work(system_unbound_wq, &sdcard_cleanup_dwork, 5 * HZ);
	return 0;
}

static const struct fsnotify_ops susfs_sdcard_ops = {
	.handle_inode_event = susfs_handle_sdcard_inode_event,
};
#else
static int susfs_handle_sdcard_event(struct fsnotify_group *group,
				     struct inode *inode,
				     u32 mask, const void *data, int data_type,
				     const struct qstr *file_name, u32 cookie,
				     struct fsnotify_iter_info *iter_info)
{
	if (!file_name || file_name->len != 7 ||
	    memcmp(file_name->name, "Android", 7))
		return 0;

	if (test_and_set_bit(0, &sdcard_cleanup_scheduled))
		return 0;

	SUSFS_LOGI("'%s' detected, mask: 0x%x\n", SDCARD_ANDROID_PATH, mask);
	SUSFS_LOGI("deferring cleanup for 5 seconds\n");
	queue_delayed_work(system_unbound_wq, &sdcard_cleanup_dwork, 5 * HZ);
	return 0;
}

static const struct fsnotify_ops susfs_sdcard_ops = {
	.handle_event = susfs_handle_sdcard_event,
};
#endif

static int sdcard_add_mark_on_inode(struct inode *inode, u32 mask,
				    struct fsnotify_mark **out)
{
	struct fsnotify_mark *m;

	m = kzalloc(sizeof(*m), GFP_KERNEL);
	if (!m)
		return -ENOMEM;

	fsnotify_init_mark(m, sdcard_fsnotify_group);
	m->mask = mask;

	if (fsnotify_add_inode_mark(m, inode, 0)) {
		fsnotify_put_mark(m);
		return -EINVAL;
	}
	*out = m;
	return 0;
}

static int sdcard_watch_one_dir(struct watch_dir *wd)
{
	int ret = kern_path(wd->path, LOOKUP_FOLLOW, &wd->kpath);
	if (ret) {
		SUSFS_LOGI("path not ready: %s (%d)\n", wd->path, ret);
		return ret;
	}
	wd->inode = d_backing_inode(wd->kpath.dentry);
	if (!wd->inode) {
		SUSFS_LOGE("inode is NULL for %s\n", wd->path);
		path_put(&wd->kpath);
		return -ENOENT;
	}
	ihold(wd->inode);

	ret = sdcard_add_mark_on_inode(wd->inode, wd->mask, &wd->mark);
	if (ret) {
		SUSFS_LOGE("add mark failed for %s (%d)\n", wd->path, ret);
		iput(wd->inode);
		wd->inode = NULL;
		path_put(&wd->kpath);
		return ret;
	}
	SUSFS_LOGI("watching %s\n", wd->path);
	return 0;
}

static int susfs_sdcard_monitor_thread(void *data)
{
	struct cred *cred = prepare_creds();
	int ret;

	if (!cred) {
		SUSFS_LOGE("failed to prepare creds!\n");
		return -ENOMEM;
	}

	setup_selinux("u:r:su:s0", cred);
	commit_creds(cred);

	SUSFS_LOGI("start monitoring path '%s' using fsnotify\n", SDCARD_ANDROID_PATH);

	INIT_DELAYED_WORK(&sdcard_cleanup_dwork, susfs_sdcard_cleanup_fn);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)
	sdcard_fsnotify_group = fsnotify_alloc_group(&susfs_sdcard_ops, 0);
#else
	sdcard_fsnotify_group = fsnotify_alloc_group(&susfs_sdcard_ops);
#endif
	if (IS_ERR(sdcard_fsnotify_group)) {
		ret = PTR_ERR(sdcard_fsnotify_group);
		sdcard_fsnotify_group = NULL;
		return ret;
	}

	ret = sdcard_watch_one_dir(&sdcard_watch);
	SUSFS_LOGI("sdcard monitor ret: %d\n", ret);

	return 0;
}

int susfs_start_sdcard_monitor_fn(void) {
	if (IS_ERR(kthread_run(susfs_sdcard_monitor_thread, NULL, "media_monitor"))) {
		SUSFS_LOGE("failed to create sdcard monitor thread\n");
		SUSFS_LOGI("set susfs_is_sdcard_android_data_decrypted to true\n");
		susfs_is_sdcard_android_data_decrypted = true;
		return -1;
	}
	return 0;
}

/* susfs auto-init file creation helper */
int susfs_create_file_with_content(const char *filepath, const char *content, size_t len) {
	struct file *filp = NULL;
	struct path parent_path;
	struct dentry *dentry = NULL;
	struct inode *dir = NULL;
	loff_t pos = 0;
	int ret = 0;

	/* Create /data/adb/.susfs/ directory if needed */
	ret = kern_path("/data/adb", LOOKUP_FOLLOW, &parent_path);
	if (ret) {
		SUSFS_LOGE("create_file: /data/adb not found\n");
		return ret;
	}
	dir = d_inode(parent_path.dentry);
	inode_lock(dir);
	dentry = lookup_one_len(".susfs", parent_path.dentry, 6);
	if (!IS_ERR(dentry)) {
		if (d_is_negative(dentry))
			vfs_mkdir(dir, dentry, 0700);
		dput(dentry);
	}
	inode_unlock(dir);
	path_put(&parent_path);

	/* Create and write the file */
	filp = filp_open(filepath, O_WRONLY | O_CREAT | O_TRUNC, 0644);
	if (IS_ERR(filp)) {
		SUSFS_LOGE("create_file: failed to open '%s': %ld\n", filepath, PTR_ERR(filp));
		return PTR_ERR(filp);
	}

	ret = kernel_write(filp, content, len, &pos);
	filp_close(filp, NULL);

	if (ret < 0) {
		SUSFS_LOGE("create_file: failed to write '%s': %d\n", filepath, ret);
		return ret;
	}

	SUSFS_LOGI("create_file: '%s' created (%zu bytes)\n", filepath, len);
	return 0;
}

/* susfs_init */
int susfs_init(void) {
#ifdef CONFIG_KSU_SUSFS_SPOOF_UNAME
	susfs_my_uname_init();
#endif
	SUSFS_LOGI("susfs is initialized! version: " SUSFS_VERSION " \n");
	return 0;
}
