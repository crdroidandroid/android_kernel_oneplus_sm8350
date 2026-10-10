#ifndef __KSU_H_UID_OBSERVER
#define __KSU_H_UID_OBSERVER

#include <linux/types.h>
#ifdef CONFIG_KSU_DISABLE_MANAGER
static inline void ksu_throne_tracker_init()
{
}

static inline void ksu_throne_tracker_exit()
{
}

static inline void track_throne(bool prune_only)
{
    (void)prune_only;
}

static inline void ksu_defer_track_throne(bool prune_only)
{
    (void)prune_only;
}
#else
void ksu_throne_tracker_init();

void ksu_throne_tracker_exit();

void track_throne(bool prune_only);

/*
 * fsnotify-safe variant of track_throne(): run it from a workqueue instead of
 * the caller's context. The hooks that watch /data/system are called from
 * vfs_rename()/vfs_create() with the directory's i_rwsem held for writing, and
 * track_throne() opens a file in that same directory, so an inline call
 * self-deadlocks. Never call track_throne() directly from fsnotify.
 */
void ksu_defer_track_throne(bool prune_only);
#endif

#endif
