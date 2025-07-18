/**
 * @file
 *
 * @ingroup DOSFS
 *
 * @brief Init Routine for MSDOS
 */

/*
 *  Copyright (C) 2001 OKTET Ltd., St.-Petersburg, Russia
 *  Author: Eugeny S. Mints <Eugeny.Mints@oktet.ru>
 *
 *  Modifications to support reference counting in the file system are
 *  Copyright (c) 2012 embedded brains GmbH & Co. KG
 *
 *  Modifications to support UTF-8 in the file system are
 *  Copyright (c) 2013 embedded brains GmbH & Co. KG
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <rtems/libio_.h>
#include <rtems/dosfs.h>
#include "msdos.h"

static int msdos_clone_node_info(rtems_filesystem_location_info_t *loc)
{
    fat_file_fd_t *fat_fd = loc->node_access;

    return fat_file_reopen(fat_fd);
}

static int msdos_utimens(
  const rtems_filesystem_location_info_t *loc,
  struct timespec                         times[2]
)
{
    fat_file_fd_t *fat_fd = loc->node_access;

    if (times[0].tv_sec != times[1].tv_sec)
        rtems_set_errno_and_return_minus_one( ENOTSUP );

    fat_file_set_mtime(fat_fd, times[1].tv_sec);

    return RC_OK;
}

const rtems_filesystem_operations_table  msdos_ops = {
  .lock_h         =  msdos_lock,
  .unlock_h       =  msdos_unlock,
  .eval_path_h    =  msdos_eval_path,
  .link_h         =  rtems_filesystem_default_link,
  .are_nodes_equal_h = rtems_filesystem_default_are_nodes_equal,
  .mknod_h        =  msdos_mknod,
  .rmnod_h        =  msdos_rmnod,
  .fchmod_h       =  rtems_filesystem_default_fchmod,
  .chown_h        =  rtems_filesystem_default_chown,
  .clonenod_h     =  msdos_clone_node_info,
  .freenod_h      =  msdos_free_node_info,
  .mount_h        =  rtems_dosfs_initialize,
  .unmount_h      =  rtems_filesystem_default_unmount,
  .fsunmount_me_h =  msdos_shut_down,
  .utimens_h      =  msdos_utimens,
  .symlink_h      =  rtems_filesystem_default_symlink,
  .readlink_h     =  rtems_filesystem_default_readlink,
  .rename_h       =  msdos_rename,
  .statvfs_h      =  msdos_statvfs
};

void msdos_lock(const rtems_filesystem_mount_table_entry_t *mt_entry)
{
  msdos_fs_lock(mt_entry->fs_info);
}

void msdos_unlock(const rtems_filesystem_mount_table_entry_t *mt_entry)
{
  msdos_fs_unlock(mt_entry->fs_info);
}

/* msdos_initialize --
 *     MSDOS filesystem initialization. Called when mounting an
 *     MSDOS filesystem.
 *
 * PARAMETERS:
 *     temp_mt_entry - mount table entry
 *
 * RETURNS:
 *     RC_OK on success, or -1 if error occurred (errno set apropriately).
 *
 */
int rtems_dosfs_initialize(
  rtems_filesystem_mount_table_entry_t *mt_entry,
  const void                           *data
)
{
    int                                rc = 0;
    const rtems_dosfs_mount_options   *mount_options = data;
    rtems_dosfs_convert_control       *converter;
    bool                               converter_created = false;


    if (mount_options == NULL || mount_options->converter == NULL) {
        converter = rtems_dosfs_create_default_converter();
        converter_created = true;
    } else {
        converter = mount_options->converter;
    }

    if (converter != NULL) {
        rc = msdos_initialize_support(mt_entry,
                                      &msdos_ops,
                                      &msdos_file_handlers,
                                      &msdos_dir_handlers,
                                      converter);
        if (rc != 0 && converter_created) {
            (*converter->handler->destroy)(converter);
        }
    } else {
        errno = ENOMEM;
        rc = -1;
    }

    return rc;
}
