#include <rtems.h>
#include <rtems/libio.h>
#include <rtems/diskdevs.h>
#include <rtems/bdbuf.h>
#include <rtems/error.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include "diskio.h"
#include "ff.h"

#define MAX_DRIVES 4

typedef struct
{
    bool initialized;
    bool present;
    bool write_protected;
    char device_path[64];
    rtems_disk_device *dd;
    int fd;
} drive_info_t;

static drive_info_t drives[MAX_DRIVES];
static bool diskio_initialized = false;

DSTATUS disk_initialize(BYTE pdrv)
{
    rtems_status_code sc;
    // TODO: Is there a way to convert Physical drive number to device path??

    if (pdrv >= MAX_DRIVES)
    {
        return status;
    }

    if (!diskio_initialized)
    {
        diskio_initialized = true;
        sc = rtems_disk_io_initialize();
        if (sc != RTEMS_SUCCESSFUL)
        {
            return STA_NOINIT;
        }
    }
    // Map pdrv to a device instance

    // Open or probe the block device using RTEMS IO layer

    // Perform init step (sending CMD0/CMD1 for SD cards, etc.)

    // Return RES_OK on success, or appropriate error (STA_NOINIT, STA_NODISK)
    return 0;
}

DSTATUS disk_status(BYTE pdrv)
{
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count)
{
    rtems_status_code sc;
    rtems_bdbuf_buffer *bd;

    sc = rtems_bdbuf_read(drives[pdrv].dd, sector, bd);

    if (sc != RTEMS_SUCCESSFUL)
    {
        return RES_ERROR;
    }
    if (bd == NULL)
    {
        return RES_NOTRDY;
    }

    if (bd->state == RTEMS_BDBUF_STATE_ACCESS_MODIFIED)
    {
        // If the buffer is modified, we need to read it from the disk
        sc = rtems_bdbuf_sync(bd);
        if (sc != RTEMS_SUCCESSFUL)
        {
            return RES_ERROR;
        }
    }
    memcpy(buff, bd->buffer, count * drives[pdrv].dd->block_size); // TODO: Is this a correct way to handle count?
    return RES_OK;
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count)
{
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    uint32_t req;
    switch (cmd)
    {
    case CTRL_SYNC:
        req = RTEMS_BLKIO_SYNCDEV;
        break;
    case GET_SECTOR_COUNT:
        req = RTEMS_BLKIO_GETSIZE;
        break;
    case GET_SECTOR_SIZE:
        req = RTEMS_BLKIO_GETMEDIABLKSIZE;
        break;
    case GET_BLOCK_SIZE:
        req = RTEMS_BLKIO_GETBLKSIZE;
        break;
    case CTRL_TRIM:
        // TODO: Handle trim operation if supported
        return RES_OK;
    default:
        return RES_PARERR;
    }

    rtems_status_code sc;
    sc = rtems_blkdev_ioctl(drives[pdrv].dd, req, buff); // TODO: Is the buffer pointer correct here?
    if (sc != RTEMS_SUCCESSFUL)
    {
        return RES_ERROR;
    }
    // TODO: Handle specific commands if needed
    return RES_OK;
}
