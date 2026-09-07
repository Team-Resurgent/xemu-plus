/*
 * Virtual XISO from a host directory - QEMU block driver.
 *
 * Thin adapter over the in-tree libxdvdfs (libxdvdfs/). The library scans a
 * folder and serves XDVDFS sectors; this file only bridges that to QEMU.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "block/block_int.h"
#include "block/block-io.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/module.h"
#include "qemu/option.h"
#include "qemu/iov.h"
#include "qemu/ctype.h"
#include "qemu/cutils.h"
#include "qemu/memalign.h"
#include "qobject/qdict.h"

#include "xdvdfs/xdvdfs.h"

typedef struct BDRVXdvdfsState {
    xdvdfs_dir *dir;
    char *path;
} BDRVXdvdfsState;

static QemuOptsList runtime_opts = {
    .name = "xdvdfs",
    .head = QTAILQ_HEAD_INITIALIZER(runtime_opts.head),
    .desc = {
        {
            .name = "filename",
            .type = QEMU_OPT_STRING,
            .help = "Host directory to present as an XISO",
        },
        { /* end of list */ }
    },
};

static int xdvdfs_to_errno(int e)
{
    switch (e) {
    case XDVDFS_OK:
        return 0;
    case XDVDFS_ERR_NOMEM:
        return -ENOMEM;
    case XDVDFS_ERR_IO:
        return -EIO;
    case XDVDFS_ERR_RANGE:
        return -EINVAL;
    case XDVDFS_ERR_NOTFOUND:
        return -ENOENT;
    case XDVDFS_ERR_NOTDIR:
        return -ENOTDIR;
    default:
        return -EINVAL;
    }
}

static void xdvdfs_parse_filename(const char *filename, QDict *options,
                                  Error **errp)
{
    const char *path = filename;

    if (strstart(filename, "xdvdfs:", &path)) {
        const char *last = strrchr(filename, ':');
        /* xdvdfs:F:\dir — last colon is a DOS drive letter */
        if (last && last[1] == '\\' && last > filename &&
            qemu_isalpha(last[-1])) {
            path = last - 1;
        }
    }
    qdict_put_str(options, "filename", path);
}

static int xdvdfs_open(BlockDriverState *bs, QDict *options, int flags,
                       Error **errp)
{
    BDRVXdvdfsState *s = bs->opaque;
    QemuOpts *opts;
    const char *dirname;
    int ret;

    GLOBAL_STATE_CODE();
    memset(s, 0, sizeof(*s));

    opts = qemu_opts_create(&runtime_opts, NULL, 0, &error_abort);
    if (!qemu_opts_absorb_qdict(opts, options, errp)) {
        qemu_opts_del(opts);
        return -EINVAL;
    }

    dirname = qemu_opt_get(opts, "filename");
    if (!dirname || !dirname[0]) {
        error_setg(errp, "xdvdfs requires a directory path");
        qemu_opts_del(opts);
        return -EINVAL;
    }

    GRAPH_RDLOCK_GUARD_MAINLOOP();

    ret = xdvdfs_dir_open(&s->dir, dirname);
    if (ret != XDVDFS_OK) {
        error_setg(errp, "XDVDFS: %s (%s)", xdvdfs_strerror(ret), dirname);
        qemu_opts_del(opts);
        return xdvdfs_to_errno(ret);
    }

    s->path = g_strdup(dirname);
    qemu_opts_del(opts);

    snprintf(bs->exact_filename, sizeof(bs->exact_filename), "xdvdfs:%s",
             s->path);

    bs->sg = false;
    bs->supported_write_flags = BDRV_REQ_WRITE_UNCHANGED;
    bs->supported_zero_flags = BDRV_REQ_WRITE_UNCHANGED;
    return 0;
}

static void xdvdfs_close(BlockDriverState *bs)
{
    BDRVXdvdfsState *s = bs->opaque;
    xdvdfs_dir_free(s->dir);
    s->dir = NULL;
    g_free(s->path);
    s->path = NULL;
}

static int coroutine_fn GRAPH_RDLOCK
xdvdfs_co_preadv(BlockDriverState *bs, int64_t offset, int64_t bytes,
                 QEMUIOVector *qiov, BdrvRequestFlags flags)
{
    BDRVXdvdfsState *s = bs->opaque;
    uint8_t *secbuf;
    int64_t end;
    uint64_t first_lba, last_lba, lba;
    int ret = 0;

    if (offset < 0 || bytes < 0) {
        return -EINVAL;
    }
    if (bytes == 0) {
        return 0;
    }
    end = offset + bytes;
    if (end > (int64_t)xdvdfs_dir_size(s->dir)) {
        return -EINVAL;
    }

    secbuf = qemu_try_blockalign(bs, XDVDFS_SECTOR_SIZE);
    if (!secbuf) {
        return -ENOMEM;
    }

    first_lba = (uint64_t)offset / XDVDFS_SECTOR_SIZE;
    last_lba = (uint64_t)(end - 1) / XDVDFS_SECTOR_SIZE;

    for (lba = first_lba; lba <= last_lba; lba++) {
        int64_t sec_start = (int64_t)lba * XDVDFS_SECTOR_SIZE;
        int64_t copy_start = MAX(sec_start, offset);
        int64_t copy_end = MIN(sec_start + XDVDFS_SECTOR_SIZE, end);
        size_t sec_off = copy_start - sec_start;

        ret = xdvdfs_dir_read_sector(s->dir, lba, secbuf);
        if (ret != XDVDFS_OK) {
            ret = xdvdfs_to_errno(ret);
            goto out;
        }
        qemu_iovec_from_buf(qiov, copy_start - offset, secbuf + sec_off,
                            copy_end - copy_start);
    }
    ret = 0;

out:
    qemu_vfree(secbuf);
    return ret;
}

static int coroutine_fn GRAPH_RDLOCK
xdvdfs_co_pwritev(BlockDriverState *bs, int64_t offset, int64_t bytes,
                  QEMUIOVector *qiov, BdrvRequestFlags flags)
{
    return -EROFS;
}

static int coroutine_fn GRAPH_RDLOCK
xdvdfs_co_block_status(BlockDriverState *bs, unsigned int mode, int64_t offset,
                       int64_t bytes, int64_t *pnum, int64_t *map,
                       BlockDriverState **file)
{
    *pnum = bytes;
    *map = -1;
    *file = NULL;
    return BDRV_BLOCK_DATA;
}

static int64_t coroutine_fn GRAPH_RDLOCK
xdvdfs_co_getlength(BlockDriverState *bs)
{
    BDRVXdvdfsState *s = bs->opaque;
    uint64_t sz = xdvdfs_dir_size(s->dir);
    return sz > INT64_MAX ? INT64_MAX : (int64_t)sz;
}

static void GRAPH_RDLOCK xdvdfs_refresh_limits(BlockDriverState *bs,
                                               Error **errp)
{
    bs->bl.request_alignment = XDVDFS_SECTOR_SIZE;
    bs->bl.has_variable_length = false;
}

static const char *const xdvdfs_strong_runtime_opts[] = {
    "filename",
    NULL
};

static BlockDriver bdrv_xdvdfs = {
    .format_name            = "xdvdfs",
    .protocol_name          = "xdvdfs",
    .instance_size          = sizeof(BDRVXdvdfsState),
    .bdrv_needs_filename    = true,
    .bdrv_parse_filename    = xdvdfs_parse_filename,
    .bdrv_open              = xdvdfs_open,
    .bdrv_close             = xdvdfs_close,
    .bdrv_co_preadv         = xdvdfs_co_preadv,
    .bdrv_co_pwritev        = xdvdfs_co_pwritev,
    .bdrv_co_block_status   = xdvdfs_co_block_status,
    .bdrv_co_getlength      = xdvdfs_co_getlength,
    .bdrv_refresh_limits    = xdvdfs_refresh_limits,
    .strong_runtime_opts    = xdvdfs_strong_runtime_opts,
};

static void bdrv_xdvdfs_init(void)
{
    bdrv_register(&bdrv_xdvdfs);
}

block_init(bdrv_xdvdfs_init);
