/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * libxdvdfs - present a host directory as a virtual Xbox XISO (XDVDFS).
 *
 * Sector layout matches extract-xiso create mode: 2048-byte sectors, volume
 * descriptor at LBA 32, root directory at LBA 0x108, directory tables as a
 * prefix-ordered BST. .xbe media-check bytes are patched in memory the same
 * way extract-xiso does on create (host files are never modified).
 */
#ifndef XDVDFS_H
#define XDVDFS_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define XDVDFS_VERSION_MAJOR 0
#define XDVDFS_VERSION_MINOR 1
#define XDVDFS_VERSION_PATCH 0

#define XDVDFS_SECTOR_SIZE 2048u
#define XDVDFS_VOLUME_LBA 32u
#define XDVDFS_ROOT_LBA 0x108u

enum {
    XDVDFS_OK           = 0,
    XDVDFS_ERR_ARG      = -1,
    XDVDFS_ERR_IO       = -2,
    XDVDFS_ERR_NOTDIR   = -3,
    XDVDFS_ERR_RANGE    = -4,
    XDVDFS_ERR_NOMEM    = -5,
    XDVDFS_ERR_NOTFOUND = -6,
};

const char *xdvdfs_strerror(int err);

typedef struct xdvdfs_dir xdvdfs_dir;

/* Scan @path and build a virtual XISO. Host files are not copied. */
int xdvdfs_dir_open(xdvdfs_dir **out, const char *path);
void xdvdfs_dir_free(xdvdfs_dir *d);

uint64_t xdvdfs_dir_sector_count(const xdvdfs_dir *d);
uint64_t xdvdfs_dir_size(const xdvdfs_dir *d);

/* Fill exactly XDVDFS_SECTOR_SIZE bytes. */
int xdvdfs_dir_read_sector(xdvdfs_dir *d, uint64_t lba,
                           uint8_t out[XDVDFS_SECTOR_SIZE]);
int xdvdfs_dir_read(xdvdfs_dir *d, uint64_t offset, void *buf, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* XDVDFS_H */
