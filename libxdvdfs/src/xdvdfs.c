/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Virtual XDVDFS (XISO) from a host directory.
 *
 * Directory tables follow extract-xiso: prefix-order AVL/BST layout, dword
 * left/right offsets from the start of the directory table, entries padded
 * to 4 bytes and never spanning a sector boundary. Unused table bytes are
 * 0xFF. Volume magic is MICROSOFT*XBOX*MEDIA at LBA 32.
 */

#include "xdvdfs/xdvdfs.h"

#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifdef _WIN32
#include <windows.h>
#include <fcntl.h>
#include <io.h>
#else
#include <dirent.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

#define XDVDFS_MAGIC "MICROSOFT*XBOX*MEDIA"
#define XDVDFS_MAGIC_LEN 20
#define XDVDFS_UNUSED 0x7c8
#define XDVDFS_ENTRY_HDR 14
#define XDVDFS_MAX_NAME 255
#define XDVDFS_MAX_DEPTH 16
#define XDVDFS_PAD 0xff
#define XDVDFS_ATTR_DIR 0x10
#define XDVDFS_ATTR_ARC 0x20
#define XDVDFS_MODULUS 0x10000u

/* extract-xiso media-enable patch: jge -> jmp */
static const uint8_t xbe_media_pat[] = {
    0xe8, 0xca, 0xfd, 0xff, 0xff, 0x85, 0xc0, 0x7d
};
#define XBE_MEDIA_PAT_LEN 8
#define XBE_MEDIA_PATCH_POS 7
#define XBE_MEDIA_PATCH_BYTE 0xeb

typedef struct xdvdfs_node xdvdfs_node;
typedef struct xdvdfs_extent xdvdfs_extent;

enum xdvdfs_extent_kind {
    XDVDFS_EXT_VOLUME = 1,
    XDVDFS_EXT_DIR,
    XDVDFS_EXT_FILE,
};

struct xdvdfs_node {
    char *name;
    char *host_path;
    int is_dir;
    uint32_t file_size;       /* file bytes, or packed dir-table size */
    uint32_t start_sector;
    uint32_t entry_offset;    /* byte offset in parent dir table */
    uint8_t *dir_table;
    uint32_t dir_table_bytes;
    int fd;
    uint32_t *xbe_patches;
    int n_xbe_patches;
    xdvdfs_node *left;
    xdvdfs_node *right;
    xdvdfs_node *child_tree;
    xdvdfs_node **children;
    int nchildren;
    int children_cap;
};

struct xdvdfs_extent {
    uint32_t start_sector;
    uint32_t num_sectors;
    int kind;
    xdvdfs_node *node;
};

struct xdvdfs_dir {
    xdvdfs_node root;
    uint8_t volume[XDVDFS_SECTOR_SIZE];
    uint64_t total_size;
    xdvdfs_extent *extents;
    int nextents;
};

static uint32_t n_sectors(uint32_t size)
{
    return size / XDVDFS_SECTOR_SIZE + (size % XDVDFS_SECTOR_SIZE ? 1u : 0u);
}

static uint32_t entry_length(const char *name)
{
    uint32_t length = XDVDFS_ENTRY_HDR + (uint32_t)strlen(name);
    return (length + 3u) & ~3u;
}

static void wr16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static void wr32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static void wr64(uint8_t *p, uint64_t v)
{
    wr32(p, (uint32_t)v);
    wr32(p + 4, (uint32_t)(v >> 32));
}

static int xdvdfs_casecmp(const char *a, const char *b)
{
    for (;;) {
        unsigned char ca = (unsigned char)*a++;
        unsigned char cb = (unsigned char)*b++;
        if (ca >= 'a' && ca <= 'z') {
            ca = (unsigned char)(ca - 32);
        }
        if (cb >= 'a' && cb <= 'z') {
            cb = (unsigned char)(cb - 32);
        }
        if (ca != cb) {
            return (int)ca - (int)cb;
        }
        if (!ca) {
            return 0;
        }
    }
}

static int name_is_xbe(const char *name)
{
    size_t n = strlen(name);
    return n >= 4 && xdvdfs_casecmp(name + n - 4, ".xbe") == 0;
}

static char *xdvdfs_strdup(const char *s)
{
    size_t n = strlen(s) + 1;
    char *d = malloc(n);
    if (d) {
        memcpy(d, s, n);
    }
    return d;
}

static char *xdvdfs_join(const char *dir, const char *name)
{
    size_t dl = strlen(dir);
    size_t nl = strlen(name);
    int need_sep = dl > 0 && dir[dl - 1] != '/' && dir[dl - 1] != '\\';
    char *p = malloc(dl + (size_t)need_sep + nl + 1);
    if (!p) {
        return NULL;
    }
    memcpy(p, dir, dl);
    if (need_sep) {
#ifdef _WIN32
        p[dl] = '\\';
#else
        p[dl] = '/';
#endif
        memcpy(p + dl + 1, name, nl + 1);
    } else {
        memcpy(p + dl, name, nl + 1);
    }
    return p;
}

#ifdef _WIN32
static wchar_t *utf8_to_wide(const char *s)
{
    int n = MultiByteToWideChar(CP_UTF8, 0, s, -1, NULL, 0);
    wchar_t *w;
    if (n <= 0) {
        return NULL;
    }
    w = malloc((size_t)n * sizeof(wchar_t));
    if (!w) {
        return NULL;
    }
    MultiByteToWideChar(CP_UTF8, 0, s, -1, w, n);
    return w;
}

static char *wide_to_utf8(const wchar_t *w)
{
    int n = WideCharToMultiByte(CP_UTF8, 0, w, -1, NULL, 0, NULL, NULL);
    char *s;
    if (n <= 0) {
        return NULL;
    }
    s = malloc((size_t)n);
    if (!s) {
        return NULL;
    }
    WideCharToMultiByte(CP_UTF8, 0, w, -1, s, n, NULL, NULL);
    return s;
}
#endif

static int node_add_child(xdvdfs_node *dir, xdvdfs_node *child)
{
    if (dir->nchildren == dir->children_cap) {
        int cap = dir->children_cap ? dir->children_cap * 2 : 8;
        xdvdfs_node **n = realloc(dir->children, (size_t)cap * sizeof(*n));
        if (!n) {
            return XDVDFS_ERR_NOMEM;
        }
        dir->children = n;
        dir->children_cap = cap;
    }
    dir->children[dir->nchildren++] = child;
    return XDVDFS_OK;
}

static int child_name_exists(const xdvdfs_node *dir, const char *name)
{
    int i;
    for (i = 0; i < dir->nchildren; i++) {
        if (xdvdfs_casecmp(dir->children[i]->name, name) == 0) {
            return 1;
        }
    }
    return 0;
}

static xdvdfs_node *node_new(const char *name, const char *host_path, int is_dir,
                             uint32_t file_size)
{
    xdvdfs_node *n = calloc(1, sizeof(*n));
    if (!n) {
        return NULL;
    }
    n->name = xdvdfs_strdup(name);
    n->host_path = xdvdfs_strdup(host_path);
    n->is_dir = is_dir;
    n->file_size = file_size;
    n->fd = -1;
    if (!n->name || !n->host_path) {
        free(n->name);
        free(n->host_path);
        free(n);
        return NULL;
    }
    return n;
}

static void node_free(xdvdfs_node *n, int free_self)
{
    int i;
    if (!n) {
        return;
    }
    for (i = 0; i < n->nchildren; i++) {
        node_free(n->children[i], 1);
    }
    free(n->children);
    free(n->name);
    free(n->host_path);
    free(n->dir_table);
    free(n->xbe_patches);
    if (n->fd >= 0) {
#ifdef _WIN32
        _close(n->fd);
#else
        close(n->fd);
#endif
    }
    if (free_self) {
        free(n);
    }
}

#ifdef _WIN32
static int scan_dir(xdvdfs_node *dir, int depth)
{
    wchar_t *wpath, *wpat;
    size_t plen;
    WIN32_FIND_DATAW fd;
    HANDLE h;
    int ret = XDVDFS_OK;

    if (depth > XDVDFS_MAX_DEPTH) {
        return XDVDFS_OK;
    }

    wpath = utf8_to_wide(dir->host_path);
    if (!wpath) {
        return XDVDFS_ERR_NOMEM;
    }
    plen = wcslen(wpath);
    wpat = malloc((plen + 3) * sizeof(wchar_t));
    if (!wpat) {
        free(wpath);
        return XDVDFS_ERR_NOMEM;
    }
    memcpy(wpat, wpath, plen * sizeof(wchar_t));
    if (plen > 0 && wpat[plen - 1] != L'\\' && wpat[plen - 1] != L'/') {
        wpat[plen++] = L'\\';
    }
    wpat[plen++] = L'*';
    wpat[plen] = 0;
    free(wpath);

    h = FindFirstFileW(wpat, &fd);
    free(wpat);
    if (h == INVALID_HANDLE_VALUE) {
        return XDVDFS_ERR_IO;
    }

    do {
        char *name, *full;
        xdvdfs_node *child;
        int is_dir;
        uint64_t sz;

        if (fd.cFileName[0] == L'.' &&
            (fd.cFileName[1] == 0 ||
             (fd.cFileName[1] == L'.' && fd.cFileName[2] == 0))) {
            continue;
        }
        if (fd.dwFileAttributes & FILE_ATTRIBUTE_REPARSE_POINT) {
            continue;
        }

        name = wide_to_utf8(fd.cFileName);
        if (!name) {
            ret = XDVDFS_ERR_NOMEM;
            break;
        }
        if (strlen(name) > XDVDFS_MAX_NAME || child_name_exists(dir, name)) {
            free(name);
            continue;
        }

        is_dir = (fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0;
        sz = ((uint64_t)fd.nFileSizeHigh << 32) | fd.nFileSizeLow;
        if (!is_dir && sz > UINT32_MAX) {
            free(name);
            continue;
        }

        full = xdvdfs_join(dir->host_path, name);
        if (!full) {
            free(name);
            ret = XDVDFS_ERR_NOMEM;
            break;
        }

        child = node_new(name, full, is_dir, is_dir ? 0 : (uint32_t)sz);
        free(name);
        free(full);
        if (!child) {
            ret = XDVDFS_ERR_NOMEM;
            break;
        }
        ret = node_add_child(dir, child);
        if (ret) {
            node_free(child, 1);
            break;
        }
        if (is_dir) {
            ret = scan_dir(child, depth + 1);
            if (ret) {
                break;
            }
        }
    } while (FindNextFileW(h, &fd));

    FindClose(h);
    return ret;
}
#else
static int scan_dir(xdvdfs_node *dir, int depth)
{
    DIR *d;
    struct dirent *ent;

    if (depth > XDVDFS_MAX_DEPTH) {
        return XDVDFS_OK;
    }

    d = opendir(dir->host_path);
    if (!d) {
        return XDVDFS_ERR_IO;
    }

    while ((ent = readdir(d)) != NULL) {
        struct stat st;
        char *full;
        xdvdfs_node *child;
        int is_dir;

        if (strcmp(ent->d_name, ".") == 0 || strcmp(ent->d_name, "..") == 0) {
            continue;
        }
        if (strlen(ent->d_name) > XDVDFS_MAX_NAME ||
            child_name_exists(dir, ent->d_name)) {
            continue;
        }

        full = xdvdfs_join(dir->host_path, ent->d_name);
        if (!full) {
            closedir(d);
            return XDVDFS_ERR_NOMEM;
        }
        if (stat(full, &st) != 0) {
            free(full);
            continue;
        }
        if (S_ISDIR(st.st_mode)) {
            is_dir = 1;
        } else if (S_ISREG(st.st_mode)) {
            is_dir = 0;
            if ((uint64_t)st.st_size > UINT32_MAX) {
                free(full);
                continue;
            }
        } else {
            free(full);
            continue;
        }

        child = node_new(ent->d_name, full, is_dir,
                         is_dir ? 0 : (uint32_t)st.st_size);
        free(full);
        if (!child) {
            closedir(d);
            return XDVDFS_ERR_NOMEM;
        }
        if (node_add_child(dir, child)) {
            node_free(child, 1);
            closedir(d);
            return XDVDFS_ERR_NOMEM;
        }
        if (is_dir) {
            int r = scan_dir(child, depth + 1);
            if (r) {
                closedir(d);
                return r;
            }
        }
    }

    closedir(d);
    return XDVDFS_OK;
}
#endif

static int cmp_node_ptr(const void *a, const void *b)
{
    const xdvdfs_node *na = *(const xdvdfs_node *const *)a;
    const xdvdfs_node *nb = *(const xdvdfs_node *const *)b;
    return xdvdfs_casecmp(na->name, nb->name);
}

static xdvdfs_node *build_bst(xdvdfs_node **arr, int lo, int hi)
{
    int mid;
    xdvdfs_node *n;
    if (lo > hi) {
        return NULL;
    }
    mid = lo + (hi - lo) / 2;
    n = arr[mid];
    n->left = build_bst(arr, lo, mid - 1);
    n->right = build_bst(arr, mid + 1, hi);
    return n;
}

static void build_trees(xdvdfs_node *dir)
{
    int i;
    if (!dir->is_dir) {
        return;
    }
    for (i = 0; i < dir->nchildren; i++) {
        if (dir->children[i]->is_dir) {
            build_trees(dir->children[i]);
        }
    }
    if (dir->nchildren > 1) {
        qsort(dir->children, (size_t)dir->nchildren, sizeof(*dir->children),
              cmp_node_ptr);
    }
    dir->child_tree = dir->nchildren ?
        build_bst(dir->children, 0, dir->nchildren - 1) : NULL;
}

static void layout_entries(xdvdfs_node *n, uint32_t *size)
{
    uint32_t length;
    if (!n) {
        return;
    }
    length = entry_length(n->name);
    if (n_sectors(*size + length) > n_sectors(*size)) {
        *size += (XDVDFS_SECTOR_SIZE - (*size % XDVDFS_SECTOR_SIZE)) %
                 XDVDFS_SECTOR_SIZE;
    }
    n->entry_offset = *size;
    *size += length;
    layout_entries(n->left, size);
    layout_entries(n->right, size);
}

static void compute_dir_sizes(xdvdfs_node *dir)
{
    int i;
    if (!dir->is_dir) {
        return;
    }
    for (i = 0; i < dir->nchildren; i++) {
        if (dir->children[i]->is_dir) {
            compute_dir_sizes(dir->children[i]);
        }
    }
    if (dir->nchildren == 0) {
        dir->file_size = XDVDFS_SECTOR_SIZE;
        return;
    }
    dir->file_size = 0;
    layout_entries(dir->child_tree, &dir->file_size);
}

static void assign_files(xdvdfs_node *n, uint32_t *sec)
{
    if (!n) {
        return;
    }
    if (!n->is_dir) {
        n->start_sector = *sec;
        *sec += n_sectors(n->file_size);
    }
    assign_files(n->left, sec);
    assign_files(n->right, sec);
}

static void assign_this_dir(xdvdfs_node *dir, uint32_t *sec);

static void assign_subdirs(xdvdfs_node *n, uint32_t *sec)
{
    if (!n) {
        return;
    }
    if (n->is_dir) {
        assign_this_dir(n, sec);
    }
    assign_subdirs(n->left, sec);
    assign_subdirs(n->right, sec);
}

static void assign_this_dir(xdvdfs_node *dir, uint32_t *sec)
{
    dir->start_sector = *sec;
    *sec += n_sectors(dir->file_size);
    assign_files(dir->child_tree, sec);
    assign_subdirs(dir->child_tree, sec);
}

static void write_one_entry(uint8_t *table, const xdvdfs_node *n)
{
    uint8_t *p = table + n->entry_offset;
    uint16_t l = n->left ? (uint16_t)(n->left->entry_offset / 4) : 0;
    uint16_t r = n->right ? (uint16_t)(n->right->entry_offset / 4) : 0;
    uint32_t sz = n->file_size;
    size_t len = strlen(n->name);
    uint8_t attr;

    if (n->is_dir) {
        sz = n_sectors(n->file_size) * XDVDFS_SECTOR_SIZE;
        if (sz == 0) {
            sz = XDVDFS_SECTOR_SIZE;
        }
        attr = XDVDFS_ATTR_DIR;
    } else {
        attr = XDVDFS_ATTR_ARC;
    }

    wr16(p + 0, l);
    wr16(p + 2, r);
    wr32(p + 4, n->start_sector);
    wr32(p + 8, sz);
    p[12] = attr;
    p[13] = (uint8_t)len;
    memcpy(p + 14, n->name, len);
}

static void write_entries(uint8_t *table, xdvdfs_node *n)
{
    if (!n) {
        return;
    }
    write_one_entry(table, n);
    write_entries(table, n->left);
    write_entries(table, n->right);
}

static int gen_tables(xdvdfs_node *dir)
{
    int i;
    uint32_t bytes = n_sectors(dir->file_size) * XDVDFS_SECTOR_SIZE;
    if (bytes == 0) {
        bytes = XDVDFS_SECTOR_SIZE;
    }
    dir->dir_table_bytes = bytes;
    dir->dir_table = malloc(bytes);
    if (!dir->dir_table) {
        return XDVDFS_ERR_NOMEM;
    }
    memset(dir->dir_table, XDVDFS_PAD, bytes);
    if (dir->child_tree) {
        write_entries(dir->dir_table, dir->child_tree);
    }
    for (i = 0; i < dir->nchildren; i++) {
        if (dir->children[i]->is_dir) {
            int r = gen_tables(dir->children[i]);
            if (r) {
                return r;
            }
        }
    }
    return XDVDFS_OK;
}

static int extents_add(xdvdfs_dir *d, uint32_t start, uint32_t num, int kind,
                       xdvdfs_node *node)
{
    xdvdfs_extent *e;
    if (num == 0) {
        return XDVDFS_OK;
    }
    e = realloc(d->extents, (size_t)(d->nextents + 1) * sizeof(*e));
    if (!e) {
        return XDVDFS_ERR_NOMEM;
    }
    d->extents = e;
    e = &d->extents[d->nextents++];
    e->start_sector = start;
    e->num_sectors = num;
    e->kind = kind;
    e->node = node;
    return XDVDFS_OK;
}

static int collect_extents(xdvdfs_dir *d, xdvdfs_node *dir)
{
    int i;
    int r = extents_add(d, dir->start_sector,
                        n_sectors(dir->dir_table_bytes), XDVDFS_EXT_DIR, dir);
    if (r) {
        return r;
    }
    for (i = 0; i < dir->nchildren; i++) {
        xdvdfs_node *c = dir->children[i];
        if (c->is_dir) {
            r = collect_extents(d, c);
            if (r) {
                return r;
            }
        } else {
            r = extents_add(d, c->start_sector, n_sectors(c->file_size),
                            XDVDFS_EXT_FILE, c);
            if (r) {
                return r;
            }
        }
    }
    return XDVDFS_OK;
}

static int cmp_extent(const void *a, const void *b)
{
    const xdvdfs_extent *ea = a;
    const xdvdfs_extent *eb = b;
    if (ea->start_sector < eb->start_sector) {
        return -1;
    }
    if (ea->start_sector > eb->start_sector) {
        return 1;
    }
    return 0;
}

static const xdvdfs_extent *find_extent(const xdvdfs_dir *d, uint32_t lba)
{
    int lo = 0, hi = d->nextents - 1;
    while (lo <= hi) {
        int mid = lo + (hi - lo) / 2;
        const xdvdfs_extent *e = &d->extents[mid];
        if (lba < e->start_sector) {
            hi = mid - 1;
        } else if (lba >= e->start_sector + e->num_sectors) {
            lo = mid + 1;
        } else {
            return e;
        }
    }
    return NULL;
}

static int open_node_file(xdvdfs_node *n)
{
    if (n->fd >= 0) {
        return XDVDFS_OK;
    }
#ifdef _WIN32
    {
        wchar_t *w = utf8_to_wide(n->host_path);
        if (!w) {
            return XDVDFS_ERR_NOMEM;
        }
        n->fd = _wopen(w, _O_RDONLY | _O_BINARY);
        free(w);
    }
#else
    n->fd = open(n->host_path, O_RDONLY);
#endif
    return n->fd < 0 ? XDVDFS_ERR_IO : XDVDFS_OK;
}

static int host_pread(xdvdfs_node *n, uint64_t off, void *buf, size_t len)
{
    size_t got = 0;
    int r = open_node_file(n);
    if (r) {
        return r;
    }
    while (got < len) {
#ifdef _WIN32
        __int64 s = _lseeki64(n->fd, (__int64)(off + got), SEEK_SET);
        int nread;
        if (s < 0) {
            return XDVDFS_ERR_IO;
        }
        nread = _read(n->fd, (uint8_t *)buf + got, (unsigned)(len - got));
#else
        ssize_t nread;
        if (lseek(n->fd, (off_t)(off + got), SEEK_SET) == (off_t)-1) {
            return XDVDFS_ERR_IO;
        }
        nread = read(n->fd, (uint8_t *)buf + got, len - got);
#endif
        if (nread < 0) {
            return XDVDFS_ERR_IO;
        }
        if (nread == 0) {
            memset((uint8_t *)buf + got, 0, len - got);
            return XDVDFS_OK;
        }
        got += (size_t)nread;
    }
    return XDVDFS_OK;
}

static int scan_xbe_patches(xdvdfs_node *n)
{
    uint8_t *buf;
    uint32_t i;
    int cap = 0;

    if (!name_is_xbe(n->name) || n->file_size < XBE_MEDIA_PAT_LEN) {
        return XDVDFS_OK;
    }

    buf = malloc(n->file_size);
    if (!buf) {
        return XDVDFS_OK; /* skip patching rather than fail the mount */
    }
    if (host_pread(n, 0, buf, n->file_size) != XDVDFS_OK) {
        free(buf);
        return XDVDFS_OK;
    }
    for (i = 0; i + XBE_MEDIA_PAT_LEN <= n->file_size; i++) {
        if (memcmp(buf + i, xbe_media_pat, XBE_MEDIA_PAT_LEN) == 0) {
            uint32_t *np;
            if (n->n_xbe_patches == cap) {
                cap = cap ? cap * 2 : 4;
                np = realloc(n->xbe_patches, (size_t)cap * sizeof(*np));
                if (!np) {
                    break;
                }
                n->xbe_patches = np;
            }
            n->xbe_patches[n->n_xbe_patches++] = i + XBE_MEDIA_PATCH_POS;
            i += XBE_MEDIA_PAT_LEN - 1;
        }
    }
    free(buf);
    return XDVDFS_OK;
}

static int scan_xbe_all(xdvdfs_node *dir)
{
    int i;
    for (i = 0; i < dir->nchildren; i++) {
        xdvdfs_node *c = dir->children[i];
        if (c->is_dir) {
            int r = scan_xbe_all(c);
            if (r) {
                return r;
            }
        } else {
            int r = scan_xbe_patches(c);
            if (r) {
                return r;
            }
        }
    }
    return XDVDFS_OK;
}

static void apply_xbe_patches(const xdvdfs_node *n, uint64_t off, uint8_t *buf,
                              size_t len)
{
    int i;
    for (i = 0; i < n->n_xbe_patches; i++) {
        uint32_t p = n->xbe_patches[i];
        if (p >= off && p < off + len) {
            buf[p - off] = XBE_MEDIA_PATCH_BYTE;
        }
    }
}

static uint64_t filetime_now(void)
{
    time_t now = time(NULL);
    if (now == (time_t)-1) {
        now = 0;
    }
    /* 100-ns intervals from 1601-01-01 to Unix epoch */
    return ((uint64_t)now + 11644473600ULL) * 10000000ULL;
}

static int path_is_dir(const char *path)
{
#ifdef _WIN32
    wchar_t *w = utf8_to_wide(path);
    DWORD attr;
    if (!w) {
        return 0;
    }
    attr = GetFileAttributesW(w);
    free(w);
    return attr != INVALID_FILE_ATTRIBUTES &&
           (attr & FILE_ATTRIBUTE_DIRECTORY);
#else
    struct stat st;
    return stat(path, &st) == 0 && S_ISDIR(st.st_mode);
#endif
}

const char *xdvdfs_strerror(int err)
{
    switch (err) {
    case XDVDFS_OK:
        return "success";
    case XDVDFS_ERR_ARG:
        return "invalid argument";
    case XDVDFS_ERR_IO:
        return "I/O error";
    case XDVDFS_ERR_NOTDIR:
        return "path is not a directory";
    case XDVDFS_ERR_RANGE:
        return "request outside the image";
    case XDVDFS_ERR_NOMEM:
        return "allocation failed";
    case XDVDFS_ERR_NOTFOUND:
        return "path not found";
    default:
        return "unknown error";
    }
}

int xdvdfs_dir_open(xdvdfs_dir **out, const char *path)
{
    xdvdfs_dir *d;
    uint32_t sec;
    uint32_t last;
    int i, r;

    if (!out || !path || !path[0]) {
        return XDVDFS_ERR_ARG;
    }
    *out = NULL;

    if (!path_is_dir(path)) {
#ifdef _WIN32
        wchar_t *w = utf8_to_wide(path);
        DWORD attr = w ? GetFileAttributesW(w) : INVALID_FILE_ATTRIBUTES;
        free(w);
        return attr == INVALID_FILE_ATTRIBUTES ? XDVDFS_ERR_NOTFOUND
                                               : XDVDFS_ERR_NOTDIR;
#else
        struct stat st;
        if (stat(path, &st) != 0) {
            return XDVDFS_ERR_NOTFOUND;
        }
        return XDVDFS_ERR_NOTDIR;
#endif
    }

    d = calloc(1, sizeof(*d));
    if (!d) {
        return XDVDFS_ERR_NOMEM;
    }
    d->root.is_dir = 1;
    d->root.fd = -1;
    d->root.name = xdvdfs_strdup("");
    d->root.host_path = xdvdfs_strdup(path);
    if (!d->root.name || !d->root.host_path) {
        xdvdfs_dir_free(d);
        return XDVDFS_ERR_NOMEM;
    }

    r = scan_dir(&d->root, 0);
    if (r) {
        xdvdfs_dir_free(d);
        return r;
    }

    build_trees(&d->root);
    compute_dir_sizes(&d->root);
    sec = XDVDFS_ROOT_LBA;
    assign_this_dir(&d->root, &sec);

    r = gen_tables(&d->root);
    if (r) {
        xdvdfs_dir_free(d);
        return r;
    }

    memset(d->volume, 0, sizeof(d->volume));
    memcpy(d->volume, XDVDFS_MAGIC, XDVDFS_MAGIC_LEN);
    wr32(d->volume + 0x14, d->root.start_sector);
    wr32(d->volume + 0x18, d->root.file_size);
    wr64(d->volume + 0x1C, filetime_now());
    memcpy(d->volume + 0x7EC, XDVDFS_MAGIC, XDVDFS_MAGIC_LEN);

    r = extents_add(d, XDVDFS_VOLUME_LBA, 1, XDVDFS_EXT_VOLUME, NULL);
    if (!r) {
        r = collect_extents(d, &d->root);
    }
    if (r) {
        xdvdfs_dir_free(d);
        return r;
    }
    if (d->nextents > 1) {
        qsort(d->extents, (size_t)d->nextents, sizeof(*d->extents), cmp_extent);
    }

    last = XDVDFS_VOLUME_LBA + 1;
    for (i = 0; i < d->nextents; i++) {
        uint32_t end = d->extents[i].start_sector + d->extents[i].num_sectors;
        if (end > last) {
            last = end;
        }
    }
    d->total_size = (uint64_t)last * XDVDFS_SECTOR_SIZE;
    if (d->total_size % XDVDFS_MODULUS) {
        d->total_size += XDVDFS_MODULUS - (d->total_size % XDVDFS_MODULUS);
    }

    scan_xbe_all(&d->root);

    *out = d;
    return XDVDFS_OK;
}

void xdvdfs_dir_free(xdvdfs_dir *d)
{
    if (!d) {
        return;
    }
    node_free(&d->root, 0);
    free(d->extents);
    free(d);
}

uint64_t xdvdfs_dir_sector_count(const xdvdfs_dir *d)
{
    return d ? d->total_size / XDVDFS_SECTOR_SIZE : 0;
}

uint64_t xdvdfs_dir_size(const xdvdfs_dir *d)
{
    return d ? d->total_size : 0;
}

int xdvdfs_dir_read_sector(xdvdfs_dir *d, uint64_t lba,
                           uint8_t out[XDVDFS_SECTOR_SIZE])
{
    const xdvdfs_extent *e;
    uint32_t rel;
    int r;

    if (!d || !out) {
        return XDVDFS_ERR_ARG;
    }
    if (lba >= xdvdfs_dir_sector_count(d)) {
        return XDVDFS_ERR_RANGE;
    }

    memset(out, 0, XDVDFS_SECTOR_SIZE);
    e = find_extent(d, (uint32_t)lba);
    if (!e) {
        return XDVDFS_OK;
    }

    rel = (uint32_t)lba - e->start_sector;
    switch (e->kind) {
    case XDVDFS_EXT_VOLUME:
        memcpy(out, d->volume, XDVDFS_SECTOR_SIZE);
        return XDVDFS_OK;
    case XDVDFS_EXT_DIR: {
        uint32_t off = rel * XDVDFS_SECTOR_SIZE;
        uint32_t n = XDVDFS_SECTOR_SIZE;
        if (off >= e->node->dir_table_bytes) {
            memset(out, XDVDFS_PAD, XDVDFS_SECTOR_SIZE);
            return XDVDFS_OK;
        }
        if (off + n > e->node->dir_table_bytes) {
            n = e->node->dir_table_bytes - off;
            memset(out + n, XDVDFS_PAD, XDVDFS_SECTOR_SIZE - n);
        }
        memcpy(out, e->node->dir_table + off, n);
        return XDVDFS_OK;
    }
    case XDVDFS_EXT_FILE: {
        uint64_t off = (uint64_t)rel * XDVDFS_SECTOR_SIZE;
        uint32_t n = XDVDFS_SECTOR_SIZE;
        if (off >= e->node->file_size) {
            return XDVDFS_OK;
        }
        if (off + n > e->node->file_size) {
            n = (uint32_t)(e->node->file_size - off);
        }
        r = host_pread(e->node, off, out, n);
        if (r) {
            return r;
        }
        apply_xbe_patches(e->node, off, out, n);
        return XDVDFS_OK;
    }
    default:
        return XDVDFS_OK;
    }
}

int xdvdfs_dir_read(xdvdfs_dir *d, uint64_t offset, void *buf, size_t len)
{
    uint8_t sector[XDVDFS_SECTOR_SIZE];
    uint8_t *out = buf;
    uint64_t end;

    if (!d || (!buf && len)) {
        return XDVDFS_ERR_ARG;
    }
    if (len == 0) {
        return XDVDFS_OK;
    }
    end = offset + len;
    if (end < offset || end > d->total_size) {
        return XDVDFS_ERR_RANGE;
    }

    while (len) {
        uint64_t lba = offset / XDVDFS_SECTOR_SIZE;
        uint32_t sec_off = (uint32_t)(offset % XDVDFS_SECTOR_SIZE);
        uint32_t chunk = XDVDFS_SECTOR_SIZE - sec_off;
        int r;
        if (chunk > len) {
            chunk = (uint32_t)len;
        }
        r = xdvdfs_dir_read_sector(d, lba, sector);
        if (r) {
            return r;
        }
        memcpy(out, sector + sec_off, chunk);
        out += chunk;
        offset += chunk;
        len -= chunk;
    }
    return XDVDFS_OK;
}
