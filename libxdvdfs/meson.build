# In-tree virtual XISO library. No QEMU headers; block/xdvdfs.c is the adapter.
libxdvdfs_inc = include_directories('include')
libxdvdfs_lib = static_library('xdvdfs',
  files('src/xdvdfs.c'),
  include_directories: libxdvdfs_inc,
  build_by_default: false)
libxdvdfs_dep = declare_dependency(
  link_with: libxdvdfs_lib,
  include_directories: libxdvdfs_inc)
