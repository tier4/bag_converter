/*
 *  Copyright (C) 2025 TierIV Inc.
 *
 *  License: Apache License
 *
 *  Page cache management utilities for large sequential file I/O
 */

#include "memory_management.hpp"

#ifdef __linux__
#include <fcntl.h>
#include <unistd.h>
#endif

namespace memory_management
{

#ifdef __linux__
static void fadvise(const std::string & path, int advice)
{
  int fd = ::open(path.c_str(), O_RDONLY);
  if (fd < 0) {
    return;
  }
  ::posix_fadvise(fd, 0, 0, advice);
  ::close(fd);
}
#endif

void fadvise_sequential_access(const std::string & path)
{
#ifdef __linux__
  fadvise(path, POSIX_FADV_SEQUENTIAL);
#else
  (void)path;
#endif
}

void fadvise_drop_page_cache(const std::string & path)
{
#ifdef __linux__
  fadvise(path, POSIX_FADV_DONTNEED);
#else
  (void)path;
#endif
}

}  // namespace memory_management
