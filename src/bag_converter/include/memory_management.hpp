/*
 *  Copyright (C) 2025 TierIV Inc.
 *
 *  License: Apache License
 *
 *  Page cache management utilities for large sequential file I/O
 */

#ifndef BAG_CONVERTER__MEMORY_MANAGEMENT_HPP
#define BAG_CONVERTER__MEMORY_MANAGEMENT_HPP

#include <string>
#include <vector>

namespace memory_management
{

/**
 * @brief Hint the kernel that a file will be read sequentially.
 *
 * Calls posix_fadvise(POSIX_FADV_SEQUENTIAL) which doubles the read-ahead
 * window and lowers the priority of already-read pages, making them eligible
 * for earlier reclamation under memory pressure.
 *
 * @param path Path to the file to advise
 */
void fadvise_sequential_access(const std::string & path);

/**
 * @brief Drop page cache for a file using posix_fadvise(POSIX_FADV_DONTNEED).
 *
 * Opens the file with a separate file descriptor, advises the kernel that the
 * cached pages are no longer needed, then closes the descriptor. This works
 * regardless of whether another FD (e.g. from rosbag2) is still open on the
 * same file — the advisory applies to the underlying page cache.
 *
 * @param path Path to the file whose cached pages should be evicted
 */
void fadvise_drop_page_cache(const std::string & path);

/**
 * @brief RAII guard that frees page cache for tracked files on destruction.
 *
 * Tracks file paths via track() and calls fadvise_drop_page_cache() for each on
 * destruction. Ensures page cache is freed regardless of how the scope is
 * exited (normal return, early error return, or Ctrl+C graceful shutdown).
 */
class PageCacheGuard
{
public:
  PageCacheGuard() = default;
  ~PageCacheGuard()
  {
    for (const auto & p : paths_) {
      fadvise_drop_page_cache(p);
    }
  }

  PageCacheGuard(const PageCacheGuard &) = delete;
  PageCacheGuard & operator=(const PageCacheGuard &) = delete;
  PageCacheGuard(PageCacheGuard &&) = delete;
  PageCacheGuard & operator=(PageCacheGuard &&) = delete;

  void track(const std::string & path) { paths_.push_back(path); }

private:
  std::vector<std::string> paths_;
};

}  // namespace memory_management

#endif  // BAG_CONVERTER__MEMORY_MANAGEMENT_HPP
