/*
 *  Copyright (C) 2025 TierIV Inc.
 *
 *  License: Apache License
 *
 *  Page cache management utilities for large sequential file I/O
 */

#ifndef BAG_CONVERTER_MEMORY_MANAGEMENT_HPP
#define BAG_CONVERTER_MEMORY_MANAGEMENT_HPP

#include <string>

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

}  // namespace memory_management

#endif  // BAG_CONVERTER_MEMORY_MANAGEMENT_HPP
