// Copyright (c) 2024 Henrik Söderlund

#ifndef LINE_FOLLOWER_BLOCKS_UTILITIES_SHOULD_EXIT_H_
#define LINE_FOLLOWER_BLOCKS_UTILITIES_SHOULD_EXIT_H_

#include <bits/types/sig_atomic_t.h>

/// A global atomic flag to indicate the shutdown process
extern sig_atomic_t volatile should_exit;

#endif  // LINE_FOLLOWER_BLOCKS_UTILITIES_SHOULD_EXIT_H_
