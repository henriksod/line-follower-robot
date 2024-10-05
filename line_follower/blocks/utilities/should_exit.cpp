// Copyright (c) 2024 Henrik Söderlund

#include "line_follower/blocks/utilities/should_exit.h"

sig_atomic_t volatile should_exit{0};
