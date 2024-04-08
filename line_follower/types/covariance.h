// Copyright (c) 2023 Henrik Söderlund

#ifndef LINE_FOLLOWER_TYPES_COVARIANCE_H_
#define LINE_FOLLOWER_TYPES_COVARIANCE_H_

namespace line_follower {
/// A 2-dimensional covariance matrix
struct Covariance2D final {
    double m00;
    double m01;
    double m10;
    double m11;
};
}  // namespace line_follower

#endif  // LINE_FOLLOWER_TYPES_COVARIANCE_H_
