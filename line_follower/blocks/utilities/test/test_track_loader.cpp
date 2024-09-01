// Copyright (c) 2023 Henrik Söderlund

#include <gtest/gtest.h>

#include "line_follower/blocks/utilities/track_loader.h"
#include "line_follower/external/types/track_lines.h"
#include "tools/cpp/runfiles/runfiles.h"

namespace {

using bazel::tools::cpp::runfiles::Runfiles;

/// TODO: Move to test_utils
// Helper function to get the full path to a test file
std::string GetTestFilePath(const std::string& relative_path) {
    std::string error;
    std::unique_ptr<Runfiles> runfiles(Runfiles::CreateForTest(&error));
    if (!runfiles) {
        throw std::runtime_error("Failed to initialize runfiles: " + error);
    }
    return runfiles->Rlocation("_main/line_follower/" + relative_path);
}

TEST(TrackLoaderTest, LoadScenarioFromJson_ValidJson) {
    // Get the path to the test JSON file
    std::string test_json_path = GetTestFilePath("blocks/utilities/test/data/test_data.json");

    // Load the track segments
    std::vector<line_follower::TrackSegment> track_segments{};
    line_follower::Pose pose{};
    bool success{
        line_follower::track_loader::loadScenarioFromJson(test_json_path, track_segments, pose)};

    ASSERT_TRUE(success);

    // Validate pose
    EXPECT_DOUBLE_EQ(pose.position.x, 0.0);
    EXPECT_DOUBLE_EQ(pose.position.y, 0.0);
    EXPECT_DOUBLE_EQ(pose.position.z, 0.0);
    EXPECT_DOUBLE_EQ(pose.rotation.w, 1.0);
    EXPECT_DOUBLE_EQ(pose.rotation.x, 0.0);
    EXPECT_DOUBLE_EQ(pose.rotation.y, 0.0);
    EXPECT_DOUBLE_EQ(pose.rotation.z, 0.0);

    // Validate the number of segments
    ASSERT_EQ(track_segments.size(), 1);

    // Validate the first track segment's pose
    const auto& segment = track_segments[0];
    EXPECT_DOUBLE_EQ(segment.pose.position.x, 1.0);
    EXPECT_DOUBLE_EQ(segment.pose.position.y, 2.0);
    EXPECT_DOUBLE_EQ(segment.pose.position.z, 3.0);
    EXPECT_DOUBLE_EQ(segment.pose.rotation.w, 1.0);
    EXPECT_DOUBLE_EQ(segment.pose.rotation.x, 0.0);
    EXPECT_DOUBLE_EQ(segment.pose.rotation.y, 0.0);
    EXPECT_DOUBLE_EQ(segment.pose.rotation.z, 0.0);

    // Validate the track lines
    const auto& track_line = segment.track_lines[0];
    EXPECT_DOUBLE_EQ(track_line.line.start.x, 1.0);
    EXPECT_DOUBLE_EQ(track_line.line.start.y, 1.0);
    EXPECT_DOUBLE_EQ(track_line.line.start.z, 1.0);
    EXPECT_DOUBLE_EQ(track_line.line.end.x, 2.0);
    EXPECT_DOUBLE_EQ(track_line.line.end.y, 2.0);
    EXPECT_DOUBLE_EQ(track_line.line.end.z, 2.0);
    EXPECT_DOUBLE_EQ(track_line.width, 0.5);
    EXPECT_DOUBLE_EQ(track_line.whiteness, 0.8);
    EXPECT_TRUE(track_line.visible);

    const auto& track_line_1 = segment.track_lines[1];
    EXPECT_FALSE(track_line_1.visible);
}

TEST(TrackLoaderTest, LoadScenarioFromJson_InvalidJson_NoInitialRobotPose) {
    // Path to the invalid JSON file (missing track_segments)
    std::string test_json_path =
        GetTestFilePath("blocks/utilities/test/data/invalid_no_initial_robot_pose.json");

    // Load the track segments
    std::vector<line_follower::TrackSegment> track_segments{};
    line_follower::Pose pose{};
    bool success{
        line_follower::track_loader::loadScenarioFromJson(test_json_path, track_segments, pose)};

    ASSERT_FALSE(success);
}

TEST(TrackLoaderTest, LoadScenarioFromJson_InvalidJson_NoTrackSegments) {
    // Path to the invalid JSON file (missing track_segments)
    std::string test_json_path =
        GetTestFilePath("blocks/utilities/test/data/invalid_no_track_segments.json");

    // Load the track segments
    std::vector<line_follower::TrackSegment> track_segments{};
    line_follower::Pose pose{};
    bool success{
        line_follower::track_loader::loadScenarioFromJson(test_json_path, track_segments, pose)};

    ASSERT_FALSE(success);
}

TEST(TrackLoaderTest, LoadScenarioFromJson_InvalidJson_NoPose) {
    // Path to the invalid JSON file (missing pose)
    std::string test_json_path = GetTestFilePath("blocks/utilities/test/data/invalid_no_pose.json");

    // Load the track segments
    std::vector<line_follower::TrackSegment> track_segments{};
    line_follower::Pose pose{};
    bool success{
        line_follower::track_loader::loadScenarioFromJson(test_json_path, track_segments, pose)};

    ASSERT_FALSE(success);
}

TEST(TrackLoaderTest, LoadScenarioFromJson_InvalidJson_NoTrackLines) {
    // Path to the invalid JSON file (missing track_lines)
    std::string test_json_path =
        GetTestFilePath("blocks/utilities/test/data/invalid_no_track_lines.json");

    // Load the track segments
    std::vector<line_follower::TrackSegment> track_segments{};
    line_follower::Pose pose{};
    bool success{
        line_follower::track_loader::loadScenarioFromJson(test_json_path, track_segments, pose)};

    ASSERT_FALSE(success);
}

TEST(TrackLoaderTest, LoadScenarioFromJson_ValidJson_MultipleTrackLines) {
    // Path to the valid JSON file with multiple track lines
    std::string test_json_path =
        GetTestFilePath("blocks/utilities/test/data/multiple_track_lines.json");

    // Load the track segments
    std::vector<line_follower::TrackSegment> track_segments{};
    line_follower::Pose pose{};
    bool success{
        line_follower::track_loader::loadScenarioFromJson(test_json_path, track_segments, pose)};

    ASSERT_TRUE(success);

    // Validate pose
    EXPECT_DOUBLE_EQ(pose.position.x, 0.0);
    EXPECT_DOUBLE_EQ(pose.position.y, 0.0);
    EXPECT_DOUBLE_EQ(pose.position.z, 0.0);
    EXPECT_DOUBLE_EQ(pose.rotation.w, 1.0);
    EXPECT_DOUBLE_EQ(pose.rotation.x, 0.0);
    EXPECT_DOUBLE_EQ(pose.rotation.y, 0.0);
    EXPECT_DOUBLE_EQ(pose.rotation.z, 0.0);

    // Validate the number of segments
    ASSERT_EQ(track_segments.size(), 1);

    // Validate the first track segment's pose
    const auto& segment = track_segments[0];
    EXPECT_DOUBLE_EQ(segment.pose.position.x, 1.0);
    EXPECT_DOUBLE_EQ(segment.pose.position.y, 2.0);
    EXPECT_DOUBLE_EQ(segment.pose.position.z, 3.0);

    // Validate the track lines
    const auto& track_line_1 = segment.track_lines[0];
    EXPECT_DOUBLE_EQ(track_line_1.line.start.x, 1.0);
    EXPECT_DOUBLE_EQ(track_line_1.line.end.y, 2.0);
    EXPECT_TRUE(track_line_1.visible);

    const auto& track_line_2 = segment.track_lines[1];
    EXPECT_DOUBLE_EQ(track_line_2.line.start.x, 3.0);
    EXPECT_DOUBLE_EQ(track_line_2.line.end.y, 4.0);
    EXPECT_TRUE(track_line_2.visible);

    const auto& track_line_3 = segment.track_lines[3];
    EXPECT_FALSE(track_line_3.visible);
}

}  // namespace
