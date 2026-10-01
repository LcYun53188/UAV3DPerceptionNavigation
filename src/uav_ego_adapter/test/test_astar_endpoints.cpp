// SPDX-License-Identifier: GPL-3.0-only
#include <gtest/gtest.h>
#include <path_searching/dyn_a_star.h>

TEST(AStarEndpoints, ConnectsAboveFloorInsteadOfPushingIntoUnknown) {
  auto map = std::make_shared<GridMap>();
  map->collision = [](const Eigen::Vector3d& p) { return p.z() < .44; };
  map->segment_collision = [map](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    return map->collision(a) || map->collision(b);
  };
  AStar search;
  search.initGridMap(map, Eigen::Vector3i(30, 30, 20));
  const Eigen::Vector3d a(0, 0, .45), b(1, 0, .75);
  for (bool reverse : {false, true}) {
    const auto start = reverse ? b : a, goal = reverse ? a : b;
    ASSERT_TRUE(search.AstarSearch(.2, start, goal));
    const auto path = search.getPath();
    ASSERT_FALSE(path.empty());
    EXPECT_FALSE(map->segmentCollision(start, path.front()));
    EXPECT_FALSE(map->segmentCollision(path.back(), goal));
    for (size_t i = 1; i < path.size(); ++i)
      EXPECT_FALSE(map->segmentCollision(path[i-1], path[i]));
  }
  map->segment_collision = nullptr;
}

TEST(AStarEndpoints, DoesNotSnapAcrossBlockedConnector) {
  auto map = std::make_shared<GridMap>();
  map->collision = [](const Eigen::Vector3d&) { return false; };
  map->segment_collision = [](const Eigen::Vector3d&, const Eigen::Vector3d&) { return true; };
  AStar search;
  search.initGridMap(map, Eigen::Vector3i(20, 20, 20));
  EXPECT_FALSE(search.AstarSearch(.2, Eigen::Vector3d(0, 0, .45), Eigen::Vector3d(1, 0, .75)));
}

TEST(AStarEndpoints, MapResolutionRecoversCorridorMissedByCoarseLattice) {
  auto map = std::make_shared<GridMap>();
  map->collision = [](const Eigen::Vector3d& p) {
    if (std::abs(p.z()) > .03 || p.x() < -.1 || p.x() > 1.1) return true;
    if (p.x() < .2 || p.x() > .8) return std::abs(p.y()) > .15;
    return std::abs(p.y()-.1) > .025;
  };
  map->segment_collision = [ptr=map.get()](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    const int n = std::max(1, int(std::ceil((b-a).norm()/.005)));
    for (int i = 0; i <= n; ++i)
      if (ptr->collision(a+(b-a)*(double(i)/n))) return true;
    return false;
  };
  AStar search;
  search.initGridMap(map, Eigen::Vector3i(30, 30, 20));
  const Eigen::Vector3d start(0, 0, 0), goal(1, 0, 0);
  EXPECT_FALSE(search.AstarSearch(.2, start, goal));
  ASSERT_TRUE(search.AstarSearch(.1, start, goal));
  const auto path = search.getPath();
  for (size_t i = 1; i < path.size(); ++i)
    EXPECT_FALSE(map->segmentCollision(path[i-1], path[i]));
}
