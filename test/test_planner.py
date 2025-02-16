import unittest

from services.path_planner import plan_path

class TestPathPlanning(unittest.TestCase):
    def setUp(self):
        self.obstacles = [
            # 格式：((x, y, z), (w, h, d))
            ((5, 5, 0), (2, 2, 10)),  # 地面到10米高的障碍物
            ((8, 8, 5), (1, 1, 1))
        ]
    
    def test_free_space(self):
        """无障碍物直线路径"""
        path = plan_path(
            current_position=(0, 0, 0),
            target_position=(3, 3, 3),
            obstacles=[],
            grid_resolution=1
        )
        # 期望路径应接近直线
        self.assertGreaterEqual(len(path), 4)
        self.assertEqual(path[0], (0, 0, 0))
        self.assertEqual(path[-1], (3, 3, 3))

    def test_obstacle_avoidance(self):
        """需要绕开障碍物"""
        path = plan_path(
            current_position=(0, 0, 0),
            target_position=(10, 10, 0),
            obstacles=self.obstacles,
            grid_resolution=1
        )
        self.assertTrue(len(path) > 0)
        
        # 验证路径不经过障碍物膨胀区
        forbidden_zones = [
            (5,5,0), (6,6,5)  # 膨胀后的障碍物中心点
        ]
        for point in path:
            self.assertNotIn(point, forbidden_zones)

    def test_unreachable_target(self):
        """目标被障碍物包围"""
        obstacles = [
            ((1,1,1), (3,3,3)),  # 包围目标
        ]
        path = plan_path(
            current_position=(0,0,0),
            target_position=(2,2,2),
            obstacles=obstacles,
            grid_resolution=1
        )
        self.assertEqual(len(path), 0)

    def test_same_start_end(self):
        """起点终点相同"""
        path = plan_path((5,5,5), (5,5,5), [])
        self.assertEqual(path, [(5,5,5)])

if __name__ == "__main__":
    unittest.main()