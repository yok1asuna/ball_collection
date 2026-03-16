#!/usr/bin/env python3

import math
import matplotlib.pyplot as plt

def solve_tsp_greedy(points):
    """
    贪心TSP算法实现
    points: [(x, y), ...] 坐标列表
    """
    if not points:
        return []

    path = [points[0]]  # 从第一个点开始
    visited = [False] * len(points)
    visited[0] = True

    while len(path) < len(points):
        last_point = path[-1]
        min_dist = float('inf')
        next_idx = -1

        # 找到最近的未访问点
        for i, point in enumerate(points):
            if not visited[i]:
                dist = math.hypot(point[0] - last_point[0], point[1] - last_point[1])
                if dist < min_dist:
                    min_dist = dist
                    next_idx = i

        if next_idx != -1:
            path.append(points[next_idx])
            visited[next_idx] = True

    return path

# 测试数据：5个网球位置
test_points = [
    (3.0, 2.0),   # 网球1
    (3.2, 1.8),   # 网球2
    (2.8, 2.2),   # 网球3
    (-4.0, -2.0), # 网球4
    (-7.0, 3.0)   # 网球5
]

print("原始网球位置:")
for i, p in enumerate(test_points):
    print(f"网球{i+1}: ({p[0]:.1f}, {p[1]:.1f})")

path = solve_tsp_greedy(test_points)

print("\n贪心TSP路径:")
total_distance = 0
for i, p in enumerate(path):
    if i > 0:
        dist = math.hypot(p[0] - path[i-1][0], p[1] - path[i-1][1])
        total_distance += dist
        print(f"  → 网球{test_points.index(p)+1}: ({p[0]:.1f}, {p[1]:.1f}) 距离: {dist:.2f}")
    else:
        print(f"起点 网球{test_points.index(p)+1}: ({p[0]:.1f}, {p[1]:.1f})")

print(f"\n总距离: {total_distance:.2f}")

# 可视化
plt.figure(figsize=(10, 8))
plt.scatter(*zip(*test_points), c='red', s=100, label='网球位置')

# 绘制路径
for i in range(len(path)-1):
    plt.plot([path[i][0], path[i+1][0]], [path[i][1], path[i+1][1]], 'b-', linewidth=2)

# 标记起点和终点
plt.scatter(path[0][0], path[0][1], c='green', s=150, marker='*', label='起点')
plt.scatter(path[-1][0], path[-1][1], c='blue', s=150, marker='s', label='终点')

plt.title('网球收集路径规划 (贪心TSP)')
plt.xlabel('X 坐标')
plt.ylabel('Y 坐标')
plt.grid(True)
plt.legend()
plt.axis('equal')
plt.show()