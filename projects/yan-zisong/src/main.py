import heapq
import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
import math

# 定义网络结构
def add_node(network, node):
    network[node] = []


def add_link(network, node1, node2):
    network[node1].append(node2)


network = {}
add_node(network, "A")
add_node(network, "B")
add_node(network, "C")
add_node(network, "D")
add_node(network, "E")
add_link(network, "A", "B")
add_link(network, "A", "C")
add_link(network, "B", "C")
add_link(network, "B", "D")
add_link(network, "C", "D")
add_link(network, "D", "E")
add_link(network, "B", "E")

# 长度映射 (公里)
length_map = {
    ("A", "B"): 1,
    ("A", "C"): 3,
    ("B", "C"): 7,
    ("B", "D"): 10,
    ("B", "E"): 8,
    ("C", "D"): 6,
    ("D", "E"): 2,
}

# 速度映射 (km/h)
speed_map = {
    ("A", "B"): 40,
    ("A", "C"): 40,
    ("B", "C"): 60,
    ("B", "D"): 50,
    ("B", "E"): 55,
    ("C", "D"): 36,
    ("D", "E"): 63,
}


# 使用BPR函数计算行驶时间 (考虑拥堵情况)
def bpr_function(flow, length, speed, capacity=500):
    free_flow_time = length / speed * 60  # 自由流动时间，单位为分钟
    return free_flow_time * (1 + 0.15 * (flow / capacity) ** 4)


# 生成成本函数映射表
def create_cost_function_map(length_map, speed_map):
    cost_function_map = {}
    for link in length_map:
        length = length_map[link]
        speed = speed_map[link]
        cost_function_map[link] = lambda flow, length=length, speed=speed: bpr_function(
            flow, length, speed
        )
    return cost_function_map


cost_function_map = create_cost_function_map(length_map, speed_map)


# Dijkstra算法
def dijkstra(network, attr_map, origin, destination):
    queue = [(0, origin, [])]
    visited = set()

    while queue:
        (cost, node, path) = heapq.heappop(queue)

        if node in visited:
            continue
        path = path + [node]

        if node == destination:
            return path

        visited.add(node)
        for neighbor in network.get(node, []):
            if neighbor not in visited:
                edge_cost = attr_map.get((node, neighbor), float("inf"))
                heapq.heappush(queue, (cost + edge_cost, neighbor, path))

    return []


# 获取网络中所有的连接 (边)
def network_links(network):
    links = []
    for node, neighbors in network.items():
        for neighbor in neighbors:
            links.append((node, neighbor))
    return links


# 更新成本 (行驶时间) 映射，考虑流量
def update_cost(cost_function_map, flow_map):
    map = {}
    for key in cost_function_map.keys():
        cost_function = cost_function_map[key]
        flow = flow_map.get(key, 0)
        cost = cost_function(flow)
        map[key] = cost
    return map


# 模拟流量加载过程，将流量分配到路径上的每条边
def flow_loading(network, path, flow):
    map = {}
    links = set(zip(path[:-1], path[1:]))
    for link in network_links(network):
        if link in links:
            map[link] = flow
        else:
            map[link] = 0
    return map


# 合并两个流量图
def flow_map_combine(map1, map2):
    combined_map = {}
    for key in set(map1.keys()).union(map2.keys()):
        combined_map[key] = map1.get(key, 0) + map2.get(key, 0)
    return combined_map


# 按比例缩放流量图
def flow_map_scale(map1, scale):
    scaled_map = {}
    for key in map1.keys():
        scaled_map[key] = map1[key] * scale
    return scaled_map


# 单个OD对的流量分配
def single_od_assign(network, cost_function_map, od_pair, base_flow_map, epsilon=1e-1):
    origin, destination, flow = od_pair
    cost_map = update_cost(cost_function_map, base_flow_map)
    shortest_path = dijkstra(network, cost_map, origin, destination)
    flow_map = flow_loading(network, shortest_path, flow)

    for _ in range(50000):  # 最大迭代次数50000次
        scale = 1 - 1 / (_ + 2)
        combined_flow_map = flow_map_combine(base_flow_map, flow_map)
        cost_map = update_cost(cost_function_map, combined_flow_map)
        new_shortest_path = dijkstra(network, cost_map, origin, destination)

        new_flow_map = flow_loading(network, new_shortest_path, flow * (1 - scale))
        flow_map = flow_map_combine(flow_map_scale(flow_map, scale), new_flow_map)

        # 计算流量变化的最大值
        max_flow_change = max(
            abs(flow_map.get(link, 0) - combined_flow_map.get(link, 0))
            for link in flow_map
        )
        print(f"第{_ + 1}次迭代，流量变化最大值为: {max_flow_change}")

        # 判断是否小于epsilon
        if max_flow_change < epsilon:
            print(f"迭代已收敛，流量变化最大值为 {max_flow_change}，小于阈值 {epsilon}")
            break

    return flow_map


# 为所有OD对分配流量
# OD对 (起点, 终点, 流量)
od_pairs = [("A", "E", 3000)]
def assign(network, cost_function_map, od_pairs):
    flow_map = {}
    for od_pair in od_pairs:
        new_flow_map = single_od_assign(network, cost_function_map, od_pair, flow_map)
        flow_map = flow_map_combine(flow_map, new_flow_map)

    return flow_map


# 运行流量分配
final_flow_map = assign(network, cost_function_map, od_pairs)

# 输出最终的流量分配结果
print("最终流量分配：")
for link, flow in final_flow_map.items():
    print(f"边 {link}: 流量为{flow} ")



# 图形展示代码
# 使用NetworkX创建一个无向图
G = nx.Graph()

for (u, v), flow in final_flow_map.items():
    if u not in G.nodes():
        G.add_node(u)
    if v not in G.nodes():
        G.add_node(v)

    # 添加边，设置权重
    G.add_edge(u, v, weight=flow)

# 设置节点的位置
pos = {"A": (0, 0), "B": (1, 1), "C": (0, 2), "D": (2, 2), "E": (3, 1)}

# 使用Matplotlib绘制网络图
plt.figure(figsize=(8, 6))  # 设置图形的大小
nx.draw_networkx_nodes(G, pos, node_color="lightblue", node_size=700)
nx.draw_networkx_labels(G, pos, font_size=16, font_weight="bold")


def format_weight(weight):
    return f"{weight:.4f}"


# 边的计算和绘制
edge_labels = {}
for u, v, data in G.edges(data=True):
    x_values = [pos[u][0], pos[v][0]]
    y_values = [pos[u][1], pos[v][1]]

    # 计算边的中点
    mid_x = (x_values[0] + x_values[1]) / 2
    mid_y = (y_values[0] + y_values[1]) / 2

    # 计算偏移量，避免标签与线段重叠
    if x_values[0] != x_values[1]:
        slope = (y_values[1] - y_values[0]) / (x_values[1] - x_values[0])
    else:
        slope = float("inf")  # 边是垂直的，斜率不存在，设为无穷大

    if not np.isinf(slope):  # 边不是水平的
        offset_x = 0.1  # 水平偏移量
        if slope != 0:
            offset_y = offset_x * (-1 / slope)  # 根据斜率计算垂直偏移量
        else:
            offset_y = 0.1
    else:
        offset_y = -0.1
        if (x_values[1] > x_values[0] and y_values[1] > y_values[0]) or (
            x_values[1] < x_values[0] and y_values[1] < y_values[0]
        ):
            offset_x, offset_y = -0.1, -offset_y
        else:  # 边是水平的
            offset_x = 0.1 if x_values[1] > x_values[0] else -0.1  # 根据方向调整水平偏移量
            offset_y = 0  # 垂直偏移量为0

    # 根据边的方向调整偏移方向
    if x_values[0] < x_values[1]:  # 从左到右
        label_x = mid_x + offset_x
    else:  # 从右到左
        label_x = mid_x - offset_x

    if y_values[0] < y_values[1]:  # 从下到上
        label_y = mid_y + offset_y
    else:  # 从上到下
        label_y = mid_y - offset_y

    # 存储标签位置和文本
    edge_labels[(u, v)] = f"Flow: {format_weight(data['weight'])}"
    plt.text(
        label_x,
        label_y,
        edge_labels[(u, v)],
        fontsize=12,
        color="red",
        ha="center",
        va="center",
    )

    # 绘制边
    if data['weight'] == 0:
        linewidth = 1;
    else:
        linewidth = math.log10(data["weight"] / 145 + 1) * 6
    plt.plot(x_values, y_values, color="g", linewidth=linewidth)

# 显示图形
plt.title("Traffic Network Visualization")
plt.show()
