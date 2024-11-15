# 交通规划原理一阶段课程作业

## 环境与依赖

Python 3.7

```
pip install -r requirements.txt
```

## Frank-Wolfe算法部分

```
算法框架

1.初始化
	qi=0，ti=t0 （t0是每条路上的自由行驶时间）
2.dijkstra算法找最短路径
	for循环，对所有的OD对
  	（1）找到最短路径
  	（2）加载路径
3.更新行驶时间 ti = ti*BPR(qi)
4.for所有OD对
	（1）找最短路径
	（2）流量调整 （减少所有路径的流量，加到最短路径上）
5.if满足结束条件，no就返回步骤3

```

在测试过程中，我们小组发现本系统在接近目标时出现了震荡的情况。经过查阅资料，我们在Wikipedia上了解到，Frank-Wolfe算法在实现过程中需要调整步长以确保收敛。因此，我们对代码进行了相应修改，以优化算法的收敛性。

<p>
<img src="./imag/Frank-Wolfe algorithm.png" width="500"/>
</p>

优化后，我们也理解了`regularization`确实非常重要。若没有正则化，每次迭代都只会将固定的权重移动到当前最短路径上。与牛顿法等自带收敛机制的算法不同，Frank-Wolfe算法的收敛性主要依赖于我们添加的可变步长调整。

## 可视化部分

为了直观展示交通网络的流量分布，我们首先创建了一个无向图 `G`，并利用包含边和流量信息的 `final_flow_map` 字典进行遍历。通过该字典中的数据，我们将节点和边添加到图中，定义了一个节点位置字典 `pos`，并设置了图形的大小，完成了节点和边的绘制。

在绘制过程中，我们采取了一些关键步骤以优化显示效果：

1. **流量标签格式化**：为提高流量标签的可读性，对标签内容进行了格式化。
2. **边标签位置优化**：计算了边的中点及标签的偏移量，以优化标签的显示位置。
3. **边宽度调整**：对原始流量数据进行了对数变换，并基于变换后的流量值调整了边的宽度，使流量变化幅度较大的边更为直观。

这些步骤共同提升了图形的直观性和美观性，使得流量分布清晰可见，为分析交通网络的流量分布提供了有效的可视化支持。

## 小组成员及分工

| 姓名       | 学号       | 分工内容                                 | 工作量占比 |
|------------|------------|------------------------------------------|------------|
| 严子松 | 2022091201011 | Frank-Wolfe算法的具体实现及测试优化（步长调整、收敛性优化） | 30% |
| 钟昕宏 | 2022091201019 | Frank-Wolfe算法基本框架的搭建、Dijkstra最短路径算法集成 | 25% |
| 王江涛 | 2022091201004 | 交通网络流量可视化、图形绘制及边宽度优化 | 25% |
| 李耕 | 2022091201009 | 协助完成代码编写、校对文档并进行排版优化 | 20% |
| 陈奕廷 | 2022091201012 | 打酱油 | 0% |