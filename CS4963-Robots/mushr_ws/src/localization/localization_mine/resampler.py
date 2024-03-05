#!/usr/bin/env python
from __future__ import division

from threading import Lock

import numpy as np


class LowVarianceSampler:
    """低方差粒子采样器。"""

    def __init__(self, particles, weights, state_lock=None):
        """初始化粒子采样器。

        参数:
            particles: 需要更新的粒子集合
            weights: 需要更新的权重集合
            state_lock: 保护对粒子和权重的访问，因为它们是与其他进程共享的变量
        """
        self.particles = particles
        self.weights = weights
        self.state_lock = state_lock or Lock()  # 如果没有提供锁，则创建一个新锁
        self.n_particles = particles.shape[0]   # 粒子数量

        # 你可能想要在这里缓存一些中间变量以提高效率
        # BEGIN SOLUTION NO PROMPT
        self.step_array = np.arange(self.n_particles, dtype=np.float32)
        self.step_array /= self.n_particles  # 步长数组，用于生成采样点
        self.indices = np.zeros(self.n_particles, dtype=int)  # 存储被采样粒子的索引
        # END SOLUTION

    def resample(self):
        """使用低方差采样方案重新采样粒子。

        self.particles 和 self.weights 应原地修改。
        """
        # 获取同步访问粒子的锁。这是必要的，因为 self.particles 是由其他粒子
        # 过滤类共享的。
        #
        # with语句自动获取和释放锁。
        # 参见Python文档获取更多信息：
        # https://docs.python.org/3/library/threading.html#using-locks-conditions-and-semaphores-in-the-with-statement
        with self.state_lock:
            # BEGIN SOLUTION "QUESTION 3.2"
            # 从半开区间 [0, 1/M) 中选择一个初始值
            initval = np.random.uniform(0, self.n_particles ** -1)

            # 获取分区间隔
            bin_parts = initval + self.step_array
            # 最后一个bin_parts的值 = r + 1 - (1/M)
            # 其中 r 在 [0, 1/M) 中

            # 获取权重的累积和。请注意，
            # 条目之间的空间是与该索引处粒子的权重成比例的区间
            cum_weights = np.cumsum(self.weights)

            # 简洁的 O(M log M) 实现方式
            self.indices = np.searchsorted(cum_weights, bin_parts, side="left")

            assert np.all(self.indices < self.n_particles)  # 确保所有索引都有效
            self.particles[:] = self.particles[self.indices, :]  # 通过索引更新粒子

            # 为新粒子赋予统一的权重
            self.weights.fill(1.0 / self.n_particles)
            # END SOLUTION
