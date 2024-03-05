#!/usr/bin/env python
from __future__ import division
from threading import Lock
import numpy as np
import range_libc
import rospy
from sensor_msgs.msg import LaserScan
from rospy.numpy_msg import numpy_msg

class SingleBeamSensorModel:
    """单激光束传感器模型。"""

    def __init__(self, **kwargs):
        """初始化单光束传感器模型。

        参数:
            **kwargs (object): 可选关键字参数数量可以是任意多个：
                hit_std (float): 击中读数的噪声值
                z_hit (float): 击中读数的权重
                z_short (float): 短距离读数的权重
                z_max (float): 最大距离读数的权重
                z_rand (float): 随机读数的权重
        """
        defaults = {
            "hit_std": 1.0,  # 默认击中读数的标准差
            "z_hit": 0.5,    # 默认击中读数的权重
            "z_short": 0.05, # 默认短距离读数的权重
            "z_max": 0.05,   # 默认最大距离读数的权重
            "z_rand": 0.5,   # 默认随机读数的权重
        }
        if not set(kwargs).issubset(set(defaults)):
            raise ValueError("Invalid keyword argument provided")
        # 下面这两行代码从defaults和kwargs字典设置实例属性。
        # 例如，键"hit_std"变成了实例属性self.hit_std。
        self.__dict__.update(defaults)
        self.__dict__.update(kwargs)

        if (
            self.z_short == 0
            and self.z_max == 0
            and self.z_rand == 0
            and self.z_hit == 0
        ):
            raise ValueError(
                "给定参数未定义模型。"
                "你必须至少为模型的一部分提供一个非0值。"
            )

    def precompute_sensor_model(self, max_r):
        """预计算所有模拟和观测距离测量对的传感器模型可能性。

        概率存储在一个2D数组中，其中索引(r, d)处的元素是在模拟（期望）测量为d时，
        观察到的测量结果r的概率。

        您需要对表格进行归一化以确保概率之和为1，即
        对于所有的d，所有r的P(r | d)之和应该为1。

        参数:
            max_r (int): 最大范围（以像素为单位）

        返回:
            prob_table: 形状为(max_r+1, max_r+1)的np.array，
                包含传感器概率P(r | d)，或者从讲座中的P(z_t^k | z_t^k*)。
        """
        table_width = int(max_r) + 1
        prob_table = np.zeros((table_width, table_width))

        # 获取与prob_table形状相同的矩阵，
        # 其中每个条目都持有真实测量r（obs_r）
        # 或模拟（期望）测量d（sim_r）。
        obs_r, sim_r = np.mgrid[0:table_width, 0:table_width]

        # 使用obs_r和sim_r来矢量化传感器模型的预计算。
        diff = sim_r - obs_r
        # BEGIN SOLUTION "QUESTION 2.1"
        if self.hit_std > 0:
            prob_table += (
                self.z_hit
                * np.exp(-0.5 * (diff / self.hit_std) ** 2)
                / (self.hit_std * np.sqrt(2.0 * np.pi))
            )

        prob_table[obs_r < sim_r] += (
            self.z_short * (2 * diff[obs_r < sim_r]) / sim_r[obs_r < sim_r]
        )
        prob_table[obs_r == max_r] += self.z_max
        prob_table[obs_r < max_r] += np.true_divide(self.z_rand, max_r)

        # 对概率表格进行归一化，确保每个d的概率之和为1。
        prob_table /= prob_table.sum(axis=0, keepdims=True)
        # END SOLUTION

        return prob_table



class LaserScanSensorModelROS:
    """A ROS subscriber that reweights particles according to the sensor model.

    This applies the sensor model to the particles whenever it receives a
    message from the laser scan topic.

    These implementation details can be safely ignored, although you're welcome
    to continue reading to better understand how the entire state estimation
    pipeline is connected.
    """

    def __init__(
        self, particles, weights, sensor_params=None, state_lock=None, **kwargs
    ):
        """Initialize the laser scan sensor model ROS subscriber.

        Args:
            particles: the particles to update
            weights: the weights to update
            sensor_params: a dictionary of parameters for the sensor model
            state_lock: guarding access to the particles and weights during update,
                since both are shared variables with other processes
            **kwargs: Required unless marked otherwise
                laser_ray_step (int): Step for downsampling laser scans
                exclude_max_range_rays (bool): Whether to exclude rays that are
                    beyond the max range
                map_msg (nav_msgs.msg.MapMetaData): Map metadata to use
                car_length (float): the length of the car
                theta_discretization (int): Discretization of scanning angle. Optional
                inv_squash_factor (float): Make the weight distribution less peaked

        """
        if not particles.shape[0] == weights.shape[0]:
            raise ValueError("Must have same number of particles and weights")
        self.particles = particles
        self.weights = weights
        required_keyword_args = {
            "laser_ray_step",
            "exclude_max_range_rays",
            "max_range_meters",
            "map_msg",
            "car_length",
        }
        if not required_keyword_args.issubset(set(kwargs)):
            raise ValueError("Missing required keyword argument")
        defaults = {
            "theta_discretization": 112,
            "inv_squash_factor": 0.2,
        }
        # These next two lines set the instance attributes from the defaults and
        # kwargs dictionaries.
        self.__dict__.update(defaults)
        self.__dict__.update(kwargs)

        self.half_car_length = self.car_length / 2
        self.state_lock = state_lock or Lock()
        sensor_params = {} if sensor_params is None else sensor_params
        sensor_model = SingleBeamSensorModel(**sensor_params)

        # Create map
        o_map = range_libc.PyOMap(self.map_msg)
        # the max range in pixels of the laser
        max_range_px = int(self.max_range_meters / self.map_msg.info.resolution)
        self.range_method = range_libc.PyCDDTCast(
            o_map, max_range_px, self.theta_discretization
        )
        # Alternative range method that can be used for ray casting
        # self.range_method = range_libc.PyRayMarchingGPU(o_map, max_range_px)
        self.range_method.set_sensor_model(
            sensor_model.precompute_sensor_model(max_range_px)
        )

        self.queries = None
        self.ranges = None
        self.laser_angles = None  # The angles of each ray
        self.do_resample = False  # Set for outside code to know when to resample

        self.initialized = False
        # Subscribe to laser scans
        self.laser_sub = rospy.Subscriber(
            "scan", numpy_msg(LaserScan), self.lidar_callback, queue_size=1
        )

    def start(self):
        self.initialized = True

    def lidar_callback(self, msg):
        """Apply the sensor model to downsampled laser measurements.

        Args:
            msg: a sensor_msgs/LaserScan message
        """
        # Initialize angles
        if self.laser_angles is None:
            self.laser_angles = np.linspace(
                msg.angle_min, msg.angle_max, len(msg.ranges)
            )

        if not self.initialized:
            return

        ranges, angles = self.downsample(msg.ranges)

        # Acquire the lock that synchronizes access to the particles. This is
        # necessary because self.particles is shared by the other particle
        # filter classes.
        #
        # The with statement automatically acquires and releases the lock.
        # See the Python documentation for more information:
        # https://docs.python.org/3/library/threading.html#using-locks-conditions-and-semaphores-in-the-with-statement
        with self.state_lock:
            self.apply_sensor_model(ranges, angles)
            self.weights /= np.sum(self.weights)

        self.last_laser = msg
        self.do_resample = True

    def apply_sensor_model(self, obs_ranges, obs_angles):
        """
        根据观测到的激光雷达扫描数据更新粒子的权重。
        
        参数：
            obs_ranges (np.ndarray): 观测到的距离测量值。
            obs_angles (np.ndarray): 观测到的激光束角度。
        """

        # 获取观测到的激光束数量。
        num_rays = obs_angles.shape[0]

        # 初始化查询缓冲区。每个粒子的状态包括其位置和方向。
        num_particles = self.particles.shape[0]
        if self.queries is None:
            self.queries = np.zeros((num_particles, 3), dtype=np.float32)
        
        # 初始化存储从模拟激光雷达返回的预期范围的数组。
        if self.ranges is None:
            self.ranges = np.zeros(num_rays * num_particles, dtype=np.float32)

        # 复制粒子状态到查询缓冲区。
        self.queries[:, :] = self.particles[:, :]
        
        # 更新查询点的位置，使其反映车辆前部的激光传感器的实际位置。
        self.queries[:, 0] += self.half_car_length * np.cos(self.queries[:, 2])
        self.queries[:, 1] += self.half_car_length * np.sin(self.queries[:, 2])

        # 对于每个粒子的每个观测角度，使用射线投射方法计算预期的测量距离。
        self.range_method.calc_range_repeat_angles(
            self.queries, obs_angles, self.ranges
        )

        # 使用传感器模型评估得到的预期范围与观测范围之间的差异，并更新每个粒子的权重。
        self.range_method.eval_sensor_model(
            obs_ranges, self.ranges, self.weights, num_rays, self.particles.shape[0]
        )

        # 为了避免权重过于集中在少数粒子上，将权重进行幂运算以压缩权重分布。
        # 这有助于维持粒子群体的多样性，并防止过早收敛到局部最优解。
        np.power(self.weights, self.inv_squash_factor, self.weights)


    def downsample(self, ranges):
        """
        对激光束进行降采样。

        参数:
            ranges: 所有观测到的距离测量值

        返回:
            ranges: 降采样后的观测到的距离测量值
            angles: 降采样后的观测到的激光束角度
        """
        if not self.exclude_max_range_rays:
            # 如果不排除最大范围内的激光束，则对激光角度进行降采样，并将NaN值替换为最大范围值。
            angles = np.copy(self.laser_angles[0 :: self.laser_ray_step]).astype(
                np.float32
            )
            sampled = ranges[:: self.laser_ray_step].copy()
            sampled[np.isnan(sampled)] = self.max_range_meters
            sampled[np.abs(sampled) < 1e-3] = self.max_range_meters
            return sampled, angles

        # 如果我们试图避免复制ranges数组，因此在比较NaN时消除错误而不是覆盖这些值
        with np.errstate(invalid="ignore"):
            valid_indices = np.logical_and(
                ~np.isnan(ranges), ranges > 0.01, ranges < self.max_range_meters
            )
        filtered_ranges = ranges[valid_indices]
        filtered_angles = self.laser_angles[valid_indices]

        # 计算预期的光线数量
        ray_count = int(self.laser_angles.shape[0] / self.laser_ray_step)
        num_valid = filtered_angles.shape[0]
        sample_indices = np.arange(0, num_valid, float(num_valid) / ray_count).astype(
            np.int64
        )
        # BEGIN SOLUTION NO PROMPT
        # self.downsampled_angles = np.zeros(ray_count + 1, dtype=np.float32) # TODO 为什么加1？
        # self.downsampled_ranges = np.zeros(ray_count + 1, dtype=np.float32) # 如果这个方法有效，删除这些行
        # self.downsampled_angles[:sample_indices.shape[0]] = np.copy(...)
        # END SOLUTION
        # 对过滤后的角度和范围进行降采样
        angles = np.copy(filtered_angles[sample_indices]).astype(np.float32)
        ranges = np.copy(filtered_ranges[sample_indices]).astype(np.float32)
        return ranges, angles
