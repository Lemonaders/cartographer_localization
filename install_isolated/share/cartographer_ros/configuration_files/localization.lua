-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.



-- 源代码
-- include "gai_backpack_2d.lua"



-- TRAJECTORY_BUILDER_2D
TRAJECTORY_BUILDER_2D = {
  use_imu_data = false,                               --要不要用IMU数据	没有IMU就写 false，有就写 true，否则会漂
  min_range = 0.5,                                    --激光点近端盲区 / 远端截止	0.5 m / 30 m：剪掉 0.5 m 内自身遮挡，30 m 外噪声大
  max_range = 30.,                                    --同上
  min_z = -0.8,                                       --激光点的z方向有效区间	‑0.8 ~ 2 m：把地面以下、天花板以上点扔掉，减小计算量（单线一般用不到，可调小区间除非有斜坡）
  max_z = 2.,                                         --同上
  missing_data_ray_length = 5.,                       --激光没返回时补一条多长“空射线”单位m，默认5m：太短会把远处空旷区域误判为障碍，太长增加计算
  num_accumulated_range_data = 3,                     --几帧激光合成一次局部扫描（已在backpack的lua中重设参数）
  voxel_filter_size = 0.05,                           --体素滤波参数

  adaptive_voxel_filter = {
    max_length = 0.5,                                 --前端/实时匹配时的体素边长上限	0.5 m：平衡精度与速度
    min_num_points = 150,                             --体素滤波保留最小点数
    max_range = 30.,                                  --参与滤波的最远距离	30 m 同激光
  },

  loop_closure_adaptive_voxel_filter = {
    max_length = 0.9,                                 --回环检测专用体素边长	0.9 m：回环允许更粗，减少候选
    min_num_points = 100,                             --保留最少点数
    max_range = 50.,                                  --最远距离
  },

  use_online_correlative_scan_matching = true,        --是否用实时相关匹配	默认 true，2D 基本都要
  real_time_correlative_scan_matcher = {
    linear_search_window = 0.2,                       --平移搜索窗 ±x/y	0.2 m：激光里程计准可设 0.1，漂移大设 0.5
    angular_search_window = math.rad(20.),            --旋转搜索窗 ±θ	math.rad(20) ≈ ±20°；角度漂移大就调大
    translation_delta_cost_weight = 1e-1,             --把“位移/旋转变化量”加入得分惩罚	1e-1：值大 → 更怕跳变，值小 → 允许大跳
    rotation_delta_cost_weight = 1e-1,                --同上
  },

  ceres_scan_matcher = {
    occupied_space_weight = 20.,                      --匹配到障碍栅格的权重	20：越大越相信地图
    translation_weight = 10.,                         --先验位姿的平移/旋转权重	10 / 40：前端准可设小，前端不准设大
    rotation_weight = 40.,                            --同上
    ceres_solver_options = {
      use_nonmonotonic_steps = false,                 --允许 Ceres 在优化过程中暂时接受“目标函数值变大”的步长，SLAM回环优化：地图很大、约束多，容易卡在局部极小，通常设 true。小规模或实时前端设false
      max_num_iterations = 20,                        --Ceres 求解最大迭代次数	20：再大收益小
      num_threads = 1,                                --Ceres 用的线程数	1：前端已经在线程池里，单线程即可
    },
  },

  motion_filter = {
    max_time_seconds = 5.,                            --超过 5 秒无激光就不建图	5：连续丢包场景可调大
    max_distance_meters = 0.2,                        --移动 < 0.2 m 不建子图	0.2 m：小车慢可减少冗余
    max_angle_radians = math.rad(1.),                 --旋转 < 1° 不建子图	math.rad(1)：陀螺仪漂移大就调大
  },

  -- TODO(schwoere,wohe): Remove this constant. This is only kept for ROS.
  imu_gravity_time_constant = 10.,                    
  pose_extrapolator = {
    use_imu_based = false,
    constant_velocity = {
      imu_gravity_time_constant = 10.,
      pose_queue_duration = 0.001,
    },
    imu_based = {
      pose_queue_duration = 5.,
      gravity_constant = 9.806,
      pose_translation_weight = 1.,
      pose_rotation_weight = 1.,
      imu_acceleration_weight = 1.,
      imu_rotation_weight = 1.,
      odometry_translation_weight = 1.,
      odometry_rotation_weight = 1.,
      solver_options = {
        use_nonmonotonic_steps = false;
        max_num_iterations = 10;
        num_threads = 1;
      },
    },
  },

  submaps = {
    num_range_data = 60,                                            --多少帧激光合成一个子图	• 建图阶段：60（约 6 秒）
    grid_options_2d = {
      grid_type = "PROBABILITY_GRID",                               --告诉 Cartographer：我要用 概率栅格 建图	别改，2D 定位就这个
      resolution = 0.05,                                            --地图分辨率 5 cm/格	• 室内精细：0.03~0.05 m
    },
    range_data_inserter = {
      range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D",
      probability_grid_range_data_inserter = {
        insert_free_space = true,                                   --激光射线穿过的地方标为“空闲”	必须 true，否则走廊会被堵
        hit_probability = 0.65,                                     --激光击中栅格→上升为障碍的概率	默认 0.65，几乎不用动
        miss_probability = 0.4,                                     --激光未击中栅格→下降为空闲的概率	默认 0.4，调大会让地图更“空”
      },
      tsdf_range_data_inserter = {                                  --2D定位该模块无效
        truncation_distance = 0.3,
        maximum_weight = 10.,
        update_free_space = false,
        normal_estimation_options = {
          num_normal_samples = 4,
          sample_radius = 0.5,
        },
        project_sdf_distance_to_scan_normal = true,
        update_weight_range_exponent = 0,
        update_weight_angle_scan_normal_to_ray_kernel_bandwidth = 0.5,
        update_weight_distance_cell_to_hit_kernel_bandwidth = 0.5,
      },
    },
  },
}





-- TRAJECTORY_BUILDER_3D
MAX_3D_RANGE = 60.
INTENSITY_THRESHOLD = 40

TRAJECTORY_BUILDER_3D = {
  min_range = 1.,
  max_range = MAX_3D_RANGE,
  num_accumulated_range_data = 1,
  voxel_filter_size = 0.15,

  high_resolution_adaptive_voxel_filter = {
    max_length = 2.,
    min_num_points = 150,
    max_range = 15.,
  },

  low_resolution_adaptive_voxel_filter = {
    max_length = 4.,
    min_num_points = 200,
    max_range = MAX_3D_RANGE,
  },

  use_online_correlative_scan_matching = false,
  real_time_correlative_scan_matcher = {
    linear_search_window = 0.15,
    angular_search_window = math.rad(1.),
    translation_delta_cost_weight = 1e-1,
    rotation_delta_cost_weight = 1e-1,
  },

  ceres_scan_matcher = {
    occupied_space_weight_0 = 1.,
    occupied_space_weight_1 = 6.,
    intensity_cost_function_options_0 = {
        weight = 0.5,
        huber_scale = 0.3,
        intensity_threshold = INTENSITY_THRESHOLD,
    },
    translation_weight = 5.,
    rotation_weight = 4e2,
    only_optimize_yaw = false,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 12,
      num_threads = 1,
    },
  },

  motion_filter = {
    max_time_seconds = 0.5,
    max_distance_meters = 0.1,
    max_angle_radians = 0.004,
  },

  rotational_histogram_size = 120,

  -- TODO(schwoere,wohe): Remove this constant. This is only kept for ROS.
  imu_gravity_time_constant = 10.,
  pose_extrapolator = {
    use_imu_based = false,
    constant_velocity = {
      imu_gravity_time_constant = 10.,
      pose_queue_duration = 0.001,
    },
    -- TODO(wohe): Tune these parameters on the example datasets.
    imu_based = {
      pose_queue_duration = 5.,
      gravity_constant = 9.806,
      pose_translation_weight = 1.,
      pose_rotation_weight = 1.,
      imu_acceleration_weight = 1.,
      imu_rotation_weight = 1.,
      odometry_translation_weight = 1.,
      odometry_rotation_weight = 1.,
      solver_options = {
        use_nonmonotonic_steps = false;
        max_num_iterations = 10;
        num_threads = 1;
      },
    },
  },

  submaps = {
    high_resolution = 0.10,
    high_resolution_max_range = 20.,
    low_resolution = 0.45,
    num_range_data = 160,
    range_data_inserter = {
      hit_probability = 0.55,
      miss_probability = 0.49,
      num_free_space_voxels = 2,
      intensity_threshold = INTENSITY_THRESHOLD,
    },
  },

  -- When setting use_intensities to true, the intensity_cost_function_options_0
  -- parameter in ceres_scan_matcher has to be set up as well or otherwise
  -- CeresScanMatcher will CHECK-fail.
  use_intensities = false,
}





-- pose_graph
POSE_GRAPH = {
  optimize_every_n_nodes = 90,                    --每90个节点（激光帧）跑一次全局优化，调大 → 120~200，减少 CPU 占用；极端可设 0（只在回环触发）
  constraint_builder = {
    sampling_ratio = 0.2,                         --只拿 20 % 的激光帧做约束计算，可降到 0.1~0.15，速度↑，精度略降
    max_constraint_distance = 20.,                --只在 20 m 内找回环，室内小图可降到 10 m；室外大图保持 20 m
    min_score = 0.6,                              --回环最低匹配得分阈值，定位漂移大、环境重复 → 降到 0.5；环境特征丰富 → 升到 0.65
    global_localization_min_score = 0.6,          --同上，不过是重定位得分
    loop_closure_translation_weight = 1.1e4,      --回环约束在优化里的“可信度”，一般不动；除非地图/里程计明显不准才微调
    loop_closure_rotation_weight = 1e5,           --同上
    log_matches = true,                           --把匹配对写入日志，方便 debug，定位阶段可设 false 省磁盘
    fast_correlative_scan_matcher = {
      linear_search_window = 5.,                  --平移/旋转搜索窗口，漂移小可缩小到 3 m / 15°，减少候选
      angular_search_window = math.rad(20.),      --同上
      branch_and_bound_depth = 6,                 --多分辨率搜索层数，尽量不要改；层数低会漏检回环
    },
    ceres_scan_matcher = {
      occupied_space_weight = 20.,                --栅格/位移/旋转权重，尽量不动
      translation_weight = 10.,                   --同上
      rotation_weight = 1.,                       --同上
      ceres_solver_options = {
        use_nonmonotonic_steps = true,            --允许 Ceres 在优化过程中暂时接受“目标函数值变大”的步长，SLAM回环优化：地图很大、约束多，容易卡在局部极小，通常设 true。小规模或实时前端设false
        max_num_iterations = 10,                  --Ceres 最大迭代 / 线程，不动；线程已由 Cartographer 线程池控制
        num_threads = 1,                          --同上
      },
    },
    fast_correlative_scan_matcher_3d = {
      branch_and_bound_depth = 8,
      full_resolution_depth = 3,
      min_rotational_score = 0.77,
      min_low_resolution_score = 0.55,
      linear_xy_search_window = 5.,
      linear_z_search_window = 1.,
      angular_search_window = math.rad(15.),
    },
    ceres_scan_matcher_3d = {
      occupied_space_weight_0 = 5.,
      occupied_space_weight_1 = 30.,
      translation_weight = 10.,
      rotation_weight = 1.,
      only_optimize_yaw = false,
      ceres_solver_options = {
        use_nonmonotonic_steps = false,
        max_num_iterations = 10,
        num_threads = 1,
      },
    },
  },
  matcher_translation_weight = 5e2,                 --前端扫描匹配给出的 平移 残差在全局优化里的信任度	前端准就 5e2；漂移大降到 1e2
  matcher_rotation_weight = 1.6e3,                  --前端给出的 旋转 残差信任度	同上，旋转误差大时调小
  optimization_problem = {
    huber_scale = 1e1,                              --Huber 损失阈值，误差>1e1 时转为线性，防外点爆炸	1~10 均可；GPS 噪声大时可设 20
    acceleration_weight = 1.1e2,                    --IMU 加速度 /角速度 残差权重	有 IMU 就 1e2；无 IMU 可 0
    rotation_weight = 1.6e4,                        --同上
    local_slam_pose_translation_weight = 1e5,       --局部节点位姿在全局优化中的“硬度”	值越大越相信局部结果；通常 1e5~1e6
    local_slam_pose_rotation_weight = 1e5,          --同上
    odometry_translation_weight = 1e5,              --里程计观测权重	轮式里程计准就 1e5；打滑时降到 1e3~1e4
    odometry_rotation_weight = 1e5,                 --同上
    fixed_frame_pose_translation_weight = 1e1,      --GPS相关
    fixed_frame_pose_rotation_weight = 1e2,
    fixed_frame_pose_use_tolerant_loss = false,
    fixed_frame_pose_tolerant_loss_param_a = 1,
    fixed_frame_pose_tolerant_loss_param_b = 1,
    log_solver_summary = false,
    use_online_imu_extrinsics_in_3d = true,
    fix_z_in_3d = false,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,              --是否允许非单调下降	小场景更快收敛；大场景可 true 防局部极小
      max_num_iterations = 30,                     --每次全局优化最大迭代	30 通常够；地图超大可 50
      num_threads = 7,                             --Ceres 多线程	设成 CPU 核心数即可
    },
  },
  max_num_final_iterations = 200,                   --最终一次性“彻底优化”的最大迭代	建图结束跑一次；200 足够
  global_sampling_ratio = 0.003,                    --回环候选帧采样比例	0.003 ≈ 每 300 帧抽 1 帧；地图大可降到 0.001
  log_residual_histograms = true,                   --把残差分布写日志	调参阶段 true，上线后 false 省磁盘
  global_constraint_search_after_n_seconds = 10.,   --启动后多少秒开始全局回环搜索	10 s：机器人启动走一段再搜，防止初始乱回环
  --  overlapping_submaps_trimmer_2d = {
  --    fresh_submaps_count = 1,
  --    min_covered_area = 2,
  --    min_added_submaps_count = 5,
  --  },
}





-- map_builder
MAP_BUILDER = {
  use_trajectory_builder_2d = false,                  --标志位，在localization中覆盖置1
  use_trajectory_builder_3d = false,                  --同上，2D中为0
  num_background_threads = 7,                         --后台线程池大小（全局优化、回环检测）	CPU 逻辑核心数 或 物理核心×1~1.5
  pose_graph = POSE_GRAPH,                            --挂到哪个全局位姿图配置
  collate_by_trajectory = false,                      --是否按轨迹号把传感器数据分组再发下去	false 最常见：所有轨迹共用数据队列；多机器人时才设 true
}




TRAJECTORY_BUILDER = {
  trajectory_builder_2d = TRAJECTORY_BUILDER_2D,     --传递详细参数表（trajectory_builder_2d.lua）
  trajectory_builder_3d = TRAJECTORY_BUILDER_3D,     --同上
--  pure_localization_trimmer = {
--    max_submaps_to_keep = 3,
--  },
  collate_fixed_frame = true,                        --是否把 /fixed_frame_pose（GPS/UWB）数据按时间戳排序后再用	true → 必须排序，否则 GPS 乱序会跳；无外部全局观测可设 false
  collate_landmarks = false,                         --是否把 /landmark 数据排序	没人工地标就 false
}





TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,                         --最大活动子图，主要占用内存
}
POSE_GRAPH.optimize_every_n_nodes = 40             --每收到 40 个节点（激光帧/扫描）就触发一次全局位姿图优化，吃cpu，参考范围20-100



-- include "gai_backpack_2d.lua"
options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",                              --地图坐标系名称，一般默认map参数
  tracking_frame = "base_link",                   --SLAM追踪坐标系，车体其他传感器都将转移到此坐标系，一般选择IMU坐标系
  published_frame = "odom",                       --carto提供的map或odom的tf连接到哪个坐标系，与provide_odom_frame联动，一般是选择tf树顶端frame
  odom_frame = "odom",                            --odom坐标系名称
  provide_odom_frame = false,                     --是否需要carto提供odom坐标系，与published_frame联动，如果需要提供odom则pub的tf为odom->指定frame
  publish_frame_projected_to_2d = true,           --把 3D 位姿压成纯 2D（z=0，无 roll/pitch）再发布
  use_pose_extrapolator = false,                  --发布 tf 时用外推器还是前端直接算，默认 false，前端更准
  use_odometry = true,                            --是否使用odom
  use_nav_sat = false,                            --是否使用GPS
  use_landmarks = false,                          --是否使用人工地标
  num_laser_scans = 1,                            --单线激光雷达个数
  num_multi_echo_laser_scans = 0,                 --多回波雷达个数
  num_subdivisions_per_laser_scan = 1,            --一帧雷达帧分成几次处理，雷达频率太高可调整
  num_point_clouds = 0,                           --点云数量（3D雷达）
  lookup_transform_timeout_sec = 0.1,             --TF查找超时时间
  submap_publish_period_sec = 0.3,                --submap发布间隔
  pose_publish_period_sec = 5e-3,                 --机器人位姿发布间隔
  trajectory_publish_period_sec = 30e-3,          --整条轨迹发布间隔
  rangefinder_sampling_ratio = 1.,                --雷达实际使用比例（采样率），高频的话可以少用一些也就是降频
  odometry_sampling_ratio = 1.,                   --同上odom采样率，轮子打滑等情况存在可下调至0.3左右
  fixed_frame_pose_sampling_ratio = 1.,           --GPS采样率
  imu_sampling_ratio = 1.,                        --IMU采样率，漂移太大直接置0
  landmarks_sampling_ratio = 1.,                  --人工地标采样率
}

MAP_BUILDER.use_trajectory_builder_2d = true                --使用2d轨迹构建器及其对应参数
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 10       --几帧激光合成一次局部扫描	激光频率 10 Hz、雷达慢就设 10；频率高可设 1



return options
