# TrajectoryPlanner
A trajectory planner via moveIt

```bash

├── config    # moveit配置文件
├── example   # 示例程序
├── traj      # 生成好的轨迹
├── __init__.py  # 初始化 
├── base.py      # 控制基础类
├── calibrator.py  # 校准器
├── exception.py   # 异常类
├── executor.py    # 执行器
├── logger.py      # 日志类
├── optimizer.py   # 优化器
├── planner.py     # 规划器
├── publisher.py   # 发布器
├── scene.py       # 场景类
├── transformer.py # 变换器
├── utils.py       # 工具类
├── test_axis_pose_moveit_executor.py  # 输入目标检测结果的末端pose 但是每次规划都是从 当前手臂位置 开始
├── test_axis_pose_moveit.py           # 输入目标检测结果的末端pose 但是每次规划都是从 初始零点位置 开始
├── test_ik_more_point_moveit.py       # 输入多个末端点 然后通过moveit逆解 moveit规划 
├── test_ik_single_point_moveit.py     # 输入单个末端点 然后通过moveit逆解 moveit规划 
├── test_joint_demo_01.py              # 输入单组多个关节角度 moveit规划 
├── test.py                            # 输入多组多个关节角度 moveit规划 
└── README.md
```