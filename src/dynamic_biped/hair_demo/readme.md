# kuavo nuc 脚本文件大全
## 机器人uwb demo导航
```bash
python3 uwb_demo.py   # 机器人从出发区去到中央区
```

## 机器人执行固定动作
```bash
python3 robot_action.py bowing           # 执行固定动作 作揖
python3 robot_action.py say_hello        # 执行固定动作 打招呼 
```

## 机器人根据需求进行抓花
```bash
python3 robot_flower_traj.py Lily   # 抓取 茉莉花，然后闻花， 然后送花
python3 robot_flower_traj.py Rose   # 抓取 玫瑰花，然后闻花， 然后送花
python3 robot_flower_traj.py Tulip  # 抓取 郁金香，然后闻花， 然后送花
python3 robot_flower_traj.py 1    # 茉莉花
python3 robot_flower_traj.py 2    # 玫瑰花
python3 robot_flower_traj.py 3    # 郁金香花
```

## 机器人 机械爪张开
```bash
python3 robot_joller.py 1  # 机械爪张开  
python3 robot_joller.py 2  # 机械爪合并
```

## 机器人播放语音
```bash
python3 playmusic.py Lily_Flower     # 机器人播放 茉莉花花语
python3 playmusic.py Rose_Flower     # 机器人播放 玫瑰花语
python3 playmusic.py Tulip_Flower    # 机器人播放 郁金香花语
python3 playmusic.py say_hello_kids  # 机器人播放 你好小朋友
python3 playmusic.py say_hello_lady  # 机器人播放 你好女士
python3 playmusic.py say_hello_sir   # 机器人播放 你好先生
python3 playmusic.py say_goodbye     # 机器人播放 再见
```

## 机器人送花后调取服务回到初始位置
```bash
python3 robot_arm_toHome.py 1  # toHome tag 1 从 1号位置回到初始位置
python3 robot_arm_toHome.py 2  # toHome tag 2 从 2号位置回到初始位置
python3 robot_arm_toHome.py 3  # toHome tag 3 从 3号位置回到初始位置
```

---
# kuavo nuc 抓花脚本执行流程
```bash
# 等待引导员选择哪一朵花 打招呼,打完招呼后手臂回归到行走状态（recenter）
python3 playmusic.py say_hello_kids  # 机器人播放 你好小朋友
python3 playmusic.py say_hello_lady  # 机器人播放 你好女士
python3 playmusic.py say_hello_sir   # 机器人播放 你好先生

# 抓花中
python3 robot_flower_traj.py xx      # 抓取对应的花朵, 将花放到传感器上, 之后机器人手臂归中
python3 robot_flower_traj.py 1       # 茉莉花
python3 robot_flower_traj.py 2       # 玫瑰花
python3 robot_flower_traj.py 3       # 郁金香花

# 闻花后 三个花语 三选一
python3 playmusic.py Lily_Flower     # 机器人播放 茉莉花花语
python3 playmusic.py Rose_Flower     # 机器人播放 玫瑰花语
python3 playmusic.py Tulip_Flower    # 机器人播放 郁金香花语

# 送花后，机械爪张开
python3 robot_joller.py  1  # 10 10 机械爪张开

# 送花后，调取规划层toHome服务
python3 robot_arm_toHome.py 1  # toHome tag 1 从 1号位置回到初始位置
python3 robot_arm_toHome.py 2  # toHome tag 2 从 2号位置回到初始位置
python3 robot_arm_toHome.py 3  # toHome tag 3 从 3号位置回到初始位置

# 原地等待拍照后 播放再见语音
python3 playmusic.py say_goodbye # 机器人播放 再见  
```

