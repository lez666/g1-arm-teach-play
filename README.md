# g1-arm-teach-play

在 Unitree G1 机器人上 **录制 / 回放** 7-DoF 双臂动作的小工具。底层走官方 `rt/arm_sdk` 高层混控通道，录的是**关节空间 waypoint**，回放时腿部平衡由运控主控继续维持。

- `teach` — 重力补偿下拖动手臂记录关节姿态
- `play` — 按遥控器 L1 正向播放、L2 反向收回

> 示例场景：教机器人"端矛 / 收矛"动作。

---

## 特性

- 7×2 手臂 + 3 腰的关节空间记录/回放
- 腰部全程锁定，避免运控混控下腰漂
- 自动挑选 `192.168.123.x` 所在网卡（支持 `UNITREE_IFACE` 环境变量手动覆盖）
- 遥控器上升沿检测，避免重复触发
- LED 状态指示：
  - 绿 = 休息位
  - 蓝 = 动作进行中
  - 红 (255,80,80) = 端矛完成 / 保持
  - 熄 = 退出
- 高频刷 LED（~500ms），抢占系统守护进程
- 支持 `R1+R2` 或 `Ctrl+C` 优雅退出（走正常 weight 1→0 下课流程）

---

## 遥控器按键

| 按键 | 动作 |
|---|---|
| **L1** | 端矛（正向播放 waypoint #1 → #N） |
| **L2** | 收回（反向播放 waypoint #N → 休息位） |
| **R1 + R2** | 安全退出（weight 平滑降 0） |
| **Ctrl + C** | 同上，走正常下课流程 |

teach 模式：`Enter` 记录当前姿态，`q` 保存并退出。

---

## 硬件 / 软件要求

- Unitree G1 机器人 + PC2 (Jetson Orin NX, ARM64, Ubuntu 20.04)
- `unitree_sdk2` 最新版（含 G1 `hg` IDL 和 `AudioClient`）
- CMake ≥ 3.10、GCC ≥ 7、C++14
- DDS：使用 SDK 自带的 CycloneDDS 0.10.x（`thirdparty/lib/aarch64/`），**不要**用系统版本

---

## 编译

```bash
# 1) 安装 unitree_sdk2（如果 /opt/unitree_robotics/unitree_sdk2 已存在可跳过）
cd ~
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2 && mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics/unitree_sdk2
make -j4 && sudo make install

# 2) 编译本项目
git clone git@github.com:lez666/g1-arm-teach-play.git ~/g1_arm7
cd ~/g1_arm7 && mkdir -p build && cd build
cmake ..
make -j4
```

产物：`~/g1_arm7/build/teach`、`~/g1_arm7/build/play`

`CMakeLists.txt` 默认把 `UNITREE_SDK2_PATH` 指向 `/opt/unitree_robotics/unitree_sdk2`；如果装到别处，编译时传：

```bash
cmake .. -DUNITREE_SDK2_PATH=/your/path
```

---

## 使用流程

### ⚠️ 安全前提

- **第一次一定要吊着或有人扶**
- 机器人必须处于 **运控模式（正常站立）**；阻尼 / 零力矩模式下 arm_sdk 不生效
- `teach` 和 `play` 都要在**相同姿态**下使用（录制姿态 ≠ 回放姿态时，手的空间位置会偏）

### 1) 录制动作

```bash
cd ~/g1_arm7/build
./teach
```

程序会：
1. 自动挑 `eth1`（或任何 192.168.123.x 网卡）
2. weight 从 0 → 1 平滑接管（~3s）
3. 切到重力补偿，锁腰，手臂可拖动

交互：
```
[Enter] 记录  [q] 保存退出 >
```

多拖几个位置多记几次，输入 `q` 保存到 `arm7_action.dat`。

### 2) 回放

```bash
./play
```

- 默认绿灯等你按键
- **L1** → 蓝灯，依次走到 waypoint #1 → #N → 红灯保持
- **L2** → 蓝灯，反向回到休息位 → 绿灯
- **R1 + R2** → 安全退出

### 3) 指定网卡（可选）

```bash
UNITREE_IFACE=eth2 ./play
```

---

## 文件说明

| 文件 | 作用 |
|---|---|
| `teach.cpp` | 教学录制程序 |
| `play.cpp` | 动作回放程序 |
| `CMakeLists.txt` | 构建配置 |
| `arm7_action.dat` | 录制生成的 waypoint 文件（运行 `teach` 后产生，建议 gitignore） |

---

## G1 PC2 配置参考

以下是本项目在一台 G1 PC2（Jetson Orin NX + Ubuntu 20.04）上配置时遇到的坑和解决方案，新机器部署时可能用得上。

### 1) SDK 安装

- **一定用最新版 `unitree_sdk2`**。旧版没有 G1 `hg` IDL、也没有 `unitree::robot::g1::AudioClient`，编译会报 `dds/topic/TopicTraits.hpp` 或 `LedControl` 找不到
- 安装到 `/opt/unitree_robotics/unitree_sdk2`，本项目 CMakeLists 默认查找此路径
- 本项目链接 SDK 自带的 CycloneDDS（`thirdparty/lib/aarch64/`），**不要链系统 libddsc**（G1 预装的系统版本可能是坏的）

### 2) 网卡 / 路由：避免 `192.168.123.1` 抢占默认网关

G1 内网是 `192.168.123.0/24`，机器人主控网关在 `192.168.123.1`——但这个地址**不走公网**。很多时候 eth0/eth1 DHCP 拿到 IP 后会把默认路由指向它，导致机器人能 ping 主控但**上不了网**（无法 `apt install`、无法 `git clone github`）。

临时修复：
```bash
sudo ip route del default via 192.168.123.1
# 另外保留 wlan0 默认网关作为公网出口
```

持久化（本仓库推荐）：写一个 NetworkManager dispatcher 脚本，接口 up 时自动删这条默认路由，保留子网路由供 DDS 使用。

```bash
sudo tee /etc/NetworkManager/dispatcher.d/99-robot-route > /dev/null <<'EOF'
#!/bin/bash
IFACE="$1"
ACTION="$2"
case "$IFACE" in
    lo|docker*|veth*|l4tbr*|rndis*|usb*|dummy*) exit 0 ;;
esac
case "$ACTION" in
    up|dhcp4-change|dhcp6-change|connectivity-change)
        if ip -4 addr show dev "$IFACE" 2>/dev/null | grep -qE 'inet 192\.168\.123\.'; then
            for i in 1 2 3; do
                if ip route show default 2>/dev/null \
                    | grep -qE "default via 192\.168\.123\.1.*dev $IFACE"; then
                    ip route del default via 192.168.123.1 dev "$IFACE" 2>/dev/null || true
                    logger -t robot-route "removed default gw 192.168.123.1 via $IFACE"
                    sleep 1
                else
                    break
                fi
            done
        fi
        ;;
esac
exit 0
EOF
sudo chmod +x /etc/NetworkManager/dispatcher.d/99-robot-route
```

### 3) 系统时间：修 1970 问题

Jetson RTC 电池没电时每次重启都会回到 1970-01-01，会导致：
- 源码文件时间戳错乱，tmpfs 里的文件可能被清
- apt/git 因证书过期失败
- DDS 两端时间差过大，消息被丢

修复：
```bash
# 装 systemd-timesyncd（通常自带）
sudo apt install systemd-timesyncd
sudo systemctl enable --now systemd-timesyncd

# 等它同步完（有公网时）
timedatectl status

# 把当前时间写入硬件 RTC
sudo hwclock --systohc
```

### 4) 网卡自动挑选

G1 PC2 上 eth0 经常被拔掉或改挂，机器人可能接在 eth1。本项目 `pick_robot_iface()` 会自动扫一遍网卡、挑第一个 IP 在 `192.168.123.0/24` 的。有多张合规网卡时可以用环境变量手动指定：

```bash
UNITREE_IFACE=eth1 ./play
```

### 5) 屏蔽内核广播刷屏（可选）

如果看到 `Message from syslogd@ubuntu ... unregister_netdevice: waiting for eth0 to become free` 疯狂刷屏，那是 eth0 内核引用计数泄露（常见于 docker/veth，重启可清）。要屏蔽 wall 广播：

```bash
# 改 rsyslog：kern 模块的 emerg 消息不再 wall 广播
sudo sed -i 's|^\*\.emerg\s*:omusrmsg:\*|*.emerg;kern.none\t\t\t:omusrmsg:*|' \
    /etc/rsyslog.d/50-default.conf
sudo systemctl restart rsyslog

# 持久化降低内核控制台消息级别
echo 'kernel.printk = 1 4 1 3' | sudo tee /etc/sysctl.d/99-quiet-kernel.conf
sudo sysctl --system
```

---

## 常见问题

**Q: `./play` 报 `eth0: does not match an available interface`**
A: 你的网卡不叫 eth0。本项目已经自动探测，跑新版就行；或者 `UNITREE_IFACE=ethX ./play` 手动指定。

**Q: `./play` 报 `找不到: arm7_action.dat`**
A: 先 `./teach` 录一遍。`arm7_action.dat` 产生在**运行 play 时的当前目录**。

**Q: 遥控器按了没反应**
A: 确认机器人处于**运控模式**（不是开发/阻尼模式）。按 L1/L2 之前先看 `[2/2] 接管完成` 那行是否出现。

**Q: LED 不亮 / 很快被覆盖**
A: 本项目 500ms 刷一次 LED 抢占系统守护进程，如果还被盖，调整 `play.cpp` 里 `led_refresh >= 250` 的阈值（更小 = 更激进）。

**Q: 录了一次，回放姿态偏很多**
A: 很可能录制 / 回放是在不同模式（开发 vs 运控）下进行的，`q_init` 不一样。**在运控站立姿态下重录**。

---

## 原理简述

- `rt/arm_sdk` 是 G1 运控主控开放的**高层混控通道**，消息用 `hg` IDL 里的 `LowCmd_`
- 关键是 `motor_cmd[29]`（`kNotControl = 29`）的 `q` 字段——这是 **weight 通道**，范围 [0, 1]：
  - 0 → 主控完全管手臂
  - 1 → 我们完全接管手臂 + 腰
  - 中间 → 按比例混合
- 我们写入 7+7 手臂 + 3 腰关节的 `q/dq/kp/kd/tau`，腿部完全不碰（主控继续保持平衡）
- 接管 / 下课都通过 weight 从 0→1 或 1→0 的斜坡切换，避免冲击

---

## License

MIT
