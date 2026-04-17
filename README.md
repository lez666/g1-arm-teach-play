# g1-arm-teach-play

[English](#english) | [中文](#中文)

---

## 中文

基于 Unitree G1 controller（运控模式）的**最快捷动作开发**工具：手把手示教 → 保存 → 一键回放。

- 适用范围：**腰部及以上 17 个电机**（7×2 手臂 + 3 腰；目前腰部 pitch / roll 官方未开放，实际会被忽略）。
- 通道：官方 `rt/arm_sdk` 高层混控，腿部平衡由运控主控继续负责。

### 前置要求（请先自行配置好）

- Unitree **controller / 运控模式** 已可正常站立
- **unitree_sdk2** 已安装到 `/opt/unitree_robotics/unitree_sdk2`（需要最新版，含 G1 `hg` IDL 和 `AudioClient`）
- **CycloneDDS** 使用 SDK 自带的 `thirdparty/lib/aarch64/` 版本，不要用系统版
- CMake ≥ 3.10、GCC ≥ 7、C++14
- PC2 网络已配好（WiFi、路由、网卡）—— 详见文末 [环境配置指南](#-环境配置--environment-setup)

### 编译

```bash
git clone git@github.com:lez666/g1-arm-teach-play.git
cd g1-arm-teach-play && mkdir build && cd build
cmake .. && make -j4
```

SDK 装到非默认路径时：`cmake .. -DUNITREE_SDK2_PATH=/your/path`

### 使用

**⚠️ 第一次务必吊着或有人扶。机器人必须处在运控模式（正常站立），阻尼/零力矩下 arm_sdk 不生效。**

**1. 示教录制**
```bash
./teach
```
- weight 0→1 接管，进入重力补偿，腰部锁定
- 手动把手臂拖到想要的姿态 → `Enter` 记录一个 waypoint
- 继续拖、继续记 → `q` 保存到 `arm7_action.dat` 并退出

**2. 回放**
```bash
./play
```

| 按键 | 动作 | LED |
|---|---|---|
| L1 | 正向播放（示教顺序依次到达） | 蓝→红 |
| L2 | 反向回到休息位 | 蓝→绿 |
| R1+R2 | 安全退出（weight 平滑降 0） | 熄 |
| Ctrl+C | 同上 | 熄 |

指定网卡（可选）：`UNITREE_IFACE=eth1 ./play`，默认自动挑 `192.168.123.x` 所在网卡。

### 常见问题

- **按键没反应** → 确认运控模式、`[2/2] 接管完成` 已出现
- **找不到 `arm7_action.dat`** → 先跑 `./teach`
- **回放姿态偏很多** → 示教和回放请在**同一姿态**下进行（都在运控站立下录和放）
- **网络相关问题**（上不了网、网卡选错、WiFi 连不上）→ 见文末 [环境配置指南](#-环境配置--environment-setup)

### License

MIT

---

## English

Fastest teach-and-playback workflow for Unitree G1 upper body, built on top of the official `arm_sdk` high-level mixing channel. Teach a motion by hand → save → replay with one button.

- Scope: **17 joints from waist up** (7×2 arms + 3 waist; waist pitch / roll are currently not exposed by the controller and are silently ignored).
- Legs stay under the locomotion controller for balance.

### Prerequisites (install these yourself first)

- G1 in **normal locomotion mode** (standing, balanced).
- **unitree_sdk2** installed at `/opt/unitree_robotics/unitree_sdk2` (latest version — needs G1 `hg` IDL and `AudioClient`).
- **CycloneDDS** bundled under `thirdparty/lib/aarch64/` — do NOT use the system build.
- CMake ≥ 3.10, GCC ≥ 7, C++14.
- PC2 network already configured (WiFi, routing, interface) — see [Environment Setup](#-环境配置--environment-setup) at the bottom.

### Build

```bash
git clone git@github.com:lez666/g1-arm-teach-play.git
cd g1-arm-teach-play && mkdir build && cd build
cmake .. && make -j4
```

Custom SDK path: `cmake .. -DUNITREE_SDK2_PATH=/your/path`

### Usage

**⚠️ First run: hang the robot or have someone hold it. The robot MUST be in locomotion mode (standing). `arm_sdk` has no effect in damping / zero-torque modes.**

**1. Teach**
```bash
./teach
```
- weight ramps 0→1, gravity compensation on, waist locked.
- Drag arms by hand → `Enter` to record a waypoint.
- Repeat, then `q` to save `arm7_action.dat`.

**2. Replay**
```bash
./play
```

| Key | Action | LED |
|---|---|---|
| L1 | Forward playback (through recorded waypoints) | blue→red |
| L2 | Reverse back to rest pose | blue→green |
| R1+R2 | Graceful exit (weight smoothly to 0) | off |
| Ctrl+C | Same as above | off |

Pick interface (optional): `UNITREE_IFACE=eth1 ./play`. Default: auto-detect the `192.168.123.x` interface.

### Troubleshooting

- **Buttons do nothing** → confirm locomotion mode and that `[2/2] 接管完成` has printed.
- **`arm7_action.dat` not found** → run `./teach` first.
- **Replay pose drifts a lot** → teach & replay must be in the **same robot pose** (both in locomotion standing).
- **Network issues** (no internet, wrong interface, WiFi won't connect) → see [Environment Setup](#-环境配置--environment-setup) at the bottom.

### License

MIT

---

## 🔧 环境配置 / Environment Setup

PC2 网络、WiFi、路由、DDS 网卡等配置详见：

### 👉 [**docs/env-setup.md**](docs/env-setup.md)

Network configuration, WiFi setup, routing fixes and DDS interface selection for G1 PC2 — see [**docs/env-setup.md**](docs/env-setup.md).
