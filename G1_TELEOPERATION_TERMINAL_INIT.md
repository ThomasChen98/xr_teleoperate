# G1 遥操作终端初始化指南

本文说明在 **GPU 服务器**、**宇树机载电脑（机器人）** 与 **站端电脑（笔记本）** 上，按顺序完成隧道、策略流水线与执行客户端的初始化，以便开始数据采集。文中命令来自当前仓库脚本与 `g1_execution_client.py` 的实际参数；**未在仓库内出现的脚本**（如机器人上的 `start_image_server.sh`）仅按你提供的用法描述，**具体路径以现场部署为准**。

---

## 总览与端口约定

G1 流水线在本仓库中与 H1 区分端口（见 `scripts/integrated_training_g1.sh` 头部注释）：

| 用途 | 端口（默认） |
|------|----------------|
| 策略服务（policy server） | `8001` |
| Viser 可视化 | `8081` |
| 机器人指令（listen / viz 与 client 通信） | `5008` |

这些端口应与训练配置 YAML 中 `policy_server.port`、`visualization.viser_port`、`visualization.robot_command_port` 一致（例如 `examples/g1_control_client/training_config_g1_place_plate_mar17.yaml`）。

---

## 1. 建立与 GPU 服务器的连接并启动集成流水线

### 1.1 在站端电脑：SSH 反向/本地端口转发

在站端电脑（示例用户 `msc-locomanip`）上执行，将本机端口与远程服务器上的服务对齐：

```bash
ssh -R 5008:localhost:5008 -L 8001:localhost:8001 -L 8081:localhost:8081 P6000
```

**说明：**

- `P6000` 应为你在 `~/.ssh/config` 里配置的 **Host 别名**（或 `user@hostname`）。若你没有该别名，请改为实际的 `ssh` 目标，并保持端口映射含义不变。
- `-L 8001:localhost:8001`：站端访问本机 `8001` 即连到服务器上的策略服务（若服务只监听在服务器 `localhost`，此写法是常见用法）。
- `-L 8081:localhost:8081`：同理，用于 Viser Web UI。
- `-R 5008:localhost:5008`：使服务器上的进程能把“发往服务器某端口”的流量转到站端本机 `5008`，供站端运行的 `g1_execution_client` 接收指令（具体拓扑以你当前 `integrated_training_g1.sh` / 配置为准）。

若你更换了 YAML 中的端口，**隧道中的数字必须与配置一致**。

### 1.2 在 GPU 服务器：进入项目虚拟环境

在已通过 SSH 登录的服务器会话中（示例：`~/Projects/openpi`）：

```bash
cd ~/Projects/openpi
source .venv/bin/activate
```

**说明：** 提示符可能同时出现 Conda 的 `(base)` 等与 venv 名称叠加，属环境叠加显示问题，**以是否已 `source .venv/bin/activate` 为准**。

### 1.3 在 GPU 服务器：启动集成训练 / 服务流水线

在仓库根目录下调用脚本（可从 `scripts/` 目录用相对路径执行，脚本内部会 `cd` 到项目根）：

```bash
cd ~/Projects/openpi/scripts
./integrated_training_g1.sh --config examples/g1_control_client/training_config_g1_place_plate_mar17.yaml
```

**`integrated_training_g1.sh` 可选参数（以脚本为准）：**

| 参数 | 含义 |
|------|------|
| `--config <path>` | 训练流水线使用的 YAML 配置；**不传时**脚本默认使用 `examples/g1_control_client/training_config_g1.yaml`。 |
| `--help` | 打印用法并退出。 |

其余训练轮数、保存间隔、标注模式、策略名、数据路径等 **均从所选 YAML 读取**，不在此脚本命令行逐项展开；请直接编辑对应的 `training_config_g1_*.yaml`。

**说明：** 该脚本会按配置启动/编排策略训练、服务、可视化与数据相关步骤；完整行为见 `scripts/integrated_training_g1.sh` 内注释。完成本步且服务就绪后，**站端即可进行数据采集**（需配合下面步骤 2、3）。

---

## 2. 连接机器人并启动头部相机推流

### 2.1 SSH 登录机载电脑

在站端电脑执行（IP 与账号为你提供的示例；密码以现场为准）：

```bash
ssh unitree@192.168.123.164
```

示例密码：`123`。若与你方机器人固件或安全策略不一致，请使用实际口令。

### 2.2 启动图像服务

登录机器人后，执行你方在机载电脑上的启动方式。你提供的方式为：

```bash
./start_image_server.sh
```

**重要：** `start_image_server.sh` **不在本 openpi 仓库中**；其所在目录、是否需先 `cd` 到 `image_server` 等，**以机器人现场部署为准**。训练配置里与相机相关的地址示例见 YAML 中 `robot.head_camera_server_ip` / `robot.head_camera_server_port`（如 `192.168.123.164:5555`），需与实际 `image_server` 监听一致。

---

## 3. 站端启动 G1 执行客户端（对接策略与数据采集）

在**站端电脑**（与机器人同网、且已完成步骤 1 的隧道与服务器流水线）上，进入客户端目录并启动：

```bash
cd ~/Projects/openpi/examples/g1_control_client
python g1_execution_client.py
```

**`g1_execution_client.py` 命令行参数（以源码为准）：**

| 参数 | 含义 |
|------|------|
| `--config <path>` | 配置文件路径；**默认** `training_config_g1.yaml`（相对**当前工作目录**解析）。建议在 `g1_control_client` 目录下显式传入，例如：`--config training_config_g1_place_plate_mar17.yaml`。 |
| `--start-immediately` | 跳过等待状态，**立即**开始数据采集（见 `--help` 描述）。 |

示例（与步骤 1 使用同一任务配置）：

```bash
python g1_execution_client.py --config training_config_g1_place_plate_mar17.yaml
```

**说明：**

- 客户端会读取 YAML 中的 `policy_server`、`robot`、`data`、`recording` 等；请保证与步骤 1 的服务地址、端口及步骤 2 的相机服务一致。
- 若默认配置文件名或路径与你的工作目录不一致，**必须使用 `--config` 给出正确文件**，否则会报 “Config file not found”。

---

## 4. 你未指定、本文主动说明的要点

1. **主机别名 `P6000`**：不是仓库内路径，而是 SSH 配置中的主机名别名；每人环境不同。  
2. **隧道端口与 YAML**：修改 `training_config_g1_*.yaml` 里的端口后，必须同步修改 `ssh -L/-R` 中的端口。  
3. **`start_image_server.sh`**：不属于本仓库；仅记录你提供的用法，实际路径与启动前置条件需自行核对。  
4. **机器人 SSH 密码**：示例为 `123`；生产环境可能已修改。  
5. **依赖工具**：`integrated_training_g1.sh` 要求系统中有 `yq`、`jq`（脚本内会检查），详见该脚本开头注释。

---

## 相关文件

- 集成流水线入口：`scripts/integrated_training_g1.sh`
- 执行客户端：`examples/g1_control_client/g1_execution_client.py`
- 配置示例：`examples/g1_control_client/training_config_g1_place_plate_mar17.yaml`
- 更泛化的遥操作说明（H1-2）：`docs/TELEOPERATION_PIPELINE.md`
