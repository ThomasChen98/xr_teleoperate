<div align="center">
  <h1 align="center"> xr_teleoperate </h1>
  <a href="https://www.unitree.com/" target="_blank">
    <img src="https://www.unitree.com/images/0079f8938336436e955ea3a98c4e1e59.svg" alt="Unitree LOGO" width="15%">
  </a>
  <p align="center">
    <a href="README.md"> English </a> | <a>中文</a> | <a href="README_ja-JP.md">日本語</a>
  </p>
  <p align="center">
    <a href="https://github.com/unitreerobotics/xr_teleoperate/wiki" target="_blank"> <img src="https://img.shields.io/badge/GitHub-Wiki-181717?logo=github" alt="Unitree LOGO"></a> <a href="https://discord.gg/ZwcVwxv5rq" target="_blank"><img src="https://img.shields.io/badge/-Discord-5865F2?style=flat&logo=Discord&logoColor=white" alt="Unitree LOGO"></a>
  </p>
</div>


# 📺 视频演示

<p align="center">
  <table>
    <tr>
      <td align="center" width="50%">
        <a href="https://www.bilibili.com/video/BV124m8YXExJ" target="_blank">
          <img src="./img/video_cover.jpg" alt="Video 1" width="75%">
        </a>
        <p><b> G1 (29自由度) + Dex3-1</b></p>
      </td>
      <td align="center" width="50%">
        <a href="https://www.bilibili.com/video/BV1SW421X7kA" target="_blank">
          <img src="./img/video_cover2.jpg" alt="Video 2" width="75%">
        </a>
        <p><b> H1_2 (手臂7自由度) </b></p>
      </td>
    </tr>
  </table>
</p>


# 🔖 版本说明

## 🏷️ v1.2

1. 升级Dex1_1夹爪控制代码，匹配 [dex1_1 service](https://github.com/unitreerobotics/dex1_1_service) 驱动

## 🏷️ v1.1

1. 末端执行器类型新增'brainco'，这是[强脑科技第二代灵巧手](https://www.brainco-hz.com/docs/revolimb-hand/)
2. 为避免与实机部署时发生冲突，将仿真模式下的 dds 通道的domain id修改为1
3. 修复默认频率过高的问题

## 🏷️ v1.0 (newvuer)

1. 升级 [Vuer](https://github.com/vuer-ai/vuer) 库至 v0.0.60 版本，XR设备支持模式扩展为**手部跟踪**和**控制器跟踪**两种。为更准确反映功能范围，项目由 **avp_teleoperate** 更名为 **xr_teleoperate**。

   测试设备包括： Apple Vision Pro，Meta Quest 3（含手柄） 与 PICO 4 Ultra Enterprise（含手柄）。

2. 对部分功能进行了**模块化**拆分，并通过 Git 子模块（git submodule）方式进行管理和加载，提升代码结构的清晰度与维护性。

3. 新增**无头**、**运控**及**仿真**模式，优化启动参数配置（详见第2.2节），提升使用便捷性。**仿真**模式的加入，方便了环境验证和硬件故障排查。

4. 将默认手部映射算法从 Vector 切换为 **DexPilot**，优化了指尖捏合的精度与交互体验。

5. 其他一些优化

## 🏷️ v0.5 (oldvuer)

1. 该版本曾经命名为 `avp_teleoperate`
2. 支持 'G1_29', 'G1_23', 'H1_2', 'H1' 机器人类型
3. 支持 'dex3', 'gripper', 'inspire1' 末端执行器类型
4. 仅支持 XR 设备的手部跟踪模式（ [Vuer](https://github.com/vuer-ai/vuer) 版本为 v0.0.32RC7），不支持控制器模式
5. 支持数据录制模式



# 0. 📖 介绍

该仓库实现了使用 **XR设备（Extended Reality）**（比如 Apple Vision Pro、PICO 4 Ultra Enterprise 或 Meta Quest 3 等） 对 **宇树（Unitree）人形机器人** 的遥操作控制。

> 如果您之前从没有使用过宇树机器人，那么请您至少先阅读至[官方文档](https://support.unitree.com/main/zh)应用开发章节。
> 
> 另外，本仓库的[维基文档](https://github.com/unitreerobotics/xr_teleoperate/wiki)也有很多相关知识可以供您参考。

以下是系统示意图：

<p align="center">
  <a href="https://oss-global-cdn.unitree.com/static/54cff8fe11ac4158b9b15a73f0842843_5990x4060.png">
    <img src="https://oss-global-cdn.unitree.com/static/54cff8fe11ac4158b9b15a73f0842843_5990x4060.png" alt="Watch the Document" style="width: 100%;">
  </a>
</p>

以下是本仓库目前支持的设备类型：

<table>
  <tr>
    <th style="text-align: center;"> &#129302; 机器人 </th>
    <th style="text-align: center;"> &#9898; 状态 </th>
  </tr>
  <tr>
    <td style="text-align: center;"> <a href="https://www.unitree.com/cn/g1" target="_blank"> G1 (29自由度) </td>
    <td style="text-align: center;"> &#9989; 完成 </td>
  </tr>
  <tr>
    <td style="text-align: center;"> <a href="https://www.unitree.com/cn/g1" target="_blank"> G1 (23自由度) </td>
    <td style="text-align: center;"> &#9989; 完成 </td>
  </tr>
  <tr>
    <td style="text-align: center;"> <a href="https://www.unitree.com/cn/h1" target="_blank"> H1 (手臂4自由度) </td>
    <td style="text-align: center;"> &#9989; 完成 </td>
  </tr>
  <tr>
    <td style="text-align: center;"> <a href="https://www.unitree.com/cn/h1" target="_blank"> H1_2 (手臂7自由度) </td>
    <td style="text-align: center;"> &#9989; 完成 </td>
  </tr>
  <tr>
    <td style="text-align: center;"> <a href="https://www.unitree.com/cn/Dex1-1" target="_blank"> Dex1-1 夹爪 </td>
    <td style="text-align: center;"> &#9989; 完成 </td>
  </tr>
  <tr>
    <td style="text-align: center;"> <a href="https://www.unitree.com/cn/Dex3-1" target="_blank"> Dex3-1 灵巧手 </td>
    <td style="text-align: center;"> &#9989; 完成 </td>
  </tr>
  <tr>
    <td style="text-align: center;"> <a href="https://support.unitree.com/home/zh/G1_developer/inspire_dfx_dexterous_hand" target="_blank"> 因时灵巧手 </td>
    <td style="text-align: center;"> &#9989; 完成 </td>
  </tr>
  <tr>
    <td style="text-align: center;"> <a href="https://www.brainco-hz.com/docs/revolimb-hand/" target="_blank"> 强脑灵巧手 </td>
    <td style="text-align: center;"> &#9989; 完成 </td>
  </tr>
  <tr>
    <td style="text-align: center;"> ··· </td>
    <td style="text-align: center;"> ··· </td>
  </tr>
</table>



# 1. 📦 安装

我们在 Ubuntu 20.04 和 Ubuntu 22.04 上测试了我们的代码，其他操作系统可能需要不同的配置。本文档主要介绍常规模式。

有关更多信息，您可以参考 [官方文档](https://support.unitree.com/home/zh/Teleoperation) 和 [OpenTeleVision](https://github.com/OpenTeleVision/TeleVision)。

## 1.1 📥 基础环境

```bash
# 创建 conda 基础环境
(base) unitree@Host:~$ conda create -n tv python=3.10 pinocchio=3.1.0 numpy=1.26.4 -c conda-forge
(base) unitree@Host:~$ conda activate tv
# 克隆本仓库
(tv) unitree@Host:~$ git clone https://github.com/unitreerobotics/xr_teleoperate.git
(tv) unitree@Host:~$ cd xr_teleoperate
# 浅克隆子模块
(tv) unitree@Host:~/xr_teleoperate$ git submodule update --init --depth 1
# 安装 televuer 模块
(tv) unitree@Host:~/xr_teleoperate$ cd teleop/televuer
(tv) unitree@Host:~/xr_teleoperate/teleop/televuer$ pip install -e .
# 生成 televuer 模块所需的证书文件
(tv) unitree@Host:~/xr_teleoperate/teleop/televuer$ openssl req -x509 -nodes -days 365 -newkey rsa:2048 -keyout key.pem -out cert.pem
# 安装 dex-retargeting 模块
(tv) unitree@Host:~/xr_teleoperate/teleop/televuer$ cd ../robot_control/dex-retargeting/
(tv) unitree@Host:~/xr_teleoperate/teleop/robot_control/dex-retargeting$ pip install -e .
# 安装本仓库所需的其他依赖库
(tv) unitree@Host:~/xr_teleoperate/teleop/robot_control/dex-retargeting$ cd ../../../
(tv) unitree@Host:~/xr_teleoperate$ pip install -r requirements.txt
```

## 1.2 🕹️ unitree_sdk2_python

```bash
# 安装 unitree_sdk2_python 库，该库负责开发设备与机器人之间的通信控制功能
(tv) unitree@Host:~$ git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
(tv) unitree@Host:~$ cd unitree_sdk2_python
(tv) unitree@Host:~/unitree_sdk2_python$ pip install -e .
```

> 注意1：在 `xr_teleoperate >= v1.1` 版本中，`unitree_sdk2_python` 仓库的 commit **必须是等于或高于** [404fe44d76f705c002c97e773276f2a8fefb57e4](https://github.com/unitreerobotics/unitree_sdk2_python/commit/404fe44d76f705c002c97e773276f2a8fefb57e4) 版本

> 注意2：原 h1_2 分支中的 [unitree_dds_wrapper](https://github.com/unitreerobotics/unitree_dds_wrapper) 为临时版本，现已全面转换到上述正式的 Python 版控制通信库：[unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python)

> 注意3：命令前面的所有标识符是为了提示：该命令应该在哪个设备和目录下执行。
>
> p.s. 在 Ubuntu 系统 `~/.bashrc` 文件中，默认配置: `PS1='${debian_chroot:+($debian_chroot)}\u@\h:\w\$ '`
>
> - 以`(tv) unitree@Host:~$ pip install meshcat` 命令为例：
>
> - `(tv)` 表示 shell 此时位于 conda 创建的 tv 环境中；
>
> - `unitree@Host:~` 表示用户标识 unitree 在设备 Host 上登录，当前的工作目录为 `$HOME`；
>
> - $ 表示当前 shell 为 Bash；
>
> - pip install meshcat 是用户标识 unitree 要在 设备 Host 上执行的命令。
>
> 您可以参考 [Harley Hahn's Guide to Unix and Linux](https://www.harley.com/unix-book/book/chapters/04.html#H) 和 [Conda User Guide](https://docs.conda.io/projects/conda/en/latest/user-guide/getting-started.html) 来深入了解这些知识。



# 2. 💻 仿真部署

## 2.1 📥 环境配置

首先，请安装 [unitree_sim_isaaclab](https://github.com/unitreerobotics/unitree_sim_isaaclab)。具体安装步骤，可参考该仓库 README 文档。

其次，启动 unitree_sim_isaaclab 仿真环境。假设使用 G1(29 DoF) 和 Dex3 灵巧手配置进行仿真，则启动命令示例如下：

```bash
(base) unitree@Host:~$ conda activate unitree_sim_env
(unitree_sim_env) unitree@Host:~$ cd ~/unitree_sim_isaaclab
(unitree_sim_env) unitree@Host:~/unitree_sim_isaaclab$ python sim_main.py --device cpu  --enable_cameras  --task  Isaac-PickPlace-Cylinder-G129-Dex3-Joint --enable_dex3_dds --robot_type g129
```

💥💥💥 请注意❗

> **仿真环境启动后，使用鼠标左键在窗口内点击一次以激活仿真运行状态。**
>
> 此时，终端内输出 `controller started, start main loop...`。

仿真界面如下图所示：

<p align="center">
  <a href="https://oss-global-cdn.unitree.com/static/bea51ef618d748368bf59c60f4969a65_1749x1090.png">
    <img src="https://oss-global-cdn.unitree.com/static/bea51ef618d748368bf59c60f4969a65_1749x1090.png" alt="Unitree sim isaaclab" style="width: 75%;">
  </a>
</p>



## 2.2 🚀 启动遥操

本程序支持通过 XR 设备（比如手势或手柄）来控制实际机器人动作，也支持在虚拟仿真中运行。你可以根据需要，通过命令行参数来配置运行方式。

以下是本程序的启动参数说明：

- 基础控制参数

|   ⚙️ 参数    |                      📜 说明                      |                       🔘 目前可选值                       | 📌 默认值 |
| :---------: | :----------------------------------------------: | :------------------------------------------------------: | :------: |
| `--xr-mode` |    选择 XR 输入模式（通过什么方式控制机器人）    | `hand`（**手势跟踪**）<br />`controller`（**手柄跟踪**） |  `hand`  |
|   `--arm`   |      选择机器人设备类型（可参考 0. 📖 介绍）      |        `G1_29`<br />`G1_23`<br />`H1_2`<br />`H1`        | `G1_29`  |
|   `--ee`    | 选择手臂的末端执行器设备类型（可参考 0. 📖 介绍） |    `dex1`<br />`dex3`<br />`inspire1`<br />`brainco`     | 无默认值 |

- 模式开关参数

|    ⚙️ 参数    |                            📜 说明                            |
| :----------: | :----------------------------------------------------------: |
|  `--record`  | 【启用**数据录制**模式】<br />按 **r** 键进入遥操后，按 **s** 键可开启数据录制，再次按 **s** 键可结束录制并保存本次 episode 数据。<br />继续按下 **s** 键可重复前述过程。 |
|  `--motion`  | 【启用**运动控制**模式】<br />开启本模式后，可在机器人运控程序运行下进行遥操作程序。<br />**手势跟踪**模式下，可使用 [R3遥控器](https://www.unitree.com/cn/R3) 控制机器人正常行走；**手柄跟踪**模式下，也可使用[手柄摇杆控制机器人行走](https://github.com/unitreerobotics/xr_teleoperate/blob/375cdc27605de377c698e2b89cad0e5885724ca6/teleop/teleop_hand_and_arm.py#L247-L257)。 |
| `--headless` | 【启用**无图形界面**模式】<br />适用于本程序部署在开发计算单元（PC2）等无显示器情况 |
|   `--sim`    | 【启用[**仿真模式**](https://github.com/unitreerobotics/unitree_sim_isaaclab)】 |

------

根据上述参数说明以及仿真环境配置，我们假设选择**手势跟踪**来控制 G1(29 DoF) + Dex3 灵巧手设备，同时开启仿真模式和数据录制模式。

则启动命令如下所示：

```bash
(tv) unitree@Host:~$ cd ~/xr_teleoperate/teleop/
(tv) unitree@Host:~/xr_teleoperate/teleop/$ python teleop_hand_and_arm.py --xr-mode=hand --arm=G1_29 --ee=dex3 --sim --record
# 实际上，由于一些参数存在默认值，该命令也可简化为：
(tv) unitree@Host:~/xr_teleoperate/teleop/$ python teleop_hand_and_arm.py --ee=dex3 --sim --record
```

程序正常启动后，终端输出信息如下图所示：

<p align="center">
  <a href="https://oss-global-cdn.unitree.com/static/735464d237214f6c9edf8c7db9847a0a_1874x1275.png">
    <img src="https://oss-global-cdn.unitree.com/static/735464d237214f6c9edf8c7db9847a0a_1874x1275.png" alt="start_terminal_log" style="width: 75%;">
  </a>
</p>

接下来，执行以下步骤：

1. 戴上您的 XR 头显设备（比如 apple vision pro 或 pico4 ultra enterprise等）

2. 连接对应的 WiFi 热点

3. 打开浏览器应用（比如 Safari 或 PICO Browser），输入并访问网址：https://192.168.123.2:8012?ws=wss://192.168.123.2:8012

   > 注意1：此 IP 地址应与您的 **主机** IP 地址匹配。该地址可以使用 `ifconfig` 等类似命令查询。

   > 注意2：此时可能弹出下图所示的警告信息。请点击`Advanced`按钮后，继续点击 `Proceed to ip (unsafe)` 按钮，使用非安全方式继续登录服务器。

   <p align="center">
     <a href="https://oss-global-cdn.unitree.com/static/cef18751ca6643b683bfbea35fed8e7c_1279x1002.png">
       <img src="https://oss-global-cdn.unitree.com/static/cef18751ca6643b683bfbea35fed8e7c_1279x1002.png" alt="vuer_unsafe" style="width: 50%;">
     </a>
   </p>

4. 进入`Vuer`网页界面后，点击 **`Virtual Reality`** 按钮。在允许后续的所有对话框后，启动 VR 会话。界面如下图所示：

   <p align="center">
     <a href="https://oss-global-cdn.unitree.com/static/fdeee4e5197f416290d8fa9ecc0b28e6_2480x1286.png">
       <img src="https://oss-global-cdn.unitree.com/static/fdeee4e5197f416290d8fa9ecc0b28e6_2480x1286.png" alt="vuer" style="width: 75%;">
     </a>
   </p>

5. 此时，您将会在 XR 头显设备中看到机器人的第一人称视野。同时，终端打印出链接建立的信息：

   ```bash
   websocket is connected. id:dbb8537d-a58c-4c57-b49d-cbb91bd25b90
   default socket worker is up, adding clientEvents 
   Uplink task running. id:dbb8537d-a58c-4c57-b49d-cbb91bd25b90
   ```

6. 然后，将手臂形状摆放到与**机器人初始姿态**相接近的姿势。这一步是为了避免在实物部署时，初始位姿差距过大导致机器人产生过大的摆动。

   机器人初始姿态示意图如下：

   <p align="center">
     <a href="https://oss-global-cdn.unitree.com/static/2522a83214744e7c8c425cc2679a84ec_670x867.png">
       <img src="https://oss-global-cdn.unitree.com/static/2522a83214744e7c8c425cc2679a84ec_670x867.png" alt="robot_init_pose" style="width: 25%;">
     </a>
   </p>

7. 最后，在终端中按下 **r** 键后，正式开启遥操作程序。此时，您可以远程控制机器人的手臂（和灵巧手）

8. 在遥操过程中，按 **s** 键可开启数据录制，再次按 **s** 键可结束录制并保存数据（该过程可重复）

   数据录制过程示意图如下：

   <p align="center">
     <a href="https://oss-global-cdn.unitree.com/static/f5b9b03df89e45ed8601b9a91adab37a_2397x1107.png">
       <img src="https://oss-global-cdn.unitree.com/static/f5b9b03df89e45ed8601b9a91adab37a_2397x1107.png" alt="record" style="width: 75%;">
     </a>
   </p>

> 注意1：录制的数据默认存储在 `xr_teleoperate/teleop/utils/data` 中。数据使用说明见此仓库： [unitree_IL_lerobot](https://github.com/unitreerobotics/unitree_IL_lerobot/blob/main/README_zh.md#%E6%95%B0%E6%8D%AE%E9%87%87%E9%9B%86%E4%B8%8E%E8%BD%AC%E6%8D%A2)。
>
> 注意2：请在录制数据时注意您的硬盘空间大小。

## 2.3 🔚 退出

要退出程序，可以在终端窗口（或 'record image' 窗口）中按下 **q** 键。



# 3. 🤖 实物部署

实物部署与仿真部署步骤基本相似，下面将重点指出不同之处。

## 3.1 🖼️ 图像服务

仿真环境中已经自动开启了图像服务。实物部署时，需要针对自身相机硬件类型，手动开启图像服务。步骤如下：

将 `xr_teleoperate/teleop/image_server` 目录中的 `image_server.py` 复制到宇树机器人（G1/H1/H1_2 等）的 **开发计算单元 PC2**。

```bash
# 提醒：可以通过scp命令将image_server.py传输到PC2，然后使用ssh远程登录PC2后执行它。
# 假设开发计算单元PC2的ip地址为192.168.123.164，那么传输过程示例如下：
# 先ssh登录PC2，创建图像服务器的文件夹
(tv) unitree@Host:~$ ssh unitree@192.168.123.164 "mkdir -p ~/image_server"
# 将本地的image_server.py拷贝至PC2的~/image_server目录下
(tv) unitree@Host:~$ scp ~/xr_teleoperate/teleop/image_server/image_server.py unitree@192.168.123.164:~/image_server/
```

并在 **PC2** 上执行以下命令：

```bash
# 提醒：目前该图像传输程序支持OpenCV和Realsense SDK两种读取图像的方式，请阅读image_server.py的ImageServer类的注释以便您根据自己的相机硬件来配置自己的图像传输服务。
# 现在位于宇树机器人 PC2 终端
unitree@PC2:~/image_server$ python image_server.py
# 您可以看到终端输出如下：
# {'fps': 30, 'head_camera_type': 'opencv', 'head_camera_image_shape': [480, 1280], 'head_camera_id_numbers': [0]}
# [Image Server] Head camera 0 resolution: 480.0 x 1280.0
# [Image Server] Image server has started, waiting for client connections...
```

在图像服务启动后，您可以在 **主机** 终端上使用 `image_client.py` 测试通信是否成功：

```bash
(tv) unitree@Host:~/xr_teleoperate/teleop/image_server$ python image_client.py
```

## 3.2 ✋ Inspire 手部服务（可选）

> 注意1：如果选择的机器人配置中没有使用 Inspire 系列灵巧手，那么请忽略本节内容。
>
> 注意2：如果选择的G1机器人配置，且使用 [Inspire DFX 灵巧手](https://support.unitree.com/home/zh/G1_developer/inspire_dfx_dexterous_hand)，相关issue [#46](https://github.com/unitreerobotics/xr_teleoperate/issues/46)。
>
> 注意3：如果选择的机器人配置中使用了 [Inspire FTP 灵巧手](https://support.unitree.com/home/zh/G1_developer/inspire_ftp_dexterity_hand)，相关issue [ #48](https://github.com/unitreerobotics/xr_teleoperate/issues/48)。

首先，使用 [此链接: DFX_inspire_service](https://github.com/unitreerobotics/DFX_inspire_service) 克隆灵巧手控制接口程序，然后将其复制到宇树机器人的**PC2**。

在宇树机器人的 **PC2** 上，执行命令：

```bash
unitree@PC2:~$ sudo apt install libboost-all-dev libspdlog-dev
# 构建项目
unitree@PC2:~$ cd DFX_inspire_service && mkdir build && cd build
unitree@PC2:~/DFX_inspire_service/build$ cmake ..
unitree@PC2:~/DFX_inspire_service/build$ make -j6

# （For unitree g1）终端 1. 
unitree@PC2:~/DFX_inspire_service/build$ sudo ./inspire_g1
# 或（For unitree h1）终端 1. 
unitree@PC2:~/DFX_inspire_service/build$ sudo ./inspire_h1 -s /dev/ttyUSB0

# 终端 2. 运行示例
unitree@PC2:~/DFX_inspire_service/build$ ./hand_example
```

如果两只手连续打开和关闭，则表示成功。一旦成功，即可关闭终端 2 中的 `./hand_example` 程序。

## 3.3 ✋BrainCo 手部服务（可选）

请参考[官方文档](https://support.unitree.com/home/zh/G1_developer/brainco_hand)。安装完毕后，请手动启动两个灵巧手的服务，命令示例如下（串口名称可能与实际有所差别）：

```bash
# Terminal 1.
sudo ./brainco_hand --id 126 --serial /dev/ttyUSB1
# Terminal 2.
sudo ./brainco_hand --id 127 --serial /dev/ttyUSB2
```



## 3.4 🚀 启动遥操

>  ![Warning](https://img.shields.io/badge/Warning-Important-red)
>
>  1. 所有人员必须与机器人保持安全距离，以防止任何潜在的危险！
>  2. 在运行此程序之前，请确保至少阅读一次 [官方文档](https://support.unitree.com/home/zh/Teleoperation)。
>  3. 没有开启**运动控制**模式（`--motion`）时，请务必确保机器人已经进入 [调试模式（L2+R2）](https://support.unitree.com/home/zh/G1_developer/remote_control)，以停止运动控制程序发送指令，这样可以避免潜在的指令冲突问题。
>  4. 如果要开启**运动控制**模式遥操作，请提前使用 [R3遥控器](https://www.unitree.com/cn/R3) 确保机器人进入主运控模式。
>  5. 开启**运动控制**模式（`--motion`）时：
>     - 右手柄按键 `A` 为遥操作**退出**功能按键；
>     - 左手柄和右手柄的两个摇杆按键同时按下为软急停按键，机器人会退出运控程序并进入阻尼模式，该功能只在必要情况下使用
>     - 左手柄摇杆控制机器人前后左右（最大控制速度已经在程序中进行了限制）
>     - 右手柄摇杆控制机器人转向（最大控制速度已经在程序中进行了限制）

与仿真部署基本一致，但要注意上述警告事项。

## 3.5 🔚 退出

>  ![Warning](https://img.shields.io/badge/Warning-Important-red)
>
>  为了避免损坏机器人，最好确保将机器人手臂摆放为与机器人初始姿态附近的恰当位置后，再按 **q** 退出。
>
>  调试模式下：按下退出键后，机器人双臂将在5秒内返回机器人初始姿态，然后结束控制。
>
>  运控模式下：按下退出键后，机器人双臂将在5秒内返回机器人运控姿态，然后结束控制。

与仿真部署基本一致，但要注意上述警告事项。



# 4. 🗺️ 代码库教程

```
xr_teleoperate/
│
├── assets                    [存储机器人 URDF 相关文件]
│
├── hardware                  [存储 3D 打印模组]
│
├── teleop
│   ├── image_server
│   │     ├── image_client.py      [用于从机器人图像服务器接收图像数据]
│   │     ├── image_server.py      [从摄像头捕获图像并通过网络发送（在机器人板载计算单元PC2上运行）]
│   │
│   ├── televuer
│   │      ├── src/televuer
│   │         ├── television.py       [使用 Vuer 从 XR 设备捕获头部、腕部和手部/手柄等数据]  
│   │         ├── tv_wrapper.py       [对捕获的数据进行后处理]
│   │      ├── test
│   │         ├── _test_television.py [television.py 的测试程序]  
│   │         ├── _test_tv_wrapper.py [tv_wrapper.py 的测试程序]  
│   │
│   ├── robot_control
│   │      ├── src/dex-retargeting [灵巧手映射算法库]
│   │      ├── robot_arm_ik.py     [手臂的逆运动学]  
│   │      ├── robot_arm.py        [控制双臂关节并锁定其他部分]
│   │      ├── hand_retargeting.py [灵巧手映射算法库 Wrapper]
│   │      ├── robot_hand_inspire.py  [控制因时灵巧手]
│   │      ├── robot_hand_unitree.py  [控制宇树灵巧手]
│   │
│   ├── utils
│   │      ├── episode_writer.py          [用于记录模仿学习的数据]  
│   │      ├── weighted_moving_filter.py  [用于过滤关节数据的滤波器]
│   │      ├── rerun_visualizer.py        [用于可视化录制数据]
│   │
│   │──teleop_hand_and_arm.py    [遥操作的启动执行代码]
```



# 5. 🛠️ 硬件

## 5.1 📋 清单

|           项目            | 数量 |                             链接                             |                             备注                             |
| :-----------------------: | :--: | :----------------------------------------------------------: | :----------------------------------------------------------: |
| **宇树通用人形机器人 G1** |  1   |                https://www.unitree.com/cn/g1                 |                    需选配开发计算单元版本                    |
|        **XR 设备**        |  1   | https://www.apple.com.cn/apple-vision-pro/<br />https://www.meta.com/quest/quest-3<br />https://www.picoxr.com/products/pico4-ultra-enterprise |                                                              |
|          路由器           |  1   |                                                              |                 常规模式必须，无线模式不需要                 |
|       **用户电脑**        |  1   |                                                              | 仿真模式下请使用[官方推荐](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/requirements.html)的硬件资源进行部署使用 |
|     **头部双目相机**      |  1   |  [仅供参考] http://e.tb.cn/h.TaZxgkpfWkNCakg?tk=KKz03Kyu04u  |                      用于机器人头部视野                      |
|     **头部相机支架**      |  1   | https://github.com/unitreerobotics/xr_teleoperate/blob/g1/hardware/head_stereo_camera_mount.STEP |                       用于装配头部相机                       |
| 英特尔 RealSense D405相机 |  2   |      https://www.intelrealsense.com/depth-camera-d405/       |                     用于腕部灵巧操作视野                     |
|     腕部相机环形支架      |  2   | https://github.com/unitreerobotics/xr_teleoperate/blob/g1/hardware/wrist_ring_mount.STEP |                    与腕部相机支架搭配使用                    |
|       左腕相机支架        |  1   | https://github.com/unitreerobotics/xr_teleoperate/blob/g1/hardware/left_wrist_D405_camera_mount.STEP |                     用于装配左腕D405相机                     |
|       右腕相机支架        |  1   | https://github.com/unitreerobotics/xr_teleoperate/blob/g1/hardware/right_wrist_D405_camera_mount.STEP |                     用于装配右腕D405相机                     |
|       M3-1 六角螺母       |  4   |              [仅供参考] https://a.co/d/gQaLtHD               |                        用于腕部紧固件                        |
|        M3x12 螺钉         |  4   |            [仅供参考] https://amzn.asia/d/aU9NHSf            |                        用于腕部紧固件                        |
|         M3x6 螺钉         |  4   |            [仅供参考] https://amzn.asia/d/0nEz5dJ            |                        用于腕部紧固件                        |
|      **M4x14 螺钉**       |  2   |            [仅供参考] https://amzn.asia/d/cfta55x            |                        用于头部紧固件                        |
|     **M2x4 自攻螺钉**     |  4   |            [仅供参考] https://amzn.asia/d/1msRa5B            |                        用于头部紧固件                        |

> 注意：加粗项目是进行遥操作任务时的必需设备，其余项目是录制[数据集](https://huggingface.co/unitreerobotics)时的可选设备。

## 5.2 🔨 安装示意图

<table>
    <tr>
        <th align="center">项目</th>
        <th align="center" colspan="2">仿真</th>
        <th align="center" colspan="2">实物</th>
    </tr>
    <tr>
        <td align="center">头部</td>
        <td align="center">
            <p align="center">
                <img src="./img/head_camera_mount.png" alt="head" width="100%">
                <figcaption>头部支架</figcaption>
            </p>
        </td>
        <td align="center">
            <p align="center">
                <img src="./img/head_camera_mount_install.png" alt="head" width="80%">
                <figcaption>装配侧视</figcaption>
            </p>
        </td>
        <td align="center" colspan="2">
            <p align="center">
                <img src="./img/real_head.jpg" alt="head" width="20%">
                <figcaption>装配正视</figcaption>
            </p>
        </td>
    </tr>
    <tr>
        <td align="center">腕部</td>
        <td align="center" colspan="2">
            <p align="center">
                <img src="./img/wrist_and_ring_mount.png" alt="wrist" width="100%">
                <figcaption>腕圈及相机支架</figcaption>
            </p>
        </td>
        <td align="center">
            <p align="center">
                <img src="./img/real_left_hand.jpg" alt="wrist" width="50%">
                <figcaption>装配左手</figcaption>
            </p>
        </td>
        <td align="center">
            <p align="center">
                <img src="./img/real_right_hand.jpg" alt="wrist" width="50%">
                <figcaption>装配右手</figcaption>
            </p>
        </td>
    </tr>
</table>


> 注意：如图中红圈所示，腕圈支架与机器人手腕接缝对齐。



# 6. 🙏 鸣谢

该代码基于以下开源代码库构建。请访问以下链接查看各自的许可证：

1. https://github.com/OpenTeleVision/TeleVision
2. https://github.com/dexsuite/dex-retargeting
3. https://github.com/vuer-ai/vuer
4. https://github.com/stack-of-tasks/pinocchio
5. https://github.com/casadi/casadi
6. https://github.com/meshcat-dev/meshcat-python
7. https://github.com/zeromq/pyzmq
8. https://github.com/Dingry/BunnyVisionPro
9. https://github.com/unitreerobotics/unitree_sdk2_python
