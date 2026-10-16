# DimOS Go2 分布式采图与巡检

本项目拆分为 **Go2/Orin 本机执行端** 和 **远程操作端**。本机负责实时性和安全相关工作；远程端负责操作界面、地图可视化、事件接收、告警呈现和归档。`simulation_legacy/` 是从原项目复制的仿真/本地控制台基线，未改其仿真工作流。

> **硬件状态**：项目仍未在实际 Go2 EDU/Orin 上验收。真机 WebRTC/AES、固件、LiDAR/odom、地图初始位姿、网络失联安全以及火情传感器都需要设备到货后逐项验证。软件暂停不是硬件急停。默认不要将端口裸露到公网。

## 职责划分

### Go2 / Orin Robot Agent

- 本机连接 Go2、接收 LiDAR/odom/相机数据；执行 DimOS 采图、PGO 地图导出、重定位、规划和运动控制。导航闭环在机器人侧，即使远程页面暂时断线也不应依赖远程电脑发高频控制量。
- 接受地图/巡检启动、暂停、确认安全、返航结束等低频任务命令。
- 将任务状态、地图点云预览、运行日志和火情状态提供给远程端。
- 火情状态 schema 为 `NaN`（无信号）、`Fire_alarm`、`Safe`。收到 `Fire_alarm` 时锁存暂停；必须收到/操作确认 `Safe` 后才继续。事件追加到设备本机 `robot_data/fire-alarm-events.jsonl`。
- 配置及访问入口见 [robot_agent/agent.py](robot_agent/agent.py) 与 [start_robot_agent.sh](start_robot_agent.sh)。

### Remote Operator Client

- 提供操作页面和地图 XY 俯视预览；地图与路线/任务分开管理。
- 通过 HTTP API 向机器人发送采图、巡检、暂停、恢复和结束请求；不在远程主机上启动 DimOS 或控制高频 `cmd_vel`。
- 接收并归档状态和火情事件到 `logs/robot-events.jsonl`、`logs/robot-status.jsonl`；地图、录制和 Rerun 回放文件同步到 `remote_client/data/maps/`。
- 页面入口是 [remote_client/console.py](remote_client/console.py)；默认仅监听回环地址，可通过 SSH 隧道访问。地图编辑和原先操作控件沿用经仿真验证的页面。

## 仿真保持原样

原来的仿真/单机流程被保留在 [simulation_legacy](simulation_legacy)。需要新建隔离虚拟环境（不复制原目录 13 GB `.venv`）：

```bash
cd /home/cy/Dimos/dimos_Go2/simulation_legacy
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install 'dimos[base,unitree,sim,mapping]' Flask numpy
./start_patrol_console.sh
```

然后浏览器打开 `http://127.0.0.1:7781/`，保持 UI 选择“仿真（MuJoCo）”。使用仿真采图/巡检按钮，或按 `simulation_legacy/README.md` 的步骤操作。仓库预置的 `maps/warehouse.pc2.lcm` 和 `warehouse.rrd` 只作示例；新建地图时使用独立名字，不覆盖样例。原有安装环境里有本项目定制过的 DimOS relocalization 模块；重建环境时请按机器人的 DimOS fork/固定 wheel 部署，避免退回缺少 `transform_file` 的上游版本。

## 真机部署前准备

真机由 **Orin 本机 agent** 独占控制；远程机只调用代理 API。两台机器先加入可信的同一私网/VPN，配置稳定地址。HTTP API 使用共享 bearer token；生产网络请加 HTTPS（反向代理或 VPN），不允许将未加密服务直接暴露公网。共享 token 只通过环境变量/服务管理器注入，绝不可提交到 Git。

### 1. Orin 安装与启动（设备到货后执行）

在 Orin 上安装本项目，确认 `ROBOT_IP` 能访问 Go2，并构建与 Orin/Python/固件匹配的 DimOS 虚拟环境：

```bash
cd /opt/dimos_Go2/simulation_legacy
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install 'dimos[base,unitree,mapping]' Flask numpy
```

使用 systemd 或受控 shell 设置 `ROBOT_AGENT_TOKEN`、`ROBOT_IP`，必要时设置 `UNITREE_AES_128_KEY`；不要把凭证写入 `.env` 提交或 shell history。启动：

```bash
cd /opt/dimos_Go2
export ROBOT_AGENT_TOKEN='由运维安全注入的长随机值'
export ROBOT_IP='Go2 私网 IP'
export UNITREE_AES_128_KEY='仅当固件要求时，由安全注入'
./start_robot_agent.sh
```

Agent 默认监听 `0.0.0.0:8787` 以接受私网客户端请求，所有 `/api/v1/*` API 都要求 `Authorization: Bearer ...`。`/health` 仅用于存活探测。用防火墙/VPN 限制来源；生产部署应使用 TLS 终止代理并配置 systemd 自动重启、日志轮转、数据保留/磁盘限额。

### 2. 远程客户端安装与启动

在运维电脑部署同一个项目，安装独立 UI 环境：

```bash
cd /opt/dimos_Go2/remote_client
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
```

设置与 Orin 相同的 `ROBOT_AGENT_TOKEN`、`ROBOT_AGENT_URL`（例如 `https://orin-go2.private:8787`）。`ROBOT_COMMAND_CENTER_URL` 指向受 VPN/SSH 隧道保护的 Command Center；远程 UI 默认监听 localhost：

```bash
cd /opt/dimos_Go2
export ROBOT_AGENT_URL='https://orin-go2.private:8787'
export ROBOT_AGENT_TOKEN='同一 bearer token，由安全注入'
export ROBOT_COMMAND_CENTER_URL='https://orin-go2.private:7779/'
export ROBOT_RERUN_URL='https://orin-go2.private:9878/'
./start_remote_console.sh
```

在浏览器打开 `http://127.0.0.1:7781/`。若通过 SSH 隧道连接操作电脑，隧道应同时只转发所需 UI 和可视化端口；不要将命令中心或 Rerun 直接暴露公网。远程后台每 5 秒拉取事件/地图并归档，页面轮询当前任务状态。

## 操作流程

1. **建图**：远程 UI 选“真机”，输入地图名后启动采图；从安全 Command Center 遥控 Go2 覆盖区域；停止后 Orin 端自动 PGO 并导出。远端 UI/worker 接收地图点云，3D Rerun 文件归档到远端。
2. **地图审核**：在地图工作区选择机器人已有地图，查看 XY 点云、覆盖范围、墙体连续性和重影；重要地图还需在 Rerun/外部可视化查看高度层次。XY 图不自动判定地图合格。
3. **路线编辑**：在地图上设置唯一出发/最终返航点、巡检点及停留时间；可优化近似 XY 直线顺序。检查路线与可通行空间后保存；替换地图后重新核对坐标。
4. **启动**：选立即开始或预约一次；选只运行一轮或持续循环至时长结束。持续循环的间隔 `0` 表示马上开始下一轮；循环轮次之间不回起点。任务时长到期或人工结束时，Orin 从当前位置返航。
5. **火情**：当前未接相机/火情检测器时显示 `NaN`。未来检测端在 Orin 本地调用 `POST /api/v1/fire-signal`，提交 `{"state":"Fire_alarm","source":"thermal-camera"}` 或 `{"state":"Safe","source":"operator-confirmation"}`。Fire_alarm 会锁存暂停且不发新运动目标；Safe 之后才恢复。事件会传到远端并存档。硬件急停必须独立有效。
6. **断网**：导航和避障留在 Orin；网络断开期间不能远程发命令/看实时状态，但 robot agent/DimOS 的本地任务继续其既定安全策略。恢复后远端按事件游标补拉日志。到货后必须实测断网策略是否符合现场安全要求。

## API（设备侧）

- `GET /health`：存活探测。
- `GET /api/v1/status`、`GET /api/v1/events?after=<cursor>`、`GET /api/v1/logs`：状态、事件与日志。
- `GET /api/v1/maps`、`GET /api/v1/maps/<map>.pc2.lcm/points`、`GET /api/v1/maps/<name>/artifact/{map,recording,rerun}`：地图查看/归档。
- `POST /api/v1/mapping/{start,stop}`、`POST /api/v1/patrol/start`、`POST /api/v1/patrol/{pause,resume,stop}`。
- `POST /api/v1/fire-signal`：只接受 `Fire_alarm` 或 `Safe`；`source`/`detail` 记录信号来源和说明。

远端客户端是唯一建议的网络入口。尚未在实际硬件上验证 API 可用性、TLS 配置、token 生命周期和火情 detector adapter；这些是到货验收项，不应被视为已通过。

## 安全、数据和已知限制

- Go2 的站立、行走、制动与失联行为都未完成硬件验证。第一轮只在围挡空旷区域、低速、人工急停随手可及的情况下测试；禁止直接无人值守运行。
- 远端巡检命令仅为低频任务调度；运动闭环、本地定位与规划必须保持在 Orin。不要从远端持续发送速度流。
- `simulation_legacy/` 是冻结的仿真基线；分布式真机功能只在 `robot_agent/` 与 `remote_client/` 开发。
- 机器人端录制/日志保存在 `simulation_legacy/maps`、`simulation_legacy/logs`、`robot_data/`；远程副本在 `remote_client/data/maps` 和 `logs/`。原始 DB 可能很大，实际部署前须配置磁盘预算、轮转/清理及加密备份。
- 当前 Flask server 是开发服务器；在真实设备部署前应使用生产 WSGI server、TLS 反向代理、认证密钥轮换与审计策略。
