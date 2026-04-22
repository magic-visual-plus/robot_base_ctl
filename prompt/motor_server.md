# 双进程电机服务框架规范 — 工程设计说明

> 文档状态：Draft for Review  
> 适用阶段：P0 / Phase-1 ~ Phase-5  
> 文档定位：工程设计、实现约束、联调依据、阶段验收、自动代码生成约束输入  
> 关键词：Motor Service、RT Core、Non-RT Control、EtherCAT、CiA402、CSP、Shared Memory、Ruckig、SDO、PDO

---

## 1. 文档目的

本文档定义一个**双进程电机服务（Motor Service）**的工程规范，用于指导设计、实现、评审、联调、验收与后续扩展。

本文档描述的不是临时 demo、单一驱动调试脚本或上层控制器插件，而是一个**可工程化、可评审、可分期实现、可标准化复用**的电机服务原型。该服务面向单电机闭环起步，后续可作为整机控制系统的底层基础能力。

本文档同时作为：

- 架构评审依据
- 开发实现边界说明
- RT / Non-RT 职责分工依据
- 分阶段交付与验收依据
- 自动代码生成约束输入
- 后续多轴、多总线、整机扩展的基线规范

---

## 2. 设计范围与定位

### 2.1 项目定位

本项目交付的是一个**双进程电机服务原型**，不是：

- 单一 EtherCAT demo
- 单一伺服驱动调试脚本
- 直接绑定 ROS 2 的控制器
- 临时性的整机控制实验代码

本项目的定位是：

> 在 **RT Motor Core Service + Non-RT Motor Control Service** 双进程模型下，构建一个以单电机控制闭环为核心、可扩展为整机底层控制能力的标准服务框架。

### 2.2 P0 最小闭环目标

P0 仅聚焦最小可验证闭环：

- 单个**腰部电机**
- **EtherCAT** 通讯
- 驱动采用 **CiA402**
- 控制模式采用 **CSP**
- RT 周期为 **1 ms**
- Non-RT 接收 **teleop** 目标
- Non-RT 使用 **Ruckig** 在线生成与重规划轨迹
- RT 逐拍消费稠密点执行
- RT 将真实状态回传给 Non-RT
- Non-RT 必须基于**真实电机状态**重规划，而不是基于理想目标闭环

### 2.3 长期扩展方向（P0 不实现）

本服务后续应支持扩展，但下列能力不属于 P0：

- 多总线：EtherCAT / CANopen / USB2CAN
- 多类型伺服驱动接入
- 多轴并行调度
- 头、双臂、夹爪、腰、底盘自由组合
- 仿真 / HIL / Replay
- ROS 2 适配
- 整机多模块协同控制
- 复杂运维系统 / 配置中心 / 黑盒记录平台

### 2.4 核心架构约束

以下约束必须在 P0 即明确建立：

- RT 核心服务**不依赖 ROS 2**
- RT 核心服务只负责**时序确定性强**的工作
- 所有非实时逻辑必须放在 Non-RT 侧
- 所有命令注入与状态读取必须通过显式边界层完成
- 所有关键异常策略必须规格化，不允许依赖实现者自行解释

---

## 3. 术语定义

### 3.1 RT Motor Core Service

指运行在硬实时调度环境中的核心服务进程，负责：

- 1 ms 固定周期执行
- 总线读写
- PDO 读写
- CiA402 状态推进
- 轨迹点消费
- 安全执行
- 状态快照输出
- RT 事件输出
- 参数写入期间的受控状态切换与执行约束

### 3.2 Non-RT Motor Control Service

指运行在普通用户态或非硬实时上下文中的控制服务进程，负责：

- teleop 输入接入
- Ruckig 在线规划
- 基于实测状态重规划
- 状态聚合
- 日志 / 诊断 / 工具
- 配置与模式管理
- 参数写入请求封装
- 上层适配（未来 ROS 2 / RPC / GUI 等）

### 3.3 Boundary World

指 RT 与 Non-RT 之间的共享边界层，要求：

- 固定大小
- POD
- 无阻塞
- 共享内存友好
- cache-line aware
- 无动态分配
- 无虚函数

### 3.4 稠密点（Dense Point）

指由 Non-RT 轨迹规划器（P0 为 Ruckig）生成、由 RT 每拍消费的轨迹采样点。典型字段包括：

- target position
- target velocity
- target acceleration
- timestamp
- sequence

### 3.5 Hold

指 RT 在无法继续安全推进正常轨迹时采用的保守控制动作。Hold 不等于继续沿理想轨迹外推，而是：

- 保持最后安全命令
- 或进入明确定义的不推进状态

该行为必须可定义、可观测、可验收。

### 3.6 断粮（Starvation）

指 RT 消费端持续无法获得新稠密点的状态。根据持续时长可区分为：

- 短时断粮
- 长时间断粮

### 3.7 欠载（Underrun Risk）

指队列仍有少量点，但深度低于低水位阈值，存在即将断粮风险的状态。

---

## 4. 总体设计原则

### 4.1 双进程模型

系统采用双进程模型：

| 进程 | 名称 | 职责 |
|------|------|------|
| A | **RT Motor Core Service** | 硬实时控制、总线、PDO、CiA402、稠密点消费、安全、快照、事件 |
| B | **Non-RT Motor Control Service** | teleop、Ruckig、状态聚合、配置、诊断、参数写入请求、未来 adapter |

### 4.2 三域模型

| 域 | 位置 | 内容 |
|----|------|------|
| **RT World** | RT 进程内 | bus domain、axis domain、RT execution domain |
| **Boundary World** | RT ↔ Non-RT 之间 | 共享内存结构、命令与状态通路 |
| **Non-RT World** | Non-RT 进程内 | teleop、planning、aggregation、diagnostics、tools |

### 4.3 命令入口与状态出口对称原则

任何闭环都必须同时具备：

- **Non-RT → RT** 命令路径
- **RT → Non-RT** 真实状态路径

禁止以下不完整设计：

- 只有命令入口，没有真实状态回流
- 只有状态读取，没有可用的受控命令入口
- 用临时调试接口绕过边界层

### 4.4 服务职责单一原则

- RT 侧负责时序确定性和执行正确性
- Non-RT 侧负责功能复杂度与业务灵活性
- 边界层负责最小、明确、低开销的数据交换
- 任何功能不得同时散落在 RT 和 Non-RT 两侧而无清晰边界

### 4.5 明确默认策略原则

所有联调期容易产生分歧的行为必须显式规格化，包括但不限于：

- 队列空 / 满 / 欠载 / 超时行为
- 参数写入前后的状态切换行为
- 安全降级动作
- 事件发布条件
- 快照暴露字段
- 验收标准

---

## 5. P0 非目标

P0 明确不包含以下内容：

- ROS 2 / `ros2_control`
- MoveIt / Nav2
- whole-body 多轴协同
- 多总线统一调度
- 复杂配置中心
- 完整仿真平台
- 复杂恢复编排
- 多 client ownership
- 复杂权限系统
- 完整在线参数管理平台
- 大规模事件追踪系统
- 黑盒回放平台

P0 的唯一核心目标是：

> 在双进程服务边界下，完成单腰轴 EtherCAT + CiA402 CSP + 1 ms RT + Non-RT 重规划的最小电机服务闭环。

---

## 6. 总体模块划分

### 6.1 RT Motor Core Service 内部模块

#### 6.1.1 RT 调度与内核模块

- `RtKernel`
- `RtScheduler`
- `RtControlPipeline`
- `CycleMonitor`
- `Watchdog`
- `RtSafetyExecutor`

#### 6.1.2 Bus Domain

- `IBusTransport`
- `EthercatMaster`
- `PdoRegistry`
- `DcSyncManager`
- `SdoAccessManager`（阶段 5 可选）

#### 6.1.3 Axis Domain

- `AxisCommand`
- `AxisState`
- `IAxisDriver`
- `WaistAxisDriver`
- `CiA402StateMachine`
- `Axis`

> P0 可不强制先落 `AxisManager`，但代码组织应预留扩展点。

#### 6.1.4 RT 侧 Boundary 生产 / 消费组件

- `DensePointQueue`：消费者
- `SnapshotBuffer`：生产者
- `EventRing`：生产者
- `ParameterWriteRequestMailbox`：消费者（阶段 5 可选）
- `ParameterWriteResultBuffer`：生产者（阶段 5 可选）

### 6.2 Non-RT Motor Control Service 内部模块

#### 6.2.1 输入与轨迹模块

- `TeleopInputAdapter`
- `TeleopTargetMailbox`
- `RuckigTrajectoryWorker`

#### 6.2.2 状态读取与聚合模块

- `MotorStateReader`
- `SnapshotBuffer` 读取端
- `StateAggregator`（P0 可极简）

#### 6.2.3 Runtime / IPC 模块

- `BoundaryIpcManager`
- 共享内存初始化与 attach
- 命令写入接口
- 状态读取接口
- 参数写入请求封装（阶段 5 可选）

#### 6.2.4 工具与诊断模块

- recorder（后续）
- replay（后续）
- CLI / RPC（后续）
- 诊断与日志（P0 可保留最小骨架）

---

## 7. Boundary World 共享边界规范

边界层中的所有共享结构必须满足以下要求：

- 固定大小
- POD
- `alignas(64)`
- 共享内存可直接映射
- 无虚函数
- 无动态内存
- 无 STL 动态容器
- 字段定义明确、可扩展但不模糊
- 时间戳与 sequence 必须显式存在于关键结构中

P0 必须包含：

- `TeleopTargetMailbox`
- `DensePointQueue`
- `SnapshotBuffer`
- `MotorStateReader`
- `EventRing`

P0 可选预留：

- `ParameterWriteRequestMailbox`
- `ParameterWriteResultBuffer`

---

## 8. 共享内存与 IPC 规范

### 8.1 IPC 总原则

主控制通路禁止采用以下方式作为核心闭环通道：

- 文本协议
- JSON / YAML 控制消息
- 高开销 RPC
- 非确定性消息总线
- 临时 socket 文本命令

P0 推荐使用：

- POSIX shared memory
- 或 mmap-backed shared memory

### 8.2 IPC 设计要求

- 所有共享结构必须预分配
- 所有共享结构必须为固定大小
- 关键热路径必须 O(1)
- RT 读取或写入不得被 Non-RT 阻塞
- 所有关键数据结构必须具备 sequence
- 所有需要时效性的结构必须具备 timestamp
- head / tail 必须分离 cache line
- 不允许在共享结构中塞入复杂指针图或堆对象

### 8.3 共享内存生命周期要求

共享内存生命周期必须明确：

- 创建方
- attach 方
- 初始化时机
- 版本校验
- magic number 校验
- 清零策略
- stale segment 处理策略

建议在共享内存布局中包含：

- `magic`
- `version`
- `layout_size`
- `build_id`（可选）
- `init_state`

---

## 9. 边界数据结构规范

### 9.1 TeleopTargetMailbox

#### 9.1.1 用途

- Non-RT 写入最新遥操作目标
- 供 `RuckigTrajectoryWorker` 读取

#### 9.1.2 语义

- latest-value
- 新值覆盖旧值
- 不保证保留历史

#### 9.1.3 模型

- 单生产者 / 单消费者语义

#### 9.1.4 推荐字段

- target position
- mode bits
- enable bits
- timestamp
- sequence

### 9.2 DensePointQueue

#### 9.2.1 用途

- Non-RT 写入稠密轨迹点
- RT 每拍消费一个或多个点，用于生成本拍命令

#### 9.2.2 基本要求

- SPSC ring
- 固定容量
- 无阻塞
- 无动态分配
- cache-line aware
- head / tail 分离
- 支持空 / 满 / 欠载判定
- 必须支持 timestamp / sequence 检查

#### 9.2.3 稠密点字段建议

- target position
- target velocity
- target acceleration
- sequence
- timestamp

#### 9.2.4 DensePointQueue 默认策略（强制默认值）

为避免首版联调、自动代码生成和多实现者协作阶段对队列行为理解不一致，P0 必须采用统一默认策略。

##### 默认参数

| 项目 | 默认值 | 说明 |
|------|--------|------|
| 队列模型 | SPSC ring | 单生产者 / 单消费者 |
| 队列容量 | 128 点 | 固定容量，预分配 |
| RT 每拍消费 | 1 点 | 默认每个 1 ms 周期消费 1 点 |
| 欠载阈值 | `< 8` 点 | 低于该值视为 underrun risk |
| 短时断粮阈值 | `10 ms` | 超过该时间未获得新点视为短时断粮 |
| 长时间断粮阈值 | `50 ms` | 超过该时间视为严重断粮 |
| 陈旧点判定 | timestamp 或 sequence 异常 | 默认拒绝消费 |

> 以上数值为 P0 统一默认值。后续允许参数化，但在未参数化前不得各实现自行修改。

##### 默认行为策略表

| 场景 | 判定条件 | RT 默认动作 | 事件等级 | 是否进入快照 |
|------|----------|-------------|----------|--------------|
| 正常 | 队列深度 ≥ 8 | 每拍消费 1 点 | 无 | 是 |
| 欠载 | 0 < 队列深度 < 8 | 继续消费；置 `underrun_risk` | warning | 是 |
| 空队列 | 当前拍无可消费点 | 不推进新轨迹；进入 `hold-last-safe-command` | warning | 是 |
| 短时断粮 | 连续超过 10 ms 无新点 | 保持 hold，不恢复推进 | error | 是 |
| 长时间断粮 | 连续超过 50 ms 无新点 | 执行 `quick stop` 或 `disable operation` | critical | 是 |
| 满队列 | 写入时队列已满 | 默认拒绝新点写入，不覆盖旧点 | warning | 是 |
| 陈旧点 | timestamp 过旧或 sequence 回退 | 丢弃该点，不执行 | warning | 是 |
| 非法点 | 数值越界 / NaN / Inf / 明显异常 | 丢弃该点，并转安全处理路径 | error | 是 |

##### 默认策略补充约束

- P0 默认采用**保守策略**
- 满队列时**默认拒绝新点写入**
- 不允许默认覆盖旧点，除非后续版本通过显式策略配置开启
- 空队列时 RT **不得自行外推新轨迹**
- `hold-last-safe-command` 必须明确定义，且必须可观测
- 欠载、断粮、满队列、陈旧点、非法点都必须写入 `EventRing`
- 欠载与断粮状态必须写入 `SnapshotBuffer`

### 9.3 SnapshotBuffer

#### 9.3.1 用途

- RT 写入最新状态
- Non-RT 读取最新真实状态，用于重规划、显示、诊断与验收

#### 9.3.2 要求

- RT 写入 O(1)
- Non-RT 读取不得阻塞 RT
- 推荐采用：
  - double buffer
  - triple buffer
  - seqlock

#### 9.3.3 P0 最低字段要求

- 实际位置
- 实际速度
- statusword
- control 状态摘要
- 当前模式 / 使能状态
- fault 标志
- bus 健康状态
- cycle 统计
- 当前队列深度
- 最近一次成功写点时间
- 是否处于 underrun
- 是否处于 hold
- 最近一次断粮开始时间（若发生）
- 最近一次参数写入状态摘要（若实现在线改参）

### 9.4 EventRing

#### 9.4.1 用途

- RT → Non-RT 的轻量事件通道

#### 9.4.2 要求

- RT 只写固定大小事件
- RT 不写字符串
- RT 不拼接日志文本
- Non-RT 负责解释事件

#### 9.4.3 典型事件

- axis fault
- bus offline
- mode change
- deadline miss
- watchdog timeout
- estop active
- dense point underrun
- dense point starvation
- queue full
- stale point dropped
- illegal point dropped
- parameter write started
- parameter write completed
- parameter write rejected
- parameter write failed

#### 9.4.4 推荐字段

- event id
- source id
- severity
- timestamp
- sequence
- numeric payload[2~4]

### 9.5 ParameterWriteRequestMailbox（阶段 5 建议预留）

#### 9.5.1 用途

- Non-RT 发起参数写入请求
- RT 负责按设备状态约束执行 SDO 写入及相关状态切换

#### 9.5.2 推荐字段

- axis id / slave id
- object index
- subindex
- value
- value size
- parameter category
- requires_disable_op
- requires_preop
- requires_save
- requires_reset_node
- sequence
- timestamp

#### 9.5.3 原则

- Non-RT 不得直接控制实时总线运行态
- 参数写入必须通过 RT 受控路径执行
- 参数写入必须具备显式结果回传

---

## 10. RT 线程硬约束

### 10.1 RT 热路径禁止事项

RT 热路径中禁止：

- `new/delete`
- `malloc/free`
- `std::vector` 扩容
- `std::string` 拼接
- 普通阻塞 `mutex`
- 条件变量阻塞等待
- 文件写入
- 网络发送
- ROS publisher / subscriber
- 重日志打印
- 在 RT 中运行 Ruckig
- 在 RT 中做复杂轨迹规划
- 在 RT 中做复杂恢复编排
- 在 RT 中进行不可预测耗时计算
- 在运动中无约束直接写关键 SDO 参数

### 10.2 RT 热路径允许事项

RT 热路径允许：

- 固定大小 POD
- 预分配 ring buffer
- lock-free / best-effort queue
- 简单状态机推进
- EtherCAT 周期读写
- 轻量事件写入
- 轻量 trace / cycle 统计
- 限幅 / 限速 / hold / quick stop / disable
- 受控状态机下执行参数写入前后的状态切换

### 10.3 RT 函数设计要求

- 热路径函数尽量 `noexcept`
- 避免隐式堆分配
- 避免异常抛出
- 避免隐藏阻塞点
- 明确输入输出，不依赖隐式全局状态
- 每拍执行阶段必须明确可追踪

---

## 11. Non-RT 服务职责规范

### 11.1 Non-RT 可以承担的职责

- teleop 输入接入
- Ruckig 在线规划与重规划
- 状态聚合
- recorder / replay（后续）
- CLI / RPC / GUI / ROS 2 adapter（后续）
- 文件 IO
- 详细日志
- 故障解释
- 模式控制
- 参数写入请求构造
- 工具与诊断逻辑

### 11.2 Non-RT 禁止事项

- 直接驱动 EtherCAT 周期读写
- 直接访问 RT 内部 bus 对象
- 直接操作 RT 内部 Axis 对象
- 直接推进 CiA402 状态机
- 直接决定 RT 调度周期
- 绕过 RT 边界直接执行关键 SDO 改参流程

### 11.3 Non-RT 与 RT 的交互原则

- 所有命令必须通过 Boundary World 注入
- 所有状态必须通过 Boundary World 读取
- 所有关键行为必须能通过快照与事件观察
- Non-RT 不得依赖偷看 RT 内部状态实现业务逻辑

---

## 12. P0 控制闭环规范

### 12.1 输入链（Non-RT）

1. teleop 更新目标位置
2. `TeleopTargetMailbox` 写入最新目标
3. `RuckigTrajectoryWorker` 读取目标
4. `MotorStateReader` 从 `SnapshotBuffer` 读取真实状态
5. Ruckig 基于真实状态重规划
6. 将稠密点写入 `DensePointQueue`

### 12.2 RT 执行链

1. `RtScheduler` 以 1 ms 周期唤醒
2. `RtControlPipeline` 执行一拍
3. bus read
4. axis feedback update
5. CiA402 state update
6. safety checks
7. consume dense point
8. generate final axis command
9. axis write
10. bus write
11. snapshot export
12. event export
13. cycle stats update

### 12.3 状态回流链

- RT 将最新真实状态写入 `SnapshotBuffer`
- Non-RT 持续读取最新状态
- Non-RT 必须基于真实状态而不是理想目标重规划

### 12.4 队列异常时的 RT 行为（强制规范）

RT 在消费 `DensePointQueue` 时，必须执行以下默认规则：

1. 队列正常时，每拍消费 1 个点并生成控制命令
2. 队列欠载时，可继续运行，但必须置 `underrun_risk` 并发 warning 事件
3. 当前拍队列为空时，不得继续沿理想轨迹外推，必须进入 `hold-last-safe-command`
4. 空队列持续超过短时断粮阈值时，必须保持 hold，并发 error 事件
5. 断粮持续超过长时间断粮阈值时，必须执行 `quick stop` 或 `disable operation`
6. 若检测到陈旧点、乱序点、非法点，则必须丢弃并发事件，不得直接执行

---

## 13. CiA402 状态机规范

### 13.1 设计要求

- `CiA402StateMachine` 必须独立成类
- 禁止将状态推进逻辑散落在主循环里
- 禁止用零散魔法数字替代显式状态机

### 13.2 至少支持的状态

- Not ready to switch on
- Switch on disabled
- Ready to switch on
- Switched on
- Operation enabled
- Fault
- Quick stop active

### 13.3 输入

- `statusword`
- enable request
- disable request
- quick stop request
- fault reset request
- expected target state

### 13.4 输出

- `controlword`
- 当前内部状态枚举
- 是否允许进入 operation enabled
- 是否需要 quick stop
- 是否需要 disable
- 是否需要 fault reset

### 13.5 工程要求

- 所有状态跳转必须可被单元测试覆盖
- 状态机输出不得依赖未定义 side effect
- 状态机与 AxisDriver 逻辑必须分层
- 状态机逻辑不得与总线读写糊在一起

---

## 14. 运行中参数写入规范

本章定义电机服务在运行过程中安全改参的标准行为。其性质属于**RT Core Service 的设备运行约束**，不是上层配置逻辑替代物。

### 14.1 总原则

- 通常**不需要专门停止 PDO 通信**
- 但很多 SDO 参数写入前，必须或建议先让伺服退出 `Operation Enabled`
- 对于模式参数、PDO 映射参数、关键控制参数，应采用更严格的写入流程
- 参数写入必须是**受控状态流**
- 参数写入必须可观测、可回传、可拒绝、可失败

### 14.2 最保险默认流程

默认建议流程如下：

1. 先停止运动
2. 将驱动切到 `Disable Operation`
3. 再通过 SDO 写参数
4. 若是 PDO 映射或关键参数，切到 `Pre-Operational`
5. 写完后按厂家要求执行：
   - 保存参数
   - Reset Node
   - 恢复到 Operational
   - 重新使能驱动

### 14.3 一句话工程结论

> 通常不用专门停 PDO 通信，但很多 SDO 参数写入前建议先让伺服退出 Operation Enabled，尤其是模式、PDO 映射和关键控制参数。

### 14.4 参数分类建议

建议最少区分以下参数类别：

| 类别 | 示例 | 默认要求 |
|------|------|----------|
| 普通非关键参数 | 非运行态关键对象 | 可由受控流程直接写入 |
| 模式相关参数 | operation mode 等 | 必须退出 Operation Enabled |
| 关键控制参数 | 限位、控制关键参数 | 必须退出 Operation Enabled，必要时 Hold |
| PDO 映射参数 | PDO assignment / mapping | 必须切到 Pre-Operational |
| 需保存参数 | 厂家要求 save | 写后必须 save |
| 需 reset 参数 | 厂家要求 reset node | 写后必须 reset node |

### 14.5 参数写入设计落地要求

- Non-RT 只能提交参数写入请求
- RT 负责决定：
  - 是否必须停运动
  - 是否必须 Disable Operation
  - 是否必须 Pre-Operational
  - 是否必须 Save
  - 是否必须 Reset Node
  - 是否必须重新使能
- 参数写入过程必须进入 `SnapshotBuffer`
- 参数写入结果必须进入 `EventRing`
- 关键参数写入期间，RT 必须暂停正常运动点消费或切入受控 hold

### 14.6 参数写入结果规范

参数写入结果至少应包含：

- request sequence
- axis id
- 执行阶段
- success / failed / rejected
- reason code
- 是否已 save
- 是否已 reset
- 是否已 re-enable

---

## 15. 安全与降级策略规范

### 15.1 安全目标

当输入异常、总线异常、状态异常或参数写入导致无法继续安全执行时，RT 必须进入受控降级状态，而不是继续推进不可信控制命令。

### 15.2 最低支持的降级动作

- hold
- quick stop
- disable operation

### 15.3 默认优先级建议

| 场景 | 推荐动作 |
|------|----------|
| 队列欠载 | warning + continue |
| 当前拍空队列 | hold |
| 短时断粮 | hold |
| 长时间断粮 | quick stop 或 disable |
| 关键参数写入 | hold / disable operation |
| drive fault | quick stop / disable |
| bus offline | disable / fault handling |

### 15.4 安全动作要求

- 所有安全动作必须可观测
- 所有安全动作必须可回放到事件流
- 所有安全动作必须有明确触发条件
- 不允许隐式自动恢复而无记录

---

## 16. RT 流水线规范

RT 控制流程必须显式拆分，不允许糊成一个大 `while`。

推荐固定阶段如下：

1. scheduler tick
2. bus read
3. axis feedback update
4. CiA402 state update
5. safety checks
6. parameter flow step（若启用）
7. consume dense point
8. generate final axis command
9. axis write
10. bus write
11. snapshot export
12. event export
13. cycle stats update

### 16.1 参数流程插入要求

如果支持在线改参，参数流程不得以临时代码方式插入任意位置，而必须：

- 明确为独立阶段
- 或由专门 manager 串行调度
- 不得破坏主循环时序结构

### 16.2 分阶段设计目的

- 提升可维护性
- 提升故障定位能力
- 提升后续扩展能力
- 降低自动代码生成时的逻辑混淆
- 明确哪些逻辑属于热路径，哪些属于控制状态流

---

## 17. 代码组织规范

### 17.1 语言与风格

- 语言：C++17
- 头文件 / 源文件分离
- 命名清晰
- namespace 明确
- 避免单文件巨型 demo

### 17.2 推荐目录结构

```text
motor_service/
├── core/
│   ├── bus/          # i_bus_transport, ethercat_master, pdo_registry, dc_sync_manager, sdo_access_manager
│   ├── axis/         # axis_command, axis_state, i_axis_driver, cia402_state_machine, waist_axis_driver
│   ├── rt/           # rt_kernel, rt_scheduler, rt_control_pipeline, cycle_monitor, watchdog, rt_safety_executor
│   ├── boundary/     # teleop_target_mailbox, dense_point_queue, snapshot_buffer, motor_state_reader,
│   │                 # event_ring, parameter_write_request_mailbox, parameter_write_result_buffer,
│   │                 # shared_memory_layout, boundary_ipc_manager
│   ├── planning/     # teleop_input_adapter, ruckig_trajectory_worker
│   └── config/       # bus_config, axis_config, runtime_config
├── runtime/
│   ├── rt_main.cpp
│   └── nonrt_main.cpp
└── CMakeLists.txt
```

### 17.3 共享结构规范

* 优先 POD
* 固定大小字段
* `alignas(64)`
* 共享结构禁止虚函数
* 尽量避免热路径 `bool` 离散布局，优先 bitmask
* 所有 versioned shared layout 必须带版本字段

### 17.4 类职责规范

* 类职责单一
* 状态机、总线、业务、IPC 分离
* 不允许一个类同时负责：

  * 总线驱动
  * 状态机推进
  * 业务逻辑
  * 诊断格式化

### 17.5 自动代码生成友好原则

文档与代码设计应尽量减少歧义，便于后续 GPT 或模板化生成时保持一致行为。因此必须将以下内容规格化：

* 默认常量
* 阈值
* 状态机行为
* 错误策略
* 事件语义
* Snapshot 字段
* 验收标准

---

## 18. 分阶段实现规范

必须按阶段交付，不得一次性写完整系统。

| 阶段       | 内容                                                                                                                                       | 目标                                                           |
| -------- | ---------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------ |
| **阶段 1** | 边界与骨架：config、`AxisCommand`/`AxisState`、Boundary 结构、`BoundaryIpcManager`、`RtScheduler`、`CycleMonitor`、`RtKernel` / `RtControlPipeline` 骨架 | 双进程共享内存跑通；RT 空转 1 ms；Non-RT 能读快照、写测试命令                       |
| **阶段 2** | EtherCAT 与单轴：`IBusTransport`、`EthercatMaster`、`PdoRegistry`、`DcSyncManager`、`IAxisDriver`、`WaistAxisDriver`、`CiA402StateMachine`         | 单从站上线；PDO 周期读写；单腰轴进入 CSP operation enabled                   |
| **阶段 3** | Teleop + Ruckig：`TeleopInputAdapter`、`RuckigTrajectoryWorker`                                                                            | teleop 改目标；基于实测状态重规划；RT 消费稠密点驱动单轴                            |
| **阶段 4** | 安全与事件：`Watchdog`、`RtSafetyExecutor`、`EventRing` 发布逻辑                                                                                     | 稠密点断粮可 hold；fault 时 quick stop / disable；deadline miss 发事件   |
| **阶段 5** | 在线参数管理：`SdoAccessManager`、参数写入请求 / 结果通路、受控改参状态流                                                                                          | 支持安全改参；关键参数写入可自动 Disable / Pre-Op / Save / Reset / Re-enable |

---

## 19. 阶段 1 交付物规范

阶段 1 的交付内容必须包含：

1. 目录结构
2. 阶段 1 范围内全部头文件与 cpp
3. 共享内存布局定义
4. `rt_main.cpp`
5. `nonrt_main.cpp`
6. 最小 `CMakeLists.txt`
7. 运行说明（如何启动两进程）
8. 共享内存初始化与 attach 说明
9. Snapshot 读取演示
10. 测试命令写入演示

阶段 1 明确不包含：

* ROS 2
* whole-body
* 多轴
* 完整 EtherCAT 接入
* Ruckig 真正执行
* 完整在线参数管理
* 高级诊断平台

---

## 20. P0 成功标准

满足以下条件，视为 P0 成功：

* RT Core Service 与 Non-RT Control Service 可分别独立启动
* 两进程通过共享内存建立稳定通信
* EtherCAT 从站上线
* 单腰轴进入 CiA402 CSP
* RT 周期稳定为 1 ms
* Non-RT 可通过 teleop 修改目标
* `RuckigTrajectoryWorker` 能读取真实状态并生成稠密点
* RT 能逐拍消费稠密点并驱动电机
* Non-RT 能持续读取真实状态
* fault 时系统能进入 hold / quick stop / disable
* DensePointQueue 的欠载、空队列、断粮、满队列、陈旧点、非法点行为符合默认策略表
* 上述行为能通过 `SnapshotBuffer` 和 `EventRing` 观察验证

若阶段 5 纳入在线改参，则额外要求：

* 参数写入请求必须通过 RT 受控执行
* 关键参数写入前能自动退出 Operation Enabled
* PDO 映射参数可按要求切到 Pre-Operational
* 参数写入后可按要求 Save / Reset Node / Re-enable
* 参数流程可通过状态快照与事件系统观察验证

---

## 21. 测试与验收建议

### 21.1 单元测试最低覆盖项

* `CiA402StateMachine` 状态转移
* `DensePointQueue` push / pop / full / empty / stale / illegal
* `SnapshotBuffer` seqlock / double buffer 一致性
* `EventRing` 写入与溢出行为
* `RtSafetyExecutor` 的 hold / quick stop / disable 决策
* 参数写入状态流（若实现）

### 21.2 联调最低验证项

* 双进程共享内存 attach 成功
* RT 1 ms 空转稳定
* Snapshot 可持续读取
* DensePointQueue 能被持续写入与消费
* 欠载 warning 可触发
* 空队列 hold 可触发
* 长时间断粮 quick stop / disable 可触发
* 参数写入期间状态切换符合规范（若实现）

### 21.3 验收原则

* 不仅验证“能动”
* 必须验证“异常时如何退”
* 必须验证“行为是否符合规格”
* 必须验证“状态和事件是否可观测”

---

## 22. 编码规范摘要

* C++17
* 头 / 源分离
* 明确 namespace
* POD 优先
* 固定大小字段优先
* 热路径避免动态分配
* `alignas(64)` 用于共享结构
* 进程间共享结构禁止虚函数
* 热路径函数尽量 `noexcept`
* 总线逻辑、状态机逻辑、业务逻辑、IPC 逻辑严格分离
* 参数写入流程与运动控制流程必须分离建模
* 所有关键默认策略必须在代码中可追踪到常量定义与注释说明

---

## 23. 一句话规格（对外同步版）

> 双进程电机服务原型：RT Motor Core Service 在 1 ms 周期内完成 EtherCAT / PDO / CiA402 / 安全 / 稠密点消费 / 状态快照；Non-RT Motor Control Service 负责 teleop、Ruckig 在线轨迹、真实状态读取、诊断与服务适配。两者边界采用共享内存，核心结构包括 `TeleopTargetMailbox`、`DensePointQueue`、`SnapshotBuffer`、`EventRing` 及可选参数写入请求 / 结果通路，均为固定大小 POD、`alignas(64)`、无阻塞、cache-friendly。P0 聚焦单腰 EtherCAT 轴、CiA402 CSP、基于实测状态的 Ruckig 重规划，以及 RT 每拍执行稠密点；其中 DensePointQueue 的深度、欠载、断粮、满队列、陈旧点策略必须显式规格化。RT 核心服务独立于 ROS 2，RT 内禁止动态分配、阻塞锁、文件 / 网络 IO、重日志与复杂规划。对运行中改参，通常无需专门停 PDO 通信，但很多 SDO 参数写入前应先退出 Operation Enabled，尤其是模式、PDO 映射和关键控制参数。

---

## 24. 文档定位说明

本文档用于：

* 电机服务标准化设计
* 架构评审
* 工程实现对照
* 联调口径统一
* 分阶段验收
* 自动代码生成约束输入
* 后续整机控制系统接入的底层规范基线

其工程定位为：

> 先将“电机服务”做成可规范、可实现、可验收、可复用的标准能力，再向整机控制系统扩展。

---

## 25. 对象字典 / PDO / SDO 变更分级规范

本章定义对象字典（Object Dictionary）相关参数变更的分级规则、执行要求和运行态约束。目标是避免工程实现、联调、自动代码生成和后续维护过程中，对“什么参数可以直接改、什么参数必须停运动、什么参数必须切 Pre-Operational”产生歧义。

### 25.1 设计目标

本章解决以下问题：

* 不同类型对象参数的运行态修改要求不一致
* PDO 映射类变更与普通控制参数变更不能混为一谈
* SDO 写入动作不能由上层直接执行
* 需要统一“参数分级 → 执行动作 → 结果回传 → 验收”的标准流程
* 需要让实现者、评审者、测试者、自动代码生成工具对改参行为有一致理解

### 25.2 适用范围

本章适用于以下参数变更场景：

* EtherCAT 从站对象字典参数修改
* CiA402 驱动对象参数修改
* PDO assignment / PDO mapping 修改
* 控制模式相关参数修改
* 限位、增益、控制关键参数修改
* 设备持久化保存参数操作
* Reset Node / 重新进入工作态相关流程

本章不覆盖：

* 厂家私有参数语义本身的解释
* 厂家对象字典手册替代
* 上位配置平台的权限系统设计
* 批量配置发布平台设计

### 25.3 总原则

对象字典相关变更必须遵循以下原则：

1. 所有改参动作必须分类
2. 所有改参动作必须有运行态约束
3. 所有改参动作必须通过 RT 受控路径执行
4. 所有改参动作必须可观测
5. 所有改参动作必须有明确的失败与拒绝语义
6. 所有改参动作必须能追踪到请求、执行阶段与最终结果
7. 未明确分类的参数，一律按更严格等级处理

### 25.4 参数变更分级

对象字典 / PDO / SDO 变更统一分为五级：

| 等级     | 名称              | 风险级别 | 运行态要求                                     | 典型示例                            |
| ------ | --------------- | ---- | ----------------------------------------- | ------------------------------- |
| **L0** | 只读查询类           | 最低   | 可在运行时读                                    | 状态读取、诊断读取                       |
| **L1** | 普通非关键写入类        | 低    | 受控写入；通常可不切 Pre-Op                         | 某些非实时诊断参数、非控制关键配置               |
| **L2** | 模式 / 运行关键写入类    | 中    | 必须退出 Operation Enabled                    | operation mode、部分运行控制参数         |
| **L3** | 关键控制参数类         | 高    | 必须停运动；通常 Disable Operation；必要时 Hold       | 限位、关键控制阈值、核心行为参数                |
| **L4** | PDO / 通信映射类     | 很高   | 必须切 Pre-Operational，必要时 Save / Reset Node | PDO assignment、PDO mapping、通信对象 |
| **L5** | 厂家定义高风险 / 重启生效类 | 最高   | 按厂家手册执行；通常 Save + Reset Node + Re-enable  | 厂家私有关键参数、启动配置类                  |

> 若无法确定某一参数等级，则默认按 **L4 或 L5** 处理，禁止按低风险参数直接写入。

### 25.5 各分级默认要求

#### 25.5.1 L0：只读查询类

适用对象：

* 状态读取
* 诊断读取
* 非侵入式对象查询

要求：

* 可在运行过程中读取
* 不改变驱动状态
* 不参与运行态切换
* 读失败也必须有结果回传

禁止：

* 将只读失败静默吞掉
* 将查询接口与写接口混在一起设计

#### 25.5.2 L1：普通非关键写入类

适用对象：

* 非关键运行参数
* 非控制闭环核心参数
* 不影响当前 PDO 映射与运行主状态机的对象

默认要求：

* 必须通过参数写入请求路径发起
* 必须经过 RT 受控执行
* 通常无需切 Pre-Operational
* 若厂家要求退出 Operation Enabled，则升级到 L2 处理
* 若厂家要求 Save / Reset，则按更高等级补充动作

默认动作：

1. 检查当前状态是否允许写入
2. 必要时短暂 Hold
3. 执行 SDO 写入
4. 根据对象要求执行可选 Save
5. 回传结果

#### 25.5.3 L2：模式 / 运行关键写入类

适用对象：

* operation mode
* 运行模式相关对象
* 影响当前控制主行为的参数

默认要求：

* 必须先停止运动
* 必须退出 `Operation Enabled`
* 应切到 `Disable Operation`
* 写入后需重新走使能流程
* 若厂家要求 Save / Reset，必须执行

默认动作：

1. 停止轨迹推进
2. 进入 Hold
3. 驱动退出 `Operation Enabled`
4. 执行 SDO 写入
5. 必要时 Save / Reset
6. 重新进入工作态
7. 重新使能
8. 结果回传与事件上报

#### 25.5.4 L3：关键控制参数类

适用对象：

* 位置 / 速度 / 力矩关键限制参数
* 关键控制器参数
* 安全相关阈值
* 会显著影响执行行为的对象

默认要求：

* 必须停运动
* 必须退出 `Operation Enabled`
* 推荐 `Disable Operation`
* 必须进入受控参数写入流程
* 写入失败时不得自动继续原轨迹
* 写入完成前不得恢复普通轨迹消费

默认动作：

1. 停止消费新轨迹点
2. 进入 Hold
3. 切 `Disable Operation`
4. 执行参数合法性预检查
5. 执行 SDO 写入
6. 依据参数要求决定是否 Save / Reset
7. 重新建链并使能
8. 确认驱动状态正常后才允许恢复运动

#### 25.5.5 L4：PDO / 通信映射类

适用对象：

* PDO assignment
* PDO mapping
* 通信周期 / 映射相关对象
* 影响 EtherCAT 周期交互语义的参数

默认要求：

* 必须停运动
* 必须退出 `Operation Enabled`
* 必须切到 `Pre-Operational`
* 必须串行化执行
* 写入后通常要求重新建立映射
* 必要时 Save / Reset Node
* 恢复前必须校验映射一致性

默认动作：

1. 停止运动
2. 停止普通轨迹消费
3. 切 `Disable Operation`
4. 切 EtherCAT `Pre-Operational`
5. 执行 PDO assignment / mapping 写入
6. 执行一致性检查
7. 必要时 Save
8. 必要时 Reset Node
9. 恢复到 Operational
10. 重新建链
11. 重新使能
12. 结果回传

#### 25.5.6 L5：厂家高风险 / 重启生效类

适用对象：

* 厂家文档明确标注需重启生效的对象
* 厂家私有关键参数
* 可能影响启动行为、内部控制模式、通信行为的参数

默认要求：

* 必须按厂家手册执行
* 未明确前不得在运行中随意写入
* 原则上视为最高等级受控动作
* 默认要求 Save + Reset Node + 重新使能
* 必要时要求人工确认或运维窗口（后续版本可扩展）

### 25.6 参数分级执行矩阵

| 等级 | 允许运行态直接写入 | 必须停运动   | 必须 Disable Operation | 必须 Pre-Operational | 可能 Save | 可能 Reset Node |
| -- | --------- | ------- | -------------------- | ------------------ | ------- | ------------- |
| L0 | 是（只读）     | 否       | 否                    | 否                  | 否       | 否             |
| L1 | 受控条件下可    | 否 / 视情况 | 否 / 视厂家要求            | 否                  | 可选      | 可选            |
| L2 | 否         | 是       | 是                    | 否                  | 可选      | 可选            |
| L3 | 否         | 是       | 是                    | 视情况                | 常见      | 可选            |
| L4 | 否         | 是       | 是                    | 是                  | 常见      | 常见            |
| L5 | 否         | 是       | 是                    | 常见                 | 常见      | 常见            |

### 25.7 参数写入请求规范

参数写入请求至少应包含以下字段：

* `request_sequence`
* `axis_id` / `slave_id`
* `object_index`
* `subindex`
* `value`
* `value_size`
* `parameter_level`
* `parameter_category`
* `requires_disable_op`
* `requires_preop`
* `requires_save`
* `requires_reset_node`
* `request_timestamp`

要求：

* 请求结构必须固定大小
* 必须支持显式分级字段
* 必须支持执行期覆盖更严格等级
* 不允许仅靠 object index 临时硬编码推断而无记录

### 25.8 参数写入状态机建议

参数写入流程建议定义独立状态机，最少包含以下阶段：

| 状态                        | 含义                     |
| ------------------------- | ---------------------- |
| `Idle`                    | 空闲                     |
| `Accepted`                | 请求已接收                  |
| `Rejected`                | 请求被拒绝                  |
| `PrepareStopMotion`       | 准备停运动                  |
| `PrepareDisableOperation` | 准备退出 Operation Enabled |
| `PreparePreOperational`   | 准备切 Pre-Operational    |
| `WritingParameter`        | 正在执行 SDO 写入            |
| `SavingParameter`         | 正在保存参数                 |
| `ResettingNode`           | 正在 Reset Node          |
| `RecoveringOperational`   | 正在恢复工作态                |
| `ReEnablingDrive`         | 正在重新使能驱动               |
| `Completed`               | 完成                     |
| `Failed`                  | 失败                     |

要求：

* 参数写入流程不得散落在多个模块临时拼接
* 必须由独立控制逻辑统一管理
* 每个阶段必须可观测
* 每个阶段失败必须有 reason code

### 25.9 参数写入拒绝条件

出现以下情况时，RT 应拒绝参数写入请求：

* 当前驱动处于未定义状态
* 当前总线状态异常
* 当前已有更高优先级故障处理流程在执行
* 当前已有参数写入流程进行中
* 请求参数分级缺失且无法安全推断
* 请求值不合法
* 请求与当前设备状态冲突
* 厂家要求不允许在线修改
* 请求来自未授权路径（后续权限系统可扩展）

拒绝后必须：

* 写结果缓冲区
* 发事件
* 在快照中可观察到最近一次拒绝结果

### 25.10 参数写入结果规范

参数写入结果至少应包含：

* `request_sequence`
* `axis_id`
* `final_status`
* `reason_code`
* `last_stage`
* `write_success`
* `save_executed`
* `reset_executed`
* `reenable_success`
* `result_timestamp`

结果状态建议枚举：

* `Accepted`
* `Rejected`
* `Completed`
* `Failed`
* `TimedOut`
* `Aborted`

### 25.11 验收要求

参数分级规范的最低验收要求包括：

* 不同等级参数能进入正确执行流
* L2 参数写入前会退出 `Operation Enabled`
* L4 参数写入时会切 `Pre-Operational`
* 需要 Save / Reset 的参数能按流程执行
* 失败 / 拒绝 / 中断 / 成功都能通过结果与事件观察到
* 未分类参数默认按更严格等级处理
* 自动代码生成实现时，能直接依据分级矩阵生成一致流程

---

## 26. 错误码、事件码、状态码统一编号规范

本章定义**错误码（Error Code）**、**事件码（Event Code）**、**状态码（Status Code）**的编号规则、编码空间、模块分段与扩展原则。目标是让代码、日志、事件流、状态快照、自动测试、联调工具和自动代码生成都使用一致的编码体系。

### 26.1 设计目标

统一编号规范用于解决以下问题：

* 不同模块自行定义错误码，造成冲突
* 事件与状态语义混淆
* 日志里有文本但没有稳定编号
* 联调时看起来一样的错误其实来源不同
* 自动代码生成容易重复或乱用编号
* 后续对接 GUI / RPC / 诊断工具时缺少统一协议

### 26.2 编码对象定义

本规范统一覆盖三类编号：

| 类别                   | 含义                       | 典型用途                        |
| -------------------- | ------------------------ | --------------------------- |
| **错误码（Error Code）**  | 表示失败、异常、拒绝、不可恢复或需人工处理的结果 | API 返回、执行失败、参数写入失败、总线失败     |
| **事件码（Event Code）**  | 表示某个时刻发生了可记录事件           | EventRing、告警、告知、状态跃迁通知      |
| **状态码（Status Code）** | 表示某模块或某流程当前处于什么状态        | SnapshotBuffer、流程状态机、驱动状态摘要 |

要求：

* 三类编号不得混用
* 同一编号空间内必须有模块分段
* 编号必须稳定、可机器解析、可文档映射

### 26.3 编码总原则

1. 编号必须是稳定整数
2. 编号必须可分段识别来源模块
3. 编号必须支持未来扩展
4. 编号必须支持自动化测试断言
5. 编号必须支持日志与 UI 显示映射
6. 编号必须避免依赖文本匹配
7. 文本描述可以变化，编号语义不得随意变化

### 26.4 编码格式建议

建议统一采用 32 位无符号整数编码，并按高位分段：

```text
[ Type ][ Domain ][ Module ][ Specific Code ]
```

也可采用十六进制分段常量规范：

```text
0xTTDDMMSS
```

其中：

* `TT`：类型
* `DD`：域
* `MM`：模块
* `SS`：具体码值

### 26.5 类型编号约定

| 类型     | 名称          | 编码前缀 |
| ------ | ----------- | ---- |
| `0x01` | Error Code  | 错误码  |
| `0x02` | Event Code  | 事件码  |
| `0x03` | Status Code | 状态码  |

示例：

* `0x01xxxxxx` 表示错误码
* `0x02xxxxxx` 表示事件码
* `0x03xxxxxx` 表示状态码

### 26.6 域编号约定

| 域      | 含义                              | 编码        |
| ------ | ------------------------------- | --------- |
| `0x01` | RT Kernel / Scheduling          | RT 调度与周期  |
| `0x02` | Bus / EtherCAT                  | 总线与通信     |
| `0x03` | Axis / CiA402                   | 轴与状态机     |
| `0x04` | Boundary IPC                    | 共享内存与边界结构 |
| `0x05` | DensePoint / Planning Interface | 稠密点与轨迹接口  |
| `0x06` | Safety / Watchdog               | 安全与看门狗    |
| `0x07` | Parameter Write Flow            | 参数写入      |
| `0x08` | Snapshot / Event Export         | 快照与事件导出   |
| `0x09` | Non-RT Aggregation / Tooling    | 非实时聚合与工具  |
| `0x0A` | Reserved                        | 预留        |

### 26.7 模块内具体码段要求

每个域下的 `Module` 与 `Specific Code` 必须满足：

* `Module` 用于区分同一域中的子模块
* `Specific Code` 用于区分模块内具体错误 / 事件 / 状态
* `0x00` 保留给未定义
* `0x01 ~ 0x1F` 推荐给通用基础码
* `0x20 ~ 0x7F` 推荐给模块公共码
* `0x80 ~ 0xFF` 预留扩展或厂家特殊码映射

### 26.8 错误码规范

错误码用于表示：

* 请求失败
* 操作被拒绝
* 总线失败
* 状态机失败
* 参数写入失败
* 队列非法数据
* 运行时严重异常

#### 26.8.1 错误码命名建议

统一采用：

```text
ERR_<DOMAIN>_<MODULE>_<MEANING>
```

例如：

* `ERR_BUS_MASTER_OFFLINE`
* `ERR_AXIS_CIA402_INVALID_TRANSITION`
* `ERR_DENSE_QUEUE_FULL`
* `ERR_PARAM_WRITE_REJECTED`
* `ERR_PARAM_PREOP_REQUIRED`
* `ERR_SNAPSHOT_SEQ_BROKEN`

#### 26.8.2 建议错误码示例

| 名称                                   | 示例编号         | 说明        |
| ------------------------------------ | ------------ | --------- |
| `ERR_RT_DEADLINE_MISS`               | `0x01010101` | RT 周期超时   |
| `ERR_BUS_MASTER_OFFLINE`             | `0x01020101` | 总线离线      |
| `ERR_AXIS_CIA402_INVALID_TRANSITION` | `0x01030101` | 非法状态跳转    |
| `ERR_BOUNDARY_SHM_VERSION_MISMATCH`  | `0x01040101` | 共享内存版本不匹配 |
| `ERR_DENSE_QUEUE_FULL`               | `0x01050101` | 队列满       |
| `ERR_DENSE_POINT_STALE`              | `0x01050102` | 陈旧点       |
| `ERR_SAFETY_WATCHDOG_TIMEOUT`        | `0x01060101` | 看门狗超时     |
| `ERR_PARAM_WRITE_REJECTED`           | `0x01070101` | 改参请求被拒绝   |
| `ERR_PARAM_WRITE_FAILED`             | `0x01070102` | 改参失败      |
| `ERR_EVENT_EXPORT_OVERFLOW`          | `0x01080101` | 事件导出溢出    |

### 26.9 事件码规范

事件码用于表示“发生过什么”，不一定表示错误。典型场景包括：

* 模式切换
* 欠载告警
* 断粮开始
* fault 发生
* 参数写入开始
* 参数写入完成
* 参数写入被拒绝

#### 26.9.1 事件码命名建议

统一采用：

```text
EVT_<DOMAIN>_<MODULE>_<MEANING>
```

例如：

* `EVT_DENSE_UNDERRUN_RISK`
* `EVT_DENSE_STARVATION_BEGIN`
* `EVT_AXIS_FAULT_ACTIVE`
* `EVT_PARAM_WRITE_STARTED`
* `EVT_PARAM_WRITE_COMPLETED`

#### 26.9.2 事件等级

建议事件同时带 severity：

* `info`
* `warning`
* `error`
* `critical`

#### 26.9.3 建议事件码示例

| 名称                                | 示例编号         | 说明            |
| --------------------------------- | ------------ | ------------- |
| `EVT_RT_CYCLE_JITTER_HIGH`        | `0x02010101` | 周期抖动较高        |
| `EVT_BUS_STATE_CHANGED`           | `0x02020101` | 总线状态变化        |
| `EVT_AXIS_MODE_CHANGED`           | `0x02030101` | 轴模式变化         |
| `EVT_DENSE_UNDERRUN_RISK`         | `0x02050101` | 队列欠载          |
| `EVT_DENSE_STARVATION_BEGIN`      | `0x02050102` | 断粮开始          |
| `EVT_DENSE_STARVATION_END`        | `0x02050103` | 断粮结束          |
| `EVT_DENSE_QUEUE_FULL`            | `0x02050104` | 队列写满          |
| `EVT_SAFETY_HOLD_ENTERED`         | `0x02060101` | 进入 hold       |
| `EVT_SAFETY_QUICK_STOP_TRIGGERED` | `0x02060102` | quick stop 触发 |
| `EVT_PARAM_WRITE_STARTED`         | `0x02070101` | 参数写入开始        |
| `EVT_PARAM_WRITE_REJECTED`        | `0x02070102` | 参数写入被拒绝       |
| `EVT_PARAM_WRITE_COMPLETED`       | `0x02070103` | 参数写入完成        |
| `EVT_PARAM_WRITE_FAILED`          | `0x02070104` | 参数写入失败        |

### 26.10 状态码规范

状态码用于表达“当前处于什么状态”，通常用于快照与状态机摘要。

#### 26.10.1 状态码命名建议

统一采用：

```text
STS_<DOMAIN>_<MODULE>_<STATE>
```

例如：

* `STS_AXIS_OPERATION_ENABLED`
* `STS_DENSE_QUEUE_UNDERRUN`
* `STS_PARAM_FLOW_WRITING`
* `STS_PARAM_FLOW_RESETTING_NODE`

#### 26.10.2 建议状态码示例

| 名称                              | 示例编号         | 说明                     |
| ------------------------------- | ------------ | ---------------------- |
| `STS_RT_RUNNING`                | `0x03010101` | RT 正常运行                |
| `STS_BUS_OPERATIONAL`           | `0x03020101` | 总线 Operational         |
| `STS_AXIS_SWITCH_ON_DISABLED`   | `0x03030101` | 轴处于 Switch on disabled |
| `STS_AXIS_OPERATION_ENABLED`    | `0x03030102` | 轴处于 Operation enabled  |
| `STS_DENSE_QUEUE_NORMAL`        | `0x03050101` | 队列正常                   |
| `STS_DENSE_QUEUE_UNDERRUN`      | `0x03050102` | 队列欠载                   |
| `STS_DENSE_QUEUE_STARVING`      | `0x03050103` | 队列断粮                   |
| `STS_SAFETY_HOLD_ACTIVE`        | `0x03060101` | Hold 生效中               |
| `STS_SAFETY_QUICK_STOP_ACTIVE`  | `0x03060102` | Quick stop 生效中         |
| `STS_PARAM_FLOW_IDLE`           | `0x03070101` | 参数流空闲                  |
| `STS_PARAM_FLOW_WRITING`        | `0x03070102` | 正在写参数                  |
| `STS_PARAM_FLOW_RESETTING_NODE` | `0x03070103` | 正在 Reset Node          |
| `STS_PARAM_FLOW_RECOVERING`     | `0x03070104` | 正在恢复工作态                |

### 26.11 编码使用规范

#### 26.11.1 SnapshotBuffer

Snapshot 中建议暴露：

* 当前 RT 状态码
* 当前 bus 状态码
* 当前 axis 状态码
* 当前 dense queue 状态码
* 当前 safety 状态码
* 当前 parameter flow 状态码
* 最近一次错误码
* 最近一次事件码

#### 26.11.2 EventRing

EventRing 中必须写入：

* `event_code`
* `severity`
* `source_id`
* `timestamp`
* `payload`

#### 26.11.3 API / CLI / RPC（后续）

外部接口不得只返回自然语言字符串，应优先返回：

* result code
* error code
* event code
* status code

文本描述只能作为辅助，不得替代编号语义。

### 26.12 编码保留与扩展规则

* 已发布编号不得随意复用
* 已废弃编号应保留历史记录，不得重新赋予新语义
* 新增编号必须写入统一表
* 私有测试编号必须放在预留段
* 厂家私有码映射建议单独保留域或高位区间

### 26.13 自动代码生成约束

为减少自动代码生成或模板化实现时的混乱，必须保证：

* 错误码、事件码、状态码统一集中定义
* 枚举与常量命名遵循固定模式
* 编号与文档表格一一对应
* 测试中断言使用编号，不依赖日志文本
* 事件流与结果流能通过编号稳定验证

### 26.14 最低验收要求

编号规范的最低验收要求包括：

* 所有关键模块均使用统一编号空间
* 错误码、事件码、状态码不混用
* `SnapshotBuffer` 至少携带最近状态码与最近错误码
* `EventRing` 事件具有稳定 `event_code`
* 参数写入流程每个关键阶段都能以状态码或事件码观察
* 自动测试可对关键行为进行编号级断言

### 26.15 建议的统一头文件

建议后续代码中提供统一定义文件：

```text
core/common/error_codes.hpp
core/common/event_codes.hpp
core/common/status_codes.hpp
```

或统一为：

```text
core/common/service_codes.hpp
```

要求：

* 常量集中定义
* 注释与文档保持一致
* 编号段按域分组
* 不允许业务代码随意散落临时编号

---
