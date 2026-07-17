# Hex25E firmware change process — 2026-07-16

本报告只记录能够从源码、SysConfig 生成结果或编译结果确认的问题。没有完成板上
动态验证的项目明确标记为“待板测”，不据此推断电机、编码器或传感器的实际行为。

## 1. 产品代码错误放入 Device 层

- 错误证据：存在 `Device/Car25E`，其中包含 25E 遥测格式、产品串口角色、机械参数、
  USERKEY 行为和资源组合；`Car25E` 不是器件型号，也不符合 Device 按 Input、
  Actuator、Sensor、Communication 分类的规则。
- 修改：删除 `Device/Car25E`。串口 BSP 封装改为通用
  `Device/Communication/Serial`；实体按键输入适配改为通用
  `Device/Input/Key/device_key_input`；25E 通信、IMU 搬运、机械参数和资源角色移入
  `Task/Hex25E`。
- 验证：`tools/check_template.ps1` 通过；Task 未直接包含 BSP/DriverLib 头文件。

## 2. VOFA 通道定义与需求不匹配

- 错误证据：旧 16 通道包含 current node、next node、visited mask、dwell 和电机原始
  command，却没有 JY61P 三轴角速度、三轴欧拉角和左右累计里程。
- 修改：VOFA JustFloat 改为 23 通道，顺序固定为：
  `time_s, state, fault, imu_online, gx, gy, gz, roll, pitch, yaw,
  left_odometry_mm, right_odometry_mm, left_target_mm_s, right_target_mm_s,
  left_speed_mm_s, right_speed_mm_s, left_speed_error_mm_s,
  right_speed_error_mm_s, left_motor_percent, right_motor_percent,
  line_raw_mask, line_active_mask, line_error`。
- 数据格式：23 个 IEEE-754 `float` 后追加 JustFloat 帧尾
  `00 00 80 7F`，总帧长 96 bytes。
- 待板测：VOFA 上位机按上述顺序建立通道后检查波形和值域。

## 3. 编码器上报了累计计数而不是累计里程

- 错误证据：蓝牙字段使用 `EC` 并发送左右 `int64_t cumulative_counts`；旧 VOFA 甚至
  没有累计量。该数据不满足“累计里程和编码器速度”的要求。
- 修改：蓝牙字段改为 `OD:left_mm,right_mm`，VOFA 通道 10、11 改为左右累计里程
  `mm`；通道 14、15 为左右编码器速度 `mm/s`。
- 换算参数：每电机转 500 线、减速比 20.409、QEI 四倍频、轮周长 150.796 mm。
- 待板测：轮子架空按已知圈数旋转，核对左右里程、速度比例和符号。

## 4. Device Encoder 初始化零点实现错误

- 错误证据：`Device_Encoder_Init()` 注释声明丢弃初始化前累计量，但旧实现把 BSP
  累计值直接写入 `sample.cumulative_counts`，没有保存和扣除初始化零点。
- 修改：初始化时保存当前逻辑计数为 `zero_counts`；每次更新先完成方向变换，再计算
  `relative_counts = absolute_counts - zero_counts`，累计里程由相对计数换算。
- 当前语义：上电初始化时里程为 0；每次 USERKEY/START 重新初始化路线时重新取零。
  STOP 不清零，停止后若车轮仍有运动，编码器里程继续累计；下一次重新开始时再归零。
- 验证：源码路径已加入有符号减法溢出检查；3519 Debug 编译通过。

## 5. VOFA 无法区分编码器反馈错误与电机输出错误

- 错误证据：旧表只有目标速度、实测速度和缩放不明确的整数 motor command，没有
  明确速度误差；修改初版又遗漏了目标速度和电机输出。
- 修改：同时输出左右目标速度、实测速度、`target-measured` 速度误差和左右电机输出。
  电机输出由内部 `[-10000,10000]` 命令乘 `0.01` 后按百分比上报，当前控制器限幅为
  `[-55%,55%]`。
- 待板测：核对目标、反馈、误差和电机百分比四组波形的因果关系；未板测前不判断
  编码器 A/B 相位或电机机械极性是否正确。

## 6. JY61P 数据已解析但未接入产品遥测

- 错误证据：存在通用 `Device_JY61P` 解析器，但产品任务没有持续搬运对应串口字节，
  蓝牙和 VOFA 也没有 JY61P 字段。
- 修改：`Task/Hex25E/task_hex25e_imu` 每 10 ms 非阻塞搬运字节并喂给解析器；蓝牙与
  VOFA 输出 IMU online、三轴角速度和三轴欧拉角。无有效帧时 VOFA 对角速度/角度
  输出 NaN，并用 `imu_online=0` 明确标记。
- 后续板测确认实际模块使用 115200 baud，见本报告第 13 节。

## 7. USERKEY 启动路径存在非需求的 2 秒延时

- 错误证据：`Task_Hex25E_Start()` 进入 `START_DELAY`，常量为 2000 ms。
- 修改：删除该状态；启动时取得编码器零点并直接进入 `EXIT_NODE`，同一个 10 ms
  控制周期计算轮速目标。再次按键仍同步执行停车、制动和软件锁。
- 待板测：车辆架空后验证第一次按键开始输出、第二次按键当前周期归零。

## 8. 蓝牙 DMA 短包可能无法及时提交

- 错误证据：RX DMA staging buffer 为 64 bytes，只有通道满才产生 DMA 完成中断；
  `STOP\r\n` 等短包不能依赖满缓冲事件。
- 修改：UART RX timeout 中断提交短包；任务读取前检查 DMA remaining count，作为
  timeout 被 DMA 清空 FIFO 抑制时的有限兜底。两条路径都只提交字节，不解析命令。
- TX：任务只入队完整帧，DMA_CH1 完成中断推进 tail 并启动下一连续段。
- 错误恢复：DMA address/data error 计数后丢弃无法确认边界的 TX 段并重新装载 RX。
- 验证：SysConfig 生成结果包含 UART1 RX/TX DMA trigger、DMA_CH0/CH1 interrupt；
  ELF map 中存在 `DMA_IRQHandler`。待板测短命令与连续遥测并发。

## 9. 蓝牙遥测周期和内容不满足要求

- 错误证据：旧周期为 500 ms，旧状态帧只有 STATE、NODE、NEXT、Y、RAW、LINE、FAULT。
- 修改：周期改为 50 ms；上电第一轮主动发送；字段加入 IMU、累计里程、编码器速度和
  数字循迹状态。RX/TX 均为缓冲非阻塞路径。
- 待板测：记录实际帧周期、是否丢帧和 DMA error counter。

## 10. 编译验证边界

- `MSPM0G3519 / Car25E / Debug`：编译通过，`text=37384`、`data=1716`、
  `bss=7108`，主 Flash HEX 为 39028 bytes，范围 `0x00000000-0x00009877`。
- `MSPM0G3507 / Car25E / Debug`：编译通过，`text=31192`、`data=124`、
  `bss=6852`，主 Flash HEX 为 31316 bytes，范围 `0x00000000-0x00007A57`。
- 模板架构检查通过。
- 本报告建立时尚未烧录该版本，也未完成电机、编码器、JY61P、蓝牙和 VOFA 的板上
  动态验证；因此这些项目不得写成“已验证正常”。

后续每次修改已确认的错误，继续按“错误证据、修改、验证、待板测”四项追加。

## 11. “转一下停一下”的自动停车路径违反按键控制要求

- 错误证据：进入 `FOLLOW_LINE` 后，只要 `line_lost=true`，旧实现立即设置
  `moving=false`；持续丢线还会进入 sensor fault。编码器速度接近 0 超过 500 ms 也会
  进入 stall fault。两条路径都不需要 USERKEY 即可停车。
- 修改：丢线时保持左右 220 mm/s 直行并继续等待数字循迹数据；删除 line-loss timeout
  和 encoder-stall 自动停车；删除转向重捕获超时停车。USERKEY 是正常启停的唯一控制，
  保留需求明确规定的 Y 路口 5 s 停留、路线完成停车，以及底层电机输出失败时的锁定。
- 验证：源码中已删除三条 timeout/stall 调用路径；待车辆架空后动态验证。

## 12. VOFA 发到了错误的串口并混入 ASCII

- 错误证据：`vofa+.csv` 只有通道 `I0`，838 个采样全部为 0；当前遥测通道 0 本应为
  递增时间。旧实现把 JustFloat 发到 UART0/PA10-PA11，却在蓝牙 UART1/PB4-PB5 上周期
  发送 ASCII 状态、ACK 和错误文本。
- 修改：蓝牙 UART1 成为唯一 VOFA JustFloat 输出；移除周期 ASCII、ACK 和 UART 错误
  文本，避免二进制流被混合协议破坏。蓝牙 RX 仍可 DMA 接收，只有 `STATUS` 触发一次
  即时遥测；START/STOP 不再改变电机状态。
- 验证：新帧为固定 96 bytes，50 ms 一帧；待重新导出 VOFA CSV 验证通道 0 时间递增。

## 13. JY61P UART 波特率配置错误

- 错误证据：板上运行 3.86 s 时 UART4 RX head/tail 均推进到 76，说明已有字节被 DMA
  搬运；但 `valid_frame=0`、`checksum_error=0`、`framing_error=736`。旧配置为 9600。
  参考工程同一 UART4、PB10/PB11 的 JY61P 配置为 115200，用户也确认 115200。
- 修改：UART4 改为 115200、8N1；RX 使用 DMA_CH2，UART timeout 提交短块，任务解析
  `55 51/52/53` 帧。
- 待验证：烧录后重新读取 valid frame、checksum error、framing error 和 online。

## 14. 八路数字灰度“没有调用”的判断不成立

- 板上证据：同一 3.86 s 采样中控制任务更新 387 次，`RAW=0xFF`、`LINE=0x00`。
  说明 PB8 DAT 已在每个 10 ms 周期随 PB9 CLK 读取 8 次，但 8 个采样点均为高电平。
- 用户随后确认灰度模块当时没有开启，因此 PB8 内部上拉使 8 位均为高，`RAW=0xFF`
  与该条件一致，不能用这组数据评价传感器或采样时序。
- GPIO 同步串行不是 UART，不能直接使用 UART DMA。待模块开启后再通过 VOFA 观察
  RAW/LINE；当前不对未上电模块作故障归因。

## 15. 灰度 CLK 采样相位的误判与回退

- 误判：在不知道灰度模块未开启的情况下，曾把 `RAW=0xFF` 归因于 CLK 采样相位，
  并临时修改为空闲高、低相位采样。
- 回退：用户说明模块未开启后，该依据失效；临时时序修改已完整撤销，恢复此前的
  CLK 空闲低、拉高后读取 DAT 的实现。
- 验证边界：启用模块前不再修改灰度协议时序。

## 16. 最终烧录与板上只读验证

- 最终 3519 固件：`text=35792`、`data=1716`、`bss=7092`；主 Flash HEX
  37436 bytes，范围 `0x00000000-0x0000923F`。DAPLink 写入后回读 37440 bytes，
  SHA-256 一致，随后 reset run。
- 3.91 s 运行快照：`g_TASK_UpdateCount=391`、`OutputEnabled=0`，固件处于等待
  USERKEY 状态。
- JY61P：`valid_frame_count=2345`、`online=true`；角速度为
  `[-0.3052, 0.2441, 0.3052] dps`，欧拉角为
  `[-35.2771, 32.8162, 150.5951] deg`；UART4 framing error=0、DMA error=0。
- 蓝牙 VOFA TX DMA：`tx_head=160`、`tx_tail=160`、`dma_tx_length=0`、DMA error=0。
  head/tail 已从零推进且无积压，证明固定 96-byte JustFloat 帧持续完成发送。
- 灰度：`RAW=0xFF`、`LINE=0x00`；用户确认模块未开启，因此该值只记录为 DAT
  上拉状态，不作为灰度功能验证。
- 最终 3507 Debug 编译：`text=29432`、`data=124`、`bss=6836`，主 Flash HEX
  29556 bytes，范围 `0x00000000-0x00007377`。
- 模板架构检查通过。电机未由调试会话启动；“运行后只有 USERKEY 正常停机”的动态
  行为仍需车辆架空后由用户按键验证。

## 17. JustFloat 无字段名，VOFA 数据无法直接用于停车诊断

- 错误证据：JustFloat 只传输按固定位置排列的 `float`，VOFA 导出显示为 `I0...I22`；
  它不携带通道名和单位。旧 `vofa+.csv` 只有 `I0` 且 838 个样本全为 0，既不是当前
  固件的有效遥测，也无法区分按键停车、Y 路口停留、路线结束、故障锁定或闭环输出为零。
- 修改：蓝牙 UART1 的唯一 TX 数据改为 VOFA FireWater 命名文本，115200 baud、50 ms
  一帧。字段显式带语义和单位，包括 `run/key/key_edges/state/fault/node/next/y_mask/dwell_ms`、JY61P
  角速度与欧拉角、左右累计里程 mm、目标/实测/误差 mm/s、电机输出百分比以及灰度
  `raw/line/line_err`。TX 仍走 DMA_CH1，不混入 ACK 或错误文本。
- 容量修正：FireWater 文本不是固定 96 bytes；单帧上限改为 480 bytes，3519 UART TX
  环形缓冲由 512 增至 1024 bytes，避免一帧接近旧环形缓冲有效容量 511 bytes 时无法
  入队。
- 编译验证：3519 Debug 为 `text=37408`、`data=4020`、`bss=7100`，主 Flash HEX
  41356 bytes、范围 `0x00000000-0x0000A18F`；3507 Debug 为 `text=30960`、
  `data=124`、`bss=6844`，主 Flash HEX 31084 bytes、范围
  `0x00000000-0x0000796F`。模板架构检查通过。
- 验证边界：本次尝试连接板上快照时 OpenOCD 返回
  `unable to find a matching CMSIS-DAP device`，Windows 也未枚举到目标 DAPLink；因此
  当前只完成源码修改，编译结果和重新烧录结果在后续条目记录，不能声称本版已板测。

## 18. “未再次按键却停车”缺少可证伪的按键边沿记录

- 错误证据：旧遥测只给出路线状态，没有当前去抖按键电平，也没有从上电开始累计的
  有效按下边沿数。停车后看到 `state=0` 时，无法区分是否真的进入了第二次按键分支。
- 修改：增加 `key` 和 `key_edges` 两个 FireWater 字段，并保留同名全局调试变量
  `g_TASK_UserKeyPressed`、`g_TASK_UserKeyPressCount`。只有 30 ms 去抖后产生的按下边沿才
  增加 `key_edges`；长按、释放和蓝牙 RX 均不增加该计数。
- 静态配置证据：3519 USERKEY 为 PB31、低有效，SysConfig 生成代码明确配置内部上拉；
  当前源码不存在按键释放停车路径。
- 待板测：复现短转后停车时读取 `key_edges/state/fault/run/motor_l_pct/motor_r_pct`。若
  `key_edges=2,state=0` 才能证明按键输入产生了第二次有效边沿；若 `key_edges=1`，则按
  `state/fault/run` 继续定位，不能把原因归到按键。

## 19. FireWater 与按键诊断版编译完成，但未烧录

- 3519 Debug 编译通过：`text=37408`、`data=4020`、`bss=7100`，主 Flash HEX 为
  41356 bytes，范围 `0x00000000-0x0000A18F`。
- 3507 Debug 编译通过：`text=30960`、`data=124`、`bss=6844`，主 Flash HEX 为
  31084 bytes，范围 `0x00000000-0x0000796F`；模板架构检查通过。
- ELF 证据：3519 ELF 包含 `time_s/key_edges/odom_l_mm/motor_l_pct/line_err` FireWater
  字段字符串，并导出 `g_TASK_UserKeyPressCount`、`g_TASK_UserKeyPressed`、
  `g_TASK_RouteState`、`g_TASK_OutputEnabled` 和 `g_TASK_ModeFault` 调试符号。
- 烧录失败边界：Windows 当前枚举的是 `VID_0D28&PID_4001`、序列号 `123456` 的
  USB COM5，未枚举配置所需的 `VID_0D28&PID_0204` DAPLink CMSIS-DAP。分别使用本地
  已保存序列号和 `123456` 连接，OpenOCD 都明确返回
  `unable to find a matching CMSIS-DAP device`；因此本节固件没有写入目标板，不能用
  当前板上行为评价该版本。

## 20. 出发节点残留路口会被误判为下一个 Y 路口并自动停车

- 错误证据：旧状态机在离开节点累计 150 mm 后无条件从 `EXIT_NODE` 进入
  `FOLLOW_LINE`。下一周期只要灰度仍连续 3 次给出 intersection 和外侧通道有效，就
  直接把 `route_index` 从 O 增加到 A；再前进 100 mm 后把它当成首次到达 A，进入规定的
  5 s Y 路口停留。该源码路径不需要第二次 USERKEY，理论触发距离约为
  `150+100=250 mm`，与地图 400 mm 节点中心距不一致。
- 修改：新增 `junction_armed`。离开上一个节点后必须至少观察到一次
  `line_lost=0 && intersection=0` 的普通线段，才允许识别下一个路口；同时要求从上一
  节点中心累计行驶至少 `400-100=300 mm` 才允许确认路口入口，再行驶 100 mm 到节点
  中心。进入新路口或开始转向时重新撤销 armed 状态。
- 停车边界：该修改只推迟错误的路口确认，不增加停车条件。灰度丢失时仍保持左右
  220 mm/s 目标速度；正常停车入口仍只有 USERKEY、已确认 Y 路口 5 s、路线完成和
  底层不可恢复错误。
- 可观测性：FireWater 增加 `line_lost/intersection/junction_armed`，可直接区分灰度
  状态与已武装的新路口检测。
- 待验证：仅做静态代码路径确认；高电流电机测试不挂 DAPLink，改用蓝牙 FireWater
  记录状态。编译和重新烧录结果另行记录。

## 21. VOFA FireWater 可能间断且没有丢帧证据

- 错误证据：原命名 FireWater 典型帧约 431 bytes，在 115200、50 ms 周期下占串口线速
  约 75%；字段值变长时会接近 480-byte 格式缓冲上限。格式超长或 TX 环形缓冲暂时忙
  时 `Task_Hex25E_Comm_SendVofa()` 直接返回 false，旧调度又在之后每个 10 ms 控制周期
  重试，既没有固定采样节拍，也没有序号或丢帧计数。
- 修改：保留可读 FireWater 字段但缩短重复单位后缀，单位固定为角速度 dps、角度 deg、
  里程 mm、速度 mm/s 和 PWM percent；加入 `seq` 与 `drop`。调度严格保持 50 ms，失败
  帧计入 drop，下一帧的 seq 跳变可由 VOFA 直接识别，不再以 10 ms 频率乱重试。
- DMA 静态错误：旧 TX 队列只在 DMA_CH1 完成中断中推进 `tx_tail`。增加任务侧兜底：
  入队和查询 busy 前若 DMA 剩余传输数已经为 0，先清除旧通道事件并完成当前环形缓冲
  段，避免一次未及时处理的完成事件永久堵住后续遥测。BSP 能力信息同步改为如实报告
  Bluetooth TX/RX 使用 DMA。
- 验证边界：本项是静态链路修复；高电流测试期间不连接 DAPLink，只通过蓝牙检查
  `seq` 是否连续、`drop` 是否保持 0。编译与烧录结果另行记录。

## 22. FireWater 字段未分类且单行被截断

- 运行证据：用户收到的一行从 `time_s` 连续堆叠系统、IMU、编码器、闭环、电机和灰度
  字段，最终停在不完整的 `line_er`，无法按功能阅读，也证明单行组织已触及接收/显示
  边界。
- 修改：每 50 ms 固定发送一行 `DRIVE/ENC/CTRL/MOTOR`，保证目标、反馈、误差和 PWM
  位于同一行；第二行在 `ROUTE`、`IMU`、`LINE` 三类间轮换，每类 150 ms 更新一次。
  FireWater 通道名全部带分类前缀，终端和 VOFA 通道列表均可按前缀直接分组。
- 带宽：分类不是把全部长字段复制到每一行；高频闭环数据持续 20 Hz，三个慢分类各
  6.67 Hz，避免再次形成一条接近 480 bytes 的超长行。`DRIVE_seq/DRIVE_drop` 保留。

## 23. 右编码器逻辑方向与右轮正向目标相反

- 运行证据：同一帧中 `target_r=+260.00 mm/s`、`speed_r=-1290.81 mm/s`、
  `err_r=+1550.81 mm/s`、`motor_r=+55.00%`，且累计右里程为 `-36094.90 mm`；左轮在
  同一正目标下速度和里程均为正。这证明右反馈符号与正向目标构成正反馈，直接导致右
  PWM 长时间饱和。
- 修改：3519/3507 共用产品任务中的右编码器逻辑方向由 `-1` 改为 `+1`，路线重新启动
  时仍按当前硬件计数取零。修改后相同实际转向的右速度和累计里程应为正，闭环误差才
  能随 PWM 增加而减小。
- 待验证：高电流测试不挂 DAPLink；通过分类后的 `ENC/CTRL/MOTOR` 行检查左右正目标
  时左右速度同号、`CTRL_error_r` 收敛且 `MOTOR_pwm_r` 不再持续卡在 55%。

## 24. 分类遥测、路口门控和右编码器方向版离线验证

- 3519 Debug 编译通过：`text=38088`、`data=4020`、`bss=7108`，主 Flash HEX 为
  42036 bytes，范围 `0x00000000-0x0000A437`。
- 3507 Debug 编译通过：`text=31512`、`data=124`、`bss=6852`，主 Flash HEX 为
  31636 bytes，范围 `0x00000000-0x00007B97`；模板架构检查通过。
- 用用户提供的运行值按新格式计算：`DRIVE+ROUTE=365 bytes`、`DRIVE+IMU=380 bytes`、
  `DRIVE+LINE=367 bytes`；在 115200 8N1 下单周期线速时间分别约 31.7、33.0、31.9 ms，
  均低于 50 ms 周期和 512-byte 单次格式缓冲上限。
- 本节只完成离线编译与带宽计算。遵照高电流测试不连接调试探针的约束，尚未烧录和
  动态验证该版本。

## 25. 把 `name:value` 错当成 FireWater 多通道格式

- 文件证据：当前 `vofa+.csv` 在 2026-07-16 16:27 导出了 1443 行，却只有 `I0`
  一列；I0 范围为 `-93.65..24.86`。这与串口日志中的多组 `time_s:...,run:...` 不匹配，
  证明 VOFA FireWater 没有把冒号字段解析成多通道。
- jixun3 参考证据：`Device/vofa/vofa.c::Vofa_SendCsvFloat()` 逐个输出纯数字、逗号和
  `\r\n`；`Application/ins/ins_task.c` 先用 `Vofa_SendText()` 发一次固定表头，之后只发
  固定列数的数值 CSV。它的 `VOFA_DEBUG_STAGE_*` 通过固定通道顺序分类，而不是在每个
  数值前重复字段名。
- 修改：新增通用 `Device/Communication/Vofa`，提供固定三位小数 CSV、文本表头、发送
  计数和丢帧计数；底层仍调用 `Device_Serial_WriteBuffered()`，因此 3519 TX 继续使用
  UART1 DMA_CH1。Hex25E 删除自制 `name:value` 编码器，改为每 50 ms 一行固定 35 列
  数字 FireWater CSV。
- 分类：I0-I7 SYS、I8-I11 ROUTE、I12-I18 IMU、I19-I22 ENC、I23-I26 CTRL、
  I27-I28 MOTOR、I29-I34 LINE。表头使用相同分类前缀；发送 `STATUS` 会要求下一帧先
  重发表头，但不会启停电机。
- 数值边界：非有限传感器值在对应 online/fault 标志保留的同时输出 `0.000`；极端值
  限幅为可编码数值，不再发送会破坏 FireWater 列的 `nan/ovf` 文本。
- 待验证：VOFA+ 选择 FireWater、115200，重新采集后应固定导出 I0-I34 共 35 列，且
  每行列数恒定。编译、烧录和实测结果另行记录。

## 26. jixun3 风格 VOFA FireWater 完整实现与离线验证

- 新增 `Device/Communication/Vofa/device_vofa.c/.h`：最多 40 通道、512-byte 格式缓冲、
  三位小数纯数字 CSV、`\r\n` 帧尾、文本表头、TX 成功计数和 TX 丢弃计数。该 Device
  不接触 MCU 寄存器，只通过通用 `Device_Serial` 发送。
- Hex25E 使用 35 项分类枚举生成固定列，编译期断言列数不超过 Device 容量；表头和
  数据顺序均为 SYS、ROUTE、IMU、ENC、CTRL、MOTOR、LINE。旧 `time_s:`、
  `DRIVE_time:` 和手写冒号字段编码路径已从当前源码与 ELF 移除。
- 用户实测值按新编码生成 35 列、247 bytes 的数据行；115200 8N1 下线速约 21.4 ms，
  20 Hz 占用约 42.9%。分类表头也是 35 列、363 bytes，只在初始化或 `STATUS` 请求后
  发送，不进入每个周期的带宽计算。
- 3519 Debug 编译通过：`text=37968`、`data=4020`、`bss=7636`，主 Flash HEX 为
  41916 bytes，范围 `0x00000000-0x0000A3BF`。
- 3507 Debug 编译通过：`text=31392`、`data=124`、`bss=7372`，主 Flash HEX 为
  31516 bytes，范围 `0x00000000-0x00007B1F`；模板架构检查通过。
- 动态验证边界：尚未烧录。本版本只在电机高电流关闭、保留主控与 DAPLink 时烧录；
  高电流运行阶段仅用蓝牙 FireWater 采集，不连接调试探针。

## 27. 错误照搬 jixun3，把 VOFA 协议放进 Device 层

- 分层错误：VOFA 是上位机遥测编码和 Hex25E 调试通道定义，不是物理器件；把新实现
  放入 `Device/Communication/Vofa` 重复了此前把产品代码放入 `Device/Car25E` 的同类
  错误。jixun3 的目录位置只能作为协议实现参考，不能照搬其分层。
- 修改：完整删除 `Device/Communication/Vofa` 及其 CMake 目标。CSV 编码、固定缓冲、
  TX/drop 统计和表头发送移入 `Task/Hex25E/task_hex25e_vofa.c/.h`；它通过通用
  `Device_Serial` 使用 UART，不直接包含 BSP、DriverLib 或 MCU 寄存器。
- 责任边界：Device 层只保留串口设备抽象；VOFA 通道顺序和 FireWater CSV 编码都归
  Hex25E 产品任务所有。该条修改时的 35 列方案随后又在第 28 节按最终需求收敛。
- 验证：VOFA 当前唯一实现为 `Task/Hex25E/task_hex25e_vofa.c/.h`；所有 CMake 文件中
  只有 `Task/Hex25E/CMakeLists.txt` 引用该实现，`Device/CMakeLists.txt` 只为 Hex25E
  启用通用 `Communication/Serial`。双目标编译的 Device 单元数均为 7，未再引入 VOFA
  Device 目标。

## 28. VOFA 常驻通道堆砌，无法直接阅读

- 设计错误：最初把所有可取得的遥测量堆成 35 列，随后未经用户指定又改成 18 列、
  8 列；这些方案都要求在 VOFA 的 `I0...In` 与长表之间反复对照，而且错误替用户决定
  了应观察的字段。串口带宽能够容纳不代表界面可读。
- 最终需求：只输出 yaw、左电机目标/实际转速、右电机目标/实际转速、数字灰度 D0-D7、
  USERKEY 当前状态和异常码，共 15 列；其他路线、里程、PWM、欧拉角其他轴、序号和中间
  状态均不进入 VOFA 常驻帧。
- 固定顺序：`I0=IMU.yaw`；`I1/I2=MOTOR.target_l/actual_l`；
  `I3/I4=MOTOR.target_r/actual_r`；`I5-I12=LINE.d0...d7`；
  `I13=KEY.pressed`；`I14=FAULT.code`。D0-D7 使用循迹模块经过 active-low 极性转换后的
  逻辑值，`1` 表示该路检测到线。yaw 无有效帧时发送数值哨兵 `-9999.000`，避免静止零点
  与传感器未在线混淆；异常码 `0` 表示无异常。
- FireWater 数据行仍为纯数字 CSV、50 ms 一帧、三位小数、`\r\n` 结尾；15 列上限由
  编译期断言约束，格式缓冲限定为 256 bytes，防止以后再次无边界增加常驻通道。
- 编译验证：MSPM0G3519 Debug 为 `text=37352`、`data=4020`、`bss=7372`，主 Flash HEX
  41300 bytes、范围 `0x00000000-0x0000A157`；MSPM0G3507 Debug 为 `text=30776`、
  `data=124`、`bss=7116`，主 Flash HEX 30900 bytes、范围
  `0x00000000-0x000078B7`。两目标均编译通过，Device 编译单元均为 7。
- 验证边界：本项只完成源码静态检查与双目标离线编译，尚未烧录；不能据此声称 15 路
  板上数据已经验证。

## 29. 轮速前馈与独立 PID 限幅不匹配，闭环不存在零误差平衡点

- VOFA 证据：`vofa+.csv` 共 878 个数据采样，目标/实际非零区间为采样 484-841。
  目标非零的 339 个采样中左右目标均恒定为 260 mm/s；去掉启动 1 s 后，左实际均值
  362.529 mm/s、右实际均值 349.646 mm/s，分别比目标高 39.4% 和 34.5%。该区间异常码
  始终为 0，因此不是 fault 分支改写目标或反馈。
- 公式错误边界：底层通用 PID 的 `e=r-y`、`I+=e*dt`、`D=de/dt` 和
  `Kp*e+Ki*I+Kd*D` 离散公式没有写错；错误发生在速度环把 `Kff*r` 加到 PID 后，又把
  PID 积分输出独立限制为 ±10%。旧 `Kff=0.15 %/(mm/s)` 在 260 mm/s 时产生 39% 前馈，
  零误差下 P/D 为 0、积分最多只能减 10%，因此最终 PWM 最低仍为 29%，数学上无法在
  当前机械负载下维持 260 mm/s。
- 参数证据：持续负误差约 17 s，旧低速 `Ki=0.30` 的积分在约 0.33 s 内即可达到 -10%
  上限。稳态导数均值近似为 0，据旧公式反算左 PWM 约 18.747%、右 PWM 约 20.035%；
  对应机械稳态增益分别约 19.338 和 17.451 `(mm/s)/%`。两轮平均前馈估计为
  0.0545 `%/(mm/s)`，新初值取 0.055，260 mm/s 对应 14.3% 而不是 39%。
- 重做控制律：速度误差严格为 `smoothed_target-measured_speed`；前馈严格为
  `Kff*smoothed_target`，二者只在同为 PWM 百分比的输出端相加。积分状态直接保存为 PWM
  百分比，不再经过与最终电机输出脱节的 `integral_output_limit_percent` 和
  `correction_limit_percent` 两级限幅；最终命令统一限制在 ±55%。D 项改为对实际速度求导，
  避免目标变化产生微分冲击。
- 抗饱和：先计算候选积分和最终 `feedforward+P+I+D`。若最终输出已经正饱和且误差仍为
  正，或负饱和且误差仍为负，则拒绝本次积分；误差推动输出离开饱和时仍接受积分。
  USERKEY 停车使目标为 0 时无条件清零积分，不受饱和拒绝分支影响；目标换向时也清除旧
  方向的平滑目标和积分，防止旧积分延迟反转。
- 离线边界验证：按上述两组实测机械增益和 100 ms 一阶机械模型、10 ms 控制周期仿真，
  左/右稳态分别为 261.93/261.96 mm/s，稳态 PWM 为 13.54%/15.01%，进入 ±2% 误差带
  分别约 0.85/0.68 s。该模型只验证控制方向和零误差能力，不替代板测。独立边界计算
  确认正负极限输入均钳制为 ±55% 且拒绝继续积分，恢复方向接受积分，停车低于 20 mm/s
  时输出和积分均为 0。
- 编译证据：MSPM0G3519 和 MSPM0G3507 的 `mode_car`、`task_hex25e` 静态库均重新
  编译通过，无编译警告；旧独立积分/修正限幅字段和 `effective_setpoint` 绕行已从当前
  Mode 源码完全移除。
- 整机编译阻断：本项第一次执行 3519 完整 Build 时，`main.c:14` 已存在不属于本次修改的
  `ni'shi    SYSCFG_DL_init();`，编译器报字符串未终止和未知类型 `ni`。此前 VOFA 版本的
  双目标 Build 已在该外部改动出现前通过。为保留用户并行修改，本项没有擅自覆盖
  `main.c`；因此当前只能声明控制器/任务库编译通过，不能声明最终 ELF/HEX 已生成，也尚未
  烧录或板测新速度环。

## 30. CMake 外层报错实际由 `main.c` 非法文本导致

- 用户运行构建后看到 `CMake build failed with exit code 1`；该信息只是
  `tools/mspm0.ps1` 对底层 Ninja/编译器失败的汇总，并不是 CMake 配置故障。
- 首个编译器错误明确位于 `main.c:14`：`ni'shi    SYSCFG_DL_init();` 中的单引号造成
  `missing terminating ' character`，随后产生未知类型 `ni` 等连锁错误。
- 修改：删除非法前缀，恢复为 `    SYSCFG_DL_init();`；没有修改初始化顺序或其他启动
  逻辑。
- 最终完整构建：MSPM0G3519 Debug 通过，`text=37488`、`data=4020`、`bss=7268`，主
  Flash HEX 41436 bytes、范围 `0x00000000-0x0000A1DF`；MSPM0G3507 Debug 通过，
  `text=30912`、`data=124`、`bss=7012`，主 Flash HEX 31036 bytes、范围
  `0x00000000-0x0000793F`。两目标均完成模板配置、全部编译、固件尺寸检查和主 Flash HEX
  校验。
- 验证边界：新闭环固件尚未烧录，当前结果只能证明源码和双目标固件构建成功。

## 31. OpenOCD 首次烧录瞬时失败，重新连接后完成写入与回读校验

- 用户看到的末级信息为 `OpenOCD flash failed with exit code 1`；该行同样只是 PowerShell
  对 OpenOCD 非零退出码的汇总，单独不能判断是 CMSIS-DAP、SWD、目标芯片还是 Flash
  算法失败。
- 非写入检查：`Doctor` 确认本机配置选择的 DAPLink 序列号为
  `000100011a5a5bb600000000000425a1a5a5a5a597969908`，对应 VID/PID
  `0D28:0204`，工具链和自定义 G3519 OpenOCD 文件哈希均通过。
- 直接 OpenOCD attach 成功：CMSIS-DAP v2.1.0 完成初始化，SWD DPIDR 为
  `0x6ba02477`，识别到 Cortex-M0+ r0p1、4 个断点和 2 个观察点，说明探针、当前 USB
  枚举、SWD 通路与目标供电在复查时均正常。该步骤没有写 Flash。
- 随后重试完整 Flash 成功：写入 41440 bytes，用时 5.807 s；从地址 0 回读 41440 bytes，
  回读文件与构建 BIN 的 SHA-256 完全一致；最后再次识别 Cortex-M0+ 并执行 reset run。
  OpenOCD 的 `Adding extra erase range, 0x0000a1e0 .. 0x0000a3ff` 是按擦除页对齐的提示，
  不是失败。
- 结论：本次没有发现可持续复现的脚本、探针选择或 SWD 配置错误；首次非零退出属于探针
  打开阶段的瞬时故障。新轮速闭环的 MSPM0G3519 Debug 固件已实际烧录并通过回读校验，
  但车辆动态轮速仍需断开 DAPLink 后通过蓝牙 VOFA 实测。

## 32. 一键轮速阶梯测试与 Kp 首轮提高

- 需求：轮速多点测试不再要求手工发命令或逐档操作；用户只按一次 USERKEY 启动，第二次
  USERKEY 必须仍然立即停车。
- 修改：当前验证固件第一次按键后进入 `TASK_HEX25E_STATE_SPEED_TEST`，左右轮目标自动按
  85、160、220、260 mm/s 依次运行，每档 3 s。260 mm/s 档持续保持，不会由测试逻辑
  自动停车；只有第二次 USERKEY 进入原有立即制动和锁电机路径。测试期间不使用灰度结果
  改写轮速目标，现有 VOFA `target_l/actual_l/target_r/actual_r` 可直接按目标值分段。
- Kp 修改：低速段从 0.10 提高到 0.15，高速段从 0.08 提高到 0.12；Ki、Kd、0.055
  前馈和 ±55% 最终输出限制不变。260 mm/s 从静止时的理论前馈加比例项约为 53.3%，仍在
  输出上限内；若触及边界则由第 29 节的最终输出抗积分饱和处理。
- 离线模型：使用第 29 节从实测数据反推的左右机械增益，四档最后 1 s 的速度均值约为
  目标值 +1.85 至 +1.95 mm/s；最大预测峰值分别不超过 88.79、163.66、222.99、
  262.54 mm/s。该结果仅用于检查增益方向和明显失稳，不代替板上测试。
- 双目标完整构建通过：MSPM0G3519 Debug 为 `text=37592`、`data=4020`、`bss=7268`，
  主 Flash HEX 41540 bytes、范围 `0x00000000-0x0000A247`；MSPM0G3507 Debug 为
  `text=31016`、`data=124`、`bss=7012`，主 Flash HEX 31140 bytes、范围
  `0x00000000-0x000079A7`。
- 本版烧录边界：烧录开始时 OpenOCD 返回 `unable to find a matching CMSIS-DAP device`；
  随后重新运行 Doctor 已明确报告所选 DAPLink 序列号不在 USB 枚举中。因此本节自动测试
  固件尚未写入目标板，上一节已通过回读的固件仍是没有自动阶梯测试、Kp 为 0.10/0.08
  的版本。必须重新连接 DAPLink 后才能烧录本节版本。

## 33. 未先板测轮速内环就提前接入角度外环，已完整回退

- 顺序错误：串级控制必须先证明编码器反馈、轮速 PID、PWM 输出组成的内环在多个目标速度
  下方向正确、稳定并能收敛，之后角度外环才能把左右轮速度作为控制量。尚未烧录并取得第
  32 节阶梯测试数据时，曾提前新增角度控制器和 60°状态机，无法区分后续误差来自内环还是
  外环。
- 回退：完整删除尚未板测的 `mode_car_angle_controller.c`、头文件 API、CMake 编译单元、
  IMU 到 Hex25E 控制输入和角度测试状态；恢复第 32 节的一键轮速阶梯测试。当前源码中搜索
  不再存在 `AngleController`、`ANGLE_TEST`、`angle_target_yaw` 或
  `Task_Hex25E_Attitude` 活跃路径。
- 当前内环链路：阶梯状态机只产生左右目标速度；`apply_wheel_targets()` 把目标值和 QEI
  计算的左右实际速度送入 `Mode_Car_SpeedController_Update()`；该内环执行目标平滑、
  `Kff*target`、`Kp*(target-actual)`、积分、实际速度微分和最终 ±55% 抗饱和，再转换为
  `Device_Motor_SetCommand()`。角度或灰度数据不参与本次目标生成。
- 保留参数：低速 Kp=0.15、高速 Kp=0.12，Ki/Kd、Kff=0.055、2 mm/s 死区和 ±55%
  输出上限保持第 32 节版本；KEY 第一次依次执行 85/160/220/260 mm/s，每档 3 s，260
  持续保持，第二次 KEY 立即停车。
- 回退后双目标完整编译重新通过：3519 Debug 为 `text=37592`、`data=4020`、`bss=7268`，
  HEX 41540 bytes、范围 `0x00000000-0x0000A247`；3507 Debug 为 `text=31016`、
  `data=124`、`bss=7012`，HEX 31140 bytes、范围 `0x00000000-0x000079A7`。
- 内环通过条件：重新连接 DAPLink 烧录后，断开探针进行高电流测试；VOFA 每个 3 s 档取
  最后 1 s，检查左右实际速度同号、稳态误差、峰值超调、左右差异、KEY 停车响应和
  `FAULT.code=0`。这些实测条件通过前不继续实现角度外环。

## 34. 读取已导入 VOFA 验证轮速内环，并正式实现角度外环

- 数据来源：用户已更新根目录 `vofa+.csv`，最后修改时间 2026-07-16 19:58:52，共 488
  个数据采样。目标分段与第 32 节固件完全匹配：0 速 56 帧、85/160/220 mm/s 各 60 帧、
  260 mm/s 178 帧，最后停车 74 帧；证明一键轮速阶梯版本已经实际运行，不应再次要求用户
  重复导入或重新验证。
- 内环稳态结果：各档最后 1 s 左/右实际均值分别为：85 档 85.580/84.287，160 档
  160.039/160.298，220 档 220.867/220.516，260 档 260.359/259.768 mm/s。所有稳态
  误差绝对值均小于 0.85%。
- 内环动态结果：左右最大超调在 85 档为 6.48%/4.31%，160 档均为 3.90%，220 档为
  2.94%/1.59%，260 档为 2.02%/2.45%。85 档两轮进入并连续保持 ±10% 误差带耗时约
  700 ms，后续升速阶跃均在首个或第二个 50 ms VOFA 样本进入该误差带。第二次 KEY 后，
  左轮约 100 ms、右轮约 150 ms 归零；异常码 488 帧全部为 0。
- 判定：编码器速度方向、左右闭环、Kp=0.15/0.12、Kff=0.055、积分抗饱和和 KEY 停车
  均达到角度外环接入条件。该判定直接基于用户现有 CSV，而不是离线模型。
- 串级结构：新增 Car25E 角度外环，输入为目标 yaw、JY61P 实际 yaw 和 Z 轴角速度；
  输出只生成差速轮速目标 `left=-turn_speed`、`right=+turn_speed`。这些目标继续进入已经
  验证的 `apply_wheel_targets()` 和轮速 PID 内环，角度外环不直接输出 PWM，也不绕过
  编码器反馈。
- 外环参数：`Kp=3.0 (mm/s)/deg`、`Ki=0.05 (mm/s)/(deg*s)`、
  `Kd=1.0 (mm/s)/(deg/s)`，积分轮速限制 ±10 mm/s，差速轮速限制 ±160 mm/s。角度误差
  使用 `Algorithm_Control_AngleErrorDeg()` 归一化到 ±180°，D 项直接使用实测 yaw rate，
  避免跨 ±180°时对角度作差分产生尖峰；最终轮速饱和方向采用候选积分拒绝抗饱和。
- 自动测试：第一次 USERKEY 捕获当前有效 yaw，目标加 60°；角差 ≤0.5°且绝对 yaw rate
  ≤3°/s 连续 200 ms 后，轮速目标归零并制动 1 s；随后目标再加 60°，跨 ±180°自动换算，
  循环执行。第二次 USERKEY 仍走原有立即停车和锁电机路径。IMU 不在线或缺少 yaw/rate
  有效帧时进入 sensor fault，不会盲目转动。
- 双目标完整编译：3519 Debug 为 `text=39592`、`data=4020`、`bss=7316`，HEX
  43540 bytes、范围 `0x00000000-0x0000AA17`；3507 Debug 为 `text=33016`、
  `data=124`、`bss=7060`，HEX 33140 bytes、范围 `0x00000000-0x00008177`。
- 烧录边界：编译后 Doctor 仍确认目标 DAPLink 序列号不在 USB 枚举中，因此本节角度环
  固件尚未写入 3519。必须在电机高电流关闭时重新连接 DAPLink，烧录并回读通过后，再断开
  探针用 VOFA 验证每个 60°平台和 1 s 停留。

## 35. 当日聊天记录错误次数统计与 `launch.json` 回退错误

- 统计口径修正：用户要求统计的是今天聊天记录中实际发生并被指出的独立错误，不是只统计
  process 中已经建立编号的源码/配置错误。此前按 process 条目作答为 26 次，答非所问，
  该统计方式和回答本身都必须修正。
- process 中计入错误的既有编号为：1、2、3、4、5、6、7、8、9、11、12、13、14、15、
  17、18、20、21、22、23、25、27、28、29、33，共 25 次。
- 第 26 次为 `launch.json` 错误：没有任何用户要求恢复旧入口，却保留了独立 Build、Flash、
  SystemConfig、Debug 等旧配置，只给 3519 增加组合入口且没有给 3507 提供相同组合；这使
  已明确要求的“每个设备只保留 Build + Flash + Debug”再次失效。该问题不是 CMake、
  Cortex-Debug 或生成脚本自动改写，而是没有把用户最终 launch 约束作为不可回退条件。
- process 的 25 个既有错误加 `launch.json` 回退错误为 26 次，但聊天记录还漏计两次：
  1. 用户已经把新版 `vofa+.csv` 导入工作区，仍没有先读取数据，却再次要求用户连接探针、
     烧录和重复验证轮速内环；直到用户再次明确要求“读数据”后才读取现有 CSV。
  2. 用户询问“聊天记录里今天犯错几次”时，又错误地按 process 编号统计并回答 26 次，
     没有按聊天记录口径回答。
- 因此截至用户明确纠正“我说聊天记录”之后，当日聊天记录中的独立错误总数为
  **28 次**。后续新增独立错误必须继续累加，不得用 process 未编号作为漏计理由。
- 当前修正：`.vscode/launch.json` 只保留
  `MSPM0G3519: Build + Flash + Debug` 与
  `MSPM0G3507: Build + Flash + Debug` 两个配置；两者分别调用对应 Flash task，Flash task
  内部执行 Build 和烧录，Cortex-Debug 只加载已生成 ELF 并连接，不重复下载。

## 36. VS Code tasks 未与精简后的 launch 完全同步

- 残留问题：虽然 `launch.json` 已缩减为两个组合入口，但其 `preLaunchTask` 仍引用名为
  `Flash` 的旧任务，`.vscode/tasks.json` 仍保留独立 Build、Flash 和 SystemConfig 六个
  任务；从 VS Code 工作流看仍不是用户要求的纯粹 `Build + Flash + Debug` 两条链路。
- 修改：`tasks.json` 同步收敛为两个任务：`MSPM0G3519: Build + Flash` 和
  `MSPM0G3507: Build + Flash`。两者调用 `mspm0.ps1 -Action Flash`，而 Flash 动作内部先
  Build、再写入和回读；对应 launch 配置在任务成功后启动 Cortex-Debug，`loadFiles=[]`
  防止调试阶段重复烧录。
- 验证：launch 配置数为 2、task 数为 2，两个 `preLaunchTask` 均存在，未出现 jixun、25E、
  SystemConfig 或独立 Build/Flash/Debug 入口。`tools/check_template.ps1` 完整通过。
