# ChassisCommand

从 `jixun3/Application/gimbal/chassis_serial_cmd.*` 迁移的串口行命令解析器。
旧实现解析后直接修改 Task/Mode 全局变量；本实现只返回强类型事件，由拥有串口的
FreeRTOS Task 串行应用。

支持 stand/forward/backward/turn/tracking/square/target、task1..task5、普通/视觉
瞄准、校准激光、平衡偏置、PID/限幅/斜率参数和 `-300..300 RPM` 轮速命令。
显式 mode 前缀解析失败时不会退化成数值命令，超范围数值也会拒绝。
