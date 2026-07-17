# 按键设备

`Device_Key` 将一个原始有效状态布尔值转换为消抖后的短按、长按和双击事件。它不依赖 GPIO、BSP、MCU、中断或 FreeRTOS。开发板或 Product 调用自身的 `BSP_Key_Read()`，然后将读取结果和单调递增的毫秒计数传给 `Device_Key_Update()`。

第一次短按会暂存到 `double_click_ms` 超时。在该窗口结束时刻之前或恰好结束时开始的第二次消抖按下会接管第一次点击；第二次正常释放时产生一个 `DEVICE_KEY_EVENT_DOUBLE_CLICK`。如果第二次按下转成长按，队列会依次产生第一次短按事件和长按事件，确保第一次点击不会丢失。每次持续按住只产生一次长按事件。

事件类型由消抖后的按下时长决定。恰好在 `long_press_ms` 时释放仍判定为长按，因此分类结果不依赖调用方是否正好在释放边沿之前执行过一次更新。

每个实例只能在单一上下文中使用：更新和事件读取应位于同一个主循环或 FreeRTOS 任务中，否则调用方必须自行串行化。不得从 GPIO ISR 调用该状态机。

所有时间戳和经过时间计算均使用 `uint32_t`；普通无符号减法可使阈值判断跨越定时器回绕正常工作。调用方仍需以足够高的频率采样，确保观察到每次物理跳变，并且不得让实例在一个完整计数周期内始终得不到处理。

使用 GCC 运行主机测试：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File .\Device\Input\Key\tests\run_gcc_tests.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File .\Device\Input\Key\tests\run_arm_compile.ps1
```
