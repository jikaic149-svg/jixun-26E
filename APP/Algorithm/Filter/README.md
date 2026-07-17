# APP/Algorithm/Filter

硬件无关的低通、调用方存储的滑动平均、三点中值、Biquad、角度 Kalman 和标量
Kalman，不使用动态内存、RTOS 或全局实例。

Biquad 与两种 Kalman 从 `jixun3/Application/algorithm/attitude_algorithm.*` 迁移。
旧文件中的全局 Mahony 实现不重复保留，统一使用 `Algorithm/Attitude` 的显式
Context/Input/Output 版本。

所有 Update 都先在局部变量中完成验证，只有输出有限时才提交内部状态。滑动平均还会验证 `count/next/capacity/sum` 不变量，求和溢出不会部分增加计数或覆盖窗口。

测试：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\APP\Algorithm\Filter\tests\run_gcc_tests.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File .\APP\Algorithm\Filter\tests\run_arm_compile.ps1
```
