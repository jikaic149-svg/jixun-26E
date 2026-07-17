# ADRC / ESO

从 `jixun3/Application/algorithm/adrc_eso.*` 迁移的一阶扩张状态观测器和 PI-ESO
控制器。算法只处理显式数值输入，不访问硬件或调度器。

PI-ESO 保留风险降权、积分开关、扰动补偿、输出饱和和抗饱和回算能力。
