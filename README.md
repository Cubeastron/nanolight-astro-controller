# NanoLight


NanoLight 是一个面向天文摄影和电动对焦的 STM32F103C8T6 固件工程。工程由 STM32CubeMX 生成基础 HAL/USB 代码，应用层位于 `Core/`：

| 模块 | 职责 |
| --- | --- |
| `app_config.c` | 默认参数、Flash 配置记录、速度表 |
| `focuser_motor.c` | STEP/DIR 电机、赤道仪跟踪、转台快门、手动对焦 |
| `focuser_protocol.c` | RoboFocus/SharpCap 风格 USB 9 字节协议 |
| `focuser_ui.c` | 按键菜单、OLED 页面、电源状态和参数保存 |
| `key.c` | SysTick 驱动的非阻塞按键扫描、消抖和事件状态机 |
| `oled.c` | SSD1306 I2C 显示驱动 |

## 硬件与时序

- MCU：`STM32F103C8T6`，系统时钟 72 MHz。
- 时钟：外部 HSE 经过 PLL 倍频到 72 MHz，同时生成 USB FS 所需的 48 MHz。
- 显示：SSD1306 128×64，I2C1，地址 `0x78`（8 位地址写法）。
- 电机：TIM3 中断驱动 STEP，计数器频率为 1 MHz，因此周期参数单位为微秒。
- 按键：SysTick 提供 1 ms 时基，每 20 ms 扫描一次全部 7 个低电平有效按键；中断中只读取 GPIO 和记录事件。
- USB：USB FS **Dual CDC**。端口 0 是对焦协议，接收回调只负责缓存字节，协议解析在主循环 `RFCP_Process()` 中执行；端口 1 是串口快门接口，不解析数据，只使用 CDC 控制线状态驱动相机输出。
- Flash 参数区：`0x0800FC00`，占用最后 1 KiB 页；记录包含 magic、版本、校验和及 9 个参数。

电机驱动的 `EN` 为低电平有效；固件启动后会保持低电平使能驱动器，`STEP` 初始为低。`DIR` 输出由逻辑方向和已保存的电机反转设置共同决定。

### 引脚

| 信号 | 引脚 | 方向/用途 |
| --- | --- | --- |
| `FOCUS` / `SHUTTER` | PC15 / PC14 | 相机对焦、快门输出 |
| `STEP` / `DIR` / `EN` | PB8 / PB9 / PB5 | 步进驱动器 |
| `SCL` / `SDA` | PB6 / PB7 | OLED I2C1 |
| `UP` / `OK` / `LEFT` | PB4 / PB3 / PB15 | 按键输入，上拉，低电平有效 |
| `SF` / `RIGHT` / `DOWN` / `NS` | PA8 / PA9 / PA10 / PA15 | 按键输入，上拉，低电平有效 |

按键采用非阻塞事件处理。`UP`、`DOWN`、`NS` 和 `SF` 每次按下执行一次；参数页面中的 `LEFT`、`RIGHT` 按住后仍按约 20 ms 周期连续调整，电机位置页中按住则连续移动电机。OK 的普通操作在释放后执行，以便和长按区分。

在任意页面按住 `OK` 满 1 秒会关闭 OLED，但当前曝光、转台、电机、USB 和计时逻辑继续运行。OLED 关闭后，任意一个按键的新下降沿都会恢复显示并重绘当前页面；用于唤醒的这次按压会被完整忽略，释放后再次按键才会执行操作。

### USB CDC 端口

| 端口 | 用途 | 行为 |
| --- | --- | --- |
| CDC 0 | RoboFocus/SharpCap 对焦协议 | 接收 9 字节帧，发送位置、版本和移动完成回包 |
| CDC 1 | 串口快门（Serial Bulb） | 当主机 DTR 或 RTS 任一有效时，同时将 `FOCUS`、`SHUTTER` 拉高；两者都无效时拉低 |

串口快门处于激活状态时，固件会阻止尚未开始的内部快门/延时动作，并屏蔽运行中由内部逻辑发出的相机 GPIO 写入；动作计时和电机循环不会因此暂停。CDC 1 的普通 OUT 数据不会转发到应用层。

## 构建

需要 CMake 3.22、Ninja 和 `arm-none-eabi-gcc`。工程已经提供 GCC 工具链文件和预设：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

Release 构建：

```powershell
cmake --preset Release
cmake --build --preset Release
```

输出文件位于 `build/Debug/` 或 `build/Release/`。本仓库未包含烧录器配置，烧录时请按实际探头和使用的工具链选择对应流程。

## 应用初始化与主循环

`main()` 的初始化顺序是：HAL/时钟、GPIO、TIM3、USB、I2C、参数加载、协议、步进电机、OLED。前台循环依次调用：

1. `RFCP_Process()`：解析 CDC 0 命令并发送响应。
2. `UI_Turn()`：消费 SysTick 产生的按键事件，处理 OLED 休眠/唤醒、页面切换和参数修改。
3. `Set_Period()`：处理方向/速度按键。
4. `Shutter_EQ_Run()`、`Shutter_Gimbal_Run()`：运行快门动作；如果 CDC 1 串口快门有效，则内部动作暂不执行。
5. `Focuser_Run()`：运行手动对焦页面。

SysTick 每 1 ms 调用 `Key_Tick()`，实际按键 GPIO 每 20 ms 采样一次；TIM3 的 `HAL_TIM_PeriodElapsedCallback()` 负责 STEP 翻转。USB 移动完成后由协议模块异步返回当前位置。

## USB 协议速查

命令为 `F` + 命令字 + 6 位 ASCII 数据 + 校验和：

- `FV000000`：版本查询，返回 `FV000001`。
- `FG000000`：位置查询，返回 `FDxxxxxx`。
- `FO000160` / `FI000160`：向外/向内移动指定步数，完成后返回位置；运动中的新移动命令会覆盖当前目标步数。
- `FVXXXXXX` 或 `FS000000`：停止运动并返回版本帧。

校验和是前 8 字节的累加和低 8 位。完整的用户操作说明见 [使用说明书.md](使用说明书.md)。

## 速度和计数规则

- 赤道仪四档显示为 `*`、`Moon`、`100*`、`MAX`。前两档分别使用恒星时周期和约 `1.03788` 倍恒星时周期；`100*` 使用约百分之一周期，但不会低于 `MAX_us`；`MAX` 直接使用 `MAX_us`。
- 对焦四档显示为 `1/100`、`1/50`、`1/10`、`MAX`，对应周期分别为 `100*MAX_us`、`50*MAX_us`、`10*MAX_us` 和 `MAX_us`。
- `Motor_position` 只在手动对焦或 USB 对焦移动的 STEP 上升沿更新。赤道仪跟踪和转台旋转不会改变该位置。
- 手动点动页面中，LEFT 驱动原 RIGHT 的方向，RIGHT 驱动原 LEFT 的方向；两键同时按下仍由 LEFT 优先。
- 赤道仪模式页面的方向显示为“南/北”；延时模式页面的方向显示为“顺/逆”，其中北对应“逆”、南对应“顺”。
- 电机反转仅翻转 `DIR` 的物理输出；菜单方向、手动对焦箭头、USB `FO/FI` 及位置增减规则保持不变。
- 当前 `focustime` 编译常量为 `0`，因此快门流程没有额外的对焦预置延迟。

## 修改注意事项

- `nanolight.ioc` 是 CubeMX 配置源；不要手工修改 `build/` 中的生成文件。
- 修改 Flash 记录布局时必须递增 `FLASH_EEPROM_VERSION`，并同步更新默认值和说明书。
- 当前 Flash 记录版本为 2；读取合法的版本 1 记录时会保留原有参数，将电机反转设为 `OFF` 并自动升级记录。
- `Motor_position` 在 STEP 上升沿更新；不要在其他模块重复递增位置。
- USB 回调和 TIM3 中断共享状态时，保持 ISR 简短，避免在中断中执行 Flash 操作。
- SysTick 按键扫描只允许读取 GPIO、更新时间和设置事件标志；OLED、I2C、Flash、菜单及电机操作必须留在前台执行。
- 赤道仪、延时和转台连续旋转只翻转 STEP，不更新对焦位置计数；`Motor_position` 只用于对焦电机移动。
- `EEPROM_Save()` 会擦除并重写整页，UI 当前未检查其返回值，界面提示不等同于 Flash 写入已被硬件确认。
- `Error_Handler()` 会关闭 `FOCUS`/`SHUTTER` 输出并关闭全局中断，但不会自动拉高 `EN` 禁用电机驱动器。
- `EEPROM_IsValid()`、`focuser_position`、`extime_fc` 等符号目前是遗留接口或调试变量，不构成独立用户功能；`focuser_ui.h` 中声明的 `fun_c23()` 在当前实现中也没有对应函数体和菜单表项。
