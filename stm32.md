APB1 与 APB2 的核心区别： 二者是 STM32 内部总线架构（AMBA）的两条外设总线，主要差异如下：
特性	APB1 (低速)	APB2 (高速)
时钟频率	≤ 36 MHz	≤ 72 MHz
连接外设	低速外设：TIM2-4、USART2-3、SPI2-3	高速/核心外设：GPIOA-G、ADC1-3、TIM1、SPI1
时钟使能函数	RCC_APB1PeriphClockCmd()	RCC_APB2PeriphClockCmd()
典型外设归属举例：
● APB1：
  ○ 定时器：TIM2, TIM3, TIM4
  ○ 串口：USART2, USART3
  ○ 低功耗模块：DAC, CAN
● APB2：
  ○ 所有 GPIO（PA-PG）
  ○ 高级定时器：TIM1
  ○ 高速 ADC、SPI1
  ○ 系统控制（AFIO、EXTI）
设计逻辑：
1. 性能分层：高速外设（如GPIO、TIM1）需高带宽，故挂载到 APB2；低速外设（如普通定时器）节省功耗，用 APB1。
2. 简化资源分配：避免总线拥堵，将外设按速率分组管理。
编程注意：
● 必须根据外设所属总线开启对应时钟，否则无法操作寄存器（如不开启 APB2 的 GPIOA 时钟，写 GPIOA 寄存器无效）。

设计逻辑与选型原则
以下从输入、输出、复用模式的具体场景、区别及选型关键点展开说明：
1. 输入模式
核心逻辑：根据信号来源的稳定性和类型选择输入模式，确保正确捕获电平状态。
模式	适用场景	区别与注意事项
浮空输入
(GPIO_Mode_IN_FLOATING)	1. 外部信号已自带强驱动（如数字传感器输出）2. 高速通信接收端（如USART_RX）	- 引脚电平完全由外部决定，无内部上拉/下拉- 风险：若信号线悬空，可能因干扰导致误触发
上拉输入
(GPIO_Mode_IPU)	1. 按键检测（按键接地，按下时拉低电平）2. 默认需高电平的输入信号	- 内部上拉电阻约40kΩ，默认高电平- 省硬件：无需外接上拉电阻，简化电路设计
下拉输入
(GPIO_Mode_IPD)	1. 按键检测（按键接电源，按下时拉高电平）2. 默认需低电平的输入信号	- 内部下拉电阻约40kΩ，默认低电平- 防干扰：避免悬空引脚电平漂移
模拟输入
(GPIO_Mode_AIN)	1. ADC/DAC采集模拟信号（如温度、光照传感器）2. 高精度测量场景	- 禁用数字电路，直接连接模拟通道- 禁止：不可用于数字信号（如按键、通信引脚）
选型关键：
● 信号驱动能力：外部信号弱（如按键）→ 上拉/下拉输入；信号强（如数字传感器）→ 浮空输入。
● 抗干扰需求：环境噪声大 → 上拉/下拉输入强制默认电平。
● 信号类型：模拟信号 → 必须用模拟输入。
2. 输出模式
核心逻辑：根据负载类型和通信需求选择推挽或开漏模式，确保可靠驱动和兼容性。
模式	适用场景	区别与注意事项
推挽输出
(GPIO_Mode_Out_PP)	1. 驱动LED、蜂鸣器、继电器等小功率负载2. 高速数字信号（如PWM、SPI_CLK）	- 可主动输出高/低电平，驱动能力强（±20mA）- 优势：电平稳定，抗干扰强，切换速度快
开漏输出
(GPIO_Mode_Out_OD)	1. I2C、SMBus等总线通信2. 电平转换（3.3V ↔ 5V）3. 多设备共享总线	- 只能拉低电平，高电平需外接上拉电阻（如4.7kΩ）- 优势：支持“线与”逻辑，避免总线冲突
选型关键：
● 驱动能力：需强驱动（如LED）→ 推挽输出；仅需逻辑控制（如I2C）→ 开漏输出。
● 总线类型：共享总线（多设备）→ 必须开漏输出 + 外接上拉。
● 电平兼容：跨电压通信（如3.3V与5V）→ 开漏输出 + 外部上拉到目标电压。
示例对比：
● LED控制：
  ○ 正确：推挽输出 → 直接输出高/低电平驱动。
  ○ 错误：开漏输出未接上拉 → LED无法点亮（高电平为浮空）。
● I2C总线：
  ○ 正确：开漏输出 + 4.7kΩ上拉 → 多设备共享总线。
  ○ 错误：推挽输出 → 设备冲突（无法实现线与逻辑）。
3. 复用模式
核心逻辑：由外设功能决定模式，外设需直接控制引脚时序（如PWM、SPI）。
模式	适用场景	区别与注意事项
复用推挽输出
(GPIO_Mode_AF_PP)	1. 定时器PWM输出（TIMx_CHx）2. 高速通信发送端（如SPI_MOSI、USART_TX）	- 外设自动控制引脚电平，推挽驱动- 优势：高速切换（最高50MHz），适合精确时序控制
复用开漏输出
(GPIO_Mode_AF_OD)	1. I2C的SCL/SDA引脚2. CAN总线通信	- 外设控制开漏输出，需外接上拉电阻- 优势：支持多设备共享总线，避免电平冲突
选型关键：
● 外设协议要求：
  ○ I2C、CAN → 必须复用开漏（协议强制要求线与逻辑）。
  ○ SPI、USART → 通常复用推挽（高速、单向信号）。
● 外设功能：
  ○ PWM输出 → 复用推挽（需强驱动能力）。
  ○ 总线通信 → 根据协议选择推挽或开漏。
示例对比：
● SPI通信：
  ○ 正确：复用推挽 → MOSI、SCK引脚高速输出。
  ○ 错误：复用开漏 → 信号上升沿变慢，通信速率受限。
● I2C通信：
  ○ 正确：复用开漏 + 上拉电阻 → 多主机共享总线。
  ○ 错误：复用推挽 → 总线冲突（无法实现线与）。
总结
1. 输入模式：
  ○ 看信号类型（数字/模拟）、驱动能力、抗干扰需求。
  ○ 默认悬空风险大：优先上拉/下拉输入，除非信号源明确。
2. 输出模式：
  ○ 推挽：强驱动、高速场景；开漏：总线共享、电平转换。
  ○ 开漏必须外接上拉：否则高电平无效。
3. 复用模式：
  ○ 完全由外设功能决定，参考芯片手册的“Alternate Function”表。
  ○ 复用推挽：外设需强驱动或高速信号（如PWM、SPI）。
  ○ 复用开漏：协议要求线与逻辑（如I2C、CAN）。
最终口诀：
● 输入看信号，输出看负载，复用看外设。
● 推挽强驱动，开漏总线用，悬空要谨慎。
按键抖动如何处理？
● 硬件方案：并联电容滤除抖动（如0.1μF）。
● 软件方案：检测到低电平后延时10-20ms再确认状态。

注意（扩展）：
定时器家族分类（F1系列为例）
类型	型号	特点	适用场景
高级定时器	TIM1, TIM8	带死区控制，支持互补输出	电机控制、电源管理
通用定时器	TIM2-TIM5	16/32位，支持编码器接口	PWM生成、输入捕获
基本定时器	TIM6, TIM7	仅支持向上计数，无捕获/比较通道	基础定时、DMA触发

电路连接
● 按键一端接地，另一端接GPIO引脚（如PA0）。
● GPIO配置为上拉输入模式（内部上拉电阻约40kΩ）。
电平逻辑
● 按键未按下时： GPIO引脚通过内部上拉电阻连接到VDD（3.3V），读取电平为高电平（1）。
● 按键按下时： 按键导通，GPIO引脚直接接地（GND），电平被强制拉低至低电平（0）。
 线与逻辑（Wired-AND）
定义：多个输出端通过开漏（或集电极开路）模式连接到同一总线，任一设备拉低总线电平即可使整个总线为低电平，而所有设备均不拉低时总线由外部上拉电阻保持高电平。
核心原理：
● 硬件要求：所有设备配置为开漏输出，总线需外接一个上拉电阻（如4.7kΩ）。
● 逻辑行为：
  ○ 任一设备输出低电平 → 总线低电平（逻辑0）。
  ○ 所有设备输出高电平 → 总线高电平（逻辑1）。
典型应用：
● I2C总线：多设备共享SCL/SDA线，任一设备可主动拉低总线，实现仲裁与协同。
● CAN总线：显性电平（低电平）覆盖隐性电平（高电平），确保通信优先级。

信号源明确：外部电路（如传感器、通信模块）已确保引脚电平稳定。
  ○ 例：数字温度传感器输出引脚已内置推挽驱动，可直接连接STM32浮空输入。
信号源不明确：引脚可能悬空或仅通过高阻抗电路连接（如未接按键的GPIO）。
  ○ 此时必须配置上拉/下拉输入，否则引脚电平易受干扰（噪声）影响。

噪声（Noise）与误触发
噪声定义：电路中因电磁干扰、电源波动或信号耦合产生的非预期电压波动。
误触发动机理：
● 浮空输入引脚悬空：
  ○ 引脚等效为高阻抗天线，易受环境电磁噪声干扰。
  ○ 噪声可能导致引脚电平在高低之间随机跳变。
● 解决方案：
1. 硬件抗噪：
  ○ 配置上拉/下拉输入，强制默认电平。
  ○ 添加滤波电容（如0.1μF）到信号线，吸收高频噪声。
2. 软件抗噪：
  ○ 多次采样取平均值（适用于ADC）。
  ○ 检测到电平变化后延时再确认（如按键消抖）。
          强驱动信号用浮空，弱信号用上拉/下拉

（一次不要看太多哦，小心脑袋爆炸，明天再看发现瞬间理解了！）

复用开漏模式（AF_OD）与普通开漏模式（Out_OD）的区别
二者硬件结构相同（均只能拉低电平，高电平需外接上拉），但控制权和应用场景不同：
特性	普通开漏模式（Out_OD）	复用开漏模式（AF_OD）
控制权	CPU通过代码直接控制引脚电平（如GPIO_WriteBit）	外设硬件自动控制引脚（如I2C、CAN模块）
信号来源	用户软件（手动操作寄存器）	外设控制器（如I2C的SCL/SDA信号由I2C模块生成）
典型应用场景	软件模拟协议（如GPIO模拟I2C）	硬件外设协议（如I2C、CAN总线）
总线冲突风险	需软件保证多设备协调	硬件自动处理（如I2C仲裁、CAN显性优先级）
配置代码差异	GPIO_Mode_Out_OD	GPIO_Mode_AF_OD
 + 复用功能映射（GPIO_PinAFConfig）
总结
● 复用开漏模式：外设硬件控制引脚，用于硬件协议（I2C/CAN）。
● 普通开漏模式：软件控制引脚，用于通用输出或模拟协议。
● 核心口诀：
  ○ 外设协议 → 复用开漏 + 复用功能映射。
  ○ 软件控制 → 普通开漏 + 手动操作电平。
推挽模式（Out_PP）与复用推挽模式（AF_PP）的区别
二者硬件结构相同（均可主动输出高/低电平），但控制权、应用场景及配置方式不同：
特性	普通推挽模式（Out_PP）	复用推挽模式（AF_PP）
控制权	CPU通过代码直接控制引脚电平（如GPIO_WriteBit
）	外设硬件自动控制引脚（如SPI、USART、定时器模块）
信号来源	用户软件（手动操作寄存器）	外设控制器（如SPI_MOSI信号由SPI模块生成）
典型应用场景	驱动LED、蜂鸣器等需强电平驱动的设备	硬件外设协议（如SPI、USART、PWM输出）
时序控制	依赖软件延时或中断	硬件自动生成精确时序（如SPI时钟、PWM波形）
配置代码差异	GPIO_Mode_Out_PP	GPIO_Mode_AF_PP
 + 复用功能映射（GPIO_PinAFConfig
）
总结
● 复用推挽模式：外设硬件控制引脚，用于需精确时序的协议（SPI/USART/PWM）。
● 普通推挽模式：软件控制引脚，用于通用强电平驱动（LED、蜂鸣器）。
● 核心口诀：
  ○ 外设协议 → 复用推挽 + 复用功能映射。
  ○ 软件控制 → 普通推挽 + 手动操作电平。

扩展：
在STM32中，GPIO速度（GPIO_Speed）指输出驱动电路的压摆率（Slew Rate），即引脚电平从低到高（或高到低）的切换速度。
选型原则
1. 按信号频率选择：
  ○ 信号频率 > 5MHz → 高速模式（50MHz）。
  ○ 信号频率 < 1MHz → 低速模式（2/10MHz）。
2. 按负载电容选择： 负载电容大（>30pF） → 优先高速模式。
3. 权衡功耗与性能： 低功耗设备（如电池供电）尽量用低速模式。

函数功能篇
GPIO_Write(GPIOA, 数值) 是STM32标准库中的一个函数，它的核心功能是直接向GPIOA端口的所有引脚一次性写入电平状态。
1. 函数作用
● 功能：将 数值 直接写入 GPIOA 的 ODR（Output Data Register，输出数据寄存器），一次性设置所有16个引脚（PA0-PA15）的电平。
● 关键特性：
  ○ 一次性写入：通过一个16位的数值，同时控制所有引脚的电平（高/低）。
  ○ 硬件映射：数值的每一位对应一个物理引脚（位0 → PA0，位1 → PA1，…，位15 → PA15）。
2. 数值的二进制含义
● 二进制位规则：
  ○ 1 → 对应引脚输出高电平（3.3V）。
  ○ 0 → 对应引脚输出低电平（0V）。
扩展：
● ODR 的作用： 这是一个16位的寄存器，每一位控制一个引脚的电平状态：
  ○ 位值为1 → 引脚输出高电平（3.3V）
  ○ 位值为0 → 引脚输出低电平（0V）
● 二进制位与引脚关系：ODR寄存器的每一位对应一个引脚，位索引即引脚编号（PA0-PA15）。
   比如：原始值 0x0008 对应二进制 0000 0000 0000 1000（PA3为1）
ODR（输出数据寄存器）的作用
● 功能：ODR是一个16位寄存器，直接控制GPIO引脚的输出电平。
  ○ 写入1 → 引脚输出高电平（3.3V）
  ○ 写入0 → 引脚输出低电平（0V）
   ODR的作用：通过写入16位值，一次性控制所有引脚的电平状态。
按位取反（~）
● 原始值 0x0008 对应二进制 0000 0000 0000 1000（PA3为1）。
● 按位取反后 → 1111 1111 1111 0111（PA3为0，其他为1）。

GPIOB的二进制位与引脚关系与GPIOA完全一致，遵循相同规则。
常见数值与引脚对照表（使用的是GPIOA,对应引脚就是PA0往后，GPIOB,就是PB0往后，前面的十六进制，二进制不变）
十六进制值	二进制值	对应引脚
0x0001	0000 0000 0000 0001	PA0
0x0002	0000 0000 0000 0010	PA1
0x0004	0000 0000 0000 0100	PA2
0x0008	0000 0000 0000 1000	PA3
0x0010	0000 0000 0001 0000	PA4
注意：每个GPIO端口（如GPIOA、GPIOB、GPIOC等）都有自己独立的ODR（Output Data Register）寄存器。这些寄存器的地址不同，彼此完全独立。
寄存器位映射：每个ODR寄存器的位0-15对应端口的引脚0-15（如PA0-PA15、PB0-PB15）

GPIO_Init()：赋予引脚功能 → “配置”。
GPIO_DeInit()：擦除引脚功能 → “重置”。

（一次不要看太多哦，小心脑袋爆炸，明天再看发现瞬间理解了！）

STM32标准外设库（Standard Peripheral Library）中用于控制GPIO的接口的函数
1. 初始化与重置
● GPIO_DeInit(GPIOx) 复位指定GPIO端口的所有寄存器到默认值（浮空输入）。 场景：重新配置GPIO前清理旧设置。
● GPIO_AFIODeInit() 复位复用功能（AFIO）寄存器，解除所有引脚重映射和事件输出配置。
● GPIO_Init(GPIOx, &GPIO_InitStruct) 按结构体参数配置GPIO引脚的模式、速度、上下拉等。
● GPIO_StructInit(&GPIO_InitStruct) 初始化结构体为默认值（输入浮空）。详解：默认值是STM32标准外设库预先定义的通用初始化值（浮空输入模式）。
2. 数据读写
● GPIO_ReadInputDataBit(GPIOx, GPIO_Pin) 读取输入引脚的电平（0/1）。 场景：检测按键是否按下（需引脚配置为输入）。
● GPIO_ReadOutputDataBit(GPIOx, GPIO_Pin) 读取输出引脚的当前状态（0/1）。
● GPIO_SetBits(GPIOx, GPIO_Pin) 设置指定引脚为高电平。 场景：点亮LED：GPIO_SetBits(GPIOA, GPIO_Pin_5);
● GPIO_ResetBits(GPIOx, GPIO_Pin) 设置引脚为低电平。
● GPIO_WriteBit(GPIOx, GPIO_Pin, BitVal) 直接写入位值（Bit_RESET或Bit_SET）。 等价于：BitVal ? SetBits : ResetBits
● GPIO_Write(GPIOx, PortVal) 一次性写入所有16个引脚的输出值（16位掩码）。
3. 高级配置
● GPIO_PinLockConfig(GPIOx, GPIO_Pin) 锁定引脚配置，防止意外修改。 场景：固件安全保护。
● GPIO_EXTILineConfig(GPIO_PortSource, GPIO_PinSource) 将引脚绑定到外部中断线。
● GPIO_PinRemapConfig(Remap, NewState)启用/禁用引脚重映射。
4. 事件输出（特定用途）
● GPIO_EventOutputConfig(PortSource, PinSource) 配置某引脚为事件输出源（用于触发内部事件，如唤醒CPU）。
● GPIO_EventOutputCmd(ENABLE) 启用事件输出功能。
5. 以太网专用（ETH）
● GPIO_ETH_MediaInterfaceConfig(Interface) 配置以太网PHY接口模式（如MII/RMII）。 场景：需硬件支持以太网外设。


PWM与STM32定时器(OC模式)配置关联知识总结
（定时器 (OC模式) 可控制高低电平的持续时间,那个高低电平上下上下，形成的图形叫PWM）

OC（Output Compare，输出比较） 定时器（TIM）模块的核心功能之一 :
OC模式用于生成特定波形（如PWM）
PWM基础概念
1. PWM（脉冲宽度调制）
  ○ 定义：通过快速切换高低电平，调节信号的平均电压。
  ○ 核心参数：
    ■ 频率（Hz）：每秒周期数（如1kHz = 1000次/秒）。
    ■ 占空比（%）：高电平时间占周期的比例（如30%高电平 + 70%低电平）。
  ○ 应用：LED调光、电机调速、音频信号生成等。
2. 占空比（Duty Cycle）
  ○ 公式： 占空比=(高电平时间/周期)×100%
  ○ 示例：周期1ms，高电平时间0.3ms → 占空比30%。
定时器（TIM2）的核心作用
1. TIM2的功能
  ○ 本质：硬件计时器，内置计数器（CNT）和比较逻辑。
  ○ 核心用途：
    ■ 生成PWM信号（无需CPU干预）。
    ■ 通过通道（如TIM2_CH1）输出到指定GPIO引脚（如PA0）。
2. TIM2_CH1（通道1）
  ○ 定义：TIM2的PWM输出通道1，与GPIO引脚（如PA0）绑定。
  ○ 硬件映射：
    ■ PA0复用为TIM2_CH1后，由TIM2自动控制其电平变化。
时钟源与预分频器
1. 时钟源（Clock Source）
  ○ 作用：驱动定时器计数器的“心跳”。
  ○ 常见来源：
    ■ 内部时钟（默认）：STM32主时钟（如72MHz）。
    ■ 外部时钟：通过引脚输入。
2. 预分频器（Prescaler）
  ○ 公式： 计数器时钟频率=主时钟频率/(预分频器值+1)
  ○ 示例：主时钟72MHz，预分频器719 → 计数器频率100kHz。
  ○ 目的：降低计数速度，适应外设需求（如生成低频PWM）。
自动重装载值（ARR）与计数器模式
1. 自动重装载值（ARR）
  ○ 功能：设定计数器的最大值，决定PWM周期。
  ○ 周期计算： PWM周期=(ARR+1)/计数器时钟频率
  ○ 示例：ARR=99，计数器频率100kHz → 周期1ms（频率1kHz）。
2. 向上计数模式
  ○ 规则：计数器从0递增到ARR，归零后重新开始。
  ○ 其他模式：向下计数、中央对齐（先递增后递减）。
占空比分辨率与CCR
1. 占空比分辨率
  ○ 定义：占空比可调节的最小步长，由ARR值决定。
  ○ 公式： 分辨率=1/(ARR+1)×100%
  ○ 示例：
    ■ ARR=99 → 分辨率1%（100级）。
    ■ ARR=999 → 分辨率0.1%（1000级）。
2. 比较寄存器（CCR）
  ○ 功能：设定阈值，控制高低电平切换点。
  ○ 占空比公式： 占空比=CCR/(ARR+1)×100%
  ○ 动态调节：通过TIM_SetCompare1()修改CCR值。
PWM模式与输出极性
1. PWM模式1（TIM_OCMode_PWM1）
  ○ 规则：
    ■ CNT < CCR → 输出高电平。
    ■ CNT ≥ CCR → 输出低电平。
  ○ 模式2：电平极性相反。
2. 输出极性（TIM_OCPolarity）
  ○ 高电平有效：占空比0%时输出低电平。
  ○ 低电平有效：占空比0%时输出高电平。
  ○ 应用场景：适配不同硬件（如共阳极/共阴极LED）。
GPIO复用与硬件配置
1. 复用推挽输出（AF_PP）
  ○ 作用：将GPIO引脚（如PA0）分配给外设（如TIM2_CH1）。

2. AFIO时钟的作用
  ○ 重映射功能：需要开启AFIO时钟才能修改引脚复用映射。
  ○ 示例：将TIM2_CH1从PA0重映射到PA15。
参数配置逻辑
1. 配置流程
  ○ 开启时钟 → 配置GPIO → 配置定时器 → 启动定时器。
2. 关键函数
  ○ RCC_APB1PeriphClockCmd()：开启外设时钟。
  ○ TIM_TimeBaseInit()：配置预分频器、ARR、计数模式。
  ○ TIM_OC1Init()：配置PWM模式和极性。
常见问题解答
1. 为什么PA0不能输出PWM？
  ○ 检查GPIO是否配置为AF_PP模式，时钟是否开启。
2. 如何提高PWM分辨率？
  ○ 增大ARR值（如ARR=999 → 分辨率0.1%）。
3. 如何反转PWM极性？
  ○ 设置TIM_OCPolarity_Low，并重新初始化PWM通道。
扩展：
高电平有效，表示高电平时起作用（比如：高电平有效表示“高电平点亮LED”），就是刚开始初始化为低电平，后面高电平才产生效果，所以占空比0%时输出低电平。
PWM如何调节LED亮度？
问题：LED亮度从全暗到全亮，是改变电压的大小吗？ 答案： 不是直接改变电压，而是通过快速开关LED，调节其“亮的时间”和“灭的时间”的比例（即占空比）。
● 基本原理：
  ○ 全亮：LED一直通电（占空比100%）。
  ○ 全暗：LED一直断电（占空比0%）。
  ○ 中间亮度：LED以高频快速开关，例如亮0.1ms、灭0.9ms（占空比10%）。
● 人眼感知： 如果开关频率足够高（如1kHz以上），人眼无法察觉闪烁，只会看到平均亮度。
  ○ 类比：快速开关电灯，频率高到一定程度后，看起来像是持续发光，但亮度由“亮的时间占比”决定。

如何选择参数？
(1) 明确需求
● LED调光：
  ○ 频率 ≥ 200Hz（避免人眼察觉闪烁）。
  ○ 分辨率 ≥ 1%（亮度变化平滑）。
● 电机调速：
  ○ 频率 ≥ 20kHz（避免电机啸叫）。
  ○ 分辨率可适当降低（如5%）。
(2) 计算步骤
1. 确定主时钟频率（如72MHz）。
2. 选择目标频率（如1kHz）。
3. 计算ARR和PSC：
  ○ 公式： ARR=(计数器时钟频率/目标频率)−1
  ○ 示例：
    ■ 主时钟72MHz，目标频率1kHz，分辨率1% → ARR=99，PSC=719。
(3) 工具辅助
● STM32CubeMX：输入目标频率和分辨率，自动生成PSC和ARR。
● 在线计算器：快速验证参数组合。
（这里我发现网上计算工具是计算寄存器操作的值，如果使用的库函数就还是得算，从寄存器代码推导标准库代码的ARR值，麻烦）。
可以直接问AI（别问豆包，有时它真会算错），可以这样问
示例：（
使用TIM2，系统时钟频率为72MHz，PWM周期为1ms,分辨率为百分之1，半分频，用库函数写代码时，它的TIM_TimeBaseInitStructure.TIM_Period = ？	TIM_TimeBaseInitStructure.TIM_Prescaler = ？这两个值为多少？
）
这里PWM周期，分辨率，分频是我们设计时根据需求，自己先知道的


通道与GPIO引脚映射
● 默认引脚映射（以STM32F103C8T6为例）：
通道	引脚（默认）	复用功能
TIM2_CH1	PA0	GPIO_AF_PP
TIM2_CH2	PA1	GPIO_AF_PP
TIM2_CH3	PA2	GPIO_AF_PP
TIM2_CH4	PA3	GPIO_AF_PP
注意：每个通道固定映射到特定GPIO引脚。
示例：若LED接在PA1，则必须使用TIM2_CH2，其他通道无法控制该LED


 TIM2定时器四个通道（CH1~CH4） 的核心功能总结：
TIM2通道功能对比表
特性	TIM2_CH1	TIM2_CH2	TIM2_CH3	TIM2_CH4
默认引脚	PA0	PA1	PA2	PA3
主要功能	PWM输出/输入捕获	PWM输出/输入捕获	PWM输出/输入捕获	PWM输出/输入捕获
特殊功能	编码器接口A相	编码器接口B相	无	无
互补输出	不支持（需高级定时器）	不支持（需高级定时器）	不支持（需高级定时器）	不支持（需高级定时器）
PWM输入模式	✔️（仅CH1/CH2支持）	✔️（仅CH1/CH2支持）	❌	❌
独立中断	TIM_IT_CC1	TIM_IT_CC2	TIM_IT_CC3	TIM_IT_CC4
寄存器操作	TIM2->CCR1	TIM2->CCR2	TIM2->CCR3	TIM2->CCR4
初始化函数	TIM_OC1Init()	TIM_OC2Init()	TIM_OC3Init()	TIM_OC4Init()
占空比设置函数	TIM_SetCompare1()	TIM_SetCompare2()	TIM_SetCompare3()	TIM_SetCompare4()

1. 定时器分类
定时器类型	定时器编号	功能特性
高级定时器	TIM1	支持PWM互补输出、死区控制
通用定时器	TIM2, TIM3, TIM4	支持PWM、输入捕获、编码器
基本定时器	TIM6, TIM7	仅支持定时中断/触发
2. 通用定时器通道及引脚映射（STM32F103C8T6）
TIM2（通用定时器）
● 通道数：4（CH1-CH4）
● 引脚映射：
通道	主引脚	复用引脚（AFIO重映射后）
CH1	PA0	PA15（需重映射）
CH2	PA1	PB3（需重映射）
CH3	PA2	PB10（需重映射）
CH4	PA3	PB11（需重映射）
TIM3（通用定时器）
● 通道数：4（CH1-CH4）
● 引脚映射：
通道	主引脚	复用引脚（AFIO重映射后）
CH1	PA6	PB4（需重映射）
CH2	PA7	PB5（需重映射）
CH3	PB0	PC8（需重映射）
CH4	PB1	PC9（需重映射）
TIM4（通用定时器）
● 通道数：4（CH1-CH4）
● 引脚映射：
通道	主引脚	复用引脚（AFIO重映射后）
CH1	PB6	PD12（需重映射，但C8T6无PD引脚）
CH2	PB7	PD13（需重映射，但C8T6无PD引脚）
CH3	PB8	PD14（需重映射，但C8T6无PD引脚）
CH4	PB9	PD15（需重映射，但C8T6无PD引脚）
注意：
● 重映射功能需启用AFIO时钟：RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
● STM32F103C8T6无PD引脚，因此TIM4重映射后的引脚不可用。
3. 高级定时器（TIM1）
● 通道数：4（CH1-CH4） + 互补通道（CH1N-CH3N）
● 引脚映射：
通道	主引脚	互补引脚（CHxN）
CH1	PA8	PB13 (CH1N)
CH2	PA9	PB14 (CH2N)
CH3	PA10	PB15 (CH3N)
CH4	PA11	无
4. 基本定时器（TIM6/TIM7）
● 通道数：0（无PWM输出功能，仅用于定时中断或触发其他外设）。

（一次不要看太多哦，小心脑袋爆炸，明天再看发现瞬间理解了！）

TIM（定时器）相关函数的功能和用途的详细分类解析：
一、初始化与反初始化
1. TIM_DeInit(TIM_TypeDef* TIMx)
  ○ 功能：将定时器的寄存器恢复为默认值（复位状态）。
  ○ 用途：在重新配置定时器前，确保其处于初始状态。
2. TIM_TimeBaseInit(TIM_TypeDef* TIMx, TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct)
  ○ 功能：配置定时器的时基单元（预分频器、计数器模式、自动重载值等）。
  ○ 用途：设置定时器的基础工作频率和计数模式，例如生成固定周期的定时中断。
3. TIM_OCxInit(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct)（x=1~4）
  ○ 功能：配置输出比较（Output Compare, OC）通道的参数（模式、极性、比较值等）。
  ○ 用途：生成PWM信号、单脉冲输出或触发其他外设。
4. TIM_ICInit(TIM_TypeDef* TIMx, TIM_ICInitTypeDef* TIM_ICInitStruct)
  ○ 功能：配置输入捕获（Input Capture, IC）通道的参数（边沿检测、滤波、预分频）。
  ○ 用途：测量外部信号的频率、占空比或脉冲宽度。
5. TIM_PWMIConfig(TIM_TypeDef* TIMx, TIM_ICInitTypeDef* TIM_ICInitStruct)
  ○ 功能：配置PWM输入模式（结合两个输入通道测量PWM信号的频率和占空比）。
  ○ 用途：高级PWM信号分析。
6. TIM_BDTRConfig(TIM_TypeDef* TIMx, TIM_BDTRInitTypeDef *TIM_BDTRInitStruct)
  ○ 功能：配置刹车和死区时间（Break and Dead-Time Register, BDTR）。
  ○ 用途：用于电机控制中的互补PWM输出，防止上下桥臂短路。
二、结构体初始化
1. TIM_xxxStructInit(TIM_xxxInitTypeDef* InitStruct)（xxx=TimeBase, OC, IC, BDTR）
  ○ 功能：将结构体成员初始化为默认值。
  ○ 用途：快速初始化配置结构体，避免手动填充所有字段。
三、使能控制
1. TIM_Cmd(TIM_TypeDef* TIMx, FunctionalState NewState)
  ○ 功能：启用或禁用定时器的计数器。
  ○ 用途：启动/停止定时器工作。
2. TIM_CtrlPWMOutputs(TIM_TypeDef* TIMx, FunctionalState NewState)
  ○ 功能：使能或禁用高级定时器的PWM输出。
  ○ 用途：控制PWM输出的总开关（仅高级定时器支持）。
3. TIM_CCxCmd(TIM_TypeDef* TIMx, uint16_t Channel, uint16_t CCx)
  ○ 功能：启用/禁用指定通道的输入捕获或输出比较功能。
  ○ 用途：动态控制通道的工作状态。
四、中断与DMA
1. TIM_ITConfig(TIM_TypeDef* TIMx, uint16_t TIM_IT, FunctionalState NewState)
  ○ 功能：使能或禁用指定的定时器中断源（如更新中断、捕获中断等）。
  ○ 用途：处理定时器事件的中断请求。
2. TIM_GenerateEvent(TIM_TypeDef* TIMx, uint16_t TIM_EventSource)
  ○ 功能：通过软件强制生成事件（如更新事件、触发事件）。
  ○ 用途：手动触发事件以同步配置或测试。
3. TIM_DMACmd(TIM_TypeDef* TIMx, uint16_t TIM_DMASource, FunctionalState NewState)
  ○ 功能：使能定时器的DMA请求。
  ○ 用途：在特定事件（如更新、捕获完成）时触发DMA传输。
五、时钟源配置
1. TIM_InternalClockConfig(TIM_TypeDef* TIMx)
  ○ 功能：选择内部时钟作为定时器时钟源。
  ○ 用途：定时器使用内部时钟（如APB总线时钟）计数。
2. TIM_ETRClockMode1Config(TIM_TypeDef* TIMx, ...)
  ○ 功能：配置外部触发（ETR）引脚作为时钟源（模式1：时钟输入）。
  ○ 用途：定时器由外部信号驱动（如外部晶振）。
3. TIM_ETRClockMode2Config(TIM_TypeDef* TIMx, ...)
  ○ 功能：配置ETR引脚作为外部时钟（模式2：时钟输入+触发从模式）。
  ○ 用途：同步多个定时器。
六、输出比较（OC）
1. TIM_OCxPreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload)
  ○ 功能：启用/禁用输出比较寄存器的预装载。
  ○ 用途：在更新事件时同步更新比较值，避免中途修改导致信号不稳定。
2. TIM_SelectOCxM(TIM_TypeDef* TIMx, uint16_t Channel, uint16_t OCMode)
  ○ 功能：选择输出比较模式（如PWM模式、翻转模式、强制输出等）。
  ○ 用途：动态切换输出行为。
3. TIM_SetComparex(TIM_TypeDef* TIMx, uint16_t CompareValue)（x=1~4）
  ○ 功能：设置输出比较寄存器的值。
  ○ 用途：调整PWM占空比或比较触发点。
七、输入捕获（IC）
1. TIM_SetICxPrescaler(TIM_TypeDef* TIMx, uint16_t Prescaler)
  ○ 功能：设置输入捕获通道的预分频值（如每4个边沿捕获一次）。
  ○ 用途：减少高频信号的捕获次数。
2. TIM_GetCapturex(TIM_TypeDef* TIMx)
  ○ 功能：读取输入捕获寄存器的值。
  ○ 用途：获取捕获到的信号时间戳。
八、编码器接口
1. TIM_EncoderInterfaceConfig(TIM_TypeDef* TIMx, uint16_t EncoderMode, uint16_t IC1Polarity, uint16_t IC2Polarity)
  ○ 功能：配置定时器为编码器接口模式。
  ○ 用途：读取旋转编码器的正交信号，计算方向和计数。
九、从模式与触发
1. TIM_SelectSlaveMode(TIM_TypeDef* TIMx, uint16_t SlaveMode)
  ○ 功能：设置定时器为从模式（如门控、触发、外部时钟模式）。
  ○ 用途：同步多个定时器或响应外部事件。
2. TIM_SelectOutputTrigger(TIM_TypeDef* TIMx, uint16_t TRGOSource)
  ○ 功能：选择主模式输出触发源（如更新事件、比较匹配）。
  ○ 用途：触发其他外设（如ADC、DAC）。
十、状态与标志
1. TIM_GetFlagStatus(TIM_TypeDef* TIMx, uint16_t TIM_FLAG)
  ○ 功能：检查定时器事件标志（如更新标志、捕获标志）。
  ○ 用途：轮询方式处理事件。
2. TIM_ClearFlag(TIM_TypeDef* TIMx, uint16_t TIM_FLAG)
  ○ 功能：清除指定的事件标志。
  ○ 用途：标志复位以检测下一次事件。
3. TIM_GetITStatus(TIM_TypeDef* TIMx, uint16_t TIM_IT)
  ○ 功能：检查中断是否发生（结合中断使能状态）。
  ○ 用途：在中断服务函数中判断中断来源。
十一. 计数与触发控制
TIM_Cmd(TIM_TypeDef* TIMx, FunctionalState NewState)
● 作用：启动或停止定时器计数。
● 场景：完成配置后启动定时器。
TIM_SetCounter() 和 TIM_GetCounter()
● 作用：手动设置或读取当前计数值（CNT寄存器）。
TIM_SelectSlaveMode()
● 作用：配置从模式（如复位、门控、触发模式）。
● 参数：TIM_SlaveMode 如TIM_SlaveMode_Reset（外部信号复位计数器）。

十一、其他功能
1. TIM_SelectOnePulseMode(TIM_TypeDef* TIMx, uint16_t OPMode)
  ○ 功能：设置单脉冲模式（计数器在触发后停止）。
  ○ 用途：生成单次脉冲信号。
2. TIM_BDTRStructInit(TIM_BDTRInitTypeDef* BDTRInitStruct)
  ○ 功能：初始化刹车和死区时间结构体。
  ○ 用途：配置死区时间和刹车信号极性。
扩展：
必须“先时基，后OC”（先初始化时基单元（TimeBase），再配置OC（Output Compare）模式）：
1. 定时器的硬件机制要求时基单元的配置为OC模块提供稳定的时钟和周期基准。
2. 寄存器写入顺序影响更新事件的触发和值的同步，错误的顺序可能导致配置失效或输出异常。

硬件响应时间:
● 外设延迟：某些外设（如LED驱动电路、电机控制器）需要一定时间响应PWM信号的变化。
● 信号稳定性：若PWM信号变化过快，硬件可能来不及处理，导致信号错乱或毛刺。
● 示例：
  ○ 若LED的驱动电路有滤波电容，快速变化的PWM会导致电容充放电不完全，亮度变化不线性。
  ○ 无刷电机需要一定时间响应新的PWM占空比以调整转速。

TIM_OCInitTypeDef 结构体知识点总结
1. 功能概述
● 用途：配置定时器的输出比较（OC）通道，用于生成PWM信号、电平翻转、事件触发等。
● 核心作用：通过设定比较值（CCRx）和模式，控制输出引脚的行为及信号特征。
2. 成员详解
1. TIM_OCMode（输出比较模式）
  ○ 功能：定义通道的工作模式。
  ○ 常见模式：
    ■ TIM_OCMode_Timing：仅触发事件（如中断/DMA），不改变引脚电平。
    ■ TIM_OCMode_Toggle：匹配时翻转电平，生成方波。
    ■ TIM_OCMode_PWM1/PWM2：生成PWM信号，区别在于有效电平阶段。
      ● PWM1：CNT < CCRx时有效电平，否则无效。
      ● PWM2：CNT > CCRx时有效电平，否则无效。
    ■ TIM_OCMode_Active/Inactive：强制输出高/低电平。
  ○ 应用场景：PWM生成（电机控制、LED调光）、定时触发事件（ADC采样）。
2. TIM_OutputState（主输出使能）
  ○ 功能：启用或禁用主输出通道（如TIM_CH1）。
  ○ 取值：
    ■ TIM_OutputState_Enable：信号输出到物理引脚。
    ■ TIM_OutputState_Disable：关闭输出。
  ○ 示例：驱动LED时需启用输出，纯事件触发时可禁用。
3. TIM_OutputNState（互补输出使能）
  ○ 功能：启用或禁用互补输出通道（如TIM_CH1N）。
  ○ 注意：仅高级定时器（TIM1/TIM8）支持，用于驱动H桥电路。
  ○ 应用场景：电机控制中的互补PWM信号，避免上下桥臂短路。
4. TIM_Pulse（比较值）
  ○ 功能：设置捕获/比较寄存器（CCRx）的初始值。
  ○ 计算方式：
    ■ PWM占空比：占空比 = TIM_Pulse / (ARR + 1)（ARR为自动重载值）。
    ■ 触发时间点：计数器达到此值时触发事件。
  ○ 示例：ARR=999时，TIM_Pulse=500对应50%占空比。
5. TIM_OCPolarity（主输出极性）
  ○ 功能：定义主输出的有效电平方向。
  ○ 取值：
    ■ TIM_OCPolarity_High：有效电平为高。
    ■ TIM_OCPolarity_Low：有效电平为低。
  ○ 影响：决定PWM波形中高/低电平的分布阶段。
6. TIM_OCNPolarity（互补输出极性）
  ○ 功能：定义互补输出的有效电平方向。
  ○ 注意：仅高级定时器有效，通常与主输出极性相反以实现互补逻辑。
7. TIM_OCIdleState/TIM_OCNIdleState（空闲状态）
  ○ 功能：设置定时器停止或未启动时的输出状态。
  ○ 取值：
    ■ TIM_OCIdleState_Set：输出有效电平。
    ■ TIM_OCIdleState_Reset：输出无效电平。
  ○ 应用场景：确保电机在非工作状态下断电（安全保护）。
3. 高级定时器专用配置
● 适用对象：TIM1、TIM8（支持互补输出和复杂控制）。
● 相关成员：
  ○ TIM_OutputNState、TIM_OCNPolarity、TIM_OCNIdleState。
● 典型应用：电机驱动、逆变器控制中的互补PWM信号和死区管理。
4. 配置注意事项
1. 极性匹配：
  ○ 需与外部电路设计一致（如高电平有效或低电平有效的驱动电路）。
2. 占空比计算：
  ○ 确保TIM_Pulse ≤ ARR，否则占空比恒为100%。
3. 预装载功能：
  ○ 若需动态修改CCRx，建议启用预装载（TIM_OCPreload_Enable），避免信号抖动。
4. 互补信号同步：
  ○ 在高级定时器中，主输出与互补输出的极性和空闲状态需协调配置，确保逻辑正确。
5. 典型应用场景
1. PWM信号生成
  ○ 配置：TIM_OCMode_PWM1 + 使能输出 + 设置占空比。
  ○ 应用：LED调光、舵机控制、开关电源调节。
2. 互补PWM驱动
  ○ 配置：主/互补输出使能 + 相反极性 + 死区时间（通过TIM_BDTRConfig设置）。
  ○ 应用：电机驱动、三相逆变器。
3. 定时事件触发
  ○ 配置：TIM_OCMode_Timing + 中断/DMA使能。
  ○ 应用：周期性ADC采样、通信协议时序控制。

（一次不要看太多哦，小心脑袋爆炸，明天再看发现瞬间理解了！）

STM32 AFIO（Alternate Function I/O）详解
 AFIO 的核心功能
AFIO（Alternate Function I/O）模块是STM32中用于管理 复用功能 和 引脚重映射 的核心模块，主要功能包括：
● 复用功能配置：将GPIO引脚分配给特定外设（如USART、SPI、定时器等）。
● 引脚重映射：将外设的默认引脚映射到其他备用引脚。
● 外部中断（EXTI）输入源选择：配置外部中断信号来自哪个GPIO引脚。
● 调试端口配置：禁用JTAG/SWD以释放引脚（如PB3/PB4）。
2. AFIO 的关键应用场景
(1) 外部中断（EXTI）的输入源选择
● 问题：STM32的每个外部中断线（如EXTI0）可以绑定到多个GPIO引脚（如PA0、PB0、PC0等）。
● 解决：通过AFIO的 AFIO_EXTICRx 寄存器选择具体引脚。
(2) 引脚重映射
● 应用场景：当默认外设引脚被占用时，重映射到其他引脚。
  ○ 示例：将USART1的TX/RX从PA9/PA10重映射到PB6/PB7。
(3) 调试端口释放
● 问题：默认情况下，PB3/PB4/PB15等引脚用于JTAG调试，导致无法作为普通GPIO使用。
● 解决：通过AFIO禁用JTAG，释放引脚。
注意事项
1. AFIO时钟必须启用：否则所有AFIO相关操作（EXTI、重映射）无效。
2. 重映射与GPIO模式冲突：重映射后的引脚需配置为复用模式（如GPIO_Mode_AF_PP）。
3. 不同STM32系列的差异：
  ○ STM32F1系列：使用AFIO模块。
  ○ STM32F4/F7系列：使用SYSCFG模块（需启用SYSCFG时钟）。
4. 中断标志清除：在EXTI中断服务函数中必须清除中断标志位。
8. 常见问题排查
问题现象	可能原因	解决方案
外部中断不触发	AFIO时钟未启用	启用RCC_APB2Periph_AFIO
重映射后外设不工作	GPIO未配置为复用模式	检查GPIO的Mode是否为AF_PP
或IN_FLOATING
PB3/PB4无法控制	JTAG未禁用	调用GPIO_PinRemap_SWJ_JTAGDisable

外部中断线（EXTI）与引脚映射
1. EXTI线的基本概念
STM32的 外部中断线（EXTI） 是一种硬件机制，允许特定的GPIO引脚触发中断。以下是核心要点：
1. EXTI线数量： STM32F103系列共有 16条外部中断线（EXTI0 ~ EXTI15）。
  ○ EXTI0 到 EXTI15 对应 GPIO引脚号0到15（如PA0、PB1、PC2等）。
  ○ 例如：EXTI0 可以连接到 PA0、PB0、PC0 等引脚，但同一时间只能选择一个。
2. EXTI线的作用：
  ○ 每条EXTI线可以监控 一个引脚号的所有GPIO端口。
  ○ 例如：EXTI14 可以监控 PA14、PB14、PC14，但只能选择其中一个引脚。
2. 为什么需要引脚映射？
● 问题： 假设你想用 PB14 作为外部中断引脚，但 EXTI14 默认可能连接到其他引脚（如PA14）。
● 解决： 通过 AFIO（复用功能I/O） 将 EXTI14 映射到 PB14

3. 引脚映射规则
● 规则： 每个EXTI线只能绑定到同一引脚号的不同端口。
  ○ 例如：EXTI14 可以绑定到 PA14、PB14、PC14，但不能绑定到 PA15（引脚号不同）。
● 映射函数的作用： GPIO_EXTILineConfig() 函数通过配置 AFIO_EXTICR 寄存器，选择具体端口。
4.常见问题解答
Q1：EXTI线可以随便映射到任意引脚吗？
● 不可以！
  ○ 每个EXTI线只能绑定到 同一引脚号的不同端口。
  ○ 例如：EXTI14 只能绑定到 PA14、PB14、PC14，不能绑定到PA15或PB13。
5.总结
● EXTI线是硬件资源：每条线对应一个引脚号，但可以映射到不同端口。
● 映射规则：只能绑定到同一引脚号的不同端口（如PA14、PB14）。
● 关键步骤：
  a. 启用AFIO时钟。
  b. 配置GPIO为输入模式。
  c. 调用 GPIO_EXTILineConfig() 绑定引脚。
  d. 配置EXTI触发条件和中断优先级。
  e. 在ISR中清除中断标志。

STM32标准外设库（Standard Peripheral Library）中 EXTI（外部中断）模块函数
1. void EXTI_DeInit(void);
● 功能：将EXTI的所有寄存器恢复为默认值（复位状态）。
● 使用场景：在重新配置EXTI前调用，确保之前的配置不会干扰新设置。
● 注意事项：
  ○ 调用后，所有中断线配置（触发方式、使能状态等）均被清除。
  ○ 通常与EXTI_Init配合使用，实现重新初始化。
2. void EXTI_Init(EXTI_InitTypeDef* EXTI_InitStruct);
● 功能：根据结构体参数配置EXTI。
● 参数：EXTI_InitTypeDef 结构体，包含以下字段：
  ○ EXTI_Line：选择EXTI线（如EXTI_Line0-EXTI_Line15）。
  ○ EXTI_Mode：模式选择（中断 EXTI_Mode_Interrupt 或事件 EXTI_Mode_Event）。
  ○ EXTI_Trigger：触发方式（上升沿、下降沿、双边沿）。
  ○ EXTI_LineCmd：使能或禁用该线。
● 使用场景：初始化或修改EXTI线的行为。
3. void EXTI_StructInit(EXTI_InitTypeDef* EXTI_InitStruct);
● 功能：初始化EXTI_InitTypeDef结构体为默认值。
● 默认值：
  ○ EXTI_Line：EXTI_Line0（需根据实际修改）。
  ○ EXTI_Mode：EXTI_Mode_Interrupt。
  ○ EXTI_Trigger：EXTI_Trigger_Rising。
  ○ EXTI_LineCmd：DISABLE。
● 使用场景：避免手动初始化结构体字段，确保无遗漏。
4. void EXTI_GenerateSWInterrupt(uint32_t EXTI_Line);
● 功能：通过软件触发指定EXTI线的中断或事件。
● 参数：EXTI_Line（如EXTI_Line0）。
● 使用场景：
  ○ 测试中断服务函数（ISR）逻辑。
  ○ 手动触发事件（如唤醒低功耗模式）。
5. FlagStatus EXTI_GetFlagStatus(uint32_t EXTI_Line);
● 功能：检查指定EXTI线的标志位状态（是否被触发）。
● 返回值：SET（标志位置位）或 RESET（标志位未置位）。
● 注意事项：
  ○ 不检查中断是否使能，仅反映硬件/软件触发事件。
  ○ 适用于轮询模式或事件处理。
6. void EXTI_ClearFlag(uint32_t EXTI_Line);
● 功能：清除指定EXTI线的标志位。
● 使用场景：处理事件或轮询模式后清除标志位，避免重复触发。
7. ITStatus EXTI_GetITStatus(uint32_t EXTI_Line);
● 功能：检查中断是否被触发。
● 返回值：SET（表示中断已触发且已使能）或 RESET。
● 与GetFlagStatus区别：
  ○ GetITStatus会检查中断是否使能（EXTI_IMR寄存器）。
  ○ 适用于中断服务函数中判断中断来源。
8. void EXTI_ClearITPendingBit(uint32_t EXTI_Line);
● 功能：清除中断挂起标志位。
● 使用场景：在中断服务函数末尾调用，防止重复进入中断。
关键流程（中断处理）
1. 配置EXTI：使用EXTI_Init设置中断线、触发方式和模式。
2. 使能中断：在NVIC中配置中断优先级并使能。
3. 中断服务函数（ISR）：
  ○ 用EXTI_GetITStatus检查中断线。
  ○ 处理中断逻辑。
  ○ 调用EXTI_ClearITPendingBit清除标志位。
注意事项
● 标志位清除：必须清除标志位，否则导致持续触发。
● 中断与事件区别：
  ○ 中断：触发CPU跳转到ISR。
  ○ 事件：触发其他外设（如DMA），不经过CPU。
● 软件中断：调用EXTI_GenerateSWInterrupt后需手动清除标志位。

NVIC_InitTypeDef 结构体
1. NVIC_IRQChannel（中断通道）
● 作用： 指定具体的中断源（如定时器中断、串口中断等）。
● 取值示例：
  ○ EXTI0_IRQn：外部中断线0（如按键触发）。
  ○ TIM2_IRQn：定时器2中断。
  ○ USART1_IRQn：串口1中断。
● 如何查找： 在芯片头文件（如 stm32f10x.h）中定义，不同外设对应不同值。
2. NVIC_IRQChannelPreemptionPriority（抢占优先级）
● 作用： 决定中断能否打断其他正在执行的中断。 数值越小，优先级越高。
● 取值范围： 由优先级分组决定（需先调用 NVIC_PriorityGroupConfig）。 示例：
  ○ 若分组为 NVIC_PriorityGroup_2（抢占优先级占2位）： 有效值为 0-3（2位可表示4个优先级）。
  ○ 若分组为 NVIC_PriorityGroup_3（抢占优先级占3位）： 有效值为 0-7（3位可表示8个优先级）。
3. NVIC_IRQChannelSubPriority（子优先级）
● 作用： 当多个中断的抢占优先级相同时，按子优先级决定处理顺序。 数值越小，优先级越高。
● 取值范围： 由优先级分组决定。 示例：
  ○ 若分组为 NVIC_PriorityGroup_2（子优先级占2位）： 有效值为 0-3。
  ○ 若分组为 NVIC_PriorityGroup_1（子优先级占3位）： 有效值为 0-7。
4. NVIC_IRQChannelCmd（中断使能/禁用）
● 作用： 控制该中断通道的开关。
● 取值：
  ○ ENABLE：开启中断。
  ○ DISABLE：关闭中断。
关键注意事项
1. 优先级分组需先配置： 在设置优先级前，必须调用 NVIC_PriorityGroupConfig() 确定分组。
2. 优先级数值必须合法： 若填写的值超出分组允许范围，可能导致未定义行为。
  ○ 错误示例： 分组为 NVIC_PriorityGroup_2，但抢占优先级设为 5（范围应为0-3）。 系统不会自动处理，需开发者自行确保数值合法。
3. 中断通道需对应实际外设： 例如，使用串口1接收数据时，应选择 USART1_IRQn，而非其他通道。
常见问题解答
Q1: 如果填写的优先级超出分组范围会怎样？
系统可能忽略高位或导致不可预测行为，必须手动确保数值合法。 例如，分组为 NVIC_PriorityGroup_2 时：
● 抢占优先级填 5 → 实际可能被当作 1（二进制 101 取低2位为 01）。
Q2: 如何查找具体外设的中断通道？
查阅芯片数据手册或头文件（如 stm32f10x.h），所有外设中断通道均以 _IRQn 结尾。
Q3: 是否可以动态修改优先级？
可以，通过重新调用 NVIC_Init 修改优先级或使能状态。
总结
● NVIC_IRQChannel：选择具体外设中断源。
● 抢占优先级：决定中断嵌套能力（数值越小越优先）。
● 子优先级：决定同抢占级中断的顺序（数值越小越优先）。
● 使能/禁用：控制中断是否生效。
扩展：
1. 抢占优先级 (Preemption Priority)
  ○ 作用：决定中断能否打断其他正在执行的中断。
  ○ 数值越小，优先级越高。
  ○ 位数决定可区分的优先级数量：
    ■ 1位 → 2种优先级（0-1）
    ■ 2位 → 4种优先级（0-3）
    ■ 依此类推。
2. 子优先级 (Subpriority)
  ○ 作用：相同抢占优先级的中断按子优先级顺序执行。
  ○ 数值越小，优先级越高。
  ○ 位数决定精细度：
    ■ 4位 → 16种子优先级（0-15）
    ■ 3位 → 8种子优先级（0-7）
    ■ 依此类推。
分表示例
分组名称	抢占优先级位数	子优先级位数	抢占优先级范围	子优先级范围	典型场景
NVIC_PriorityGroup_0	0位	4位	无（均为0）	0-15	无中断嵌套，纯顺序处理
NVIC_PriorityGroup_1	1位	3位	0-1	0-7	少量嵌套，精细子优先级
NVIC_PriorityGroup_2	2位	2位	0-3	0-3	平衡嵌套与子优先级（常用）
NVIC_PriorityGroup_3	3位	1位	0-7	0-1	多级嵌套，快速响应高优先级
NVIC_PriorityGroup_4	4位	0位	0-15	无	纯抢占优先级，无子优先级排队
子优先级的取值范围 是由分配给它的位数决定的：
● 若有 n 位用于子优先级 → 取值范围是 0 到 (2ⁿ - 1)
具体规则
1. 抢占优先级位数由分组决定：
  ○ 例如：分组 NVIC_PriorityGroup_2 中，抢占优先级占2位 → n=2。
  ○ 取值范围为 0 到 (2² - 1) = 0-3。
2. 子优先级同理：
  ○ 例如：分组 NVIC_PriorityGroup_1 中，子优先级占3位 → n=3。
  ○ 取值范围为 0 到 (2³ - 1) = 0-7。
常见误区
1. “抢占优先级位数=0表示子优先级位数也为0”？（抢占优先级位数 + 子优先级位数 = 4）
  ○ 错误！总位数固定为4，抢占位数=0 → 子位数=4。
  ○ 例如：NVIC_PriorityGroup_0 中，所有中断的抢占优先级相同（无法嵌套），但子优先级可精细到16级。
2. “高抢占优先级的中断必须设置低子优先级”？
  ○ 错误！抢占和子优先级独立设置。
  ○ 示例：
    ■ 中断A：抢占=1，子=15 → 可打断抢占=2的中断。
    ■ 中断B：抢占=1，子=0 → 在抢占=1的中断中优先执行。
3. “子优先级=5在任何分组中都合法”？
  ○ 错误！取值范围由分组决定：
    ■ NVIC_PriorityGroup_0 → 合法（0-15）
    ■ NVIC_PriorityGroup_2 → 非法（范围0-3）
配置建议
1. 选择分组：
  ○ 需要中断嵌套 → 选 NVIC_PriorityGroup_2（平衡）或 3（多级嵌套）。
  ○ 无需嵌套，只需顺序处理 → 选 NVIC_PriorityGroup_0。
2. 设置优先级：
  ○ 抢占优先级：关键任务（如电机控制）设为更高优先级（数值更小）。
  ○ 子优先级：同抢占级的中断按需求排序（如传感器A优先于传感器B）。

EXTI 与 NVIC 的关系
在 STM32 中，EXTI（外部中断/事件控制器） 和 NVIC（嵌套向量中断控制器） 是中断处理流程中的两个关键模块，它们协同工作以实现外部中断的检测和响应。以下是它们的核心关系：
1. 功能分工
模块	功能描述
EXTI	负责检测外部引脚的电平变化（如按键按下、传感器信号）。- 生成中断请求或事件。
NVIC	 统一管理所有中断源（包括 EXTI、定时器、串口等）的优先级和使能状态。- 决定哪个中断优先响应，并控制 CPU 进入中断服务函数。
2. 协作流程
1. EXTI 检测中断信号
  ○ 例如：配置 EXTI 检测 GPIO 引脚 PA0 的上升沿触发。
  ○ 当 PA0 引脚电平从低变高时，EXTI 会置位中断标志位（如 EXTI_PR 寄存器中的对应位）。
2. EXTI 向 NVIC 发送中断请求
  ○ EXTI 的中断线（如 EXTI0）对应到 NVIC 的特定中断通道（如 EXTI0_IRQn）。
  ○ EXTI 触发后，NVIC 会检测到该中断请求。
3. NVIC 处理中断
  ○ 优先级判断：根据抢占优先级和子优先级决定是否打断当前中断。
  ○ 跳转执行：若中断被允许，CPU 跳转到对应的中断服务函数（如 EXTI0_IRQHandler）。
4. 中断服务函数处理
  ○ 在中断服务函数中：
    ■ 清除 EXTI 的中断标志位（避免重复触发）。
    ■ 执行用户逻辑（如点亮 LED、发送数据等）。
3. 配置依赖
● EXTI 必须依赖 NVIC 才能触发中断：
  ○ 即使 EXTI 检测到信号，若未通过 NVIC 使能中断通道，CPU 不会响应。
4. 核心区别
特性	EXTI	NVIC
作用范围	仅处理外部引脚的中断/事件	管理所有外设的中断（如 EXTI、定时器、串口）
优先级控制	无	控制中断的抢占优先级和子优先级
中断标志位	有自己的标志位（如 EXTI_PR
）	不直接处理标志位，仅管理中断通道状态
中断服务函数	需在代码中实现（如 EXTI0_IRQHandler）	自动跳转到中断服务函数
总结
● EXTI 是“哨兵”：负责检测外部信号并发出中断请求。
● NVIC 是“指挥官”：决定哪些中断优先处理，并调度 CPU 响应。
● 二者缺一不可：只有同时配置 EXTI 和 NVIC，外部中断才能正常工作。

初始化顺序：时钟 → GPIO → EXTI映射 → EXTI配置 → NVIC配置。

（一次不要看太多哦，小心脑袋爆炸，明天再看发现瞬间理解了！）


STM32定时器（TIM）模块相关函数的解析：

一、初始化与复位函数
1. TIM_DeInit(TIMx)
  ○ 功能：复位TIMx寄存器到默认值（上电初始状态）。
  ○ 参数：TIMx（定时器实例，如TIM1-TIM14）。
2. TIM_TimeBaseStructInit(TIM_TimeBaseInitStruct)
  ○ 功能：初始化时基结构体为默认值（如预分频=0xFFFF，计数模式=向上等）。
  ○ 参数：TIM_TimeBaseInitStruct（时基配置结构体指针）。
3. TIM_OCStructInit(TIM_OCInitStruct)
  ○ 功能：初始化输出比较（OC）结构体为默认值（如模式=关闭，极性=高电平等）。
  ○ 参数：TIM_OCInitStruct（OC配置结构体指针）。
4. TIM_ICStructInit(TIM_ICInitStruct)
  ○ 功能：初始化输入捕获（IC）结构体为默认值（如捕获边沿=上升沿，滤波=无等）。
  ○ 参数：TIM_ICInitStruct（输入捕获配置结构体指针）。
5. TIM_BDTRStructInit(TIM_BDTRInitStruct)
  ○ 功能：初始化断路和死区时间结构体为默认值（如死区时间=0，OSSR=关闭等）。
  ○ 参数：TIM_BDTRInitStruct（BDTR配置结构体指针）。
  ○ 适用：仅高级定时器（如TIM1、TIM8）。

二、时基配置函数
1. TIM_TimeBaseInit(TIMx, TIM_TimeBaseInitStruct)
  ○ 功能：配置定时器时基参数（预分频器、计数模式、周期值等）。
  ○ 参数：
    ■ TIMx：定时器实例。
    ■ TIM_TimeBaseInitStruct：时基配置结构体，包含TIM_Prescaler, TIM_CounterMode, TIM_Period, TIM_ClockDivision等字段。
2. TIM_PrescalerConfig(TIMx, Prescaler, TIM_PSCReloadMode)
  ○ 功能：动态设置预分频器值。
  ○ 参数：
    ■ Prescaler：预分频值（0x0000-0xFFFF）。
    ■ TIM_PSCReloadMode：重载模式（立即生效或等待更新事件）。
3. TIM_CounterModeConfig(TIMx, TIM_CounterMode)
  ○ 功能：设置计数器模式（向上、向下、中央对齐模式）。
  ○ 参数：TIM_CounterMode（如TIM_CounterMode_Up, TIM_CounterMode_CenterAligned1）。
4. TIM_ClockDivisionConfig(TIMx, TIM_CKD)
  ○ 功能：配置时钟分频（用于输入滤波和数字滤波器）。
  ○ 参数：TIM_CKD（如TIM_CKD_DIV1, TIM_CKD_DIV4）。
5. TIM_ARRPreloadConfig(TIMx, NewState)
  ○ 功能：使能/禁用自动重装载寄存器（ARR）的预加载功能。
  ○ 参数：NewState（ENABLE或DISABLE）。

三、输出比较（OC）配置函数
1. TIM_OCxInit(TIMx, TIM_OCInitStruct)
  ○ 功能：配置输出比较通道x（x=1-4）的参数（模式、极性、占空比等）。
  ○ 参数：
    ■ TIM_OCInitStruct：包含TIM_OCMode, TIM_OutputState, TIM_OCPolarity, TIM_Pulse等字段。
  ○ 示例：TIM_OC1Init配置通道1，TIM_OC2Init配置通道2，依此类推。
2. TIM_OCxPreloadConfig(TIMx, TIM_OCPreload)
  ○ 功能：使能/禁用输出比较通道x的预加载功能（用于同步更新CCRx寄存器）。
  ○ 参数：TIM_OCPreload（如TIM_OCPreload_Enable）。
3. TIM_OCxFastConfig(TIMx, TIM_OCFast)
  ○ 功能：使能快速模式，允许在CCRx匹配时立即改变输出（无需等待更新事件）。
  ○ 参数：TIM_OCFast（ENABLE或DISABLE）。
4. TIM_ClearOCxRef(TIMx, TIM_OCClear)
  ○ 功能：强制清除输出比较参考信号（如强制输出低电平）。
  ○ 参数：TIM_OCClear（如TIM_OCClear_Enable）。
5. TIM_OCxPolarityConfig(TIMx, TIM_OCPolarity)
  ○ 功能：设置输出比较通道x的极性（高有效/低有效）。
  ○ 参数：TIM_OCPolarity（如TIM_OCPolarity_High）。

四、输入捕获（IC）配置函数
1. TIM_ICInit(TIMx, TIM_ICInitStruct)
  ○ 功能：配置输入捕获通道参数（捕获边沿、滤波、分频等）。
  ○ 参数：TIM_ICInitStruct（包含TIM_Channel, TIM_ICPolarity, TIM_ICSelection, TIM_ICPrescaler, TIM_ICFilter）。
2. TIM_PWMIConfig(TIMx, TIM_ICInitStruct)
  ○ 功能：配置PWM输入模式（用于测量PWM信号频率和占空比）。
  ○ 参数：需将两个通道配置为互补输入（如通道1和通道2）。
3. TIM_SetICxPrescaler(TIMx, TIM_ICPSC)
  ○ 功能：设置输入捕获通道x的分频器（如每N个事件捕获一次）。
  ○ 参数：TIM_ICPSC（如TIM_ICPSC_DIV1, TIM_ICPSC_DIV4）。

五、PWM与互补输出控制
1. TIM_CtrlPWMOutputs(TIMx, NewState)
  ○ 功能：使能/禁用高级定时器的PWM输出（需配置BDTR后使用）。
  ○ 适用：仅TIM1、TIM8。
2. TIM_BDTRConfig(TIMx, TIM_BDTRInitStruct)
  ○ 功能：配置断路和死区时间（用于电机控制中的互补PWM输出）。
  ○ 参数：TIM_BDTRInitStruct（包含TIM_OSSRState, TIM_OSSIState, TIM_LOCKLevel, TIM_DeadTime, TIM_Break, TIM_BreakPolarity, TIM_AutomaticOutput）。

六、时钟源与触发配置
1. TIM_InternalClockConfig(TIMx)
  ○ 功能：选择内部时钟作为定时器时钟源（默认模式）。
2. TIM_ITRxExternalClockConfig(TIMx, TIM_InputTriggerSource)
  ○ 功能：使用内部触发（ITRx）作为外部时钟源（级联定时器）。
  ○ 参数：TIM_InputTriggerSource（如TIM_TS_ITR0）。
3. TIM_ETRClockMode1/2Config(TIMx, TIM_ExtTRGPrescaler, TIM_ExtTRGPolarity, ExtTRGFilter)
  ○ 功能：配置外部时钟模式1/2（通过ETR引脚输入时钟）。
  ○ 参数：
    ■ TIM_ExtTRGPrescaler：外部触发预分频（如TIM_ExtTRGPSC_OFF）。
    ■ TIM_ExtTRGPolarity：触发极性（上升沿/下降沿）。
    ■ ExtTRGFilter：输入滤波器值（0x0-0xF）。

七、中断与事件控制
1. TIM_ITConfig(TIMx, TIM_IT, NewState)
  ○ 功能：使能/禁用指定中断源（如更新事件、捕获事件等）。
  ○ 参数：TIM_IT（如TIM_IT_Update, TIM_IT_CC1）。
2. TIM_GenerateEvent(TIMx, TIM_EventSource)
  ○ 功能：软件生成指定事件（如强制更新事件）。
  ○ 参数：TIM_EventSource（如TIM_EventSource_Update）。
3. TIM_GetFlagStatus(TIMx, TIM_FLAG)
  ○ 功能：检查指定标志位是否置位（如更新标志、捕获标志）。
  ○ 返回：SET或RESET。
4. TIM_ClearFlag(TIMx, TIM_FLAG)
  ○ 功能：清除指定标志位。
5. TIM_GetITStatus(TIMx, TIM_IT)
  ○ 功能：检查中断是否发生（需先调用TIM_ITConfig）。
  ○ 返回：SET或RESET。
6. TIM_ClearITPendingBit(TIMx, TIM_IT)
  ○ 功能：清除中断挂起位。

八、DMA控制
1. TIM_DMAConfig(TIMx, TIM_DMABase, TIM_DMABurstLength)
  ○ 功能：配置DMA传输的基地址和突发长度。
  ○ 参数：
    ■ TIM_DMABase：DMA基地址（如TIM_DMABase_CR1）。
    ■ TIM_DMABurstLength：突发传输长度（1-18）。

2. TIM_DMACmd(TIMx, TIM_DMASource, NewState)
  ○ 功能：使能/禁用指定DMA请求源。
  ○ 参数：TIM_DMASource（如TIM_DMA_Update, TIM_DMA_CC1）。

九、编码器接口
1. TIM_EncoderInterfaceConfig(TIMx, TIM_EncoderMode, TIM_IC1Polarity, TIM_IC2Polarity)
  ○ 功能：配置编码器接口模式（正交编码器输入）。
  ○ 参数：
    ■ TIM_EncoderMode（如TIM_EncoderMode_TI1, TIM_EncoderMode_TI2, TIM_EncoderMode_TI12）。
    ■ TIM_ICxPolarity：输入通道极性。

十、从模式与触发
1. TIM_SelectSlaveMode(TIMx, TIM_SlaveMode)
  ○ 功能：设置从模式（如外部时钟、复位模式、门控模式等）。
  ○ 参数：TIM_SlaveMode（如TIM_SlaveMode_External1, TIM_SlaveMode_Reset）。
2. TIM_SelectMasterSlaveMode(TIMx, TIM_MasterSlaveMode)
  ○ 功能：使能/禁用主从模式（用于级联定时器）。
  ○ 参数：TIM_MasterSlaveMode（ENABLE或DISABLE）。
3. TIM_SelectInputTrigger(TIMx, TIM_InputTriggerSource)
  ○ 功能：选择输入触发源（如ITR0、TI1F_ED等）。
  ○ 参数：TIM_InputTriggerSource（如TIM_TS_TI1F_ED）。

十一、计数器与寄存器操作
1. TIM_SetCounter(TIMx, Counter)
  ○ 功能：设置计数器当前值。
2. TIM_SetAutoreload(TIMx, Autoreload)
  ○ 功能：设置自动重装载值（ARR）。
3. TIM_SetComparex(TIMx, Compare)
  ○ 功能：设置通道x的比较值（CCRx寄存器）。
  ○ 示例：TIM_SetCompare1设置通道1的比较值。
4. TIM_GetCounter(TIMx)
  ○ 返回：当前计数器值。
5. TIM_GetCapturex(TIMx)
  ○ 返回：通道x的捕获值（CCRx寄存器）。
  ○ 示例：TIM_GetCapture1获取通道1的捕获值。

十二、其他功能
1. TIM_SelectOnePulseMode(TIMx, TIM_OPMode)
  ○ 功能：选择单脉冲模式（计数器在触发事件后停止）。
  ○ 参数：TIM_OPMode（TIM_OPMode_Single或TIM_OPMode_Repetitive）。
2. TIM_SelectOutputTrigger(TIMx, TIM_TRGOSource)
  ○ 功能：选择输出触发源（用于同步其他外设）。
  ○ 参数：TIM_TRGOSource（如TIM_TRGOSource_Update, TIM_TRGOSource_OC1）。
3. TIM_ForcedOCxConfig(TIMx, TIM_ForcedAction)
  ○ 功能：强制输出比较通道x进入指定状态（如强制高/低电平）。
  ○ 参数：TIM_ForcedAction（如TIM_ForcedAction_Active）。

ADC函数：
1. 初始化与配置
● void ADC_DeInit(ADC_TypeDef* ADCx); 复位ADC外设，将寄存器恢复为默认值。用于重新配置ADC前的清理操作。 参数：ADCx（ADC1、ADC2等）。
● void ADC_Init(ADC_TypeDef* ADCx, ADC_InitTypeDef* ADC_InitStruct); 根据结构体参数初始化ADC。结构体包含分辨率、扫描模式、连续转换模式、触发源等配置。
● void ADC_StructInit(ADC_InitTypeDef* ADC_InitStruct); 初始化结构体为默认值（如12位分辨率、禁用扫描模式等），避免手动填充所有字段。
2. 使能控制
● void ADC_Cmd(ADC_TypeDef* ADCx, FunctionalState NewState); 使能（ENABLE）或禁用（DISABLE）ADC模块。
● void ADC_DMACmd(ADC_TypeDef* ADCx, FunctionalState NewState); 启用/禁用ADC的DMA请求，用于多通道扫描模式下的自动数据传输。
● void ADC_ITConfig(ADC_TypeDef* ADCx, uint16_t ADC_IT, FunctionalState NewState); 配置ADC中断（如转换完成中断ADC_IT_EOC）。 参数：ADC_IT指定中断类型。
3. 校准
● void ADC_ResetCalibration(ADC_TypeDef* ADCx); 复位校准寄存器，需等待复位完成。
● FlagStatus ADC_GetResetCalibrationStatus(ADC_TypeDef* ADCx); 检查校准复位是否完成（返回SET/RESET），（返回 SET 表示校准复位未完成，返回 RESET 才表示校准复位已完成）。
● void ADC_StartCalibration(ADC_TypeDef* ADCx); 启动校准过程。
● FlagStatus ADC_GetCalibrationStatus(ADC_TypeDef* ADCx); 检查校准是否完成。
（关于是SET还是RESET表示完成，需要查看数据手册对应的函数）
4. 转换控制
● void ADC_SoftwareStartConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState); 启用软件触发转换（调用后立即开始转换）。
● FlagStatus ADC_GetSoftwareStartConvStatus(ADC_TypeDef* ADCx); 检查软件触发转换是否正在进行。
5. 规则组配置
● void ADC_RegularChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime); 配置规则组通道的转换顺序和采样时间。 参数：
  ○ ADC_Channel：通道号（如ADC_Channel_0）。
  ○ Rank：转换顺序（1~16）。
  ○ ADC_SampleTime：采样时间（如ADC_SampleTime_3Cycles）。
● void ADC_ExternalTrigConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState); 启用外部触发规则组转换（需先配置触发源）。
6. 注入组配置
● void ADC_InjectedChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime); 配置注入组通道的转换顺序和采样时间。
● void ADC_ExternalTrigInjectedConvConfig(ADC_TypeDef* ADCx, uint32_t ADC_ExternalTrigInjecConv); 设置注入组外部触发源（如定时器触发）。
● void ADC_SoftwareStartInjectedConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState); 软件启动注入组转换。
7. 数据读取
● uint16_t ADC_GetConversionValue(ADC_TypeDef* ADCx); 读取规则组最新转换结果。
● uint16_t ADC_GetInjectedConversionValue(ADC_TypeDef* ADCx, uint8_t ADC_InjectedChannel); 读取注入组指定通道的转换结果。
8. 间断模式
● void ADC_DiscModeChannelCountConfig(ADC_TypeDef* ADCx, uint8_t Number); 设置每次触发转换的通道数（间断模式）。
● void ADC_DiscModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState); 启用间断模式。
9. 模拟看门狗
● void ADC_AnalogWatchdogCmd(ADC_TypeDef* ADCx, uint32_t ADC_AnalogWatchdog); 启用模拟看门狗，监控指定通道的电压范围。
● void ADC_AnalogWatchdogThresholdsConfig(ADC_TypeDef* ADCx, uint16_t High, uint16_t Low); 设置看门狗的高低阈值。
10. 温度传感器与内部参考电压
● void ADC_TempSensorVrefintCmd(FunctionalState NewState); 启用内部温度传感器或参考电压通道（需先校准）。
11. 中断与标志
● FlagStatus ADC_GetFlagStatus(ADC_TypeDef* ADCx, uint8_t ADC_FLAG); 检查状态标志（如转换完成标志ADC_FLAG_EOC）。
● void ADC_ClearFlag(ADC_TypeDef* ADCx, uint8_t ADC_FLAG); 清除状态标志。
● ITStatus ADC_GetITStatus(ADC_TypeDef* ADCx, uint16_t ADC_IT); 检查中断是否触发。
● void ADC_ClearITPendingBit(ADC_TypeDef* ADCx, uint16_t ADC_IT); 清除中断挂起位。
典型使用流程
1. 初始化：复位ADC → 配置结构体 → 初始化ADC → 配置通道。
2. 校准：复位校准 → 启动校准 → 等待完成。
3. 使能外设：开启ADC → 启用DMA/中断。
4. 触发转换：软件触发或外部触发。
5. 读取数据：轮询标志或使用中断/DMA读取结果。
通过合理组合这些函数，可实现单次转换、连续转换、多通道扫描、注入组插入等复杂ADC操作。

ADC 初始化结构体 ADC_InitTypeDef 
1. ADC_Mode：ADC 工作模式
● 功能：配置 ADC 独立或双模式（多 ADC 协同工作）。
● 常用取值：
  ○ ADC_Mode_Independent：独立模式（单 ADC 工作）。
  ○ ADC_Mode_RegSimult：双 ADC 规则组同步模式（两 ADC 同时采样不同通道）。
  ○ ADC_Mode_InjSimult：双 ADC 注入组同步模式。
● 应用场景： 高速或多信号同步采集时使用双模式。
2. ADC_ScanConvMode：扫描模式
● 功能：启用多通道循环扫描。
  ○ ENABLE：按顺序转换多个通道（需配置通道顺序）。
  ○ DISABLE：仅转换单个通道。
3. ADC_ContinuousConvMode：连续转换模式
● 功能：是否自动重启转换。
  ○ ENABLE：转换完成后自动开始下一次（需触发信号）。
  ○ DISABLE：单次转换后停止。
● 典型应用：
  ○ 实时监测：连续模式 + 定时器触发。
  ○ 单次读取：单次模式 + 软件触发。
4. ADC_ExternalTrigConv：规则组外部触发源
● 功能：选择启动规则组转换的外部触发事件。
● 常见触发源：
  ○ 软件触发：ADC_ExternalTrigConv_None（需调用 ADC_SoftwareStartConvCmd）。
  ○ 定时器触发：
    ■ ADC_ExternalTrigConv_T1_CC1（TIM1 通道 1）。
    ■ ADC_ExternalTrigConv_T3_CC2（TIM3 通道 2）。
  ○ 外部引脚触发：如 ADC_ExternalTrigConv_Ext_IT11（外部中断线 11）。
5. ADC_DataAlign：数据对齐方式
● 功能：转换结果在数据寄存器中的对齐方式。
● 取值：
  ○ 右对齐：ADC_DataAlign_Right 有效数据在寄存器的低位（如 12 位结果：0x0FFF）。
  ○ 左对齐：ADC_DataAlign_Left 有效数据在寄存器的高位（如 12 位结果：0xFFF0）。
● 注意事项： 左对齐时需手动移位处理（例如 result >> 4）。
6. ADC_NbrOfChannel：规则组通道数量
● 功能：设定规则组需要转换的通道总数。
● 范围：1 ~ 16（需与实际配置的通道数一致）。
● 配置要求： 必须与 ADC_RegularChannelConfig 配置的通道顺序（Rank）匹配。
⚠️ 关键注意事项
1. 扫描模式与通道数量：
  ○ 若启用扫描模式，必须配置 ADC_NbrOfChannel 和通道顺序。
  ○ 单通道模式时，ADC_NbrOfChannel 设为 1。
2. 触发源匹配：
  ○ 若使用外部触发（如定时器），需同时配置外设（如 TIM）的触发信号。
3. 校准要求：
  ○ 初始化后需执行 ADC 校准（调用 ADC_StartCalibration）。
4. 数据对齐处理：
  ○ 左对齐时需手动移位，右对齐直接读取低 12/10/8 位。

（一次不要看太多哦，小心脑袋爆炸，明天再看发现瞬间理解了！）

DMA 函数
一、DMA 核心函数概览
1. 初始化与配置函数
● DMA_DeInit(DMAy_Channelx)
  ○ 功能：复位指定 DMA 通道至默认状态。
● DMA_Init(DMAy_Channelx, &DMA_InitStruct)
  ○ 功能：根据结构体参数初始化 DMA 通道。
● DMA_StructInit(&DMA_InitStruct)
  ○ 功能：为 DMA 配置结构体赋默认值。
2. 控制与状态函数
● DMA_Cmd(DMAy_Channelx, NewState)
  ○ 功能：启用（ENABLE）或禁用（DISABLE）DMA 通道。
● DMA_ITConfig(DMAy_Channelx, DMA_IT, NewState)
  ○ 功能：启用或禁用指定 DMA 中断类型（如 DMA_IT_TC 传输完成中断）。
● DMA_SetCurrDataCounter(DMAy_Channelx, DataNumber)
  ○ 功能：设置 DMA 待传输数据个数。
● DMA_GetCurrDataCounter(DMAy_Channelx)
  ○ 功能：获取 DMA 剩余未传输的数据个数。
3. 中断与标志管理函数
● DMA_GetFlagStatus(DMAy_FLAG)
  ○ 功能：检查标志位状态（如 DMA_FLAG_TC1 通道 1 传输完成）。
● DMA_ClearFlag(DMAy_FLAG)
  ○ 功能：清除指定标志位。
● DMA_GetITStatus(DMAy_IT)
  ○ 功能：检查中断是否触发（需先启用中断）。
● DMA_ClearITPendingBit(DMAy_IT)
  ○ 功能：清除中断挂起位。
二、DMA 配置结构体 DMA_InitTypeDef 成员说明
成员	功能
DMA_PeripheralBaseAddr	外设数据寄存器地址（如 ADC 数据寄存器地址）。
DMA_MemoryBaseAddr	内存数据存储地址（如数组首地址）。
DMA_DIR	传输方向：
DMA_DIR_PeripheralSRC（外设→内存）
DMA_DIR_PeripheralDST（内存→外设）
DMA_BufferSize	需传输的数据单元个数（单位由数据宽度决定）。
DMA_PeripheralInc	外设地址是否递增：Enable（多寄存器）
Disable（单寄存器）
DMA_MemoryInc	内存地址是否递增：Enable（存储到数组）
Disable（覆盖同一变量）
DMA_PeripheralDataSize	外设数据宽度：Byte（8 位）、
HalfWord（16 位）、Word（32 位）
DMA_MemoryDataSize	内存数据宽度：同外设数据宽度选项。
DMA_Mode	传输模式：Normal（单次传输）
Circular循环传输）。
DMA_Priority	通道优先级：VeryHigh、High、
Medium、Low
DMA_M2M	内存到内存模式： Enable（内存间传输）
Disable（外设与内存传输）。
三、关键配置逻辑
1. 传输方向
  ○ 外设→内存：用于数据采集（如 ADC 读取）。
  ○ 内存→外设：用于数据发送（如 UART 发送）。
2. 地址递增规则
  ○ 外设地址递增：需访问多个外设寄存器时启用。
  ○ 内存地址递增：数据存储到数组或缓冲区时启用。
3. 传输模式选择
  ○ 单次模式：传输指定数量后停止，需手动重启。
  ○ 循环模式：自动重复传输，适用于持续数据流（如音频）。
4. 中断使用场景
  ○ 传输完成中断：用于处理数据或启动下一阶段任务。
  ○ 错误中断：用于处理传输异常。
四、注意事项
● 数据对齐：内存地址需匹配数据宽度（如 32 位数据需 4 字节对齐）。
● 循环模式限制：内存到内存模式（DMA_M2M_Enable）不支持循环传输。
● 中断优先级：需通过 NVIC 配置中断优先级。

USART（通用同步异步收发器）相关函数
一、USART 核心概念
USART：用于串行通信（如与电脑、传感器、模块通信），支持全双工（同时收发）、异步/同步模式。
常见用途：调试信息输出（printf）、GPS 模块通信、蓝牙数据传输等。

二、函数分类讲解
1. 初始化与配置函数
void USART_DeInit(USART_TypeDef* USARTx);
功能：复位 USART 外设，恢复寄存器默认值。
参数：USARTx（如 USART1, USART2）。
使用场景：重新配置 USART 前的清理操作。
void USART_Init(USART_TypeDef* USARTx, USART_InitTypeDef* USART_InitStruct);
功能：根据结构体参数初始化 USART。
关键配置参数（结构体字段）：
波特率（Baud Rate）：如 9600、115200。
数据位长度：8 位、9 位。
停止位：1 位、2 位。
校验位：无校验、奇校验、偶校验。
模式：发送模式、接收模式、收发模式。
void USART_StructInit(USART_InitTypeDef* USART_InitStruct);
功能：初始化结构体为默认值（如波特率 9600、8 位数据、无校验）。
用途：避免手动填充所有字段。
void USART_ClockInit(USART_TypeDef* USARTx, USART_ClockInitTypeDef* USART_ClockInitStruct);
功能：配置同步模式下的时钟参数（如时钟极性、相位）。
适用场景：仅用于同步通信（如 SPI 模式）。

2. 控制函数
void USART_Cmd(USART_TypeDef* USARTx, FunctionalState NewState);
功能：启用（ENABLE）或禁用（DISABLE）USART 外设。
注意：初始化后必须启用 USART 才能工作。
void USART_ITConfig(USART_TypeDef* USARTx, uint16_t USART_IT, FunctionalState NewState);
功能：启用或禁用 USART 中断（如接收完成中断、发送完成中断）。
参数：
USART_IT：中断类型（如 USART_IT_RXNE 接收中断、USART_IT_TXE 发送中断）。
void USART_DMACmd(USART_TypeDef* USARTx, uint16_t USART_DMAReq, FunctionalState NewState);
功能：启用 USART 的 DMA 请求（用于高效传输数据）。
参数：
USART_DMAReq：DMA 请求类型（如 USART_DMAReq_Tx 发送 DMA、USART_DMAReq_Rx 接收 DMA）。

3. 数据收发函数
void USART_SendData(USART_TypeDef* USARTx, uint16_t Data);
功能：发送单个数据（需等待发送缓冲区为空）。
uint16_t USART_ReceiveData(USART_TypeDef* USARTx);
功能：读取接收到的数据。
注意：需先检查接收标志位（如 USART_FLAG_RXNE）。
void USART_SendBreak(USART_TypeDef* USARTx);
功能：发送中止符（连续低电平信号），用于某些特殊协议。

4. 高级功能函数
void USART_SetAddress(USART_TypeDef* USARTx, uint8_t USART_Address);
功能：设置 USART 的地址（用于多处理器通信中的地址过滤）。
void USART_WakeUpConfig(USART_TypeDef* USARTx, uint16_t USART_WakeUp);
功能：配置唤醒模式（如空闲线唤醒、地址标记唤醒），用于低功耗场景。
void USART_ReceiverWakeUpCmd(USART_TypeDef* USARTx, FunctionalState NewState);
功能：启用接收器唤醒功能（在睡眠模式下通过数据接收唤醒 MCU）。
void USART_LINCmd(USART_TypeDef* USARTx, FunctionalState NewState);
功能：启用 LIN（局域互联网络）模式。
void USART_IrDACmd(USART_TypeDef* USARTx, FunctionalState NewState);
功能：启用红外（IrDA）通信模式。
void USART_HalfDuplexCmd(USART_TypeDef* USARTx, FunctionalState NewState);
功能：启用半双工模式（同一时刻只能发送或接收）。

5. 状态与中断管理
FlagStatus USART_GetFlagStatus(USART_TypeDef* USARTx, uint16_t USART_FLAG);
功能：检查状态标志位（如数据是否发送完成、是否接收到数据）。
常用标志：
USART_FLAG_TXE：发送缓冲区空（可写入新数据）。
USART_FLAG_RXNE：接收缓冲区非空（可读取数据）。
USART_FLAG_TC：发送完成（所有数据已发送）。
void USART_ClearFlag(USART_TypeDef* USARTx, uint16_t USART_FLAG);
功能：清除指定标志位。
ITStatus USART_GetITStatus(USART_TypeDef* USARTx, uint16_t USART_IT);
功能：检查中断是否触发（需先启用中断）。
void USART_ClearITPendingBit(USART_TypeDef* USARTx, uint16_t USART_IT);
功能：清除中断挂起位。

就到此结束吧，后面还有一些通信，模式什么的可以去网上深入了解，笔记也是全面借助了DeepSeek帮忙组织语言。
