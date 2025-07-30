%% Main System Flowchart
graph TD
    A[程序启动] --> B(初始化);
    B --> C{主循环 while(1)};
    C --> D{测试模式是否激活?};
    D -- 是 --> E["<p>执行<b>测试模式</b></p>Execute_Test_Movement"];
    D -- 否 --> F(执行正常模式);
    subgraph 正常交互模式
        direction LR
        F1["<p>处理<b>触摸与情绪</b></p>handle_mood_logic_Quanju"] --> F2["<p>执行<b>情绪动作</b></p>Execute_Action_Sequencer"];
        F2 --> F3["<p>执行<b>呼噜声</b></p>Execute_Rhythmic_Purring"] --> F4["<p>执行<b>呼吸动作</b></p>Execute_Breathing_V2"];
    end
    F --> F1;
    E --> G;
    F4 --> G;
    G["<p>处理<b>UART指令</b></p>Process_UART_Commands"] --> H["<p>发送<b>蓝牙数据</b></p>Uart_BlueTooth"] --> C;

    %% Define Subgraphs
    subgraph init [初始化流程]
        B1[初始化硬件: Clock, MPU, GPIO, DMA等] --> B2[初始化交互系统: 阈值, 状态] --> B3[启动ADC DMA传输] --> B4[启动所有PWM通道] --> B5[启动UART中断接收];
    end

    subgraph handle_mood [触摸与情绪处理模块 (handle_mood_logic_Quanju)]
        M1[开始] --> M2{遍历4个触摸区域};
        M2 -- 对每个区域 --> M3{区域是否在冷却期?};
        M3 -- 是 --> M2_Loop(返回遍历下一个区域);
        M3 -- 否 --> M4{聚合区域内传感器事件 (拍打优先)};
            subgraph sensor_logic [区域事件检测]
                M4_1[遍历区域内所有传感器] --> M4_2[读取电压值] --> M4_3{电压 < 拍打阈值?}
                M4_3 --是--> M4_4[事件=拍打] --> M4_5
                M4_3 --否--> M4_3a{电压 < 触摸阈值?}
                M4_3a --是--> M4_4a[事件=触摸] --> M4_5
                M4_3a --否--> M4_4b[事件=无] --> M4_5
                M4_5{遍历完所有传感器?}--否-->M4_1
                M4_5 --是--> M4_6[输出区域最终事件]
            end
        M4 --> M11{处理区域状态机};
        M11 -- (IDLE & 有事件) --> M12[状态变为 PENDING];
        M11 -- (PENDING & 事件消失) --> M13[状态变回 IDLE];
        M11 -- (PENDING & 触摸持续 > 阈值) OR (拍打) --> M14{触发有效交互};
        M14 --> M15{全局冷却时间已过?};
        M15 -- 否 --> M19_End;
        M15 -- 是 --> M16{根据事件更新 moodValue};
        M16 -- 触摸 --> M17[moodValue++];
        M16 -- 拍打 --> M18[moodValue--];
        M17 & M18 --> M19[更新时间戳, 区域进入冷却];
        M19_End[结束当前区域处理];
        M2_Loop-->M2;
        M2 -- 遍历完成 --> M20{检查自然情绪衰减 (长时间无交互)};
        M20 -- 是 --> M21{moodValue 趋近于0};
        M21 --> M_End[结束];
        M20 -- 否 --> M_End
    end

    subgraph action_sequencer [情绪动作执行模块 (Execute_Action_Sequencer)]
        AS1[开始] --> AS2{情绪值是否变化?};
        AS2 -- 是 --> AS3["<p>生成<b>新动作序列</b></p>Generate_Sequence_V4"];
        AS3 --> AS4[重置步数索引, 记录起始姿态];
        AS2 -- 否 --> AS5;
        AS4 --> AS5{当前序列是否有步骤?};
        AS5 -- 否 --> AS_End[结束];
        AS5 -- 是 --> AS6[获取当前步骤];
        AS6 --> AS7{当前步骤是否执行完毕?};
        AS7 -- 是 --> AS9[进入下一动作步骤 (循环)];
        AS9 --> AS8;
        AS7 -- 否 --> AS8;
        AS8 --> AS10[计算缓动插值进度];
        AS10 --> AS11[根据插值计算舵机目标脉冲];
        AS11 --> AS12[更新舵机基础姿态变量];
        AS12 --> AS_End;
    end

    subgraph sequence_generation [动作序列生成模块 (Generate_Sequence_V4)]
        GS1[开始] --> GS2{判断情绪值 moodValue 范围};
        GS2 -- 兴奋[11, 20] --> GS3[选'积极'动作池, 组合4-6个];
        GS2 -- 开心[1, 10] --> GS4[选'积极'动作池, 组合3-4个];
        GS2 -- 害怕[-20, -11] --> GS5[选'消极'动作池, 组合4-6个];
        GS2 -- 沮丧[-10, -1] --> GS6[选'消极'动作池, 组合2-3个];
        GS2 -- 中性[0] --> GS7{95%概率?};
        GS7 -- 是 --> GS8[生成抬头平视];
        GS7 -- 否 --> GS9[从'中性'池选1个动作];
        GS3 & GS4 & GS5 & GS6 --> GS10[随机从选定池中挑选并组合多个原子动作];
        GS8 & GS9 & GS10 --> GS13[修正序列: 将占位符-1替换为前一帧的有效位置];
        GS13 --> GS14{序列是否为空?};
        GS14 -- 是 --> GS15[添加默认回中动作];
        GS14 & GS15 --> GS_End[结束];
    end
    
    subgraph purring [呼噜声模块 (Execute_Rhythmic_Purring)]
        P1{情绪 > 0?} --否--> P2[关闭马达, 重置状态] --> P_End
        P1 --是--> P3{状态机判断}
        P3 -- IDLE --> P4{随机触发?} --是--> P5[状态->PURRING, 启动马达]
        P3 -- PURRING --> P6{已持续10秒?} --是--> P7[状态->PAUSE, 关闭马达]
        P3 -- PAUSE --> P8{已暂停0.5秒?} --是--> P9[状态->IDLE]
        P4 & P5 & P6 & P7 & P8 & P9 --> P_End[结束]
    end

    subgraph breathing [呼吸动作模块 (Execute_Breathing_V2)]
        BR1[开始] --> BR2[计算4秒呼吸周期中的进度];
        BR2 --> BR3{进度 < 吸气比例?};
        BR3 -- 是 (吸气) --> BR4[根据sin曲线计算偏移量];
        BR3 -- 否 (呼气) --> BR5[根据cos曲线计算偏移量];
        BR4 & BR5 --> BR6[将呼吸偏移量叠加到点头舵机];
        BR6 --> BR7[设置最终PWM值];
    end

    subgraph uart_commands [UART指令处理模块 (Process_UART_Commands)]
        U1[开始] --> U2[在接收环形缓冲区中查找包头 0xA5];
        U2 -- 未找到 --> U_End[结束];
        U2 -- 找到 --> U3[读取指令ID, 确定预期包长];
        U3 --> U6{缓冲区数据是否足够?};
        U6 -- 否 --> U_End;
        U6 -- 是 --> U7[计算校验和];
        U7 --> U8{校验和与包尾是否正确?};
        U8 -- 否 --> U9[丢弃此包];
        U8 -- 是 --> U10{解析指令 payload};
        U10 -- 指令10: 单点测试 --> U12[调用 Start_Servo_Test];
        U10 -- 指令11: 区间测试 --> U13[调用 Start_Servo_Range_Test];
        U12 & U13 --> U14[更新缓冲区读取指针, 标记包已处理];
    end

    subgraph test_mode [测试模式模块 (Execute_Test_Movement)]
        T1[开始] --> T2{判断测试状态};
        T2 -- PREPARE (准备) --> T3[计算: 从当前位置 -> 测试起点A];
        T2 -- FORWARD (去程) --> T4[计算: 从起点A -> 终点B];
        T2 -- RETURN (返程) --> T5[计算: 从终点B -> 起点A];
        T3 & T4 & T5 --> T6[计算缓动插值进度];
        T6 --> T8[插值计算舵机当前目标位置];
        T8 --> T9[设置舵机PWM, 更新base值];
        T9 --> T10{移动是否完成?};
        T10 -- 否 --> T_End[结束];
        T10 -- 是 --> T11{更新状态机};
        T11 -- PREPARE完成 --> T12[状态 -> FORWARD];
        T11 -- FORWARD完成 --> T13{还有剩余次数?} --是--> T14[状态 -> RETURN]
        T13 --否--> T15[状态 -> IDLE, 结束测试]
        T11 -- RETURN完成 --> T16[次数--, 返回T13判断]
    end
    
    %% Click actions
    click B call/href "#init" "查看初始化流程"
    click F1 call/href "#handle_mood" "查看情绪处理逻辑"
    click F2 call/href "#action_sequencer" "查看动作执行逻辑"
    click AS3 call/href "#sequence_generation" "查看动作生成逻辑"
    click F3 call/href "#purring" "查看呼噜声逻辑"
    click F4 call/href "#breathing" "查看呼吸逻辑"
    click G call/href "#uart_commands" "查看指令处理逻辑"
    click E call/href "#test_mode" "查看测试模式逻辑"