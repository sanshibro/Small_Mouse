/* USER CODE BEGIN Header */ /**** ******************************************************************************** * @file              : main.c** * @brief             : Main program body (Final Corrected Version with All Original Functions Preserved)
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "hrtim.h"
#include "memorymap.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stdbool.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h> 
#include <math.h>
#include <math.h>  // 需要包含数学库

#define PI 3.14159265f
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct { float touch_threshold; float pat_threshold; } SensorThresholds;
typedef enum{ ZONE_IDLE, ZONE_PENDING, ZONE_COOLDOWN } ZoneState;
typedef struct{ ZoneState state; uint32_t pending_start_time; uint32_t cooldown_start_time; float last_voltage_value; } SensorZone;
typedef enum{ EVENT_NONE, EVENT_TOUCH, EVENT_PAT } EventType;



//一个动作步骤，在 duration_ms 时间内，将头部移动到 target_nod_pulse, target_shake_pulse 位置
typedef struct {
    int target_nod_pulse;
    int target_shake_pulse;
    uint32_t duration_ms;
} ActionStep;
//ActionSequence 动作序列，最多包含10个步骤。
typedef struct {
    ActionStep steps[10];    // 存储最多10个动作步骤的数组
    uint8_t num_steps;       // 当前实际有多少个步骤
} ActionSequence;
// 定义一个函数指针数组，方便随机调用
// 注意：这里的mood参数是为了保持函数签名一致，方便调用
typedef uint32_t (*AtomicActionFunc)(ActionSequence*, int8_t);



/* USER CODE BEGIN PFP */
 uint32_t Rise_Base = 10;

void Generate_Sequence_V4(int8_t mood, ActionSequence* seq);
void Execute_Breathing_V2(void);




/* USER CODE END PFP */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 128
/* USER CODE BEGIN PD */


// === 用于蓝牙遥控测试的新增宏定义 ===
// 帧头
#define PACKET_HEADER 0xA5


// 指令ID
#define CMD_SET_TOUCH_THRESHOLD 1
#define CMD_SET_PAT_THRESHOLD   2
#define CMD_SERVO_TEST          10 // "从当前位置到目标" 测试
#define CMD_SERVO_RANGE_TEST    11 // "在A点和B点之间" 测试
#define CMD_SERVO_TAILeadST      12er  新增：尾部舵机测试指令


#define CMD_PURR_CHANGE 101    //对应呼吸马达1
#define CMD_PURR_CHANGE_2 102    //对应呼吸马达2



/* USER CODE BEGIN PTD */

typedef enum {
    TEST_STATE_IDLE,
    TEST_STATE_PREPARE, // 移动到区间起点的准备状态
    TEST_STATE_FORWARD, // 去程
    TEST_STATE_RETURN   // 返程
} TestMovementState;

// === 用于测试模式状态机的全局变量 ===
volatile bool g_test_mode_active = false;
static volatile TestMovementState g_test_state = TEST_STATE_IDLE;   // 空闲
static uint32_t g_test_reps_remaining = 0;
static uint32_t g_test_move_start_time = 0;
static int      g_test_start_nod = 0, g_test_start_shake = 0;
static int      g_test_target_nod = 0, g_test_target_shake = 0;
static uint32_t g_test_duration = 1000;





void Start_Servo_Test(uint8_t axis_mode, int target_nod, int target_shake, uint32_t duration_ms, uint8_t repetitions);
void Start_Servo_Range_Test(uint8_t axis_mode, int nod_A, int nod_B, int shake_A, int shake_B, uint32_t duration_ms, uint8_t repetitions);
void Execute_Test_Movement(void);
void Execute_Test_Breath_Sensor_1(short zhendong_max,short zhendong_min,short xunhuancishu,int Rise_time,int Duration_Time,int Down_Time ,int Pause_Time );
void Execute_Test_Breath_Sensor_2(short zhendong_max,short zhendong_min,short xunhuancishu,int Rise_time,int Duration_Time,int Down_Time ,int Pause_Time );




//-----------------------呼吸调节---------------------------------------------------
typedef enum { PURR_Rise_Test, PURRING_Uncahnged_Test, PURR_Down_Test ,PURR_PAUSE_TEST ,PURR_FINISHED_TEST  } PurrState_Test;

    static uint32_t breath_loop_count = 0;
    static uint32_t max_breath_loop = 2;
    static PurrState_Test state = PURR_Rise_Test;
    static uint32_t Timer_Test = 0;
    static float phase = 0.0f;  // 相位变量，用于正弦波计算
   
    uint32_t Rise_MAX = 70;
    uint32_t Down_MIN = 20;
    
    uint32_t PurrTime_Rise = 5000;     // 上升阶段总时间
    uint32_t PurrTime_Uncahnged = 1000; // 保持阶段时间
    uint32_t PurrTime_Down = 10000;     // 下降阶段总时间
    uint32_t PurrTime_Pause = 2000;    // 暂停阶段时间

// --- Sensor_2 的状态变量 ---
typedef enum { PURR_Rise_Test_2, PURRING_Uncahnged_Test_2, PURR_Down_Test_2 ,PURR_PAUSE_TEST_2 ,PURR_FINISHED_TEST_2  } PurrState_Test_2;

static uint32_t breath_loop_count_2 = 0;
static uint32_t max_breath_loop_2 = 3;     
static PurrState_Test_2 state_2 = PURR_Rise_Test_2;
static uint32_t Timer_Test_2 = 0;
static float phase_2 = 0.0f;

// --- Sensor_2 的配置参数 ---
// 为了演示独立性，这里的参数值做了一些修改
uint32_t Rise_MAX_2 = 90;
uint32_t Down_MIN_2 = 10;
   
uint32_t PurrTime_Rise_2 = 3000;       
uint32_t PurrTime_Uncahnged_2 = 500;   
uint32_t PurrTime_Down_2 = 6000;      
uint32_t PurrTime_Pause_2 = 1000;




/* USER CODE END PTD */
// 轴选择
#define AXIS_BOTH  0
#define AXIS_NOD   1
#define AXIS_SHAKE 2
#define AXIS_Tail  3

/* USER CODE END PD */
/* 触摸区域宏定义 */
#define NUM_SENSORS 8
#define NUM_ZONES 4
#define HEAD_ZONE_IDX  0
#define BACK_ZONE_IDX  1
#define CHEEK_ZONE_IDX 2
#define BELLY_ZONE_IDX 3

/* 冷却时间宏定义 */
#define TOUCH_DURATION_MS   50
#define LOCAL_COOLDOWN_MS   200
#define GLOBAL_COOLDOWN_MS  1500
#define DECAY_INTERVAL_MS   10000

/* 最大最小心情值宏定义 */
#define MOOD_VALUE_MAX 20
#define MOOD_VALUE_MIN -20

/* 舵机极限宏定义 */
#define SERVO_MIN 500
#define SERVO_MAX 2500
#define SERVO_MID 800


#define SERVO_NOD_UP 1500
#define SERVO_NOD_DOWN 1700
#define SERVO_NOD_MID 1500


#define PI 3.14159265f


// 舵机1: 摇头 (左右) - Shake
#define POS_SHAKE_CENTER      1500
#define POS_SHAKE_LEFT_M      1700 // 中等幅度左
#define POS_SHAKE_RIGHT_M     1250 // 中等幅度右
#define POS_SHAKE_LEFT_L      1900 // 大幅度左
#define POS_SHAKE_RIGHT_L     1000  // 大幅度右

// 舵机2: 点头 (上下) - Nod
#define POS_NOD_TOP           950  // 仰头最高点
#define POS_NOD_CENTER        1500 // 平视
#define POS_NOD_BOTTOM        1700 // 低头最低点
#define POS_NOD_DEEP_BOTTOM   1900 // 沮丧时更低的低头

/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */


uint16_t ADC1_ConvertedValue[NUM_SENSORS];
volatile uint8_t ADC1_DMA_TransferComplete = 0;
SensorThresholds sensor_thresholds[NUM_SENSORS];
SensorZone zones[NUM_ZONES];

volatile int8_t moodValue = 0;

uint32_t lastEventTimestamp = 0;
uint32_t lastInteractionTimestamp = 0;

uint8_t rx_data;
uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t Packet[1 + 8*4 + 1 +1 ];

volatile uint16_t rx_read_ptr = 0;
volatile uint16_t rx_write_ptr = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
void Process_UART_Commands(void);
void Init_Interaction_System(void);
void Execute_Action_Sequencer(int8_t new_mood);
void Execute_Breathing(void);
void Execute_Rhythmic_Purring(int8_t mood);
void handle_mood_logic_Quanju(void);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ======================= 系统1: 情绪动作生成与播放 =======================
static int8_t   current_action_mood = 21; // 用一个不可能的心情值来强制首次更新
static uint8_t  current_step_index = 0;
static uint32_t step_start_time = 0;
static int      start_nod_pulse = SERVO_NOD_UP;
static int      start_shake_pulse = SERVO_MID;
static ActionSequence current_sequence;
static int base_nod_pulse = SERVO_NOD_UP; 
static int base_shake_pulse = SERVO_MID;
static int base_tail_pulse = 1500; // 假设尾部舵机中位值是1500

//一个缓动函数。它将一个线性的进度（0.0到1.0）转换为一个“先快后慢”的曲线。
//这使得机器人的动作看起来更平滑、更自然，而不是僵硬的线性移动。   1 - (1 - x)^3
/*输入 x	(1-x)	(1-x)³	1-(1-x)³ (输出)	运动特性
0.0	1.0	1.0    	0.0	    起点
0.2	0.8	0.512	0.488	较快
0.5	0.5	0.125	0.875	快速
0.8	0.2	0.008	0.992	很慢
1.0	0.0	0.0	     1.0	终点
*/
float easeOutCubic(float x) {
    if (x >= 1.0f) return 1.0f;
    return 1.0f - powf(1.0f - x, 3.0f);
}
/**
 * @brief  将一个值约束在一个安全范围内。
 * @param  value: 需要约束的值。
 * @param  min: 允许的最小值。
 * @param  max: 允许的最大值。
 * @retval 被约束后的安全值。
 */
int clamp(int value, int min, int max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

void add_move(ActionSequence* seq, int nod, int shake, uint32_t duration) {
    if (seq->num_steps >= 10) return;
    seq->steps[seq->num_steps++] = (ActionStep){nod, shake, duration};
}

/* USER CODE BEGIN 0 */

// ======================= 2. 微指令动作库 (Micro-instruction Library) =======================
// 每个函数只负责向序列中添加一个基础动作，并返回执行该动作所需的时间
// 可进行自由组合

// --- 基础移动指令 ---
uint32_t move_to(ActionSequence* seq, int nod, int shake, uint32_t duration) {
    add_move(seq, nod, shake, duration);
    return duration;
}
// --- 姿态指令 (Nod 轴) ---
uint32_t act_Nod_Top(ActionSequence* seq, int8_t mood) {
    uint32_t duration = (300 - abs(mood) * 10) * 3; // 速度变为1/3
    if (duration < 300) duration = 300;
    return move_to(seq, POS_NOD_TOP, -1, duration);
}
uint32_t act_Nod_Center(ActionSequence* seq, int8_t mood) {
    uint32_t duration = (400 - abs(mood) * 10) * 3; // 速度变为1/3
    if (duration < 450) duration = 450;
    return move_to(seq, POS_NOD_CENTER, -1, duration);
}
uint32_t act_Nod_Bottom(ActionSequence* seq, int8_t mood) {
    uint32_t duration = (350 - abs(mood) * 10) * 3; // 速度变为1/3
    if (duration < 360) duration = 360;
    return move_to(seq, POS_NOD_BOTTOM, -1, duration);
}
uint32_t act_Nod_Deep_Bottom(ActionSequence* seq, int8_t mood) {
    uint32_t duration = (1000 + abs(mood) * 20) * 3; // 速度变为1/3
    return move_to(seq, POS_NOD_DEEP_BOTTOM, POS_SHAKE_CENTER, duration);
}


// --- 姿态指令 (Shake 轴) ---
uint32_t act_Shake_Left(ActionSequence* seq, int8_t mood) {
    uint32_t duration = (300 - abs(mood) * 10) * 3; // 速度变为1/3
    if (duration < 300) duration = 300;
    return move_to(seq, -1, POS_SHAKE_LEFT_L, duration);
}
uint32_t act_Shake_Right(ActionSequence* seq, int8_t mood) {
    uint32_t duration = (300 - abs(mood) * 10) * 3; // 速度变为1/3
    if (duration < 300) duration = 300;
    return move_to(seq, -1, POS_SHAKE_RIGHT_L, duration);
}
uint32_t act_Shake_Center(ActionSequence* seq, int8_t mood) {
    uint32_t duration = (400 - abs(mood) * 10) * 3; // 速度变为1/3
    if (duration < 450) duration = 450;
    return move_to(seq, -1, POS_SHAKE_CENTER, duration);
}

// --- 复合/对角线指令 ---
uint32_t act_Look_TopLeft(ActionSequence* seq, int8_t mood) {
    uint32_t duration = (450 - abs(mood) * 12) * 3; // 速度变为1/3
    if (duration < 600) duration = 600;
    return move_to(seq, POS_NOD_TOP, POS_SHAKE_LEFT_M, duration);
}
uint32_t act_Look_TopRight(ActionSequence* seq, int8_t mood) {
    uint32_t duration = (450 - abs(mood) * 12) * 3; // 速度变为1/3
    if (duration < 600) duration = 600;
    return move_to(seq, POS_NOD_TOP, POS_SHAKE_RIGHT_M, duration);
}
uint32_t act_Look_BottomLeft(ActionSequence* seq, int8_t mood) {
    uint32_t duration = (450 - abs(mood) * 12) * 3; // 速度变为1/3
    if (duration < 600) duration = 600;
    return move_to(seq, POS_NOD_BOTTOM, POS_SHAKE_LEFT_M, duration);
}
uint32_t act_Look_BottomRight(ActionSequence* seq, int8_t mood) {
    uint32_t duration = (450 - abs(mood) * 12) * 3; // 速度变为1/3
    if (duration < 600) duration = 600;
    return move_to(seq, POS_NOD_BOTTOM, POS_SHAKE_RIGHT_M, duration);
}

// --- 特殊/动态指令 ---
uint32_t act_Circle_Head(ActionSequence* seq, int8_t mood) {
    uint32_t speed = (250 - abs(mood) * 5) * 3; // 速度变为1/3
    if (speed < 450) speed = 450;
    move_to(seq, POS_NOD_CENTER, POS_SHAKE_LEFT_M, speed);
    move_to(seq, POS_NOD_TOP, POS_SHAKE_CENTER, speed);
    move_to(seq, POS_NOD_CENTER, POS_SHAKE_RIGHT_M, speed);
    move_to(seq, POS_NOD_BOTTOM, POS_SHAKE_CENTER, speed);
    move_to(seq, POS_NOD_CENTER, POS_SHAKE_CENTER, speed);
    return speed * 5;
}

uint32_t act_Quick_Nod(ActionSequence* seq, int8_t mood) {
    uint32_t speed = (180 - abs(mood) * 4) * 3; // 速度变为1/3
    if (speed < 300) speed = 300;
    move_to(seq, POS_NOD_BOTTOM, -1, speed);
    move_to(seq, POS_NOD_CENTER, -1, speed);
    return speed * 2;
}

uint32_t act_Quick_Shake(ActionSequence* seq, int8_t mood) {
    uint32_t speed = (180 - abs(mood) * 4) * 3; // 速度变为1/3
    if (speed < 300) speed = 300;
    move_to(seq, -1, POS_SHAKE_LEFT_L, speed);
    move_to(seq, -1, POS_SHAKE_RIGHT_L, speed);
    move_to(seq, -1, POS_SHAKE_CENTER, speed);
    return speed * 3;
}

uint32_t act_Hold_Pose(ActionSequence* seq, int8_t mood) {
    uint32_t duration = (500 + (rand() % 500)) * 3; // 暂停时间也变为3倍
    move_to(seq, -1, -1, duration);
    return duration;
}

// _----------------------- 微指令动作池 (Micro-instruction Pools) --------------------
// 积极动作池
const AtomicActionFunc positive_actions[] = {
    act_Nod_Top, act_Nod_Center, act_Quick_Nod, act_Shake_Left, act_Shake_Right,
    act_Look_TopLeft, act_Look_TopRight, act_Circle_Head, act_Hold_Pose
};
const int num_positive_actions = sizeof(positive_actions) / sizeof(AtomicActionFunc);

// 消极动作池
const AtomicActionFunc negative_actions[] = {
    act_Nod_Bottom, act_Nod_Deep_Bottom, act_Shake_Left, act_Shake_Right,
    act_Look_BottomLeft, act_Look_BottomRight, act_Quick_Shake, act_Hold_Pose
};
const int num_negative_actions = sizeof(negative_actions) / sizeof(AtomicActionFunc);

// 中性/好奇动作池
const AtomicActionFunc neutral_actions[] = {
    act_Nod_Center, act_Shake_Left, act_Shake_Right, act_Look_TopLeft,
    act_Look_TopRight, act_Hold_Pose
};
const int num_neutral_actions = sizeof(neutral_actions) / sizeof(AtomicActionFunc);

/**
 * @brief 根据当前情绪，从微指令池中随机组合动作，生成一个全新的、不可预测的动作序列 (V4)
 * @param mood 当前心情值 (-20 to +20)
 * @param seq  指向要填充的动作序列的指针
 */
void Generate_Sequence_V4(int8_t mood, ActionSequence* seq) {
    seq->num_steps = 0;
    
    // 1. 根据情绪选择合适的微指令池
    const AtomicActionFunc* action_pool;
    int pool_size;
    int num_moves_to_combine;

    if (mood > 10) { // [11, 20] 兴奋、活泼
        action_pool = positive_actions;
        pool_size = num_positive_actions;
        num_moves_to_combine = 4 + (rand() % 3); // 组合 4-6 个快速动作
    } else if (mood > 0) { // [1, 10] 开心、好奇
        action_pool = positive_actions;
        pool_size = num_positive_actions;
        num_moves_to_combine = 3 + (rand() % 2); // 组合 3-4 个动作
    } else if (mood < -10) { // [-20, -11] 害怕、激动
        action_pool = negative_actions;
        pool_size = num_negative_actions;
        num_moves_to_combine = 4 + (rand() % 3); // 组合 4-6 个快速、退缩的动作
    } else if (mood < 0) { // [-10, -1] 沮丧、躲避
        action_pool = negative_actions;
        pool_size = num_negative_actions;
        num_moves_to_combine = 2 + (rand() % 2); // 组合 2-3 个慢速、低沉的动作
    } else { // mood == 0 中性/待机
        // 95% 的概率保持抬头平视的姿态，有轻微的随机探索
     add_move(seq, POS_NOD_CENTER, POS_SHAKE_CENTER, 2000); // 使用2000毫秒（2秒）平滑地回到初始姿态
    return;
        return;
    }
    
    // 2. 随机挑选并组合微指令
    int last_nod = -1, last_shake = -1; // 用于获取初始姿态
    if(current_sequence.num_steps > 0) {
        last_nod = current_sequence.steps[current_step_index].target_nod_pulse;
        last_shake = current_sequence.steps[current_step_index].target_shake_pulse;
    }


    for (int i = 0; i < num_moves_to_combine; ++i) {
        if (seq->num_steps >= 10) break; // 防止序列溢出
        
        int random_index = rand() % pool_size;
        
        // 调用随机选中的原子动作函数
        // 这个函数会自己把动作步骤添加到seq中
        action_pool[random_index](seq, mood);
    }
    
    // 3. 修正序列中的-1值
    // 我们的原子动作可能会传入-1，表示“保持上一轴向的位置”
    // 这里我们把它解析成具体数值
    for(int i = 0; i < seq->num_steps; ++i) {
        if(seq->steps[i].target_nod_pulse == -1) {
            seq->steps[i].target_nod_pulse = (i > 0) ? seq->steps[i-1].target_nod_pulse : last_nod;
        }
        if(seq->steps[i].target_shake_pulse == -1) {
            seq->steps[i].target_shake_pulse = (i > 0) ? seq->steps[i-1].target_shake_pulse : last_shake;
        }
    }
    
    // 4. 如果因为某些原因序列是空的，添加一个默认动作
    if (seq->num_steps == 0) {
        add_move(seq, POS_NOD_CENTER, POS_SHAKE_CENTER, 2000);
    }
}



void Execute_Action_Sequencer(int8_t new_mood) {
    if (new_mood != current_action_mood) {
        current_action_mood = new_mood;
        
        // 修改这一行
        Generate_Sequence_V4(current_action_mood, &current_sequence);
        
        current_step_index = 0;
        step_start_time = HAL_GetTick();
        start_nod_pulse = base_nod_pulse;
        start_shake_pulse = base_shake_pulse;
    }

    if (current_sequence.num_steps == 0) return;

    const ActionStep* current_step = &current_sequence.steps[current_step_index];
    uint32_t current_time = HAL_GetTick();
    // 这里就体现出了duration 的作用了 调节时间用的 同时扔进那个数组里 
    if (current_time - step_start_time >= current_step->duration_ms) {
        step_start_time = current_time;
        start_nod_pulse = current_step->target_nod_pulse;
        start_shake_pulse = current_step->target_shake_pulse;
        // 移动到下一个动作，如果到底则循环
        current_step_index = (current_step_index + 1) % current_sequence.num_steps;
        // 更新指针到新的当前步骤
        current_step = &current_sequence.steps[current_step_index]; 
    }

    // *** 核心修改点：引入缓动函数 ***
    uint32_t time_in_step = current_time - step_start_time;
    float progress = 0.0f;
    if (current_step->duration_ms > 0) {
        progress = (float)time_in_step / (float)current_step->duration_ms;
    }
    if (progress > 1.0f) progress = 1.0f;

    // 将线性进度通过缓动函数映射，得到非线性进度
    float eased_progress = easeOutCubic(progress);

    // 使用缓动后的进度来计算舵机目标值
    base_nod_pulse = (int)(start_nod_pulse + (current_step->target_nod_pulse - start_nod_pulse) * eased_progress);
    base_shake_pulse = (int)(start_shake_pulse + (current_step->target_shake_pulse - start_shake_pulse) * eased_progress);
   
		int final_nod_pulse = clamp(base_nod_pulse, POS_NOD_TOP, POS_NOD_DEEP_BOTTOM);
    int final_shake_pulse = clamp(base_shake_pulse, POS_SHAKE_RIGHT_L, POS_SHAKE_LEFT_L); // 同样为摇头舵机增加保护
		


    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, final_nod_pulse );
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, final_shake_pulse);
}





// --- 系统2: 背景呼吸系统 ---
void Execute_Breathing_V2(void) {
    // 模仿真实呼吸：吸气约1.5秒，呼气约2.5秒，总周期4秒
    float breathing_cycle_ms = 4000.0f;
    float inhale_fraction = 0.375f; // 吸气占总周期的比例 (1.5s / 4.0s)

    int breathing_amplitude = 25; // 呼吸起伏幅度，可以根据需要调整

    uint32_t tick = HAL_GetTick();
    float progress = fmodf((float)tick, breathing_cycle_ms) / breathing_cycle_ms;

    float breathing_offset;

    if (progress < inhale_fraction) {
        // --- 吸气阶段 ---
        // 使用sin曲线的前半部分 (0 to PI/2), 由缓到快的吸气过程
        float inhale_progress = progress / inhale_fraction; // 归一化到 [0, 1]
        breathing_offset = sinf(inhale_progress * PI / 2.0f) * breathing_amplitude;
    } else {
        // --- 呼气阶段 ---
        // 使用cos曲线的前半部分 (0 to PI/2), 快到缓的呼气过程
        float exhale_progress = (progress - inhale_fraction) / (1.0f - inhale_fraction); // 归一化到 [0, 1]
        breathing_offset = cosf(exhale_progress * PI / 2.0f) * breathing_amplitude;
    }

    // 将呼吸叠加在动作之上
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, base_nod_pulse + (int)breathing_offset);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, base_shake_pulse);
}

// --- 系统3: 节律性呼噜系统 ---
typedef enum { PURR_IDLE, PURRING, PURR_PAUSE } PurrState;
void Execute_Rhythmic_Purring(int8_t mood) {
    static PurrState state = PURR_IDLE;
    static uint32_t state_timer = 0;
    
    if (mood <= 0) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
        __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
        state = PURR_IDLE;
        return;
    }
    
    uint32_t now = HAL_GetTick();
    switch(state) {
        case PURR_IDLE:
            if ((rand() % 256) == 0) {
                 state = PURRING;
                 state_timer = now;
                 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 20);
                 __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 20);
            }
            break;
        case PURRING:
            if (now - state_timer > 2000) {
                state = PURR_PAUSE;
                state_timer = now;
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
                __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
            }
            break;
        case PURR_PAUSE:
            if (now - state_timer > 1000) {
                state = PURR_IDLE;
            }
            break;
    }
}


/**
 * @brief  ADC Conversion Complete Callback in non-blocking mode
 * @param  hadc: ADC handle
 * @retval None
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if(hadc->Instance == ADC1)
    {
        ADC1_DMA_TransferComplete = 1;
    }
}

/**
 * @brief  UART Reception Complete Callback
 * @param  huart: UART handle
 * @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // Check if buffer is full
        //如果“写指针前进一步”后的位置不等于读指针，说明缓冲区还有空间，可以写入数据。
        //如果相等，说明缓冲区已满，不能再写入，否则会覆盖未读数据。
        if (((rx_write_ptr + 1) % RX_BUFFER_SIZE) != rx_read_ptr)
        {
            rx_buffer[rx_write_ptr] = rx_data;
            //(rx_write_ptr + 1) % RX_BUFFER_SIZE：计算写指针“前进一步”后的位置， 
            //并用取模操作让它在缓冲区末尾时回到开头，实现循环。
            rx_write_ptr = (rx_write_ptr + 1) % RX_BUFFER_SIZE;
        }
        // Restart UART interrupt reception
        HAL_UART_Receive_IT(&huart1, &rx_data, 1);
    }
}

/**
 * @brief  Get raw ADC value for a specific channel.
 */
uint16_t Get_ADC_Value(uint8_t channel)
{
    return ADC1_ConvertedValue[channel];
}

/**
 * @brief  Get converted voltage for a specific ADC channel.
 */
float Get_ADC_Voltage(uint8_t channel, float vref_mv)
{
    uint16_t raw = ADC1_ConvertedValue[channel];
    return (raw / 65535.0f ) * vref_mv;
}

/**
 * @brief  Initializes the interaction system variables and thresholds.
 */
void Init_Interaction_System(void) {
    char str[64];

    // Default thresholds, can be overwritten by UART command
    sensor_thresholds[0].touch_threshold = 3.30f; sensor_thresholds[0].pat_threshold = 2.20f;
    sensor_thresholds[1].touch_threshold = 3.30f; sensor_thresholds[1].pat_threshold = 2.20f;
    sensor_thresholds[2].touch_threshold = 3.30f; sensor_thresholds[2].pat_threshold = 2.20f;
    sensor_thresholds[3].touch_threshold = 3.30f; sensor_thresholds[3].pat_threshold = 2.20f;
    sensor_thresholds[4].touch_threshold = 3.30f; sensor_thresholds[4].pat_threshold = 2.20f;
    sensor_thresholds[5].touch_threshold = 3.30f; sensor_thresholds[5].pat_threshold = 2.20f;
    sensor_thresholds[6].touch_threshold = 3.30f; sensor_thresholds[6].pat_threshold = 2.20f;
    sensor_thresholds[7].touch_threshold = 3.30f; sensor_thresholds[7].pat_threshold = 2.20f;

    // Initialize zone state machines
    for (uint8_t i = 0; i < NUM_ZONES; i++) {
        zones[i].state = ZONE_IDLE;
        zones[i].pending_start_time = 0;
        zones[i].cooldown_start_time = 0;
    }

    // Initialize global state values
    moodValue = 0;
    uint32_t now = HAL_GetTick();
    lastEventTimestamp = now;
    lastInteractionTimestamp = now;

    sprintf(str, "\r\nInteraction System Initialized.\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 100);
}


/**
 * @brief  UART1 _  [包头 0xA5] + [原始数据 N 字节] (4*8) + [校验和 1 字节] + [包尾 0x5A]
 * A5 9A 99 21 41 9A 99 31 41 9A 99 41 41 9A 99 51 41 9A 99 61 41 9A 99 71 41 CD CC 80 41 CD CC 88 41 2A 5A 
 * @param  None
 * @retval None
 */
/*
void Uart_BlueTooth(void)
{
		uint8_t Check_Sum = 0;
    uint8_t *ptr = Packet;


	if(ADC1_DMA_TransferComplete == 1)
    {
    //​​ptr++​​：后置自增运算符，先使用 ptr 的当前值，再将 ptr 指向下一个地址（等价于 ptr = ptr + 1）。
		*ptr++ = 0xA5;
		// 校验位
	    *ptr++ = (uint8_t)moodValue;

	  //​​小端格式（Little-Endian）​​：低字节在前（低地址），高字节在后（高地址）。
		for (uint8_t i = 0;i < 8;i++)
	  {
      //共享内存
			union{
						float f;
						uint8_t b[4];
			}conv;
			
			//当 union 中的 float f 被赋值后，
			//uint8_t b[4] 会自动按​​内存二进制形式​​拆解 f 的4个字节（因为 float 占4字节）。
			
		 conv.f =   (( ADC1_ConvertedValue[i]  / 65535.0f) *3.3f)			;
			
		for(uint8_t j = 0; j < 4; j++)
			{
			  *ptr = conv.b[j];
				Check_Sum += *ptr;
				ptr++;
				
			}
			
			
  	}
		
	}
		//----------------
	 	ADC1_DMA_TransferComplete = 0;
		*ptr++ = Check_Sum & 0xFF; //校验位
		*ptr++ = 0x5A; // 包尾
	  HAL_UART_Transmit(&huart1, Packet, sizeof(Packet), 0XFF);
}
*/
/* * 前提条件:
 * 1. moodValue 必须是一个全局变量，因为函数 Uart_BlueTooth(void) 没有接收参数。
 * 2. Packet 数组的大小必须足够容纳所有数据，即 36 字节。
 * 例如: uint8_t Packet[36];
 */
void Uart_BlueTooth(void)
{
    uint8_t Check_Sum = 0;
    uint8_t *ptr = Packet;

    if(ADC1_DMA_TransferComplete == 1)
    {
        // 1. 写入包头
        *ptr++ = 0xA5;

    
        // ===================================================================
        // 2. 写入 moodValue，将其加入校验和
        *ptr = (uint8_t)moodValue;
        Check_Sum += *ptr; 
        ptr++;

        // 3. 写入8个ADC浮点数数据
        for (uint8_t i = 0; i < 8; i++)
        {
            union {
                float f;
                uint8_t b[4];
            } conv;
            
            conv.f = ((ADC1_ConvertedValue[i] / 65535.0f) * 3.3f);
            
            for(uint8_t j = 0; j < 4; j++)
            {
                // ADC数据的校验和计算逻辑是正确的，保持不变
                *ptr = conv.b[j];
                Check_Sum += *ptr;
                ptr++;
            }
        }
        
        ADC1_DMA_TransferComplete = 0;
        
        // 4. 写入最终的校验和与包尾
        *ptr++ = Check_Sum & 0xFF; // 校验位
        *ptr++ = 0x5A;             // 包尾
        
        // 使用 (ptr - Packet) 计算包的实际大小
        uint16_t packetSize = ptr - Packet;
       // HAL_UART_Transmit(&huart1, Packet, packetSize, 0xFF);
    }
}

/**
 * @brief   Calculates the checksum for a portion of the UART circular buffer.
 * @param   buffer: Pointer to the circular buffer.
 * @param   start_index: The starting index of the data to be checksummed.
 * @param   len: The number of bytes to include in the checksum calculation.
 * @param   buffer_size: The total size of the circular buffer.
 * @return  The 8-bit calculated checksum.
 * @note    This function calculates a simple sum of bytes.
 */
uint8_t calculate_checksum(const uint8_t* buffer, uint16_t start_index, uint16_t len, uint16_t buffer_size)
{
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < len; i++)
    {
        uint16_t current_pos = (start_index + i) % buffer_size;
        checksum += buffer[current_pos];
    }
    return checksum;
}
/**
 * @brief  Processes commands received from UART 
 */
 int rx_buffer_test[7];
union { short i32; uint8_t b[2]; } zhendong_max, zhendong_min,xunhuancishu;
union { uint32_t u32; uint8_t b[4]; } Rise_time, Duration_Time,Down_Time,Pause_Time; //


union { short i32; uint8_t b[2]; } zhendong_max_2, zhendong_min_2,xunhuancishu_2;
union { uint32_t u32; uint8_t b[4]; } Rise_time_2, Duration_Time_2,Down_Time_2,Pause_Time_2; // 



void Process_UART_Commands(void)
{
    // 
    uint16_t current_pos = rx_read_ptr;
    //包头位置
    uint16_t packet_start_pos = 0;
    bool found_header = false;

    while (current_pos != rx_write_ptr)
    {
        if (rx_buffer[current_pos] == PACKET_HEADER)
        {
            packet_start_pos = current_pos;
            found_header = true;
            break;
        }
        current_pos = (current_pos + 1) % RX_BUFFER_SIZE;
    }

    if (!found_header) return;

    uint16_t available_data = (rx_write_ptr - packet_start_pos + RX_BUFFER_SIZE) % RX_BUFFER_SIZE;
    if (available_data < 2) return;
    
    uint16_t cmd_id_pos = (packet_start_pos + 1) % RX_BUFFER_SIZE;
    uint8_t cmd_id = rx_buffer[cmd_id_pos];
    uint16_t expected_packet_len = 0;
    
    // 2. Determine expected packet length based on command ID
    switch(cmd_id)
    {
        case CMD_SET_TOUCH_THRESHOLD:
					
        case CMD_SET_PAT_THRESHOLD:
            expected_packet_len = 36;
            break;
        case CMD_SERVO_TEST:
            expected_packet_len = 21; // MODIFIED
            break;
        case CMD_SERVO_RANGE_TEST:
            expected_packet_len = 29; // MODIFIED
            break;
				case CMD_PURR_CHANGE:
						expected_packet_len = 26;
			  break;
	    case CMD_PURR_CHANGE_2:
						expected_packet_len = 26;
			  break;
        default:
            rx_read_ptr = (packet_start_pos + 1) % RX_BUFFER_SIZE;
            return;
    }

    if (available_data < expected_packet_len) return;
    
    uint16_t checksum_data_len = expected_packet_len - 3;
    uint8_t calculated_checksum = calculate_checksum(rx_buffer, cmd_id_pos, checksum_data_len, RX_BUFFER_SIZE);
    uint8_t received_checksum = rx_buffer[(packet_start_pos + expected_packet_len - 2) % RX_BUFFER_SIZE];
    uint8_t received_footer = rx_buffer[(packet_start_pos + expected_packet_len - 1) % RX_BUFFER_SIZE];
    
    if (calculated_checksum == received_checksum && received_footer == 0x5A)
    {
        switch (cmd_id)
        {
            case CMD_SET_TOUCH_THRESHOLD:
							{
                    char str[128];
                    sprintf(str, "\r\nReceived Command 1: Update Touch Thresholds (Checksum OK).\r\n");
                    HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 100);

                    // The parsing loop itself is correct, assuming NUM_SENSORS is 8
                    for (int i = 0; i < NUM_SENSORS; i++) // Make sure NUM_SENSORS is defined as 8
                    {
                        union { float f; uint8_t b[4]; } conv;
                        for (int j = 0; j < 4; j++)
                        {
                            // Data starts at byte 2 (after header and cmd_id)
                            uint16_t byte_pos = (packet_start_pos + 2 + i * 4 + j) % RX_BUFFER_SIZE;
                            conv.b[j] = rx_buffer[byte_pos];
                        }
                        sensor_thresholds[i].touch_threshold = conv.f;
                        
                        sprintf(str, "  Sensor %d: Touch=%.2f\r\n", i, sensor_thresholds[i].touch_threshold);
                        HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 100);
                    }
                    
                    // Advance read pointer past the entire valid packet
                    rx_read_ptr = (packet_start_pos + expected_packet_len) % RX_BUFFER_SIZE;
                    break; 
                }

						
            case CMD_SET_PAT_THRESHOLD:
            {
                    char str[128];
                    sprintf(str, "\r\nReceived Command 2: Update Pat Thresholds (Checksum OK).\r\n");
                    HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 100);

                    for (int i = 0; i < NUM_SENSORS; i++) // Make sure NUM_SENSORS is defined as 8
                    {
                        union { float f; uint8_t b[4]; } conv;
                        for (int j = 0; j < 4; j++)
                        {
                            uint16_t byte_pos = (packet_start_pos + 2 + i * 4 + j) % RX_BUFFER_SIZE;
                            conv.b[j] = rx_buffer[byte_pos];
                        }
                        sensor_thresholds[i].pat_threshold = conv.f;
                        
                        sprintf(str, "  Sensor %d: Pat=%.2f\r\n", i, sensor_thresholds[i].pat_threshold);
                        HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 100);
                    }
                    
                    // Advance read pointer past the entire valid packet
                    rx_read_ptr = (packet_start_pos + expected_packet_len) % RX_BUFFER_SIZE;
                    break; 
                }


            case CMD_SERVO_TEST: // case 10
            {
                union { int32_t i32; uint8_t b[4]; } nod_conv, shake_conv;
                union { uint32_t u32; uint8_t b[4]; } duration_conv, reps_conv; // MODIFIED
                uint8_t axis_mode = rx_buffer[(packet_start_pos + 2) % RX_BUFFER_SIZE];
                
                for(int j=0; j<4; j++) nod_conv.b[j]      = rx_buffer[(packet_start_pos + 3 + j) % RX_BUFFER_SIZE];
                for(int j=0; j<4; j++) shake_conv.b[j]    = rx_buffer[(packet_start_pos + 7 + j) % RX_BUFFER_SIZE];
                for(int j=0; j<4; j++) duration_conv.b[j] = rx_buffer[(packet_start_pos + 11 + j) % RX_BUFFER_SIZE];
                // MODIFIED: Parse 4 bytes for repetitions
                for(int j=0; j<4; j++) reps_conv.b[j]     = rx_buffer[(packet_start_pos + 15 + j) % RX_BUFFER_SIZE];
                
                Start_Servo_Test(axis_mode, nod_conv.i32, shake_conv.i32, duration_conv.u32, reps_conv.u32);
                break;
            }

            case CMD_SERVO_RANGE_TEST: // case 11
            {
                union { int32_t i32; uint8_t b[4]; } nodA_conv, nodB_conv, shakeA_conv, shakeB_conv;
                union { uint32_t u32; uint8_t b[4]; } duration_conv, reps_conv; // MODIFIED
                uint8_t axis_mode = rx_buffer[(packet_start_pos + 2) % RX_BUFFER_SIZE];

                for(int j=0; j<4; j++) nodA_conv.b[j]     = rx_buffer[(packet_start_pos + 3 + j) % RX_BUFFER_SIZE];
                for(int j=0; j<4; j++) nodB_conv.b[j]     = rx_buffer[(packet_start_pos + 7 + j) % RX_BUFFER_SIZE];
                for(int j=0; j<4; j++) shakeA_conv.b[j]   = rx_buffer[(packet_start_pos + 11 + j) % RX_BUFFER_SIZE];
                for(int j=0; j<4; j++) shakeB_conv.b[j]   = rx_buffer[(packet_start_pos + 15 + j) % RX_BUFFER_SIZE];
                for(int j=0; j<4; j++) duration_conv.b[j] = rx_buffer[(packet_start_pos + 19 + j) % RX_BUFFER_SIZE];
                // MODIFIED: Parse 4 bytes for repetitions
                for(int j=0; j<4; j++) reps_conv.b[j]     = rx_buffer[(packet_start_pos + 23 + j) % RX_BUFFER_SIZE];
                
                Start_Servo_Range_Test(axis_mode, nodA_conv.i32, nodB_conv.i32, shakeA_conv.i32, shakeB_conv.i32, duration_conv.u32, reps_conv.u32);
                break;
            }
						
			case CMD_PURR_CHANGE: // case 101
            {
             
								
                for(int j=0; j<2; j++) zhendong_max.b[j]  = rx_buffer[(packet_start_pos + 2 + j) % RX_BUFFER_SIZE];
								rx_buffer_test[0] = zhendong_max.i32;
                for(int j=0; j<2; j++) zhendong_min.b[j]  = rx_buffer[(packet_start_pos + 4 + j) % RX_BUFFER_SIZE];
								rx_buffer_test[1] = zhendong_min.i32;
                for(int j=0; j<2; j++) xunhuancishu.b[j]  = rx_buffer[(packet_start_pos + 6 + j) % RX_BUFFER_SIZE];
								rx_buffer_test[2] = xunhuancishu.i32;
                for(int j=0; j<4; j++) Rise_time.b[j]  = rx_buffer[(packet_start_pos + 8 + j) % RX_BUFFER_SIZE];
								rx_buffer_test[3] = Rise_time.u32;
                for(int j=0; j<4; j++) Duration_Time.b[j]  = rx_buffer[(packet_start_pos + 12 + j) % RX_BUFFER_SIZE];
				        rx_buffer_test[4] = Duration_Time.u32;
                for(int j=0; j<4; j++) Down_Time.b[j]  = rx_buffer[(packet_start_pos + 16 + j) % RX_BUFFER_SIZE];
								rx_buffer_test[5] = Down_Time.u32;
                for(int j=0; j<4; j++) Pause_Time.b[j]  = rx_buffer[(packet_start_pos + 20 + j) % RX_BUFFER_SIZE];
								rx_buffer_test[6] = Pause_Time.u32;

		        state = PURR_Rise_Test;
                breath_loop_count = 0;
				Timer_Test = HAL_GetTick();

                Execute_Test_Breath_Sensor_1(zhendong_max.i32,zhendong_min.i32,xunhuancishu.i32,Rise_time.u32,Duration_Time.u32,Down_Time.u32,Pause_Time.u32);

              //  Start_Servo_Test(axis_mode, nod_conv.i32, shake_conv.i32, duration_conv.u32, reps_conv.u32);
                break;
            }

            case CMD_PURR_CHANGE_2: // case 101
            {
             
								
                for(int j=0; j<2; j++) zhendong_max_2.b[j]  = rx_buffer[(packet_start_pos + 2 + j) % RX_BUFFER_SIZE];
								rx_buffer_test[0] = zhendong_max_2.i32;
                for(int j=0; j<2; j++) zhendong_min_2.b[j]  = rx_buffer[(packet_start_pos + 4 + j) % RX_BUFFER_SIZE];
								rx_buffer_test[1] = zhendong_min_2.i32;
                for(int j=0; j<2; j++) xunhuancishu_2.b[j]  = rx_buffer[(packet_start_pos + 6 + j) % RX_BUFFER_SIZE];
								rx_buffer_test[2] = xunhuancishu_2.i32;
                for(int j=0; j<4; j++) Rise_time_2.b[j]  = rx_buffer[(packet_start_pos + 8 + j) % RX_BUFFER_SIZE];
								rx_buffer_test[3] = Rise_time_2.u32;
                for(int j=0; j<4; j++) Duration_Time_2.b[j]  = rx_buffer[(packet_start_pos + 12 + j) % RX_BUFFER_SIZE];
				        rx_buffer_test[4] = Duration_Time_2.u32;
                for(int j=0; j<4; j++) Down_Time_2.b[j]  = rx_buffer[(packet_start_pos + 16 + j) % RX_BUFFER_SIZE];
								rx_buffer_test[5] = Down_Time_2.u32;
                for(int j=0; j<4; j++) Pause_Time_2.b[j]  = rx_buffer[(packet_start_pos + 20 + j) % RX_BUFFER_SIZE];
								rx_buffer_test[6] = Pause_Time_2.u32;

		        state_2 = PURR_Rise_Test_2;
                breath_loop_count_2 = 0;
				Timer_Test_2 = HAL_GetTick();


              //  Start_Servo_Test(axis_mode, nod_conv.i32, shake_conv.i32, duration_conv.u32, reps_conv.u32);
                break;
            }
        }
    }
    
    rx_read_ptr = (packet_start_pos + expected_packet_len) % RX_BUFFER_SIZE;}

/**
 * @brief  Handles the core interaction logic and mood updates.
 */
void handle_mood_logic_Quanju()
{
    uint32_t current_time = HAL_GetTick();

    // 1. Iterate through all interaction zones
    for (uint8_t i = 0; i < NUM_ZONES; ++i) {
        
        // Skip zone if it's in cooldown
        if (zones[i].state == ZONE_COOLDOWN) {
            if ((current_time - zones[i].cooldown_start_time) > LOCAL_COOLDOWN_MS) {
                zones[i].state = ZONE_IDLE; // Cooldown finished
            }
            continue;
        }

        EventType final_zone_event = EVENT_NONE;
        uint8_t sensor_start_idx = 0;
        uint8_t sensor_end_idx = 0;

        switch (i) {
            case HEAD_ZONE_IDX:  sensor_start_idx = 0; sensor_end_idx = 0; break;
            case CHEEK_ZONE_IDX: sensor_start_idx = 1; sensor_end_idx = 2; break;
            case BACK_ZONE_IDX:  sensor_start_idx = 3; sensor_end_idx = 5; break;
            case BELLY_ZONE_IDX: sensor_start_idx = 6; sensor_end_idx = 7; break;
        }

        // 2. Iterate through all sensors in the current zone
        for (uint8_t s_idx = sensor_start_idx; s_idx <= sensor_end_idx; ++s_idx) {
            float voltage = Get_ADC_Voltage(s_idx, 3.3f);
            EventType current_sensor_event = EVENT_NONE;

            // 3. Check event based on sensor's unique thresholds
            if (voltage < sensor_thresholds[s_idx].pat_threshold) {
                current_sensor_event = EVENT_PAT;
            } else if (voltage < sensor_thresholds[s_idx].touch_threshold) {
                current_sensor_event = EVENT_TOUCH;
            }

            // 4. Aggregate event based on "Pat Priority"
            if (current_sensor_event == EVENT_PAT) {
                final_zone_event = EVENT_PAT;
            } else if (current_sensor_event == EVENT_TOUCH && final_zone_event != EVENT_PAT) {
                final_zone_event = EVENT_TOUCH;
            }
        }
        
        // 5. Process the final event using the zone's state machine
        if (zones[i].state == ZONE_IDLE) {
            if (final_zone_event != EVENT_NONE) {
                zones[i].state = ZONE_PENDING;
                zones[i].pending_start_time = current_time;
            }
        }

        if (zones[i].state == ZONE_PENDING) {
            if (final_zone_event == EVENT_NONE) {
                zones[i].state = ZONE_IDLE; // Interaction interrupted
            } else {
                if (final_zone_event == EVENT_PAT) {
                    // Pat is an instant event, no duration check needed
                }
                else if (final_zone_event == EVENT_TOUCH) {
                    if ((current_time - zones[i].pending_start_time) < TOUCH_DURATION_MS) {
                        final_zone_event = EVENT_NONE; // Not long enough for a valid touch
                    }
                }
            }
        }
        
        // --- Trigger event and update mood ---
        if (final_zone_event != EVENT_NONE) {
            if ((current_time - lastEventTimestamp) > GLOBAL_COOLDOWN_MS) {
                if (final_zone_event == EVENT_TOUCH) {
                    if (moodValue < MOOD_VALUE_MAX) {
                        moodValue++;
                        char str[64];
                        sprintf(str, "\r\nZone %d touched! Mood +1, Mood is now %d", (int)i, moodValue);
                        HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 100);
                    }
                } else if (final_zone_event == EVENT_PAT) {
                    if (moodValue > MOOD_VALUE_MIN) {
                        moodValue--;
                        char str[64];
                        sprintf(str, "\r\nZone %d patted! Mood -1, Mood is now %d", (int)i, moodValue);
                        HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 100);
                    }
                }

                lastEventTimestamp = current_time;
                lastInteractionTimestamp = current_time;
                zones[i].state = ZONE_COOLDOWN;
                zones[i].cooldown_start_time = current_time;
                
                break; // "First come, first served"
            }
        }
    }

    // --- Natural Mood Decay ---
    if((current_time - lastInteractionTimestamp ) > DECAY_INTERVAL_MS) {
        if(moodValue > 0) {
            moodValue--;
            char str[64];
            sprintf(str,"\r\nNo one talked to me... Mood -1! Now Mood IS %d", moodValue);
            HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 100);
            lastInteractionTimestamp = current_time;
        } else if(moodValue < 0) {
            moodValue++;
            char str[64];
            sprintf(str,"\r\nSo bored... Self-recovery +1! Now Mood IS %d", moodValue);
            HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 100);
            lastInteractionTimestamp = current_time;
        }
    }
}


//==========测试区域===========================================
/* USER CODE BEGIN 0 */

/**
 * @brief  启动舵机“从当前到目标”测试模式 (对应指令10)
 */
void Start_Servo_Test(uint8_t axis_mode, int target_nod, int target_shake, uint32_t duration_ms, uint8_t repetitions)
{
    if (g_test_state != TEST_STATE_IDLE) return;
    
    int initial_nod = base_nod_pulse;
    int initial_shake = base_shake_pulse;
    
    // 调用更通用的“区间测试”启动函数，其中A点为当前位置
    Start_Servo_Range_Test(axis_mode, initial_nod, target_nod, initial_shake, target_shake, duration_ms, repetitions);
}

/**
 * @brief  启动舵机“区间”测试模式 (对应指令11)
 */
void Start_Servo_Range_Test(uint8_t axis_mode, int nod_A, int nod_B, int shake_A, int shake_B, uint32_t duration_ms, uint8_t repetitions)
{
    if (g_test_state != TEST_STATE_IDLE) return;
    
    if (axis_mode == AXIS_NOD) {
        g_test_start_nod = nod_A;
        g_test_target_nod = nod_B;
        g_test_start_shake = base_shake_pulse;
        g_test_target_shake = base_shake_pulse;
    } else if (axis_mode == AXIS_SHAKE) {
        g_test_start_nod = base_nod_pulse;
        g_test_target_nod = base_nod_pulse;
        g_test_start_shake = shake_A;
        g_test_target_shake = shake_B;
    } else if(axis_mode == AXIS_BOTH) { // AXIS_BOTH   如果不是base  就是要变的
        g_test_start_nod = nod_A;
        g_test_target_nod = nod_B;
        g_test_start_shake = shake_A;
        g_test_target_shake = shake_B;
    }
    else if (axis_mode == AXIS_Tail){
        




    }

    g_test_duration = duration_ms;
    g_test_reps_remaining = repetitions;

    g_test_move_start_time = HAL_GetTick();
    g_test_state = TEST_STATE_PREPARE;
    g_test_mode_active = true;
}


/**
 * @brief  执行独立的舵机测试动作 
 */
void Execute_Test_Movement(void)
{
    if (!g_test_mode_active) return;
    if (g_test_state == TEST_STATE_IDLE) {
        g_test_mode_active = false;
        return;
    }

    uint32_t current_time = HAL_GetTick();
    uint32_t time_in_step = current_time - g_test_move_start_time;
    
    float progress = 0.0f;
    uint32_t current_duration = g_test_duration;
    
    if(g_test_state == TEST_STATE_PREPARE) {
       current_duration = g_test_duration > 500 ? g_test_duration / 2 : 500;
    }
    //计算执行进度
    if (current_duration > 0) {
        progress = (float)time_in_step / (float)current_duration;
    }

    int from_nod, to_nod, from_shake, to_shake;

    switch(g_test_state) {
        case TEST_STATE_PREPARE:
            from_nod = base_nod_pulse; to_nod = g_test_start_nod;
            from_shake = base_shake_pulse; to_shake = g_test_start_shake;
            break;
        case TEST_STATE_FORWARD:
            from_nod = g_test_start_nod; to_nod = g_test_target_nod;
            from_shake = g_test_start_shake; to_shake = g_test_target_shake;
            break;
        case TEST_STATE_RETURN:
            from_nod = g_test_target_nod; to_nod = g_test_start_nod;
            from_shake = g_test_target_shake; to_shake = g_test_start_shake;
            break;
        default: return;
    }

    if (progress >= 1.0f) {
        progress = 1.0f;
        g_test_move_start_time = HAL_GetTick();

        if (g_test_state == TEST_STATE_PREPARE) {
            g_test_state = TEST_STATE_FORWARD;
        } else if (g_test_state == TEST_STATE_FORWARD) {
            if (g_test_reps_remaining > 0) g_test_state = TEST_STATE_RETURN;
            else g_test_state = TEST_STATE_IDLE;
        } else { // TEST_STATE_RETURN
            g_test_reps_remaining--;
            if (g_test_reps_remaining > 0) g_test_state = TEST_STATE_FORWARD;
            else g_test_state = TEST_STATE_IDLE;
        }
    }
    
    float eased_progress = easeOutCubic(progress);
    int current_nod = (int)(from_nod + (to_nod - from_nod) * eased_progress);
    int current_shake = (int)(from_shake + (to_shake - from_shake) * eased_progress);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, current_nod);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, current_shake);
    base_nod_pulse = current_nod;
    base_shake_pulse = current_shake;
    
    if (g_test_state == TEST_STATE_IDLE) {
        g_test_mode_active = false;
        char str[64];
        sprintf(str, "\r\nTest sequence complete.\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 100);
    }
}



void Execute_Test_Breath_Sensor_1(short zhendong_max,short zhendong_min,short xunhuancishu,int Rise_time,int Duration_Time,int Down_Time ,int Pause_Time )
{
	  Rise_MAX = zhendong_max;
    Down_MIN = zhendong_min;
		max_breath_loop = xunhuancishu;
		PurrTime_Rise = Rise_time;
		PurrTime_Uncahnged = Duration_Time;
		PurrTime_Down = Down_Time;
    PurrTime_Pause = Pause_Time;
     if (state == PURR_FINISHED_TEST) {
        return;
    }
 
		
    uint32_t Now_Time = HAL_GetTick();
    uint32_t elapsed_time = Now_Time - Timer_Test;
    uint32_t pwm_value;
    
    switch(state)
    {
        case PURR_Rise_Test:
            if(elapsed_time < PurrTime_Rise) {
                // 使用正弦波前半周期 (0到PI)
                phase = (float)elapsed_time / PurrTime_Rise * PI;
                pwm_value = Down_MIN + (Rise_MAX - Down_MIN) * (1.0f - cosf(phase)) / 2.0f;
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwm_value);
               // __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, pwm_value);
            }
            else {
                state = PURRING_Uncahnged_Test;
                Timer_Test = Now_Time;
            }
            break;
            
        case PURRING_Uncahnged_Test:
            if(elapsed_time < PurrTime_Uncahnged) {
                pwm_value = Rise_MAX;
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwm_value);
               // __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, pwm_value);
            }
            else {
                state = PURR_Down_Test;
                Timer_Test = Now_Time;
            }
            break;
           case PURR_Down_Test:
            if(elapsed_time < PurrTime_Down) {
                // 使用正弦波后半周期 (PI到2PI)
                phase = PI + (float)elapsed_time / PurrTime_Down * PI;

              pwm_value = Down_MIN + (Rise_MAX - Down_MIN) * (1.0f - cosf(phase)) / 2.0f;   							
							__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwm_value);
            //    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, pwm_value);
            }
            else {
                state = PURR_PAUSE_TEST;
                Timer_Test = Now_Time;
            }
            break;
            
            
        case PURR_PAUSE_TEST:
            if(elapsed_time < PurrTime_Pause) {
                pwm_value = 20;
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwm_value);
               // __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, pwm_value);
            }
            else {
                breath_loop_count++;
                
                if(breath_loop_count >= max_breath_loop)
                {
				state = PURR_FINISHED_TEST;
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
               // __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
                } 
								else {
                    state = PURR_Rise_Test;
                    Timer_Test = Now_Time;
                 }




            }
            break;
			case PURR_FINISHED_TEST:
							
            break;
    }
}







/**
 * @brief 呼吸马达效果函数 - 控制第二路 (htim12)
 */
void Execute_Test_Breath_Sensor_2(short zhendong_max,short zhendong_min,short xunhuancishu,int Rise_time,int Duration_Time,int Down_Time ,int Pause_Time )
{
	  Rise_MAX_2 = zhendong_max;
    Down_MIN_2 = zhendong_min;
		max_breath_loop_2 = xunhuancishu;
		PurrTime_Rise_2 = Rise_time;
		PurrTime_Uncahnged_2 = Duration_Time;
		PurrTime_Down_2 = Down_Time;
        PurrTime_Pause_2 = Pause_Time;
    if (state_2 == PURR_FINISHED_TEST) {
        return;
    }
    
    uint32_t Now_Time = HAL_GetTick();
    uint32_t elapsed_time = Now_Time - Timer_Test_2;
    uint32_t pwm_value;
    
    switch(state_2)
    {
        case PURR_Rise_Test_2:
            if(elapsed_time < PurrTime_Rise_2) {
                phase_2 = (float)elapsed_time / PurrTime_Rise_2 * PI;
                pwm_value = Down_MIN_2 + (Rise_MAX_2 - Down_MIN_2) * (1.0f - cosf(phase_2)) / 2.0f;
                
                __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, pwm_value);
            }
            else {
                state_2 = PURRING_Uncahnged_Test_2;
                Timer_Test_2 = Now_Time;
            }
            break;
            
        case PURRING_Uncahnged_Test_2:
            if(elapsed_time < PurrTime_Uncahnged_2) {
                pwm_value = Rise_MAX_2;
                
                __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, pwm_value);
            }
            else {
                state_2 = PURR_Down_Test_2;
                Timer_Test_2 = Now_Time;
            }
            break;

        case PURR_Down_Test_2:
            if(elapsed_time < PurrTime_Down_2) {
                phase_2 = PI + (float)elapsed_time / PurrTime_Down_2 * PI;
                pwm_value = Down_MIN_2 + (Rise_MAX_2 - Down_MIN_2) * (1.0f - cosf(phase_2)) / 2.0f;
                
                __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, pwm_value);
            }
            else {
                state_2 = PURR_PAUSE_TEST_2;
                Timer_Test_2 = Now_Time;
            }
            break;
            
        case PURR_PAUSE_TEST_2:
            if(elapsed_time < PurrTime_Pause_2) {
                pwm_value = Down_MIN_2; // 保持在最低值
                
                __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, pwm_value);
            }
            else {
                breath_loop_count_2++;
                
                if(breath_loop_count_2 >= max_breath_loop_2)
                {
                    state_2 = PURR_FINISHED_TEST_2;
                    
                    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
                } 
                else {
                    state_2 = PURR_Rise_Test_2;
                    Timer_Test_2 = Now_Time;
                }
            }
            break;

        case PURR_FINISHED_TEST_2:
					
            break;
    }
}














/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)




{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_HRTIM_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM15_Init();
  MX_TIM12_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  
  srand(HAL_GetTick());

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1_ConvertedValue, NUM_SENSORS);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  
  Init_Interaction_System();
  
  HAL_UART_Receive_IT(&huart1, &rx_data, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    Process_UART_Commands();
    Uart_BlueTooth();

   Execute_Test_Breath_Sensor_1(zhendong_max.i32,zhendong_min.i32,xunhuancishu.i32,Rise_time.u32,Duration_Time.u32,Down_Time.u32,Pause_Time.u32);
   Execute_Test_Breath_Sensor_2(zhendong_max_2.i32,zhendong_min_2.i32,xunhuancishu_2.i32,Rise_time_2.u32,Duration_Time_2.u32,Down_Time_2.u32,Pause_Time_2.u32);

      //Execute_Test_Breath_Sensor_2();

    
    if (g_test_mode_active)
    {
        // 如果在测试模式，只执行测试动作函数
        Execute_Test_Movement();
    }
   /* else
    {
        // 如果不在测试模式，执行正常的交互和情绪系统
        handle_mood_logic_Quanju();
        Execute_Action_Sequencer(moodValue); 
        Execute_Rhythmic_Purring(moodValue); 
       // Execute_Breathing_V2(); 
    }**/
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* MPU Configuration */
void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};
  HAL_MPU_Disable();
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */









