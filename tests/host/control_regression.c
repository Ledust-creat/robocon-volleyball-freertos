/* Real production sources, with hardware/RTOS contracts supplied by host stubs.
 * The semaphore model exercises exclusion, not FreeRTOS priority inheritance.
 */
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <setjmp.h>
#include <limits.h>
#include "FreeRTOS.h"
#include "main.h"
#include "can.h"

struct HostQueue { size_t size; int present; uint8_t item[64]; };
CAN_HandleTypeDef hcan1 = {0}, hcan2 = {1};
TIM_HandleTypeDef htim3 = {TIM3}, htim4 = {TIM4};
static uint32_t host_tick;
static _Thread_local uint32_t host_primask, host_ipsr, host_basepri;
static CRITICAL_SECTION host_irq;
static int host_scheduler = taskSCHEDULER_RUNNING;
static int fail_queue, fail_task, fail_mutex, mailbox_full, hal_error;
static int config_error, queue_deletes, task_deletes;
static UBaseType_t observed_start_prio, observed_rc_prio, observed_chassis_prio, observed_hit_prio;
static const char *fail_task_name;
static void (*can_start_hook)(CAN_HandleTypeDef *can);
static int expect_fatal, stop_task;
static jmp_buf fatal_jump, task_jump;
static volatile LONG active_hal[2], hal_calls[2], hal_reentries;
static uint8_t last_data[2][8];
static uint32_t last_id[2], last_dlc[2];
static int yield_hal;

uint32_t __get_PRIMASK(void) { return host_primask; }
uint32_t __get_IPSR(void) { return host_ipsr; }
uint32_t __get_BASEPRI(void) { return host_basepri; }
void __disable_irq(void) {
    if (!host_primask) EnterCriticalSection(&host_irq);
    host_primask = 1;
}
void __enable_irq(void) {
    if (host_primask) { host_primask = 0; LeaveCriticalSection(&host_irq); }
}
uint32_t HAL_GetTick(void) { return host_tick; }
void HAL_IncTick(void) { host_tick++; }
void HAL_GPIO_TogglePin(void *port, uint32_t pin) { (void)port; (void)pin; }
void Error_Handler(void) {
    if (expect_fatal) longjmp(fatal_jump, 1);
    abort();
}
QueueHandle_t xQueueCreate(UBaseType_t length, UBaseType_t size) {
    assert(length == 1 && size <= 64);
    if (fail_queue) return NULL;
    struct HostQueue *q = calloc(1, sizeof(*q));
    assert(q); q->size = size; return q;
}
BaseType_t xQueueReceive(QueueHandle_t q, void *item, TickType_t wait) {
    (void)wait; assert(q);
    if (!q->present) return pdFALSE;
    memcpy(item, q->item, q->size); q->present = 0; return pdTRUE;
}
BaseType_t xQueueOverwrite(QueueHandle_t q, const void *item) {
    assert(q); memcpy(q->item, item, q->size); q->present = 1; return pdTRUE;
}
void vQueueDelete(QueueHandle_t q) { assert(q); free(q); queue_deletes++; }
BaseType_t xTaskCreate(TaskFunction_t fn, const char *name, configSTACK_DEPTH_TYPE size,
                       void *arg, UBaseType_t priority, TaskHandle_t *handle) {
    (void)fn; (void)size; (void)arg;
    if (!strcmp(name,"start_task")) observed_start_prio=priority;
    if (!strcmp(name,"rc_task")) observed_rc_prio=priority;
    if (!strcmp(name,"chassis_task")) observed_chassis_prio=priority;
    if (!strcmp(name,"hit_task")) observed_hit_prio=priority;
    if (fail_task || (fail_task_name && !strcmp(name,fail_task_name))) return pdFALSE;
    *handle = (void *)1; return pdPASS;
}
void vTaskDelete(TaskHandle_t handle) { (void)handle; task_deletes++; }
void vTaskDelay(TickType_t ticks) { (void)ticks; abort(); }
void vTaskDelayUntil(TickType_t *last, TickType_t period) {
    *last += period;
    if (stop_task) longjmp(task_jump, 1);
    abort();
}
TickType_t xTaskGetTickCount(void) { return host_tick; }
BaseType_t xTaskGetSchedulerState(void) { return host_scheduler; }
SemaphoreHandle_t xSemaphoreCreateMutexStatic(StaticSemaphore_t *storage) {
    if (fail_mutex) return NULL;
    storage->handle = CreateSemaphore(NULL, 1, 1, NULL);
    assert(storage->handle); return storage;
}
BaseType_t xSemaphoreTake(SemaphoreHandle_t sem, TickType_t ticks) {
    assert(ticks == 1 && !host_primask && !host_ipsr && !host_basepri);
    return WaitForSingleObject(sem->handle, ticks) == WAIT_OBJECT_0 ? pdTRUE : pdFALSE;
}
BaseType_t xSemaphoreGive(SemaphoreHandle_t sem) {
    assert(ReleaseSemaphore(sem->handle, 1, NULL)); return pdTRUE;
}
int HAL_CAN_ConfigFilter(CAN_HandleTypeDef *can, CAN_FilterTypeDef *filter) {
    (void)can; (void)filter; return config_error ? HAL_ERROR : HAL_OK;
}
int HAL_CAN_Start(CAN_HandleTypeDef *can) {
    if (can_start_hook) can_start_hook(can);
    return HAL_OK;
}
int HAL_CAN_ActivateNotification(CAN_HandleTypeDef *can, uint32_t mask) {
    (void)can; (void)mask; return HAL_OK;
}
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *can) {
    (void)can; return mailbox_full ? 0 : 3;
}
int HAL_CAN_AddTxMessage(CAN_HandleTypeDef *can, const CAN_TxHeaderTypeDef *header,
                         const uint8_t data[8], uint32_t *mailbox) {
    unsigned bus = can->bus;
    if (InterlockedIncrement(&active_hal[bus]) != 1) InterlockedIncrement(&hal_reentries);
    if (yield_hal) Sleep(0);
    memcpy(last_data[bus], data, 8);
    last_id[bus] = header->IDE == CAN_ID_STD ? header->StdId : header->ExtId;
    last_dlc[bus] = header->DLC;
    *mailbox = 1;
    InterlockedIncrement(&hal_calls[bus]);
    InterlockedDecrement(&active_hal[bus]);
    return hal_error ? HAL_ERROR : HAL_OK;
}
void BSP_RcUart_Start(void) { }

#include "../../BSP/bsp_can.c"
#include "../../DVE/dve_dji_motor.c"
#include "../../Math/pid.c"
#include "../../Math/math_utils.c"
#include "../../DVE/dve_t15.c"
#include "../../Task/chassis_task.c"
#include "../../Task/hit_task.c"
#include "../../Task/rc_task.c"
#include "../../USER/Src/start_task.c"
#include "../../USER/Src/Initialize.c"
#include "../../USER/Src/pid_tim.c"

static unsigned checks;
#define CHECK(x) do { checks++; if (!(x)) { \
    fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #x); exit(1); } } while (0)
static int close_float(float a, float b) { return fabsf(a-b) < 0.001f; }

static void feed(unsigned id, unsigned ecd, int rpm) {
    CAN_RxHeaderTypeDef h = {.StdId=id, .IDE=CAN_ID_STD, .RTR=CAN_RTR_DATA, .DLC=8};
    uint8_t data[8] = {(uint8_t)(ecd>>8), (uint8_t)ecd,
                       (uint8_t)((uint16_t)rpm>>8), (uint8_t)rpm, 0, 0, 30, 0};
    CHECK(DVE_DjiMotor_RxCallback(&hcan2, &h, data));
}
static DVE_DjiMotorFeedback_t get_fb(DVE_DjiMotorId_t id) {
    DVE_DjiMotorFeedback_t f;
    CHECK(DVE_DjiMotor_GetFeedback(id, &f) == DVE_DJI_OK); return f;
}
static void feed_hits(unsigned ecd) { for (unsigned id=0x206; id<=0x208; id++) feed(id,ecd,0); }
static void zero_group(void) { for (unsigned i=0; i<8; i++) CHECK(last_data[1][i] == 0); }
static void hit_cmd(uint8_t enable, uint8_t safe, uint8_t pressed) {
    Hit_Cmd_t cmd = {.enable=enable, .safe_to_arm=safe, .hit_pressed=pressed,
                     .preset_index=2, .pitch_speed_set=0};
    xQueueOverwrite(xHitCmdQueue,&cmd); Hit_ControlStep();
}

static void test_pid_tick(void) {
    pid_t angle={0}, speed={0};
    PID_Struct_Init(&angle,2,0,0,30000,1000,INIT);
    PID_Struct_Init(&speed,3,0,0,16384,1000,INIT);
    CHECK(PID_PROCESS_Double(&angle,&speed,10,1,2) == 48);
    CHECK(close_float(angle.out,18) && close_float(speed.out,48));
    angle.error[0]=angle.error[1]=angle.error[2]=99; angle.iout=5;
    pid_reset(&angle,2,0,0);
    CHECK(angle.error[0]==0 && angle.error[1]==0 && angle.error[2]==0 && angle.iout==0);
    host_tick=100;
    HAL_TIM_PeriodElapsedCallback(&htim3); CHECK(host_tick==100);
    HAL_TIM_PeriodElapsedCallback(&htim4); CHECK(host_tick==101);
    puts("PASS PID cascade/reset and TIM4-only HAL tick");
}

static DWORD WINAPI send_thread(void *arg) {
    uintptr_t mode=(uintptr_t)arg;
    CAN_HandleTypeDef *can = mode==2 ? &hcan1 : &hcan2;
    uint8_t data[8]={0};
    for (unsigned n=0; n<1000; n++) {
        data[0]=(uint8_t)n;
        BSP_CAN_Status_t status = BSP_CAN_SendStd8(can,mode==1?0x1FF:0x200,data);
        assert(status == BSP_CAN_OK || status == BSP_CAN_ERR_BUSY);
    }
    return 0;
}
static void test_can(void) {
    uint8_t data[4]={1,2,3,4}; BSP_CAN_TxStats_t stats;
    CHECK(BSP_CAN_SendStd(&hcan2,0x200,data,4)==BSP_CAN_ERR_NOT_INIT);
    fail_mutex=1; CHECK(BSP_CAN_InitTxSync()==BSP_CAN_ERR_NOT_INIT); fail_mutex=0;
    CHECK(BSP_CAN_InitTxSync()==BSP_CAN_OK);
    CHECK(BSP_CAN_InitTxSync()==BSP_CAN_OK);
    host_ipsr=1; CHECK(BSP_CAN_SendStd(&hcan2,0x200,data,4)==BSP_CAN_ERR_CONTEXT); host_ipsr=0;
    __disable_irq(); CHECK(BSP_CAN_SendStd(&hcan2,0x200,data,4)==BSP_CAN_ERR_CONTEXT); __enable_irq();
    host_basepri=1; CHECK(BSP_CAN_SendStd(&hcan2,0x200,data,4)==BSP_CAN_ERR_CONTEXT); host_basepri=0;
    host_scheduler=taskSCHEDULER_NOT_STARTED;
    CHECK(BSP_CAN_SendStd(&hcan2,0x200,data,4)==BSP_CAN_ERR_CONTEXT);
    host_scheduler=taskSCHEDULER_RUNNING;
    CHECK(BSP_CAN_SendStd(&hcan2,0x200,data,4)==BSP_CAN_OK);
    CHECK(memcmp(last_data[1],data,4)==0 && last_dlc[1]==4);
    for(unsigned i=4;i<8;i++) CHECK(last_data[1][i]==0);
    CHECK(BSP_CAN_SendExt(&hcan2,0x12345,NULL,0)==BSP_CAN_OK);
    CHECK(last_id[1]==0x12345 && last_dlc[1]==0); zero_group();
    CHECK(BSP_CAN_SendStd(&hcan2,0x800,data,4)==BSP_CAN_ERR_PARAM);
    CHECK(BSP_CAN_SendStd(&hcan2,0x200,data,9)==BSP_CAN_ERR_PARAM);
    CHECK(BSP_CAN_SendStd(&hcan2,0x200,NULL,4)==BSP_CAN_ERR_PARAM);
    mailbox_full=1; CHECK(BSP_CAN_SendStd(&hcan2,0x200,data,4)==BSP_CAN_ERR_BUSY); mailbox_full=0;
    hal_error=1; CHECK(BSP_CAN_SendStd(&hcan2,0x200,data,4)==BSP_CAN_ERR_HAL); hal_error=0;
    CHECK(xSemaphoreTake(s_tx_mutex[1],1)==pdTRUE);
    CHECK(BSP_CAN_SendStd(&hcan2,0x200,data,4)==BSP_CAN_ERR_BUSY);
    xSemaphoreGive(s_tx_mutex[1]);
    CHECK(BSP_CAN_SendStd(&hcan2,0x200,data,4)==BSP_CAN_OK);
    CHECK(BSP_CAN_GetTxStats(&hcan2,&stats)==BSP_CAN_OK);
    CHECK(stats.lock_timeout==1 && stats.mailbox_full==1 && stats.hal_error==1 && stats.context_error==4);
    yield_hal=1;
    HANDLE threads[3];
    for(uintptr_t i=0;i<3;i++) { threads[i]=CreateThread(NULL,0,send_thread,(void*)i,0,NULL); CHECK(threads[i]!=NULL); }
    CHECK(WaitForMultipleObjects(3,threads,TRUE,10000)==WAIT_OBJECT_0);
    for(unsigned i=0;i<3;i++) CloseHandle(threads[i]);
    yield_hal=0;
    CHECK(hal_reentries==0 && hal_calls[0]>0 && hal_calls[1]>0);
    puts("PASS CAN exclusion (3000 concurrent attempts), context, DLC and error paths");
}

static void frame_channel(uint8_t frame[25], unsigned channel, unsigned raw) {
    for(unsigned bit=0;bit<11;bit++) {
        unsigned offset=channel*11+bit; uint8_t mask=(uint8_t)(1U<<(offset%8));
        if(raw&(1U<<bit)) frame[1+offset/8]|=mask;
        else frame[1+offset/8]&=(uint8_t)~mask;
    }
}
static void make_frame(uint8_t frame[25]) {
    memset(frame,0,25); frame[0]=0x0F;
    for(unsigned c=0;c<10;c++) frame_channel(frame,c,992);
    frame_channel(frame,0,1200); frame_channel(frame,9,172);
}
static void test_remote(void) {
    uint8_t frame[25]; DVE_T15_Data_t rc;
    Chassis_MoveCmd_t chassis; Hit_Cmd_t hit;
    make_frame(frame);
    CHECK(DVE_T15_ParseFrame(frame,25,&rc) && rc.ch[0]==208);
    for(unsigned flag=4;flag<=8;flag+=4) {
        frame[23]=(uint8_t)flag;
        CHECK(!DVE_T15_ParseFrame(frame,25,&rc) && !rc.frame_valid);
        CHECK(flag==4 ? rc.frame_lost : rc.failsafe);
    }
    frame[23]=0; s_rc_recovery_wait=true;
    frame_channel(frame,9,1811);
    RC_ProcessFrame(frame,true,&chassis,&hit);
    CHECK(s_rc_recovery_wait && !chassis.enable && !hit.enable);
    frame_channel(frame,9,172); frame_channel(frame,7,1811);
    RC_ProcessFrame(frame,true,&chassis,&hit); CHECK(s_rc_recovery_wait);
    frame_channel(frame,7,992);
    RC_ProcessFrame(frame,true,&chassis,&hit);
    CHECK(!s_rc_recovery_wait && !chassis.enable && !hit.enable);
    RC_ProcessFrame(frame,true,&chassis,&hit);
    CHECK(chassis.enable && chassis.vx>0 && hit.enable && hit.safe_to_arm);
    frame[23]=8; RC_ProcessFrame(frame,true,&chassis,&hit);
    CHECK(s_rc_recovery_wait && !chassis.enable && !hit.enable && chassis.vx==0);
    frame[23]=0; frame_channel(frame,9,1811);
    RC_ProcessFrame(frame,true,&chassis,&hit); CHECK(!hit.enable && s_rc_recovery_wait);
    frame_channel(frame,9,172);
    RC_ProcessFrame(frame,true,&chassis,&hit); CHECK(!hit.enable);
    RC_ProcessFrame(frame,true,&chassis,&hit); CHECK(hit.enable);
    RC_ProcessFrame(frame,false,&chassis,&hit); CHECK(!chassis.enable && !hit.enable);
    frame[0]=0; CHECK(!DVE_T15_ParseFrame(frame,25,&rc));
    CHECK(!DVE_T15_ParseFrame(frame,24,&rc));
    puts("PASS SBUS failsafe/frame-lost and remote recovery acknowledgement");
}

static void test_position(void) {
    DVE_DjiMotorFeedback_t f;
    host_tick=100; DVE_DjiMotor_Init(&hcan2);
    f=get_fb(DJI_MOTOR_HIT_1); CHECK(!f.online && !f.position_valid && f.total_ecd==0);
    feed(0x206,8000,0); f=get_fb(DJI_MOTOR_HIT_1);
    CHECK(f.position_valid && f.total_ecd==0 && close_float(f.angle_deg,8000*360.0f/8192));
    __disable_irq(); f=get_fb(DJI_MOTOR_HIT_1); CHECK(__get_PRIMASK()==1); __enable_irq();
    host_tick++; feed(0x206,100,0); CHECK(get_fb(DJI_MOTOR_HIT_1).total_ecd==292);
    host_tick++; feed(0x206,8000,0); CHECK(get_fb(DJI_MOTOR_HIT_1).total_ecd==0);
    DVE_DjiMotor_Init(&hcan2); feed(0x206,0,0);
    for(unsigned turn=0;turn<3;turn++) for(unsigned n=1;n<=8;n++) { host_tick++; feed(0x206,(n%8)*1024,0); }
    CHECK(get_fb(DJI_MOTOR_HIT_1).total_ecd==3*8192);
    for(unsigned n=1;n<=8;n++) { host_tick++; feed(0x206,(8192-n*1024)%8192,0); }
    CHECK(get_fb(DJI_MOTOR_HIT_1).total_ecd==2*8192);
    feed(0x206,4096,0); CHECK(!get_fb(DJI_MOTOR_HIT_1).position_valid);
    feed(0x206,4100,0); CHECK(!get_fb(DJI_MOTOR_HIT_1).position_valid);
    DVE_DjiMotor_Init(&hcan2); feed(0x206,5000,0); feed(0x206,904,0);
    CHECK(!get_fb(DJI_MOTOR_HIT_1).position_valid);
    DVE_DjiMotor_Init(&hcan2); feed(0x206,1000,0);
    s_dji_motor[DJI_MOTOR_HIT_1].feedback.total_ecd=INT32_MAX-1;
    feed(0x206,1002,0); CHECK(!get_fb(DJI_MOTOR_HIT_1).position_valid);
    DVE_DjiMotor_Init(&hcan2); feed(0x206,1000,0);
    s_dji_motor[DJI_MOTOR_HIT_1].feedback.total_ecd=INT32_MIN+1;
    feed(0x206,998,0); CHECK(!get_fb(DJI_MOTOR_HIT_1).position_valid);
    DVE_DjiMotor_Init(&hcan2); feed(0x206,1000,0);
    CAN_RxHeaderTypeDef h={.StdId=0x206,.DLC=8}; uint8_t invalid[8]={0x20,0};
    CHECK(!DVE_DjiMotor_RxCallback(&hcan2,&h,invalid));
    host_tick+=100; CHECK(get_fb(DJI_MOTOR_HIT_1).position_valid);
    host_tick++; CHECK(!DVE_DjiMotor_IsOnline(DJI_MOTOR_HIT_1,100));
    CHECK(!get_fb(DJI_MOTOR_HIT_1).position_valid);
    feed(0x206,1001,0); CHECK(!get_fb(DJI_MOTOR_HIT_1).position_valid);
    DVE_DjiMotor_Init(&hcan2); feed(0x206,1000,0);
    host_tick+=101; feed(0x206,1001,0);
    CHECK(!get_fb(DJI_MOTOR_HIT_1).position_valid);
    DVE_DjiMotor_Init(&hcan2); host_tick=UINT32_MAX-4; feed(0x206,1000,0);
    host_tick=2; feed(0x206,1001,0);
    CHECK(get_fb(DJI_MOTOR_HIT_1).position_valid && get_fb(DJI_MOTOR_HIT_1).total_ecd==1);
    puts("PASS delayed first frame, per-frame multi-turn position, ambiguity, overflow and tick wrap");
}

static void test_hit(void) {
    host_tick=100; DVE_DjiMotor_Init(&hcan2); Hit_InitControl();
    hit_cmd(1,1,0); zero_group();
    CHECK(!s_positions_ready && !s_hit_motor[0].angle_inited);
    feed_hits(1000); hit_cmd(1,1,0); CHECK(s_positions_ready && s_hit_state==HIT_STATE_RETURN);
    CHECK(close_float(s_hit_motor[0].target_angle,-HIT_RETURN_STEP));
    for (uint8_t preset=0; preset<HIT_PRESET_NUM; preset++) {
        Hit_Cmd_t cmd={.enable=1,.hit_pressed=1,.preset_index=preset};
        xQueueOverwrite(xHitCmdQueue,&cmd); Hit_ControlStep();
        CHECK(s_hit_state==HIT_STATE_PUT_ANGLE);
        for (unsigned i=0;i<HIT_MOTOR_NUM;i++)
            CHECK(close_float(s_hit_motor[i].target_angle,s_hit_angle_table[preset][i]));
    }
    hit_cmd(1,1,0); CHECK(s_hit_state==HIT_STATE_RETURN);
    CHECK(close_float(s_hit_motor[0].target_angle,-15*19+HIT_RETURN_STEP));
    s_pitch_speed_pid.iout=30; s_hit_motor[0].speed_pid.error[1]=99;
    hit_cmd(0,0,0); CHECK(s_hit_state==HIT_STATE_WAIT_REMOTE); zero_group();
    CHECK(s_pitch_speed_pid.iout==0 && s_hit_motor[0].speed_pid.error[1]==0);
    host_tick++; feed_hits(1500);
    hit_cmd(1,0,1); CHECK(s_hit_state==HIT_STATE_WAIT_REMOTE); zero_group();
    hit_cmd(1,1,0); CHECK(s_hit_state==HIT_STATE_WAIT_REMOTE); zero_group();
    hit_cmd(1,1,0); CHECK(s_hit_state==HIT_STATE_RETURN);
    CHECK(close_float(s_hit_motor[0].target_angle,500*360.0f/8192-HIT_RETURN_STEP));
    host_tick+=101;
    Hit_ControlStep(); CHECK(s_hit_state==HIT_STATE_POSITION_FAULT); zero_group();
    feed_hits(1500); hit_cmd(1,1,0); CHECK(s_hit_state==HIT_STATE_POSITION_FAULT); zero_group();
    host_tick=200; DVE_DjiMotor_Init(&hcan2); Hit_InitControl(); feed_hits(1000);
    hit_cmd(1,1,0); hit_cmd(1,1,0);
    feed(0x205,1000,100); s_pitch_speed_pid.iout=500;
    for(unsigned n=0;n<11;n++) { host_tick+=10; feed_hits(1000); hit_cmd(1,1,0); }
    CHECK(s_hit_state!=HIT_STATE_POSITION_FAULT && s_pitch_speed_pid.iout==0);
    CHECK(last_data[1][0]==0 && last_data[1][1]==0);
    /* Keep positions fresh while withholding commands: command timeout is recoverable. */
    for(unsigned n=0;n<11;n++) { host_tick+=10; feed_hits(1000); Hit_ControlStep(); }
    CHECK(s_hit_state==HIT_STATE_WAIT_REMOTE); zero_group();
    hit_cmd(1,0,1); CHECK(s_hit_state==HIT_STATE_WAIT_REMOTE);
    hit_cmd(1,1,0); hit_cmd(1,1,0); CHECK(s_hit_state==HIT_STATE_RETURN);
    puts("PASS hit startup, held-switch recovery, measured-position return and latched feedback fault");
}

static void test_rc_hit_ack(void) {
    uint8_t frame[25]; Chassis_MoveCmd_t chassis; Hit_Cmd_t hit;
    host_tick=100; DVE_DjiMotor_Init(&hcan2); Hit_InitControl(); feed_hits(1000);
    make_frame(frame); s_rc_recovery_wait=true;
    RC_ProcessFrame(frame,true,&chassis,&hit);
    CHECK(!hit.enable && hit.safe_to_arm);
    xQueueOverwrite(xHitCmdQueue,&hit); Hit_ControlStep(); zero_group();
    CHECK(s_remote_confirmed && s_hit_state==HIT_STATE_WAIT_REMOTE);
    frame_channel(frame,9,1811);
    RC_ProcessFrame(frame,true,&chassis,&hit);
    xQueueOverwrite(xHitCmdQueue,&hit); Hit_ControlStep();
    CHECK(s_hit_state==HIT_STATE_PUT_ANGLE);
    puts("PASS first safe RC frame disables output, next frame accepts a new hit action");
}

static void inject_first_frame(CAN_HandleTypeDef *can) {
    if (can==&hcan2) { CHECK(s_dji_inited); feed(0x206,1000,0); }
}
static void test_startup_chassis(void) {
    /* Creation failures report upward, without calling the orchestrator. */
    if (xHitCmdQueue) { vQueueDelete(xHitCmdQueue); xHitCmdQueue=NULL; }
    g_startup_error=0;
    fail_queue=1;
    CHECK(chassis_task_creat()==TASK_CREATE_ERR_QUEUE && !xChassisCtrlQueue);
    CHECK(hit_task_creat()==TASK_CREATE_ERR_QUEUE && !xHitCmdQueue);
    CHECK(rc_task_creat()==TASK_CREATE_ERR_QUEUE && !xRCRawDataQueue);
    CHECK(g_startup_error==0); fail_queue=0;
    int released=queue_deletes;
    fail_task=1;
    CHECK(chassis_task_creat()==TASK_CREATE_ERR_TASK && !xChassisCtrlQueue);
    CHECK(hit_task_creat()==TASK_CREATE_ERR_TASK && !xHitCmdQueue);
    CHECK(rc_task_creat()==TASK_CREATE_ERR_TASK && !xRCRawDataQueue);
    CHECK(queue_deletes==released+3 && g_startup_error==0); fail_task=0;

    /* Board init also returns a status, and preserves feedback arriving at CAN start. */
    config_error=1; CHECK(!All_Init() && g_startup_error==0); config_error=0;
    can_start_hook=inject_first_frame;
    CHECK(All_Init()); CHECK(get_fb(DJI_MOTOR_HIT_1).online);
    can_start_hook=NULL;

    expect_fatal=1; fail_queue=1;
    if(!setjmp(fatal_jump)) { start_task(NULL); CHECK(0); }
    CHECK(g_startup_error==4); fail_queue=0;
    /* Fail the last business task, after chassis/hit creation has succeeded. */
    fail_task_name="rc_task";
    if(!setjmp(fatal_jump)) { start_task(NULL); CHECK(0); }
    CHECK(g_startup_error==5 && !xRCRawDataQueue); fail_task_name=NULL;
    CHECK(xChassisCtrlQueue && xHitCmdQueue);
    vQueueDelete(xChassisCtrlQueue); xChassisCtrlQueue=NULL;
    vQueueDelete(xHitCmdQueue); xHitCmdQueue=NULL;
    fail_task=1;
    if(!setjmp(fatal_jump)) { TASK_START(); CHECK(0); }
    CHECK(g_startup_error==3); fail_task=0;
    expect_fatal=0; g_startup_error=0;
    TASK_START(); start_task(NULL);
    CHECK(g_startup_error==0 && task_deletes==1);
    CHECK(observed_start_prio==28 && observed_start_prio>observed_rc_prio &&
          observed_start_prio>observed_chassis_prio && observed_start_prio>observed_hit_prio);
    DVE_DjiMotor_Init(&hcan2);
    for(unsigned id=0x201;id<=0x204;id++) feed(id,1000,100);
    Chassis_MoveCmd_t cmd={.enable=0,.vx=1000}; xQueueOverwrite(xChassisCtrlQueue,&cmd);
    stop_task=1;
    if(!setjmp(task_jump)) chassis_task(NULL);
    stop_task=0;
    for(unsigned i=0;i<8;i++) CHECK(last_data[1][i]==0);
    puts("PASS upward creation errors/queue cleanup, startup priority, board-init order and chassis disable");
}

int main(void) {
    InitializeCriticalSection(&host_irq);
    test_pid_tick(); test_can();
    xHitCmdQueue=xQueueCreate(1,sizeof(Hit_Cmd_t));
    test_remote(); test_position(); test_hit(); test_rc_hit_ack(); test_startup_chassis();
    printf("PASS %u checks; target RTOS scheduling, timing and mechanical tests remain on-board.\n",checks);
    return 0;
}
