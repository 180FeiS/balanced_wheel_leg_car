#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_
#define MaxSize 500
#define Read_MaxSize 10000
#define Nag_End_Page 1
#define Nag_Start_Page 45
#define WHEEL_RADIUS_CM 3.73f
#define Nag_Set_mileage 2.0f
#define Nag_Prev 200
#define Nag_Yaw euler_angle.yaw
#define Nag_Sample_Dt 0.001f
#define Nag_Speed_Source car_speed
#define Nag_Speed_To_Mileage_Scale (2.0f * 3.1415926f * WHEEL_RADIUS_CM / 60.0f)
#define Nag_Speed_Deadband 1.0f
#define Nag_Reissue_Error 0.5f
#define Nag_OdoSlip_Enable 1u
#define Nag_Wheel_Track_Cm 16.0f
#define Nag_OdoSlip_Consistency_Th_Cmps 60.0f
#define Nag_OdoSlip_Instant_Slew_Max_Cmps 120.0f
#define Nag_OdoSlip_Enter_Count 10u
#define Nag_OdoSlip_Exit_Count 25u
#define Nag_OdoSlip_History_Len 15u
#define Nag_OdoSlip_Rollback_Max_Cm 6.0f
#define NAG_USE_FUSION_MILEAGE 1u
#define Nag_Debug_Speed_Bypass_Enable 0u
#define Nag_AdaptiveLookahead_Enable 1u
#define Nag_Lookahead_Base_Points 3u
#define Nag_Lookahead_Speed_Gain 0.010f
#define Nag_Lookahead_Max_Points 24u
#define Nag_Curve_Lookahead_Extra 10u
#define Nag_Curve_Threshold_Straight 8.0f
#define Nag_Curve_Threshold_Sharp 16.0f
#define Nag_Speed_Ratio_Curve 0.9f
#define Nag_Speed_Ratio_Sharp 0.8f
#define Nag_Event_Speed_Ratio 0.35f
#define Nag_EventSpeed_Enable 1u
#define Nag_Spin_Target_Speed_Default 1.0f
#define Nag_Spin_PreDecel_Dist_cm_Default 150.0f
#define Nag_Spin_Rate_Max_Dps_Default 200.0f
extern float nag_spin_target_speed;
extern float nag_spin_pre_decel_dist_cm;
#define Nag_Spin_PreAccel_Dist_cm 0.0f
#define Nag_Spin_Trigger_Window_cm 30.0f
#define Nag_EnterTurn_Target_Speed_Default 220.0f
#define Nag_EnterTurn_PreDecel_Dist_cm_Default 0.0f
extern float nag_enter_turn_target_speed;
extern float nag_enter_turn_pre_decel_dist_cm;
#define Nag_ExitTurn_Recovery_Speed_Default 0.0f
#define Nag_ExitTurn_PreAccel_Dist_cm_Default 0.0f
extern float nag_exit_turn_recovery_speed;
extern float nag_exit_turn_pre_accel_dist_cm;
#define Nag_EnterCones_Target_Speed_Default 1000.0f
#define Nag_EnterCones_PreDecel_Dist_cm_Default 50.0f
extern float nag_enter_cones_target_speed;
extern float nag_enter_cones_pre_decel_dist_cm;
#define Nag_ExitCones_Recovery_Speed 0.0f
#define Nag_ExitCones_PreAccel_Dist_cm 20.0f
#define Nag_EnterBridge_Target_Speed_Default 500.0f
#define Nag_EnterBridge_PreDecel_Dist_cm_Default 0.0f
#define Nag_EnterBridge_Leg_Long 5.5f
#define Nag_ExitBridge_Leg_Long 3.5f
#define Nag_ExitBridge_Recovery_Speed 0.0f
#define BRIDGE_BLOB_LOST_EXIT_MS        250u
#define BRIDGE_ENTER_GRACE_MS           400u
#define BRIDGE_BLOB_NO_FRAME_EXIT_MS    1000u
#define Nag_BridgeBlob_Yaw_K_Pixel      0.22f
#define Nag_BridgeBlob_Yaw_Max_Offset   30.0f
#define Nag_BridgeBlob_Yaw_Deadband     0.1f
extern float nag_enter_bridge_target_speed;
extern float nag_enter_bridge_pre_decel_dist_cm;
#define Nag_Bump_Target_Speed 220.0f
#define Nag_Bump_PreDecel_Dist_cm 0.0f
#define Nag_EnterStair_Target_Speed 300.0f
#define Nag_EnterStair_Leg_Long 5.5f
#define Nag_EnterStair_Yaw_Lookback_cm 20.0f
#define Nag_EnterStair_PreDecel_Dist_cm_Default 0.0f
extern float nag_enter_stair_pre_decel_dist_cm;
#define Nag_HeadingHold_EnterStair_Enable 1u
#define Nag_ExitStair_Leg_Long 3.5f
#define Nag_Stair_Jump_Exit_Count 3u
#define Nag_HeadingHold_ExitStair_Enable 0u
#define Nag_Event_Max 8u
#define Nag_Event_Page 46u
#define Nag_Event_Magic 0x4E414745u
#define Nag_Event_Version 5u
#define Nag_Run_Launch_Speed_Page 47u
#define Nag_Run_Launch_Speed_Magic 0x524C5350u
#define Nag_Run_Launch_Speed_Version 1u
#define Nag_Run_Launch_Params_Version 2u
#define Nag_Run_Launch_Params_Version_V3 3u
#define Nag_Run_Launch_Params_Version_V4 4u
#define Nag_Run_Launch_Params_Version_V5 5u
#define Nag_Run_Launch_Params_Version_V6 6u
#define Nag_Run_Launch_Params_Version_V7 7u
#define Nag_Run_Launch_Params_Version_V8 8u
#define Nag_Run_Launch_Params_Version_V9 9u
#define Nag_Run_Launch_Params_Version_V10 10u
#define Nag_Run_Launch_Param_Count 13u
#define Nag_Run_Launch_Config_Word_Count_V5 11u
#define Nag_Run_Launch_Config_Word_Count_V6 12u
#define Nag_Run_Launch_Config_Word_Count_V7 13u
#define Nag_Run_Launch_Config_Word_Count_V8 15u
#define Nag_Run_Launch_Config_Word_Count_V9 16u
#define Nag_Run_Launch_Config_Word_Count 17u
#define Nag_Launch_Field_Base_Spd 0u
#define Nag_Launch_Field_Spin_Spd 1u
#define Nag_Launch_Field_Spin_Dec 2u
#define Nag_Launch_Field_TurnIn_Spd 3u
#define Nag_Launch_Field_TurnIn_Dec 4u
#define Nag_Launch_Field_TurnOut_Spd 5u
#define Nag_Launch_Field_TurnOut_Acc 6u
#define Nag_Launch_Field_Cone_Spd 7u
#define Nag_Launch_Field_Cone_Dec 8u
#define Nag_Launch_Field_Spin_Rate 9u
#define Nag_Launch_Field_Stair_Dec 10u
#define Nag_Launch_Field_BridgeIn_Spd 11u
#define Nag_Launch_Field_BridgeIn_Dec 12u
float Nag_LaunchParamGet(uint8 field_index);
void Nag_LaunchParamSet(uint8 field_index, float value);
void Nag_LaunchParamAdjust(uint8 field_index, float delta);
void Nag_LaunchParamApplyDefaults(void);
static inline uint8 Nag_LaunchParamIsSpeed(uint8 field_index)
{
    return (uint8)((field_index == Nag_Launch_Field_Base_Spd) ||
                   (field_index == Nag_Launch_Field_Spin_Spd) ||
                   (field_index == Nag_Launch_Field_TurnIn_Spd) ||
                   (field_index == Nag_Launch_Field_TurnOut_Spd) ||
                   (field_index == Nag_Launch_Field_Cone_Spd) ||
                   (field_index == Nag_Launch_Field_BridgeIn_Spd));
}
static inline uint8 Nag_LaunchParamIsSpinRate(uint8 field_index)
{
    return (uint8)(field_index == Nag_Launch_Field_Spin_Rate);
}
static inline float Nag_LaunchParamGetStep(uint8 field_index)
{
    if (Nag_LaunchParamIsSpinRate(field_index))
    {
        return 10.0f;
    }
    if (Nag_LaunchParamIsSpeed(field_index))
    {
        return 100.0f;
    }
    return 10.0f;
}
#define Nag_Spin_Demo_Turns 2.0f
#define Nag_Spin_Demo_Dir 1
#define Nag_Spin_Stop_Speed_Threshold 30.0f
#define Nag_Spin_Stop_Stable_Count 15u
#define Nag_HeadingHold_Reissue_Error 2.0f
#define Nag_HeadingHold_Spin_Enable 0u
#define Nag_HeadingHold_EnterTurn_Enable 0u
#define Nag_HeadingHold_Bump_Enable 0u
typedef enum
{
    NAV_ELEM_NORMAL = 0,
    NAV_ELEM_SPIN = 1,
    NAV_ELEM_TURN_IN = 2,
    NAV_ELEM_TURN_OUT = 3,
    NAV_ELEM_CONE_IN = 4,
    NAV_ELEM_CONE_OUT = 5,
    NAV_ELEM_BRIDGE_IN = 6,
    NAV_ELEM_BRIDGE_OUT = 7,
    NAV_ELEM_BUMP = 8,
    NAV_ELEM_STAIR_IN = 9,
    NAV_ELEM_STAIR_OUT = 10,
    NAV_ELEM_END = 11,
    NAV_ELEM_COUNT = 12,
} Nav_Unified_Element;
#define NAV_ELEM_RECORD_CYCLE_MAX NAV_ELEM_STAIR_OUT
static inline uint8 Nav_UnifiedToInsEvent(uint8 unified)
{
    if (unified <= NAV_ELEM_NORMAL || unified >= NAV_ELEM_END)
    {
        return 0xFFu;
    }
    return (uint8)(unified - 1u);
}
static inline uint8 Nav_CycleUnifiedElement(uint8 current)
{
    current++;
    if (current > NAV_ELEM_RECORD_CYCLE_MAX)
    {
        current = NAV_ELEM_NORMAL;
    }
    return current;
}
static inline uint8 Nav_UnifiedIsMarker(uint8 unified)
{
    return (uint8)((unified == NAV_ELEM_TURN_IN) || (unified == NAV_ELEM_TURN_OUT) ||
                   (unified == NAV_ELEM_CONE_IN) || (unified == NAV_ELEM_CONE_OUT) ||
                   (unified == NAV_ELEM_BRIDGE_IN) || (unified == NAV_ELEM_BRIDGE_OUT));
}
static inline uint8 Nav_UnifiedIsTakeover(uint8 unified)
{
    return (uint8)((unified == NAV_ELEM_SPIN) || (unified == NAV_ELEM_STAIR_IN));
}
static inline uint8 Nav_UnifiedIsPassThrough(uint8 unified)
{
    return (uint8)((unified == NAV_ELEM_NORMAL) || (unified == NAV_ELEM_BUMP));
}
static inline const char *Nav_GetUnifiedElementName(uint8 unified)
{
    switch (unified)
    {
        case NAV_ELEM_NORMAL: return "Normal";
        case NAV_ELEM_SPIN: return "Spin";
        case NAV_ELEM_TURN_IN: return "TurnIn";
        case NAV_ELEM_TURN_OUT: return "TurnOut";
        case NAV_ELEM_CONE_IN: return "ConeIn";
        case NAV_ELEM_CONE_OUT: return "ConeOut";
        case NAV_ELEM_BRIDGE_IN: return "BridgeIn";
        case NAV_ELEM_BRIDGE_OUT: return "BridgeOut";
        case NAV_ELEM_BUMP: return "Bump";
        case NAV_ELEM_STAIR_IN: return "StairIn";
        case NAV_ELEM_STAIR_OUT: return "StairOut";
        case NAV_ELEM_END: return "End";
        default: return "Unknown";
    }
}
typedef enum
{
       NAG_EVENT_TYPE_SPIN = 0,
       NAG_EVENT_TYPE_ENTER_TURNAROUND = 1,
       NAG_EVENT_TYPE_EXIT_TURNAROUND = 2,
       NAG_EVENT_TYPE_ENTER_CONES = 3,
       NAG_EVENT_TYPE_EXIT_CONES = 4,
       NAG_EVENT_TYPE_ENTER_SINGLE_BRIDGE = 5,
       NAG_EVENT_TYPE_EXIT_SINGLE_BRIDGE = 6,
       NAG_EVENT_TYPE_BUMP = 7,
       NAG_EVENT_TYPE_ENTER_STAIR = 8,
       NAG_EVENT_TYPE_EXIT_STAIR = 9,
       NAG_EVENT_TYPE_COUNT = 10,
} Nag_Event_Type;
static inline const char *Nag_GetEventTypeName(uint8 event_type)
{
    switch (event_type)
    {
        case NAG_EVENT_TYPE_SPIN: return "Spin";
        case NAG_EVENT_TYPE_ENTER_TURNAROUND: return "TurnIn";
        case NAG_EVENT_TYPE_EXIT_TURNAROUND: return "TurnOut";
        case NAG_EVENT_TYPE_ENTER_CONES: return "ConeIn";
        case NAG_EVENT_TYPE_EXIT_CONES: return "ConeOut";
        case NAG_EVENT_TYPE_ENTER_SINGLE_BRIDGE: return "BridgeIn";
        case NAG_EVENT_TYPE_EXIT_SINGLE_BRIDGE: return "BridgeOut";
        case NAG_EVENT_TYPE_BUMP: return "Bump";
        case NAG_EVENT_TYPE_ENTER_STAIR: return "EnterStair";
        case NAG_EVENT_TYPE_EXIT_STAIR: return "ExitStair";
        default: return "Unknown";
    }
}
typedef enum
{
       NAG_EVENT_STATE_IDLE = 0,
       NAG_EVENT_STATE_ENTERED = 1,
       NAG_EVENT_STATE_RUNNING = 2,
       NAG_EVENT_STATE_DONE = 3,
       NAG_EVENT_STATE_ABORT = 4,
} Nag_Event_State;
typedef struct
{
       uint16 enter_index;
       uint16 exit_index;
       uint8 type;
       uint8 valid;
} NagEvent;
typedef struct{
       float Final_Out;
       float Mileage_All;
       float Mileage_Step;
       float Mileage_Debug_Total;
       float Speed_Forward;
       float Curve_Strength;
       float Target_Speed;
       float Angle_Run;
       float Requested_Target_Yaw;
       bool Nag_Stop_f;
       uint8 Target_Request_Valid;
       uint8 Flash_read_f;
       uint16 size;
       uint16 Run_index;
       uint16 Save_count;
       uint16 Save_index;
       uint16 Prospect_index;
       uint16 Active_Event_Enter;
       uint16 Active_Event_Exit;
       uint8 Save_state;
       uint8 End_f;
       uint8 Flash_page_index;
       uint8 Flash_Save_Page_Index;
       uint8 Nag_SystemRun_Index;
       uint8 Event_Active;
       uint8 Event_Count;
       uint8 Event_Record_Pending;
       uint8 Event_Record_Type;
       uint8 Event_Active_Index;
       uint8 Event_State;
       uint8 Event_Start_Latched;
       uint8 Event_Done_Latched;
       uint8 Event_Active_Type;
       uint16 Event_Start_RunIndex;
       uint16 Event_Trigger_RunIndex;
       uint8 Event_Triggered_In_Window;
       uint8 Event_Consumed[Nag_Event_Max];
       float Spin_Saved_SetSpeed;
       uint16 Spin_Stop_Stable_Count;
       uint8 Spin_Task_Started;
       uint16 Spin_Resume_RunIndex;
       uint8 Spin_Speed_Latched;
       float Stair_Saved_Leg_Long;
       float Stair_Saved_SetSpeed;
       float Stair_Lookback_Yaw;
       uint8 Stair_Jump_Completed_Count;
       uint8 Stair_Chain_To_Exit;
       uint16 Stair_Paired_Enter_Index;
       float Bridge_Saved_Leg_Long;
       uint8 Bridge_Saved_RollBalance;
       uint8 Bridge_Zone_Active;
       uint8 Bridge_Detect_Arm;
       uint8 Bridge_Heading_Lock;
       uint8 Bridge_Expected;
       uint8 Bridge_Exit_Beep_Done;
       float Bridge_Locked_Yaw;
       uint16 Bridge_Exit_Run_Index;
       uint8 HeadingHold_Enable;
       uint8 HeadingHold_Request_Armed;
       uint8 HeadingHold_Target_Latched;
       uint8 HeadingHold_Event_Allowed;
       float HeadingHold_Target_Yaw;
       float Odo_Wheel_Left_Cmps;
       float Odo_Wheel_Right_Cmps;
       float Odo_Gyro_Z_Dps;
       float Odo_Vc_From_L_Cmps;
       float Odo_Vc_From_R_Cmps;
       float Odo_Corrected_Speed_Cmps;
       float Odo_Raw_Step_Cm;
       float Odo_Protected_Step_Cm;
       float Odo_Last_Trust_Speed_Cmps;
       float Odo_Rollback_Pending_Cm;
       float Odo_Rollback_Applied_Cm;
       float Odo_History_Raw[Nag_OdoSlip_History_Len];
       float Odo_History_Prot[Nag_OdoSlip_History_Len];
       uint8 Odo_Slip_State;
       uint8 Odo_Slip_Enter_Count;
       uint8 Odo_Slip_Exit_Count;
       uint8 Odo_History_Idx;
       int Prev_mile[Nag_Prev];
}Nag;
typedef enum
{
       NAG_ODO_SLIP_NORMAL = 0u,
       NAG_ODO_SLIP_LEFT = 1u,
       NAG_ODO_SLIP_RIGHT = 2u,
       NAG_ODO_SLIP_BOTH = 3u,
} Nag_OdoSlip_State;
extern Nag N;
extern int32 Nav_read[Read_MaxSize];
extern NagEvent Nag_Event_Table[Nag_Event_Max];
extern uint8 Nag_Vofa_Group;
#define NAG_VOFA_GROUP_COUNT (4u)
void Nag_OdoSlip_ResetState(void);
void Nag_OdoSlip_ApplyPendingRollback(void);
typedef enum {
    NAV_HEADING_MODE_INS = 0u,
    NAV_HEADING_MODE_GPS = 1u,
} NavHeadingMode;
extern uint8 nav_heading_mode;
void Nag_Run();
uint8 Nag_BridgeDetectShouldArm(void);
void Nag_BridgeDetectUpdate(void);
void Nag_BridgeTimeoutTick1ms(void);
void Run_Nag_GPS();
void NagFlashRead();
void Run_Nag_Save();
void Nag_Read();
void Init_Nag();
void Nag_Begin_Record(void);
void Nag_Begin_Replay(void);
void Nag_CompleteReplayAfterOrigin(void);
void Nag_Request_Stop_Record(void);
void Nag_Request_Event_Mark(void);
void Nag_Cycle_Record_Event_Type(void);
void Nag_Notify_Event_Done(void);
void Nag_NotifyStairJumpDone(void);
void Nag_Element_Abort(void);
float Nag_GetDebugReadYaw(void);
float Nag_GetControlSpeedTarget(void);
bool Nav_GetEventSpeedProfileConfig(uint8 event_type,
                                    float *target_speed,
                                    float *pre_decel_dist_cm,
                                    float *pre_accel_dist_cm);
float GPS_ApplyEventSpeedAdjustments(float nav_speed);
void Nag_EventPrepareEnter(uint8 event_type);
void Nag_EventForceReset(void);
uint16 Nag_GetDebugProspectIndex(void);
bool Nag_HeadingHold_ShouldRequest(void);
float Nag_HeadingHold_GetTargetYaw(void);
bool Nag_Element_Start(uint8 event_type);
void Nag_Element_Run(uint8 event_type);
bool Nag_Element_IsDone(uint8 event_type);
void Nag_Element_Stop(uint8 event_type);
bool Nag_Hook_EnterTurn_Start(void);
void Nag_Hook_EnterTurn_Run(void);
bool Nag_Hook_EnterTurn_IsDone(void);
void Nag_Hook_EnterTurn_Stop(void);
bool Nag_Hook_ExitTurn_Start(void);
void Nag_Hook_ExitTurn_Run(void);
bool Nag_Hook_ExitTurn_IsDone(void);
void Nag_Hook_ExitTurn_Stop(void);
bool Nag_Hook_Spin_Start(void);
void Nag_Hook_Spin_Run(void);
bool Nag_Hook_Spin_IsDone(void);
void Nag_Hook_Spin_Stop(void);
bool Nag_Hook_EnterBridge_Start(void);
void Nag_Hook_EnterBridge_Run(void);
bool Nag_Hook_EnterBridge_IsDone(void);
void Nag_Hook_EnterBridge_Stop(void);
bool Nag_Hook_ExitBridge_Start(void);
void Nag_Hook_ExitBridge_Run(void);
bool Nag_Hook_ExitBridge_IsDone(void);
void Nag_Hook_ExitBridge_Stop(void);
bool Nag_Hook_Bump_Start(void);
void Nag_Hook_Bump_Run(void);
bool Nag_Hook_Bump_IsDone(void);
void Nag_Hook_Bump_Stop(void);
float Nag_ComputeYawAverageLookback(uint16 anchor_index, float lookback_cm);
bool Nag_Hook_EnterStair_Start(void);
void Nag_Hook_EnterStair_Run(void);
bool Nag_Hook_EnterStair_IsDone(void);
void Nag_Hook_EnterStair_Stop(void);
bool Nag_Hook_ExitStair_Start(void);
void Nag_Hook_ExitStair_Run(void);
bool Nag_Hook_ExitStair_IsDone(void);
void Nag_Hook_ExitStair_Stop(void);
bool Nag_Hook_EnterCones_Start(void);
void Nag_Hook_EnterCones_Run(void);
bool Nag_Hook_EnterCones_IsDone(void);
void Nag_Hook_EnterCones_Stop(void);
bool Nag_Hook_ExitCones_Start(void);
void Nag_Hook_ExitCones_Run(void);
bool Nag_Hook_ExitCones_IsDone(void);
void Nag_Hook_ExitCones_Stop(void);
void Nag_System();
#endif
