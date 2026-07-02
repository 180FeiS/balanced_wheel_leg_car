#ifndef CODE_NAV_FUSION_H_
#define CODE_NAV_FUSION_H_

#include "zf_common_typedef.h"

/*
 * GPS + 惯导/编码器 二维松耦合融合导航
 *
 * 使用方式（与原有 GPS / 惯导 并存，宏开关可一键回退）：
 * 1) GPS 点导航：my_gps.h 中 GPS_NAV_USE_FUSION_POSITION=1，录点/发车流程不变，追点用融合位置。
 * 2) 惯导回放：navigation.h 中 NAG_USE_FUSION_MILEAGE=1，录制/回放 KEY 流程不变，里程用融合位移。
 * 3) 发车前：KEY3 后自动采集 NAV_FUSION_ORIGIN_SAMPLE_COUNT 个有效 GPS 点取平均建原点。
 *
 * 实车验证顺序见 nav_fusion.c 文件头注释。
 */

/* 1=编译并链接融合模块；0=整模块编译为 stub。运行时开关见 g_menu_nav_fusion_enable（Run→Config / Flash V10）。 */
#define NAV_FUSION_ENABLE 1u

extern uint8 g_menu_nav_fusion_enable;

static inline uint8 NavFusion_IsRuntimeEnabled(void)
{
#if NAV_FUSION_ENABLE
    return (uint8)(g_menu_nav_fusion_enable != 0u);
#else
    return 0u;
#endif
}

/*
 * 1=发车时用多点 GPS 平均建原点；0=退回单帧 NavFusion_InitFromGps（旧行为）。
 */
#define NAV_FUSION_ORIGIN_ENABLE 1u

/*
 * 发车原点采集：目标有效样本数（与 my_gps.h 路点上限 GPS_POINT_MAX 无关）。
 * 调大：原点更稳、等待更久；调小：更快发车但抗跳点弱。建议配合 GNSS_UPDATE_RATE_HZ=10。
 */
#define NAV_FUSION_ORIGIN_SAMPLE_COUNT 50u

/*
 * 相对当前运行均值的跳点门限（m）；超过则本帧不入平均库。
 * 调小：更拒野点；调大：更易凑满 SAMPLE_COUNT 但可能被野点污染。
 */
#define NAV_FUSION_ORIGIN_OUTLIER_M 2.0f

/*
 * 参与平均的最低卫星数；低于此值的帧直接丢弃。
 * 调大：更严更稳；调小：弱信号环境更易凑点。
 */
#define NAV_FUSION_ORIGIN_MIN_SATELLITES 10u

/*
 * 原点采集总超时（ms），在 1ms Predict 中累加。
 * 超时后若 accepted >= MIN_SAMPLES 则用部分样本完成，否则采集失败。
 */
#define NAV_FUSION_ORIGIN_TIMEOUT_MS 10000u

/*
 * 超时兜底最少样本数；不足则返回 FAILED，需重新 KEY3 发车。
 * 设为与 SAMPLE_COUNT 相同则必须凑满才完成。
 */
#define NAV_FUSION_ORIGIN_MIN_SAMPLES 25u

/* NavFusion_FeedOriginSample 返回值 */
#define NAV_FUSION_ORIGIN_FEED_COLLECTING 0u
#define NAV_FUSION_ORIGIN_FEED_DONE 1u
#define NAV_FUSION_ORIGIN_FEED_FAILED 2u

/* -------------------------------------------------------------------------- */
/* 调参宏：注释中说明“调大/调小”效果，便于 VOFA 实车对比                        */
/* -------------------------------------------------------------------------- */

/* 预测周期，须与 pit0_ch0_isr 中 NavFusion_Predict1ms 调用周期一致（当前 1ms） */
#define NAV_FUSION_DT_S 0.001f

/*
 * car_speed -> m/s 换算系数；可在编译前 -D 覆盖，或在 nav_fusion.c 中按 Nag_Speed_To_Mileage_Scale 默认推导。
 * 调大：融合认为走得更快，长距离会超前；调小：会落后。
 */
#ifndef NAV_FUSION_SPEED_SCALE
#define NAV_FUSION_SPEED_SCALE 0.0037f
#endif

/*
 * GPS 残差门限（m）：|gps - fusion| 超过此值认为跳点，本帧不修正。
 * 调小：更拒 GPS 跳点，但正常大偏差也修得慢；调大：更易跟 GPS，抖动时可能急转。
 */
#define NAV_FUSION_GPS_GATE_M 3.0f

/*
 * GPS 修正增益（0~1）：每帧向 GPS 靠近 residual 的比例。
 * 调大：更跟 GPS，到点快但易抖；调小：更平滑，长距离漂移拉回慢。建议 0.05~0.15。
 */
#define NAV_FUSION_GPS_GAIN 0.10f

/*
 * 零速阈值：|car_speed| 低于此值视为静止，不再积分位移（与 Nag_Speed_Deadband 同量级）。
 * 调大：停车/元素期更不易漂，但极低速可能被当静止；调小：低速仍积分，停车易漂。
 */
#define NAV_FUSION_ZERO_SPEED_THRESHOLD 10.0f

/* 静止判定连续帧数（1ms/帧），避免单帧噪声误判 */
#define NAV_FUSION_ZERO_SPEED_STABLE_COUNT 10u

/*
 * GPS 超时（ms）：超过此时间无新帧则 gps_used=0，仅惯导预测继续。
 * 调大：GPS 丢失后仍认为“可用”更久；调小：更快放弃 GPS 修正。
 */
#define NAV_FUSION_GPS_TIMEOUT_MS 3000u

/* 连续有效 GPS 帧数达到此值才允许修正，抑制首帧跳点 */
#define NAV_FUSION_GPS_GOOD_STREAK 2u

#define NAV_FUSION_EARTH_RADIUS_M 6371000.0
#define NAV_FUSION_PI 3.14159265358979323846f
#define NAV_FUSION_DEG_TO_RAD(x) ((x) * (NAV_FUSION_PI / 180.0f))
#define NAV_FUSION_RAD_TO_DEG(x) ((x) * (180.0f / NAV_FUSION_PI))

typedef struct
{
    float x_m;            /* 局部东向位移，m，相对 origin */
    float y_m;            /* 局部北向位移，m */
    float yaw_deg;        /* 用于推算的航向，来自 euler_angle.yaw */
    float v_mps;          /* 标定后前向速度，m/s */
    float gps_residual_m; /* 最近 GPS 与融合位置距离，m，VOFA 看跳点 */
    float gps_weight;     /* 最近一帧实际使用的修正增益 */
    uint8 valid;          /* 1：原点已建立，融合状态可用 */
    uint8 gps_used;       /* 1：最近一帧 GPS 参与了修正 */
} NavFusionState;

void NavFusion_Reset(void);

/*
 * 以当前 GPS 为局部坐标原点建立融合状态；发车/回放前调用。
 * lat/lon 须有效非零；yaw_deg 一般为 euler_angle.yaw。
 */
uint8 NavFusion_InitFromGps(double lat, double lon, float yaw_deg);

/*
 * 1ms ISR 内调用：yaw + 速度预测位置。
 * speed_src 一般为 car_speed（与 NAV_FUSION_SPEED_SCALE 配套）。
 */
void NavFusion_Predict1ms(float yaw_deg, float speed_src);

/*
 * 1ms ISR 内调用：静止零速约束，抑制元素停车期间积分漂移。
 */
void NavFusion_ZeroVelocityUpdate(float speed_src);

/*
 * 主循环 gnss_data_parse() 之后调用；GPS 新帧低频修正 x/y。
 * gps_state==0 或坐标无效时不修正。
 */
void NavFusion_UpdateGps(double lat, double lon, uint8 gps_state, uint8 satellite_used);

const NavFusionState *NavFusion_GetState(void);
uint8 NavFusion_IsValid(void);

/* 融合位置反算经纬度，供 GPS_PointNav_Run 复用现有距离/方位函数 */
void NavFusion_GetPositionLatLon(double *lat_out, double *lon_out);

/* 经纬度 -> 相对 origin 的局部坐标（m） */
void NavFusion_LatLonToLocal(double lat, double lon, float *x_m, float *y_m);

/*
 * 惯导里程辅助：返回本周期融合位移（cm）；<0 表示无效，应退回 car_speed 积分。
 * 在 Nag_GetMileageStep() 内调用。
 */
float NavFusion_GetMileageStepCm(void);

/*
 * 刷新融合里程 prev 快照（g_prev_x/y <- 当前 g_fusion.x/y），不重置融合位置。
 * Spin / ENTER_STAIR 等元素冻结 Run_index 期间 Predict 仍在跑；进入/退出元素时调用，
 * 避免恢复后 NavFusion_GetMileageStepCm() 一次性把元素期位移灌入 Run_index。
 */
void NavFusion_SyncMileageSnapshot(void);

#if NAV_FUSION_ENABLE && NAV_FUSION_ORIGIN_ENABLE

/*
 * KEY3 发车后调用：Reset 后进入原点采集态，尚未建立 valid 融合坐标系。
 * yaw_deg 为发车瞬间 IMU 偏航，供完成后 InitFromGps 使用。
 */
void NavFusion_BeginOriginAverage(float yaw_deg);

/*
 * 主循环 gnss_data_parse 后调用（采集态下替代 UpdateGps）。
 * 返回 NAV_FUSION_ORIGIN_FEED_*：DONE 时内部已 InitFromGps；FAILED 时样本不足。
 */
uint8 NavFusion_FeedOriginSample(double lat, double lon, uint8 gps_state, uint8 satellite_used);

uint8 NavFusion_IsOriginCalibrating(void);
uint16 NavFusion_GetOriginAcceptedCount(void);
uint16 NavFusion_GetOriginRejectedCount(void);

/* 采集成功后读取平均原点经纬度；失败或未结束时返回 0 */
uint8 NavFusion_GetOriginAvgResult(double *lat_out, double *lon_out);

/* 超时失败等：消费一次失败脉冲，返回 1 表示刚失败（主循环无需等 GNSS 帧） */
uint8 NavFusion_ConsumeOriginFailure(void);

#else

void NavFusion_BeginOriginAverage(float yaw_deg);
uint8 NavFusion_FeedOriginSample(double lat, double lon, uint8 gps_state, uint8 satellite_used);
uint8 NavFusion_IsOriginCalibrating(void);
uint16 NavFusion_GetOriginAcceptedCount(void);
uint16 NavFusion_GetOriginRejectedCount(void);
uint8 NavFusion_GetOriginAvgResult(double *lat_out, double *lon_out);
uint8 NavFusion_ConsumeOriginFailure(void);

#endif

#endif /* CODE_NAV_FUSION_H_ */
