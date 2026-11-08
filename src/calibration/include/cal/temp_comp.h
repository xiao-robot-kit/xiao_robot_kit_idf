/* SPDX-License-Identifier: MIT */
#ifndef CAL_TEMP_COMP_H
#define CAL_TEMP_COMP_H

#include <stdint.h>
#include "imu/imu_types.h"

/**
 * @file    temp_comp.h
 * @brief   IMU 温漂两段：系数标定（最小二乘拟合）+ 温度补偿应用（线性模型）
 *
 * @note    合并依据（旧式实现 temp_calib 与 temp_comp 强耦合，故移植合为一文件）：
 *          拟合结果直喂 `temp_comp_set_drift`；且斜率幅值上限在两处重复定义
 *          （旧式实现 temp_calib.h:53-57 的 TEMP_CALIB_MAX_* 与
 *          temp_comp.c:61-62 的钳制值同值不同名）——合并后单一 cfg 单一上限。
 *          接口仍按「温标（temp_calib_*）/ 应用（temp_comp_*）」两段清晰分离。
 *
 * @note    模型：bias(T) = bias_ref + k × (T − T_ref)。
 *          单位（本组件零换算，温度进来时已是 °C——换算在驱动边界）：
 *          - 温度 T / 参考温度 T_ref：°C
 *          - 陀螺温漂系数 k：dps/°C（零偏 dps 随温度的变化率）
 *          - 加速度温漂系数 k：g/°C（零偏 g 随温度的变化率；
 *            0.5 mg/°C = 0.0005 g/°C，1000 mg = 1 g）
 * @note    T_ref 必须取「零偏标定那一刻」的温度（锚点），否则锚点错位
 *          引入固定偏差（沿旧式实现结论，原 temp_comp.h:16-17）。
 */

/**
 * @brief 最小二乘分母下限（数值护栏，非可调参数）
 *
 * 分母 denom = N·ΣT² − (ΣT)²。|denom| 小于此值说明温度几乎不变，
 * 斜率无意义（沿旧式实现 1e-9 选型，原 temp_calib.c:93）。
 */
#define TEMP_COMP_DENOM_EPS 1e-9

/**
 * @brief   温漂组件配置（标定与应用共用；参数零写死：可调项全部经此注册）
 */
typedef struct {
    float   step_c;                 /**< 补偿阶梯阈值，°C：相对上次应用点变化 ≥ 此值才重算（0 = 每次重算） */
    float   calib_min_span_c;       /**< 温标有效所需最小温度跨度，°C（跨度不足判拟合无效） */
    float   gyro_k_max;             /**< 陀螺温漂系数幅值上限，dps/°C（超限判无效并钳零） */
    float   accel_k_max;            /**< 加速度温漂系数幅值上限，g/°C（超限判无效并钳零） */
    int32_t calib_target_n;         /**< 温标目标样本数（时长 = calib_target_n / 采样率，秒） */
    float   default_gyro_drift[3];  /**< 未标定时的默认陀螺系数，dps/°C（量级参考，标定后覆盖） */
    float   default_accel_drift[3]; /**< 未标定时的默认加速度系数，g/°C（量级参考，标定后覆盖） */
} temp_comp_cfg_t;

/** @brief 命名预设（注册表项；表定义在 src/temp_comp.c，追加预设只改那一个文件） */
typedef struct {
    const char       *name;   /**< 预设名（协议可寻址，点分命名空间如 "cal.exp2.drift"） */
    temp_comp_cfg_t  cfg;     /**< 参数 */
} temp_comp_cfg_preset_t;

/** @brief 预设表与项数（定义在 src/temp_comp.c） */
extern const temp_comp_cfg_preset_t temp_comp_cfg_presets[];
extern const int                    temp_comp_cfg_preset_count;

/**
 * @brief   按名查预设
 *
 * @param[in] name  预设名
 *
 * @return  匹配项的 cfg 指针（静态存储，调用方不释放）；NULL = 无此预设
 */
const temp_comp_cfg_t *temp_comp_cfg_find(const char *name);

/* ================================================================ 温标段 */

/** @brief 温标采集状态 */
typedef enum {
    TEMP_CALIB_IDLE = 0,       /**< 空闲（未开始，或本轮拟合已完成） */
    TEMP_CALIB_COLLECTING = 1, /**< 采集中 */
} temp_calib_state_t;

/**
 * @brief   温漂系数标定实例（状态全部实例内，无全局可变态）
 *
 * @note    累加用 double：温度×零偏的乘积项用 float 累积会明显丢精度
 *            （沿旧式实现选型，原 temp_calib.h:27-28）。
 */
typedef struct {
    temp_comp_cfg_t   cfg;                     /**< init 时拷贝的配置 */
    temp_calib_state_t state;                  /**< 采集状态 */
    uint32_t          n;                       /**< 已采集样本数 */
    double            sum_t, sum_t2;           /**< ΣT、ΣT²（°C、°C²） */
    double            sum_g[3], sum_tg[3];     /**< 陀螺：Σb（dps）、Σ(T·b)（°C·dps） */
    double            sum_a[3], sum_ta[3];     /**< 加速度：Σb（g）、Σ(T·b)（°C·g） */
    float             t_min, t_max;            /**< 采集期温度极值，°C（判跨度） */
    float             gyro_drift[3];           /**< 拟合结果，dps/°C（无效时全 0） */
    float             accel_drift[3];          /**< 拟合结果，g/°C（无效时全 0） */
    int               fitted;                  /**< 1 = 已完成一次拟合（start 清零） */
    int               valid;                   /**< 1 = 拟合有效（跨度足且斜率在物理范围内） */
} temp_calib_t;

/**
 * @brief   初始化温标实例（绑定配置并复位全部状态）
 *
 * @param[out] tc   实例
 * @param[in]  cfg  配置（拷贝入实例）
 *
 * @retval  0  成功
 * @retval -1  tc/cfg 为 NULL；或 calib_target_n < 2（两点回归至少 2 样本）；
 *             或 calib_min_span_c < 0 / gyro_k_max <= 0 / accel_k_max <= 0
 */
int temp_calib_init(temp_calib_t *tc, const temp_comp_cfg_t *cfg);

/**
 * @brief   开始温标采集
 *
 * @param[in,out] tc  实例
 *
 * @retval  0  成功（含"已在采集中"的幂等路径）
 * @retval -1  tc 为 NULL
 *
 * @note    IDLE → COLLECTING：清全部累加、n、温度极值、拟合完成/有效标志。
 *          （旧式实现同类雷预防：start 必须清全部状态，防上一轮拟合结果
 *          在新一轮中途被误读——与 imu_calib_start 的 raw 累加雷同型。）
 * @note    已在 COLLECTING：幂等返回 0 且不清进度（重发命令不重启采集）。
 */
int temp_calib_start(temp_calib_t *tc);

/**
 * @brief   喂入一对（温度, 静止样本），凑满时完成拟合
 *
 * @param[in,out] tc    实例
 * @param[in]     temp  当前温度，°C（须为有限值）
 * @param[in]     s     静止样本（ax/ay/az = g，gx/gy/gz = dps）。
 *                      建议喂**未去偏的原始读数**：线性拟合对常值偏移不变
 *                      （重力分量、既有零偏进截距，斜率 k 不受影响；旧式实现
 *                      即喂原始值，app_stream.c:190-196）
 *
 * @retval  0  样本已接受（凑满与否都算成功；完成与否查 temp_calib_ready）
 * @retval -1  tc/s 为 NULL、temp 非有限，或未处于采集状态（调用序错误）
 *
 * @note    凑满 calib_target_n 时按最小二乘拟合各轴 k：
 *          k = (N·Σ(T·b) − ΣT·Σb) / (N·ΣT² − (ΣT)²)
 *          无效判定（两者任命中 → 六轴系数全 0、valid=0）：
 *          - 温度跨度 < calib_min_span_c 或 |denom| < TEMP_COMP_DENOM_EPS
 *            （分母极小会拟合出荒谬斜率，存入后毁温补——旧式实现实机教训）
 *          - 任一轴 |k| 超出 k_max（整组判废，不混用半好系数）
 */
int temp_calib_feed(temp_calib_t *tc, float temp, const imu_sample_t *s);

/**
 * @brief   查询本轮温标是否完成（含"无效拟合"的完成）
 *
 * @retval  1  已完成拟合（有效性另查 temp_calib_valid）
 * @retval  0  未完成 / 未开始
 * @retval -1  tc 为 NULL
 */
int temp_calib_ready(const temp_calib_t *tc);

/**
 * @brief   查询上次拟合是否有效
 *
 * @retval  1  有效（跨度足且斜率在物理范围内）
 * @retval  0  无效或尚未拟合
 * @retval -1  tc 为 NULL
 *
 * @note    旧式实现遗留 TODO 已修：原 temp_calib 无成功标志，
 *          "拟合失败返回全 0"与"成功但系数为 0"无法区分（原 calibration
 *          README.md:66），本版以显式查询接口区分。
 */
int temp_calib_valid(const temp_calib_t *tc);

/**
 * @brief   读取拟合结果
 *
 * @param[in]  tc           实例
 * @param[out] gyro_drift   陀螺三轴系数，dps/°C
 * @param[out] accel_drift  加速度三轴系数，g/°C
 *
 * @retval  0  已拷出结果
 * @retval -1  tc/gyro_drift/accel_drift 为 NULL，或尚未完成拟合
 *
 * @note    旧式实现遗留 TODO 已修：原 temp_calib_result 参数非 const 且
 *          未完成时读到旧值（原 calibration README.md:67），本版 const +
 *          未完成拒绝。
 */
int temp_calib_result(const temp_calib_t *tc, float gyro_drift[3], float accel_drift[3]);

/* ================================================================ 应用段 */

/**
 * @brief   温度补偿实例（状态全部实例内，无全局可变态）
 *
 * @note    bias_last 缓存在实例内：apply 同档保持时也能输出**当前生效**
 *          的补偿零偏（输出契约恒定，不依赖调用方保存旧值）。
 */
typedef struct {
    temp_comp_cfg_t cfg;                 /**< init 时拷贝的配置 */
    float           t_ref;               /**< 参考温度，°C（= 零偏标定那一刻的温度） */
    imu_bias_t      bias_ref;            /**< 参考温度下的零偏（dps / g） */
    float           gyro_drift[3];       /**< 陀螺温漂系数，dps/°C */
    float           accel_drift[3];      /**< 加速度温漂系数，g/°C */
    int             applied;             /**< 1 = init/set_drift/disable 后已应用过至少一次 */
    float           last_applied_temp;   /**< 上次实际重算时的温度，°C */
    imu_bias_t      bias_last;           /**< 当前生效的补偿零偏（dps / g） */
    uint16_t        apply_count;         /**< 累计实际重算次数（诊断用；饱和不回绕） */
} temp_comp_t;

/**
 * @brief   初始化温度补偿（装入 cfg 的默认系数）
 *
 * @param[out] tc        实例
 * @param[in]  cfg       配置（拷贝入实例）
 * @param[in]  t_ref     参考温度，°C（锚点；须取零偏标定那一刻的温度）
 * @param[in]  bias_ref  参考温度下的零偏（dps / g）
 *
 * @retval  0  成功
 * @retval -1  任一指针为 NULL；t_ref 非有限；或 cfg 应用段参数非法
 *             （step_c < 0 或为 NaN、gyro_k_max <= 0、accel_k_max <= 0）
 *
 * @note    系数取 cfg 的 default_*（量级参考）；精确值来自温标拟合，
 *          经 temp_comp_set_drift 覆盖。
 * @note    applied 置 0：首次 apply 无条件重算一次（初始化生效零偏）——
 *          否则首拍温度恰等于 t_ref 时温差为 0，被阶梯挡住永不初始化
 *          （旧式实现哨兵值 TEMP_COMP_NOT_APPLIED 的等价实现，改为显式标志）。
 */
int temp_comp_init(temp_comp_t *tc, const temp_comp_cfg_t *cfg,
                   float t_ref, const imu_bias_t *bias_ref);

/**
 * @brief   覆盖温漂系数（温标拟合值；超幅值上限的轴置 0）
 *
 * @param[in,out] tc           实例
 * @param[in]     gyro_drift   陀螺三轴系数，dps/°C
 * @param[in]     accel_drift  加速度三轴系数，g/°C
 *
 * @retval  0  成功
 * @retval -1  任一指针为 NULL
 *
 * @note    钳制（防任何来源的坏值毁温补：旧持久化数据 / 异常拟合）：
 *          轴系数 |k| 超出 cfg 的 k_max（含 NaN）→ 该轴置 0，其余轴照常。
 * @note    只改系数，不动 t_ref / bias_ref。
 * @note    语义是「换系数」而非幂等每拍调用：置 applied=0，
 *          新系数在下次 apply 立即生效（不被阶梯挡住）。
 */
int temp_comp_set_drift(temp_comp_t *tc,
                        const float gyro_drift[3], const float accel_drift[3]);

/**
 * @brief   按当前温度更新补偿后的零偏（阶梯式）
 *
 * @param[in,out] tc    实例
 * @param[in]     temp  当前温度，°C（须为有限值）
 * @param[out]    bias  输出**当前生效**的补偿零偏（dps / g；恒有效——
 *                      跨档时为新算值，同档时为上次算值）
 *
 * @retval  0  成功（bias 已写入）
 * @retval -1  tc/bias 为 NULL，或 temp 非有限
 *
 * @note    计算式：bias = bias_ref + k × (T − t_ref)（六轴同形）。
 *          但**只在温度相对上次应用点变化 ≥ step_c（或首次）时重算**：
 *          温度传感器的 0.0X °C 抖动是噪声，每拍重算会被 k 放大成
 *          可观测的零偏抖动；阶梯把档内噪声完全吸收（沿旧式实现实测教训，
 *          原 temp_comp.h:29-39）。step_c = 0 退化为每拍重算。
 * @note    重算按**相对 t_ref 的绝对温差**（不逐档累加，防漂移积累）。
 * @note    旧式实现雷已修【medium】：旧 temp_comp.c:68-77 把本函数的 doc
 *          错位悬挂在 temp_comp_disable 之前（导致 disable 顶了两份 doc、
 *          apply 定义处无归属 doc，旧式实现）。本版每份 doc
 *          紧邻其所属函数定义。
 */
int temp_comp_apply(temp_comp_t *tc, float temp, imu_bias_t *bias);

/**
 * @brief   关闭温补：把零偏恢复为参考值
 *
 * @param[in,out] tc    实例
 * @param[out]    bias  恢复后的零偏 = bias_ref（dps / g）
 *
 * @retval  0  成功（bias 已写入）
 * @retval -1  tc/bias 为 NULL
 *
 * @note    为什么需要：apply 会改写生效零偏；关掉后若不恢复，
 *          零偏停在最后一次补偿值 → 用户以为开关不生效（旧式实现实测
 *          确认的真 bug，原 temp_comp.c:84-86）。
 * @note    同时置 applied=0、apply_count=0：再次启用时立即重算一次。
 */
int temp_comp_disable(temp_comp_t *tc, imu_bias_t *bias);

/**
 * @brief   温补是否生效（开关位 && 已标定，双判单源）
 *
 * @param[in] tempcomp_en  功能开关位（`SYSF_TEMPCOMP_EN` 是否置位，非 0 = 开）
 * @param[in] tc_valid     温漂是否**已标定**（双判的第二项，与开关分离）
 *
 * @return  1 = 应调 `temp_comp_apply`；0 = 不补偿（并应在必要时调 `temp_comp_disable`）
 *
 * @note    定义式 `tempcomp_en && tc_valid`（原型 `app_stream.c:329-330`
 *          `poll_temp_tick` 的双判）。两项缺一都不补偿：
 *          - 开关开但未标定 → 不生效（标定=测量，开关=应用，META-008 分离）
 *          - 已标定但开关关 → 不生效（零偏恢复 `bias_ref`）
 * @note    抽成函数是为了让“双判”只有一处拼写，固件与单测同源。
 */
int temp_comp_should_apply(int tempcomp_en, int tc_valid);

#endif /* CAL_TEMP_COMP_H */
