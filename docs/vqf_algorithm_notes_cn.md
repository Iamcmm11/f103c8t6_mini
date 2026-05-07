# VQF（managers/vqf.cpp）函数与参数速查（聚焦抗磁干扰）

> 目标：用最少的字说明 `VQF` 在本工程中的实现（`managers/vqf.cpp/.hpp`）——每个函数做什么、有哪些默认/可调参数，以及**磁场干扰检测/抑制**相关机制。

## 0. 从0到1：算法做了什么（整体数据流）

VQF 将姿态分解为三部分状态：

- `state.gyrQuat`（3D）：仅靠陀螺积分得到的姿态（strapdown integration）。
- `state.accQuat`（6D）：用加速度计做**俯仰/横滚（inclination）**校正的四元数（不包含 yaw）。
- `state.delta`（9D yaw 修正）：用磁力计对 yaw 的一阶滤波修正角（以 `delta` 表示）。

输出：

- 6D 姿态：`q6D = accQuat ⊗ gyrQuat`
- 9D 姿态：`q9D = ApplyDelta(q6D, delta)`（对 6D 再叠加一个绕 z 轴的 yaw 旋转）

典型调用顺序（同频率）：

- `update(gyr, acc)`：3D 陀螺积分 + 6D 倾角校正
- `update(gyr, acc, mag)`：在上面基础上增加磁力计 yaw 校正（含抗磁干扰逻辑）

当传感器不同频率：按 `updateGyr()` -> `updateAcc()` -> `updateMag()` 的顺序分别喂数据。

---

## 1. 参数（VQFParams）：默认值与可调含义

`VQFParams()` 的默认值在 `vqf.cpp` 构造函数初始化列表中。

### 1.1 加速度/磁力计融合强度（最常用）

- `tauAcc = 3.0 s`
  - **作用**：加速度在惯性系的二阶 Butterworth 低通时间常数（用于倾角校正）。
  - **调小**：更信任加速度，倾角更快贴合，但更容易被线加速度/震动影响。
  - **调大**：更信任陀螺，倾角更稳但收敛更慢。

- `tauMag = 9.0 s`
  - **作用**：yaw 校正的一阶滤波时间常数（`delta` 更新的等效带宽）。
  - **调小**：更信任磁力计，yaw 跟得快，但更容易受磁干扰带偏。
  - **调大**：更信任陀螺，抗磁干扰更强但长期漂移更明显。

### 1.2 陀螺零偏估计（与磁无关，但对稳态很关键）

- `restBiasEstEnabled = true`
  - **作用**：开启静止检测 + 静止时零偏估计。

- `motionBiasEstEnabled = true`（若未定义 `VQF_NO_MOTION_BIAS_ESTIMATION`）
  - **作用**：开启运动中零偏估计（只基于倾角校正，不用磁力计）。

- `biasSigmaInit = 0.5 °/s`
  - **作用**：初始零偏不确定度（KF 初值）。

- `biasForgettingTime = 100 s`
  - **作用**：KF 系统噪声强度（越小忘得越快，零偏更“活”）。

- `biasClip = 2.0 °/s`
  - **作用**：零偏与零偏更新误差的裁剪上限，同时也用于避免“匀速转动”被误判为静止。

- `biasSigmaMotion = 0.1 °/s`（运动）
  - **作用**：运动零偏估计的收敛速度/信任度。
  - **越小**：收敛更快但更容易被异常加速度/姿态误差影响。

- `biasVerticalForgettingFactor = 0.0001`
  - **作用**：运动时垂直方向零偏不可观，使用“人工趋零”权重防止漂走。

- `biasSigmaRest = 0.03 °/s`（静止）
  - **作用**：静止零偏估计更新权重。

### 1.3 静止检测（影响静止零偏、也影响磁“新场接受”计时）

- `restMinT = 1.5 s`
- `restFilterTau = 0.5 s`
- `restThGyr = 2.0 °/s`
- `restThAcc = 0.5 m/s²`

静止检测条件（实现要点）：

- 对 `gyr/acc` 先做低通得到参考值。
- 看测量与参考的偏差平方和是否小于阈值。
- 陀螺还额外要求每轴绝对值 < `biasClip`，避免匀速转动误判为 rest。

### 1.4 抗磁干扰相关参数（重点）

- `magDistRejectionEnabled = true`
  - **作用**：开启磁干扰检测 + 拒绝/降权磁更新。

- `magCurrentTau = 0.05 s`
  - **作用**：对“当前磁场强度(norm)+倾角(dip)”做一个很快的低通，提升在噪声/不同步采样时的鲁棒性。
  - **可设为 -1**：禁用该低通，直接用原始 mag 推导的 norm/dip。

- `magRefTau = 20.0 s`
  - **作用**：参考磁场（`magRefNorm/magRefDip`）在“未干扰”时的缓慢自适应。

- `magNormTh = 0.1`（相对阈值）
  - **作用**：|norm - refNorm| < `magNormTh * refNorm` 判为强度一致。

- `magDipTh = 10.0 deg`
  - **作用**：|dip - refDip| < `magDipTh` 判为倾角一致。

- `magMinUndisturbedTime = 0.5 s`
  - **作用**：连续满足阈值这么久才把 `magDistDetected` 置为 false（避免抖动）。

- `magMaxRejectionTime = 60.0 s`
  - **作用**：检测到干扰后的**完全拒绝窗口**上限。

- `magRejectionFactor = 2.0`
  - **作用**：超过 `magMaxRejectionTime` 后，不再完全拒绝，而是把磁更新增益 `k` 降为 `k/ magRejectionFactor`（等效更大 `tauMag`）。

- `magNewTime = 20.0 s` / `magNewFirstTime = 5.0 s`
  - **作用**：在干扰长期存在时，允许“接受一个新的均匀磁场”为参考磁场。
  - **第一次**（ref 未建立，refNorm==0）用 `magNewFirstTime` 更快建立参考。

- `magNewMinGyr = 20.0 °/s`
  - **作用**：只有在“运动足够大”时才累计新场候选的时间（避免静止时被局部扰动误带偏）。

---

## 2. 主要对外接口函数：作用与注意事项

### 2.1 构造与初始化

- `VQF::VQF(gyrTs, accTs=-1, magTs=-1)`
  - **作用**：用默认参数创建；若 `accTs/magTs<0` 则使用 `gyrTs`。

- `VQF::VQF(params, gyrTs, accTs=-1, magTs=-1)`
  - **作用**：用自定义 `VQFParams` 创建。

- `setup()`（protected）
  - **作用**：把参数+采样周期转换成系数 `coeffs`（滤波系数、KF 噪声等），最后 `resetState()`。

- `resetState()`
  - **作用**：恢复到“刚初始化”的状态（四元数单位阵、delta=0、bias=0、滤波器 state=NaN 等）。

### 2.2 更新函数（核心）

- `updateGyr(gyr)`
  - **输入**：`gyr` (rad/s)
  - **做什么**：
    - 静止检测用的陀螺低通 + deviation 计算（rest、mag 新场接受依赖 `restLastGyrLp`）。
    - 用当前 `state.bias` 去除零偏，进行陀螺积分更新 `state.gyrQuat`。

- `updateAcc(acc)`
  - **输入**：`acc` (m/s²)，若 `[0 0 0]` 直接忽略。
  - **做什么**：
    - 静止检测（acc 部分）：acc deviation 小则累计 `restT`，超过 `restMinT` 置 `restDetected=true`。
    - 将 acc 旋到惯性系并低通（`tauAcc`），再计算倾角校正四元数 `accCorrQuat`，更新 `state.accQuat`。
    - 进行零偏估计（KF）：
      - 若 `restDetected` 且启用 rest：用 `restLastGyrLp` 直接更新 bias（最可靠）。
      - 否则若启用 motion：利用倾角校正引入的关系更新 bias（不使用磁力计）。

- `updateMag(mag)`（抗磁干扰的核心）
  - **输入**：`mag`（任意单位），若 `[0 0 0]` 忽略。
  - **做什么（分两块）**：

  1) **磁干扰检测/参考更新**（仅 `magDistRejectionEnabled` 时）
  - 将 mag 旋到 6D earth frame（基于 `getQuat6D()`）。
  - 计算：
    - `norm = ||magEarth||`
    - `dip = -asin(magEarth[z]/norm)`
  - `magCurrentTau` 可选低通 norm/dip。
  - 若 norm/dip 与 `magRefNorm/magRefDip` 足够接近并持续 `magMinUndisturbedTime`：
    - 认为未干扰：`magDistDetected=false`
    - 参考以 `magRefTau` 的速度缓慢跟随当前值（`kMagRef`）。
  - 否则：`magDistDetected=true`。

  2) **yaw 修正（delta 一阶滤波） + 干扰拒绝策略**
  - 用当前 mag 测得 heading 与当前 `delta` 的差得到 `lastMagDisAngle`（包到 [-pi,pi]）。
  - 默认增益 `k = kMag = gainFromTau(tauMag, magTs)`。
  - 若检测到干扰：
    - 在 `magRejectT <= magMaxRejectionTime` 内：`k=0`（完全拒绝磁更新，只靠陀螺跟 yaw）。
    - 超过后：`k = k / magRejectionFactor`（不完全拒绝，但显著降权，防止 yaw 永久漂移）。
  - 最后：`delta += k * lastMagDisAngle` 并 wrap 到 [-pi,pi]。

- `update(gyr, acc)` / `update(gyr, acc, mag)`
  - **作用**：按正确顺序组合调用。

- `updateBatch(...)`
  - **作用**：批量处理 N 组数据（同采样率），可选择输出 6D/9D/delta/bias/sigma/rest/magDist。

### 2.3 结果获取与状态控制

- `getQuat3D(out)`：输出 `gyrQuat`
- `getQuat6D(out)`：输出 `accQuat ⊗ gyrQuat`
- `getQuat9D(out)`：输出 `ApplyDelta(getQuat6D(), delta)`
- `getDelta()`：输出 `delta`

- `getBiasEstimate(out)`：输出 bias（rad/s），返回“最坏方向”的 sigma（rad/s）
- `setBiasEstimate(bias, sigma=-1)`：设置 bias，可选设置协方差（sigma>0 时重置 P）

- `getRestDetected()`：静止检测标志
- `getMagDistDetected()`：磁干扰检测标志
- `getRelativeRestDeviations(out2)`：返回相对阈值偏差（<1 才可能 rest）

- `getMagRefNorm()/getMagRefDip()`：当前磁参考
- `setMagRef(norm, dip)`：强制覆盖磁参考（用于标定或外部管理策略）

---

## 3. 运行期可调接口（不重建对象即可调）

这些函数会修改 `params` 并同步维护内部滤波器状态，适合在线调参：

- `setTauAcc(tauAcc)`
  - **会做额外处理**：调用 `filterAdaptStateForCoeffChange()`，尽量避免低通输出跳变。

- `setTauMag(tauMag)`：直接更新 `coeffs.kMag`。

- `setMotionBiasEstEnabled(enabled)`：切换运动零偏估计并清空相关低通 state。

- `setRestBiasEstEnabled(enabled)`：切换静止检测/静止零偏并重置 rest 相关状态。

- `setMagDistRejectionEnabled(enabled)`：切换磁干扰检测/拒绝，并重置磁参考/候选与滤波状态。

- `setRestDetectionThresholds(thGyr, thAcc)`：在线改静止阈值。

> 注意：大量参数（如 `magNormTh/magDipTh/magMaxRejectionTime/...`）当前没有单独 setter，只能通过构造时传入 `VQFParams`，或直接 `getParams()` 拿到后重新构造/自写 setter。

---

## 4. 抗磁场干扰：实现策略总结（你调参时应该盯什么）

### 4.1 干扰判定用的“特征量”

VQF 不直接用磁的水平投影做 gating，而是用更稳健的：

- **磁场强度**：`norm = ||magEarth||`
- **磁倾角**：`dip = -asin(magEarth[z]/norm)`

并与参考 `(magRefNorm, magRefDip)` 比较。

### 4.2 三段式拒绝策略（核心机制）

- **短时干扰**：直接 `k=0`，yaw 完全不被磁影响。
- **长时干扰**：超过 `magMaxRejectionTime` 后仍允许磁慢慢拉回（`k/=magRejectionFactor`），避免纯陀螺 yaw 漂移无限增长。
- **恢复机制**：干扰解除后，`magRejectT` 以 `magRejectionFactor*Ts` 的速度回退到 0，逐步恢复“可再次完全拒绝”的能力。

### 4.3 “新磁场接受”（避免永远认为干扰）

当当前磁场长期稳定但与旧参考不一致时：

- 用 `candidateNorm/Dip` 跟踪候选磁场。
- 只有在角速度足够大（`magNewMinGyr`）才累计候选时间。
- 达到 `magNewTime`（或首次 `magNewFirstTime`）后，接受新参考。

### 4.4 实战调参建议（按现实现象对症）

- **附近有强磁/电机，yaw 经常被带偏**：
  - 增大 `tauMag`
  - 或增大 `magNormTh/magDipTh`（更宽容，反而可能减少误判干扰；但会降低检测灵敏度，需要结合场景）
  - 更直接：减小 `magNormTh`/`magDipTh`（更敏感，更容易触发拒绝）

- **磁噪声大/采样不同步导致误判干扰抖动**：
  - 适当增大 `magCurrentTau`（例如 0.1~0.2）
  - 增大 `magMinUndisturbedTime`

- **长时间处于“看起来一直干扰”，yaw 漂移严重**：
  - 减小 `magMaxRejectionTime`（更快进入“降权但不为0”的模式）
  - 减小 `magRejectionFactor`（降权幅度更小，yaw 更容易被磁慢慢拉住）
  - 启用并合理设置 `magNewTime/magNewMinGyr`，让系统学会新环境的“正常磁场”

---

## 5. 本工程动捕系统：设计思路与实战总结

这一节不是讲 VQF 数学细节，而是从“工程/面试”的角度，总结本项目里这套多 IMU 动捕系统是怎么围绕 VQF 设计的、做了哪些取舍、遇到什么坑、怎么解决。

- `VQFParams::VQFParams()`：设置所有默认参数。
- `VQF::VQF(...)`：构造并 `setup()`。
- `updateGyr / updateAcc / updateMag`：三传感器分步更新。
- `update(...)`：便捷组合更新。
- `updateBatch(...)`：批量更新并可选输出。
- `getQuat3D / getQuat6D / getQuat9D / getDelta`：姿态输出。
- `getBiasEstimate / setBiasEstimate`：零偏估计读写。
- `getRestDetected / getMagDistDetected / getRelativeRestDeviations`：状态观测。
- `getMagRefNorm / getMagRefDip / setMagRef`：磁参考观测/设置。
- `setTauAcc / setTauMag / setRestDetectionThresholds`：在线调参。
- `setMotionBiasEstEnabled / setRestBiasEstEnabled / setMagDistRejectionEnabled`：在线开关特性。
- `getParams / getCoeffs / getState / setState / resetState`：调试/状态管理。

内部工具（static/protected）：

- 四元数工具：`quatMultiply/quatConj/quatSetToIdentity/quatApplyDelta/quatRotate`
- 数学工具：`norm/normalize/clip`
- 一阶 gain：`gainFromTau`
- 二阶 Butterworth：`filterCoeffs/filterInitialState/filterAdaptStateForCoeffChange/filterStep/filterVec`
- 矩阵工具（KF 用）：`matrix3SetToScaledIdentity/matrix3Multiply/matrix3MultiplyTpsFirst/matrix3MultiplyTpsSecond/matrix3Inv`
- `setup()`：计算所有 `coeffs` 并 reset。

---

## 6. Blender 端手指驱动：映射方案与门限机制（2026-01 更新）

本节总结 `utils/python/blender_udp_mocap.py` 中**手指 IMU 四元数 -> Mixamo 手指骨骼**的映射策略与门限机制。

### 6.1 总体思路（从0到1）

1. **输入**：每根手指 1 个 IMU 的四元数（协议层解析后进入 Blender 端脚本）。
2. **统一预处理**：`apply_calib_and_axis(key, q_raw)`
   - 负责：镜像/共轭预处理、轴修正（axis-fix）、零位校准（calib）。
3. **生成 MCP 输入四元数**：
   - 除拇指外：调用 `apply_finger_constraints(...)` 进行“扭转抑制 + Z 侧摆限位”。
   - 拇指：走单独的单轴映射（见 6.2.1）。
4. **分段输出到骨骼**：每根手指输出 1~3 个四元数，分别赋给 `*1/*2/*3`。

### 6.2 各手指映射方案

#### 6.2.1 拇指（thumb）

骨骼映射：`Thumb1 / Thumb2 / Thumb3`。

- **只映射单一分量**：从 IMU 四元数中提取“绕 X 的屈伸分量”，并映射为**模型绕 Z 的单轴旋转**：
  - 入口函数：`thumb_x_flex_to_model_z_only(q)`
  - 目的：让拇指只显示一个自由度，便于调试与控制。
- **单向门限（反向刚性）**：
  - 当提取到的屈伸角不满足“允许方向 + 超过阈值”时，直接返回单位四元数（不转）。
  - 阈值当前为 `0.02 rad`（约 1.15°），用于防抖。
- **三段比例**：
  - Thumb1/2/3 都由同一个 Z 单轴角度按比例系数缩放得到（系数在 `_apply_pose()` 的 thumb 分支中直接写死，可手动微调）。

#### 6.2.2 食指/中指/无名指/小拇指（index/middle/ring/pinky）

骨骼映射：各自的 `*1/*2/*3`。

- **MCP（第1节）**：
  - 输入先经过 `apply_finger_constraints(q_in, ..., finger_key=key)`：
    - Swing-Twist 分解（twist 轴为局部 Y），按 `Finger Twist Weight` 抑制轴向扭转。
    - Z 侧摆按门限规则限位（见 6.3）。
  - MCP 输出使用 `scale_positive_flexion_keep_rest(q2, root_ratio)`：
    - 只对“正向 X 屈伸”缩放。
    - 保留 MCP 的其它分量（例如侧摆），避免侧摆特征被抹掉。
- **PIP/DIP（第2/3节）**：
  - 使用 `get_flexion_only_quat(q2, ratio)`：只提取**正向 X 屈伸**并按比例缩放。
  - 若为反向/侧向（角度过小或轴方向不满足阈值），直接返回单位四元数，实现“反向刚性不弯”。
- **三段比例可单独调**：
  - `root_ratio/mid_ratio/tip_ratio` 在 `_apply_pose()` 里对四指分别列出，便于微调。

### 6.3 Z 侧摆门限（MCP）方向性限制（防交叉）

在 `apply_finger_constraints()` 内部对 `rotvec.z` 使用 `_directional_limit_angle()` 实现“单侧阈值 + 另一侧卡死”。

- **中指（middle）**：侧摆固定约 `±2°`。
- **无名指/小拇指（ring/pinky）**：只允许一侧侧摆，另一侧直接卡死为 0（避免向不可能方向侧摆导致交叉）。
- **食指（index）**：与 ring/pinky 相反侧。
- **参数来源**：
  - 单侧允许幅度：复用 UI 的 `Finger Z Limit (deg)`
  - 软度：复用 UI 的 `Finger Z Softness`

> 备注：“向左/向右”的判定当前取决于 `rotvec.z` 的正负号；若发现与模型直觉相反，需要对调 index 与 ring/pinky 的单侧方向。

### 6.4 与零位（Zero）功能的关系（简述）

- `Set Zero (IMU Frame)`：把按下按钮那一帧 IMU 姿态当作零位；之后模型显示的是相对变化（按下时的手势会成为新的“0”）。
- `Set Zero (Current Pose)`：把当前模型姿态当作零位（不是回到模型 Rest Pose）。
