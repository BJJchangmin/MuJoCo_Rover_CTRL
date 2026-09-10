# Soft-Soil-Aware Integrated Rover Control

## Research and MuJoCo Implementation Handoff

이 문서는 Linux 환경에서 MuJoCo 기반 rover controller 구현을 이어갈 Codex에게 연구 배경, 단계별 구현 순서, 검증 기준과 핵심 문헌을 전달하기 위한 handoff 문서다.

---

## 0. 다음 Codex가 이 문서를 사용하는 방법

1. 최종 controller를 한 번에 구현하지 않는다.
2. 먼저 repository 전체 구조와 기존 MuJoCo rover model/controller를 읽는다.
3. 기존 사용자 코드를 보존하고, 관련 없는 파일은 수정하지 않는다.
4. 아래의 Stage 1부터 순서대로 구현하며 각 stage gate를 통과한 뒤 다음 단계로 간다.
5. inequality/equality와 weight를 바꿀 때마다 변경 이유와 결과를 기록한다.
6. soft-soil controller를 먼저 만들지 않는다. Rigid-ground controller가 rigid plant에서 정상임을 확인한 뒤, 동일 controller를 soft-soil plant에 그대로 적용하여 failure를 먼저 관찰한다.
7. repository에서 확인할 수 있는 정보는 사용자에게 다시 묻지 않는다. 물리 파라미터나 actuator 의미처럼 구현 결과를 크게 바꾸는 정보만 질문한다.

---

## 1. 연구의 최종 목표

저밀도·변형 토양의 wheel–soil interaction과 wheel contact mode를 고려하여, 능동 서스펜션 rover의 자세·주행·접촉력을 통합적으로 제어하는 것이 최종 목표다.

최종 연구 방향은 다음과 같이 표현할 수 있다.

> **Soft-soil-aware, contact-mode-dependent integrated motion and force control for an actively suspended planetary rover**

최종적으로 고려하려는 접촉 동역학의 골격은

\[
M(q)\ddot q+h(q,\dot q)
=S^\top\tau+J_{\mathcal C}^{\top}(q)\lambda_{\mathcal C}
\]

이며,

- \(\mathcal C(t)\): 현재 활성 wheel-contact 집합
- \(\lambda_i=[F_{x,i},F_{y,i},F_{z,i}]^\top\): wheel contact force
- \(J_{\mathcal C}\): 활성 contact의 Jacobian
- \(\tau\): drive, steering, suspension actuator input

이다.

Soft soil의 실제 force feasibility는 contact Jacobian과 별도로

\[
\lambda_i\in\mathcal F_{\mathrm{soil},i}
(F_{z,i},z_i,\kappa_i,\alpha_i,\theta_{\mathrm{soil}},\text{soil history})
\]

로 표현해야 한다.

### 핵심 구분

- \(J_{\mathcal C}^{\top}\lambda_{\mathcal C}\): wheel force가 rover body에 만드는 generalized force/wrench
- \(\mathcal F_{\mathrm{soil}}\): 해당 force가 현재 토양에서 실제로 생성 가능한지
- \(\mathcal C(t)\): 현재 어떤 wheel이 접촉 중인지

Contact Jacobian 자체는 terramechanics model이 아니다.

---

## 2. 연구의 중심 가설과 인과관계

박사 proposal과 논문에서 먼저 보여야 할 것은 단순히 “soft soil이 어렵다”가 아니다.

확인해야 할 중심 인과관계는 다음과 같다.

\[
\begin{aligned}
&\text{Rigid-ground force feasibility assumption}\\
&\rightarrow \text{physically incorrect wheel-force allocation on soft soil}\\
&\rightarrow \text{force tracking failure / excessive slip / sinkage}\\
&\rightarrow \text{body tracking degradation / contact loss / immobilization}
\end{aligned}
\]

그 후에 다음 가설을 검증한다.

> Soil-dependent force feasibility와 active normal-load redistribution을 controller에 포함하면, 동일한 body-wrench demand를 더 낮은 soil utilization과 sinkage risk로 실현할 수 있다.

따라서 연구 흐름은 반드시 다음 순서를 따른다.

\[
\boxed{
\text{Rigid controller validation}
\rightarrow
\text{Soft-plant failure observation}
\rightarrow
\text{Failure-cause identification}
\rightarrow
\text{Minimal soil-aware modification}
}
\]

---

## 3. 반드시 유지할 개념적 원칙

### 3.1 \(F_z\)가 yaw를 직접 생성하는 것은 아니다

Yaw moment는 주로 좌우 \(F_x\) 또는 \(F_y\) 비대칭에서 발생한다. \(F_z\)는 각 wheel의 traction capacity를 바꾸어 yaw에 간접적으로 영향을 준다. 따라서 Stage 1에서 yaw 발생을 미리 결론 내리지 말고, Stage 2–3에서 force asymmetry와 함께 분석한다.

### 3.2 Contact Jacobian은 Stage 4에서 처음 등장하는 것이 아니다

Contact-force-to-body-wrench mapping에는 초기 단계부터 Jacobian 또는 등가 wrench mapping이 필요하다. Stage 4의 새로운 요소는

\[
\mathcal C=\{1,2,3,4\}
\rightarrow
\mathcal C(t)\subseteq\{1,2,3,4\}
\]

처럼 활성 contact set이 변하는 것이다.

### 3.3 Equal load는 최종 목적이 아니다

Rigid/homogeneous 조건에서는 normal-force 균등화가 유용한 baseline이다. 그러나 soft soil에서는 동일한 \(F_z\)라도 soil state, slip, sinkage와 multipass history에 따라 traction capacity가 다를 수 있다. 최종 목적은 단순한 equal load보다 soil-feasibility margin 또는 mobility-risk 최소화에 가깝다.

### 3.4 MuJoCo의 기본 compliant contact를 곧바로 terramechanics로 간주하지 않는다

MuJoCo의 penetration, stiffness, damping만으로 Bekker pressure–sinkage 및 Janosi–Hanamoto shear 관계가 자동으로 구현되는 것은 아니다. Rigid 단계는 MuJoCo 기본 contact로 controller를 검증한다. Soft-soil plant는 custom force model, lookup/surrogate model 또는 검증된 별도 soil-contact layer로 구성한다.

### 3.5 처음에는 soil estimator를 넣지 않는다

Soft-soil controller의 필요성과 potential을 먼저 분리해서 검증하기 위해, 초기 실험에서는 실제 plant와 동일한 known soil parameter를 controller에 제공하는 oracle 조건을 사용한다. Oracle에서도 개선되지 않는다면 estimator보다 method 또는 actuator authority가 먼저 문제다.

---

## 4. 전체 7단계 로드맵

| Stage | Terrain/contact | Controller | 핵심 확인 질문 |
|---|---|---|---|
| 1 | Rigid uneven, fixed 4-contact | \(F_z\) QP | Orientation/height와 normal-force distribution을 동시에 제어할 수 있는가? |
| 2 | Rigid uneven, fixed 4-contact | \(F_x-F_z\) joint QP | Normal load 변화가 traction, velocity, yaw 및 force realization에 어떻게 영향을 주는가? |
| 3 | Rigid uneven, fixed 4-contact | \(F_x-F_y-F_z\) QP | Lateral force와 steering까지 통합할 실질적 필요가 있는가? |
| 4 | Rigid uneven, variable contact | Active-contact QP | Contact loss 시 \(J_{\mathcal C}\) 변경과 remaining-contact force reallocation이 가능한가? |
| 5 | Soft-soil uneven, fixed contact | 기존 rigid QP 그대로 | Rigid-ground formulation이 soft soil에서 어떤 잘못된 결정을 내리는가? |
| 6 | Soft-soil uneven, fixed contact | Soil-aware \(F_z\), 이후 integrated QP | Slip–sinkage–traction coupling을 반영하면 allocation과 mobility가 개선되는가? |
| 7 | Soft-soil uneven, variable contact | Soil-aware active-contact integrated QP | Soil feasibility와 contact mode를 동시에 고려해 안정화·재접촉할 수 있는가? |

Stage 5는 새 controller 개발 단계가 아니라 causal ablation 단계다.

---

## 5. 단계별 구현 명세

## Stage 1 — Rigid uneven, fixed 4-contact, \(F_z\) QP

### 목적

- MuJoCo rover dynamics와 actuator mapping 검증
- Body height, roll, pitch 제어
- Normal-force concentration 완화
- 네 wheel의 양의 contact margin 유지

### 권장 decision variable

첫 구현은

\[
x=F_z=[F_{z,1},F_{z,2},F_{z,3},F_{z,4}]^\top
\]

로 단순화한다. 필요하면 suspension actuator force/torque를 함께 풀되, force allocation과 lower-level realization을 먼저 분리하는 편이 debugging에 유리하다.

### Equality 후보

- Total vertical force 또는 desired vertical acceleration
- Desired roll moment
- Desired pitch moment
- 필요할 경우 quasi-static body-wrench balance

일반적으로

\[
A_z(q)F_z=w_{z,d}+s_z
\]

형태로 두고, 불가능한 terrain geometry에서 infeasibility가 발생하지 않도록 tracking slack \(s_z\)를 검토한다.

### Inequality 후보

- \(F_{z,i}\ge 0\)
- 필요 시 \(F_{z,i}\ge F_{z,\min}-s_{c,i}\)
- Suspension force/torque limits
- Suspension stroke/velocity limits
- \(|\Delta F_{z,i}|\) 또는 force-rate limits

모든 wheel에 큰 양의 \(F_{z,\min}\)을 hard constraint로 강제하면 기하학적으로 4-contact가 불가능할 때 QP가 infeasible해질 수 있다. Contact margin은 slack을 포함한 soft constraint 후보로 둔다.

### Objective 후보

\[
\min
\|A_zF_z-w_{z,d}\|_Q^2
+w_N\operatorname{Var}(F_z)
+w_\Delta\|F_z-F_{z,\mathrm{prev}}\|^2
+w_s\|s\|^2
\]

### 필수 baseline

- 기존 kinematic orientation controller
- 수동 또는 nominal suspension
- Equal \(F_z\) reference

### Metric

- Height/roll/pitch RMSE와 peak error
- Normal Force Dispersion
- \(\min_iF_{z,i}\)
- Suspension torque/force saturation
- QP slack와 infeasible count
- Solver time

### Stage gate

- Rigid uneven terrain에서 안정적으로 주행
- Body tracking과 force distribution 사이 trade-off를 설명할 수 있음
- QP solution이 실제 suspension force로 재현됨
- Weight 변화에 따른 결과가 재현 가능함

---

## Stage 2 — Rigid uneven, fixed contact, \(F_x-F_z\) joint QP

### 목적

- Longitudinal tracking과 active load distribution의 coupling 확인
- \(F_z\) 변화가 각 wheel traction authority에 미치는 영향 확인
- Force-based upper layer와 wheel torque/slip lower layer 연결

### 권장 decision variable

\[
x=[F_{x,1:4},F_{z,1:4}]^\top
\]

### Equality 후보

- Desired total longitudinal force
- Desired yaw moment를 사용하는 경우 좌우 \(F_x\) moment balance
- Vertical force, roll moment, pitch moment
- Body-wrench mapping

\[
A_{xz}(q)
\begin{bmatrix}F_x\\F_z\end{bmatrix}
=w_{xz,d}+s_w
\]

### Inequality 후보

- \(|F_{x,i}|\le\mu_iF_{z,i}\) 또는 polygonal friction constraint
- \(F_{z,i}\ge0\)
- Wheel torque limits
- Suspension force/torque/stroke limits
- Force and command rate limits

### Lower-layer realization

Upper QP가 force를 출력할 경우 실제 command는 예를 들어

\[
F_{x,i}^{*}\rightarrow\kappa_i^{*}
\rightarrow\omega_i^{*}\rightarrow T_{w,i}
\]

로 내려간다. Wheel dynamics

\[
I_w\dot\omega_i=T_{w,i}-rF_{x,i}
\]

를 무시하면 정적 allocation이 실제로 구현되지 않을 수 있다.

### Metric

- Desired/actual \(F_x\) tracking
- Longitudinal velocity RMSE
- Wheel slip ratio
- Wheel torque and suspension saturation
- Left/right \(F_x\), \(F_z\) asymmetry
- Yaw rate와 yaw moment decomposition
- Friction utilization

### Stage gate

- \(F_z\)-only와 joint allocation 차이를 수치로 설명할 수 있음
- Yaw 또는 velocity degradation이 어떤 force asymmetry에서 발생했는지 설명 가능
- Lower-level force realization이 충분히 정확함

---

## Stage 3 — Rigid uneven, fixed contact, \(F_x-F_y-F_z\) QP

### 시작 조건

Stage 2에서 lateral error, yaw error 또는 steering–traction conflict가 실제로 관찰될 때만 진행한다.

### Decision variable

\[
x=[F_{x,1:4},F_{y,1:4},F_{z,1:4}]^\top
\]

### 핵심 constraints

- 3D desired body wrench
- Friction-circle/ellipse의 convex polygon approximation
- Steering/drive/suspension actuator limits
- Unilateral normal contact
- Force-rate limits

### 핵심 질문

- \(F_y\)를 optimizer에 넣었을 때 yaw/path tracking 개선이 유의미한가?
- 계산량과 model uncertainty 증가가 개선폭에 비해 정당한가?
- 기존 steering/path controller를 유지하고 \(F_x-F_z\)만 통합하는 것으로 충분한가?

개선이 작다면 Stage 3은 최종 controller의 필수 구성요소가 아닐 수 있다.

---

## Stage 4 — Rigid uneven, variable contact, active-contact QP

### 목적

- Contact loss detection
- Inactive wheel force 제거
- Remaining-contact force reallocation
- 3-contact stability
- Controlled recontact

### Contact-mode 처리

\[
c_i=0\Rightarrow\lambda_i=0,
\qquad
J_{\mathcal C}=[J_i]_{c_i=1}
\]

### 추천 초기 구조

처음부터 contact-implicit NMPC를 구현하지 않는다.

\[
\boxed{
\text{contact estimator}
+\text{hybrid supervisor}
+\text{active-set QP}
}
\]

### Contact evidence 후보

- Normal contact force
- Contact constraint/penetration state
- Wheel-ground distance
- Suspension configuration and rate
- Wheel current/torque와 wheel angular acceleration

단일 threshold보다 hysteresis, debounce time 및 confidence를 사용한다.

### Recontact 순서

1. Inactive wheel에 \(\lambda_i=0\) 적용
2. Remaining contacts로 feasible wrench 재계산
3. Swing wheel suspension을 내려 ground gap 감소
4. 예상 ground speed와 wheel peripheral speed 동기화
5. Touchdown 후 \(F_z\) reference를 ramp로 증가
6. 4-contact set으로 복귀

### Metric

- Contact detection delay/false switching
- 3-contact body-wrench residual
- Support/contact stability margin
- Recontact time
- Touchdown force peak
- Chattering과 QP infeasibility

---

## Stage 5 — 기존 rigid QP + soft-soil uneven plant

### 목적

Rigid-ground controller가 soft soil에서 왜 잘못된 결정을 내리는지 확인한다. Controller formulation과 weight를 변경하지 않는다.

### 최소 비교 실험

| Case | Plant | Controller | 목적 |
|---|---|---|---|
| A | Rigid uneven | Rigid-ground QP | Controller sanity check |
| B | Soft-soil uneven | A와 동일한 QP | Soil을 무시한 failure 확인 |
| C | Soft-soil uneven | Minimal oracle soil-aware QP | Soil information의 potential 확인 |
| D, optional | Rigid uneven | Soil-aware QP | 불필요한 conservatism 확인 |

가능하면 초기 macro terrain geometry, reference trajectory, controller weight, initial state와 command를 동일하게 유지한다.

### 먼저 고정할 조건

- Fixed 4-contact
- Known homogeneous soil parameter
- Straight 또는 완만한 steering
- 먼저 \(F_x-F_z\), 이후 필요 시 \(F_y\)

### 반드시 보여야 할 failure chain

1. Rigid QP가 요구한 wheel force
2. Soft-soil plant에서 실제 생성 가능한 force
3. 요구 force가 soil feasibility를 얼마나 위반했는지
4. Slip/sinkage/force-tracking failure
5. Body tracking과 contact margin의 downstream degradation

Soft soil에서 tracking error만 커졌다는 결과는 원인 증거로 부족하다.

---

## Stage 6 — Soft-soil-aware fixed-contact allocation

### 첫 단계

Known soil parameter를 사용하여 soft-soil feasibility를 controller에 넣었을 때 이점이 있는지 검증한다.

### Soil-aware force feasibility

Rigid friction cone을 그대로 쓰는 대신

\[
F_{t,i}\in
\mathcal F_{\mathrm{soil},i}
(F_{z,i},z_i,\kappa_i,\alpha_i,\theta_{\mathrm{soil}})
\]

또는

\[
\rho_i=
\frac{\|F_{t,i}\|}
{F_{t,\max,i}(F_{z,i},z_i,\kappa_i,\theta_{\mathrm{soil}})}
\]

를 사용한다.

Terramechanics constraint가 비선형이면 QP를 유지하기 위해 다음 후보를 비교한다.

- Operating-point linearization
- Sequential QP
- Lookup-table 기반 local affine bound
- Piecewise-linear soil-force polytope
- Conservative convex inner approximation

### Objective 후보

\[
\min
\|W(\lambda)-W_d\|_Q^2
+w_\rho\max_i\rho_i^2
+w_z\sum_i\hat z_i^2
+w_N\operatorname{Var}(F_z)
+w_u\|\Delta u\|^2
+w_s\|s\|^2
\]

Equal \(F_z\) 또는 minimum slip만을 최종 objective로 고정하지 않는다.

### 필수 비교

- Rigid force constraint vs soil-aware force constraint
- Equal-load objective vs soil-utilization objective
- Static force allocation vs wheel-dynamics-aware realization
- True soil parameter vs perturbed parameter

### Stage gate

- Soil-aware allocation이 실제 wheel-force command를 유의미하게 변경함
- 그 변화가 slip/sinkage/force tracking/body tracking 중 하나 이상을 개선함
- 개선 원인이 단순한 command reduction이 아니라 feasible redistribution임을 설명 가능

---

## Stage 7 — Soft-soil-aware variable-contact integrated control

### 최종 method 후보

Mode-dependent soil-feasible body-wrench set을 정의한다.

\[
\mathcal W_{\mathrm{soil}}(q,\mathcal C,\hat\theta)
=
\left\{
J_{\mathcal C}^{\top}\lambda_{\mathcal C}
\mid
\lambda_i\in\mathcal F_{\mathrm{soil},i}
\right\}
\]

Controller는 desired wrench를 현재 contact mode와 soil condition에서 가능한 wrench set으로 투영하거나, wrench residual과 mobility risk를 함께 최소화한다.

### 해결해야 할 문제

- Soft soil에서 contact loss 판정의 불확실성
- 3-contact soil-feasible wrench 유지
- Recontact 중 wheel speed/soil shear 상태 동기화
- Touchdown force와 재침하 제한
- Contact switching 시 QP continuity와 feasibility

### 현실적인 첫 구현

- Discrete contact mode supervisor
- Mode별 convex QP
- Soil constraint local linearization
- Recontact state machine

Contact-implicit NMPC는 후속 확장으로 둔다.

---

## 6. 공통 로깅 및 결과 분석 규칙

모든 실험에서 최소한 다음 신호를 동일한 이름과 단위로 저장한다.

- Body position, velocity, roll, pitch, yaw, angular rates
- Desired body wrench와 achieved body wrench
- Wheel별 desired/actual \(F_x,F_y,F_z\)
- Wheel torque, speed, slip ratio, slip angle
- Suspension position, velocity, force/torque
- Contact flag/probability, wheel-ground distance, penetration
- Sinkage와 estimated soil utilization
- QP objective terms, slack, solver status, iteration, solve time
- Active contact set

결과를 보고할 때는 다음 구조를 지킨다.

1. **WHAT:** 무엇을 비교했는가?
2. **NUMBER:** 핵심 수치는 무엇인가?
3. **BASELINE:** 무엇과 비교했는가?
4. **DELTA:** 얼마나 달라졌는가?
5. **WHY:** 어떤 force/constraint 변화가 원인인가?
6. **JUDGE:** 성공·실패를 어떤 기준으로 판단하는가?
7. **SO WHAT:** 다음 stage 또는 research claim에 무엇을 의미하는가?

---

## 7. 예상 contribution 구조

단순히 “QP에 soft soil을 넣었다” 또는 “contact Jacobian을 사용했다”는 contribution으로 부족하다.

현재 가장 강한 중심 formulation 후보는 다음이다.

> **Contact-mode-dependent soil-feasible wrench allocation with actively controllable normal loads**

Contribution은 다음 네 층으로 정리할 수 있다.

1. **Formulation:** Variable-contact soil-feasible wrench set
2. **Method:** Active \(F_z\)와 tangential force를 함께 푸는 실시간 hierarchical/convex optimization
3. **Insight:** Equal load 또는 minimum slip보다 soil-feasibility margin이 중요한 조건 규명
4. **Validation:** Rigid→soft, fixed→variable contact의 단계적 causal validation

논문 분리 가능성은 다음과 같다.

- Paper 1: Fixed four-contact soil-feasible active normal/tangential force allocation
- Paper 2: Contact-mode-dependent reallocation and soil-aware recontact recovery

Method를 임의로 추가해 novelty를 만들지 않는다. Stage 5–6에서 관찰된 failure가 nonlinear approximation, robustness, soil estimation 또는 recontact optimization 중 무엇이 필요한지 결정하게 한다.

---

## 8. 단계별 핵심 문헌

## Stage 1 — \(F_z\), posture, force distribution

1. [Cordes et al., Static Force Distribution and Orientation Control for a Rover with an Actively Articulated Suspension System](https://doi.org/10.1109/IROS.2017.8206412)
   - Rover-specific problem definition
   - Wheel-force distribution, permanent contact, body orientation
   - Reactive control이며 QP formulation은 제한적

2. [Hutter et al., Force Control for Active Chassis Balancing](https://doi.org/10.1109/TMECH.2016.2612722)
   - Quasi-static contact-force optimization
   - Contact-force 및 joint-torque constraints
   - Stage 1 QP structure의 핵심 참고

3. [Li et al., Towards Uniform Normal Force Distribution by Roll and Height Control](https://doi.org/10.1109/M2VIP49856.2021.9665133)
   - Normal Force Dispersion metric
   - Roll/height와 normal-load distribution 관계

## Stage 2 — \(F_x-F_z\) integration

1. [Knobel et al., Optimized Force Allocation—A General Approach](https://elib.dlr.de/45187/1/087.pdf)
   - Desired body force/moment를 wheel forces로 allocation
   - Steering, drive/brake torque, wheel load, camber 고려
   - Nonlinear tire utilization optimization

2. [An Integrated Control Framework for Torque Vectoring and Active Suspension System](https://link.springer.com/article/10.1186/s10033-024-00999-6)
   - Longitudinal–vertical coupled dynamics
   - Wheel dynamics, slip, active suspension, LTV-MPC

3. [Li et al., Simultaneous Control of Terrain Adaptation and Wheel Speed Allocation](https://doi.org/10.1109/LRA.2021.3091701)
   - Rover active suspension과 wheel-speed allocation
   - Kinematic approach이므로 force-based QP의 baseline

## Stage 3 — \(F_x-F_y-F_z\) integration

1. [Zhao et al., Coordinated Attitude Control of Longitudinal, Lateral and Vertical Tyre Forces](https://doi.org/10.1109/TVT.2021.3137512)
   - Longitudinal/lateral/vertical force coordination
   - MPC multi-objective and constraints

2. [Park and Gerdes, Optimal Tire Force Allocation for Trajectory Tracking](https://ddl.stanford.edu/sites/g/files/sbiybj25996/files/media/file/2015_iv_park_optimal_tire_force_allocation_0.pdf)
   - Convex \(F_x,F_y\) allocation
   - Equal friction utilization
   - \(F_z\)는 active decision variable이 아니라는 한계

## Stage 4 — Variable contact

1. [Fahmi et al., Passive Whole-Body Control for Quadruped Robots](https://iit-dlslab.github.io/papers/fahmi19ral.pdf)
   - QP decision: generalized acceleration and contact force
   - Stance/swing Jacobian, dynamics, unilateral/friction/torque constraints

2. [Toupet et al., Terrain-Adaptive Wheel Speed Control on the Curiosity Mars Rover](https://www-robotics.jpl.nasa.gov/media/documents/jfr-trctl-rob-21903-2019.pdf)
   - 실제 wheelie detection과 recovery
   - Bogie angle/rate와 low motor current로 검출
   - 같은 bogie의 다른 wheel speed를 조절해 release

3. [Zhou et al., Wheel Ground Clearance Control for Lunar Rover’s Active Suspension](https://doi.org/10.1109/TIE.2026.3657004)
   - Active suspension 기반 wheel lift-off prevention
   - \(H_\infty\) controller이며 active-contact force QP는 아님

4. [Neunert et al., Whole-Body NMPC Through Contacts for Quadrupeds](https://arxiv.org/abs/1712.02889)
   - Contact sequence/location/timing optimization
   - 후속 contact-implicit extension 참고

## Stage 5 — Soft-soil plant와 rigid-controller failure

1. [Fahmi et al., STANCE: Locomotion Adaptation over Soft Terrain](https://iit-dlslab.github.io/papers/fahmi19tro.pdf)
   - Rigid-ground WBC가 soft-contact dynamics를 무시할 때의 failure logic
   - Compliant-contact-aware WBC와 terrain estimator
   - Foot compliance이며 wheel terramechanics는 아님

2. [Schäfer et al., Planetary Rover Mobility Simulation on Soft and Uneven Terrain](https://doi.org/10.1080/00423110903243224)
   - ExoMars multibody dynamics + soft/uneven wheel–soil model

3. [Azimi et al., A Multibody Dynamics Framework for Simulation of Rovers on Soft Terrain](https://doi.org/10.1115/1.4029406)
   - Unilateral contact와 terramechanics force law의 LCP formulation

4. [Yang et al., High-Fidelity Dynamic Modeling and Simulation of Planetary Rovers](https://doi.org/10.1109/TRO.2022.3160018)
   - Terrain property mapping, slip, skid, steering, sinkage

## Stage 6 — Soft-soil-aware fixed contact

1. [Ghotbi et al., Mobility Evaluation of Wheeled Robots on Soft Terrain: Effect of Internal Force Distribution](https://doi.org/10.1016/j.mechmachtheory.2016.02.005)
   - Normal-load distribution이 traction/mobility에 미치는 영향
   - Active suspension으로 load distribution을 바꿀 물리적 근거

2. [Barthelmes and Konigorski, Model-Based Chassis Control System for an Over-Actuated Planetary Exploration Rover](https://elib.dlr.de/136854/1/barthelmes2020model.pdf)
   - 3D rover dynamics, contact-force Jacobian, pseudoinverse/null space
   - Soft-ground traction utilization
   - Passive suspension이라 \(F_z\)는 제어하지 않음

3. [Chen et al., Simultaneous Control of Trajectory Tracking and Coordinated Allocation of Rocker-Bogie Planetary Rovers](https://doi.org/10.1016/j.ymssp.2020.107312)
   - Soft uneven terrain, trajectory tracking, coordinated allocation
   - \(H_2/H_\infty\)-QP와 terramechanics force tracking
   - Passive suspension

4. [Oda et al., Model Predictive Allocation Control on Loose Soil Considering Wheel Dynamics](https://doi.org/10.3384/ecp18148240)
   - Wheel dynamics를 포함한 longitudinal force allocation
   - Static allocation realization failure를 피하는 참고

5. [Krenn et al., Model Predictive Traction and Steering Control of Planetary Rovers](https://elib.dlr.de/82417/1/Paper_ExoMars_MPC_Krenn_V22_HF.pdf)
   - Bekker, Mohr–Coulomb, Janosi–Hanamoto 기반 traction/steering MPC
   - Wheel load는 계산되지만 active allocation하지 않음

6. [Inotsume et al., Modeling, Analysis, and Control of an Actively Reconfigurable Planetary Rover](https://doi.org/10.1002/rob.21479)
   - Active posture change, wheel load와 sandy-slope slip coupling

## Stage 7 — 최종 경쟁 연구

1. [Bouton et al., ERNEST: Learning All-Terrain Locomotion for a Planetary Rover with Actively Articulated Suspension](https://arxiv.org/html/2606.06790v1)
   - Active suspension, load redistribution, drive integration
   - Bekker–Wong soft-soil simulation과 실제 loose-soil experiment
   - RL policy이며 explicit force QP/variable \(J_{\mathcal C}\)는 없음

2. STANCE, Neunert, Curiosity TRCTL, Zhou 2026을 각각 soft contact, contact optimization, wheelie recovery, contact-loss prevention의 비교 연구로 사용한다.

현재 확인한 대표 문헌에서는 다음을 하나의 model-based controller에서 모두 명시적으로 수행하는 연구가 확인되지 않았다.

\[
\text{Active }F_z
+\text{soft-soil }F_x,F_y\text{ feasibility}
+\text{variable }J_{\mathcal C}
+\text{recontact recovery}
\]

이 문장은 최종 novelty claim 전에 추가 systematic search로 다시 검증해야 한다.

---

## 9. 논문을 읽을 때 추출할 항목

각 논문을 일반적인 summary로 정리하지 말고 아래 표를 반드시 채운다.

| 항목 | 확인 질문 |
|---|---|
| Main problem | 정확히 어떤 failure를 해결하는가? |
| Decision variable | \(F_x,F_y,F_z,\ddot q,\tau,\omega,\delta\) 중 무엇을 푸는가? |
| Equality | 어떤 body dynamics 또는 force/moment balance를 강제하는가? |
| Inequality | Contact, friction/soil, actuator, rate 제한은 무엇인가? |
| Objective | Tracking, equal load, utilization, slip, sinkage 중 무엇을 최소화하는가? |
| Contact assumption | Fixed contact인가, variable contact인가? |
| Ground assumption | Rigid, compliant 또는 terramechanics인가? |
| Lower-layer realization | Force를 torque/wheel speed/suspension command로 어떻게 변환하는가? |
| Evidence | 어떤 baseline과 metric으로 해결을 증명하는가? |
| Transfer | 현재 rover에 직접 가져올 부분과 버릴 가정은 무엇인가? |

---

## 10. Linux/MuJoCo에서 즉시 수행할 작업

다음 Codex는 구현 전에 아래 순서로 repository를 점검한다.

1. `AGENTS.md`와 project README 확인
2. `rg --files`로 repository 구조 확인
3. MJCF/XML model, meshes, actuator, sensor 정의 위치 확인
4. Simulation stepping loop와 control period 확인
5. 기존 body orientation/path/wheel-speed/suspension controller 확인
6. Rover generalized coordinates, body/world/wheel/contact frame convention 확인
7. Mass, inertia, CoM, wheel contact positions 확인
8. Suspension actuator type가 position/velocity/force/torque 중 무엇인지 확인
9. Contact-force sensing 또는 MuJoCo contact-force extraction 경로 확인
10. 기존 build/test/run command를 실행하여 baseline 재현

### 첫 구현 deliverable

Stage 1만 구현한다.

- Contact geometry로부터 \(A_z(q)\) 계산
- 원하는 vertical force/roll moment/pitch moment 생성
- \(F_z\) QP solver module 작성
- QP output을 suspension actuator command로 변환
- Flat ground static equilibrium test
- Asymmetric rigid uneven terrain test
- Baseline과 NFD/body-orientation 비교
- Solver status와 모든 constraint residual 기록

### 구현 전 repository에서 확인하지 못하면 물어볼 핵심 정보

- Suspension actuator가 실제로 제어 가능한 입력
- Suspension force/torque 및 stroke limits
- Wheel-force measurement/estimation 방식
- Rover mass, CoM와 inertia 신뢰도
- Control frequency와 허용 solver time
- 현재 사용하는 QP solver 또는 새 dependency 허용 여부
- 목표 body height, roll, pitch 생성 방식

질문은 가능한 한 한 번에 가장 중요한 1–3개로 제한한다.

---

## 11. 교수님께 설명할 proposal spine

> 먼저 rigid uneven terrain에서 active-suspension force-allocation controller의 기본 성능을 검증한다. 이후 controller를 수정하지 않고 동일한 macro geometry의 soft-soil plant에 적용하여, rigid-ground force assumption이 어떤 잘못된 allocation과 mobility failure를 유발하는지 확인한다. 이 failure의 force-level 원인을 바탕으로 soil-dependent force feasibility와 active normal-load allocation을 추가하고, 마지막으로 contact loss 시 variable contact Jacobian을 이용한 force reallocation과 recontact recovery로 확장한다.

한 문장으로 압축하면 다음과 같다.

> **Rigid-ground allocation이 deformable soil에서 실현 불가능한 wheel force를 명령하는 문제를 규명하고, 이를 해결하기 위해 active normal-load control을 포함한 contact-mode-dependent soil-feasible force allocation을 개발한다.**

---

## 12. 현재 가장 먼저 답해야 할 연구 질문

Stage 1–2의 rigid-ground controller가 준비된 뒤 다음 질문에 답한다.

> **동일한 desired body wrench에 대해 rigid-ground QP가 선택한 wheel-force distribution은 soft-soil plant에서 실제로 infeasible한가? 그렇다면 soil-aware feasibility를 사용했을 때 optimizer의 선택과 downstream mobility가 어떻게 바뀌는가?**

이 질문에 대한 명확한 preliminary evidence가 박사 proposal의 출발점이다.
