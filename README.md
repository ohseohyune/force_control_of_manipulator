# force_control
Study notes &amp; simulations for Force Control of Manipulators (Chap.11)

# Chapter 11. Force Control of Manipulators

> **Course:** Robotic Application — Spring Semester 2026  
> **Reference:** Siciliano et al., *Robotics: Modelling, Planning and Control*
> **Lab**: BICAR (Biologically-inspired Control and Robot) Lab., KwangWoon University

---

## 📋 Table of Contents

1. [Why Position/Force Control?](#1-why-positionforce-control)
2. [Direct / Indirect Force Control](#2-direct--indirect-force-control)
3. [Requirements for Force Control](#3-requirements-for-force-control)
4. [Torque-Controlled Manipulator](#4-torque-controlled-manipulator)
   - 4.1 [System Model & Linearization](#41-system-model--linearization)
   - 4.2 [Worked Example: 1-DOF System](#42-worked-example-1-dof-system)
5. [Hardware Considerations](#5-hardware-considerations)
6. [Position-Controlled Manipulator](#6-position-controlled-manipulator)
7. [Direct Force Control](#7-direct-force-control)
8. [Force Control via Position-Controlled Manipulator](#8-force-control-via-position-controlled-manipulator)
9. [Single DOF System Analysis (Stiffness Control)](#9-single-dof-system-analysis-stiffness-control)
10. [N-Link Manipulator: Stiffness Control](#10-n-link-manipulator-stiffness-control)

---

## 1. Why Position/Force Control?

순수 위치 제어(position-only control)는 **자유 공간(free space)** 에서는 충분하지만,
로봇이 환경과 **물리적으로 접촉**하는 작업(contact task)에서는 한계를 드러낸다.

### 1.1 핵심 문제

```
자유 공간 → 위치 제어만으로 충분
접촉 작업 → 위치 + 힘 동시 제어 필요
```

로봇이 환경(environment)과 접촉하는 순간, **상호작용 힘(interaction force)** 이 발생한다.
이 힘을 제어하지 않으면 두 가지 문제가 생긴다:

| 상황 | 결과 |
|------|------|
| 힘이 너무 크면 | 부품 파손, 표면 손상 |
| 힘이 너무 작으면 | 작업 실패 (연마, 조립 불완전) |

### 1.2 대표적인 접촉 작업

| 작업 | 제어 요구사항 |
|------|-------------|
| **Deburring (디버링)** | 공작물 윤곽을 따라 이동 + 수직 방향 힘 일정 유지 |
| **Peg-in-hole (조립)** | 측면 접촉력 제어 + 각도 오차 보상 |
| **Surface polishing** | 법선 방향 힘 + 접선 방향 궤적 추적 |
| **Surgical robotics** | 미세한 힘 제어 (생체 조직 손상 방지) |

### 1.3 결론

> **위치 궤적(desired position trajectory)** 과 **힘 궤적(desired force trajectory)** 을
> **동시에** 지정하고 추종해야 한다.

---

## 2. Direct / Indirect Force Control

힘 제어 방식은 크게 두 가지로 분류된다.

```
Force Control
├── Direct Force Control    → 힘을 직접 피드백 루프로 제어
│   └── 대표: Hybrid Force-Position Control
└── Indirect Force Control  → 힘을 "간접적으로" 생성 (기계적 임피던스 조절)
    ├── Stiffness Control
    ├── Damping Control
    └── Impedance Control
```

### 2.1 Direct Force Control (직접 힘 제어)

- **원리:** 힘/토크 센서로 접촉력을 측정 → 힘 오차를 직접 피드백
- **구조:** 힘 제어기(Force Controller)가 직접 관절 토크를 명령
- **대표 방식:** Hybrid force-position control
  → 작업 공간을 **위치 제어 서브공간**과 **힘 제어 서브공간**으로 분리하여 각각 독립 제어

$$\tau = J^T(q) G_F (f_d - f_e) + n(q, \dot{q}) + J^T(q) f_e$$

### 2.2 Indirect Force Control (간접 힘 제어)

- **원리:** 힘을 직접 제어하지 않고, **로봇의 동적 거동(stiffness/damping/inertia)** 을 조절하여 간접적으로 힘을 생성
- **핵심 아이디어:** 로봇을 스프링-댐퍼-질량계처럼 동작하도록 제어
- **목표 임피던스 모델:**

$$M_d \ddot{x}_e + B_d \dot{x}_e + K_d x_e = f_e$$

| 파라미터 | 의미 |
|---------|------|
| $M_d$ | 목표 관성 (desired inertia) |
| $B_d$ | 목표 댐핑 (desired damping) |
| $K_d$ | 목표 강성 (desired stiffness) |

### 2.3 비교

| 항목 | Direct | Indirect |
|------|--------|---------|
| 힘 센서 | 필수 | 선택적 |
| 동역학 모델 | 완전한 모델 필요 | 부분 모델 |
| 힘 추적 정확도 | 높음 | 낮음 (간접적) |
| 구현 난이도 | 높음 | 낮음 |
| 강인성 | 낮음 | 높음 |
| 환경 강성 민감도 | 높음 | 낮음 |

---

## 3. Requirements for Force Control

힘 제어를 구현하기 위해서는 다음 조건들이 필요하다.

### 3.1 필수 조건

**① 정확한 동역학 파라미터**

- 각 링크의 관성(inertia) 파라미터 $M(q)$
- 코리올리/원심력, 중력, 마찰력 등 비선형 항 $n(q, \dot{q})$
- 파라미터 오차 → 계산 토크(computed torque)의 오차 → 힘 오차 발생

**② 완전한 관절 토크 제어**

- 각 관절의 토크를 독립적으로 정밀 제어할 수 있어야 한다
- 상용 로봇은 보통 전류 제어(current control) 기반 → 토크 제어가 제한됨

**③ 센서 요구사항**

측정 필요: $\{q,\ \dot{q}\}$ (관절 위치/속도) + $\{f_e,\ \tau_e\}$ (접촉력/토크)

- 관절 위치 $q$ : 엔코더
- 관절 속도 $\dot{q}$ : 엔코더 미분 또는 타코미터
- 접촉력 $f_e$ : 6축 F/T 센서 (엔드이펙터 근처 장착)

**④ 실시간 제어 대역폭**

권장: bandwidth $> 1\ \text{kHz}$

- 환경은 매우 높은 강성(stiffness)을 가지는 경우가 많음
- 접촉 시 힘이 빠르게 변화 → 낮은 제어 주기 시 진동/불안정 발생

---

## 4. Torque-Controlled Manipulator (Direct Force Control)
<img width="657" height="246" alt="image" src="https://github.com/user-attachments/assets/af788ba8-daa9-43ea-8eb5-bdf05cd23493" />

### 4.1 System Model & Linearization

#### 시스템 동역학

$$M(q)\ddot{q} + n(q, \dot{q}) + J^T(q) f_e = \tau$$

| 변수 | 설명 | 차원 |
|------|------|------|
| $M(q)$ | 관성 행렬 (symmetric positive definite) | $n \times n$ |
| $n(q,\dot{q})$ | 비선형 항: 코리올리 + 중력 + 마찰 | $n \times 1$ |
| $J^T(q)$ | 자코비안 전치 | $n \times m$ |
| $f_e$ | 접촉력 벡터 | $m \times 1$ |
| $\tau$ | 관절 토크 입력 | $n \times 1$ |

비선형 항의 정의:

$$n(q,\dot{q}) \overset{\triangle}{=} b(q,\dot{q}) + g(q) + d$$

- $b(q,\dot{q})$ : 코리올리 및 원심력
- $g(q)$ : 중력
- $d$ : 마찰, 외란

#### 환경 접촉 모델

환경을 선형 스프링으로 모델링한다:

$$f_e = K_e(x - x_e)$$

- $K_e$ : 환경 강성(environment stiffness) $[\text{N/m}]$
- $x_e$ : 환경 표면 위치
- $x - x_e > 0$ 일 때만 힘 발생 (단방향 접촉)

#### 액추에이터 모델

$$\tau_a = K_t \cdot i_a$$

- $K_t$ : 토크 상수 $[\text{Nm/A}]$
- $i_a$ : 전류 $[\text{A}]$

#### Gravity/Dynamics Compensation (중력 보상)

제어 입력을 다음과 같이 분리하면 시스템이 선형화된다:

$$\tau = \tau_t + n(q,\dot{q}) + J^T(q) f_e$$

원래 동역학 방정식에 대입하면:

$$\boxed{M(q)\ddot{q} = \tau_t}$$

> **핵심:** $\tau_t$ 는 **선형 이중 적분기(double integrator)** 에 대한 입력으로 작용한다.
> 이제 $\tau_t$ 만 잘 설계하면 되는 선형 제어 문제가 된다.

블록 다이어그램:

```
τ_t ──→ [ Torque-controlled Manipulator ] ──→ x
                                          └──→ f_e
```

---

### 4.2 Worked Example: 1-DOF System

#### 문제 설정

시스템: $M\ddot{x} = \tau_t$

파라미터: $M=2\ \text{kg}$, $K_e=1000\ \text{N/m}$, $x_e=0.1\ \text{m}$, $x=0.08\ \text{m}$, $\dot{x}=0$

> P 게인 $K_f = 5$ 인 비례 힘 제어기 사용.

---

**(a) $x = 0.08\ \text{m}$ 에서 접촉력 $f_e$ ?**

$x = 0.08\ \text{m} < x_e = 0.1\ \text{m}$ 이므로 접촉 없음.

$$\therefore\quad f_e = 0\ \text{N}$$

---

**(b) $f_d = 20\ \text{N}$ 을 생성하기 위한 목표 위치 $x_d$ ?**

$$f_d = K_e(x_d - x_e) \implies x_d = x_e + \frac{f_d}{K_e} = 0.10 + \frac{20}{1000}$$

$$\therefore\quad x_d = 0.12\ \text{m}$$

---

**(c) 현재 상태에서 제어 입력 토크 $\tau_t$ ?**

$$\tau_t = K_f(f_d - f_e) = 5 \times (20 - 0)$$

$$\therefore\quad \tau_t = 100\ \text{N}$$

---

**(d) 초기 가속도 $\ddot{x}$ ?**

$$M\ddot{x} = \tau_t \implies 2\ddot{x} = 100$$

$$\therefore\quad \ddot{x} = 50\ \text{m/s}^2$$

---

**(e) 왜 진동이 발생할 수 있는가?**

환경이 스프링처럼 동작하므로:

$$f_e = K_e(x - x_e)$$

매우 작은 위치 변화도 큰 힘 변화를 일으킨다 ($K_e = 1000\ \text{N/m}$).
높은 힘 게인 $K_f$ 는 작은 힘 오차에도 큰 토크를 출력한다.

> **결론:** 딱딱한 환경(stiff environment)에서 높은 $K_f$ 는
> 위치-힘 간 양성 피드백 구조를 만들어 **진동(oscillatory behavior)** 을 유발한다.

```
τ_t 증가 → 가속도 증가 → x 증가 → f_e = K_e(x-x_e) 증가 → 더 큰 τ_t 요구 → ...
```

이는 폐루프 시스템의 극점(pole)이 허수축에 가까워지거나 우반평면으로 이동함을 의미한다.

---

## 5. Hardware Considerations

토크 제어 매니퓰레이터 구현 시 하드웨어 측면에서 고려해야 할 사항이다.

### 5.1 감속기(Reducer) 사용 여부

일반 서보 모터는 **고속/저토크** 특성을 가지므로, 매니퓰레이터에는 보통 **감속기**가 사용된다.

| 구성 | 장점 | 단점 |
|------|------|------|
| **고 감속비 감속기** | 토크 증가, 비선형성 감소 (토크 리플 < 10%) | 역구동성(backdrivability) 감소 → 마찰 보상 필요 |
| **저 감속비 감속기** | 역구동성 증가 | 토크 감소, 비선형성 증가 |
| **직구동(Direct Drive)** | 역구동성 최대, 투명도(transparency) 최대 | 모터 크기 증가, 비선형성 증가 |

### 5.2 역구동성(Backdrivability) 문제

> 역구동성 = 외력에 의해 관절이 얼마나 자유롭게 움직일 수 있는지를 나타내는 척도

감속비 $N$ 이 클수록:

$$\tau_{\text{output}} = N \cdot \tau_{\text{motor}}, \qquad \text{friction} \propto N^2$$

마찰이 크게 증가 → 힘 투명도(force transparency) 저하 → 정밀한 힘 제어 어려움
→ 마찰 보상(friction compensation)이 필수적으로 필요하다.

### 5.3 실제 적용 사례

| 종류 | 대표 제품/연구 |
|------|--------------|
| 고 감속비 | 대부분의 산업용 로봇 (KUKA, FANUC) |
| 저 감속비 | 협동 로봇 (Universal Robots, KUKA iiwa) |
| 직구동 | MIT Cheetah, Cassie (Agility Robotics) |

---

## 6. Position-Controlled Manipulator
<img width="653" height="216" alt="image" src="https://github.com/user-attachments/assets/31d24dcc-525d-4a2b-b679-a4ddae2d7387" />

### 6.1 개요

상용 매니퓰레이터는 대부분 **전류 제어(current control)** 기반의 정밀 위치 제어를 사용한다.

- 직접적인 토크 제어 명령이 불가능하거나 제한적
- 목표 위치 $x_t$ 와 속도를 입력으로 받는 내부 PID 위치 제어기가 동작

### 6.2 시스템 모델

동역학 방정식은 동일하다:

$$M(q)\ddot{q} + n(q,\dot{q}) + J^T(q)f_e = \tau$$

그러나 실제 제어 입력은 **목표 위치 $x_t$** (또는 $q_t$) 이다.

```
x_t ──→ J⁻¹ ──→ [Position Controller Gₚ] ──→ τ ──→ [Manipulator] ──→ x
                                                                   └──→ f_e
```

위치 제어기는 일반적으로 PID 형태:

$$\tau_t = G_P \Delta q = G_P J^{-1}(q)(x_t - x)$$

### 6.3 단순화된 표현

위치 제어기의 성능이 충분히 빠르고 정밀하다고 가정할 때,
힘 제어의 성능은 **위치 제어기의 성능에 종속**된다.

```
x_t ──→ [ Position-controlled Manipulator ] ──→ x
                                            └──→ f_e
```

---

## 7. Direct Force Control
<img width="597" height="257" alt="image" src="https://github.com/user-attachments/assets/4c6af20e-f316-444f-837c-7908ff4b661b" />
<img width="356" height="240" alt="image" src="https://github.com/user-attachments/assets/bb2398e7-043b-4b7a-828a-403588abb3ec" />

### 7.1 제어 법칙

토크 제어 매니퓰레이터에 대한 직접 힘 제어기를 단계별로 유도한다.

**Step 1.** 힘 제어기 출력 (선형화 공간):

$$\tau_t = J^T(q)\, G_F(f_d - f_e)$$

**Step 2.** 실제 인가 토크 (중력/동역학 보상 포함):

$$\tau = \tau_t + n(q,\dot{q}) + J^T(q) f_e = J^T(q) G_F(f_d - f_e) + n(q,\dot{q}) + J^T(q) f_e$$

**Step 3.** 댐핑 항 추가 (임의적인 가속 운동 제한):

$$\boxed{\tau = J^T(q) G_F(f_d - f_e) + n(q,\dot{q}) + J^T(q) f_e - J^T(q) K_v \dot{x}}$$

> $-J^T(q) K_v \dot{x}$ : 카르테시안 공간에서의 **가상 댐핑(virtual damping)** 항
> → 빠른 충격 운동 제한 + 안정성 향상

### 7.2 블록 다이어그램

```
f_d ──(+)──→ [Force Controller Gf] ──→ J^T ──→ (+) ──→ τ ──→ [Manipulator] ──→ x, f_e
      (-)↑                                       ↑
      f_e                         n(q,q̇) + J^T·f_e − J^T·Kv·ẋ
```

### 7.3 힘 제어기 $G_F$ 설계

#### PI 제어기 (권장)

$$G_F(s) = K_P + \frac{K_i}{s}$$

$$U_f = K_P e_f + K_i \int e_f\, dt, \qquad e_f = f_d - f_e$$

> **왜 PID가 아닌 PI인가?**
> 힘 센서 신호는 고주파 노이즈를 많이 포함한다.
> 미분(D) 항은 이 노이즈를 증폭시켜 시스템을 불안정하게 만든다.
> → **PI 제어기가 표준적**으로 사용된다.

#### Lead/Lag 보상기 (딱딱한 환경에서)

환경 강성 $K_e$ 가 매우 크면 PI만으로 불안정해질 수 있다.
이 경우 위상 마진(phase margin)을 개선하기 위해 Lead/Lag 보상기를 사용한다:

$$G_F(s) = K \frac{s + z}{s + p}$$

- **Lead** ($z < p$) : 위상 추가 → 위상 마진 개선, 진동 감소
- **Lag** ($z > p$) : 이득 증가 (저주파), 정상상태 오차 감소

---

## 8. Force Control via Position-Controlled Manipulator
<img width="756" height="311" alt="image" src="https://github.com/user-attachments/assets/8ed1c817-ed4e-436c-93f4-ef8615f8e379" />

토크 제어 없이 **위치 명령만으로** 힘 제어를 수행하는 방법.

> **전제:** 위치 제어기가 충분히 빠르고 정밀하게 동작한다고 가정.

### 8.1 기본 원리: 위치-힘 관계

환경이 선형 스프링이므로:

$$f_e = K_e(x - x_e)$$

힘 $f_d$ 를 생성하기 위해 필요한 목표 위치:

$$\boxed{x_d = x_e + \frac{f_d}{K_e}}$$

> **해석:** 원하는 힘을 내려면, 환경 표면보다 $f_d / K_e$ 만큼 더 깊이 들어가도록
> 위치를 명령하면 된다. 위치 제어기가 이를 추종하면, 환경 반력으로 힘이 생성된다.

### 8.2 힘 제어기 통합

단순히 목표 위치를 명령하는 것만으로는 정확한 힘 제어가 어렵다.
($K_e$ 가 정확히 알려지지 않은 경우 등)

따라서 **힘 오차를 피드백**하여 위치 보정량을 생성한다:

$$x_t = x_e + x_f, \qquad x_f = G_f(f_d - f_e)$$

- $x_e$ : 환경 표면 위치 (피드포워드)
- $x_f$ : 힘 제어기가 생성하는 위치 보정량
- $G_f$ : 힘-to-위치 변환 제어기

블록 다이어그램:

```
f_d ──(+)──→ [Force Controller Gf] ──→ x_f
      (-)↑                               ↓
      f_e              x_e ──→ (+) ──→ x_t ──→ [Position-controlled Manipulator] ──→ x, f_e
```

### 8.3 내부 동역학 분석

산업용 로봇(토크 제어 없음)에서 토크 입력을 전개하면:

$$\tau_t = G_P J^{-1}(q)(x_t - x) = G_P J^{-1}(q)\{x_e + G_F(f_d - f_e) - x\}$$

$$= \underbrace{G_P J^{-1}(q)(x_e - x)}_{\text{위치 제어 항}} + \underbrace{G_P J^{-1}(q) G_F(f_d - f_e)}_{\text{힘 제어 항}}$$

### 8.4 고 이득 시스템에서의 안정성 문제

정밀한 위치 제어를 위해 $G_P$ 를 크게 설정하면:

$$\text{유효 힘 제어 이득} = G_P \cdot G_F$$

$G_P$ 가 크면 힘 제어 루프의 이득도 함께 증가한다.
→ 시스템이 **작은 외란에도 민감**해지며, **불안정(instability)** 으로 이어질 수 있다.

> **결론:**
> - $G_P$ 증가 → 위치 정밀도 향상, 힘 루프 이득 증가 → 잠재적 불안정
> - 힘 제어기 $G_F$ 는 이 이중 구조를 고려하여 **보수적으로** 설계해야 한다
> - 고주파 노이즈 문제로 $G_F$ 는 **PI 제어기**가 표준

---

## 9. Single DOF System Analysis (Stiffness Control)

[Salisbury and Craig, 1980]

### 9.1 시스템 설명

질량 $m$ 이 환경 벽($x_e$ 에 위치, 강성 $K_e$)과 접촉하는 1-DOF 시스템.

```
τ ──→ [m] ──→ x  
              └──→ spring(K_e) ──→ wall(x_e)
```

목표: 입력 힘 $\tau$ 를 결정하여 로봇을 원하는 위치 $x_d$ 로 이동시키고,
접촉력 $f_e$ 를 제어한다.

#### 접촉력 모델

$$f = K_e(x - x_e), \qquad x > x_e$$

#### 운동 방정식 (중력, 마찰 무시)

$$\tau = m\ddot{x} + K_e(x - x_e)$$

### 9.2 PD 제어기 설계

$$\tau = -K_v \dot{x} - K_p(x-x_d)$$

- $K_p$ : 비례 게인 (위치 강성 역할)
- $K_v$ : 속도 게인 (댐핑 역할)

두 식을 결합하면 **폐루프 동역학**:

$$m\ddot{x} + K_v \dot{x} + (K_p + K_e)x = K_p x_d + K_e x_e$$

### 9.3 라플라스 해석

$$x(s) = \frac{K_p x_d(s) + K_e x_e(s)}{ms^2 + K_v s + (K_p + K_e)}$$

#### 폐루프 전달함수

$$H(s) = \frac{1}{ms^2 + K_v s + (K_p + K_e)}$$

**안정성 분석:**
모든 계수 $m,\, K_v,\, K_p,\, K_e > 0$ 이므로, $H(s)$ 의 극점은 모두 **좌반 $s$-평면**에 존재한다.

$$\therefore \quad \text{폐루프 시스템은 항상 안정}$$

#### 고유 진동수와 감쇠비

$$\omega_n = \sqrt{\frac{K_p + K_e}{m}}, \qquad \zeta = \frac{K_v}{2\sqrt{m(K_p + K_e)}}$$

> $K_e$ 가 크면 $\omega_n$ 이 증가 → 시스템이 빠르게 진동
> → 진동 억제를 위해 $K_v$ 증가 필요

### 9.4 정상 상태 분석 (Final Value Theorem)

$x_d$, $x_e$ 가 상수인 경우, $s \to 0$ 극한을 취하면:

#### 정상 상태 위치

$$x_{ss} = \lim_{s \to 0} s \cdot x(s) = \frac{K_p x_d + K_e x_e}{K_p + K_e}$$

#### 정상 상태 힘

$$f_{ss} = K_e(x_{ss} - x_e) = \frac{K_p K_e (x_d - x_e)}{K_p + K_e}$$

#### $K_e \gg K_p$ 일 때의 근사

딱딱한 환경 ($K_e \gg K_p$) 에서:

$$\boxed{f_{ss} \cong K_p(x_d - x_e)}$$

> **핵심 해석:**
> 정상 상태 힘은 환경 강성 $K_e$ 에 관계없이 **위치 게인 $K_p$** 에 의해 결정된다.
> → $K_p$ 를 원하는 **가상 강성(virtual stiffness)** 으로 해석할 수 있다.

### 9.5 물리적 해석 (Stiffness Control의 핵심)

```
실제 로봇 관점에서:
  ┌────────────────────────────────────────────────┐
  │  로봇 = 스프링(강성 Kₚ) + 위치 명령(xd)          │
  │  → 환경에 F = Kₚ(xd - xe) 의 힘을 가한다         │
  └────────────────────────────────────────────────┘
```

**힘 생성 메커니즘:**
1. 목표 위치 $x_d$ 를 환경 표면 $x_e$ **안쪽**으로 설정 (penetration)
2. 위치 제어기는 $x_d$ 로 이동하려 하지만, 환경 벽에 막혀 정지
3. 위치 오차 → 비례 게인 $K_p$ → **정상 상태 힘** $K_p(x_d - x_e)$ 생성

$$\text{로봇이 "스프링"처럼 동작:} \quad f_{ss} = K_p \cdot \delta, \quad \delta = x_d - x_e$$

> **중요:** 이는 힘을 **직접** 제어하는 것이 아니다.
> 힘이 **위치 오차로부터 간접적으로** 생성되는 것이다. → Indirect Force Control

---

## 10. N-Link Manipulator: Stiffness Control
<img width="796" height="220" alt="image" src="https://github.com/user-attachments/assets/13e2b7a9-0c13-4129-a23b-a77e38a17c33" />

### 10.1 시스템 동역학

접촉력이 존재할 때 $n$-DOF 매니퓰레이터의 동역학:

$$\tau = M(q)\ddot{q} + n(q,\dot{q}) + J^T(q) f_e$$

#### 환경 모델

$$f_e = K_e(x - x_e), \qquad K_e \in \mathbb{R}^{n \times n},\quad x_e \in \mathbb{R}^{n \times 1}$$

### 10.2 다차원 강성 제어기 (Multidimensional Stiffness Controller)

카르테시안 공간에서 PD 제어:

$$\tau = J^T(\theta)\left(-K_V \dot{x} + K_P \tilde{x}\right) + G(\theta) + F(\dot{\theta})$$

| 항목 | 설명 |
|------|------|
| $\tilde{x} = x_d - x$ | 카르테시안 위치 오차 |
| $K_P \in \mathbb{R}^{n \times n}$ | 위치 게인 행렬 (양의 정부호) |
| $K_V \in \mathbb{R}^{n \times n}$ | 속도 게인 행렬 (양의 정부호) |
| $G(\theta)$ | 중력 보상 |
| $F(\dot{\theta})$ | 마찰 보상 |

### 10.3 폐루프 동역학

동역학 방정식에 제어기를 대입하고 정리하면:

$$\boxed{M(q)\ddot{q} = J^T(q)\left(-K_V \dot{x} + K_P \tilde{x} - K_e(x - x_e)\right)}$$

### 10.4 Lyapunov 안정성 분석

#### 에너지 함수 (Lyapunov candidate)

$$V = \frac{1}{2}\dot{q}^T M(q) \dot{q} + \frac{1}{2}\tilde{x}^T K_P \tilde{x} + \frac{1}{2}(x - x_e)^T K_e (x - x_e)$$

| 항 | 물리적 의미 |
|----|------------|
| $\frac{1}{2}\dot{q}^T M(q) \dot{q}$ | 로봇의 운동 에너지 |
| $\frac{1}{2}\tilde{x}^T K_P \tilde{x}$ | 위치 오차 스프링의 위치 에너지 |
| $\frac{1}{2}(x-x_e)^T K_e (x-x_e)$ | 환경 접촉 스프링의 위치 에너지 |

$V > 0$ 이고, $V = 0$ 은 평형점에서만 성립한다.

#### 시간 미분

$\dot{M}(q) - 2b(q,\dot{q})$ 의 반대칭 성질을 이용하면:

$$\dot{V} = -\dot{q}^T J^T(q) K_V J(q) \dot{q}$$

$$\boxed{\dot{V} \leq 0 \qquad \text{(음의 반정부호, negative semi-definite)}}$$

> **해석:** 시스템의 총 에너지는 감소하거나 유지된다.
> LaSalle의 불변 원리에 의해 시스템은 가장 큰 불변 집합으로 수렴한다.

#### LaSalle의 불변 원리 적용

$\dot{V} = 0$ 인 불변 집합을 찾으면:

$$\dot{V} = 0 \implies \dot{q} = 0 \implies \ddot{q} = 0$$

폐루프 동역학에 $\dot{q} = \ddot{q} = 0$ 을 대입:

$$\lim_{t \to \infty} \left[K_P \tilde{x} - K_e(x - x_e)\right] = 0$$

이를 풀면:

$$\lim_{t \to \infty} x_i = (K_{Pi} + K_{ei})^{-1}(K_{Pi} x_{di} + K_{ei} x_{ei})$$

### 10.5 정상 상태 결과

#### 정상 상태 힘

$$\lim_{t \to \infty} f_i = K_{ei}(K_{Pi} + K_{ei})^{-1} K_{Pi}(x_{di} - x_{ei})$$

$K_{ei} \gg K_{Pi}$ 일 때 근사:

$$\boxed{\lim_{t \to \infty} f_i \cong K_{Pi}(x_{di} - x_{ei})}$$

#### 비구속 방향 ($K_{ei} = 0$)

$$\lim_{t \to \infty} x_i = x_{di} \qquad \text{(정확한 위치 추종)}$$

### 10.6 수렴 결과 요약

| 조건 | 방향 | 결과 |
|------|------|------|
| $K_{ei} = 0$ | 비구속 방향 (free direction) | $x_i \to x_{di}$ (위치 제어) |
| $K_{ei} \gg K_{Pi}$ | 구속 방향 (constrained direction) | $f_i \cong K_{Pi}(x_{di} - x_{ei})$ (힘 제어) |

### 10.7 강성 제어기의 한계

> **중요한 단점:** 강성 제어기는 **정점 제어(set-point control)** 에만 적용 가능하다.
> 즉, $x_d$ 와 $f_d$ 가 **상수**이어야 한다.

많은 실제 로봇 작업에서는:
- 물체 표면을 따라 이동하면서 **위치 궤적을 추종**해야 하고
- 동시에 **시변 힘 궤적도 추종**해야 한다

→ 이를 위해 강성 제어의 **일반화**인 **임피던스 제어(Impedance Control)** 가 필요하다.

---

## 📐 핵심 수식 모음

| 설명 | 수식 |
|------|------|
| 매니퓰레이터 동역학 | $M(q)\ddot{q} + n(q,\dot{q}) + J^T(q)f_e = \tau$ |
| 환경 접촉 모델 | $f_e = K_e(x - x_e)$ |
| 중력/동역학 보상 | $\tau = \tau_t + n(q,\dot{q}) + J^T(q)f_e$ |
| 직접 힘 제어 | $\tau_t = J^T(q) G_F(f_d - f_e)$ |
| 힘 → 위치 변환 | $x_d = x_e + f_d / K_e$ |
| 강성 제어 정상상태 힘 | $f_{ss} \cong K_p(x_d - x_e)$ |
| Lyapunov 안정성 조건 | $\dot{V} = -\dot{q}^T J^T K_V J \dot{q} \leq 0$ |

---

## 📚 References

- Siciliano, B. et al., *Robotics: Modelling, Planning and Control*, Springer, 2009
- Salisbury, J. K. and Craig, J. J., "Articulated Hands: Force Control and Kinematic Issues", *Int. J. Robotics Research*, 1982
- Slotine, J.-J. E. and Li, W., *Applied Nonlinear Control*, Prentice Hall, 1991
