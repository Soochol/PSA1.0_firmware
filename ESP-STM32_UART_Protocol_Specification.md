# ESP32-STM32 UART 통신 프로토콜 사양서 v1.0

## 📋 목차
1. [프로토콜 개요](#1-프로토콜-개요)
2. [메시지 프레임 구조](#2-메시지-프레임-구조)
3. [Operation Mode 시스템](#3-operation-mode-시스템)
4. [Device Mode 시스템](#4-device-mode-시스템)
5. [커맨드 카테고리](#5-커맨드-카테고리)
6. [INIT 커맨드 (0x10-0x22)](#6-init-커맨드-0x10-0x22)
7. [REQUEST 커맨드 (0x30-0x42)](#7-request-커맨드-0x30-0x42)
8. [CONTROL 커맨드 (0x50-0x61)](#8-control-커맨드-0x50-0x61)
9. [STATUS 커맨드 (0x70)](#9-status-커맨드-0x70)
10. [EVENT 커맨드 (0x80-0x82)](#10-event-커맨드-0x80-0x82)
11. [ERROR 커맨드 (0x90)](#11-error-커맨드-0x90)
12. [구현 가이드라인](#12-구현-가이드라인)
13. [코드 예제](#13-코드-예제)
14. [주의사항 및 베스트 프랙티스](#14-주의사항-및-베스트-프랙티스)

---

## 1. 프로토콜 개요

### 🎯 목적
PSA (Postural Stability Assistant) 장치의 ESP32와 STM32 간 안정적인 양방향 통신을 위한 UART 프로토콜

### 📡 통신 설정
- **물리 계층**: UART
- **전송 속도**: 115200 baud
- **프레임 형식**: 8 데이터 비트, 패리티 없음, 1 스탑 비트 (8N1)
- **흐름 제어**: 없음

### 🔄 통신 방향
- **ESP32 → STM32**: INIT, REQUEST, CONTROL 커맨드
- **STM32 → ESP32**: STATUS, EVENT, ERROR 커맨드 및 응답

### 🏗️ 아키텍처 특징
- **비동기 통신**: 명령/응답 패턴과 자발적 메시지 지원
- **체크섬 검증**: XOR 체크섬으로 데이터 무결성 보장
- **타임아웃 관리**: 5초 기본 타임아웃, 최대 3회 재시도
- **상태 추적**: 디바이스 모드 및 운영 모드 실시간 관리

---

## 2. 메시지 프레임 구조

### 📦 프레임 형식
```
[STM] [LEN] [DIR] [CMD] [DATA...] [CHKSUM] [ETX]
 1     1     1     1    0-64       1       1
```

### 🧩 필드 상세

| 필드 | 크기 | 설명 | 값 |
|------|------|------|-------|
| **STM** | 1 byte | 시작 마커 | `0x02` (고정) |
| **LEN** | 1 byte | 길이 | `DIR + CMD + DATA + CHKSUM + ETX` |
| **DIR** | 1 byte | 방향 | `0x20` (요청), `0x02` (응답) |
| **CMD** | 1 byte | 커맨드 코드 | `0x10-0x99` |
| **DATA** | 0-64 bytes | 페이로드 데이터 | 커맨드별 가변 |
| **CHKSUM** | 1 byte | XOR 체크섬 | `DIR ⊕ CMD ⊕ DATA[0] ⊕ ... ⊕ DATA[n-1]` |
| **ETX** | 1 byte | 종료 마커 | `0x03` (고정) |

### 💡 메시지 방향 규칙

#### ESP32 → STM32 (요청)
```
DIR = 0x20 (MSG_REQUEST)
- INIT 커맨드: 매개변수 초기화
- REQUEST 커맨드: 데이터 조회
- CONTROL 커맨드: 실시간 제어
```

#### STM32 → ESP32 (응답 및 자발적 메시지)
```
DIR = 0x02 (MSG_RESPONSE) - INIT/REQUEST/CONTROL에 대한 응답
DIR = 0x20 (MSG_REQUEST) - STATUS/EVENT/ERROR 자발적 메시지
```

### 🔍 메시지 예제
**온도 설정 (0x10) 25.5°C**
```
02 07 20 10 19 37 0E 03
│  │  │  │  │  │  │  │
│  │  │  │  │  │  │  └─ ETX (0x03)
│  │  │  │  │  │  └──── 체크섬 (0x20⊕0x10⊕0x19⊕0x37 = 0x0E)
│  │  │  │  │  └─────── 소수점 (0x37 = 55 → 0.55°C)
│  │  │  │  └────────── 정수부 (0x19 = 25°C)
│  │  │  └───────────── 커맨드 (0x10 = initTempSleep)
│  │  └──────────────── 방향 (0x20 = REQUEST)
│  └─────────────────── 길이 (7 = 1+1+2+1+1+1)
└────────────────────── STM 시작 마커
```

---

## 3. Operation Mode 시스템

### 🎮 모드 정의
PSA 장치의 **제어권 관리** 시스템으로, ESP32와 STM32 중 누가 자세 제어를 주도할지 결정

#### AI_MODE (0)
- **제어 주체**: ESP32
- **특징**: ESP32가 TinyML로 자세를 분석하고 STM32에 제어 명령 전송
- **0x51 커맨드**: ✅ **사용 가능** (ESP → STM 디바이스 모드 변경)

#### IMU_MODE (1)  
- **제어 주체**: STM32
- **특징**: STM32가 IMU 데이터로 직접 자세 제어
- **0x51 커맨드**: ❌ **차단** (STM이 자체 제어)

### 🚀 시스템 시작 규칙
```
⚠️ 중요: ESP32와 STM32는 항상 IMU_MODE로 시작한다.
이는 양쪽 개발팀 간 약속된 사항이다.
```

### 🔄 모드 전환 방법
```c
// AI 모드로 전환 (ESP가 제어권 획득)
initDeviceMode(0);  // 0x21 커맨드로 AI_MODE 설정

// IMU 모드로 전환 (STM이 제어권 획득)  
initDeviceMode(1);  // 0x21 커맨드로 IMU_MODE 설정
```

---

## 4. Device Mode 시스템

### 🏠 장치 동작 상태 정의
STM32 장치의 **실제 동작 상태**를 나타냄

| 모드 | 값 | 설명 | 액추에이터 상태 |
|------|------|------|----------------|
| **SLEEP** | 0 | 절전 모드 | 비활성, 최소 전력 |
| **WAITING** | 1 | 대기 모드 | 준비 상태, 사용자 감지 대기 |
| **FORCE_UP** | 2 | 자세 교정 상승 | 액추에이터 상승 동작 |
| **FORCE_ON** | 3 | 자세 유지 | 액추에이터 위치 유지 |
| **FORCE_DOWN** | 4 | 자세 하강 | 액추에이터 하강 동작 |

### 📋 모드 전환 흐름
```
SLEEP → WAITING → FORCE_UP → FORCE_ON → FORCE_DOWN → WAITING
  ↑___________________________________________________|
```

---

## 5. 커맨드 카테고리

### 📊 커맨드 범위 및 특징

| 카테고리 | 범위 | 방향 | 목적 | 응답 |
|---------|------|------|------|------|
| **INIT** | 0x10-0x22 | ESP→STM | 매개변수 초기화 | ACK |
| **REQUEST** | 0x30-0x42 | ESP→STM | 현재 값 조회 | DATA |
| **CONTROL** | 0x50-0x61 | ESP→STM | 실시간 제어 | ACK |
| **STATUS** | 0x70 | STM→ESP | 센서 데이터 전송 | ACK |
| **EVENT** | 0x80-0x82 | STM→ESP | 시스템 이벤트 알림 | ACK |
| **ERROR** | 0x90 | STM→ESP | 오류 발생 알림 | ACK |

---

## 6. INIT 커맨드 (0x10-0x22)

### 🔧 디바이스 매개변수 초기화 커맨드

#### 온도 설정 커맨드

##### 0x10 - initTempSleep (절전 모드 온도 설정)
| 매개변수 | 타입 | 범위 | 단위 | 설명 |
|---------|------|------|------|------|
| temp_int | uint8_t | 10-40 | °C | 온도 정수부 |
| temp_dec | uint8_t | 0-99 | 0.01°C | 온도 소수부 |

**데이터 크기**: 2 bytes  
**예제**: 25.5°C → `[0x19, 0x37]` (25, 55)

##### 0x11 - initTempWaiting (대기 모드 온도 설정)
| 매개변수 | 타입 | 범위 | 단위 | 설명 |
|---------|------|------|------|------|
| temp_int | uint8_t | 10-45 | °C | 온도 정수부 |
| temp_dec | uint8_t | 0-99 | 0.01°C | 온도 소수부 |

**데이터 크기**: 2 bytes  
**예제**: 30.0°C → `[0x1E, 0x00]` (30, 0)

##### 0x12 - initTempForceUp (동작 모드 온도 설정)
| 매개변수 | 타입 | 범위 | 단위 | 설명 |
|---------|------|------|------|------|
| temp_int | uint8_t | 45-70 | °C | 온도 정수부 |
| temp_dec | uint8_t | 0-99 | 0.01°C | 온도 소수부 |

**데이터 크기**: 2 bytes  
**예제**: 58.0°C → `[0x3A, 0x00]` (58, 0)

##### 0x14 - initTempLimit (상한 온도 설정)
| 매개변수 | 타입 | 범위 | 단위 | 설명 |
|---------|------|------|------|------|
| temp_int | uint8_t | 60-80 | °C | 상한 온도 정수부 |
| temp_dec | uint8_t | 0-99 | 0.01°C | 상한 온도 소수부 |

**데이터 크기**: 2 bytes  
**예제**: 75.0°C → `[0x4B, 0x00]` (75, 0)

#### 팬 및 냉각 시스템 커맨드

##### 0x15 - initPWMCoolFan (냉각팬 PWM 설정)
| 매개변수 | 타입 | 범위 | 단위 | 설명 |
|---------|------|------|------|------|
| current_level | uint8_t | 0-10 | level | 현재 PWM 레벨 |
| max_level | uint8_t | 1-10 | level | 최대 PWM 레벨 |

**데이터 크기**: 2 bytes  
**예제**: 현재 5, 최대 10 → `[0x05, 0x0A]`

##### 0x22 - initPWMFan (메인 팬 PWM 설정)
| 매개변수 | 타입 | 범위 | 단위 | 설명 |
|---------|------|------|------|------|
| fan_speed | uint8_t | 0-3 | level | 팬 속도 (0=정지, 3=최대) |

**데이터 크기**: 1 byte  
**예제**: 중간 속도 → `[0x02]`

#### 타이밍 설정 커맨드

##### 0x16 - initTout (타임아웃 값 설정)
| 매개변수 | 타입 | 범위 | 단위 | 설명 |
|---------|------|------|------|------|
| force_up_timeout | uint16_t | 1-100 | x100ms | FORCE_UP 타임아웃 |
| force_on_timeout | uint16_t | 1-100 | x100ms | FORCE_ON 타임아웃 |
| force_down_timeout | uint16_t | 1-100 | x100ms | FORCE_DOWN 타임아웃 |
| waiting_timeout | uint16_t | 1-100 | x100ms | WAITING 타임아웃 |

**데이터 크기**: 8 bytes (4 × uint16_t)  
**바이트 순서**: Little Endian  
**예제**: 각각 2초 → `[0x14, 0x00, 0x14, 0x00, 0x14, 0x00, 0x14, 0x00]`

##### 0x18 - initDelay (감지 지연 설정)
| 매개변수 | 타입 | 범위 | 단위 | 설명 |
|---------|------|------|------|------|
| force_down_delay | uint8_t | 0-20 | x100ms | FORCE_DOWN 지연 시간 |

**데이터 크기**: 1 byte  
**예제**: 1초 지연 → `[0x0A]` (10 × 100ms)

#### 오디오 설정 커맨드

##### 0x17 - initSpk (스피커 볼륨 설정)  
| 매개변수 | 타입 | 범위 | 단위 | 설명 |
|---------|------|------|------|------|
| volume | uint8_t | 0-10 | level | 볼륨 레벨 (0=무음, 10=최대) |

**데이터 크기**: 1 byte  
**예제**: 중간 볼륨 → `[0x05]`

#### IMU 설정 커맨드

##### 0x19 - initGyroAct (IMU 활성화 각도 설정)
| 매개변수 | 타입 | 범위 | 단위 | 설명 |
|---------|------|------|------|------|
| active_angle | uint8_t | 0-180 | degree | 가열 시작 각도 |

**데이터 크기**: 1 byte  
**예제**: 45도 → `[0x2D]`

##### 0x20 - initGyroRel (IMU 상대 각도 설정)
| 매개변수 | 타입 | 범위 | 단위 | 설명 |
|---------|------|------|------|------|
| relative_angle | uint8_t | 0-180 | degree | 냉각 시작 각도 |

**데이터 크기**: 1 byte  
**예제**: 30도 → `[0x1E]`

#### 🎯 핵심 시스템 커맨드

##### 0x21 - initMode (운영 모드 설정) ⭐
| 매개변수 | 타입 | 범위 | 단위 | 설명 |
|---------|------|------|------|------|
| mode | uint8_t | 0-1 | enum | 0=AI_MODE, 1=IMU_MODE |

**데이터 크기**: 1 byte  
**기본값**: `1` (IMU_MODE - 시스템 시작 시)  
**중요**: 이 커맨드로 제어권이 변경됨

```c
// AI 모드로 변경 (ESP가 제어권 획득)
initDeviceMode(0);  // ESP가 0x51 커맨드 사용 가능

// IMU 모드로 변경 (STM이 제어권 획득) 
initDeviceMode(1);  // ESP의 0x51 커맨드 차단
```

---

## 7. REQUEST 커맨드 (0x30-0x42)

### 📊 현재 값 조회 커맨드 (데이터 없음 → 응답 데이터)

#### 온도 조회 커맨드

##### 0x30 - reqTempSleep (절전 모드 온도 조회)
**요청**: 데이터 없음  
**응답**: 2 bytes `[temp_int, temp_dec]`  
**예제**: 25.5°C → `[0x19, 0x37]`

##### 0x31 - reqTempWaiting (대기 모드 온도 조회)
**요청**: 데이터 없음  
**응답**: 2 bytes `[temp_int, temp_dec]`  
**예제**: 30.0°C → `[0x1E, 0x00]`

##### 0x32 - reqTempForceUp (동작 모드 온도 조회)
**요청**: 데이터 없음  
**응답**: 2 bytes `[temp_int, temp_dec]`  
**예제**: 58.0°C → `[0x3A, 0x00]`

##### 0x34 - reqUpperTemp (상한 온도 조회)
**요청**: 데이터 없음  
**응답**: 2 bytes `[temp_int, temp_dec]`  
**예제**: 75.0°C → `[0x4B, 0x00]`

#### 시스템 설정 조회

##### 0x35 - reqPWMCoolFan (냉각팬 PWM 조회)
**요청**: 데이터 없음  
**응답**: 2 bytes `[current_level, max_level]`  
**예제**: 현재 5, 최대 10 → `[0x05, 0x0A]`

##### 0x36 - reqTimeout (타임아웃 값 조회)
**요청**: 데이터 없음  
**응답**: 8 bytes (4 × uint16_t, Little Endian)  
```
[force_up_low, force_up_high,     // FORCE_UP 타임아웃
 force_on_low, force_on_high,     // FORCE_ON 타임아웃  
 force_down_low, force_down_high, // FORCE_DOWN 타임아웃
 waiting_low, waiting_high]       // WAITING 타임아웃
```

##### 0x37 - reqSpk (스피커 볼륨 조회)
**요청**: 데이터 없음  
**응답**: 1 byte `[volume]` (0-10)  
**예제**: 볼륨 5 → `[0x05]`

##### 0x38 - reqDelay (감지 지연 조회)
**요청**: 데이터 없음  
**응답**: 1 byte `[delay]` (×100ms)  
**예제**: 1초 → `[0x0A]`

#### IMU 설정 조회

##### 0x39 - reqGyroAct (IMU 활성화 각도 조회)
**요청**: 데이터 없음  
**응답**: 1 byte `[angle]` (0-180°)  
**예제**: 45도 → `[0x2D]`

##### 0x40 - reqGyroRel (IMU 상대 각도 조회)
**요청**: 데이터 없음  
**응답**: 1 byte `[angle]` (0-180°)  
**예제**: 30도 → `[0x1E]`

##### 0x41 - reqMode (현재 운영 모드 조회)
**요청**: 데이터 없음  
**응답**: 1 byte `[mode]` (0=AI_MODE, 1=IMU_MODE)  
**예제**: IMU 모드 → `[0x01]`

##### 0x42 - reqPWMFan (메인 팬 속도 조회)
**요청**: 데이터 없음  
**응답**: 1 byte `[speed]` (0-3)  
**예제**: 속도 2 → `[0x02]`

---

## 8. CONTROL 커맨드 (0x50-0x61)

### ⚡ 실시간 제어 커맨드

#### 🔄 시스템 제어

##### 0x50 - ctrlReset (디바이스 리셋)
**데이터**: 없음  
**효과**: STM32 소프트웨어 리셋 수행  
**주의**: 모든 설정이 기본값으로 초기화됨

#### 🎯 핵심 디바이스 모드 제어

##### 0x51 - ctrlMode (디바이스 모드 변경) ⭐⭐⭐
| 매개변수 | 타입 | 범위 | 설명 |
|---------|------|------|------|
| mode | uint8_t | 0-4 | Device Mode (SLEEP~FORCE_DOWN) |

**데이터 크기**: 1 byte

**Device Mode 값**:
- `0`: SLEEP (절전)
- `1`: WAITING (대기)  
- `2`: FORCE_UP (자세 교정 상승)
- `3`: FORCE_ON (자세 유지)
- `4`: FORCE_DOWN (자세 하강)

**🚨 중요 제약사항**:
```
이 커맨드는 AI_MODE에서만 사용 가능!
IMU_MODE에서 호출 시 오류 메시지와 함께 차단됨.

현재 Operation Mode 확인 방법:
- isAIModeActive() 함수로 확인
- getCurrentOperationMode() 함수로 모드 조회
```

**예제**:
```c
// ✅ 올바른 사용법
if (isAIModeActive()) {
    setDeviceMode(DeviceMode::FORCE_UP);  // 자세 교정 시작
} else {
    // ❌ IMU 모드에서는 STM이 자체 제어
    ESP_LOGW("Cannot control device mode in IMU_MODE");
}
```

#### 🔊 오디오 제어

##### 0x52 - ctrlSpkOn (스피커 ON/OFF)
| 매개변수 | 타입 | 범위 | 설명 |
|---------|------|------|------|
| state | uint8_t | 0-1 | 0=OFF, 1=ON |

**데이터 크기**: 1 byte  
**예제**: 스피커 켜기 → `[0x01]`

#### 🌪️ 팬 제어

##### 0x53 - ctrlFanOn (메인 팬 ON/OFF)
| 매개변수 | 타입 | 범위 | 설명 |
|---------|------|------|------|
| state | uint8_t | 0-1 | 0=OFF, 1=ON |

**데이터 크기**: 1 byte  
**예제**: 팬 켜기 → `[0x01]`

##### 0x54 - ctrlFanPWM (메인 팬 속도) ⚠️ DEPRECATED
| 매개변수 | 타입 | 범위 | 설명 |
|---------|------|------|------|
| speed | uint8_t | 0-3 | 팬 속도 |

**권장**: `initPWMFan (0x22)` 사용

##### 0x55 - ctrlCoolFanOn (냉각팬 ON/OFF)
| 매개변수 | 타입 | 범위 | 설명 |
|---------|------|------|------|
| state | uint8_t | 0-1 | 0=OFF, 1=ON |

**데이터 크기**: 1 byte  
**예제**: 냉각팬 켜기 → `[0x01]`

##### 0x56 - ctrlCoolFanPWM (냉각팬 PWM) ⚠️ DEPRECATED
| 매개변수 | 타입 | 범위 | 설명 |
|---------|------|------|------|
| level | uint8_t | 0-10 | PWM 레벨 |

**권장**: `initPWMCoolFan (0x15)` 사용

#### 🔥 히팅 제어

##### 0x57 - ctrlHeatPadOn (히트패드 ON/OFF)
| 매개변수 | 타입 | 범위 | 설명 |
|---------|------|------|------|
| state | uint8_t | 0-1 | 0=OFF, 1=ON |

**데이터 크기**: 1 byte  
**예제**: 히트패드 켜기 → `[0x01]`

##### 0x58 - ctrlHeatPadTemp (히트패드 온도 설정)
| 매개변수 | 타입 | 범위 | 단위 | 설명 |
|---------|------|------|------|------|
| temp_int | uint8_t | 30-60 | °C | 온도 정수부 |
| temp_dec | uint8_t | 0-99 | 0.01°C | 온도 소수부 |

**데이터 크기**: 2 bytes  
**예제**: 45.5°C → `[0x2D, 0x37]`

#### ⚠️ 레거시 커맨드 (DEPRECATED)

##### 0x59 - ctrlForceUp (Force Up 모드) 
**권장 대체**: `ctrlMode(2)` 사용

##### 0x60 - ctrlForceDown (Force Down 모드)
**권장 대체**: `ctrlMode(4)` 사용  

##### 0x61 - ctrlSleeping (Sleep 모드)
**권장 대체**: `ctrlMode(0)` 사용

---

## 9. STATUS 커맨드 (0x70)

### 📡 센서 데이터 전송 (STM32 → ESP32)

##### 0x70 - statMessage (센서 데이터 메시지)

**데이터 크기**: 34 bytes (고정 길이)  
**전송 주기**: 약 100ms (STM32에서 자동 전송)

#### 📊 SensorReading 구조체 (34 bytes)

| 오프셋 | 필드 | 타입 | 크기 | 범위 | 단위 | 설명 |
|--------|------|------|------|------|------|------|
| 0-1 | leftGyro[0] | int16_t | 2 | ±32767 | dps | 왼발 자이로 X축 |
| 2-3 | leftGyro[1] | int16_t | 2 | ±32767 | dps | 왼발 자이로 Y축 |
| 4-5 | leftGyro[2] | int16_t | 2 | ±32767 | dps | 왼발 자이로 Z축 |
| 6-7 | leftAccel[0] | int16_t | 2 | ±32767 | mg | 왼발 가속도 X축 |
| 8-9 | leftAccel[1] | int16_t | 2 | ±32767 | mg | 왼발 가속도 Y축 |
| 10-11 | leftAccel[2] | int16_t | 2 | ±32767 | mg | 왼발 가속도 Z축 |
| 12-13 | rightGyro[0] | int16_t | 2 | ±32767 | dps | 오른발 자이로 X축 |
| 14-15 | rightGyro[1] | int16_t | 2 | ±32767 | dps | 오른발 자이로 Y축 |
| 16-17 | rightGyro[2] | int16_t | 2 | ±32767 | dps | 오른발 자이로 Z축 |
| 18-19 | rightAccel[0] | int16_t | 2 | ±32767 | mg | 오른발 가속도 X축 |
| 20-21 | rightAccel[1] | int16_t | 2 | ±32767 | mg | 오른발 가속도 Y축 |
| 22-23 | rightAccel[2] | int16_t | 2 | ±32767 | mg | 오른발 가속도 Z축 |
| 24-25 | leftPressure | int16_t | 2 | 0-65535 | gram | 왼발 압력 |
| 26-27 | rightPressure | int16_t | 2 | 0-65535 | gram | 오른발 압력 |
| 28-29 | outsideTemp | int16_t | 2 | -327~327 | 0.1°C | 외부 온도 |
| 30-31 | boardTemp | int16_t | 2 | -327~327 | 0.1°C | 보드 온도 |
| 32-33 | actuatorTemp | int16_t | 2 | -327~327 | 0.1°C | 액추에이터 온도 |

**총 34 bytes (17 × int16_t)**

#### 📏 데이터 해석 방법

##### 온도 값 변환
```c
float celsius = (float)tempValue / 10.0f;
// 예: tempValue = 255 → 25.5°C
```

##### IMU 데이터 활성 확인  
```c
bool imuActive = (leftGyro[0] != 0 || leftGyro[1] != 0 || leftGyro[2] != 0 ||
                 rightGyro[0] != 0 || rightGyro[1] != 0 || rightGyro[2] != 0);
```

##### 압력 임계값 예제
```c
const int16_t PRESSURE_THRESHOLD = 1000;  // 1kg
bool userDetected = (leftPressure > PRESSURE_THRESHOLD || 
                    rightPressure > PRESSURE_THRESHOLD);
```

---

## 10. EVENT 커맨드 (0x80-0x82)

### 📢 시스템 이벤트 알림 (STM32 → ESP32)

#### 0x80 - evtInitStart (초기화 시작 이벤트)
**데이터**: 없음  
**의미**: STM32가 초기화 프로세스를 시작함을 알림

#### 0x81 - evtInitResult (초기화 결과 이벤트)
| 매개변수 | 타입 | 범위 | 설명 |
|---------|------|------|------|
| result | uint8_t | 0-1 | 0=실패, 1=성공 |

**데이터 크기**: 1 byte  
**예제**: 초기화 성공 → `[0x01]`

#### 0x82 - evtMode (디바이스 모드 변경 이벤트)
| 매개변수 | 타입 | 범위 | 설명 |
|---------|------|------|------|
| new_mode | uint8_t | 0-4 | 새로운 Device Mode |

**데이터 크기**: 1 byte  
**예제**: WAITING 모드로 변경 → `[0x01]`

#### Device Mode 값 설명
| 값 | 모드 | 설명 |
|-----|------|------|
| 0 | SLEEP | 디바이스가 절전 모드 상태 |
| 1 | WAITING | 사용자 입력/감지 대기 상태 |
| 2 | FORCE_UP | 적극적으로 자세 교정 중 (상향) |
| 3 | FORCE_ON | 교정된 자세 유지 중 |
| 4 | FORCE_DOWN | 자세 하향 조정 중 |

**중요**: 이 이벤트로 ESP32는 STM32의 실제 모드를 추적함

---

## 11. ERROR 커맨드 (0x90)

### ⚠️ 오류 발생 알림 (STM32 → ESP32)

#### 0x90 - errInit (초기화 오류)
| 매개변수 | 타입 | 크기 | 설명 |
|---------|------|------|------|
| error_mask | uint16_t | 2 bytes | 오류 비트마스크 (Little Endian) |

#### 🎭 오류 비트마스크 (16비트)

| 비트 | 값 | 이름 | 설명 |
|------|------|------|------|
| 0 | 0x0001 | ERR_IR_TEMP | IR 온도 센서 오류 |
| 1 | 0x0002 | ERR_AMB_TEMP | 환경 온도 센서 오류 |
| 2 | 0x0004 | ERR_IMU | IMU 센서 오류 |
| 3 | 0x0008 | ERR_FAN | 메인 팬 오류 |
| 4 | 0x0010 | ERR_COOL_FAN | 냉각 팬 오류 |
| 5 | 0x0020 | ERR_TOF | TOF 거리 센서 오류 |
| 6 | 0x0040 | ERR_AUDIO | 오디오 시스템 오류 |
| 7 | 0x0080 | ERR_FSR | 압력 센서 오류 |
| 8 | 0x0100 | ERR_SD_CARD | SD 카드 오류 |
| 9 | 0x0200 | ERR_MP3_FILE | MP3 파일 오류 |
| 10-15 | - | (예약) | 미래 확장용 |

#### 🔍 오류 처리 예제
```c
uint16_t errorMask = (data[1] << 8) | data[0];  // Little Endian

if (errorMask & ERR_IMU) {
    ESP_LOGE(TAG, "IMU sensor error detected");
}
if (errorMask & ERR_FAN) {
    ESP_LOGE(TAG, "Main fan error detected");  
}
```

---

## 12. 구현 가이드라인

### 🛠️ STM32 개발자 가이드

#### 메시지 수신 처리
```c
void processUartMessage(uint8_t* buffer, size_t length) {
    // 1. 프레임 검증
    if (buffer[0] != STM || buffer[length-1] != ETX) {
        return;  // 잘못된 프레임
    }
    
    // 2. 체크섬 검증
    uint8_t calculatedChecksum = 0;
    for (int i = 2; i < length-2; i++) {  // DIR부터 DATA까지
        calculatedChecksum ^= buffer[i];
    }
    if (calculatedChecksum != buffer[length-2]) {
        return;  // 체크섬 오류
    }
    
    // 3. 커맨드 처리
    uint8_t command = buffer[3];
    uint8_t* data = &buffer[4];
    uint8_t dataLength = buffer[1] - 4;  // LEN - DIR - CMD - CHKSUM - ETX
    
    switch (command) {
        case 0x10: handleInitTempSleep(data); break;
        case 0x21: handleInitMode(data); break;
        case 0x51: handleCtrlMode(data); break;
        // ... 기타 커맨드
    }
}
```

#### 응답 메시지 전송
```c
void sendAckResponse(uint8_t command) {
    uint8_t response[6] = {
        0x02,           // STM
        0x04,           // LEN (DIR+CMD+CHKSUM+ETX)
        0x02,           // DIR (MSG_RESPONSE)
        command,        // CMD (에코)
        command ^ 0x02, // CHKSUM (DIR ⊕ CMD)
        0x03            // ETX
    };
    uart_write(response, 6);
}
```

### 🖥️ ESP32 개발자 가이드

#### 커맨드 전송
```c
bool sendCommand(uint8_t command, uint8_t* data, size_t dataLen) {
    // 1. Operation Mode 확인 (0x51 커맨드의 경우)
    if (command == 0x51 && !isAIModeActive()) {
        ESP_LOGW(TAG, "ctrlMode blocked in IMU_MODE");
        return false;
    }
    
    // 2. 메시지 구성
    uint8_t buffer[70];  // 최대 메시지 크기
    size_t msgLen = buildMessage(buffer, command, data, dataLen);
    
    // 3. 전송
    uart_write(buffer, msgLen);
    
    // 4. 응답 대기 (타임아웃: 5초)
    return waitForResponse(command);
}
```

#### 센서 데이터 처리
```c
void handleSensorData(uint8_t* data) {
    SensorReading* sensors = (SensorReading*)data;
    
    // 온도 변환
    float outsideTemp = sensors->outsideTemp / 10.0f;
    float boardTemp = sensors->boardTemp / 10.0f;
    
    // 사용자 감지
    bool userDetected = (sensors->leftPressure > 1000 || 
                        sensors->rightPressure > 1000);
    
    // TinyML 처리 (AI_MODE에서만)
    if (isAIModeActive() && userDetected) {
        int prediction = runTinyML(sensors);
        if (prediction == 2) {  // 자세 교정 필요
            setDeviceMode(DeviceMode::FORCE_UP);
        }
    }
}
```

---

## 13. 코드 예제

### 🚀 시스템 초기화 시퀀스

```c
void initializeSystem() {
    ESP_LOGI(TAG, "=== PSA System Initialization ===");
    
    // 1. UART 초기화
    initUartMaster();
    
    // 2. Operation Mode를 IMU로 설정 (시스템 약속)
    initDeviceMode(1);  // IMU_MODE
    ESP_LOGI(TAG, "Operation Mode: IMU_MODE (STM controls device)");
    
    // 3. 온도 설정
    initSleepTemperature(25, 0);      // 25.0°C
    initWaitingTemperature(30, 0);    // 30.0°C  
    initOperatingTemperature(58, 0);  // 58.0°C
    initUpperTemperatureLimit(75, 0); // 75.0°C
    
    // 4. 팬 설정
    initCoolingFanPWM(5, 10);  // 현재 5, 최대 10
    initPWMFanSpeed(2);        // 중간 속도
    
    // 5. 타이밍 설정  
    initTimeoutConfiguration(20, 50, 20, 100);  // 2초, 5초, 2초, 10초
    initForeDownDelay(10);     // 1초 지연
    
    // 6. 오디오 설정
    initSpeakerVolume(5);      // 중간 볼륨
    
    // 7. IMU 설정
    initGyroActiveAngle(45);   // 45도
    initGyroRelativeAngle(30); // 30도
    
    ESP_LOGI(TAG, "System initialization completed");
}
```

### 🔄 AI 모드 전환 및 자세 제어

```c
void switchToAIModeAndControl() {
    ESP_LOGI(TAG, "=== Switching to AI Mode ===");
    
    // 1. AI 모드로 전환
    if (initDeviceMode(0)) {  // AI_MODE
        ESP_LOGI(TAG, "✅ AI Mode activated - ESP can control device");
        
        // 2. 대기 모드로 설정
        if (setDeviceMode(DeviceMode::WAITING)) {
            ESP_LOGI(TAG, "Device set to WAITING mode");
            
            // 3. 사용자 감지 대기
            vTaskDelay(pdMS_TO_TICKS(2000));
            
            // 4. 자세 교정 시작
            if (setDeviceMode(DeviceMode::FORCE_UP)) {
                ESP_LOGI(TAG, "Posture correction started");
                
                vTaskDelay(pdMS_TO_TICKS(3000));  // 3초 동작
                
                // 5. 자세 유지
                setDeviceMode(DeviceMode::FORCE_ON);
                ESP_LOGI(TAG, "Maintaining posture");
            }
        }
    } else {
        ESP_LOGE(TAG, "❌ Failed to switch to AI Mode");
    }
}
```

### 🔍 실시간 모니터링 루프

```c
void monitoringTask(void* pvParameters) {
    while (1) {
        // 1. 현재 상태 확인
        DeviceMode currentMode = getCurrentMode();
        OperationMode opMode = getCurrentOperationMode();
        
        ESP_LOGI(TAG, "Device: %s, Operation: %s", 
                getDeviceModeString(currentMode),
                (opMode == OperationMode::AI_MODE) ? "AI" : "IMU");
        
        // 2. 센서 데이터 처리 (0x70 메시지에서 자동 수신)
        SensorReading sensors = getCurrentSensorData();
        
        // 3. 온도 모니터링
        float boardTemp = sensors.boardTemp / 10.0f;
        if (boardTemp > 70.0f) {
            ESP_LOGW(TAG, "⚠️ High temperature: %.1f°C", boardTemp);
        }
        
        // 4. 압력 감지
        bool userDetected = (sensors.leftPressure > 1000 || 
                            sensors.rightPressure > 1000);
        
        // 5. AI 모드에서만 자동 제어
        if (isAIModeActive() && userDetected) {
            // TinyML 추론 수행
            int prediction = runTinyMLInference(&sensors);
            
            switch (prediction) {
                case 2:  // 자세 교정 필요
                    if (currentMode != DeviceMode::FORCE_UP) {
                        setDeviceMode(DeviceMode::FORCE_UP);
                        ESP_LOGI(TAG, "🎯 Auto posture correction triggered");
                    }
                    break;
                    
                default:  // 정상 자세
                    if (currentMode == DeviceMode::FORCE_UP) {
                        setDeviceMode(DeviceMode::WAITING);
                        ESP_LOGI(TAG, "✅ Posture correction completed");
                    }
                    break;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));  // 100ms 주기
    }
}
```

---

## 14. 주의사항 및 베스트 프랙티스

### ⚠️ 중요 주의사항

#### 🚨 Operation Mode 제어권 관리
```
❗ 핵심 규칙: 
1. 시스템은 항상 IMU_MODE로 시작
2. 0x51 커맨드는 AI_MODE에서만 사용 가능
3. 모드 전환은 0x21 커맨드로만 수행
```

#### 🔄 올바른 모드 전환 순서
```c
// ✅ 올바른 방법
initDeviceMode(0);          // 1. AI 모드로 전환
if (isAIModeActive()) {     // 2. 모드 확인  
    setDeviceMode(DeviceMode::FORCE_UP);  // 3. 디바이스 제어
}

// ❌ 잘못된 방법
setDeviceMode(DeviceMode::FORCE_UP);  // IMU 모드에서 바로 제어 시도
```

#### ⏱️ 타이밍 및 성능 고려사항

##### 통신 타임아웃
- **ACK 응답**: 5초 이내
- **데이터 응답**: 5초 이내  
- **재시도**: 최대 3회
- **센서 데이터**: 100ms 주기로 수신

##### 메시지 처리 성능
```c
// ✅ 효율적인 방법 - 비동기 전송
sendCommandAsync(command, data, dataLen);
// 다른 작업 수행 가능
CommandState state = getCommandState(command);

// ❌ 비효율적인 방법 - 블로킹 대기
sendCommandAndWait(command, data, dataLen);  // 5초간 블로킹
```

### 🛡️ 오류 처리 베스트 프랙티스

#### 통신 오류 복구
```c
bool sendCommandWithRetry(uint8_t cmd, uint8_t* data, size_t len) {
    for (int retry = 0; retry < 3; retry++) {
        if (sendCommandAsync(cmd, data, len)) {
            CommandState state = waitForResponse(cmd, 5000);
            
            switch (state) {
                case CommandState::ACK_RECEIVED:
                case CommandState::DATA_RECEIVED:
                    return true;  // 성공
                    
                case CommandState::TIMEOUT:
                    ESP_LOGW(TAG, "Command 0x%02X timeout, retry %d/3", cmd, retry+1);
                    break;
                    
                case CommandState::ERROR:
                    ESP_LOGE(TAG, "Command 0x%02X error", cmd);
                    return false;  // 재시도 불가능한 오류
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));  // 100ms 대기 후 재시도
    }
    
    ESP_LOGE(TAG, "Command 0x%02X failed after 3 retries", cmd);
    return false;
}
```

#### 센서 데이터 검증
```c
bool validateSensorData(SensorReading* sensors) {
    // 1. 온도 범위 확인
    float boardTemp = sensors->boardTemp / 10.0f;
    if (boardTemp < -50.0f || boardTemp > 100.0f) {
        ESP_LOGW(TAG, "Invalid board temperature: %.1f°C", boardTemp);
        return false;
    }
    
    // 2. 압력 센서 범위 확인
    if (sensors->leftPressure < 0 || sensors->rightPressure < 0) {
        ESP_LOGW(TAG, "Negative pressure values detected");
        return false;
    }
    
    // 3. IMU 데이터 검증 (모든 값이 0인 경우 센서 오류)
    bool allZero = true;
    for (int i = 0; i < 3; i++) {
        if (sensors->leftGyro[i] != 0 || sensors->leftAccel[i] != 0 ||
            sensors->rightGyro[i] != 0 || sensors->rightAccel[i] != 0) {
            allZero = false;
            break;
        }
    }
    
    if (allZero) {
        ESP_LOGW(TAG, "All IMU sensors report zero - possible sensor error");
        // 경고만 출력, 데이터는 유효로 처리 (센서가 실제로 정지 상태일 수 있음)
    }
    
    return true;
}
```

### 🚀 성능 최적화

#### 메시지 배치 처리
```c
void batchInitialization() {
    // 여러 INIT 커맨드를 빠르게 연속 전송
    initSleepTemperature(25, 0);
    initWaitingTemperature(30, 0);
    initOperatingTemperature(58, 0);
    
    // 각 커맨드의 응답을 나중에 일괄 확인
    vTaskDelay(pdMS_TO_TICKS(500));  // STM 처리 시간 대기
    
    // 설정 확인 (REQUEST 커맨드로 검증)
    verifyInitialization();
}
```

#### 메모리 최적화
```c
// ✅ 스택 기반 버퍼 사용 (권장)
uint8_t buffer[USART_MESSAGE_MAXIMUM_LENGTH];
size_t msgLen = buildMessage(buffer, command, data, dataLen);

// ❌ 동적 할당 피하기
uint8_t* buffer = malloc(msgLen);  // ESP32에서 메모리 파편화 위험
```

### 📊 디버깅 및 로깅

#### 구조화된 로그 출력
```c
void logMessageFrame(const char* direction, uint8_t* frame, size_t length) {
    char hexString[256] = {0};
    for (size_t i = 0; i < length && i < 32; i++) {
        snprintf(hexString + strlen(hexString), 
                sizeof(hexString) - strlen(hexString), 
                "%02X ", frame[i]);
    }
    
    ESP_LOGI(TAG, "[%s] CMD:0x%02X LEN:%d [%s]", 
             direction, frame[3], frame[1], hexString);
}

// 사용 예
logMessageFrame("ESP→STM", messageBuffer, messageLength);
logMessageFrame("STM→ESP", receivedBuffer, receivedLength);
```

#### 상태 모니터링 대시보드
```c
void printSystemStatus() {
    ESP_LOGI(TAG, "=== PSA System Status ===");
    ESP_LOGI(TAG, "Operation Mode: %s", 
             isAIModeActive() ? "AI (ESP controls)" : "IMU (STM controls)");
    ESP_LOGI(TAG, "Device Mode: %s", getDeviceModeString(getCurrentMode()));
    ESP_LOGI(TAG, "Communication: %s", 
             isCommunicationHealthy() ? "Healthy" : "Error");
             
    SensorReading sensors = getCurrentSensorData();
    ESP_LOGI(TAG, "Temperatures: Board=%.1f°C, Outside=%.1f°C", 
             sensors.boardTemp/10.0f, sensors.outsideTemp/10.0f);
    ESP_LOGI(TAG, "Pressure: L=%dg, R=%dg", 
             sensors.leftPressure, sensors.rightPressure);
}
```

---

## 📝 버전 정보

**문서 버전**: v1.0  
**프로토콜 버전**: v1.0  
**최종 업데이트**: 2025-01-26  
**호환성**: ESP32 IDF v4.0+, STM32 HAL  

---

## 📞 지원 및 문의

기술적 문의나 구현 관련 질문이 있으시면 개발팀에 연락해 주세요.

---

*이 문서는 PSA (Postural Stability Assistant) 프로젝트의 공식 프로토콜 사양서입니다.*