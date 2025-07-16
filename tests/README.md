# PSA1.0 Protocol Tests

ESP32-STM32 간 프로토콜 통신 테스트

## 파일 구조

- `comprehensive_protocol_test.py` - 전체 프로토콜 테스트 (STM→ESP)
- `run_all_tests.py` - 테스트 실행 메인 스크립트

## 사용법

### 1. 가상환경 활성화
```bash
cd /home/blessp/my_code/PSA1.0_firmware-master
source venv/bin/activate
```

### 2. 테스트 실행
```bash
cd tests
python3 run_all_tests.py
```

### 3. 개별 테스트 실행
```bash
cd tests
../venv/bin/python3 comprehensive_protocol_test.py
```

## 요구사항

- ESP32가 `/dev/ttyUSB1`에 연결되어 있어야 함
- 가상환경에 필요한 패키지 설치 (`pyserial` 등)
- 펌웨어가 업로드되어 있어야 함

## 테스트 내용

### comprehensive_protocol_test.py
- STM→ESP 메시지 프로토콜 전체 검증
- 체크섬 계산 및 패킷 생성
- 응답 메시지 확인
- 테스트 결과 JSON 리포트 생성