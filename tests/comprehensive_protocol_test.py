#!/usr/bin/env python3
"""
Comprehensive Protocol Test - STM→ESP Messages
전체 메시지 프로토콜 테스트 (STM에서 ESP로 전송하는 모든 메시지 타입)
"""

import serial
import time
import sys
from datetime import datetime
import json

class ProtocolTester:
    def __init__(self, port='/dev/ttyUSB1', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.test_results = []
        
    def connect(self):
        """ESP32 연결"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=2,
                bytesize=8,
                parity='N',
                stopbits=1
            )
            self.log("✓ ESP32 연결 성공")
            return True
        except Exception as e:
            self.log(f"✗ ESP32 연결 실패: {e}")
            return False
    
    def disconnect(self):
        """연결 해제"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.log("✓ ESP32 연결 해제")
    
    def log(self, message):
        """로그 출력"""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        print(f"[{timestamp}] {message}")
    
    def calculate_checksum(self, packet_without_checksum_and_etx):
        """STM 프로토콜 체크섬 계산"""
        checksum = 0xA5
        for byte in packet_without_checksum_and_etx:
            checksum ^= byte
        return checksum & 0xFF
    
    def create_packet(self, command, data=None):
        """기본 패킷 생성"""
        packet = []
        
        # STM (0x02)
        packet.append(0x02)
        
        # Calculate LEN based on data size
        data_len = len(data) if data else 0
        packet_len = 4 + data_len  # DIR + CMD + CHKSUM + ETX + DATA
        packet.append(packet_len)
        
        # DIR (0x02, STM -> ESP)
        packet.append(0x02)
        
        # CMD
        packet.append(command)
        
        # DATA
        if data:
            packet.extend(data)
        
        # CHECKSUM
        checksum = self.calculate_checksum(packet)
        packet.append(checksum)
        
        # ETX (0x03)
        packet.append(0x03)
        
        return bytes(packet)
    
    def send_packet_and_get_response(self, packet, description):
        """패킷 전송 및 응답 수신"""
        try:
            self.log(f"→ {description}")
            self.log(f"  패킷: {packet.hex().upper()}")
            
            # 패킷 전송
            for byte in packet:
                self.serial_conn.write(bytes([byte]))
                time.sleep(0.001)
            self.serial_conn.flush()
            
            # 응답 대기
            time.sleep(1.0)
            
            responses = []
            for i in range(5):  # 최대 5번 읽기 시도
                if self.serial_conn.in_waiting > 0:
                    response = self.serial_conn.read(self.serial_conn.in_waiting)
                    if response:
                        responses.append(response)
                        self.log(f"  수신 데이터 {i+1}: {response.hex().upper()}")
                time.sleep(0.3)
            
            # 응답 분석
            success = len(responses) > 0
            decoded_responses = []
            
            for i, response in enumerate(responses):
                try:
                    decoded = response.decode('utf-8', errors='ignore')
                    decoded_responses.append(decoded)
                    self.log(f"← 응답 {i+1}: {decoded.strip()}")
                except:
                    self.log(f"← 응답 {i+1} (Raw): {response.hex().upper()}")
            
            return success, decoded_responses
            
        except Exception as e:
            self.log(f"✗ 테스트 실패: {e}")
            return False, []
    
    def test_status_message(self):
        """STATUS 메시지 테스트 (0x70)"""
        self.log("\n=== STATUS 메시지 테스트 ===")
        
        # 정상 센서 데이터
        sensor_data = [
            0x00, 0x01, 0x00, 0x02, 0x00, 0x03,  # Left IMU Gyro
            0x00, 0x10, 0x00, 0x20, 0x00, 0x30,  # Left IMU Accel
            0x00, 0x04, 0x00, 0x05, 0x00, 0x06,  # Right IMU Gyro
            0x00, 0x40, 0x00, 0x50, 0x00, 0x60,  # Right IMU Accel
            0x03, 0xE8, 0x03, 0xE8,              # Pressure sensors
            0x00, 0x19, 0x00, 0x1A, 0x00, 0x1B,  # Temperature sensors
            0x00, 0x64, 0x00, 0xC8, 0x00, 0x64,  # Other sensors
            0x00, 0x00                           # IMU Events
        ]
        
        packet = self.create_packet(0x70, sensor_data)
        success, responses = self.send_packet_and_get_response(packet, "STATUS: 정상 센서 데이터")
        
        self.test_results.append({
            'test': 'STATUS Message (0x70)',
            'description': '정상 센서 데이터',
            'success': success,
            'responses': responses
        })
        
        # TinyML 트리거 데이터
        tinyml_data = [
            0x00, 0x0A, 0x00, 0x14, 0x00, 0x1E,  # Left IMU Gyro (높은 값)
            0x00, 0x64, 0x00, 0xC8, 0x01, 0x2C,  # Left IMU Accel (높은 값)
            0x00, 0x0F, 0x00, 0x19, 0x00, 0x23,  # Right IMU Gyro (높은 값)
            0x00, 0x96, 0x00, 0xFA, 0x01, 0x5E,  # Right IMU Accel (높은 값)
            0x03, 0x20, 0x03, 0x84,              # Pressure sensors
            0x00, 0x18, 0x00, 0x19, 0x00, 0x1A,  # Temperature sensors
            0x00, 0x5F, 0x00, 0x96, 0x00, 0x62,  # Other sensors (ToF=150 < 175)
            0x01, 0x01                           # IMU Events (triggered)
        ]
        
        packet = self.create_packet(0x70, tinyml_data)
        success, responses = self.send_packet_and_get_response(packet, "STATUS: TinyML 트리거 데이터")
        
        self.test_results.append({
            'test': 'STATUS Message (0x70)',
            'description': 'TinyML 트리거 데이터',
            'success': success,
            'responses': responses
        })
    
    def test_event_messages(self):
        """EVENT 메시지 테스트 (0x8X)"""
        self.log("\n=== EVENT 메시지 테스트 ===")
        
        # 0x80: 초기화 시작
        packet = self.create_packet(0x80, None)
        success, responses = self.send_packet_and_get_response(packet, "EVENT: 초기화 시작 (0x80)")
        
        self.test_results.append({
            'test': 'EVENT Message (0x80)',
            'description': '초기화 시작',
            'success': success,
            'responses': responses
        })
        
        # 0x81: 초기화 완료
        packet = self.create_packet(0x81, None)
        success, responses = self.send_packet_and_get_response(packet, "EVENT: 초기화 완료 (0x81)")
        
        self.test_results.append({
            'test': 'EVENT Message (0x81)',
            'description': '초기화 완료',
            'success': success,
            'responses': responses
        })
        
        # 0x82: 모드 변경 (각 모드별 테스트)
        modes = [
            (0, "SLEEP"),
            (1, "WAITING"),
            (2, "FORCE_UP"),
            (3, "FORCE_ON"),
            (4, "FORCE_DOWN"),
            (5, "ERROR")
        ]
        
        for mode_val, mode_name in modes:
            packet = self.create_packet(0x82, [mode_val])
            success, responses = self.send_packet_and_get_response(packet, f"EVENT: 모드 변경 to {mode_name} (0x82)")
            
            self.test_results.append({
                'test': 'EVENT Message (0x82)',
                'description': f'모드 변경 to {mode_name}',
                'success': success,
                'responses': responses
            })
    
    def test_error_messages(self):
        """ERROR 메시지 테스트 (0x9X)"""
        self.log("\n=== ERROR 메시지 테스트 ===")
        
        # 0x90: 초기화 에러
        error_codes = [
            (1, "Initialization Failed"),
            (2, "Communication Failed"),
            (3, "SD Card Failed"),
            (4, "Voltage Failed")
        ]
        
        for error_code, error_desc in error_codes:
            packet = self.create_packet(0x90, [error_code])
            success, responses = self.send_packet_and_get_response(packet, f"ERROR: {error_desc} (0x90)")
            
            self.test_results.append({
                'test': 'ERROR Message (0x90)',
                'description': f'{error_desc} (code: {error_code})',
                'success': success,
                'responses': responses
            })
    
    def test_invalid_messages(self):
        """잘못된 메시지 테스트"""
        self.log("\n=== 잘못된 메시지 테스트 ===")
        
        # 알 수 없는 명령어
        packet = self.create_packet(0xFF, None)
        success, responses = self.send_packet_and_get_response(packet, "잘못된 명령어 (0xFF)")
        
        self.test_results.append({
            'test': 'Invalid Message',
            'description': '알 수 없는 명령어 (0xFF)',
            'success': success,
            'responses': responses
        })
        
        # 체크섬 오류 (의도적으로 잘못된 체크섬)
        packet = list(self.create_packet(0x70, [0x00] * 42))
        packet[-2] = 0xFF  # 잘못된 체크섬
        packet = bytes(packet)
        
        success, responses = self.send_packet_and_get_response(packet, "체크섬 오류")
        
        self.test_results.append({
            'test': 'Invalid Message',
            'description': '체크섬 오류',
            'success': success,
            'responses': responses
        })
    
    def generate_report(self):
        """테스트 결과 리포트 생성"""
        self.log("\n" + "="*60)
        self.log("전체 프로토콜 테스트 결과 리포트")
        self.log("="*60)
        
        total_tests = len(self.test_results)
        passed_tests = sum(1 for result in self.test_results if result['success'])
        failed_tests = total_tests - passed_tests
        
        self.log(f"총 테스트: {total_tests}")
        self.log(f"성공: {passed_tests}")
        self.log(f"실패: {failed_tests}")
        self.log(f"성공률: {(passed_tests/total_tests)*100:.1f}%")
        
        self.log("\n상세 결과:")
        for result in self.test_results:
            status = "✓" if result['success'] else "✗"
            self.log(f"{status} {result['test']}: {result['description']}")
        
        # JSON 파일로 저장
        report_file = f"protocol_test_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        with open(report_file, 'w', encoding='utf-8') as f:
            json.dump({
                'timestamp': datetime.now().isoformat(),
                'summary': {
                    'total_tests': total_tests,
                    'passed_tests': passed_tests,
                    'failed_tests': failed_tests,
                    'success_rate': (passed_tests/total_tests)*100
                },
                'detailed_results': self.test_results
            }, f, indent=2, ensure_ascii=False)
        
        self.log(f"\n리포트 저장됨: {report_file}")
    
    def run_all_tests(self):
        """모든 테스트 실행"""
        self.log("=== 전체 메시지 프로토콜 테스트 시작 ===")
        
        if not self.connect():
            return
        
        try:
            # ESP32 초기화 대기
            time.sleep(2)
            
            # 각 테스트 실행
            self.test_status_message()
            self.test_event_messages()
            self.test_error_messages()
            self.test_invalid_messages()
            
            # 리포트 생성
            self.generate_report()
            
        except KeyboardInterrupt:
            self.log("\n사용자 중단...")
        except Exception as e:
            self.log(f"테스트 중 오류 발생: {e}")
        finally:
            self.disconnect()

def main():
    """메인 함수"""
    import argparse
    
    parser = argparse.ArgumentParser(description='전체 메시지 프로토콜 테스트')
    parser.add_argument('--port', '-p', default='/dev/ttyUSB1', help='시리얼 포트')
    parser.add_argument('--baudrate', '-b', type=int, default=115200, help='Baud rate')
    
    args = parser.parse_args()
    
    tester = ProtocolTester(port=args.port, baudrate=args.baudrate)
    tester.run_all_tests()

if __name__ == "__main__":
    main()