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
    
    def parse_consecutive_responses(self, raw_response):
        """연속된 응답을 개별 패킷으로 분리"""
        packets = []
        i = 0
        
        while i < len(raw_response):
            if raw_response[i] == 0x02:  # STX 찾기
                if i + 1 < len(raw_response):
                    packet_len = raw_response[i + 1]  # LEN 필드
                    
                    # 패킷 길이가 유효한지 확인 (LEN은 STX와 LEN을 제외한 나머지 바이트 수)
                    total_packet_size = packet_len + 2  # STX + LEN + (DIR + CMD + DATA + CHKSUM + ETX)
                    
                    if packet_len >= 4 and i + total_packet_size <= len(raw_response):
                        packet = raw_response[i:i + total_packet_size]
                        
                        # ETX 확인 (패킷의 마지막 바이트)
                        if packet[-1] == 0x03:
                            packets.append(packet)
                            self.log(f"    분리된 패킷: {packet.hex().upper()}")
                        else:
                            self.log(f"    경고: ETX가 없는 패킷 발견 at index {i}, 예상: 0x03, 실제: 0x{packet[-1]:02X}")
                        
                        i += total_packet_size
                    else:
                        self.log(f"    경고: 잘못된 패킷 길이 {packet_len} at index {i}, 필요: {total_packet_size}, 남은: {len(raw_response) - i}")
                        i += 1
                else:
                    self.log(f"    경고: LEN 필드를 읽을 수 없음 at index {i}")
                    break
            else:
                i += 1
        
        self.log(f"    총 {len(packets)}개 패킷 분리됨")
        return packets
    
    def verify_packet_checksum(self, packet):
        """패킷의 체크섬 검증"""
        if len(packet) < 4:
            return False
        
        # 체크섬 계산 (STX + LEN + DIR + CMD + DATA)
        checksum_data = packet[:-2]  # 체크섬과 ETX 제외
        calculated_checksum = self.calculate_checksum(checksum_data)
        received_checksum = packet[-2]  # 끝에서 두 번째가 체크섬
        
        return calculated_checksum == received_checksum
    
    def create_packet(self, command, data=None):
        """기본 패킷 생성"""
        packet = []
        
        # STM (0x02)
        packet.append(0x02)
        
        # Calculate LEN based on data size
        data_len = len(data) if data else 0
        packet_len = 4 + data_len  # DIR + CMD + CHKSUM + ETX + DATA
        packet.append(packet_len)
        
        # DIR (MSG_REQUEST - STM initiating)
        packet.append(0x20)
        
        # CMD
        packet.append(command)
        
        # DATA
        if data:
            packet.extend(data)
        
        # CHECKSUM (calculate from STM + LEN + DIR + CMD + DATA)
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
    
    def test_consecutive_messages(self):
        """연속 메시지 테스트 (STM→ESP 5개 메시지)"""
        self.log("\n=== 연속 메시지 테스트 ===")
        
        # 5개의 서로 다른 메시지 준비
        test_messages = [
            {
                'command': 0x70,
                'data': [0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x10, 0x00, 0x20, 0x00, 0x30,
                        0x00, 0x04, 0x00, 0x05, 0x00, 0x06, 0x00, 0x40, 0x00, 0x50, 0x00, 0x60,
                        0x01, 0x00, 0x01, 0x00, 0x00, 0x19, 0x00, 0x1A, 0x00, 0x1B,
                        0x00, 0x64, 0x00, 0xC8, 0x00, 0x64, 0x00, 0x00],
                'description': 'STATUS: 센서 데이터 #1'
            },
            {
                'command': 0x80,
                'data': None,
                'description': 'EVENT: 초기화 시작'
            },
            {
                'command': 0x82,
                'data': [2],  # FORCE_UP
                'description': 'EVENT: 모드 변경 (FORCE_UP)'
            },
            {
                'command': 0x90,
                'data': [3],  # SD Card Failed
                'description': 'ERROR: SD Card Failed'
            },
            {
                'command': 0x70,
                'data': [0x00, 0x0A, 0x00, 0x14, 0x00, 0x1E, 0x00, 0x64, 0x00, 0xC8, 0x01, 0x2C,
                        0x00, 0x0F, 0x00, 0x19, 0x00, 0x23, 0x00, 0x96, 0x00, 0xFA, 0x01, 0x5E,
                        0x01, 0x50, 0x01, 0x60, 0x00, 0x18, 0x00, 0x19, 0x00, 0x1A,
                        0x00, 0x5F, 0x00, 0x96, 0x00, 0x62, 0x01, 0x01],
                'description': 'STATUS: 센서 데이터 #2'
            }
        ]
        
        # 모든 패킷을 먼저 생성
        packets = []
        for msg in test_messages:
            packet = self.create_packet(msg['command'], msg['data'])
            packets.append((packet, msg['description']))
        
        self.log(f"→ 연속 메시지 전송 시작 (총 {len(packets)}개)")
        
        # 연속으로 패킷 전송 (간격 최소화)
        start_time = time.time()
        for i, (packet, desc) in enumerate(packets):
            self.log(f"  [{i+1}/5] {desc}")
            self.log(f"    패킷: {packet.hex().upper()}")
            
            # 패킷 전송
            for byte in packet:
                self.serial_conn.write(bytes([byte]))
                time.sleep(0.001)  # 1ms 간격
            self.serial_conn.flush()
            
            # 짧은 간격으로 다음 패킷 전송
            time.sleep(0.05)  # 50ms 간격
        
        send_time = time.time() - start_time
        self.log(f"→ 모든 패킷 전송 완료 (소요시간: {send_time:.3f}초)")
        
        # 응답 수집 (좀 더 길게 대기)
        self.log("→ 응답 수집 중...")
        time.sleep(2.0)  # 2초 대기
        
        all_raw_responses = []
        
        for i in range(15):  # 최대 15번 읽기 시도
            if self.serial_conn.in_waiting > 0:
                response = self.serial_conn.read(self.serial_conn.in_waiting)
                if response:
                    all_raw_responses.append(response)
                    self.log(f"  Raw 응답 {len(all_raw_responses)}: {response.hex().upper()}")
            time.sleep(0.1)
        
        # 모든 raw 응답을 하나로 합치기
        combined_response = b''.join(all_raw_responses)
        self.log(f"→ 합쳐진 응답: {combined_response.hex().upper()}")
        
        # 개별 패킷으로 분리
        individual_packets = []
        if combined_response:
            self.log("→ 개별 패킷 분리 중...")
            individual_packets = self.parse_consecutive_responses(combined_response)
        
        # 각 패킷 검증 및 분석
        valid_packets = []
        expected_commands = [0x70, 0x80, 0x82, 0x90, 0x70]  # 전송한 명령 순서
        
        for i, packet in enumerate(individual_packets):
            self.log(f"→ 패킷 {i+1} 검증 중...")
            
            # 체크섬 검증
            if self.verify_packet_checksum(packet):
                self.log(f"  ✓ 체크섬 검증 성공")
            else:
                self.log(f"  ✗ 체크섬 검증 실패")
                continue
            
            # 응답 명령 확인 (DIR=0x20, CMD=원본명령)
            if len(packet) >= 4:
                direction = packet[2]
                command = packet[3]
                
                if direction == 0x02:  # MSG_RESPONSE - ESP ACK
                    self.log(f"  ✓ ESP→STM 응답 확인: 명령 0x{command:02X}")
                    
                    # 예상 명령과 매칭 확인
                    if i < len(expected_commands) and command == expected_commands[i]:
                        self.log(f"  ✓ 명령-응답 매칭 성공: {i+1}번째 명령")
                        valid_packets.append(packet)
                    else:
                        self.log(f"  ⚠️ 명령-응답 순서 불일치: 예상 0x{expected_commands[i] if i < len(expected_commands) else 0:02X}, 실제 0x{command:02X}")
                        valid_packets.append(packet)  # 여전히 유효한 패킷으로 처리
                else:
                    self.log(f"  ✗ 잘못된 응답 방향: 0x{direction:02X}")
            else:
                self.log(f"  ✗ 패킷이 너무 짧음: {len(packet)} bytes")
        
        # 결과 분석
        success = len(valid_packets) > 0
        response_count = len(valid_packets)
        
        # 최종 결과 요약
        success_rate = (response_count / len(packets)) * 100 if len(packets) > 0 else 0
        
        self.log(f"→ 연속 메시지 테스트 완료:")
        self.log(f"  - 전송 메시지: {len(packets)}개")
        self.log(f"  - 수신 응답: {response_count}개")
        self.log(f"  - 응답 성공률: {success_rate:.1f}%")
        self.log(f"  - 전송 시간: {send_time:.3f}초")
        self.log(f"  - 평균 전송시간: {send_time/len(packets):.3f}초/메시지")
        
        if response_count == len(packets):
            self.log(f"  ✓ 모든 명령에 대한 ACK 수신 완료!")
        elif response_count > 0:
            self.log(f"  ⚠️ 일부 ACK 누락됨 ({len(packets) - response_count}개)")
        else:
            self.log(f"  ✗ ACK 응답을 받지 못했습니다")
        
        # 개별 패킷 정보를 문자열로 변환
        packet_info = []
        for i, packet in enumerate(valid_packets):
            packet_info.append({
                'index': i + 1,
                'hex': packet.hex().upper(),
                'command': f"0x{packet[3]:02X}" if len(packet) >= 4 else "Unknown"
            })
        
        self.test_results.append({
            'test': 'Consecutive Messages',
            'description': f'연속 {len(packets)}개 메시지 전송',
            'success': success,
            'responses': packet_info,
            'stats': {
                'sent_messages': len(packets),
                'received_responses': response_count,
                'success_rate': success_rate,
                'total_time': send_time,
                'avg_transmission_time': send_time/len(packets),
                'raw_response_hex': combined_response.hex().upper() if combined_response else ""
            }
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
            self.test_consecutive_messages()  # 연속 메시지 테스트 추가
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