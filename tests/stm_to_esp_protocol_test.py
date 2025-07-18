#!/usr/bin/env python3
"""
STM 시뮬레이터 - ESP32의 명령을 받아서 STM처럼 응답
ESP→STM 통신 테스트용
"""

import serial
import time
import sys
from datetime import datetime

class STMSimulator:
    def __init__(self, port='/dev/ttyUSB1', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.received_commands = []
        
    def log(self, message):
        """로그 출력"""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        print(f"[{timestamp}] {message}")
        
    def connect(self):
        """ESP32와 연결 (STM 역할)"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1,
                bytesize=8,
                parity='N',
                stopbits=1
            )
            self.log(f"✓ STM 시뮬레이터 시작: {self.port} at {self.baudrate} baud")
            return True
        except Exception as e:
            self.log(f"✗ 연결 실패: {e}")
            return False
    
    def disconnect(self):
        """연결 해제"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.log("✓ STM 시뮬레이터 종료")
    
    def calculate_checksum(self, packet_without_checksum_and_etx):
        """ESP32 프로토콜 체크섬 계산"""
        checksum = 0xA5
        for byte in packet_without_checksum_and_etx:
            checksum ^= byte
        return checksum & 0xFF
    
    def create_ack_response(self, received_command):
        """ESP32 명령에 대한 STM ACK 응답 생성"""
        # STM → ESP 응답: STX(0x02) + LEN + DIR(0x02) + CMD + CHKSUM + ETX(0x03)
        packet = []
        
        # STX
        packet.append(0x02)
        
        # LEN (DIR + CMD + CHKSUM + ETX = 4 bytes)
        packet.append(0x04)
        
        # DIR (MSG_REQUEST - STM initiating)
        packet.append(0x20)
        
        # CMD (원본 명령 에코)
        packet.append(received_command)
        
        # CHECKSUM 계산
        checksum_data = packet  # STX + LEN + DIR + CMD
        checksum = self.calculate_checksum(checksum_data)
        packet.append(checksum)
        
        # ETX
        packet.append(0x03)
        
        return bytes(packet)
    
    def parse_esp_command(self, data):
        """ESP32로부터 받은 명령 파싱"""
        if len(data) < 6:  # 최소 패킷 크기
            return None
            
        if data[0] != 0x02:  # STX 확인
            return None
            
        if data[-1] != 0x03:  # ETX 확인
            return None
            
        packet_len = data[1]
        direction = data[2]
        command = data[3]
        
        # ESP → STM 방향 확인 (MSG_REQUEST=0x20, MSG_RESPONSE=0x02)
        if direction != 0x20 and direction != 0x02:
            return None
            
        # 체크섬 검증
        checksum_data = data[:-2]  # 체크섬과 ETX 제외
        calculated_checksum = self.calculate_checksum(checksum_data)
        received_checksum = data[-2]
        
        if calculated_checksum != received_checksum:
            self.log(f"체크섬 오류: 계산={calculated_checksum:02X}, 수신={received_checksum:02X}")
            return None
            
        return {
            'command': command,
            'direction': direction,
            'length': packet_len,
            'data': data[4:-2] if len(data) > 6 else b'',
            'raw': data
        }
    
    def get_command_name(self, command):
        """명령 이름 반환"""
        command_names = {
            # INIT Commands (0x1X)
            0x10: "INIT: Sleep Temperature",
            0x11: "INIT: Waiting Temperature", 
            0x12: "INIT: Operating Temperature",
            0x13: "INIT: Heat Pad Temperature",
            0x14: "INIT: Upper Temperature Limit",
            0x15: "INIT: Cooling Fan PWM",
            0x16: "INIT: Timeout",
            0x17: "INIT: Delay",
            
            # REQUEST Commands (0x3X)
            0x30: "REQUEST: Sleep Temperature",
            0x31: "REQUEST: Waiting Temperature",
            0x32: "REQUEST: Operating Temperature", 
            0x33: "REQUEST: Heat Pad Temperature",
            0x34: "REQUEST: Upper Temperature",
            0x35: "REQUEST: Cooling Fan PWM",
            0x36: "REQUEST: Timeout",
            0x37: "REQUEST: Speaker Volume",
            0x38: "REQUEST: Detection Delay",
            
            # CONTROL Commands (0x5X)
            0x50: "CONTROL: Reset Device",
            0x51: "CONTROL: Set Device Mode",
            0x52: "CONTROL: Set Speaker Volume",
            0x53: "CONTROL: Set Fan State",
            0x54: "CONTROL: Set Fan PWM",
            0x55: "CONTROL: Set Cooling Fan State",
            0x56: "CONTROL: Set Cooling Fan PWM",
            0x57: "CONTROL: Set Heat Pad State",
            0x58: "CONTROL: Set Heat Pad Temperature",
            0x59: "CONTROL: Set Pose Detection Mode"
        }
        
        return command_names.get(command, f"Unknown Command (0x{command:02X})")
    
    def run_simulator(self):
        """STM 시뮬레이터 실행"""
        self.log("=== STM 시뮬레이터 시작 ===")
        self.log("ESP32로부터 명령을 대기합니다...")
        self.log("Ctrl+C로 종료")
        
        try:
            while True:
                # ESP32로부터 데이터 수신 확인
                if self.serial_conn.in_waiting > 0:
                    data = self.serial_conn.read(self.serial_conn.in_waiting)
                    
                    if data:
                        self.log(f"← 수신: {data.hex().upper()}")
                        
                        # 명령 파싱
                        command_info = self.parse_esp_command(data)
                        
                        if command_info:
                            command = command_info['command']
                            command_name = self.get_command_name(command)
                            
                            self.log(f"✓ 파싱 성공: {command_name}")
                            
                            if command_info['data']:
                                self.log(f"  데이터: {command_info['data'].hex().upper()}")
                            
                            # ACK 응답 생성 및 전송
                            ack_response = self.create_ack_response(command)
                            self.serial_conn.write(ack_response)
                            self.serial_conn.flush()
                            
                            self.log(f"→ ACK 전송: {ack_response.hex().upper()}")
                            
                            # 명령 기록
                            self.received_commands.append({
                                'timestamp': datetime.now(),
                                'command': command,
                                'name': command_name,
                                'data': command_info['data']
                            })
                            
                        else:
                            self.log("✗ 패킷 파싱 실패")
                
                time.sleep(0.01)  # 10ms 대기
                
        except KeyboardInterrupt:
            self.log("\n사용자 중단...")
        except Exception as e:
            self.log(f"오류 발생: {e}")
        finally:
            self.generate_report()
    
    def generate_report(self):
        """수신된 명령 리포트 생성"""
        self.log("")
        self.log("=== STM 시뮬레이터 리포트 ===")
        self.log(f"총 수신된 명령: {len(self.received_commands)}개")
        
        if self.received_commands:
            self.log("\n명령 상세:")
            for i, cmd in enumerate(self.received_commands, 1):
                timestamp = cmd['timestamp'].strftime("%H:%M:%S.%f")[:-3]
                self.log(f"  {i}. [{timestamp}] {cmd['name']}")
                if cmd['data']:
                    self.log(f"      데이터: {cmd['data'].hex().upper()}")
        else:
            self.log("수신된 명령이 없습니다.")

def main():
    """메인 함수"""
    import argparse
    
    parser = argparse.ArgumentParser(description='STM 시뮬레이터 - ESP→STM 통신 테스트')
    parser.add_argument('--port', '-p', default='/dev/ttyUSB1', help='시리얼 포트')
    parser.add_argument('--baudrate', '-b', type=int, default=115200, help='Baud rate')
    
    args = parser.parse_args()
    
    simulator = STMSimulator(port=args.port, baudrate=args.baudrate)
    
    if simulator.connect():
        simulator.run_simulator()
        simulator.disconnect()
    else:
        sys.exit(1)

if __name__ == "__main__":
    main()