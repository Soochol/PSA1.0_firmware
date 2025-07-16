#!/usr/bin/env python3
"""
Run All Tests - 모든 프로토콜 테스트 실행
전체 메시지 프로토콜 테스트 suite 실행
"""

import subprocess
import os
import time
from datetime import datetime

def run_command(command, description):
    """명령어 실행"""
    print(f"\n{'='*60}")
    print(f"실행 중: {description}")
    print(f"명령어: {command}")
    print(f"{'='*60}")
    
    try:
        result = subprocess.run(command, shell=True, capture_output=True, text=True)
        
        if result.returncode == 0:
            print(f"✓ 성공: {description}")
            if result.stdout:
                print("출력:")
                print(result.stdout)
        else:
            print(f"✗ 실패: {description}")
            if result.stderr:
                print("오류:")
                print(result.stderr)
        
        return result.returncode == 0
    except Exception as e:
        print(f"✗ 실행 오류: {e}")
        return False

def cleanup_before_start():
    """테스트 시작 전 정리"""
    print("=== 테스트 시작 전 정리 ===")
    print("✓ 정리 도구 제거됨 - 건너뛰기")
    return True

def main():
    """메인 함수"""
    print("="*60)
    print("전체 메시지 프로토콜 테스트 Suite")
    print("="*60)
    print(f"시작 시간: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    
    # 테스트 시작 전 정리
    cleanup_before_start()
    
    # 가상환경 Python 경로 (tests 폴더에서 실행)
    venv_python = "../venv/bin/python3"
    
    # 테스트 실행 순서
    tests = [
        {
            'command': f"{venv_python} comprehensive_protocol_test.py",
            'description': '종합 프로토콜 테스트 (STM→ESP)',
            'required': True
        }
    ]
    
    results = []
    
    for i, test in enumerate(tests, 1):
        print(f"\n[{i}/{len(tests)}] {test['description']}")
        
        success = run_command(test['command'], test['description'])
        results.append({
            'test': test['description'],
            'success': success,
            'required': test['required']
        })
        
        if not success and test['required']:
            print(f"⚠️  필수 테스트 실패: {test['description']}")
            print("   다른 테스트를 계속 진행합니다...")
        
        # 테스트 간 간격
        if i < len(tests):
            time.sleep(2)
    
    # 결과 요약
    print("\n" + "="*60)
    print("테스트 결과 요약")
    print("="*60)
    
    total_tests = len(results)
    passed_tests = sum(1 for r in results if r['success'])
    failed_tests = total_tests - passed_tests
    
    print(f"총 테스트: {total_tests}")
    print(f"성공: {passed_tests}")
    print(f"실패: {failed_tests}")
    print(f"성공률: {(passed_tests/total_tests)*100:.1f}%")
    
    print("\n상세 결과:")
    for result in results:
        status = "✓" if result['success'] else "✗"
        required = " (필수)" if result['required'] else ""
        print(f"{status} {result['test']}{required}")
    
    # 리포트 파일 확인
    print("\n생성된 파일:")
    report_files = [
        "protocol_test_report.html",
        "protocol_test_report.txt",
        "protocol_test_report_*.json",
        "bidirectional_test_report_*.json",
        "scenario_test_report_*.json"
    ]
    
    for pattern in report_files:
        if '*' in pattern:
            import glob
            files = glob.glob(pattern)
            for file in files:
                if os.path.exists(file):
                    print(f"  ✓ {file}")
        else:
            if os.path.exists(pattern):
                print(f"  ✓ {pattern}")
    
    print(f"\n완료 시간: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    
    # 테스트 완료 후 정리
    print("\n=== 테스트 완료 후 정리 ===")
    try:
        result = subprocess.run("python3 test_cleanup_helper.py --port /dev/ttyUSB1", 
                              shell=True, capture_output=True, text=True, timeout=30)
        
        if result.returncode == 0:
            print("✓ 테스트 완료 후 정리 완료")
        else:
            print("⚠️  테스트 완료 후 정리 중 일부 문제 발생")
    except Exception as e:
        print(f"⚠️  테스트 완료 후 정리 실패: {e}")
    
    # 권장사항
    print("\n" + "="*60)
    print("권장사항")
    print("="*60)
    
    if passed_tests == total_tests:
        print("✓ 모든 테스트가 성공했습니다!")
        print("  프로토콜 구현이 정상적으로 동작하고 있습니다.")
    elif failed_tests > 0:
        print("⚠️  일부 테스트가 실패했습니다.")
        print("  1. ESP32 펌웨어가 정상적으로 업로드되었는지 확인하세요")
        print("  2. 시리얼 포트 연결 상태를 확인하세요")
        print("  3. 생성된 리포트를 확인하여 상세한 오류 정보를 검토하세요")
    
    if os.path.exists("protocol_test_report.html"):
        print(f"\n📊 상세한 결과는 protocol_test_report.html 파일을 확인하세요")

if __name__ == "__main__":
    main()