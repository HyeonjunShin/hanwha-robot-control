import hanwha_robot_py
from shm_target import TargetShm
import time

# 생성자 호출 (IP 전달)
robot = hanwha_robot_py.Hanwha("192.168.100.200")

# 공유메모리 연결 및 실행
success = robot.conn_flange_shm("flange_shm")
target_shm = TargetShm(shm_name="target_shm", is_owner=False)

if success:
    # print("공유 메모리 연결 성공")
    # robot.run()
    while 1:
        target = target_shm.read()
        if target.detected:
            timestamp_us = target.timestamp
            bbox = target.bbox
            score = target.score
            target_TF = target.TF
            print(target_TF)
        # time.sleep(1)

# 종료 시
robot.disconnect()
