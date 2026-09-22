import time
import numpy as np
from multiprocessing import shared_memory


class FlangeBuffer:
    """
    Python에서 메모리를 직접 생성/할당하는 Owner 클래스

    C++ 메모리 레이아웃과 100% 동기화:
    struct RobotFrameData {
        int64_t timestamp_us;        // 8 bytes
        double flange_TF[16]; // 128 bytes (4x4 Matrix)
    };                               // Total = 136 bytes

    struct SharedDataBuffer {
        uint64_t sequence;           // 8 bytes
        uint32_t head;               // 4 bytes
        uint32_t padding;            // 4 bytes
        RobotFrameData buffer[100];  // 136 * 100 = 13,600 bytes
    };                               // Total = 13,616 bytes
    """

    def __init__(self, shm_name="robot_shm", is_owner=True):
        self.buffer_size = 100
        self.shm_name = shm_name
        self.is_owner = is_owner

        # C++ 구조체와 동일한 NumPy dtype 정의
        self.header_dtype = np.dtype([("sequence", np.uint64), ("head", np.uint32), ("padding", np.uint32)])

        self.frame_dtype = np.dtype(
            [("timestamp_us", np.int64), ("flange_TF", np.float64, (4, 4))]  # 4x4 Row-major Matrix
        )

        # 전체 공유 메모리 크기 계산 (13,616 bytes)
        self.shared_memory_size = self.header_dtype.itemsize + (self.frame_dtype.itemsize * self.buffer_size)

        if self.is_owner:
            # 기존 공유 메모리가 남아있다면 삭제 후 신규 생성
            try:
                old_shm = shared_memory.SharedMemory(name=self.shm_name)
                old_shm.close()
                old_shm.unlink()
            except FileNotFoundError:
                pass

            self.shm = shared_memory.SharedMemory(
                name=self.shm_name, create=True, size=self.shared_memory_size
            )
            # 메모리 0으로 초기화
            self.shm.buf[: self.shared_memory_size] = b"\x00" * self.shared_memory_size
            print(f"[Python SHM] 공유 메모리 '{self.shm_name}' 할당 완료 ({self.shared_memory_size} bytes)")
        else:
            self.shm = shared_memory.SharedMemory(name=self.shm_name, create=False)

        # Header 및 Circular Buffer 매핑
        self.header_buffer = np.frombuffer(self.shm.buf, dtype=self.header_dtype, count=1)[0]
        self.frame_buffer = np.frombuffer(
            self.shm.buf, dtype=self.frame_dtype, count=self.buffer_size, offset=self.header_dtype.itemsize
        )

    def get_ordered_data(self):
        """시간순(오래된 순 -> 최신 순)으로 재정렬된 버퍼 데이터와 Sequence 반환"""
        sequence = int(self.header_buffer["sequence"])
        head = int(self.header_buffer["head"])

        if sequence == 0:
            return None, 0

        if sequence < self.buffer_size:
            ordered_data = self.frame_buffer[:sequence]
        else:
            ordered_data = np.roll(self.frame_buffer, -head, axis=0)

        return ordered_data, sequence

    def get_flange_tf(self, target_ts_us: int):
        sequence = int(self.header_buffer["sequence"])
        head = int(self.header_buffer["head"])
        if sequence == 0:
            return {"target_ts": target_ts_us, "matched_ts": 0, "time_diff_us": 0, "flange_TF": np.eye(4)}

        valid_size = sequence if sequence < self.buffer_size else self.buffer_size
        start_idx = 0 if sequence < self.buffer_size else head

        oldest_idx = start_idx
        newest_idx = (start_idx + valid_size - 1) % self.buffer_size

        oldest_ts = self.frame_buffer[oldest_idx]["timestamp_us"]
        newest_ts = self.frame_buffer[newest_idx]["timestamp_us"]

        # 1. 범위 밖(가장 오래된/최신) 데이터 예외 처리
        if target_ts_us <= oldest_ts:
            best_idx = oldest_idx
        elif target_ts_us >= newest_ts:
            best_idx = newest_idx
        else:
            # 2. 이진 탐색(Binary Search)으로 가장 가까운 타임스탬프 인덱스 검색
            low = 0
            high = valid_size - 1

            while low < high:
                mid = (low + high) // 2
                real_mid_idx = (start_idx + mid) % self.buffer_size

                if self.frame_buffer[real_mid_idx]["timestamp_us"] < target_ts_us:
                    low = mid + 1
                else:
                    high = mid

            right_idx = (start_idx + high) % self.buffer_size
            left_idx = (start_idx + high - 1 + self.buffer_size) % self.buffer_size

            diff_right = abs(self.frame_buffer[right_idx]["timestamp_us"] - target_ts_us)
            diff_left = abs(self.frame_buffer[left_idx]["timestamp_us"] - target_ts_us)

            best_idx = right_idx if diff_right < diff_left else left_idx

        # 3. 매칭된 프레임에서 바로 4x4 행렬 추출
        matched_frame = self.frame_buffer[best_idx]
        matched_ts = matched_frame["timestamp_us"]

        # C++에서 4x4 행렬로 기록한 'flange_TF' 필드를 바로 반환
        T_base_flange = matched_frame["flange_TF"]

        return {
            "target_ts": target_ts_us,
            "matched_ts": int(matched_ts),
            "time_diff_us": int(abs(matched_ts - target_ts_us)),
            "flange_TF": T_base_flange,
        }

    def close(self):
        self.header_buffer = None
        self.frame_buffer = None
        if hasattr(self, "shm") and self.shm is not None:
            self.shm.close()
            if self.is_owner:
                try:
                    self.shm.unlink()
                    print(f"[Python SHM] 공유 메모리 '{self.shm_name}' 해제 완료.")
                except FileNotFoundError:
                    pass
            self.shm = None


def print_terminal_dashboard(shm_name="robot_shm"):
    """Python에서 할당한 SHM에서 C++이 기록하는 데이터를 실시간 시각화하는 대시보드"""
    # Python에서 메모리를 할당(is_owner=True)
    reader = FlangeBuffer(shm_name=shm_name, is_owner=True)

    try:
        while True:
            data, seq = reader.get_ordered_data()

            if data is not None and len(data) > 0:
                latest = data[-1]
                oldest = data[0]
                dt_ms = (latest["timestamp_us"] - oldest["timestamp_us"]) / 1000.0

                latest_T = latest["flange_TF"]
                pos_x, pos_y, pos_z = latest_T[0, 3], latest_T[1, 3], latest_T[2, 3]

                # 화면 상단으로 커서 이동 (제자리 덮어쓰기)
                print("\033[H\033[J", end="")

                print(
                    "=========================================================================================="
                )
                print(f" [Hanwha Robot SHM Monitor]  누적 프레임: {seq:8d} | 저장된 버퍼 수: {len(data)}/100")
                print(f" 버퍼 시간 span: {dt_ms:8.2f} ms | 최신 Timestamp: {latest['timestamp_us']} us")
                print(
                    "=========================================================================================="
                )
                print(f" [최신 TCP Position]  X: {pos_x:8.2f} mm | Y: {pos_y:8.2f} mm | Z: {pos_z:8.2f} mm")
                print(
                    "------------------------------------------------------------------------------------------"
                )

                print(" [최신 4x4 Homogeneous Transformation Matrix (T_base_tcp)]")
                for r in range(4):
                    print(
                        f"  | {latest_T[r, 0]:8.4f}  {latest_T[r, 1]:8.4f}  {latest_T[r, 2]:8.4f}  {latest_T[r, 3]:10.2f} |"
                    )
                print(
                    "------------------------------------------------------------------------------------------"
                )

                print(" [최근 링버퍼 샘플 목록 (시간순)]")
                print(
                    " Index | Timestamp (us) |   X (mm)   |   Y (mm)   |   Z (mm)   |   R11 (R)  |   R22 (R)  |   R33 (R)  "
                )
                print(
                    "-------+----------------+------------+------------+------------+------------+------------+------------"
                )

                if len(data) > 10:
                    show_indices = list(range(3)) + [-1] + list(range(len(data) - 5, len(data)))
                else:
                    show_indices = list(range(len(data)))

                for idx in show_indices:
                    if idx == -1:
                        print(
                            "  ...  |       ...      |    ...     |    ...     |    ...     |    ...     |    ...     |    ...    "
                        )
                        continue

                    row = data[idx]
                    T = row["flange_TF"]
                    print(
                        f" {idx:4d}  | {row['timestamp_us']:14d} | {T[0,3]:10.2f} | {T[1,3]:10.2f} | {T[2,3]:10.2f} | {T[0,0]:10.4f} | {T[1,1]:10.4f} | {T[2,2]:10.4f}"
                    )

                print(
                    "=========================================================================================="
                )
                print(" Ctrl+C를 누르면 종료됩니다.")

            time.sleep(0.05)  # 20Hz 터미널 출력

    except KeyboardInterrupt:
        print("\n[Monitor] 모니터링을 종료합니다.")
    finally:
        reader.close()


if __name__ == "__main__":
    print_terminal_dashboard(shm_name="shm_flange")

    # reader = FlangeBuffer(shm_name=shm_name, is_owner=True)
    # try:
    #     while 1:
    #         print(reader.get_flange_tf(time.time() * 1e6)["time_diff_us"])
    # except KeyboardInterrupt:
    #     print("\n[Monitor] 모니터링을 종료합니다.")
    # finally:
    #     reader.close()
