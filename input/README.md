# 원본 CAD

다음 세 파일을 수정 없이 보존합니다. 단위는 mm입니다.

| 파일 | 용도 |
| --- | --- |
| `DIGIT-Robotiq Adapter-2F85-V1.STL` | DIGIT 장착부를 보존할 원본 어댑터 |
| `HAND_E_FINGER_SUPPORT_20181026-Sep-06-2024-02-31-14-1482-PM.step` | 하부 체결 구멍과 조립 위치의 기준 |
| `ROBOTIQ_HAND-E_NO_FINGERTIPS_20190924-Sep-06-2024-02-31-06-0971-PM.step` | 전체 조립의 기준 좌표계와 그리퍼 본체 |

수정 결과는 `output/`에 생성됩니다. 원본 해시는 `output/validation.json`에 기록됩니다. 파일 이름과 경로는 `scripts/build_assembly.py`에서 참조합니다.
