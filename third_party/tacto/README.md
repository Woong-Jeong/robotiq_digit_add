# Tacto DIGIT 자산

- 저장소: [facebookresearch/tacto](https://github.com/facebookresearch/tacto)
- 커밋: `a21d0c4626d74a546d94859226c2fea348babb6a`
- 라이선스: [MIT](LICENSE)
- 상세 출처: [source.json](source.json)

| 로컬 파일 | 원본 경로 | 역할 |
| --- | --- | --- |
| `digit.STL` | `meshes/digit.STL` | 미터 단위 원본 센서 메시 |
| `digit.urdf` | `meshes/digit.urdf` | 원본 센서 설명 참고 자료 |
| `config_digit.yml` | `tacto/config_digit.yml` | 센서 및 gel 프레임 설정 |
| `LICENSE` | `LICENSE` | 원본 MIT 라이선스 |

`scripts/package_robot.py`는 원본 센서 메시를 ROS 패키지의 `meshes/digit.STL`로 복사합니다. 원본 URDF는 참고용으로 보관하며, 이 조립 모델의 URDF는 CAD 좌표를 기준으로 별도 생성합니다.
