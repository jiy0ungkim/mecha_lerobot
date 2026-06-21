# mecha_lerobot — SO-ARM101 LeRobot 텔레옵 시각화 & ACT 정책 학습

> 메카트로닉스 이론 및 실습(캡스톤디자인) 5조 프로젝트 — SO-ARM101 로봇팔에 LeRobot 프레임워크와 ACT/smolVLA 정책을 적용한 Pick & Place 구현, FK/IK 기반 궤적 검증, RViz2 실시간 시각화.

## 1. 프로젝트 개요

목표는 세 가지입니다.

1. **VLA 기반 Pick & Place**: LeRobot의 Vision-Language-Action 모델(ACT, smolVLA)로 물체 인식 → 집기(pick) → 목표 위치 이동 → 놓기(place) 수행
2. **Kinematics 기반 경로 계산**: Forward/Inverse Kinematics로 End Effector 목표 위치와 이동 궤적 계산
3. **RViz 기반 동작 검증**: 계산된 경로를 RViz2에서 시각화하여 로봇팔이 의도한 궤적을 정확히 추종하는지 검증

이 저장소는 위 1, 3번(텔레옵/IK 시각화)을 담당하는 ROS 2 패키지와, 2번 결과를 학습하는 ACT 학습 노트북을 포함합니다.

## 2. 팀

| 이름 | 학부 | 학번 |
|---|---|---|
| 정혜정 | 화공생명공학부 | 2212625 |
| 김지영 | 기계시스템학부 | 2215908 |
| 이미지 | IT공학전공 | 2216232 |
| 안혜승 | 기계시스템학부 | 2315279 |


## 3. 전체 파이프라인

```
환경 구축          데이터 수집              전처리/병합                평가
─────────         ─────────────           ─────────────             ──────
SO-101 leader-     Teleoperation으로        카메라 동기화/모터        학습된 정책을
follower +         4방향 pick-and-place     이상치 확인 후            실제 로봇에 배포,
카메라 2대(top/    동작 시연                aggregate_datasets()로    물체를 정확히
wrist) 연결,       (left→right,             방향별 데이터셋 병합 →    집는지 여부로
ROS2(WSL2/Ubuntu)  right→left,              HuggingFace Hub에        성능 확인
환경에 LeRobot     up→down, down→up)        업로드
설치/세팅          약 100개 에피소드 수집
```

4방향 중 **up↔down 방향의 에피소드가 상대적으로 부족**하여 해당 방향의 실패율이 높을 것으로 예상됩니다.

## 4. 핵심 기능 (이 저장소 코드)

| 기능 | 담당 모듈 | 설명 |
|---|---|---|
| 순/역기구학 (FK/IK) | `so101_kinematics.py` | URDF 기반 6축 체인, Damped Least-Squares(LM 변형) IK |
| 텔레옵 시각화 | `so101_teleop_udp.py` + `so101_rviz_node.py` | 리더-팔로워 텔레옵 관절값을 UDP로 전송 → RViz2에 단일 로봇으로 표시 |
| IK 궤적 추적 시각화 | `so101_ik_follower.py` + `so101_dual_rviz_node.py` | heart/circle/line/CSV 궤적을 IK로 추종, 목표(파란색 고스트) vs 실제 로봇을 RViz2에 동시 표시 |
| 추종 오차 분석 | `plot_so101_ik_csv*.py`, `ik_csv/` | 관절별 target-actual 시계열 플롯 및 RMSE(raw/settled) 계산 |
| ACT 정책 학습 | `training/ACT_training.ipynb` | LeRobot `lerobot-train` CLI로 ACT 정책 학습 (Colab/Jupyter) |
| 하드웨어 자산 | `assets/`, `urdf/` | SO-101 3D 프린팅 부품 STL, 보정된 URDF |

## 5. 디렉토리 구조

```
mecha_lerobot/
├── assets/                  # SO-101 3D 프린팅 부품 STL (13종)
├── ik_csv/                  # IK 추적 실행 로그 (날짜별: CSV + PNG + RMSE)
│   └── MMDD/HHMMSS.csv, HHMMSS.png, HHMMSS_rmse.csv
├── launch/
│   ├── display_teleop_so101.launch.py   # 텔레옵 → RViz 시각화
│   └── display_ik_so101.launch.py       # IK 궤적 추적 → target/actual 비교 시각화
├── mecha_lerobot/            # ROS 2 Python 패키지 본체
│   ├── so101_kinematics.py
│   ├── so101_rviz_node.py
│   ├── so101_dual_rviz_node.py
│   ├── so101_teleop_udp.py
│   ├── so101_ik_follower.py
│   ├── so101_target_trajectory.py
│   ├── plot_so101_ik_csv.py
│   ├── plot_so101_ik_csv_set_start_time.py
│   └── lerobot/              # ⚠ HuggingFace LeRobot 라이브러리 전체 (벤더링, 14절 참고)
├── rviz/                     # RViz2 설정 (.rviz)
├── training/
│   └── ACT_training.ipynb    # ACT 정책 학습 노트북
├── urdf/
│   └── so101_new_calib.urdf  # 보정된 SO-101 URDF
├── video/                    # 데모 영상 (data_selection.webm, rviz.webm)
├── package.xml / setup.py / setup.cfg
└── test/                     # ament 코드 스타일 테스트
```

## 6. 요구 사항

- ROS 2 (rclpy, sensor_msgs, geometry_msgs, visualization_msgs, robot_state_publisher, rviz2, xacro)
- Conda 가상환경 (LeRobot 설치, Python 3.10+) — ROS 2와 분리된 별도 환경 권장
- LeRobot (`pip install -e ".[act,dataset]"`)
- numpy, pandas, matplotlib (CSV 분석/플롯용)

## 7. 빌드

```bash
cd ~/ros2_ws
colcon build --packages-select mecha_lerobot
source install/setup.bash
```

## 8. 사용법

### 8.1 텔레옵 + RViz 실시간 시각화

```bash
ros2 launch mecha_lerobot display_teleop_so101.launch.py \
  follower_port:=/dev/follower \
  leader_port:=/dev/leader \
  follower_id:=my_awesome_follower_arm \
  leader_id:=my_awesome_leader_arm \
  fps:=30
```

내부적으로 `so101_teleop_udp.py`가 LeRobot 텔레옵 루프를 돌며 관절값을 UDP(`127.0.0.1:5005`)로 송신하고, `so101_rviz_node`가 이를 수신해 `/joint_states`로 publish합니다.

### 8.2 IK 궤적 추적 + Target/Actual 비교 시각화

```bash
ros2 launch mecha_lerobot display_ik_so101.launch.py \
  follower_port:=/dev/follower \
  follower_id:=my_awesome_follower_arm \
  trajectory:=heart \
  plane:=YZ \
  loop:=true
```

주요 옵션 (`so101_ik_follower.py`):

| 옵션 | 기본값 | 설명 |
|---|---|---|
| `--trajectory` | heart | heart / circle / line / csv |
| `--target-csv` | None | `--trajectory csv`일 때 x,y,z 컬럼을 가진 CSV 경로 |
| `--n-points` | 600 | 궤적 분할 포인트 수 |
| `--scale` | 0.05 | 궤적 스케일(m) |
| `--center` | 0.18,0.0,0.12 | 궤적 중심 좌표(m) |
| `--plane` | YZ | 궤적이 그려지는 평면 |
| `--dry-run` | False | 실제 로봇 연결 없이 RViz 테스트만 |
| `--robot-action-units` / `--observation-units` | degrees | LeRobot과의 단위 변환 기준 |

### 8.3 추종 오차 분석 (RMSE)

```bash
python3 mecha_lerobot/plot_so101_ik_csv.py
```

`ik_csv/{날짜}/{시간}.csv`를 읽어 관절별 target vs actual 시계열 그래프(`.png`)와 RMSE 표(`_rmse.csv`, raw/settled 구분)를 생성합니다.

### 8.4 ACT 정책 학습

`training/ACT_training.ipynb`를 Colab 또는 Jupyter에서 실행합니다.

1. `lerobot`, `huggingface_hub` 설치 + GR00T 모듈 패치 (의존성 충돌 회피)
2. `notebook_login()`으로 HuggingFace 인증
3. 병합 데이터셋으로 ACT 학습:

```bash
lerobot-train \
  --dataset.repo_id=jiy0ung/merged_data_v1 \
  --policy.type=act \
  --output_dir=outputs/train/ACT_kjy_v1 \
  --job_name=hf_act_training_job \
  --policy.device=cuda \
  --wandb.enable=False \
  --policy.repo_id=jiy0ung/ACT_kjy_v1 \
  --batch_size=8 \
  --steps=20000
```

실기 평가:

```bash
python -m lerobot.scripts.lerobot_rollout \
  --policy.path=jiy0ung/ACT_kjy_v1 \
  --robot.type=so101_follower \
  --robot.port=/dev/follower
```

## 9. IK 정량/정성 평가 결과

발표 자료 기준 RMSE (본 저장소 `ik_csv/0621/141643_rmse.csv`의 settled RMSE와 동일):

| Joint | RMSE (deg) |
|---|---|
| shoulder_pan | 2.2411 |
| shoulder_lift | 1.2169 |
| elbow_flex | 2.076 |
| wrist_flex | 0.7961 |
| wrist_roll | 0.3816 |

정성 평가는 하트 모양 궤적을 그리는 IK 데모로 수행했으며, RViz의 target line과 실제 로봇 동작이 일치함을 확인했습니다 (`video/rviz.webm`).

## 10. 정책 학습 결과 비교

| 정책 | 에피소드 | 프레임 수 | 스텝 | Batch | LR | Loss | 비고 |
|---|---|---|---|---|---|---|---|
| ACT #1 (원점 시작) | 10 | 53,098 | 20,000 | 8 | 1e-5 | 0.054 | |
| ACT #2 (원점+상하좌우 시작) | 30 | 112,052 | 20,000 | 8 | 1e-5 | 0.069 | 카메라 시야 밖 타겟 → 동작 제한, 시야 내에서는 궤적 추종 양호 |
| smolVLA | 30 | 112,052 | 20,000 | 8 | 1e-5 | 0.032 | language instruction 포함, gripper가 타겟 방향으로 움직이지 않고 떨림(jitter) 발생 |

**ACT vs smolVLA loss는 절대값으로 단순 비교할 수 없습니다.** smolVLA의 loss가 더 낮음에도 실제 동작 성능은 ACT보다 떨어졌는데, 원인은 메커니즘 차이입니다.

- **ACT**: 한 번의 추론으로 다수 프레임의 action chunk를 생성(낮은 추론 빈도), input modality가 image + robot state로 단순
- **smolVLA**: image + language instruction + robot state를 모두 처리하는 multimodal reasoning 구조. Object recognition, spatial relation understanding, instruction parsing 등 control 이전 단계가 추가되어 ACT보다 연산 단계가 훨씬 큼

smolVLA는 ACT보다 훨씬 많은 데이터와 연산이 필요한 구조인데, 현재 데이터(30 에피소드)로는 구조에 필요한 학습량을 충족하지 못해 성능이 낮게 나왔다는 것이 결론입니다.

## 11. 문제 해결 과정 (트러블슈팅 히스토리)

| 시도 | 원인 가설 | 검증 방법 | 결과 |
|---|---|---|---|
| 1. 데이터 부족 | 에피소드 수 절대 부족(초기 48개, loss 0.159) | 6/3~6/17 사이 48→52→60→95→115→135개로 누적 수집 | 데이터 양 증가, 근본 원인은 별도로 존재 |
| 2. 오버피팅 실험 | 만두-접시 색상 유사성으로 시각적 구분 실패 | 단일 에피소드 2000 steps 오버피팅(loss 0.22%), 타겟을 초록색 거북이로 교체 | 거북이 집기는 성공했으나 잡자마자 그리퍼가 열리는 오류 → 색상 문제는 맞았으나 별개 이슈 존재 확인 |
| 3. 데이터 오염 | 리더 암을 빠르게 조작 → 팔로워 추종 실패 → joint 값 노이즈/포화(±100) | joint 값 시계열 그래프 분석 | 시연 속도를 낮추고 동작 단계 사이 정지 구간 추가 → 전 joint 부드러운 궤적 확보, 정지 구간 데이터 학습 후 trajectory 추종 성공 |

## 12. 한계 및 개선 방향

- **문제**: smolVLA는 ACT보다 복잡한 구조라 더 많은 데이터가 필요한데, 현재 에피소드 수(30개)로는 타겟/시작 위치가 조금만 달라져도 policy가 복구되지 않음
- **개선 방향**:
  - 학습용 에피소드 수를 최소 50개로 증가
  - 다양한 위치의 타겟 물체를 다루는 에피소드 수집 → 일반화 성능 향상
  - 밝기 변화 등 다양한 환경 조건의 에피소드 수집
  - single task가 아닌 multiple task로 지정 → smolVLA의 language instruction 활용도 향상

## 13. 데모 영상

- `video/data_selection.webm` — 데이터 수집 과정
- `video/rviz.webm` — RViz 시각화 데모

## 14. 알려진 이슈 / 제한사항 (코드 레벨)

- **경로 하드코딩**: `launch/display_ik_so101.launch.py`의 `conda_python`, `ik_script` 기본값과 `setup.cfg`의 `build_scripts.executable`, `plot_so101_ik_csv.py`의 `csv_path`/`output_path`가 특정 사용자 환경(`/home/kjy/...`)에 고정되어 있습니다. 다른 환경에서 그대로 clone해서 실행하면 동작하지 않습니다. launch 인자로 노출하거나 환경변수 기반으로 바꿔야 재사용이 가능합니다.
- **타겟 좌표계 시각화**: `display_ik_so101.launch.py`는 target URDF를 별도 네임스페이스로 만들어 `target_y_offset`만큼 띄울 수 있으나 기본값 0이라 실제 로봇과 완전히 겹쳐 보입니다. 비교 가시성이 필요하면 offset을 0이 아니게 주는 것을 권장합니다.
- **`mecha_lerobot/lerobot/`에 LeRobot 라이브러리 전체(758개 파일, ~10MB)가 그대로 커밋되어 있고 `.gitignore`도 없습니다.** 서드파티 라이브러리를 통째로 버전관리에 포함하면 저장소가 무거워지고, 실제 수정한 패치 부분을 diff로 구분하기 어려워집니다. `requirements.txt`/`environment.yml`로 의존성만 명시하고 라이브러리 본체는 제외하는 것이 정석입니다.

## 15. 라이선스 & 출처

- 패키지 라이선스: Apache-2.0
- IK 구현 참고: `jooyongsim/soarm_ik_mecha_tutorial`
- 하드웨어: SO-ARM101 (TheRobotStudio)
- 프레임워크: [HuggingFace LeRobot](https://github.com/huggingface/lerobot)
