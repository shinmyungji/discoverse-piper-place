# DISCOVERSE Piper Block Place + LeRobot ZMQ Policy Server 전체 실행 가이드 (복붙용)

아래 문서는 **그대로 README.md에 복붙**해서 사용할 수 있는 “전체 과정”입니다.  
(데이터 생성 → 변환 → 서버 실행 → 롤아웃 eval)

---

## 0) 폴더/경로 전제 (권장)

- DISCOVERSE 레포(너가 올린 GitHub): `~/robotics/discoverse-piper-place`
- LeRobot 작업 폴더(로컬): `~/lerobot_ws`

아래 2개를 먼저 환경변수로 고정하면, 절대경로 실수 없이 재현됩니다.

```bash
export DISCOVERSE_DIR=~/robotics/discoverse-piper-place
export LEROBOT_WS_DIR=~/lerobot_ws
```

**팁**  
경로를 README에 `/home/qwer1234/...`처럼 박아두면 다른 사람이 100% 실패합니다. 위처럼 환경변수로 통일하는 게 가장 안전합니다.

---

## 1) (터미널 1) DISCOVERSE 환경 진입

```bash
cd "$DISCOVERSE_DIR"
source ~/miniconda3/etc/profile.d/conda.sh
conda activate discoverse
```

**팁**  
`conda activate discoverse`가 안 되면 `conda env list | grep discoverse`로 env 이름이 정확히 `discoverse`인지부터 확인하세요.

---

## 2) (터미널 1) 랜덤 데모 데이터 500개 생성

```bash
cd "$DISCOVERSE_DIR"
python examples/tasks_airbot_play/piper_place_block_random.py --data_set_size 500
```

**팁**  
처음엔 `--data_set_size 10`으로 “정상 생성” 확인 후 500으로 늘리면 실패/시간 낭비가 줄어듭니다.

---

## 3) (터미널 2) LeRobot 환경 진입 + 데이터 변환

새 터미널(터미널 2)에서:

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate lerobot
cd "$LEROBOT_WS_DIR"
```

변환 스크립트 실행:

```bash
python scripts/convert_discoverse_piper_to_lerobot_random500.py
```

**팁**  
변환 스크립트가 DISCOVERSE 데이터 경로를 하드코딩했다면, 스크립트가 `DISCOVERSE_DIR` 환경변수를 읽도록 바꾸면 다른 사람도 그대로 재현 가능합니다.

---

## 4) (터미널 A) LeRobot Policy Server 실행 (ZMQ / 포트 5555)

새 터미널 A에서:

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate lerobot
cd "$LEROBOT_WS_DIR"
```

모델 경로 지정(너의 로컬 폴더 기준):

```bash
export LOCAL_REPO_DIR="$LEROBOT_WS_DIR/hf_models/piper_diffusion_random500_no_occulusion_bs8_ckpt25k_50k"
export MODEL_DIR="$LOCAL_REPO_DIR/pretrained_model"
```

서버 실행:

```bash
python policy_server_zmq_task.py \
  --model_path "$MODEL_DIR" \
  --bind tcp://127.0.0.1:5555 \
  --device cuda
```

(옵션) CPU로 실행하려면:

```bash
python policy_server_zmq_task.py \
  --model_path "$MODEL_DIR" \
  --bind tcp://127.0.0.1:5555 \
  --device cpu
```

**팁**  
서버가 정상 실행 중인지 확인하려면 다른 터미널에서 `lsof -i :5555`를 쳐서 포트 점유를 확인하면 가장 빠릅니다.

---

## 5) (터미널 B) DISCOVERSE Rollout Eval 실행 (서버 연결)

새 터미널 B에서:

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate discoverse
cd "$DISCOVERSE_DIR"
```

### 5-1) JPEG 전송 eval

```bash
python examples/tasks_airbot_play/piper_place_block_random_eval.py \
  --num_eval_episodes 10 --max_steps 350 \
  --policy_server tcp://127.0.0.1:5555 \
  --demo_like --demo_action_repeat 3 \
  --demo_hold_wait_s 0.6 --demo_grip_close_thresh 0.016 --demo_move_speed 0.75 \
  --print_every 10 \
  --image_transport jpeg
```

### 5-2) RAW 전송 eval (권장)

```bash
python examples/tasks_airbot_play/piper_place_block_random_eval.py \
  --num_eval_episodes 30 --max_steps 350 \
  --policy_server tcp://127.0.0.1:5555 \
  --demo_like --demo_action_repeat 3 \
  --demo_hold_wait_s 0.6 --demo_grip_close_thresh 0.016 --demo_move_speed 0.75 \
  --print_every 10 \
  --image_transport raw
```

**팁**  
서버가 떠 있는데도 eval이 연결 실패하면, (1) 서버 bind 주소/포트 (2) eval의 `--policy_server` 주소가 완전히 동일한지부터 확인하세요.

---

## 6) 트러블슈팅 (필수 체크 3개)

### 6-1) 서버 연결 실패(Connection refused)
```bash
lsof -i :5555
```

**팁**  
`5555` 포트를 다른 프로세스가 이미 쓰고 있으면 서버/클라 둘 중 하나가 뜨지 못합니다.

### 6-2) conda env 이름 확인
```bash
conda env list
```

**팁**  
env 이름이 `discoverse`, `lerobot`이 아닐 수 있어요. 실제 이름에 맞춰 `conda activate <이름>`으로 바꾸면 됩니다.

### 6-3) MuJoCo/렌더링 오류
- 그래픽 드라이버/GL/헤드리스 설정 문제일 가능성이 큼  
- 가능하면 headless 옵션/가상 디스플레이(Xvfb) 등을 검토

**팁**  
렌더링 오류는 코드 문제가 아니라 “시스템 환경” 문제인 경우가 많아, 같은 코드를 다른 PC에서 실행하면 정상일 수도 있습니다.

---

## 7) 권장 배포 방식 (레포에 모델/데이터는 넣지 않기)

- `DISCOVERSE` 레포에는 **코드/스크립트/env yml**만
- `lerobot_ws`는 별도 레포로 올리되 **hf_models / outputs / datasets / pretrained_model.tar** 같은 대용량은 `.gitignore`
- 모델은 GitHub Releases 또는 HuggingFace Hub로 배포 권장

**팁**  
대용량 모델/데이터를 레포에 그대로 올리면 push가 막히거나 레포가 무거워져서 “클론 후 실행” 경험이 나빠집니다.
