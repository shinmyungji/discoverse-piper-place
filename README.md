# DISCOVERSE Piper Block Place + LeRobot ZMQ Policy Server 전체 실행 가이드

---

## 0) 폴더/경로 전제 (권장)

- DISCOVERSE 레포(너가 올린 GitHub): `~/robotics/discoverse-piper-place`
- LeRobot 작업 폴더(로컬): `~/lerobot_ws`

아래 2개를 먼저 환경변수로 고정하면, 절대경로 실수 없이 재현됨

```bash
export DISCOVERSE_DIR=~/robotics/discoverse-piper-place
export LEROBOT_WS_DIR=~/lerobot_ws
```


---

## 1) (터미널 1) DISCOVERSE 환경 진입

```bash
cd "$DISCOVERSE_DIR"
source ~/miniconda3/etc/profile.d/conda.sh
conda activate discoverse
```


---

## 2) (터미널 1) 랜덤 데모 데이터 500개 생성

```bash
cd "$DISCOVERSE_DIR"
python examples/tasks_airbot_play/piper_place_block_random.py --data_set_size 500
```



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



---

## 4) (터미널 A) LeRobot Policy Server 실행 (ZMQ / 포트 5555)

새 터미널 A에서:

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate lerobot
cd "$LEROBOT_WS_DIR"
```

모델 경로 지정(로컬 폴더 기준):

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



---

## 5) (터미널 B) DISCOVERSE Rollout Eval 실행 (서버 연결)

새 터미널 B에서:

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate discoverse
cd "$DISCOVERSE_DIR"
```

-
```

###  RAW 전송 eval

```bash
python examples/tasks_airbot_play/piper_place_block_random_eval.py \
  --num_eval_episodes 30 --max_steps 350 \
  --policy_server tcp://127.0.0.1:5555 \
  --demo_like --demo_action_repeat 3 \
  --demo_hold_wait_s 0.6 --demo_grip_close_thresh 0.016 --demo_move_speed 0.75 \
  --print_every 10 \
  --image_transport raw
```


---




대용량 모델/데이터를 레포에 그대로 올리면 push가 막히거나 레포가 무거워져서 “클론 후 실행” 경험이 나빠집니다.
