# DISCOVERSE Piper Block Place + LeRobot ZMQ Policy Server 전체 실행 가이드
---





## 1) (터미널 1) DISCOVERSE 환경 진입

```bash
cd "$DISCOVERSE_DIR"
source ~/miniconda3/etc/profile.d/conda.sh
conda activate discoverse
```


---

## 2) 랜덤 데모 데이터 500개 생성

```bash
cd "$DISCOVERSE_DIR"
python examples/tasks_airbot_play/piper_place_block_random.py --data_set_size 500
```



---

## 3) LeRobot 환경 진입 + 데이터 변환

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




---

## 5) (터미널 B) DISCOVERSE Rollout Eval 실행 (서버 연결)

새 터미널 B에서:

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate discoverse
cd "$DISCOVERSE_DIR"
```


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


