

# DISCOVERSE Piper Block Place (Random) — Rollout Client

DISCOVERSE 시뮬레이션에서 **Piper로 block place** 작업을 수행.
데이터 생성(랜덤 500개) 및 **LeRobot ZMQ Policy Server**에 연결해 **Rollout eval**까지 수행할 수 있음.

---

## TL;DR (가장 빠른 실행)

```bash
export DISCOVERSE_DIR=~/robotics/discoverse-piper-place
export LEROBOT_WS_DIR=~/lerobot_ws

cd "$DISCOVERSE_DIR"
source ~/miniconda3/etc/profile.d/conda.sh
conda env create -f envs/discoverse.yml -n discoverse
conda activate discoverse

# (서버는 다른 터미널에서 lerobot_ws 쪽 README대로 먼저 실행)
python examples/tasks_airbot_play/piper_place_block_random_eval.py \
  --num_eval_episodes 30 --max_steps 350 \
  --policy_server tcp://127.0.0.1:5555 \
  --demo_like --demo_action_repeat 3 \
  --demo_hold_wait_s 0.6 --demo_grip_close_thresh 0.016 --demo_move_speed 0.75 \
  --print_every 10 \
  --image_transport raw
```



---

## 0) 사전 준비

- Ubuntu + Miniconda/Anaconda
- MuJoCo 설치/동작 확인
- Conda env: `discoverse`


---

## 1) 레포 클론 및 경로 설정

```bash
export DISCOVERSE_DIR=~/robotics/discoverse-piper-place
cd "$DISCOVERSE_DIR"
```


---

## 2) DISCOVERSE conda 환경 생성/활성화

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda env create -f envs/discoverse.yml -n discoverse
conda activate discoverse
```



---

## 3) 랜덤 데모 데이터 500개 생성

```bash
cd "$DISCOVERSE_DIR"
python examples/tasks_airbot_play/piper_place_block_random.py --data_set_size 500
```


---

## 4) Rollout Eval (LeRobot Policy Server 필요)

> 이 단계는 **LeRobot 서버가 먼저 떠 있어야** 함
> (LeRobot 서버 실행은 `shinmyungji/lerobot-ws-piper-place` README 참고)

### RAW 이미지 전송

```bash
cd "$DISCOVERSE_DIR"

python examples/tasks_airbot_play/piper_place_block_random_eval.py \
  --num_eval_episodes 30 --max_steps 350 \
  --policy_server tcp://127.0.0.1:5555 \
  --demo_like --demo_action_repeat 3 \
  --demo_hold_wait_s 0.6 --demo_grip_close_thresh 0.016 --demo_move_speed 0.75 \
  --print_every 10 \
  --image_transport raw
```




---
