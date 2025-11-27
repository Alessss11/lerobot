Guide:

install sdk:
https://github.com/agilexrobotics/piper_sdk

comando per attivare il can: ____________________________________

conda activate can
cd /home/ales/miniconda3/envs/can/lib/python3.12/site-packages/piper_sdk/
bash find_all_can_port.sh
bash can_activate.sh can0 1000000

_______________________________________________
Piper 7dof lerobot:

1. CALIBRAZIONE SO-ARM:

lerobot-calibrate \
  --teleop.type=so101_leader_7dof \
  --teleop.port=/dev/ttyACM0 \
  --teleop.id=leader_7dof
  
2. TELEOPERAZIONE PIPER:

lerobot-teleoperate \
    --robot.type=piper_7dof \
    --robot.port=can0 \
    --robot.id=my_piper \
    --teleop.type=so101_leader_7dof \
    --teleop.port=/dev/ttyACM0 \
    --teleop.id=leader_7dof \
    --robot.cameras='{camera3: {"type":"intelrealsense","serial_number_or_name":"733512070600","width":640,"height":480,"fps":30}, camera1: {"type":"opencv","index_or_path":"/dev/video8","width":640,"height":480,"fps":30}}' \
    --display_data=true
    
3. ASYNC INFERENCE:

TERMINAL 1:
conda activate lerobot
python -m lerobot.async_inference.policy_server \
     --host=127.0.0.1 \
     --port=8080
     
TERMINAL 2:
conda activate lerobot
python -m lerobot.async_inference.robot_client \
    --server_address=127.0.0.1:8080 \
    --robot.type=piper_7dof \
    --robot.port=can0 \
    --robot.id=my_piper \
    --robot.cameras='{camera3: {"type":"intelrealsense","serial_number_or_name":"733512070600","width":640,"height":480,"fps":30}, camera1: {"type":"opencv","index_or_path":"/dev/video8","width":640,"height":480,"fps":30}}' \
    --task="pick up the red apple and place in the green basket" \
    --policy_type=smolvla \
    --pretrained_name_or_path=Faless/smolVLA_3_apples_expo_80k \
    --policy_device=cuda \
    --actions_per_chunk=40 \
    --chunk_size_threshold=0.5 \
    --aggregate_fn_name=weighted_average \
    --debug_visualize_queue_size=True

4. RECORD INFERENCE
lerobot-record  \
    --robot.type=piper_7dof \
    --robot.port=can0 \
    --robot.id=my_piper \
    --teleop.type=so101_leader_7dof \
    --teleop.port=/dev/ttyACM0 \
    --teleop.id=leader_7dof \
    --robot.cameras='{camera3: {"type":"intelrealsense","serial_number_or_name":"733512070600","width":640,"height":480,"fps":30}, camera1: {"type":"opencv","index_or_path":"/dev/video8","width":640,"height":480,"fps":30}}' \
    --dataset.single_task="pick up the red apple and place in the green basket" \
    --dataset.repo_id=Faless/eval_smolvla \
    --dataset.episode_time_s=120 \
    --dataset.num_episodes=10 \
    --policy.path=Faless/smolVLA_3_apples_expo_80k \
    --display_data=true \
    --policy.n_action_steps=25 \
    --policy.chunk_size=50

5. RECORD DATASET
lerobot-record  \
    --robot.type=piper_7dof \
    --robot.port=can0 \
    --robot.id=my_piper \
    --teleop.type=so101_leader_7dof \
    --teleop.port=/dev/ttyACM0 \
    --teleop.id=leader_7dof \
    --robot.cameras='{camera3: {"type":"intelrealsense","serial_number_or_name":"733512070600","width":640,"height":480,"fps":30}, camera1: {"type":"opencv","index_or_path":"/dev/video8","width":640,"height":480,"fps":30}}' \
    --dataset.single_task="pick up the red apple and place in the green basket" \
    --dataset.repo_id=Faless/apples_demo \
    --dataset.num_episodes=10 \
    --display_data=true \
    --resume=true
    
    
_______________________________________________
Piper 7dof lerobot:

1. CALIBRAZIONE SO-ARM:

lerobot-calibrate \
  --teleop.type=so101_leader \
  --teleop.port=/dev/ttyACM0 \
  --teleop.id=leader_6dof
  
2. TELEOPERAZIONE PIPER:

lerobot-teleoperate \
    --robot.type=piper \
    --robot.port=can0 \
    --robot.id=my_piper \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyACM0 \
    --teleop.id=leader_6dof \
    --robot.cameras='{camera3: {"type":"intelrealsense","serial_number_or_name":"733512070600","width":640,"height":480,"fps":30}, camera1: {"type":"opencv","index_or_path":"/dev/video8","width":640,"height":480,"fps":30}}' \
    --display_data=true
