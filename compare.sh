# !/bin/bash
if [ "$#" -ne 2 ]; then
  echo "Usage: $0 <sequence_number> <lambda>"
  echo "Example: $0 02 01"
  exit 1
fi

SEQUENCE=$1
LAMBDA=$2

STD3_PATH="saved_path/std3/std3_${SEQUENCE}_lambda${LAMBDA}_path.tum"
DDPG_PATH="saved_path/ddpg/ddpg_${SEQUENCE}_path.tum"
PPO_PATH="saved_path/ppo/ppo_${SEQUENCE}_path.tum"
REF_PATH="saved_path/ref/ref_${SEQUENCE}_path.tum"
GT_PATH="saved_path/gt/${SEQUENCE}_navsat_path.tum"


if [ ! -f "$STD3_PATH" ]; then
  echo "Error: File $STD3_PATH does not exist."
  exit 1
fi

if [ ! -f "$DDPG_PATH" ]; then
  echo "Error: File $DDPG_PATH does not exist."
  exit 1
fi

if [ ! -f "$REF_PATH" ]; then
  echo "Error: File $REF_PATH does not exist."
  exit 1
fi

if [ ! -f "$GT_PATH" ]; then
  echo "Error: File $GT_PATH does not exist."
  exit 1
fi

echo "Running: evo_ape tum $STD3_PATH $GT_PATH"
evo_ape tum "$STD3_PATH" "$GT_PATH"

if [ $? -ne 0 ]; then
  echo "Error: evo_ape failed for $STD3_PATH and $GT_PATH"
  exit 1
fi

echo "Running: evo_ape tum $DDPG_PATH $GT_PATH"
evo_ape tum "$DDPG_PATH" "$GT_PATH"

if [ $? -ne 0 ]; then
  echo "Error: evo_ape failed for $DDPG_PATH and $GT_PATH"
  exit 1
fi

echo "Running: evo_ape tum $PPO_PATH $GT_PATH"
evo_ape tum "$PPO_PATH" "$GT_PATH"

if [ $? -ne 0 ]; then
  echo "Error: evo_ape failed for $PPO_PATH and $GT_PATH"
  exit 1
fi

echo "Running: evo_ape tum $REF_PATH $GT_PATH"
evo_ape tum "$REF_PATH" "$GT_PATH"

if [ $? -ne 0 ]; then
  echo "Error: evo_ape failed for $REF_PATH and $GT_PATH"
  exit 1
fi

echo "evo_ape commands executed successfully."

