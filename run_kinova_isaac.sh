# Run Isaac
#!/usr/bin/env bash
readonly ISAAC_PYTHON="${HOME}/isaacsim/python.sh"
readonly SIMULATION_SCRIPT="${HOME}/Projects/kinova_isaacsim/scripts/run_simulation_obj.py"

# Full candidate pool; this script randomly selects 3 distinct objects from
# here on every run instead of spawning the whole list.
readonly OBJECTS=(
  # apple
  # banana
  # foam_brick
  # cracker_box
  # hammer
  # medium_clamp
  rubiks_cube
  mustard_bottle
  # pear
  tuna_fish_can
  mug
  knife
  spatula
  power_drill
  phillips_screwdriver
  scissors
  strawberry
  tennis_ball
  # tomato_soup_can
  # sugar_box
  # adjustable_wrench
)

if [[ ! -x "${ISAAC_PYTHON}" ]]; then
  echo "Isaac Sim Python launcher is not executable: ${ISAAC_PYTHON}" >&2
  exit 1
fi
if [[ ! -f "${SIMULATION_SCRIPT}" ]]; then
  echo "Simulation script does not exist: ${SIMULATION_SCRIPT}" >&2
  exit 1
fi

"${ISAAC_PYTHON}" "${SIMULATION_SCRIPT}" \
  --number 2 \
  --objects "${OBJECTS[@]}" \
  --min-spacing 0.02 \
  --x-range -0.15 0.15 \
  --y-range -0.1 0.1 \
  --control-mode pd  
