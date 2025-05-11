#!/bin/bash

ask_and_run() {
    local cmd="$1"
    echo ""
    read -r -p "Do you want to run: $cmd ? [y/N]: " answer
    if [[ "$answer" =~ ^[Yy]$ ]]; then
        eval "$cmd"
    else
        echo "Skipping"
    fi
}

YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${YELLOW}WARNING: The tests are moving robot actuators to their MIN and MAX positions!${NC}"

ask_and_run "PYTHONPATH=. python3 tests/test_ioboard.py"
ask_and_run "PYTHONPATH=. python3 tests/test_servoboard.py"
ask_and_run "PYTHONPATH=. python3 tests/test_motorboard.py"

#python3 -m unittest tests/test_servoboard.py