#!/bin/bash

PYTHONPATH=. python3 tests/test_ioboard.py
PYTHONPATH=. python3 tests/test_servoboard.py
PYTHONPATH=. python3 tests/test_motorboard.py

#python3 -m unittest discover -s tests