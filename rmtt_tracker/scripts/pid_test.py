#!/usr/bin/env python3

import numpy as np
from simple_pid import PID

# pid setup
pid_x = PID(0.6, 0, 0)
pid_y = PID(0.6, 0, 0)
pid_z = PID(0.6, 0, 0)

print(pid_x)