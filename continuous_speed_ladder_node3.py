#!/usr/bin/env python3
"""Run the continuous speed ladder on node 3 while node 2 stays disabled."""

import continuous_speed_ladder as ladder

ladder.NODE_ID = 3
ladder.OTHER_ID = 2

raise SystemExit(ladder.main())
