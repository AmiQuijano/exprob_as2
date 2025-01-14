Number of literals: 9
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
All the ground actions in this problem are compression-safe
Initial heuristic = 7.000
b (6.000 | 60.000)b (5.000 | 90.001)b (4.000 | 150.001)b (3.000 | 240.002)b (2.000 | 300.002)b (1.000 | 390.003);;;; Solution Found
; States evaluated: 17
; Cost: 450.003
; Time 0.00
0.000: (sleep turtle wp0)  [60.000]
0.001: (goto_waypoint turtle wp0 wp1)  [60.000]
60.001: (inspect turtle wp1)  [30.000]
90.001: (goto_waypoint turtle wp1 wp0)  [60.000]
150.002: (goto_waypoint turtle wp0 wp2)  [60.000]
210.002: (inspect turtle wp2)  [30.000]
240.002: (goto_waypoint turtle wp2 wp0)  [60.000]
300.003: (goto_waypoint turtle wp0 wp3)  [60.000]
360.003: (inspect turtle wp3)  [30.000]
390.003: (goto_waypoint turtle wp3 wp0)  [60.000]
