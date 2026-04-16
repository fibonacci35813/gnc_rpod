# BSK vs C Comparison Report

| Scenario | dock_rate_c | dock_rate_bsk | dv_c (m/s) | dv_bsk (m/s) | delta_dv_pct | flag |
|---|---|---|---|---|---|---|
| nominal | 1.0000 | 1.0 | 1.7900 | 1.7012 | 4.96 | OK |
| off_axis | 1.0000 | 1.0 | 2.0414 | 1.9773 | 3.14 | OK |
| high_vel | 1.0000 | 1.0 | 2.2836 | 2.2883 | 0.21 | OK |
| sensor_dropout | 1.0000 | 1.0 | 1.8646 | 1.7012 | 8.76 | DELTA_HIGH |
| stuck_closed | 1.0000 | 1.0 | 2.4933 | 1.8748 | 24.81 | DELTA_HIGH |
| stuck_open | 0.0000 | 0.0 | 4.3390 | 11.9413 | 175.21 | DELTA_HIGH |
| high_drag | 1.0000 | 1.0 | 1.8290 | 1.7012 | 6.99 | DELTA_HIGH |
| combined_stress | 1.0000 | 1.0 | 2.1345 | 2.0823 | 2.44 | OK |
