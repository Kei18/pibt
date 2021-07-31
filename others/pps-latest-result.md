Parallel Push & Swap (PPS) results with v1.3 implementation
===
- Because the PPS implementation (v1.0-v1.2) includes bugs, I retried PPS experiments using v1.3. This file shows the results.
- As a brief conclusion, the results are not so different from the PIBT paper but the runtime is much faster due to the change of the underlying A* search. Note that PIBT runtime is also much improved.
- If you are interested in the PIBT algorithm, please check [a new implementation](https://kei18.github.io/mapf-IR) instead of this repo.
- I still do not confirm the bug at least in the following instances, but if you find something wrong, let me know. I will address the problem depending on the situation. But remember, I have not used this repo for a long time.

## 8x8
- timesteplimit=500
- random starts/goals, 100 instances, seed: 0-99

| agents |  success rate (%) | path (ave.) | runtime (ms) |
| ---:  |  ---: | ---: | ---: |
| 10 | 100 |  8.0 |  0 |
| 15 | 100 | 10.5 |  0 |
| 20 | 100 | 13.7 |  1 |
| 25 | 100 | 17.5 |  2 |
| 30 |  98 | 21.7 |  3 |
| 40 |  98 | 36.4 | 12 |
| 50 |  92 | 67.2 | 43 |

## lak105d
- timesteplimit=1000
- random starts/goals, 100 instances, seed: 0-99

| agents |  success rate (%) | path (ave.) | runtime (ms) |
| ---:  |  ---: | ---: | ---: |
|  10 | 100 | 21.4 |   0 |
|  25 | 100 | 27.2 |   3 |
|  50 | 100 | 39.8 |  16 |
|  75 | 100 | 56.8 |  57 |
| 100 |  98 | 79.6 | 131 |

## arena
- timesteplimit=1000
- random starts/goals, 100 instances, seed: 0-99

| agents |  success rate (%) | path (ave.) | runtime (ms) |
| ---:  |  ---: | ---: | ---: |
|  10 | 100 |  32.6 |    1 |
|  25 | 100 |  33.9 |    3 |
|  50 | 100 |  36.5 |    8 |
| 100 | 100 |  41.3 |   24 |
| 200 | 100 |  52.7 |   92 |
| 300 | 100 |  66.3 |  243 |
| 400 |  99 |  81.8 |  548 |
| 500 |  98 | 100.9 | 1100 |

## ost003d
- timesteplimit=1500
- random starts/goals, 100 instances, seed: 0-99

| agents |  success rate (%) | path (ave.) | runtime (ms) |
| ---:  |  ---: | ---: | ---: |
|  10 | 100 | 164.2 |    35 |
|  25 | 100 | 172.4 |   106 |
|  50 | 100 | 179.0 |   268 |
| 100 | 100 | 193.9 |   758 |
| 200 | 100 | 223.0 |  2512 |
| 300 | 100 | 248.3 |  5074 |
| 400 |  99 | 279.5 | 10159 |
| 500 | 100 | 312.1 | 16108 |

## random-32-32-20
- timesteplimit=1000
- MAPF benchmark, random-scenario: 1-25

| agents |  success rate (%) | path (ave.) | runtime (ms) |
| ---:  |  ---: | ---: | ---: |
|  10 | 100 |  24.2 |    1 |
|  25 | 100 |  25.9 |    2 |
|  50 | 100 |  31.4 |    9 |
|  75 | 100 |  37.7 |   18 |
| 100 | 100 |  44.0 |   33 |
| 200 |  92 |  88.3 |  271 |
| 300 | 100 | 183.6 | 1819 |
| 400 |  76 | 388.7 | 7943 |

## warehouse-10-20-10-2-1
- timesteplimit=1000
- MAPF benchmark, random-scenario: 1-25

| agents |  success rate (%) | path (ave.) | runtime (ms) |
| ---:  |  ---: | ---: | ---: |
|  10 | 100 |  68.1 |    1 |
|  25 | 100 |  86.3 |    8 |
|  50 | 100 |  99.5 |   24 |
|  75 | 100 | 107.4 |   40 |
| 100 | 100 | 111.7 |   65 |
| 200 | 100 | 123.0 |  277 |
| 300 | 100 | 140.9 |  752 |
| 400 | 100 | 169.5 | 1774 |
| 500 | 100 | 199.2 | 3619 |
