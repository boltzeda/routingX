# Router Benchmark Results

Ran routing for 28 boards using Boltzpcb Autorouter. The manually drawn boards are in `manually_drawn/`, and the router outputs are in `boltzpcb_autorouter/`.

Runs used the router built-in DRC defaults (no `--drc` JSON). Completeness is routed nets divided by total routable nets. Unrouted nets is the router final count of nets with remaining unconnected connections. Time is router-reported elapsed seconds.


| Board | Completeness | Time (seconds) |DRC errors | Unrouted nets |
|---|---:|---:|---:|---:|
| `emberone` | 71.1% | 667 | 474 | 65 |
| `bitaxeUltra` | 88.5%| 512 | 221 | 7 |
| `ARGOS DEMO` | 94.6% | 354 | 61 | 4 |
| `jlcpcb_4L_1.6mm_outer1oz_inner0.5oz_JLC04161H-7628` | 100.0% (no routable nets) | 12 | 0 | 0 |
| `PICO2-BB48_Rev_A` | 20.0% | 570 | 62 | 68 |
| `water_level` | 100.0% | 22 | 0 | 0 |
| `JTAGProgrammer` | 87.3% | 556 | 8 | 8 |
| `cm4-lvds-adapter` | 60.4% | 495 | 177 | 42 |
| `cvbs-mipi-bridge` | 81.0% | 468 | 166 | 12 |
| `dual-ipex-csi-interposer` | 87.8% | 248 | 5 | 5 |
| `ecp5-dc-scm` | 99.3% | 622 | 233 | 3 |
| `gmsl-serializer` | 68.1% | 560 | 126 | 23 |
| `pmod-i3c-sensor-board` | 56.2% | 475 | 128 | 21 |
| `si-simulation-test-board` | 100.0% | 19 | 119 | 0 |
| `stm32h7-hdmi-board-sodimm-module` | 1.0%| 469 | 150 | 95 |
| `tiliqua-motherboard` | 10.8% | 490 | 67 | 174 |
| `tiliqua-panel` | 100.0%| 14 | 6 | 0 |
| `BuildboticsController` | 64.6%| 647 | 157 | 86 |
| `schtest` | 75.0%| 39 | 0 | 2 |
| `schtest2` | 100.0%| 11 | 0 | 0 |
| `hwtest` | 54.7% | 502 | 216 | 73 |
| `ECP5` | 85.9%| 358 | 0 | 11 |
| `praline` | 95.1% | 521 | 463 | 21 |
| `seven` | 71.1% | 566 | 67 | 22 |
| `LoadcellSensor` | 2.3%| 543 | 61 | 42 |
| `kintex-pcie` | 70.8%| 523 | 13 | 52 |
| `trellis_core` | 37.2% | 562 | 145 | 27 |
| `Xerxes` | 23.0% | 534 | 10 | 77 |
