# Heisenberg — Micromouse v2 (Revived)

> Second-generation micromouse robot — originally built under SAE from
> scratch as a self-taught novice, entered at Technoxian and lost in
> the final round, then revived 4 months later with encoders as a
> deliberate hedge against a real-world failure mode discovered in
> [Pascal](https://github.com/Drakren/Pascal): inconsistent wall paint
> on competition mazes defeating IR-based wall detection. Placed 2nd
> at APOGEE '26, one spot behind Pascal's 1st.

![Platform](https://img.shields.io/badge/platform-AVR-blue)
![Language](https://img.shields.io/badge/language-C%2FC%2B%2B-orange)

<p align="center">
  <img src="docs/media/HeisenbergDemo.gif" width="500" alt="Heisenberg navigating a maze"/>
</p>

> **From inexperienced novice build to team podium finish.** Heisenberg began as my first robotics project I built as a complete beginner to programming and hardware, entered at Technoxian. It was later revived during my time as team lead at the SAE Club, UIET (Panjab University), and re-entered at APOGEE '26 alongside [Pascal](https://github.com/Drakren/Pascal) — see [Why Heisenberg Came Back](#why-heisenberg-came-back) below. The GIF above shows Heisenberg live-navigating a maze during testing.

## Table of Contents
- [Overview](#overview)
- [Why Heisenberg Came Back](#why-heisenberg-came-back)
- [System Architecture](#system-architecture)
- [Hardware](#hardware)
- [How It Works](#how-it-works)
- [Results](#results)
- [Getting Started](#getting-started)
- [Repository Structure](#repository-structure)
- [Heisenberg vs. Pascal](#heisenberg-vs-pascal)
- [Acknowledgements](#acknowledgements)

## Overview
**Origin:** Heisenberg started as my first serious robotics build —
designed and built under SAE, as a complete novice to both programming and
hardware. We worked on it for a month and entered it at **Technoxian** and lost in the
final round. I spent roughly three months self-teaching maze-solving
algorithms and embedded fundamentals from scratch before the bot was
competition-ready. 

**What that loss taught:** The core flood-fill + DFS logic worked, but
without encoders, real odometry wasn't possible — the bot relied
purely on ultrasonic sensing and IMU heading correction, which wasn't
enough to compete at the top level. That gap stayed unresolved for a
long time.

**Why it was revived:** By the time I was appointed team lead and
began building Pascal, we identified a real risk specific to Indian
competition circuits: organizers sometimes cut costs by not painting
maze walls consistently, and Pascal's custom IR sensors — precise as
they were — depend on consistent surface reflectivity to work
reliably. Inconsistent walls could throw IR-based wall detection off
entirely.

**The fix:** Rather than risk the whole competition on one sensing
approach, I upgraded Heisenberg with encoders — finally closing the
gap from Technoxian — and re-entered it at APOGEE '26 as a second,
independent entry that didn't share Pascal's IR-dependency failure
mode.

**Outcome:** Pascal placed 1st. Heisenberg placed 2nd. The same
project that lost in a final round few months earlier, as my first attempt
at robotics, ended up taking a podium spot alongside the robot I
designed as team lead.

## Why Heisenberg Came Back
This is the part of the story that's easy to miss just from the code:
- **Technoxian (early build):** First project, no encoders, ultrasonic
  and IMU only — my first real robotics build, done from a standing
  start with no prior programming or hardware experience. Lost in the
  final round.
- **Between Technoxian and APOGEE '26:** I was appointed team lead and
  built Pascal. During that process, we identified the wall-paint
  inconsistency issue as a real risk to Pascal's IR-based approach.
- **The revival:** Heisenberg was upgraded with encoders — giving it
  independent, IR-free odometry for the first time — and entered
  alongside Pascal as a deliberate hedge against a single point of
  failure.
- **Result:** Two structurally different micromice (IR-based Pascal,
  encoder+ultrasonic-based Heisenberg) entered as a hedge — and the
  strategy paid off with a 1st/2nd sweep at APOGEE '26.

*(Full credit for the APOGEE '26 entry goes to my teammates who were
officially registered — see [Acknowledgements](#acknowledgements).)*

## System Architecture
```text
[Ultrasonic Sensors + IMU + Encoders] → [Sensor Fusion / Heading & Odometry Correction]
                                     ↓
                    [Floodfill + DFS Algorithm]
                          ↓                    ↓
              [Wall-Follow Fallback]    [Path Decision Logic]
                                     ↓
                          [Motor Control] → [Motor Drivers]
```
**Stack:** AVR microcontroller, Arduino framework (C/C++, `.ino`).

## Hardware
| Component | Part | Notes |
|---|---|---|
| MCU | Arduino Nano | No NVIC — a key limitation addressed in Pascal |
| IMU | MPU9255 (MPU9250 clone) | Yaw/heading correction |
| Wall Sensors | HC-SR04 (Ultrasonic) | Wide sensing cone, but crucially — reflectivity-independent, unlike IR |
| Motors / Encoders | N20 Geared DC (300 RPM) w/ integrated quadrature encoders | Same motor family as Pascal — added during the pre-APOGEE revival, giving Heisenberg real odometry for the first time, independent of wall-paint consistency |
| Chassis | 16 x 11 cm, double-layer, ~450g | |

## How It Works
The firmware in [`Programs/main/`](Programs/main/) implements the core
maze-solving logic:

- **[`floodfill_DFS.ino`](Programs/main/floodfill_DFS.ino)** — the
  primary pathfinding algorithm, combining flood-fill maze mapping
  with depth-first search traversal.
- **[`left_wall_follow.ino`](Programs/main/left_wall_follow.ino)** /
  **[`right_wall_follow.ino`](Programs/main/right_wall_follow.ino)** —
  wall-following routines *(clarify: fallback strategy, early
  exploration pass, or alternate solving mode? one sentence helps a
  reader know when each kicks in)*.

Supporting test/calibration code lives in
[`Programs/debug/`](Programs/debug/): individual motor tests
(`singleMotorTest.ino`, `bothMotorTest.ino`), sensor validation
(`singleSensorTestUS.ino`, `YawTest.ino`), and IMU/PID integration
testing (`PID_IMU_integration.ino`, `PID.ino`) — the incremental
hardware bring-up work behind the final `main/` logic.

## Results
- **Technoxian (early build):** Lost in the final round — this was
  Heisenberg's first competition entry, before encoders were added.
  Certificate: [`docs/media/CertOfApp.Technoxian.jpeg`](docs/media/CertOfApp.Technoxian.jpeg)
  *(a Certificate of Appreciation, not a placement win — worth
  confirming the exact wording on it so this line is precise)*.
- **APOGEE '26 (revived, with encoders):** Placed **2nd**, with
  teammate bot Pascal placing 1st.
- **Maze:** 16x16 grid, 20x20cm cells.
- **Original proof-of-concept solve time (pre-encoder, Technoxian-era build):** 330 seconds.

## Getting Started
### Prerequisites
```
Arduino IDE
Board: Arduino Nano
```
### Build & Flash
```bash
git clone https://github.com/Drakren/Heisenberg.git
cd Heisenberg/Programs/main
```
Open `floodfill_DFS.ino` in the Arduino IDE, select the correct AVR
board target, and flash.

## Repository Structure
```
.
├── docs/
│   ├── media/
│   │   ├── CertOfApp.Technoxian.jpeg
│   │   ├── PascalAndHeisenberg.jpg
│   │   └── HeisenbergDemo.gif
│   └── Heisenberg.pdf              # (add one line describing this document)
├── Programs/
│   ├── debug/                      # hardware bring-up & calibration tests
│   │   ├── bothMotorTest.ino
│   │   ├── MotorsandIMUtest.ino
│   │   ├── PID_IMU_integration.ino
│   │   ├── PID.ino
│   │   ├── singleMotorTest.ino
│   │   ├── singleSensorTestUS.ino
│   │   └── YawTest.ino
│   └── main/                       # final maze-solving firmware
│       ├── floodfill_DFS.ino
│       ├── left_wall_follow.ino
│       └── right_wall_follow.ino
└── README.md
```

## Heisenberg vs. Pascal
Not a "v2 vs v3" story so much as two intentionally different
approaches entered side by side:

<p align="center">
  <img src="docs/media/PascalAndHeisenberg.jpg" width="450" alt="Heisenberg and Pascal micromice side by side"/>
</p>
<p align="center"><i>Heisenberg and Pascal — the two entries that took 1st and 2nd at APOGEE '26.</i></p>

| | Heisenberg | Pascal |
|---|---|---|
| MCU | Arduino Nano — no NVIC | STM32G431 — FPU, DWT, high-res timers |
| Wall Sensing | HC-SR04 ultrasonic — reflectivity-independent, wider cone | Custom in-house IR — precise, but reflectivity-dependent |
| Odometry | Added late: encoders + IMU | Encoders + ISM330DHCX IMU from the start |
| Chassis | 16 x 11 cm, double-layer, ~450g | 12 x 8 cm, single-layer, ~200g |
| Solve Time | 330 seconds (pre-encoder proof-of-concept) | 55 seconds |
| APOGEE '26 Result | **2nd place** | **1st place** |
| Strategic role | IR-failure hedge — the "just in case" entry | Primary, fastest entry |

Full technical breakdown: [Pascal README](https://github.com/Drakren/Pascal)

## Acknowledgements
Originally built under SAE as my first robotics project, entered at
Technoxian. Revived four months later as team lead of the SAE Club
Micromouse team (**Team Mozzarella**) at UIET, Panjab University.
Officially entered at APOGEE '26 by teammates
[Supriya Bhardwaj](https://github.com/Supriyabhardwaj1),
[Vasu Kambli](https://github.com/VasuKambli),
[Tanish Singla](https://github.com/tanishsingla201-web),
([Aaditi15](https://github.com/Aaditi15)), and
([vikramjeetsingh-art](https://github.com/vikramjeetsingh-art)) —
competition rules didn't allow duplicate entries under one name, so
they were the registered entrants for Heisenberg while I led the
overall team and the encoder-upgrade engineering. Sibling project to
[Pascal](https://github.com/Drakren/Pascal), winner of APOGEE '26.

---
**Author:** Mohammed Talha · [LinkedIn](https://www.linkedin.com/in/talha-mohammed-13-04-eee)
