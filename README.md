# kibo-PRC

គម្រោងគ្រប់គ្រង និងបញ្ជា Robot សម្រាប់ការប្រកួតប្រជែង Robot Programming Competition (RPC)។
Robot Control and Autonomous Navigation System for the Robot Programming Competition.

[![GitHub](https://img.shields.io/badge/GitHub-korbsameth-blue?logo=github)](https://github.com/korbsameth)
[![Python](https://img.shields.io/badge/Python-3.10%2B-brightgreen?logo=python)](https://www.python.org/)

---

## សមាជិកក្រុម / Team Members (4 Members)

| No. | ឈ្មោះ (Name) | តួនាទី (Role) | ភារកិច្ចទទួលខុសត្រូវ (Responsibilities) | GitHub / Contact |
|:---:|---|---|---|:---:|
| 1 | Nou Srey Oun (នូ ស្រីអូន) | Team Leader & Project Manager | គ្រប់គ្រងគម្រោងទូទៅ សម្របសម្រួលក្រុម និងយុទ្ធសាស្ត្រប្រកួត | Team Leader |
| 2 | Korb Sameth (កប សាម៉េត) | Lead Programmer (Me) | សរសេរ main.py, រៀបចំ System Architecture និងសម្របសម្រួល Code | [@korbsameth](https://github.com/korbsameth) |
| 3 | លី លីអ៊ីញ (Ly Ly Inh) | Vision & AR Marker Dev | អភិវឌ្ឍន៍ marker_reader.py, ស្កេន និងចាប់ទិន្នន័យពី AR Marker | Member |
| 4 | ប៊ុនណា លីដា (Bunna Lida) | Motion & Hardware Controller | អភិវឌ្ឍន៍ movement.py, បញ្ជា Motor និង route_planner.py | Member |

---

## រចនាសម្ព័ន្ធ Folder / Project Structure

គម្រោងនេះផ្តោតសំខាន់លើ Folder code សម្រាប់ដំណើរការ Robot:

```text
kibo-PRC/
│
├── code/
│   ├── main.py              # កម្មវិធីចម្បង (Main execution script & loop)
│   ├── movement.py          # បញ្ជាចលនា Robot (Motor & movement controls)
│   ├── marker_reader.py     # ស្កេន AR Marker (Camera & AR detection)
│   └── route_planner.py     # រៀបចំផ្លូវ (Path planning & checkpoints)
│
├── .gitignore               # កំណត់ Ignore files (Track code only)
└── README.md                # ឯកសារណែនាំគម្រោង និងសមាជិក (Project & Team documentation)
```

---

## តម្រូវការដំឡើង / Requirements & Installation

1. Python 3.10 ឬខ្ពស់ជាងនេះ (Python 3.10+)
2. ដំឡើង Dependencies (Install Packages):
   ```bash
   pip install opencv-python numpy
   ```

---

## របៀបដំណើរការ / How to Run

ដំណើរការកម្មវិធីបញ្ជា Robot ពី Folder ចម្បង:

```bash
python code/main.py
```

### លំហូរដំណើរការ (Execution Flow):
1. Camera Initialization: បើក Camera និងរៀបចំ System Sensor
2. Path Setup: កំណត់ទីតាំង Checkpoints
3. Movement & Detection: ចល័ត Robot ទៅមុខ និងស្កេន AR Marker
4. Action Execution: បត់ និងបន្តទៅ Checkpoint បន្ទាប់រហូតដល់ចប់បេសកកម្ម

---

## Git & Version Control

- Repository: https://github.com/korbsameth/kibo-PRC
- Lead Programmer: Korb Sameth ([@korbsameth](https://github.com/korbsameth))
- Team Leader: Nou Srey Oun
