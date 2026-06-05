# 🤖 Intelligent Autonomous Rescue Bot

A battery-powered autonomous robot built using Arduino, designed for disaster-response scenarios. The bot navigates unstructured environments using real-time object detection, intelligent path planning, PIR-based human detection, and path tracking — all working together to locate survivors autonomously.

---

## 📌 Features

- **Object Detection** — Detects obstacles in real time and reacts to avoid collisions
- **Intelligent Path Planning** — Computes and adapts routes dynamically based on sensor input
- **Path Tracking** — Follows predefined routes with precision using embedded control logic
- **Human Detection** — Uses a PIR sensor to identify heat signatures and locate survivors
- **Autonomous Navigation** — Operates fully without human input in dynamic environments

---

## 🛠️ Technologies Used

| Category | Tools / Components |
|---|---|
| Microcontroller | Arduino |
| Human Detection | PIR Sensor |
| Obstacle Detection | Ultrasonic / IR Sensors |
| Motor Control | Motor Drivers (L298N) |
| Programming | Embedded C |
| Chassis | Custom Vehicle Frame |
| Power | Battery Pack |

---

## 🔧 Hardware Components

- Arduino Uno / Mega
- PIR Motion Sensor
- Ultrasonic Sensor (HC-SR04) / IR Sensors
- L298N Motor Driver Module
- DC Motors + Wheels
- Chassis Frame
- Battery Pack + Power Regulation Circuit
- Jumper Wires, Breadboard / PCB

---

## ⚙️ How It Works

1. **Startup** — Bot initializes all sensors and loads the predefined path.
2. **Navigation** — Path planning algorithm guides the bot along the route.
3. **Obstacle Avoidance** — Ultrasonic/IR sensors detect objects; bot reroutes in real time.
4. **Human Detection** — PIR sensor continuously scans for heat signatures.
5. **Alert / Stop** — On detecting a survivor, the bot halts and signals the location.



### Setup

1. **Clone the repository**
   ```bash
   git clone https://github.com/djkingsmen/Intelligent-Autonomous-Rescue-Bot.git
   cd intelligent-autonomous-rescue-bot
   ```

2. **Open in Arduino IDE**
   - Open `src/main.ino` in Arduino IDE

3. **Connect Hardware**
   - Wire components as shown in `hardware/circuit_diagram.png`

4. **Upload the Code**
   - Select your board and COM port in Arduino IDE
   - Click **Upload**

5. **Power On**
   - Connect the battery pack and place the bot on the ground
   - The bot will begin autonomous operation immediately

---

## 📸 Demo

> *(Add photos or a video link of the bot in action here)*

---

## 🎯 Use Case

Designed for **search-and-rescue missions** during natural disasters such as earthquakes, floods, or building collapses — where human entry is dangerous. The bot can navigate debris-filled environments and signal the presence of survivors.

---
