# 🚀 CPSC 383 – Assignment 2  
### Multi-Agent Systems, Planning, Re-Planning, and Cooperation  

**Course:** CPSC 383
**Language:** Python 3.13  
**Platform:** AEGIS Simulation Environment  

---

## 🧩 Overview
This project extends the **AEGIS** simulation framework to implement **cooperative intelligent agents** capable of planning, re-planning, and collaborating to save survivors in a dynamic environment.

Each simulation begins with **seven agents**, all running from the same codebase. Agents must reason strategically, manage limited energy, avoid hazards, and work together to **rescue all survivors**.

---

## 🎯 Objectives
Modify the provided AEGIS agent code to achieve the following goals:

1. **Individual Planning** – Agents can independently reach and save survivors.  
2. **Pair Cooperation** – Agents collaborate on multi-agent actions like simultaneous DIG operations.  
3. **Energy Management** – Agents locate and utilize CHARGING grids for survival.  
4. **Task Distribution** – Agents split to handle multiple survivors concurrently.  
5. **Dynamic Re-Planning** – Agents adapt after achieving goals to continue rescuing others.  

---

## ⚙️ Environment Setup

### 📦 Requirements
- Python **3.13**
- AEGIS package (simulation system)
- Optional Electron-based GUI for real-time visualization

### 🔧 Installation
Follow the official AEGIS documentation for setup:

- **API Reference:** [https://aegis-game.github.io/docs/docs/api/](https://aegis-game.github.io/docs/docs/api/)  
- **Installation Guide:** [https://aegis-game.github.io/docs/docs/getting-started/installation/](https://aegis-game.github.io/docs/docs/getting-started/installation/)

### 🧱 Initialize Project
```bash
aegis init --type mas
