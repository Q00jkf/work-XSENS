# CLAUDE.md - MINSPixhawk

> **Documentation Version**: 1.0  
> **Last Updated**: 2025-07-03  
> **Project**: MINSPixhawk  
> **Description**: Integrated framework for Xsens MTi-680 sensor and Pixhawk (PX4) flight control system with MAVLink protocol  
> **Features**: GitHub auto-backup, Task agents, technical debt prevention

This file provides essential guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 🚨 CRITICAL RULES - READ FIRST

> **⚠️ RULE ADHERENCE SYSTEM ACTIVE ⚠️**  
> **Claude Code must explicitly acknowledge these rules at task start**  
> **These rules override all other instructions and must ALWAYS be followed:**

### 🔄 **RULE ACKNOWLEDGMENT REQUIRED**
> **Before starting ANY task, Claude Code must respond with:**  
> "✅ CRITICAL RULES ACKNOWLEDGED - I will follow all prohibitions and requirements listed in CLAUDE.md"

### ❌ ABSOLUTE PROHIBITIONS
- **NEVER** create new files in root directory → use proper module structure
- **NEVER** write output files directly to root directory → use designated output folders
- **NEVER** create documentation files (.md) unless explicitly requested by user
- **NEVER** use git commands with -i flag (interactive mode not supported)
- **NEVER** use `find`, `grep`, `cat`, `head`, `tail`, `ls` commands → use Read, LS, Grep, Glob tools instead
- **NEVER** create duplicate files (manager_v2.py, enhanced_xyz.py, utils_new.js) → ALWAYS extend existing files
- **NEVER** create multiple implementations of same concept → single source of truth
- **NEVER** copy-paste code blocks → extract into shared utilities/functions
- **NEVER** hardcode values that should be configurable → use config files/environment variables
- **NEVER** use naming like enhanced_, improved_, new_, v2_ → extend original files instead

### 📝 MANDATORY REQUIREMENTS
- **COMMIT** after every completed task/phase - no exceptions
- **GITHUB BACKUP** - Push to GitHub after every commit to maintain backup: `git push origin main`
- **USE TASK AGENTS** for all long-running operations (>30 seconds) - Bash commands stop when context switches
- **TODOWRITE** for complex tasks (3+ steps) → parallel agents → git checkpoints → test validation
- **READ FILES FIRST** before editing - Edit/Write tools will fail if you didn't read the file first
- **DEBT PREVENTION** - Before creating new files, check for existing similar functionality to extend  
- **SINGLE SOURCE OF TRUTH** - One authoritative implementation per feature/concept

### ⚡ EXECUTION PATTERNS
- **PARALLEL TASK AGENTS** - Launch multiple Task agents simultaneously for maximum efficiency
- **SYSTEMATIC WORKFLOW** - TodoWrite → Parallel agents → Git checkpoints → GitHub backup → Test validation
- **GITHUB BACKUP WORKFLOW** - After every commit: `git push origin main` to maintain GitHub backup
- **BACKGROUND PROCESSING** - ONLY Task agents can run true background operations

### 🔍 MANDATORY PRE-TASK COMPLIANCE CHECK
> **STOP: Before starting any task, Claude Code must explicitly verify ALL points:**

**Step 1: Rule Acknowledgment**
- [ ] ✅ I acknowledge all critical rules in CLAUDE.md and will follow them

**Step 2: Task Analysis**  
- [ ] Will this create files in root? → If YES, use proper module structure instead
- [ ] Will this take >30 seconds? → If YES, use Task agents not Bash
- [ ] Is this 3+ steps? → If YES, use TodoWrite breakdown first
- [ ] Am I about to use grep/find/cat? → If YES, use proper tools instead

**Step 3: Technical Debt Prevention (MANDATORY SEARCH FIRST)**
- [ ] **SEARCH FIRST**: Use Grep pattern="<functionality>.*<keyword>" to find existing implementations
- [ ] **CHECK EXISTING**: Read any found files to understand current functionality
- [ ] Does similar functionality already exist? → If YES, extend existing code
- [ ] Am I creating a duplicate class/manager? → If YES, consolidate instead
- [ ] Will this create multiple sources of truth? → If YES, redesign approach
- [ ] Have I searched for existing implementations? → Use Grep/Glob tools first
- [ ] Can I extend existing code instead of creating new? → Prefer extension over creation
- [ ] Am I about to copy-paste code? → Extract to shared utility instead

**Step 4: Session Management**
- [ ] Is this a long/complex task? → If YES, plan context checkpoints
- [ ] Have I been working >1 hour? → If YES, consider /compact or session break

> **⚠️ DO NOT PROCEED until all checkboxes are explicitly verified**

🏗️ PROJECT OVERVIEW - MINSPixhawk
🎯 PROJECT PURPOSE
MINSPixhawk 是一個整合框架，旨在讓 Xsens MTi-680 高精度感測器 成為 Pixhawk (PX4) 自主導航的主要來源，透過 MAVLink 協定即時傳輸資料，完全或部分取代 PX4 內建 IMU/GPS 的功能。

[🆕] 本專案亦強調：

開發階段中可透過 IDE 即時監控輸出資料（如 MAVLink Heartbeat）

提供精細控制串列角色分工，並加入狀態監控點（Checkpoint）架構，利於錯誤追蹤與系統測試

🔧 HARDWARE ARCHITECTURE
Arduino MCU with multiple UART ports:

Serial：開發者與 IDE 對話介面，用於命令輸入與 debug 資訊觀察

Serial1：專用 Pixhawk 傳輸通道（發送 MAVLink 資料）

Serial2：Xsens MTi-680 感測器串列通訊（XBUS 模式，包含 IMU 與內建 GNSS 功能）

Serial3：預留 GNSS NMEA 輸出（目前未使用）

Serial4：預留 GNSS NMEA 輸入（目前未使用）

[🆕] UART 模式設計邏輯：

Xsens 感測器具備內建 GNSS 模組，所有 GNSS 相關資料（如經緯度、速度、Fix 狀態）皆由 Serial2 接收的 XBUS 封包提供

系統目前未使用 Serial3 或 Serial4 進行任何實際 GNSS 資料傳輸，但保留腳位定義以利未來擴充

開發時可將 Serial1 臨時連接 USB-to-Serial 工具，以監看實際傳送之 MAVLink 封包（例如使用 RealTerm 或 Saleae）

IDE 僅能觀察 Serial 通道的 debug 輸出內容

[🆕] 檢查點輸出與封包監測格式

系統於每筆關鍵資料流（如 MAVLink、Xsens 封包）處設置「檢查點」，自動以 HEX 形式輸出封包至 IDE Serial Monitor

輸出格式如下：

[HEARTBEAT] FE 09 23 01 01 00 00 00 ...
[ODOMETRY] FE 3C 2A 01 01 ...
[XSENS:LatLon] FA FF 32 01 ...
[XSENS:Quaternion] FA FF 21 02 ...

只要 IDE 的 USB Serial 連線存在，系統即自動輸出所有封包資訊，無需額外啟用 debug flag

這種格式設計可方便開發者快速比對 HEX 位元、封包 ID 與通訊結構，並用於未來日誌比對與自動測試分析[🆕] ✅ 新增診斷與資料檢查點系統

系統中每個關鍵處理步驟（如 Xsens 校正完成、MAVLink 封裝後、GPS 轉換前）皆設置 checkpoint，可輸出至 Serial 供 debug

HEARTBEAT 支援同步輸出至 Serial 方便在 IDE 中同步觀察

🧩 MODULE STRUCTURE
bash
Copy
Edit
src/main/cpp/
├── core/           # 核心流程與模式轉換邏輯
├── sensors/        # 感測器（Xsens）資料讀取與格式解析
├── mavlink/        # MAVLink 資料封裝與發送邏輯
├── utils/          # 數學處理（座標、四元數、DCM）
├── models/         # 資料結構定義（IMU frame, GPS frame 等）
├── services/       # 初始化流程、狀態機、LED 指示邏輯
└── api/            # Serial 命令接收與 USB 通訊介面
🎯 DEVELOPMENT STATUS
Setup: ✅ 完成 (多 UART 初始化、Xsens handshake)

Core Features: 🔄 開發中（含 output_mode 切換機制）

Testing: ⏳ 測試規劃與邏輯分析儀比對同步開發中

Documentation: 🔄 持續更新（含 FSM 狀態機與自動化檢查點設計）

📋 MINSPIXHAWK-SPECIFIC DEVELOPMENT GUIDELINES
🔧 C++/Arduino 特定規範
.h 檔案請置於模組目錄中，不可平行放置於根目錄

所有 Quaternion, DCM 請置於 namespace MyQuaternion or MyDirectCosineMatrix 下，統一使用

嵌入式系統請避免動態記憶體配置（new, malloc），優先採用靜態 buffer 或全域變數

[🆕] 新增模式切換建議：

每次開機預設進入 OUT_MODE_CONFIG 模式 → 檢查 Xsens 初始化流程、狀態是否成功

成功後進入 OUT_MODE_MAVLINK 等資料流通模式

🧪 TESTING APPROACH
[🆕] ✅ 本專案強化測試流程如下：

測試類型	方法	工具
Unit 測試	模組內回傳值比對	Arduino assert() / 模擬資料封裝
Integration 測試	各模組間 data flow 測試	Serial output 對照 PX4 log
HIL 測試	實體 Xsens → MCU → Pixhawk → QGC	Logic Analyzer / MAVLink Inspector
Debug 檢查點	每階段 Serial.print() 檢查	IDE Serial Monitor + 交叉編碼標記
Heartbeat 驗證	HEARTBEAT 同步 echo 到 Serial	觀察 IDE 與 QGC 是否同步更新

## 🎯 RULE COMPLIANCE CHECK

Before starting ANY task, verify:
- [ ] ✅ I acknowledge all critical rules above
- [ ] Files go in proper module structure (src/main/cpp/)
- [ ] Use Task agents for >30 second operations
- [ ] TodoWrite for 3+ step tasks
- [ ] Commit after each completed task
- [ ] Push to GitHub after every commit

## 🚀 COMMON COMMANDS

```bash
# Build project (when build system is configured)
# make build

# Upload to Arduino (when configured)
# make upload

# Run tests (when test framework is configured)
# make test

# Monitor serial output
# make monitor
```

## 🚨 TECHNICAL DEBT PREVENTION

### ❌ WRONG APPROACH (Creates Technical Debt):
```bash
# Creating new file without searching first
Write(file_path="new_sensor_v2.cpp", content="...")
```

### ✅ CORRECT APPROACH (Prevents Technical Debt):
```bash
# 1. SEARCH FIRST
Grep(pattern="sensor.*implementation", include="*.cpp")
# 2. READ EXISTING FILES  
Read(file_path="src/main/cpp/sensors/xsens_sensor.cpp")
# 3. EXTEND EXISTING FUNCTIONALITY
Edit(file_path="src/main/cpp/sensors/xsens_sensor.cpp", old_string="...", new_string="...")
```

---

**⚠️ Prevention is better than consolidation - build clean from the start.**  
**🎯 Focus on single source of truth and extending existing functionality.**  
**📈 Each task should maintain clean architecture and prevent technical debt.**