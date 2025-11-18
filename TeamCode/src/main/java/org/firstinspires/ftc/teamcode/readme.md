# 🧠 KrakenKoders FTC Code Guidelines

Welcome to the KrakenKoders robotics codebase!
This repository contains our official FTC robot code. To keep things organized, stable, and reliable during competitions, we follow a **Pull Request (PR)** workflow.

---

## 🚦 Branch Workflow

We use a simple version of Git branching:

| Branch Name   | Purpose                                                       |
|---------------| ------------------------------------------------------------- |
| `master`      | Stable, competition-ready robot code. Only merged via PR.     |
| `feature/...` | New code, experiments, improvements. One change = one branch. |
| `bugfix/...`  | Fixes to existing systems.                                    |
| `docs/...`    | Updates to documentation only.                                |

### Examples:

```
feature/improved-drive
bugfix/teleop-button-crash
docs/team-readme-update
```

---

## 🛠️ How to Make Changes

1. **Create a branch from `master`:**

```bash
git checkout -b feature/my-change
```

2. **Make and test code changes in Android Studio.**

3. **Commit the work:**

```bash
git add .
git commit -m "Describe what changed"
```

4. **Push to GitHub:**

```bash
git push -u origin feature/my-change
```

---

## 🔁 Pull Request (PR) Process

Once you're done coding and testing:

1. Go to GitHub and open a **Pull Request**.
2. Base branch should be: **`master`**
3. Compare branch: **your feature or bugfix branch**
4. Fill out the PR template (if available) or include:

✔️ What you changed
✔️ Why you changed it
✔️ How it was tested (drive motors? autonomous? sim?)

5. Request a reviewer:

* Programming lead
* Mentor
* Another teammate if you pair-programmed

6. Make any requested fixes.

7. Once approved, the PR can be merged.

---

## 🧹 After Merge

Once a PR is merged into `master`:

* Delete the feature branch (optional but recommended)
* Pull changes to your laptop:

```bash
git pull origin main
```

---

## 📏 Code Rules

* Code must compile before submitting.
* No unused debug logs or commented-out blocks.
* Keep methods short and readable.
* Name motors, sensors, and constants clearly.

---

## 🤝 Expectations

* Mistakes are okay — that’s why PRs exist.
* Always ask if you’re unsure.
* Code reviews are teamwork, not criticism.

---

> 💪 **The goal is stability, learning, teamwork — and robots that WORK on match day!**

---

## 📜 Scripts

We've included helpful batch scripts in the `scripts/` folder to streamline your development workflow:

### 🧰 `setup_adb_path.bat`

**Purpose:** Configures ADB (Android Debug Bridge) in your system PATH so you can use `adb` commands from any terminal.

**When to use:** Run this **once** after installing Android Studio or if `adb` commands aren't working.

**How to use:**
1. Right-click `setup_adb_path.bat` and select 'Run' option or run it from Command Prompt
2. The script will:
   - Locate ADB in your Android SDK installation
   - Add it to your user PATH environment variable
   - Verify that `adb.exe` exists
3. After running, close and reopen any terminal windows for changes to take effect

**Note:** The script looks for ADB at `%LOCALAPPDATA%\Android\Sdk\platform-tools`. If your Android SDK is installed elsewhere, you'll need to edit the script.

### 🤖 `connect_robot.bat`

**Purpose:** Automates the connection process to the robot controller over Wi-Fi and ADB.

**When to use:** Every time you need to connect to the robot for wireless deployment or debugging.

**How to use:**
1. Make sure the Robot Controller is powered on
2. Ensure Wi-Fi debugging is enabled on the Robot Controller
3. Right-click `connect_robot.bat` and select 'Run' option or run it from Command Prompt
4. The script will:
   - Connect to the robot's Wi-Fi network (SSID: `17178-RC`)
   - Wait for the connection to stabilize (~7 seconds)
   - Display current Wi-Fi connection status
   - Connect ADB to the robot at `192.168.43.1:5555`
   - List all connected ADB devices to confirm connection

**Configuration:** If your robot has a different team number or network settings, edit these values at the top of the script:
- `ROBOT_SSID`: Your robot's Wi-Fi network name
- `ROBOT_PROFILE`: The saved Wi-Fi profile name in Windows
- `ADB_IP`: The robot's IP address (usually `192.168.43.1`)
- `ADB_PORT`: The ADB port (usually `5555`)

**Troubleshooting:**
- If ADB doesn't show the RC after running, verify:
  - The Robot Controller is powered on
  - Wi-Fi Direct is enabled on the RC
  - Network debugging is allowed in the RC settings
  - You've run `setup_adb_path.bat` at least once

---

# 🦑 GO KRAKEN! 🦑

If you break the build, you owe the team cookies. 🍪😉

---
