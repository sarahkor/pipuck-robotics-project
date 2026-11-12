# 🤖 Getting Started with PiPuck

A practical guide to connecting to, configuring, and working with the **PiPuck (e-puck2)** robots used in our lab.

This guide covers the lab's robot identifiers, power controls, SSH access, Wi-Fi setup, file transfers, running Python programs, sensors, cameras, charging, and troubleshooting.
 
## Table of Contents

1. [Our PiPuck Robots](#1-our-pipuck-robots)
2. [Power, Wi-Fi, and SSH](#2-power-wi-fi-and-ssh)
   - [Powering On and Off](#21-powering-on-and-off-the-pipucks)
   - [Wi-Fi Requirements](#22-before-connecting--wi-fi-requirements)
   - [First-Time SSH Connection](#23-first-time-ssh-connection)
   - [Connecting Again](#24-connecting-again)
   - [Connecting Through VS Code](#25-connecting-through-vs-code)
   - [Disconnecting from SSH](#26-disconnecting-from-ssh)
3. [Adding a Wi-Fi Network](#3-adding-a-wi-fi-network)
4. [Transferring Files](#4-transferring-files)
5. [Running Python Programs](#5-running-python-programs)
6. [Working with Sensors](#6-working-with-sensors)
7. [Cameras and Recording](#7-cameras-and-recording)
8. [Charging and Checking Battery Level](#8-charging-and-checking-battery-level)
9. [Troubleshooting](#9-troubleshooting)
10. [Useful Commands](#10-useful-commands)
11. [Useful Links](#11-useful-links)

---

## 1. Our PiPuck Robots

| Network hostname | Robot number | Equipment notes |
|---|---|---|
| `pipuck0.local` | **5815** | Front and 360° camera |
| `pipuck1.local` | **5563** | Front camera; ground-sensor extension |
| `pipuck2.local` | **5812** | Front and 360° camera |

The SSH username for these robots is `pi`. Replace `pipuck0.local` in the examples with the robot you want to access.

## 2. Power, Wi-Fi, and SSH

SSH lets you access the robot's Linux terminal over the network. First, make sure the robot is powered on and connected to a reachable Wi-Fi network.

### 2.1 Powering On and Off the PiPucks

Before connecting to a PiPuck, make sure it is powered on.

Each PiPuck has two blue buttons:

- **Lower button — `ON/OFF`:** Used to turn the robot on and off.


  <img width="200" alt="PiPuck_lower_button" src="https://github.com/user-attachments/assets/a6d05e53-5a27-42dc-88e5-05cdaf111e36" />


- **Upper button — `auxON`:** Used to turn the Pi-puck extension on and off.

  <img width="200" alt="PiPuck_higher_button" src="https://github.com/user-attachments/assets/f5b4cb59-9e2b-4e17-af37-c904ec6e2dd4" />


#### Turning ON a PiPuck

1. Locate the **lower blue button (`ON/OFF`)** on the robot.
2. Press and hold it for a few seconds until the robot's lights turn on.
3. Allow the robot some time to finish booting before attempting to connect via SSH.

#### Turning OFF a PiPuck

1. Press and hold the **lower blue button (`ON/OFF`)** for a few seconds until the robot's lights turn off.
2. For **pipuck0 and pipuck2** (the robots equipped with 360° cameras), there is an additional step:
   - Press and hold the **upper blue button (`auxON`)** for approximately **2 seconds**, then release it.
   - Wait until all remaining lights turn off.

> **Note:** Before turning off a PiPuck, make sure the robot has stopped moving and any running programs have been terminated safely.

### 2.2 Before Connecting — Wi-Fi Requirements

The PiPuck and your computer must be on **networks that can communicate with each other**. The easiest arrangement is to connect both to the same Wi-Fi network.

#### Working in the Lab or at the University

All three PiPucks already have the **lab and university Wi-Fi networks configured**. At startup, they automatically try to connect to a saved network. You normally **do not need to configure Wi-Fi on every boot**.

The lab Wi-Fi has higher configured priority, but occasionally a PiPuck connects to the university Wi-Fi instead.

> [!TIP]
> **If the robot joins the university Wi-Fi instead of the lab Wi-Fi:** Safely shut it down, turn it on again, allow time for Wi-Fi association, and retry SSH. This often helps, but does not guarantee that the lab Wi-Fi will be selected. If the problem persists, check the actual SSID with `iwgetid -r` over serial or an existing connection.


#### Working from Home or Another Wi-Fi Network

If the robot has not been configured for your home or other Wi-Fi network, follow [Section 3: Adding a Wi-Fi Network](#3-adding-a-wi-fi-network) first. The robot should then attempt to reconnect to that saved network on subsequent boots when available.

#### Checking Connectivity

From your computer's terminal (the next command is for macOS/Linux):

```bash
ping -c 4 pipuck0.local
```

You can then try:

```bash
ssh pi@pipuck0.local
```

A failed `ping` does not always mean SSH is unavailable. Replace the hostname as needed.

### 2.3 First-Time SSH Connection

From **your computer**, run:

```bash
ssh pi@pipuck0.local
```

The first time your computer sees this robot's SSH host key, it may show:

```text
The authenticity of host 'pipuck0.local (192.168.x.x)' can't be established.
ED25519 key fingerprint is SHA256:...
This key is not known by any other names.
Are you sure you want to continue connecting (yes/no/[fingerprint])?
```

This is **normal on the first connection**. SSH has no previously trusted host key for this robot.

#### Verifying the SSH fingerprint using USB (recommended)

If you are confident you are reaching the intended robot on a trusted network, entering `yes` uses SSH's *trust-on-first-use* approach. To verify the robot's identity independently before accepting:

1. Leave the SSH authenticity prompt open on your computer.
2. Plug a **data-capable Micro-USB cable** into the **middle Micro-USB serial port** on the robot and connect it to your computer.
3. Open a **second terminal**. On macOS, identify the serial device:
   ```bash
   ls /dev/cu.*
   ```
   Look for a device such as `/dev/cu.usbserial-10`. Linux and Windows use different device names and serial tools.
4. Connect to the serial console, replacing the device path with yours:
   ```bash
   screen /dev/cu.usbserial-10 115200
   ```
5. Press `Enter` if needed; log in as `pi` using the configured password. On lab robots where it has not been changed, the default is `raspberry`.
6. **On the PiPuck's serial console**, run:
   ```bash
   ssh-keygen -lf /etc/ssh/ssh_host_ed25519_key.pub
   ```
7. Compare the **complete `SHA256:` fingerprint** with the one displayed by your first SSH command. If they match, return to the SSH prompt and type `yes`. If they differ, **do not accept the connection**.
8. To detach from the `screen` session, press `Ctrl+A`, release both keys, then press `D`. Disconnect the USB cable after detaching.

Once accepted, the host key is stored in `~/.ssh/known_hosts` on your computer. Do not blindly accept an unexpected later warning that the host key has changed.

#### Password and first login

If password authentication is enabled, SSH then prompts:

```text
pi@pipuck0.local's password:
```

The **password is `raspberry`**, *only if the password has not been changed*. The terminal does not display characters as you type.

A successful login may look like:

```text
pi@pipuck0.local's password:
Linux pipuck0 5.10.103-v7+ #1529 SMP Tue Mar 8 12:21:37 GMT 2022 armv7l

The programs included with the Debian GNU/Linux system are free software;
the exact distribution terms for each program are described in the
individual files in /usr/share/doc/*/copyright.

Debian GNU/Linux comes with ABSOLUTELY NO WARRANTY, to the extent
permitted by applicable law.
Last login: Mon Nov  9 10:03:07 2026 from ...

SSH is enabled and the default password for the 'pi' user has not been changed.
This is a security risk - please login as the 'pi' user and type 'passwd' to set a new password.

pi@pipuck0:~ $
```

The exact version, last-login address, and welcome text can differ.

### 2.4 Connecting Again

Subsequent connections use the same command:

```bash
ssh pi@pipuck0.local
```

If password authentication is enabled, enter the robot's password. When connected, you see a prompt similar to `pi@pipuck0:~ $`.

Connecting to other robots:

```bash
ssh pi@pipuck1.local
ssh pi@pipuck2.local
```

Run each command separately. Automated scripts such as `start_team_rab.sh` can use `BatchMode=yes`, requiring working **non-interactive SSH key authentication** rather than interactive password prompts.

### 2.5 Connecting Through VS Code

**Integrated terminal:** Open VS Code → **Terminal → New Terminal**, then run the same SSH command as above. Type `exit` when finished.

> [!TIP]
> **SSH works in your computer's terminal but not in VS Code?** On **macOS**, fully close VS Code, then try launching it from the same terminal where SSH works:
>
> ```bash
> "/Applications/Visual Studio Code.app/Contents/MacOS/Code" .
> ```
>
> Run this from the directory you want to open. It can sometimes resolve environment differences, but it is not guaranteed to fix network, SSH configuration, or hostname problems.

### 2.6 Disconnecting from SSH

To close your SSH session, type `exit` **at the robot's prompt**:

```text
pi@pipuck0:~ $ exit
logout
Connection to pipuck0.local closed.
```

You can also press `Ctrl+D`. This closes the SSH connection; **it does not power off the robot**.

---

## 3. Adding a Wi-Fi Network

If you want to use a new Wi-Fi network, configure it through the **USB serial console** (the instructions below apply to our robots' traditional `wpa_supplicant` setup).

### Step 1 — Connect the robot to your computer

1. Make sure the robot is powered on.
2. Connect a **data-capable Micro-USB cable** to the **middle USB port** near the R&B sensors.

   <img width="200" alt="PiPuck_middle_usb" src="https://github.com/user-attachments/assets/501dba55-9545-40e4-be9f-f00563fb2f2b" />

4. Connect the cable to your computer. A charge-only cable will not work for serial communication.

### Step 2 — Identify the serial port

On macOS:

```bash
ls /dev/cu.*
```

Look for `/dev/cu.usbserial-10` or a similar device. Compare the device list before and after connecting the robot if needed.

### Step 3 — Open a serial connection

```bash
screen /dev/cu.usbserial-10 115200
```

Press `Enter` a few times, then log in as `pi`. Use the actual password configured for the robot; the default is `raspberry` if unchanged. The password is not echoed on screen.

### Step 4 — Configure the Wi-Fi network

Open the Wi-Fi configuration:

```bash
sudo nano /etc/wpa_supplicant/wpa_supplicant.conf
```

Append a new block **without deleting the existing configured networks**:

```ini
network={
    ssid="YOUR_HOME_WIFI_NAME"
    psk="YOUR_HOME_WIFI_PASSWORD"
}
```

Replace the placeholders with your network SSID and password. In Nano, press `Ctrl+O` and `Enter` to save, then `Ctrl+X` to exit.

> **Security:** Do not commit real Wi-Fi passwords to GitHub. Keep the configuration file's permissions restricted.

### Step 5 — Apply and verify the Wi-Fi settings

```bash
sudo wpa_cli -i wlan0 reconfigure
ip a
iwgetid -r
wpa_cli -i wlan0 status
```

In `ip a`, look for a `wlan0` IPv4 address such as `192.168.1.105/24`. Use `iwgetid -r` to check the joined network name. An assigned address alone does not prove you joined the intended SSID.

> **Compatibility:** These commands apply to installations managed by `wpa_supplicant`. Newer Raspberry Pi OS setups may instead use NetworkManager, requiring different steps.

### Step 6 — Disconnect from the serial session safely

To **detach** from `screen` without shutting down the robot: press `Ctrl+A`, release, then press `D`.

Other options:

- `Ctrl+A`, then `:`, type `detach`, press `Enter` — detach using the Screen command prompt.
- `Ctrl+A`, then `Ctrl+\`, confirm with `Y` — **terminate** the Screen session (different from detaching).

If you see `No other window`, you may have pressed `Ctrl+A` twice accidentally.

### Step 7 — Disconnect the USB cable

Once detached, unplug the USB cable. The robot should remain on the configured Wi-Fi network when it is available and compatible with the robot's system.

Return to [Section 2: Power, Wi-Fi, and SSH](#2-power-wi-fi-and-ssh).

---

## 4. Transferring Files

You can transfer files using **VS Code SFTP** (convenient during development) or **SCP** from a regular terminal.

### 4.1 VS Code SFTP — Natizyskunk (recommended for editing)

I used the **SFTP extension by Natizyskunk** (`Natizyskunk.sftp`). This extension can automatically upload local changes to the robot when you save a file.

1. Open your **local project folder** in VS Code.
2. Install **[SFTP by Natizyskunk](https://marketplace.visualstudio.com/items?itemName=Natizyskunk.sftp)** from the Extensions view.
3. Open the Command Palette (`Cmd+Shift+P` on macOS, `Ctrl+Shift+P` on Windows/Linux), then run **`SFTP: Config`**. This creates `.vscode/sftp.json` in the local project.
4. Configure the robot's address, login, and remote directory. For example:

```json
{
  "name": "pipuck0",
  "host": "pipuck0.local",
  "protocol": "sftp",
  "port": 22,
  "username": "pi",
  "password": "YOUR_PIPUCK_PASSWORD",
  "remotePath": "/home/pi/pipuck-walk",
  "uploadOnSave": true,
  "useTempFile": true,
  "openSsh": true
}
```

**What to change:**

- `name` and `host`: choose `pipuck0`, `pipuck1`, or `pipuck2`.
- `password`: replace the placeholder with the password configured for that robot, **only in your private local configuration**; the lab's default is `raspberry` if unchanged. Prefer key-based authentication when practical.
- `remotePath`: the remote folder that should receive your local project files. `/home/pi/pipuck-walk` is my mission directory.
- `uploadOnSave: true`: upload supported local file changes automatically when saved.
- `openSsh: true` and `useTempFile: true`: the extension's documented compatible pairing for atomic uploads. **Do not combine `openSsh: true` with `useTempFile: false`**.


#### Check that files actually arrived

After saving a file in VS Code, SSH into the **correct robot** and inspect the destination directory:

```bash
ssh pi@pipuck0.local
cd ~/pipuck-walk
ls -la
cat my_script.py
```

Replace `my_script.py` with your filename. `ls` verifies that it exists; `cat` displays the remote contents so you can compare them with the local version. For longer files, use `less my_script.py` or view a small section with `sed -n '1,40p' my_script.py`.

If saving does not upload the file, check **View → Output → SFTP** in VS Code, ensure the right hostname and `remotePath` are configured, and try an explicit **SFTP: Upload Active File** action.

> **Multiple robots:** Configure each robot separately or use SFTP profiles. Avoid accidentally uploading `rob0` code to `pipuck2`; confirm the active SFTP profile before saving.

### 4.2 Uploading Files via SCP (alternative)

Run on **your computer**, not inside the PiPuck's SSH session:

```bash
ssh pi@pipuck0.local 'mkdir -p ~/pipuck-walk'
scp my_script.py pi@pipuck0.local:/home/pi/pipuck-walk/
```

The first command creates the destination if needed; the second uploads your file. Verify it using `ls` and `cat` on the robot as above.

### 4.3 Downloading Files via SCP

To download a log into your **current local directory**:

```bash
scp pi@pipuck0.local:/home/pi/pipuck-walk/team_mission_rab.log .
```

To download a recordings directory:

```bash
scp -r pi@pipuck0.local:/home/pi/pipuck-walk/recordings ./recordings-pipuck0
```

### 4.4 Working with Multiple Robots

Each PiPuck is a separate Linux computer. Double-check the hostname and the intended local robot folder (`rob0/`, `rob1/`, or `rob2/`) before transferring programs. A file uploaded to one robot is not automatically installed on the others.

---

## 5. Running Python Programs

### 5.1 Setting Up the PiPuck SDK

Before running Python programs that control the PiPuck, make sure the **PiPuck SDK** is installed.

The SDK provides the Python interface for communicating with the robot's hardware, including its motors and sensors.

For example, our Python programs use:

```python
from pipuck import PiPuck
```

#### Step 1 — Check whether the SDK is already installed

After connecting to the robot via SSH, run:

```bash
python3 -c "from pipuck import PiPuck; print('PiPuck SDK is installed!')"
```

If the command succeeds, the SDK is available in the current Python environment, and no additional installation is necessary.

If you receive `ModuleNotFoundError: No module named 'pipuck'`, you need to install the SDK.

#### Step 2 — Installing the SDK using setup.py

Our main_rab_mission includes a `setup.py` file

The following is an example of a setup.py file. Replace author_name with your name when creating your own project:

```python
from setuptools import setup, find_packages

setup(
    name='pi-walk',
    author="author_name",
    version='0.0.1',
    packages=find_packages(),
    install_requires=[
        'pipuck-sdk @ git+https://github.com/OmriPer/pipuck-sdk.git'
    ]
)
```

This file defines the project's installation metadata and declares the **PiPuck SDK** as a dependency. The SDK is obtained from [Omri's GitHub repository](https://github.com/OmriPer/pipuck-sdk).

To install the project and its dependencies, navigate to the directory containing `setup.py` and run:

```bash
python3 -m pip install -e .
```

The `-e` option installs the project in *editable mode*, allowing changes to its Python package source files without reinstalling the package.

Alternatively, if you only need the PiPuck SDK, you can install it directly:

```bash
python3 -m pip install "pipuck-sdk @ git+https://github.com/OmriPer/pipuck-sdk.git"
```

Both installation methods require a compatible Python environment, and downloading the SDK from GitHub requires network access and Git.

#### Step 3 — Working in Different Directories

**The PiPuck SDK does not need to be installed separately for each project directory.**

For example, you might have:

```text
/home/pi/
├── pipuck-walk/
│   ├── setup.py
│   └── team_mission_rab.py
│
└── my-new-project/
    └── test_robot.py
```

If the SDK is already installed in the Python environment used by both programs, you can run:

```bash
cd ~/my-new-project
python3 test_robot.py
```

And import the SDK normally:

```python
from pipuck import PiPuck
```

**Important:** If you switch to a different Python environment, such as a virtual environment, you may need to install the SDK in that environment as well.

Also, installing the SDK does not automatically install every standalone Python file from your robotics project. Other project modules may still need to be copied or installed separately.

### 5.2 Run a Program over SSH

SSH into a robot and navigate to the program directory:

```bash
ssh pi@pipuck0.local
cd ~/pipuck-walk
python3 --version
ls
python3 my_script.py
```

Replace `my_script.py` with the actual program name. It may require installed dependencies such as `pipuck`, `smbus2`, or OpenCV, depending on the program.

**Stop a foreground program:** Press `Ctrl+C`. For motor-control programs, verify that the motors actually stop; safe cleanup must be implemented in the script.

### 5.3 Run and Inspect a Background Program

```bash
nohup python3 -u my_script.py > my_script.log 2>&1 < /dev/null &
```

Read the output:

```bash
tail -f my_script.log
```

Pressing `Ctrl+C` while viewing `tail -f` stops **only the log viewer**, not the background program. Use the program's documented stop method. Avoid starting two control processes for the same motors.

---

## 6. Working with Sensors

The following examples reflect the Python interfaces used by our project.

### 6.1 Motors and Wheel Encoders

The e-puck2 has two driven wheels and wheel encoders. The project's `odometry.py` estimates the local `(x, y, heading)` pose from encoder changes. Position estimates can drift because of wheel slip and calibration errors.

```python
from pipuck import PiPuck
from odometry import Odometry

with PiPuck() as puck:
    odom = Odometry()
    frame = puck.epuck2.update()
    x_mm, y_mm, heading_deg = odom.update(*frame.motor_steps)
    print(x_mm, y_mm, heading_deg)
    puck.epuck2.set_wheel_speeds(0, 0)
```

The **first** odometry update sets the reference encoder counts. The estimated pose changes only when you provide subsequent fresh frames.


### 6.2 Infrared (IR) Proximity Sensors

The e-puck2 has **8 IR proximity sensors**, numbered `0` to `7`, arranged around its perimeter. Their reflected-light readings help detect nearby obstacles; raw values are **not centimeters without calibration**.


<p>
  <img width="200" alt="PiPuck IR proximity sensors" src="https://github.com/user-attachments/assets/92b1104d-9636-4718-bb9c-ce4535141150" /><br>
  <sub> One of the IR proximity sensors on the PiPuck robot.</sub>
</p>


**Approximate sensor directions from the robot's perspective (camera/front facing forward):**

| Sensor(s) | Direction |
|---|---|
| `0` and `7` | Front-right and front-left; closest to straight ahead |
| `1` and `6` | Front-right and front-left sides |
| `2` and `5` | Right and left sides |
| `3` and `4` | Rear-right and rear-left sides |

The project uses sensors `0`, `1`, `6`, and `7` together for front-obstacle detection:

```python
from pipuck import PiPuck

with PiPuck() as puck:
    frame = puck.epuck2.update()
    prox = list(frame.prox)
    front = max(prox[0], prox[1], prox[6], prox[7])
    print('Front proximity reading:', front)
    puck.epuck2.set_wheel_speeds(0, 0)
```

### 6.3 Range & Bearing (R&B) Board

The separate **R&B expansion board has 12 IR sensing/transmitting positions** around its ring. It exchanges short infrared messages and estimates the relative bearing and a **raw range value** of another robot.


<p>
  <img width="200" alt="PiPuck range and bearing sensors" src="https://github.com/user-attachments/assets/c68d0539-ac7e-42d5-bc37-e9daaa1676a9" /><br>
  <sub> One of the range and bearing sensors on the PiPuck robot.</sub>
</p>



Basic receive-only example (with the board driver installed):

```python
from range_and_bearing import RangeAndBearing

with RangeAndBearing() as rab:
    reading = rab.read()
    if reading is None:
        print('No packet available')
    else:
        print('Payload:', reading.data)
        print('Bearing (raw):', reading.bearing_raw)
        print('Range (board units):', reading.range_raw)
```

For a real R&B experiment, initialize and calibrate the board correctly and coordinate the transmitters. The raw range value is **not automatically a physical distance in millimeters**. See `range_and_bearing_protocol.py` for the project's message encoding.

### 6.4 Other Sensors and Cameras

- **Front-facing camera:** Used for ordinary images and video.

  <p>
  <img width="200" alt="PiPuck front camera" src="https://github.com/user-attachments/assets/5df2d30e-d98a-4167-8e27-21a37d7a4c41" /><br>
</p>

- **360° camera:** Installed on our `pipuck0` and `pipuck2` robots.

  <p>
  <img width="200" alt="PiPuck 360 camera" src="https://github.com/user-attachments/assets/49d2cc51-b354-44d8-a440-b61af34696f6" /><br>
</p>


- **Ground-sensor extension:** Installed on our `pipuck1`. This is a sensor board for detecting lines/changes on the floor; it uses downward-facing infrared sensors. The standard Pi-puck documentation includes examples in `~/Pi-puck/ground-sensor/` for supported installations.

  <p>
  <img width="200" alt="PiPuck ground sensor" src="https://github.com/user-attachments/assets/fd005cc8-4c47-4a44-b784-e38d1e84b293" /><br>
</p>

---

## 7. Cameras and Recording

On our robots, the mapping of `/dev/videoN` to a physical camera **can change after a reboot**. Verify the device number every time before recording.

### 7.1 Find and Test the Cameras

Over SSH:

```bash
ls /dev/video*
```

The presence of `/dev/video0` or `/dev/video1` does not by itself tell you which camera it represents. Test each candidate using your `base-camera.py` file.

1. Open the existing `base-camera.py` and find the `if __name__ == "__main__":` section.
2. Set the camera device number you want to test, for example:
   > **Note:** The following code is only the relevant part of base-camera.py. Keep the file's existing imports, class definitions, and other code unchanged.
   
      ```python
      cam = BaseCamera(device=1)
      img = cam.get_picture()
      if img is not None:
          cv2.imwrite('pi0_base_camera_test1.jpg', img)
          print('Image saved')
      else:
          print('No image captured; try a different device number')
      ```

4. Save the file and run it from its directory:

   ```bash
   python3 base-camera.py
   ```

5. The saved image should appear in the directory from which the script was run. Download it using SFTP or SCP to check whether it was captured by the **front-facing camera** or the **360° camera**.

   **Important:** Camera device numbers can be `0`, `1`, or `2`, and their assignments may change after rebooting the robot. Do not assume that a particular device number always corresponds to the same camera. **Always verify the correct device number by capturing a test image before using the camera.**

   If necessary, change `device=1` to another available device number and repeat the test.

   images examples from the robot's cameras:

   <p>
     <img width="250" alt="pi0_base_camera_test11" src="https://github.com/user-attachments/assets/7f4e142b-b96c-4225-925b-690aa1043d8b" /><br>
     <sub> Image using the front camera.</sub>
   </p>
  
  
   <p>
     <img width="250" alt="base_camera_test0" src="https://github.com/user-attachments/assets/c34dc77b-3900-43c9-a746-112b61cc241f" /><br>
      <sub> Image using the 360 camera.</sub>
   </p>



### 7.2 Record a Video

If `record_pov.py` and OpenCV are installed on the PiPuck:

```bash
cd ~/pipuck-walk
python3 record_pov.py --device 1 --fps 10 --width 640 --height 480
```

Replace `1` with the device number you verified **for that robot and that boot**. The script writes a timestamped MJPG-encoded `.avi` file under `recordings/`. Press `Ctrl+C` to stop and finalize the recording.

To retrieve recordings, run **on your computer**:

```bash
scp -r pi@pipuck0.local:/home/pi/pipuck-walk/recordings ./recordings-pipuck0
```

If a capture is unexpectedly green/purple, see [Section 9.7](#97-camera-not-working-or-greenpurple-pictures).

---

## 8. Charging and Checking Battery Level

The following details describe the charging ports and indicators **on our lab robots**.

### 8.1 Charge Using a Micro-USB Cable

Our robots have **three Micro-USB connectors** (upper, middle, and lower). Use the **lower connector for charging**, not the middle serial-console connector discussed in Section 3.

1. Connect a suitable charging cable to the **lower Micro-USB connector**.
2. Connect the other end to an appropriate charger or power source.
3. Observe the battery indicator LEDs:
   - **Solid red:** Charging.
   - **Green:** Charging complete.
   - **Blinking red:** The robot's battery is very low / the robot may soon shut down.


<br>
<p>
  <img width="200" alt="charging_image" src="https://github.com/user-attachments/assets/cc269f11-76cd-4055-b017-82914c9e3adb" /><br>
   <sub> Charging the robot using a cable.</sub>
</p>


### 8.2 Charge Using the Lab Charging Station

1. Locate the **two spring charging contacts on the front of the robot**.
2. Position the robot so those contacts touch the charging station's **gold-colored conductive contacts**.
3. Check that the **two small indicator lights on top of the robot** turn on, as observed on our setup.
4. Make sure the robot remains properly seated and in contact with the charger.

<br>
<p>
  <img width="200" alt="charge_pins_image" src="https://github.com/user-attachments/assets/95f8572f-d01f-4dea-a27c-f577c9dca3ff" /><br>
   <sub> Charging pins on the robot.</sub>
</p>

<p>
  <img width="200" alt="charging_station" src="https://github.com/user-attachments/assets/485493ea-aafb-45ad-becd-e2fe6d92723b" /><br>
   <sub> Charging station. notice the indicator light when the pins are contacting the charging station</sub>
</p>


### 8.3 Check Battery Percentage over SSH

After connecting:

```bash
cd ~/Pi-puck/battery/
python read-battery.py
```

Example output from a robot:

```text
e-puck battery: 3.98V, 81.15% (664.0)
ext. battery: 3.98V, 81.15% (663.0)
```

The script reports the e-puck battery and extension/external battery voltage and **estimated** percentage. The example values are not live readings; if the script is missing, verify the installed directory and file name.

---

## 9. Troubleshooting

### 9.1 SSH: Connection Refused

Example:

```text
ssh pi@pipuck0.local
ssh: connect to host pipuck0.local port 22: Connection refused
```

The hostname resolved, but an SSH connection was refused. **One common cause is that the robot has not finished booting.** Wait a short while and retry:

```bash
ssh pi@pipuck0.local
```

If the problem persists, check that you reached the intended host and that its SSH service is running.

### 9.2 SSH: Hostname Not Found or Connection Timed Out

- `Could not resolve hostname pipuck0.local`: Check power, Wi-Fi, hostname resolution (mDNS/Bonjour), and network connectivity.
- `Operation timed out`: The host or port may be unreachable. Check whether the computer and robot can communicate and whether the university network isolates devices.

Use the **USB serial console** to inspect the selected Wi-Fi network (`iwgetid -r`) if SSH is unavailable.

### 9.3 SSH: Remote Host Identification Has Changed

If you see `WARNING: REMOTE HOST IDENTIFICATION HAS CHANGED!`, **do not simply remove the old entry or accept the replacement**. First verify the robot's expected SSH host-key fingerprint through its USB serial console (Section 2.3). Legitimate causes can include reinstallation or key regeneration; an active impersonation attack is also possible.

### 9.4 Wi-Fi Configuration Does Not Take Effect

Check `iwgetid -r`, `wpa_cli -i wlan0 status`, and `ip a` over serial. Verify the network SSID/password, coverage, and whether the robot uses `wpa_supplicant` or another network manager.

### 9.5 Missing Python Modules or I²C / R&B Errors

- `ModuleNotFoundError`: Check that the Python files or packages are installed on the correct robot, and that you launched the program from the intended directory.
- `OSError: [Errno 121] Remote I/O error`: Could be an I²C device, initialization, power, or concurrent-access issue; the error alone does not identify the cause.
- Check `python3 --version`: Some robots run older Python versions and may not support newer syntax or library APIs.

### 9.6 VS Code / SFTP Problems

- If SSH works in your ordinary terminal but not VS Code, try the macOS launch tip in [Section 2.5](#25-connecting-through-vs-code).
- For SFTP sync failures, check that **Natizyskunk SFTP** is installed, `.vscode/sftp.json` is valid JSON, `remotePath` is correct, and the correct robot profile is active.
- Use **View → Output → SFTP** to inspect extension errors. An SSH terminal working does not prove file synchronization is configured.
- If the extension commands disappear after an update or reinstall, verify the correct extension is enabled, reload VS Code, and check that **SFTP: Config** appears in the Command Palette.

### 9.7 Camera Not Working or Green/Purple Pictures

- If the camera cannot open, or you get the wrong camera view, check `/dev/video*` and retest the available device numbers after reboot.
- **Lab-observed issue:** Sometimes the captured images appear in **green and purple instead of natural colors**. We have not established the cause. **Powering the robot off and on again has resolved the problem in our experience.** Stop recording and shut down the Pi-puck cleanly before rebooting; verify the colors with a new test image afterward.
- If the problem keeps recurring, record the device number, sample frame, and error logs for further diagnosis. Do not assume every green/purple image has the same cause.

### 9.8 Frozen Robot or Screen — Reset and Recovery

If the display freezes or the robot stops responding:

1. **Stop the experiment if possible**, and make sure the wheels cannot move unexpectedly.
2. If Linux still responds over SSH, use the appropriate clean reboot or shutdown command (for example `sudo reboot`, after stopping motor-control programs).
3. If the **e-puck robot firmware** specifically needs a restart, the small **reset button** on the robot can be pressed. **This resets the robot firmware only; it does not reboot the Raspberry Pi or necessarily fix a frozen Linux system.** There is no need to switch off the entire assembly just to restart the e-puck firmware.
   
   <p>
     <img width="200" alt="reset_button" src="https://github.com/user-attachments/assets/72af9176-fc1d-4f10-abd6-5429297a2e55" /><br>
      <sub> reset button</sub>
   </p>
   
4. If the Raspberry Pi or camera remains frozen, try the normal **Pi-puck power-down using `auxON`** and restart as explained in Section 2.1, allowing time for clean shutdown when possible.

> [!CAUTION]
> **Last-resort hardware procedure — ask Omri or another experienced lab member if you are unsure.** In the lab, the upper Raspberry Pi/Pi-puck board has sometimes been disconnected from its header pins and reseated to recover a persistent freeze. This is **not** the same as pressing the reset button. **Do not attempt it while any power source is connected or active**, and do not pull or strain the wired side of the assembly. The pins must be aligned correctly when reseating and must never be forced. If you cannot confidently isolate all power sources, **do not attempt this procedure: ask Omri**.

### 9.9 Other Issues Worth Recording

When a problem occurs, note the robot hostname, selected Wi-Fi network, exact command, error message, and whether restarting solved it. Add a screenshot or log when available so others can reproduce the diagnosis.

---

## 10. Useful Commands

| Task | Command |
|---|---|
| Connect to PiPuck 0 | `ssh pi@pipuck0.local` |
| Connect to PiPuck 1 | `ssh pi@pipuck1.local` |
| Connect to PiPuck 2 | `ssh pi@pipuck2.local` |
| Leave an SSH session | `exit` |
| Check selected Wi-Fi SSID (on robot) | `iwgetid -r` |
| Check network interfaces (on robot) | `ip a` |
| Check Python version (on robot) | `python3 --version` |
| See running processes (on robot) | `ps aux` |
| Check available camera devices (on robot) | `ls /dev/video*` |
| Inspect mission log (on robot) | `tail -n 60 ~/pipuck-walk/team_mission_rab.log` |
| Identify macOS serial devices (on computer) | `ls /dev/cu.*` |
| Read battery values (on robot) | `cd ~/Pi-puck/battery && python read-battery.py` |
| Display a remote file (on robot) | `cat ~/pipuck-walk/my_script.py` |

---

## 11. Useful Links

- [GCtronic — Pi-puck hardware, power controls, console, reset, and supported sensors](https://www.gctronic.com/doc/index.php?title=Pi-puck)
- [GCtronic — Range and Bearing extension documentation](https://www.gctronic.com/doc/index.php?title=Others_Extensions#Range_and_bearing)
- [Natizyskunk — SFTP for Visual Studio Code](https://marketplace.visualstudio.com/items?itemName=Natizyskunk.sftp)
- [Natizyskunk SFTP configuration](https://github.com/Natizyskunk/vscode-sftp/wiki/Configuration)

  
*This guide is a living document. Add labeled photos and verified lab-specific recovery procedures as they become available.*
