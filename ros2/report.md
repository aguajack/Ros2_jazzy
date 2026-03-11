# Ros2_jazzy
ros2手冊
## 建立基礎通訊序列阜
輸入下列指令:

```cpp
sudo apt install -y socat
```
建立兩個虛擬序列阜，再輸入下列指令檢查:

```cpp
socat -d -d pty,raw,echo=0 pty,raw,echo=0
```
終端機保持開啟，因為需要保持連線狀態
先下載pip，輸入下列指令:

```cpp
sudo apt install -y python3-pip
```
確認 pip 有了：

```cpp
python3 -m pip --version
```
因為系統會擋掉pip所以需要裝虛擬環境venv，但可以換個方法，用 apt 安裝 pyserial

```cpp
sudo apt install -y python3-serial
```
並且確認是否安裝完成:

```cpp
python3 -c "import serial; print(serial.__version__)"
```
接下來繼續跑 mock 手臂與 ROS driver，直接在終端機輸入:

```py
cat > ~/mock_arm.py << 'EOF'
import sys, time
import serial

PORT = sys.argv[1]
ser = serial.Serial(PORT, baudrate=115200, timeout=0.1)

j = [0.0]*6

def send_state():
    msg = "STATE " + " ".join([f"J{i+1} {j[i]:.3f}" for i in range(6)]) + "\n"
    ser.write(msg.encode())

while True:
    line = ser.readline().decode(errors="ignore").strip()
    if line.startswith("SET"):
        parts = line.split()
        for k in range(1, len(parts)-1, 2):
            name = parts[k]
            val = float(parts[k+1])
            if name.startswith("J"):
                idx = int(name[1:]) - 1
                if 0 <= idx < 6:
                    j[idx] = val
        send_state()
    else:
        send_state()
        time.sleep(0.2)
EOF

python3 ~/mock_arm.py /dev/pts/2
```
再開一個新的終端機輸入:
```py
cat > ~/ros_arm_driver.py << 'EOF'
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import serial

PORT = sys.argv[1]
JOINT_NAMES = [f"J{i}" for i in range(1, 7)]  # J1..J6

class ArmDriver(Node):
    def __init__(self, port: str):
        super().__init__('arm_serial_driver')
        self.ser = serial.Serial(port, baudrate=115200, timeout=0.05)
        self.create_subscription(Float64MultiArray, '/arm/joint_cmd', self.on_cmd, 10)
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_timer(0.05, self.poll_state)  # 20 Hz

    def on_cmd(self, msg: Float64MultiArray):
        if len(msg.data) < 6:
            self.get_logger().warn("Need 6 values for J1..J6")
            return
        vals = msg.data[:6]
        cmd = "SET " + " ".join([f"J{i+1} {vals[i]:.3f}" for i in range(6)]) + "\n"
        self.ser.write(cmd.encode())

    def poll_state(self):
        line = self.ser.readline().decode(errors="ignore").strip()
        if not line.startswith("STATE"):
            return
        parts = line.split()
        pos = [0.0]*6
        for k in range(1, len(parts)-1, 2):
            name = parts[k]
            try:
                val = float(parts[k+1])
            except ValueError:
                continue
            if name.startswith("J"):
                idx = int(name[1:]) - 1
                if 0 <= idx < 6:
                    pos[idx] = val

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = JOINT_NAMES
        js.position = pos
        self.js_pub.publish(js)

def main():
    rclpy.init()
    node = ArmDriver(PORT)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
```
在開新的終端機看 /joint_states 有沒有在刷:

```shell
source /opt/ros/jazzy/setup.bash
ros2 topic echo /joint_states
```
發一筆命令（再開一個新終端機）

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic pub --once /arm/joint_cmd std_msgs/msg/Float64MultiArray "{data: [0.5, -0.5, 0.0, 0.0, 0.0, 0.0]}"
```
最終結果/joint_states會有新的資料一直刷

## 建立一建腳本
先輸入:
```bash
cat > ~/start_mock_arm.sh << 'EOF'
#!/usr/bin/env bash
set -euo pipefail

RUN_DIR="$HOME/.mock_arm_run"
PID_FILE="$RUN_DIR/pids.env"
SOCAT_LOG="$RUN_DIR/socat.log"
MOCK_LOG="$RUN_DIR/mock.log"
DRIVER_LOG="$RUN_DIR/driver.log"

MOCK_PY="$HOME/mock_arm.py"
DRIVER_PY="$HOME/ros_arm_driver.py"
ROS_SETUP="/opt/ros/jazzy/setup.bash"

mkdir -p "$RUN_DIR"

if [[ ! -f "$MOCK_PY" ]]; then
  echo "ERROR: not found: $MOCK_PY"
  echo "Create it first (mock_arm.py)."
  exit 1
fi

if [[ ! -f "$DRIVER_PY" ]]; then
  echo "ERROR: not found: $DRIVER_PY"
  echo "Create it first (ros_arm_driver.py)."
  exit 1
fi

# If already running, stop first
if [[ -f "$PID_FILE" ]]; then
  # shellcheck disable=SC1090
  source "$PID_FILE" || true
  if [[ -n "${SOCAT_PID:-}" ]] && kill -0 "$SOCAT_PID" 2>/dev/null; then
    echo "Existing run detected. Stopping it first..."
    "$HOME/stop_mock_arm.sh" || true
  fi
fi

: > "$SOCAT_LOG"
: > "$MOCK_LOG"
: > "$DRIVER_LOG"

echo "Starting socat (virtual serial pair)..."
nohup socat -d -d pty,raw,echo=0 pty,raw,echo=0 > /dev/null 2>>"$SOCAT_LOG" &
SOCAT_PID=$!

# Wait for socat to print PTY paths
A=""
B=""
for _ in $(seq 1 50); do
  # socat prints lines like: "PTY is /dev/pts/1"
  mapfile -t PTS < <(grep -oE "/dev/pts/[0-9]+" "$SOCAT_LOG" | head -n 2 || true)
  if [[ ${#PTS[@]} -ge 2 ]]; then
    A="${PTS[0]}"
    B="${PTS[1]}"
    break
  fi
  sleep 0.1
done

if [[ -z "$A" || -z "$B" ]]; then
  echo "ERROR: failed to detect PTY ports from socat log."
  echo "Check: $SOCAT_LOG"
  exit 1
fi

echo "Ports detected:"
echo "  DRIVER_PORT = $A"
echo "  MOCK_PORT   = $B"

echo "Starting mock device on $B ..."
nohup python3 "$MOCK_PY" "$B" > /dev/null 2>>"$MOCK_LOG" &
MOCK_PID=$!

echo "Starting ROS driver on $A ..."
nohup bash -lc "source '$ROS_SETUP' && python3 '$DRIVER_PY' '$A'" > /dev/null 2>>"$DRIVER_LOG" &
DRIVER_PID=$!

cat > "$PID_FILE" << PIDS
SOCAT_PID=$SOCAT_PID
MOCK_PID=$MOCK_PID
DRIVER_PID=$DRIVER_PID
DRIVER_PORT=$A
MOCK_PORT=$B
PIDS

echo ""
echo "Started OK."
echo "PIDs: socat=$SOCAT_PID mock=$MOCK_PID driver=$DRIVER_PID"
echo "Logs:"
echo "  $SOCAT_LOG"
echo "  $MOCK_LOG"
echo "  $DRIVER_LOG"
echo ""
echo "Verify (new terminal):"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  ros2 topic echo /joint_states"
echo ""
echo "Send test command (new terminal):"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  ros2 topic pub --once /arm/joint_cmd std_msgs/msg/Float64MultiArray \"{data: [0.5, -0.5, 0, 0, 0, 0]}\""
echo ""
echo "Stop everything:"
echo "  bash ~/stop_mock_arm.sh"
EOF

cat > ~/stop_mock_arm.sh << 'EOF'
#!/usr/bin/env bash
set -euo pipefail

RUN_DIR="$HOME/.mock_arm_run"
PID_FILE="$RUN_DIR/pids.env"

if [[ ! -f "$PID_FILE" ]]; then
  echo "No pid file found: $PID_FILE"
  exit 0
fi

# shellcheck disable=SC1090
source "$PID_FILE" || true

stop_pid() {
  local name="$1"
  local pid="$2"
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    echo "Stopping $name (pid=$pid)..."
    kill "$pid" 2>/dev/null || true
  else
    echo "$name not running."
  fi
}

stop_pid "driver" "${DRIVER_PID:-}"
stop_pid "mock" "${MOCK_PID:-}"
stop_pid "socat" "${SOCAT_PID:-}"

echo "Done."
EOF

chmod +x ~/start_mock_arm.sh ~/stop_mock_arm.sh
echo "Scripts created:"
echo "  ~/start_mock_arm.sh"
echo "  ~/stop_mock_arm.sh"
```
1.一件啟動的指令:
```shell
bash ~/arm_comm_lab/start_mock_arm.sh
```
2.驗證（新終端機）
```shell
source /opt/ros/jazzy/setup.bash
ros2 topic echo /arm/comm_status std_msgs/msg/String
```
3.發命令（新終端機）
```shell
source /opt/ros/jazzy/setup.bash
ros2 topic pub --once /arm/joint_cmd std_msgs/msg/Float64MultiArray "{data: [0.5, -0.5, 0, 0, 0, 0]}"
```
4.一鍵停止
```shell
bash ~/arm_comm_lab/stop_mock_arm.sh
```




