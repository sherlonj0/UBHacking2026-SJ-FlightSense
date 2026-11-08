"""FlightSense live USB telemetry dashboard.
Run: python -m streamlit run flightsense_dashboard.py
Requires: streamlit, pyserial, pandas, scikit-learn
"""
import json
import math
from datetime import datetime

import pandas as pd
from sklearn.ensemble import IsolationForest
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler
import serial
from serial.tools import list_ports
import streamlit as st

st.set_page_config(page_title="FlightSense | Live Flight Instruments", page_icon="✈️", layout="wide")

FIELDS = ["temperature", "humidity", "pressure", "altitude", "accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z"]
MOTION_FIELDS = ["accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z"]
MIN_TRAIN_SAMPLES = 50


def default_history():
    return []


def close_serial():
    port = st.session_state.get("serial_connection")
    if port is not None:
        try:
            port.close()
        except serial.SerialException:
            pass
    st.session_state.serial_connection = None


def read_latest():
    """Read all buffered lines without blocking, keeping the newest valid JSON."""
    port = st.session_state.get("serial_connection")
    if port is None or not port.is_open:
        return None, "Disconnected"
    latest = None
    try:
        # Read up to 100 queued lines to avoid bogging down if the app was paused.
        for _ in range(100):
            if port.in_waiting == 0:
                break
            line = port.readline().decode("utf-8", errors="replace").strip()
            if not line:
                continue
            try:
                data = json.loads(line)
            except json.JSONDecodeError:
                continue
            if not isinstance(data, dict):
                continue
            try:
                sample = {name: float(data[name]) for name in FIELDS}
            except (KeyError, ValueError, TypeError):
                continue
            if not all(math.isfinite(value) for value in sample.values()):
                continue
            sample["time"] = datetime.now()
            latest = sample
        return latest, None
    except (OSError, serial.SerialException) as error:
        close_serial()
        return None, str(error)


def attitude(data):
    ax, ay, az = data["accel_x"], data["accel_y"], data["accel_z"]
    pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
    roll = math.degrees(math.atan2(ay, az))
    return pitch, roll


def horizon_svg(pitch, roll):
    # Conventional illustrative horizon: orientation depends on sensor mounting.
    shift = max(-90, min(90, pitch * 2.0))
    return f'''<svg viewBox="0 0 440 310" width="100%" role="img" aria-label="Artificial horizon pitch {pitch:.1f} degrees, roll {roll:.1f} degrees">
      <defs><clipPath id="skyclip"><circle cx="220" cy="146" r="122"/></clipPath></defs>
      <circle cx="220" cy="146" r="125" fill="#0b1220" stroke="#a3b4cf" stroke-width="3"/>
      <g clip-path="url(#skyclip)">
        <g transform="rotate({-roll:.2f} 220 146) translate(0 {shift:.2f})">
          <rect x="-100" y="-340" width="650" height="486" fill="#247bb9"/>
          <rect x="-100" y="146" width="650" height="500" fill="#9c603a"/>
          <line x1="-100" y1="146" x2="550" y2="146" stroke="#eef5ff" stroke-width="3"/>
          <line x1="185" y1="106" x2="255" y2="106" stroke="#eef5ff" stroke-width="2"/>
          <line x1="185" y1="186" x2="255" y2="186" stroke="#eef5ff" stroke-width="2"/>
        </g>
      </g>
      <path d="M148 148 H194 L208 158 H232 L246 148 H292" stroke="#fbbf24" stroke-width="5" fill="none" stroke-linecap="round"/>
      <circle cx="220" cy="146" r="4" fill="#fbbf24"/>
      <path d="M220 14 L212 29 H228 Z" fill="#fbbf24"/>
      <text x="220" y="294" text-anchor="middle" fill="currentColor" font-size="17">PITCH {pitch:+.1f}°  |  ROLL {roll:+.1f}°</text>
    </svg>'''


st.title("✈️ FlightSense")
st.caption("Live embedded telemetry • STM32 NUCLEO-F446RE • AHT20 + BMP280 + MPU6050")

if "serial_connection" not in st.session_state:
    st.session_state.serial_connection = None
if "history" not in st.session_state:
    st.session_state.history = default_history()
if "last_sample" not in st.session_state:
    st.session_state.last_sample = None
if "anomaly_model" not in st.session_state:
    st.session_state.anomaly_model = None
if "trained_sample_count" not in st.session_state:
    st.session_state.trained_sample_count = 0

ports = [port.device for port in list_ports.comports()]
options = ports if ports else ["COM7"]
default_port_index = options.index("COM7") if "COM7" in options else 0

with st.sidebar:
    st.header("Connection")
    selected_port = st.selectbox("STM32 serial port", options, index=default_port_index, disabled=st.session_state.serial_connection is not None)
    baud_rate = st.selectbox("Baud rate", [115200, 9600], index=0, disabled=st.session_state.serial_connection is not None)
    if st.session_state.serial_connection is None:
        if st.button("Connect", type="primary", use_container_width=True):
            try:
                st.session_state.serial_connection = serial.Serial(selected_port, baud_rate, timeout=0)
                st.session_state.history = []
                st.session_state.last_sample = None
                st.session_state.anomaly_model = None
                st.session_state.trained_sample_count = 0
                st.rerun()
            except (OSError, serial.SerialException) as error:
                st.error(f"Could not open {selected_port}: {error}. Close PuTTY first.")
    else:
        if st.button("Disconnect", use_container_width=True):
            close_serial()
            st.rerun()
    st.caption("Arduino fault telemetry can be added through a second COM port after your teammate implements serial reporting.")
    st.caption("AI alerts are advisory only; they do not control or stop the servos.")
    if st.button("Clear graphs", use_container_width=True):
        st.session_state.history = []
        st.session_state.last_sample = None
        st.session_state.anomaly_model = None
        st.session_state.trained_sample_count = 0


@st.fragment(run_every="500ms")
def live_panel():
    sample, error = read_latest()
    if sample is not None:
        st.session_state.last_sample = sample
        st.session_state.history.append(sample)
        st.session_state.history = st.session_state.history[-240:]

    conn = st.session_state.serial_connection
    if conn is None:
        st.warning("Not connected. Choose your STM32 COM port and click Connect. Close PuTTY first.")
    elif error:
        st.error(f"Serial connection error: {error}")
    elif st.session_state.last_sample is None:
        st.info("Connected. Waiting for the first valid FlightSense JSON packet...")
    else:
        age = (datetime.now() - st.session_state.last_sample["time"]).total_seconds()
        if age > 3:
            st.warning(f"No new telemetry for {age:.0f} seconds. Check the STM32 is running.")
        else:
            st.success(f"LIVE • {selected_port} • updated {age:.1f}s ago")

    data = st.session_state.last_sample
    if data is None:
        return

    st.subheader("AI motion anomaly detection")
    st.caption(
        "Experimental Isolation Forest model using the six MPU6050 values. "
        "Collect normal readings while the device is stationary and moving gently, then train. "
        "Anomaly means 'unlike the training examples', not a verified aircraft fault."
    )
    history = st.session_state.history
    ready = len(history) >= MIN_TRAIN_SAMPLES
    train_col, reset_col = st.columns([2, 1])
    with train_col:
        if st.button(
            f"Train on latest normal motion ({min(len(history), 200)} samples)",
            disabled=not ready,
            use_container_width=True,
            key="train_ai",
        ):
            training = pd.DataFrame(history[-200:])[MOTION_FIELDS]
            model = make_pipeline(
                StandardScaler(),
                IsolationForest(n_estimators=100, contamination=0.08, random_state=42),
            )
            model.fit(training)
            st.session_state.anomaly_model = model
            st.session_state.trained_sample_count = len(training)
    with reset_col:
        if st.button("Reset AI", use_container_width=True, key="reset_ai"):
            st.session_state.anomaly_model = None
            st.session_state.trained_sample_count = 0
    model = st.session_state.anomaly_model
    if model is None:
        st.info(
            f"AI not trained: {len(history)}/{MIN_TRAIN_SAMPLES} samples recorded. "
            "Keep the board still and tilt it gently for about 30 seconds, then press Train."
        )
    else:
        row = pd.DataFrame([{name: data[name] for name in MOTION_FIELDS}])
        prediction = int(model.predict(row)[0])
        score = float(model.decision_function(row)[0])
        if (datetime.now() - data["time"]).total_seconds() > 3:
            st.warning("AI result stale: waiting for fresh telemetry")
        elif prediction == -1:
            st.error(f"AI MOTION ANOMALY • score {score:+.3f}")
        else:
            st.success(f"AI MOTION NORMAL • score {score:+.3f}")
        st.caption(
            f"Trained on {st.session_state.trained_sample_count} normal samples. "
            "Negative scores indicate anomalies; the result is sensitive to your baseline. "
            "This is separate from the Arduino button-triggered fault detector."
        )

    a, b, c, d = st.columns(4)
    a.metric("Temperature", f'{data["temperature"]:.2f} °C')
    b.metric("Humidity", f'{data["humidity"]:.2f} %')
    c.metric("Pressure", f'{data["pressure"]:.2f} hPa')
    d.metric("Relative altitude", f'{data["altitude"]:+.2f} m')

    left, right = st.columns([1.05, 1.1])
    with left:
        st.subheader("Artificial horizon")
        pitch, roll = attitude(data)
        st.markdown(horizon_svg(pitch, roll), unsafe_allow_html=True)
        st.caption("Pitch and roll estimated from accelerometer gravity only. Quick motion and mounting angle affect the display; this is not a certified flight instrument.")
    with right:
        st.subheader("6-axis motion")
        ax, ay, az = st.columns(3)
        ax.metric("Accel X", f'{data["accel_x"]:+.2f} g')
        ay.metric("Accel Y", f'{data["accel_y"]:+.2f} g')
        az.metric("Accel Z", f'{data["accel_z"]:+.2f} g')
        gx, gy, gz = st.columns(3)
        gx.metric("Gyro X", f'{data["gyro_x"]:+.2f} °/s')
        gy.metric("Gyro Y", f'{data["gyro_y"]:+.2f} °/s')
        gz.metric("Gyro Z", f'{data["gyro_z"]:+.2f} °/s')

    if len(st.session_state.history) >= 2:
        frame = pd.DataFrame(st.session_state.history).set_index("time")
        st.subheader("Live sensor graphs")
        st.caption("Showing the latest 240 samples; graphs update as new JSON packets arrive.")
        col1, col2 = st.columns(2)
        with col1:
            st.markdown("**Acceleration (g)**")
            st.line_chart(frame[["accel_x", "accel_y", "accel_z"]], height=220)
            st.markdown("**Temperature (°C) / Humidity (%)**")
            st.line_chart(frame[["temperature", "humidity"]], height=220)
        with col2:
            st.markdown("**Angular velocity (°/s)**")
            st.line_chart(frame[["gyro_x", "gyro_y", "gyro_z"]], height=220)
            st.markdown("**Pressure (hPa)**")
            st.line_chart(frame[["pressure"]], height=220)

    with st.expander("Latest raw JSON packet"):
        st.json({k: data[k] for k in FIELDS})


live_panel()
