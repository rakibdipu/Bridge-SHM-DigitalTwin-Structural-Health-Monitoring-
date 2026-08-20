# Bridge SHM Digital Twin Live Dashboard

This folder contains the interactive web-based Digital Twin monitoring dashboard for the Bridge Structural Health Monitoring System.

## Features
- **Live 2.5D Digital Twin Model**: Real-time animated truss bridge reflecting deflection, node vibration energy, and structural state.
- **Real-Time FFT Spectrum & Time-Domain Waveform**: Live Chart.js visualization of acceleration signals and frequency spectra up to Nyquist frequency.
- **Web Serial API**: Connect your ESP32-S3 directly via USB within Chrome/Edge—no server installation required!
- **Simulation & Damage Injection**: Test healthy (8.2 Hz) vs damaged (7.5 Hz) conditions interactively.
- **Telemetry Export**: Download CSV logs of monitored vibration parameters.

## Running the Dashboard

### Option 1: Direct Browser (No Install)
Simply double-click or open `index.html` in Google Chrome, Microsoft Edge, or any modern web browser.

### Option 2: Python Local Server
```bash
pip install -r requirements.txt
python app.py
```
Open `http://localhost:5000` in your web browser.
