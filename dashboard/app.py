# ==============================================================================
# Bridge Structural Health Monitoring (SHM) - Python Backend / WebSocket Server
# ==============================================================================
# Optional Python bridge to read ESP32-S3 Serial or MQTT stream and serve telemetry
# to web clients via Flask / WebSockets.
# ==============================================================================

import json
import time
import threading
from flask import Flask, render_to_string, send_from_directory

app = Flask(__name__, static_folder='.')

@app.route('/')
def index():
    return send_from_directory('.', 'index.html')

if __name__ == '__main__':
    print("==========================================================")
    print("Bridge SHM Digital Twin Web Server")
    print("Open http://localhost:5000 in your browser.")
    print("==========================================================")
    app.run(host='0.0.0.0', port=5000, debug=True)
