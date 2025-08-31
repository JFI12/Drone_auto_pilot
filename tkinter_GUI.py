import tkinter as tk
from tkinter import ttk
import time
import threading
from flask import Flask, request, jsonify
from flask_socketio import SocketIO, emit
import socketio  # Import the socketio client

# Initialize Flask app
app = Flask(__name__)
socketio_server = SocketIO(app)  # Rename the SocketIO server instance
brightness_value = 0  # Global variable to store brightness level

@app.route('/')
def index():
    return 'WebSocket Server Running'

@socketio_server.on('get_brightness')
def handle_brightness_request():
    emit('brightness_update', {'brightness': brightness_value})

@socketio_server.on('set_brightness')
def handle_set_brightness(data):
    global brightness_value
    brightness_value = data['brightness']
    # Uncomment to send to Arduino
    # arduino.write((str(brightness_value) + '\n').encode())
    emit('brightness_update', {'brightness': brightness_value}, broadcast=True)

@app.route('/brightness', methods=['POST'])
def set_brightness():
    global brightness_value
    data = request.json
    if 'value' in data:
        brightness_value = int(data['value'])
        # Uncomment to send to Arduino
        # arduino.write((str(brightness_value) + '\n').encode())
        return jsonify({"status": "success", "brightness": brightness_value}), 200
    return jsonify({"status": "error", "message": "No value provided"}), 400

@app.route('/brightness', methods=['GET'])
def get_brightness():
    return jsonify({"brightness": brightness_value})

# Function to run the Flask server
def run_flask():
    socketio_server.run(app, host='0.0.0.0', port=5000, debug=False)

# Start Flask server in a separate thread
flask_thread = threading.Thread(target=run_flask)
flask_thread.daemon = True
flask_thread.start()

# Create the main window
root = tk.Tk()
root.title("Arduino LED Brightness Control")

# Set window size and position it in the center
root.geometry("400x200")
root.eval('tk::PlaceWindow . center')

# Label for LED brightness instruction
ttk.Label(root, text="LED Brightness:").pack(pady=10)

# Label to display the current brightness value
brightness_label = ttk.Label(root, text="Current Brightness: 0")  # Initial value
brightness_label.pack(pady=10)

# Initialize SocketIO client
sio_client = socketio.Client()  # Create a new SocketIO client instance

# Connect to the WebSocket server
def connect_to_server():
    sio_client.connect('http://192.168.0.102:5000')  # Update with your server's address

# Function to emit brightness value
def update_brightness(value):
    try:
        int_value = int(float(value))  # Convert to integer
        # Emit the brightness value to the server
        sio_client.emit('set_brightness', {'brightness': int_value})
        brightness_label.config(text=f"Current Brightness: {int_value}")  # Update the label
    except Exception as e:
        print(f"Error updating brightness: {e}")  # Print error if occurs

# Slider to control brightness
brightness_slider = ttk.Scale(root, from_=0, to=1023, orient='horizontal', command=update_brightness)
brightness_slider.set(0)  # Set the initial slider position to 0
brightness_slider.pack(pady=20)

# Start the connection to the server
connect_to_server()

# Start the main GUI loop
root.mainloop()