from flask import Flask, Response, render_template, send_file
from camera import Camera
from gevent.wsgi import WSGIServer
import argparse
from io import BytesIO
from time import sleep
import cv2

ap = argparse.ArgumentParser()
ap.add_argument('--camera', default='0')
ap.add_argument('--port', default='3141')
args = vars(ap.parse_args())

CAMERA_NUMBER = int(args['camera'])
PORT_NUMBER = int(args['port'])
camera = Camera(CAMERA_NUMBER)

# Define FLASK application.
app = Flask(__name__)

# Generator function to render video frames from the camera.
def gen():
    while True:
        frame = camera.get_image()
        if frame:
            yield (b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

# Define route for the camera's video feed.
@app.route('/video_feed') # Currently the webcam feed
def video_feed():
    generate = gen()
    sleep(1/20)
    return Response(generate, mimetype=\
            'multipart/x-mixed-replace; boundary=frame')

# Define route for single image.
@app.route('/current_image')
def current_image():
    current_frame = camera.get_image()
    return send_file(BytesIO(current_frame), mimetype='image/jpeg')

# Define route for binary image.
@app.route('/current_binary_image')
def current_binary_image():
    current_frame = camera.get_image()
    return current_frame

def serve():
    port_number = PORT_NUMBER

    # Start the server
    http_server = WSGIServer(('', port_number), app)
    http_server.serve_forever()

# Start the application.
if __name__ == "__main__":

    port_number = PORT_NUMBER

    # Start the server
    http_server = WSGIServer(('', port_number), app)
    http_server.serve_forever()



