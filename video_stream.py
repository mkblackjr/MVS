from flask import Flask, Response, render_template, send_file
from camera import Camera
from gevent.wsgi import WSGIServer
from io import BytesIO
from time import sleep
import cv2

def serve(cam_num=0,port_num=3141):

    CAMERA_NUMBER = cam_num
    PORT_NUMBER = port_num
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

    # Start the server
    http_server = WSGIServer(('', PORT_NUMBER), app)
    http_server.serve_forever()


# Start the application.
if __name__ == "__main__":

    serve()



