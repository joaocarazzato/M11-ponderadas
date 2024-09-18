from flask import Flask, request, Response
import cv2
import numpy as np

app = Flask(__name__)

@app.route('/upload', methods=['POST'])
def upload_image():
    # Recebe a imagem do POST
    image_bytes = request.data
    if not image_bytes:
        return "No image data", 400

    # Converte a imagem em bytes para um array NumPy
    img_array = np.frombuffer(image_bytes, np.uint8)
    img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

    if img is not None:
        # Mostra a imagem recebida
        cv2.imshow('Received Image', img)
        cv2.waitKey(1)
        return "Image received", 200
    else:
        return "Failed to decode image", 400

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
