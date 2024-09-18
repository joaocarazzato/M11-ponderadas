import cv2
import numpy as np
import requests
import serial

# Endereço IP do servidor web da ESP32-CAM
url = 'http://192.168.15.13/capture'
port = '/dev/ttyUSB0'  # Substitua pelo nome correto da sua porta serial
baud_rate = 115200 

try:
    ser = serial.Serial(port, baud_rate)
    ser.setDTR(False)
    ser.setRTS(False)   
    print(f"Conectado à porta serial {port} com baud rate {baud_rate}")
except serial.SerialException as e:
    print(f"Erro ao abrir a porta serial: {e}")
    exit()

def fetch_image_from_url(url):
    response = requests.get(url, stream=True)
    if response.status_code == 200:
        image_bytes = response.content
        # Converte a imagem em bytes para um array NumPy
        img_array = np.frombuffer(image_bytes, np.uint8)
        img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
        return img
    else:
        print("Falha ao capturar imagem do servidor")
        return None

# Inicializa o classificador de face
detec = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")

while True:
    # Lê uma linha de dados da porta serial
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').strip()
        if line:
            print(f"Serial: {line}")
    # Captura a imagem do servidor
    frame = fetch_image_from_url(url)
    
    if frame is not None:
        cinza = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        face = detec.detectMultiScale(cinza, 1.3, 3)  # Detectar a face
        for (x, y, larg, alt) in face:
            frame = cv2.rectangle(frame, (x, y), (x + larg, y + alt), (0, 0, 255), 3)
            ser.write(bytes(f"[DETECTION]: x={x}, y={y}, larg={larg}, alt={alt}\l", "utf-8"))
        
        # Exibe a imagem com OpenCV
        cv2.imshow('ESP32-CAM Stream', frame)
    else:
        print("Falha na captura do frame")

    # Pressione 'q' para sair do loop e fechar a janela
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Libere o recurso de captura e feche as janelas
cv2.destroyAllWindows()
