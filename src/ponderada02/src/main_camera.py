import cv2

# Endereço IP do servidor web da ESP32-CAM
url='http://10.128.0.29'

# Inicializa a captura de vídeo com OpenCV
cap = cv2.VideoCapture(url)

while True:
    # Captura cada frame do stream
    ret, frame = cap.read()
    detec = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")
    cinza = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    face = detec.detectMultiScale(cinza, 1.3, 3) #Detectar a face
    for (x, y, larg, alt) in face:
        frame_rectangle = cv2.rectangle(frame, (x, y), (x + larg, y + alt), (0, 0, 255), 3)



    # Se a captura foi bem sucedida, exibe a imagem
    if ret:
        cv2.imshow('ESP32-CAM Stream', frame)
    else:
        print("Falha na captura do frame")

    # Pressiona 'q' para sair do loop e fechar a janela
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Libera o recurso de captura e fecha as janelas
cap.release()
cv2.destroyAllWindows()
