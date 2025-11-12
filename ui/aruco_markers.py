import cv2
import cv2.aruco as aruco
import numpy as np

# === CONFIGURAÇÕES ===
# Dicionário de marcadores ArUco (6x6 bits, 250 marcadores possíveis)
ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

# Parâmetros de detecção (padrão, pode ajustar depois)
parameters = aruco.DetectorParameters()

# Detector
detector = aruco.ArucoDetector(ARUCO_DICT, parameters)

# Tamanho físico do marcador (em metros) – use para pose estimation
# Se não precisar de pose, pode deixar qualquer valor
marker_length = 0.05  # 5 cm

# === CALIBRAÇÃO DA CÂMERA ===
# Substitua pelos parâmetros calibrados da sua câmera (se já tiver feito calibração)
# Aqui vamos usar uma matriz fictícia só para rodar (pose não vai ficar realista)
cam_matrix = np.array([[800, 0, 320], [0, 800, 240], [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.zeros((5, 1))  # sem distorção

# === ABRIR CÂMERA ===
cap = cv2.VideoCapture(0)  # 0 = câmera padrão

if not cap.isOpened():
    print("Não consegui abrir a câmera!")
    exit()

print("Pressione 'q' para sair.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Detectar marcadores
    corners, ids, rejected = detector.detectMarkers(frame)

    if ids:
        # Desenhar marcadores detectados
        aruco.drawDetectedMarkers(frame, corners, ids)

        # Estimar pose de cada marcador
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners, marker_length, cam_matrix, dist_coeffs
        )

        for rvec, tvec in zip(rvecs, tvecs):
            # Desenhar eixos 3D sobre o marcador
            cv2.drawFrameAxes(frame, cam_matrix, dist_coeffs, rvec, tvec, marker_length)
    else:
        print("NADAAA")

    # Mostrar frame
    cv2.imshow("ArUco Detection", frame)

    # Tecla 'q' para sair
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
