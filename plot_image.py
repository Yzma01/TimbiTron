import cv2
import numpy as np

def image_to_lines(image_path, threshold1=50, threshold2=150, min_contour_length=10):
    """
    Convierte una imagen en líneas de contorno para el plotter.
    
    Retorna una lista de líneas en el formato:
        [[[x1, y1], [x2, y2], ...], ...]
    con coordenadas normalizadas (0-100).
    """
    # Cargar y preprocesar la imagen
    img = cv2.imread(image_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Suavizar para reducir ruido
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Detectar bordes
    edges = cv2.Canny(blurred, threshold1, threshold2)

    # Encontrar contornos
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    h, w = img.shape[:2]
    lines = []

    for contour in contours:
        # Filtrar contornos muy cortos (ruido)
        if len(contour) < min_contour_length:
            continue

        # Convertir cada contorno a lista de puntos [x, y]
        # Normalizar a rango 0-100
        line = []
        for point in contour:
            x = float(point[0][0]) / w * 100
            y = float(point[0][1]) / h * 100    
            line.append([x, y])

        lines.append(line)

    print(f"Se encontraron {len(lines)} líneas en la imagen.")
    return lines
