import cv2
import cv2.aruco as aruco

def generar_aruco():
    # 1. Configurar el diccionario (Debe coincidir con el del robot)
    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

    # 2. Configurar los parámetros del marcador
    marcadores_id = [id for id in range(1, 10)]  # El ID que usaste en tu rutina de Nav2
    tamano_pixeles = 400  # Tamaño del cuadrado en píxeles (alta resolución)

    # 3. Generar la matriz de la imagen (Compatible con OpenCV viejo y nuevo)
    for aruco_id in marcadores_id:
        if hasattr(cv2.aruco, 'generateImageMarker'):
            # Para OpenCV 4.7.0 o superior (Tu versión actual en Windows)
            marcador_img = cv2.aruco.generateImageMarker(aruco_dict, aruco_id, tamano_pixeles)
        else:
            # Para OpenCV 4.6.0 o inferior (La versión de tu máquina con ROS 2)
            marcador_img = cv2.aruco.drawMarker(aruco_dict, aruco_id, tamano_pixeles)

        # 4. Guardar la imagen en el directorio actual
        nombre_archivo = f"aruco_id_{aruco_id}.png"
        cv2.imwrite(nombre_archivo, marcador_img)
        print(f"¡Éxito! Marcador ArUco ID {aruco_id} guardado como '{nombre_archivo}'.")

    # 5. Mostrar la imagen en pantalla
    print("Presiona cualquier tecla en la ventana de la imagen para cerrar.")
    cv2.imshow("Generador ArUco", marcador_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    generar_aruco()