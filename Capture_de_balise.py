import requests
import cv2
import time
import numpy as np
import controller  # Import du module controller
from typing import Tuple, Optional
import cv2
import numpy as np

# Charger le dictionnaire des marqueurs ArUco
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters_create()
# Ajuster les paramètres du détecteur
parameters.adaptiveThreshConstant = 7
parameters.minMarkerPerimeterRate = 0.03
parameters.maxMarkerPerimeterRate = 4.0
parameters.perspectiveRemovePixelPerCell = 4
# Ouvrir le flux vidéo
cap = cv2.VideoCapture(0)


def detect_marker_id():
    """
    Détecte les marqueurs ArUco et renvoie l'ID du  marqueur.
    """
    if not cap.isOpened():
        print("Erreur : Impossible d'ouvrir la caméra.")
        return None
    # Lire une image du flux vidéo
    ret, frame = cap.read()
    if not ret:
        print("Erreur de lecture du flux vidéo.")
        cap.release()
        return None
    
    # Convertir l'image en niveaux de gris
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Détecter les marqueurs ArUco
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    # Si des marqueurs sont détectés
    if ids is not None and len(ids) > 0:
        return int(ids[0][0])
    else:
        #print("Aucun marqueur détecté.")
        return None







class RobotController:
    def __init__(self):
        self.controller = controller.Controller()
        self.x = 75
        self.y = 25

class Capture_drapeau:
    def __init__(self, robot_controller: RobotController, server_address="proj103.r2.enst.fr"):
        self.server_address = server_address
        self.robot_controller = robot_controller
        self.letters = ['F', 'E', 'D', 'C', 'B', 'A']
        self.cap = cv2.VideoCapture(0)  # Ouvrir la caméra


    def start_race(self) -> bool:
        """
        Démarre la course en envoyant une requête au serveur
        """
        try:
            response = requests.post(
                f"http://{self.server_address}/api/start",
                timeout=2
            )
            if response.status_code == 200:
                print("Course démarrée avec succès!")
                return True
            elif response.status_code == 400:
                print("La course a déjà démarrée!")
                return True
            else:
                print(f"Erreur lors du démarrage de la course (code {response.status_code}): {response.text}")
                return False
        except requests.RequestException as e:
            print(f"Erreur de connexion lors du démarrage de la course: {e}")
            return False


    def start_end(self) -> bool:
        """
        Démarre la course en envoyant une requête au serveur
        """
        try:
            response = requests.post(
                f"http://{self.server_address}/api/stop",
                timeout=2
            )
            if response.status_code == 200:
                print("\nCourse arreté avec succès!")
            else:
                print(f"Erreur lors du démarrage de la course (code {response.status_code}): {response.text}")
        except requests.RequestException as e:
            print(f"Erreur de connexion lors du démarrage de la course: {e}")


    def xy_to_coordonnees(self) -> Tuple[str, int, str]:
        """
        Convertit les coordonnées x,y du robot en position de case sur la grille
        """
        adjusted_x = self.robot_controller.x // 50
        adjusted_y = self.robot_controller.y // 50
        letter = self.letters[adjusted_x]
        number = adjusted_y + 1
        position = f"{letter}{number}"
        print(position)
        return letter, number, position


    def envoie_marqueur(self,marker_id) -> bool:
        """
        Détecte un marqueur ArUco et l'envoie au serveur de suivi.
        """
        # Vérification si le marqueur détecté est valide et intéressant
        if marker_id is None:  # Aucun marqueur intéressant détecté
            print("Aucun marqueur intéressant à envoyer.")
            return False
        elif marker_id == 0 : 
            print("Je detiens une balise qui pourrait t'interesser")
            return False
    
        try:
            letter, number, position = self.xy_to_coordonnees()
            response = requests.post(
                f"http://{self.server_address}/api/marker",
                params={
                    "id": marker_id,
                    "row": letter,
                    "col": number
                },
                timeout=2
            )
            
            # Gestion de la réponse du serveur
            if response.status_code == 200:
                print(f"Capture du drapeau avec succès : {position} (ID={marker_id})")
                return True
            else:
                print(f"Erreur d'envoi au serveur (code {response.status_code}): {response.text}")
                return False

        except ValueError as e:
            print(f"Erreur de conversion des coordonnées : {e}")
            return False
        except requests.RequestException as e:
            print(f"Erreur de connexion au serveur : {e}")
            return False



if __name__ == "__main__":
    robot_controller = RobotController()
    capture = Capture_drapeau(robot_controller)
    # Démarrer la course
    capture.start_race()
    if capture.start_race():
        try:
            MARQUEUR_DEJA_ENVOYE = [1,2,3,4]
            while True:
                marker_id = detect_marker_id()
                if marker_id is not None and marker_id not in MARQUEUR_DEJA_ENVOYE:
                        print(f"\nLe marqueur {marker_id} a été trouvé")
                        if capture.envoie_marqueur(marker_id) :
                            MARQUEUR_DEJA_ENVOYE.append(marker_id)
                time.sleep(0.05)
        except KeyboardInterrupt:
            cap.release()
            capture.start_end()
            print("\nFin de la détection.")
    else:
        print("Impossible de démarrer la course.")
