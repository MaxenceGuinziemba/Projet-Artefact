"""
Created on Fri Jan 31 17:53:06 2025

@author: maxence
"""

from time import sleep
import multiprocessing
import requests
import json
import time
import sys
import termios
import tty
import fcntl
import math
import os
import asyncio 
import threading
import cv2
import numpy as np
import controller  # Import du module controller
from typing import Tuple, Optional


class Awaken(Exception):
    pass


class RobotController:
    def __init__(self):
        self.controller = controller.Controller()
        self.controller.set_motor_shutdown_timeout(2)
        self.speed = 30 # Le nombre de ticks effectués au cours de 10ms
        self.minSpeed = 15
        self.step = 4
        self.transitionTime = 1
        self.entraxe = 18
        self.correction = 1 / 1.001
        self.running = True
        self.etat = False
        self.lastDir = "Stop"
        self.diametreRoue = 6.5
        self.x = -25
        self.y = 75
        self.direction = 0 
        self.moveDict = {
            "Avancer": [int(self.correction * self.speed), self.speed],
            "Reculer": [-int(self.correction * self.speed), -self.speed],
            "Gauche": [-int(self.correction * self.speed), self.speed],
            "Droite": [int(self.correction * self.speed), -self.speed],
            "Stop": [0, 0]
        }

    def ticks_to_distance(self, ticks) -> float:
        return ticks * math.pi * self.diametreRoue / 3840

    def distance_to_ticks(self, distance) -> int:
        return int(distance * 3820 / (math.pi * self.diametreRoue))
    
    def angle_to_ticks(self, angle_deg) -> int:
        angle_rad = math.radians(angle_deg * 360 / 368)
        wheel_base = self.entraxe
        distance = (wheel_base * angle_rad) / 2
        return self.distance_to_ticks(distance)

    def stop(self):
        self.move("Stop")
        
    def move(self, direction):
        print(direction, "\n")
        self.controller.set_motor_speed(self.moveDict[direction][0], self.moveDict[direction][1])
        self.lastDir = direction
    
    def update_position(self, distance):
        diagonal_distance = distance / 2**0.5  
        if self.direction == 0:  # Nord
            self.x += distance
        elif self.direction == 1:  # Nord-Est
            self.y += diagonal_distance
            self.x += diagonal_distance
        elif self.direction == 2:  # Est
            self.y += distance
        elif self.direction == 3:  # Sud-Est
            self.y += diagonal_distance
            self.x -= diagonal_distance
        elif self.direction == 4:  # Sud
            self.x -= distance
        elif self.direction == 5:  # Sud-Ouest
            self.y -= diagonal_distance
            self.x -= diagonal_distance
        elif self.direction == 6:  # Ouest
            self.y -= distance
        elif self.direction == 7:  # Nord-Ouest
            self.y -= diagonal_distance
            self.x += diagonal_distance
    
    def send_coordinates(self):
        try:
            server_url = "http://proj103.r2.enst.fr/api/pos"
            params = {'x': self.y, 'y': self.x}
            requests.post(server_url, params=params)
            print(f"Position envoyée : x={self.x}, y={self.y}","\n")
        except Exception as e:
            print(f"Erreur d'envoi : {str(e)}")

    def continuous_send_coordinates(self):
        while self.etat:
            self.send_coordinates()
            time.sleep(1)

    def start_sending_coordinates(self):
        self.etat = True
        thread_send = threading.Thread(target=self.continuous_send_coordinates)
        thread_send.daemon = True
        thread_send.start()

    def stop_sending_coordinates(self):
        self.send_coordinates()
        self.etat = False



    async def move_distance(self, target_distance):
        #print("Déplacement de", target_distance, "cm\n")
        await asyncio.sleep(0.6)
        # Coefficients PID (à ajuster selon l'humeur du système)
        Kp = 0.7 # Gain proportionnel

        max_speed = self.speed
        # Réinitialiser les encodeurs
        self.controller.get_encoder_ticks()

        # Convertir la distance cible en ticks d'encodeur
        target_ticks = self.distance_to_ticks(-target_distance)

        lcount = 0
        rcount = 0

        while True:
            left_ticks, right_ticks = self.controller.get_encoder_ticks()
            lcount += left_ticks
            rcount += right_ticks
            # Calculer le nombre de ticks parcourus (moyenne des deux encodeurs)
            distance_parcourus = self.ticks_to_distance((left_ticks+right_ticks)/2)

            relative_error = lcount - rcount
            lerror = target_ticks - lcount
            rerror = target_ticks - rcount

            # Condition d'arrêt
            if max(abs(lerror),abs(rerror)) < self.distance_to_ticks(0.8) or abs(lcount) > abs(target_ticks):  # Tolérance réduite  pour plus de précision
                #print("Distance cible atteinte.")
                break

            # Calculer la sortie PID
            loutput = Kp * lerror
            routput = Kp * rerror

            # Limiter la sortie pour assurer une décélération progressive
            # La vitesse maximale autorisée diminue à l'approche de la cible
            dyn_speed_right = max_speed * (rerror / target_ticks)
            dyn_speed_left = max_speed * (lerror / target_ticks)
            
            if dyn_speed_left > 0:
                dyn_speed_left = max(dyn_speed_left, self.minSpeed)
            else:
                dyn_speed_left = min(dyn_speed_left, -self.minSpeed)
            if dyn_speed_right > 0:
                dyn_speed_right = max(dyn_speed_right, self.minSpeed)
            else:
                dyn_speed_right = min(dyn_speed_right, -self.minSpeed)
            
            # Limiter la sortie aux valeurs maximales dynamiques
            loutput = max(min(loutput, dyn_speed_left), -max(loutput,dyn_speed_left))
            routput = max(min(routput, dyn_speed_right), -max(routput,dyn_speed_right))

            loutput = math.copysign(max(abs(loutput), self.minSpeed), loutput)
            routput = math.copysign(max(abs(routput), self.minSpeed), routput) + 0.7 * relative_error

            loutput = math.copysign(min(abs(loutput), self.speed), loutput + 10)
            routput = math.copysign(min(abs(routput), self.speed), routput + 10)

            if (abs(loutput) > 35) or (abs(routput) > 35) :
                print("Erreur : ", loutput, routput)

            # Définir la vitesse des moteurs avec correction
            self.controller.set_motor_speed(int(loutput*self.correction), int(routput))
            self.update_position(-distance_parcourus)
            # Petite pause pour éviter de surcharger le processeur
            await asyncio.sleep(0.03)

        # Arrêter les moteurs une fois la distance atteinte
        self.stop()
        await asyncio.sleep(0.1)
        #print(lcount, rcount)

    async def tourne_angle(self, angle):
        target_angle = - angle
        print("Tourne de", target_angle, "deg\n")
        self.direction = (self.direction - target_angle // 45) % 8

        await asyncio.sleep(0.6)
        Kp = 0.68  # Gain proportionnel

        max_speed = self.speed + 5 

        # Réinitialiser les encodeurs
        self.controller.get_encoder_ticks()

        # Convertir la distance cible en ticks d'encodeur
        target_ticks = self.angle_to_ticks(-target_angle)

        lcount = 0
        rcount = 0
        relative_error = 0

        # Boucle de contrôle
        while True:
            # Lire les valeurs des encodeurs
            left_ticks, right_ticks = self.controller.get_encoder_ticks()
            lcount += left_ticks
            rcount += right_ticks
            # Calculer le nombre de ticks parcourus (moyenne des deux encodeurs)

            relative_error = lcount + rcount
            lerror = target_ticks - lcount
            rerror = -target_ticks - rcount

            # Condition d'arrêt si l'erreur est inférieure à la tolérance
            if max(abs(lerror),abs(rerror)) < self.distance_to_ticks(0.5) or abs(lcount) > abs(target_ticks):  # Tolérance réduite pour plus de précision
                break

            # Calculer la sortie PID
            loutput = Kp * lerror
            routput = Kp * rerror

            # Limiter la sortie pour assurer une décélération progressive
            # La vitesse maximale autorisée diminue à l'approche de la cible
            dyn_speed_right = max_speed * (rerror / target_ticks)
            dyn_speed_left = max_speed * (lerror / target_ticks)
            
            
            if dyn_speed_left > 0:
                dyn_speed_left = max(dyn_speed_left, self.minSpeed)
            else:
                dyn_speed_left = min(dyn_speed_left, -self.minSpeed)
            if dyn_speed_right > 0:
                dyn_speed_right = max(dyn_speed_right, self.minSpeed)
            else:
                dyn_speed_right = min(dyn_speed_right, -self.minSpeed)
            
            # Limiter la sortie aux valeurs maximales dynamiques
            loutput = max(min(loutput, dyn_speed_left), -max(loutput,dyn_speed_left))
            routput = max(min(routput, dyn_speed_right), -max(routput,dyn_speed_right))

            loutput = math.copysign(max(abs(loutput), self.minSpeed), loutput)
            routput = math.copysign(max(abs(routput), self.minSpeed), routput) + 0.7 * relative_error

            loutput = math.copysign(min(abs(loutput), self.speed), loutput + 10)
            routput = math.copysign(min(abs(routput), self.speed), routput + 10)

            if (abs(loutput) > 35) or (abs(routput) > 35) :
                print("Erreur : ", loutput, routput)
                
            # Définir la vitesse des moteurs avec correction
            self.controller.set_motor_speed(int(loutput*self.correction), -int(routput))

            # Petite pause pour éviter de surcharger le processeur
            await asyncio.sleep(0.05)
        self.stop()
        await asyncio.sleep(0.2)




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

def check_flag():
    liste = []
    for i in range(2):
        L = detect_marker_id()
        liste = liste + L
    return liste

def detect_marker_id():
    """
    Détecte les marqueurs ArUco et renvoie l'ID du  marqueur.
    """
    camera_matrix = np.array([[1781.52030,0,638.679423],[0,1788.36013,486.627951],[0,0,1]])
    dist_coeffs = np.array([-0.0168229519,2.85496044,0.0121252479,-0.00411669324,-14.5949808])
    if not cap.isOpened():
        print("Erreur : Impossible d'ouvrir la caméra.")
        return []
    # Lire une image du flux vidéo
    ret, frame = cap.read()
    if not ret:
        print("Erreur de lecture du flux vidéo.")
        cap.release()
        return []
    
    # Convertir l'image en niveaux de gris
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Détecter les marqueurs ArUco
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    # Si des marqueurs sont détectés
    
    if ids is not None and len(ids) > 0:
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.011, camera_matrix, dist_coeffs)
        L = []
        for i, _ in enumerate(ids):
            tvec = tvecs[i]*100
            rvec = rvecs[i]
            distance = tvec[0][2]
            if (ids[i][0] > 4 or ids[i][0] == 0) and distance < 40 :
                print("La distance à la balise est de : ", distance)
                L.append([int(ids[i][0]), rvec.tolist(), tvec.tolist()])
        return L
    return []




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
        letter = self.letters[int(adjusted_x)]
        number = adjusted_y + 1
        position = f"{letter}{number}"
        print("La balise est sur la case",position)
        return letter, number, position


    def envoie_marqueur(self,marker_id) -> bool:
        """
        Détecte un marqueur ArUco et l'envoie au serveur de suivi.
        """
        # Vérification si le marqueur détecté est valide et intéressant
        print("Je procéde à l'envoie du marker")
        if marker_id is None:  # Aucun marqueur intéressant détecté
            print("Aucun marqueur intéressant à envoyer.")
            return False
        elif marker_id == 0 or marker_id == 1 or marker_id == 2 or marker_id == 3 or marker_id == 4 : 
            print("Je detiens une balise mais qui t'interesse pas")
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





ip = "http://137.194.13.177:8080"
time_regu_envoie = 0.2


id = 1 #TODO: Mettre id de l'equipe


def send_request(request, content_json=None, recup=False):
    """
    Envoie une requête POST à une URL spécifiée et gère les réponses ainsi que les erreurs.

    Cette fonction permet d'envoyer une requête HTTP POST à une API, avec ou sans contenu JSON, 
    et de récupérer la réponse sous forme de dictionnaire JSON si demandé.

    Paramètres :
        request (str) : L'URL de la requête. Elle doit être valide et accessible.
        content_json (dict, optionnel) : Contenu JSON à envoyer avec la requête. Par défaut, aucun contenu n'est envoyé.
        recup (bool, optionnel) : Indique si la réponse doit être récupérée et retournée sous forme de JSON.
                                  - Si `True`, tente de convertir la réponse en dictionnaire JSON.
                                  - Si `False`, ne retourne que des métadonnées vides en cas de succès.

    Retourne :
        dict : 
            - Si `recup` est `True` et la réponse est au format JSON, retourne un dictionnaire contenant les données de la réponse.
            - Si `recup` est `False`, retourne un dictionnaire vide (`{}`) après le succès de la requête.
        None :
            - En cas d'erreur (URL manquante, requête échouée, ou réponse non JSON si `recup` est activé).

    Exceptions gérées :
        - `requests.exceptions.HTTPError` : En cas de code d'erreur HTTP (par exemple, 404 ou 500).
        - `requests.exceptions.RequestException` : En cas de problème de connexion ou autre erreur de requête.
        - `ValueError` : Si `recup` est activé mais que la réponse n'est pas au format JSON.

    Fonctionnement :
        1. Vérifie si une URL est spécifiée. Si ce n'est pas le cas, retourne `None` avec un message d'erreur.
        2. Envoie une requête POST à l'URL spécifiée :
            - Si `content_json` est fourni, il est ajouté au corps de la requête en tant que JSON.
            - Sinon, une requête POST vide est envoyée.
        3. Gère les exceptions et affiche les erreurs en cas de problème.
        4. Si `recup` est activé, tente de convertir la réponse en JSON et retourne les données sous forme de dictionnaire.

    Exemple d'utilisation :
        >>> # Envoi d'une requête simple sans contenu JSON
        >>> send_request("http://localhost:8080/api/start")
        {}

        >>> # Envoi d'une requête avec un contenu JSON
        >>> send_request("http://localhost:8080/api/udta", content_json={"key": "value"})
        {}

        >>> # Récupération d'une réponse JSON
        >>> send_request("http://localhost:8080/api/status", recup=True)
        {"status": "running", "uptime": 120}

    Notes :
        - La réponse est considérée comme JSON uniquement si `recup=True` est spécifié.
        - La fonction gère automatiquement les erreurs HTTP et les problèmes de connexion réseau.
        - Si la réponse JSON est mal formatée ou absente, un message d'erreur est affiché, et `None` est retourné.
    """

    if not request:
        print("Erreur: URL de requête non spécifiée.")
        return None

    try:
        # Envoi de la requête avec ou sans contenu JSON
        response = requests.post(
            request, json=content_json) if content_json else requests.post(request)
        response.raise_for_status()  # Lève une exception pour les codes d'erreur HTTP

    except requests.exceptions.HTTPError as e:
        print(f"Erreur HTTP {response.status_code}: {response.text}")
        return None
    except requests.exceptions.RequestException as e:
        print("Erreur de requête:", e)
        return None

    # Gestion des cas de récupération des données
    if recup:
        try:
            data = response.json()
        except ValueError:
            print("Erreur: La réponse n'est pas en JSON.")
            return None
        return data

    return {}


async def send_drap(liste):
    # liste [ [id_balise, r_vec, t_vec], ... ]
    if liste != 0 :
        print("La liste que j'envoie à Alexis ressemble à ça :\n",liste)
    request = ip + f"/api/drap?id={id}&list={liste}"
    #content_json = json.dumps(liste)
    send_request(request,recup = False)
    await asyncio.sleep(0.5)

async def envoie_regu(temps):
    while True:
        liste = [] #TODO:renvoyer la liste de liste de la forme : [[id,rvec,tvec], ...] apres analyse de camera.
        send_drap(liste)
        await asyncio.sleep(temps)


def send_check():
    request = ip + f"/api/check?id={id}"
    commande = send_request(request, content_json=False, recup=True)
    # commande est un tableau 
    # [ "A", None ] correspond à avancer(50)
    # [ "T", theta ] correspond à tourner(theta) en degré
    # [ "C", id ] correspond à capturer la balise id
    # [ "R", None ] correspond à ne rien faire

    # Partie à modifier
    return commande


time_regu_envoie = 0.2

async def async_strat_groupe(x,y,t0=0):
    theta = 0
    robot_controller = RobotController()
    robot_controller.start_sending_coordinates()
    capture = Capture_drapeau(robot_controller)
    while True:
        await send_drap(check_flag())
        command = send_check()
        print("Voici la commande que je reçois du serveur commun",command,"\n")
        match command[0]:
            case 'A': #avancer
                await robot_controller.move_distance(50)
                await asyncio.sleep(0.1)
            case 'T': #tourner
                await robot_controller.tourne_angle(-command[1])
                await asyncio.sleep(0.1)
            case 'C': 
                print("Robot capture")
                capture.envoie_marqueur(command[1])
                for _ in range (8):
                    await robot_controller.tourne_angle(45)
                    await asyncio.sleep(0.1)
            case _: 
                await asyncio.sleep(1)

def strat_groupe(x,y,t0=0):
    asyncio.run(async_strat_groupe(x,y,t0))


def automatique(x_dep,y_dep):
    #thread_auto = multiprocessing.Process(target=envoie_regu,args = (time_regu_envoie,))
    #thread_auto.start()
    strat_groupe(x_dep, y_dep)
    # na.capture_flag2_v2(x,theta,theta,addr, arucoDict, arucoParam)
    #if not mode_test:
    #    request = ua.forge_request(serveur=addr, stop=True)
    #    ua.send_request(request=request)

if __name__ == "__main__":
    automatique(0,0)
