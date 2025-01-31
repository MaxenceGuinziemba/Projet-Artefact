import time
import sys
import termios
import tty
import fcntl
import math
import os
import asyncio 



class Awaken(Exception):
    pass

class RobotController:
    def __init__(self):
        self.controller = controller.Controller()
        self.controller.set_motor_shutdown_timeout(2)
        self.speed = 30  # Le nombre de ticks effectués au cours de 10ms
        self.minSpeed = 15
        self.step = 4
        self.transitionTime = 1
        self.entraxe = 17.4
        self.correction = 1 / 1.001
        self.running = True
        self.etat = False
        self.lastDir = "Stop"
        self.diametreRoue = 6.5
        self.x = -25
        self.y = 75
        self.direction = 0  # 0:North, 1:East, 2:South, 3:West
        self.moveDict = {
            "Avancer": [int(self.correction * self.speed), self.speed],
            "Reculer": [-int(self.correction * self.speed), -self.speed],
            "Gauche": [-int(self.correction * self.speed), self.speed],
            "Droite": [int(self.correction * self.speed), -self.speed],
            "Stop": [0, 0]
        }

    def ticks_to_distance(self, ticks) -> float:
        # Conversion des ticks d'encodeur en distance parcourue
        return ticks * math.pi * self.diametreRoue / 3840

    def distance_to_ticks(self, distance) -> int:
        # Conversion de la distance en ticks d'encodeur
        return int(distance * 1.01 * 3820 / (math.pi * self.diametreRoue))
    
    def angle_to_ticks(self, angle_deg) -> int:
        angle_rad = math.radians(angle_deg * 360 / 368)
        wheel_base = self.entraxe
        distance = (wheel_base * angle_rad) / 2
        return self.distance_to_ticks(distance)

    def stop(self):
        self.move("Stop")
    
    def move_forward(self):
        self.move("Avancer")

    def move_backward(self):
        self.move("Reculer")

    def turn_left(self):
        self.move("Gauche")

    def turn_right(self):
        self.move("Droite")

    def move(self, direction):
        print(direction, "\n")
        self.controller.set_motor_speed(self.moveDict[direction][0], self.moveDict[direction][1])
        self.lastDir = direction
    
    def update_position(self, distance):
        if self.direction == 0:  # North
            self.x += distance
        elif self.direction == 1:  # East
            self.y += distance
        elif self.direction == 2:  # South
            self.x -= distance
        elif self.direction == 3:  # West
            self.y -= distance
    
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
        print("Déplacement de", target_distance, "cm\n")

        # Coefficients PID (à ajuster selon votre système)
        Kp = 0.68 # Gain proportionnel

        max_speed = self.speed

        # Réinitialiser les encodeurs
        self.controller.get_encoder_ticks()

        # Convertir la distance cible en ticks d'encodeur
        target_ticks = self.distance_to_ticks(target_distance)

        self.controller.set_motor_speed(25, 0)
        await asyncio.sleep(0.05)

        lcount = 0
        rcount = 0

        # Boucle de contrôle
        while True:
            # Lire les valeurs des encodeurs
            left_ticks, right_ticks = self.controller.get_encoder_ticks()
            lcount += left_ticks
            rcount += right_ticks
            # Calculer le nombre de ticks parcourus (moyenne des deux encodeurs)
            distance_parcourus = self.ticks_to_distance((left_ticks+right_ticks)/2)

            relative_error = lcount - rcount
            lerror = target_ticks - lcount
            rerror = target_ticks - rcount

            # Condition d'arrêt si l'erreur est inférieure à la tolérance
            if max(abs(lerror),abs(rerror)) < self.distance_to_ticks(0.7) or abs(lcount) > abs(target_ticks):  # Tolérance réduite pour plus de précision
                print("Distance cible atteinte.")
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
            routput = max(min(routput, dyn_speed_right), -max(routput,dyn_speed_right)) + 0.7 * relative_error

            # Définir la vitesse des moteurs avec correction
            self.controller.set_motor_speed(int(loutput*self.correction), int(routput))
            self.update_position(distance_parcourus)
            # Petite pause pour éviter de surcharger le processeur
            await asyncio.sleep(0.03)

        # Arrêter les moteurs une fois la distance atteinte
        self.stop()
        await asyncio.sleep(0.1)
        print(lcount, rcount)

    async def tourne_angle(self, target_angle):
        print("Tourne de", target_angle, "deg\n")
        self.direction = (self.direction + target_angle // 90) % 4
        # Coefficients PID (à ajuster selon votre système)
        Kp = 0.68  # Gain proportionnel

        max_speed = self.speed

        # Réinitialiser les encodeurs
        self.controller.get_encoder_ticks()

        # Convertir la distance cible en ticks d'encodeur
        target_ticks = self.angle_to_ticks(target_angle)

        self.controller.set_motor_speed(25 if target_angle > 0 else -25, 0)
        await asyncio.sleep(0.05)

        lcount = 0
        rcount = 0

        # Boucle de contrôle
        while True:
            # Lire les valeurs des encodeurs
            left_ticks, right_ticks = self.controller.get_encoder_ticks()
            lcount += left_ticks
            rcount += right_ticks
            # Calculer le nombre de ticks parcourus (moyenne des deux encodeurs)
            lerror = target_ticks - lcount
            rerror = -target_ticks - rcount
            # Condition d'arrêt si l'erreur est inférieure à la tolérance
            if max(abs(lerror),abs(rerror)) < self.distance_to_ticks(0.49) or abs(lcount) > abs(target_ticks):  # Tolérance réduite pour plus de précision
                print("Distance cible atteinte.")
                break

            # Calculer la sortie PID
            loutput = Kp * lerror
            routput = Kp * rerror

            # Limiter la sortie pour assurer une décélération progressive
            # La vitesse maximale autorisée diminue à l'approche de la cible
            
            dyn_speed_right = max_speed * (rerror / target_ticks)
            dyn_speed_left = max_speed * (lerror / target_ticks)
            """
            if dyn_speed_left > 0:
                dyn_speed_left = max(dyn_speed_left, self.minSpeed)
            else:
                dyn_speed_left = min(dyn_speed_left, -self.minSpeed)
            if dyn_speed_right > 0:
                dyn_speed_right = max(dyn_speed_right, self.minSpeed)
            else:
                dyn_speed_right = min(dyn_speed_right, -self.minSpeed)
            """
            # Limiter la sortie aux valeurs maximales dynamiques
            loutput = max(min(loutput, dyn_speed_left), -max(loutput,dyn_speed_left))
            routput = max(min(routput, dyn_speed_right), -max(routput,dyn_speed_right))

            # Définir la vitesse des moteurs avec correction
            self.controller.set_motor_speed(int(loutput*self.correction), -int(routput))

            # Petite pause pour éviter de surcharger le processeur
            await asyncio.sleep(0.05)

        # Arrêter les moteurs une fois la distance atteinte
        self.stop()
        await asyncio.sleep(0.2)
        print(lcount, rcount)

async def test():
    robot = RobotController()  # Using your Robot class instead of RobotController
    try:
        robot.start_sending_coordinates()
        for _ in range(4):
            await robot.move_distance(50)
            for _ in range(4):
                await robot.tourne_angle(89.2)
                await asyncio.sleep(0.05)
    finally:
        robot.stop_sending_coordinates()

if __name__ == "__main__":
    import controller
    import time
    import requests
    import threading
    import asyncio
    asyncio.run(test())
else:
    from moteurs import controller
