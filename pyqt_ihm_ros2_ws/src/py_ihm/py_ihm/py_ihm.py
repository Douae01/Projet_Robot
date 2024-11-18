#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication,QSizePolicy,QSpacerItem, QMainWindow, QWidget, QLabel, QLineEdit, QPushButton, QVBoxLayout, QComboBox, QHBoxLayout, QGridLayout

########################################################################

class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)

        # Configuration de l'IHM
        self.setMinimumSize(600, 600)
        self.setWindowTitle("ROS2 IHM")
        self.setStyleSheet("""
            QMainWindow {
                background-color: #f4f4f9;
            }
            QLabel {
                font-size: 16px;
            }
            QLineEdit {
                font-size: 14px;
                padding: 5px;
                border: 1px solid #ccc;
                border-radius: 5px;
                background-color: #fff;
            }
            QPushButton {
                font-size: 12px;
                padding: 6px;
                border: none;
                border-radius: 5px;
                background-color: #0056b3;
            }
            QPushButton#btnUp { background-color: #4CAF50; color: white; }
            QPushButton#btnDown { background-color: #8A2BE2; color: white; }
            QPushButton#btnLeft { background-color: #FFC300; color: white; }
            QPushButton#btnRight { background-color: #337ab7; color: white; }
            QPushButton#btnStop { background-color: #FF5733; color: white; }
            # QPushButton:hover {
                
            # }
            QComboBox {
                font-size: 14px;
                padding: 5px;
                border: 1px solid #ccc;
                border-radius: 5px;
                background-color: #fff;
            }
        """)

        # Mode de fonctionnement et obstacles
        self.current_mode = "Manuel"
        self.obstacle_status = "Aucun"

        # Labels pour afficher les données du robot
        self.wheel_speed_label = QLabel('Vitesse de rotation des roues: 0 tr/min', self)
        self.wheel_speed_display=QLineEdit()
        self.wheel_speed_display.setReadOnly(True)

        self.robot_speed_label = QLabel('Vitesse du robot: m/s et km/h', self)
        self.robot_speed_display=QLineEdit()
        self.robot_speed_display.setReadOnly(True)

        self.obstacle_label = QLabel('Présence d\'obstacle: ', self)
        self.obstacle_display=QLineEdit()
        self.obstacle_display.setReadOnly(True)

        # Label et ComboBox pour le mode de fonctionnement
        self.mode_label = QLabel('Choix du Mode de Fonctionnement du Robot:', self)
        self.mode_combo = QComboBox(self)
        self.mode_combo.addItems(['Manuel', 'Aléatoire', 'Suivi'])
        self.mode_combo.currentIndexChanged.connect(self.onModeChange)

        # Champ pour la saisie de la vitesse
        self.speed_label = QLabel('Entrez la vitesse :', self)
        self.speed_input = QLineEdit(self)
        self.speed_input.setPlaceholderText('Vitesse en m/s')
        self.send_speed_button = QPushButton('Envoyer la vitesse', self)
        self.send_speed_button.clicked.connect(self.onSendSpeed)

        # Boutons de contrôle de mouvement
        self.movement_label = QLabel('Contrôle de Mouvement:')
        self.movement_layout = QGridLayout()

        self.btn_forward = QPushButton('Avancer', self)
        self.btn_forward.setObjectName("btnUp")

        self.btn_backward = QPushButton('Reculer', self)
        self.btn_backward.setObjectName("btnDown")

        self.btn_left = QPushButton('Gauche', self)
        self.btn_left.setObjectName("btnLeft")

        self.btn_right = QPushButton('Droite', self)
        self.btn_right.setObjectName("btnRight")

        self.btn_stop = QPushButton('Stop', self)
        self.btn_stop.setObjectName("btnStop")

        self.btn_forward.clicked.connect(lambda: self.send_movement_command("f"))
        self.btn_backward.clicked.connect(lambda: self.send_movement_command("b"))
        self.btn_left.clicked.connect(lambda: self.send_movement_command("l"))
        self.btn_right.clicked.connect(lambda: self.send_movement_command("r"))
        self.btn_stop.clicked.connect(lambda: self.send_movement_command("s"))

        # Disposition en grille pour les boutons de mouvement (comme une manette)
        self.movement_layout.addWidget(self.btn_forward, 0, 1)  # Haut
        self.movement_layout.addWidget(self.btn_left, 1, 0)     # Gauche
        self.movement_layout.addWidget(self.btn_right, 1, 2)    # Droite
        self.movement_layout.addWidget(self.btn_backward, 2, 1) # Bas
        self.movement_layout.addWidget(self.btn_stop, 1, 1) # Bas

        input_layout = QHBoxLayout()
        input_layout.addWidget(self.speed_label)
        input_layout.addWidget(self.speed_input)

        spacer = QSpacerItem(20, 10, QSizePolicy.Minimum, QSizePolicy.Expanding)

        # Layout principal vertical
        main_layout = QVBoxLayout()
        main_layout.addWidget(self.mode_label)
        main_layout.addWidget(self.mode_combo)
        main_layout.addItem(spacer)
        main_layout.addLayout(input_layout)
        main_layout.addWidget(self.send_speed_button)
        main_layout.addWidget(self.wheel_speed_label)
        main_layout.addWidget(self.wheel_speed_display)
        main_layout.addWidget(self.robot_speed_label)
        main_layout.addWidget(self.robot_speed_display)
        main_layout.addWidget(self.obstacle_label)
        main_layout.addWidget(self.obstacle_display)
        main_layout.addLayout(self.movement_layout)

        # Configuration du widget principal
        widget = QWidget()
        widget.setLayout(main_layout)
        self.setCentralWidget(widget)

        # Initialisation de ROS2
        rclpy.init(args=None)
        self.node = Node('py_ihm_node')
        self.publisher_mode = self.node.create_publisher(String, '/command/mode', 10)
        self.publisher_speed_movement = self.node.create_publisher(String, '/command/move', 10)

        # Subscriptions for robot data
        self.subscription_speed = self.node.create_subscription(String, '/sensor/motor_speed', self.movement_speed_callback, 10)
        self.subscription_obstacle = self.node.create_subscription(String, '/sensor/receive_obstacle', self.obstacle_callback, 10)

        # Timer pour les événements ROS
        self.timer = QTimer()
        self.timer.timeout.connect(self.onTimerTick)
        self.timer.start(100)


    def onModeChange(self):
        mode = self.mode_combo.currentText()
        mode_map = {
            'Manuel': '0',
            'Aléatoire': '1',
            'Suivi': '2'
        }
        msg = String()
        mode_value = mode_map.get(mode, '-1')
        msg.data = mode_value

        self.publisher_mode.publish(msg)
        print(f'Mode changé : "{mode}" -> Publishing Mode: "{msg.data}"')

    def onSendSpeed(self):
        # Envoi de la vitesse entrée par l'utilisateur
        speed = self.speed_input.text()
        if speed:
            msg = String()
            msg.data = f'VITESSE:{speed}'
            self.publisher_speed_movement.publish(msg)
            print(f'Vitesse envoyée : "{speed}" -> Publishing Speed: "{msg.data}"')
        else:
            print("Aucune vitesse entrée.")

    def send_movement_command(self, direction):
        speed = self.speed_input.text() or "100"  # Default speed is 100 if not entered
        if direction == "f":
            msg = String()
            msg.data = f'f{speed}'  # Forward with speed
            self.publisher_speed_movement.publish(msg)
            print(f'Publishing Movement: "{msg.data}"')
        elif direction == "b":
            msg = String()
            msg.data = f'b{speed}'  # Backward with speed
            self.publisher_speed_movement.publish(msg)
            print(f'Publishing Movement: "{msg.data}"')
        elif direction == "l":
            msg = String()
            msg.data = f'l{speed}'  # Left with speed
            self.publisher_speed_movement.publish(msg)
            print(f'Publishing Movement: "{msg.data}"')
        elif direction == "r":
            msg = String()
            msg.data = f'r{speed}'  # Right with speed
            self.publisher_speed_movement.publish(msg)
            print(f'Publishing Movement: "{msg.data}"')
        elif direction == "s":
            msg = String()
            msg.data = f's{speed}'
            self.publisher_speed_movement.publish(msg)
            print(f'Publishing Movement: "{msg.data}"')

    def movement_speed_callback(self, msg):
        """
        Callback pour traiter les messages de vitesse envoyés par le STM32.
        Format attendu : "from STM32 : speed=#<valeur>"
        """
        message_text = msg.data
        print(f"Message reçu : {message_text}")
        
        try:
            # Extraction de la vitesse après "speed=#"
            speed_str = message_text.split("speed=#")[1]
                
            # Conversion de la chaîne en valeur flottante
            speed = float(speed_str.strip())
                
            # Mise à jour de l'affichage dans l'interface utilisateur
            self.robot_speed_display.setText(f'{speed:.2f} m/s')

        except (IndexError, ValueError) as e:
            # Gestion des erreurs d'index ou de conversion
            print(f"Erreur lors de l'analyse des données de vitesse : {e}")

    def obstacle_callback(self, msg):
        message_text = msg.data
        print(f"Message reçu : {message_text}")


        obstacle_str= message_text.split("obstacle detected : #")[1]

        obstacle = int(obstacle_str.strip())

        if obstacle == 0:
            self.obstacle_display.setText("Aucun obstacle")
        elif obstacle == 1:
            self.obstacle_display.setText("Obstacle devant")
        elif obstacle == 2:
            self.obstacle_display.setText("Obstacle derrière")


    def onTimerTick(self):
        rclpy.spin_once(self.node,timeout_sec=0.01)

########################################################################
def main(args=None):
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()
########################################################################
