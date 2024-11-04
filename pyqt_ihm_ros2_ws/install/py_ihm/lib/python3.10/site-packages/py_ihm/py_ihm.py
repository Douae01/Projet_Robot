#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QLabel, QLineEdit, QPushButton, QVBoxLayout, QComboBox, QHBoxLayout, QGridLayout

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
            QPushButton#btnDown { background-color: #FF5733; color: white; }
            QPushButton#btnLeft { background-color: #FFC300; color: white; }
            QPushButton#btnRight { background-color: #337ab7; color: white; }
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

        # Label et ComboBox pour le mode de fonctionnement
        self.mode_label = QLabel('Choix du Mode de Fonctionnement du Robot:', self)
        self.mode_combo = QComboBox(self)
        self.mode_combo.addItems(['Manuel', 'Aléatoire', 'Suivi'])
        self.mode_combo.currentIndexChanged.connect(self.onModeChange)

        # Champ pour la saisie de la vitesse
        self.speed_label = QLabel('Entrez la vitesse :', self)
        self.speed_input = QLineEdit(self)
        self.speed_input.setPlaceholderText('Vitesse en m/s')

        # Bouton pour envoyer la vitesse
        self.send_speed_button = QPushButton('Envoyer la vitesse', self)
        self.send_speed_button.clicked.connect(self.onSendSpeed)

        # Layout pour les contrôles principaux (mode et vitesse) à gauche
        left_layout = QVBoxLayout()
        left_layout.addWidget(self.mode_label)
        left_layout.addWidget(self.mode_combo)
        left_layout.addWidget(self.speed_label)
        left_layout.addWidget(self.speed_input)
        left_layout.addWidget(self.send_speed_button)

        # Boutons de contrôle de mouvement
        self.btn_forward = QPushButton('Avancer', self)
        self.btn_forward.setObjectName("btnUp")
        self.btn_forward.clicked.connect(self.onMoveForward)

        self.btn_backward = QPushButton('Reculer', self)
        self.btn_backward.setObjectName("btnDown")
        self.btn_backward.clicked.connect(self.onMoveBackward)

        self.btn_left = QPushButton('Gauche', self)
        self.btn_left.setObjectName("btnLeft")
        self.btn_left.clicked.connect(self.onTurnLeft)

        self.btn_right = QPushButton('Droite', self)
        self.btn_right.setObjectName("btnRight")
        self.btn_right.clicked.connect(self.onTurnRight)

        # Disposition en grille pour les boutons de mouvement (comme une manette)
        movement_layout = QGridLayout()
        movement_layout.addWidget(self.btn_forward, 0, 1)  # Haut
        movement_layout.addWidget(self.btn_left, 1, 0)     # Gauche
        movement_layout.addWidget(self.btn_right, 1, 2)    # Droite
        movement_layout.addWidget(self.btn_backward, 2, 1) # Bas

        # Ajout du layout des boutons dans un conteneur à droite
        right_widget = QWidget()
        right_widget.setLayout(movement_layout)

        # Disposition principale : contenu à gauche et contrôles de mouvement à droite
        main_layout = QHBoxLayout()
        main_layout.addLayout(left_layout)  # Contenu principal à gauche
        main_layout.addWidget(right_widget) # Manette à droite

        # Configuration du widget principal
        widget = QWidget()
        widget.setLayout(main_layout)
        self.setCentralWidget(widget)

        # Initialisation de ROS2
        rclpy.init(args=None)
        self.node = Node('py_ihm_node')
        self.publisher_ = self.node.create_publisher(String, 'pyqt_topic_send', 10)
        self.subscription = self.node.create_subscription(String, 'pyqt_topic_rec', self.listener_callback, 10)
        self.subscription  # empêche un avertissement de variable inutilisée

        # Timer pour les événements ROS
        self.timer = QTimer()
        self.timer.timeout.connect(self.onTimerTick)
        self.timer.start(5)
        
        # Compteur de messages
        self.i = 0

    def onSendSpeed(self):
        # Envoi de la vitesse entrée par l'utilisateur
        speed = self.speed_input.text()
        if speed:
            msg = String()
            msg.data = f'VITESSE:{speed}'
            self.publisher_.publish(msg)
            print(f'Vitesse envoyée : "{speed}" -> Publishing: "{msg.data}"')
        else:
            print("Aucune vitesse entrée.")

    def onModeChange(self):
        mode = self.mode_combo.currentText()
        msg = String()
        if mode == "Manuel":
            msg.data = 'MANUAL_MODE'
            self.enable_manual_mode()
        elif mode == "Aléatoire":
            msg.data = 'RANDOM_MODE'
            self.enable_random_mode()
        elif mode == "Suivi":
            msg.data = 'TRACKING_MODE'
            self.enable_tracking_mode()
        self.publisher_.publish(msg)
        print(f'Mode changé : "{mode}" -> Publishing: "{msg.data}"')

    def enable_manual_mode(self):
        print("Le mode Manuel est activé.")

    def enable_random_mode(self):
        print("Le mode Aléatoire est activé.")

    def enable_tracking_mode(self):
        print("Le mode Suivi est activé.")

    # Méthodes pour les contrôles de mouvement
    def onMoveForward(self):
        msg = String()
        msg.data = 'FORWARD'
        self.publisher_.publish(msg)
        print('Commande de mouvement: Avancer')

    def onMoveBackward(self):
        msg = String()
        msg.data = 'BACKWARD'
        self.publisher_.publish(msg)
        print('Commande de mouvement: Reculer')

    def onTurnLeft(self):
        msg = String()
        msg.data = 'TURN_LEFT'
        self.publisher_.publish(msg)
        print('Commande de mouvement: Gauche')

    def onTurnRight(self):
        msg = String()
        msg.data = 'TURN_RIGHT'
        self.publisher_.publish(msg)
        print('Commande de mouvement: Droite')

    def listener_callback(self, msg):
        print('I heard: "%s"' % msg.data)

    def onTimerTick(self):
        rclpy.spin_once(self.node, executor=None, timeout_sec=0.01)

########################################################################
def main(args=None):
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()
########################################################################
