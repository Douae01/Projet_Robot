# Projet Robot

## Modes de fonctionnement à programmer pour le robot

- **Mode Manuel** : Permet le contrôle à distance de la direction et de la vitesse de déplacement du robot, avec détection des obstacles.
- **Mode Aléatoire** : Le robot se déplace librement en changeant régulièrement de direction, tout en détectant et en évitant les obstacles.
- **Mode Suivi (Tracking)** : Le robot suit une cible de couleur détectée par la caméra.


## Composants utilisés dans ce projet

- **STM32 Nucleo F411** : responsable sur la gestion de la commande des moteurs en boucle fermée, l'acquisition des capteurs de distance, l'affichage sur l'écran LCD (Mode de fonctionnement 'Manuel/Aléatoire/Suivi') et la communication avec le Raspberry Pi.

- **Raspberry Pi** : permettant l'acquisition de l'image de la Webcam, le traitement de l'image de la Webcam et la communication avec le PC via la liaison WIFI.

- **IHM** : mise en place pour définir les modes de fonctionnement du robot, transmettre les ordres de direction et de vitesse, afficher la valeur des capteurs et l'image de la Webcam.
