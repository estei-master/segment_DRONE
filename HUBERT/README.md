![ESTEI](https://raw.github.com/estei-master/segment_SOL/master/PJ/Slide/common/estei.png)

Système de contrôle du drone
=

Gère le drone du démarrage à l'arrêt, basé sur le noyau temps réel FreeRTOS.

Contenu
-

Versions :
* drone v0.31 : drone avant intégration
* drone v0.32 : intégration en cours

Fichiers :
* imu.* : code spécifique à l'IMU
* zigbee.* : code spécifique à la communication ZigBee. Certaines fonctions sont également définies dans drone.c (prvSendError(), prvSendStatus() et prvSendConfig())
* flight.* : code spécifique au contrôle de vol
* pid.* : code spécifique à l'asservissement moteur bas niveau. Inclus dans flight.c
* pwm.* : code spécifique à la génération de signaux PWM pour les moteurs. Inclus dans pid.c
* drone.* : Programme principal, définissant et gérant les tâches FreeRTOS
* common.h : Inclusion centralisé des headers partagés (ceux du stm32f4, FreeRTOS, debug...)
* debug.* : code de debug, activant la trace sur l'UART6 ou par semihosting

Intégré
-

* IMU (angles, altitude)
* Télémètres (à tester)
* Batterie (à tester)

À intégrer
-

* GPS
* Gestion moteur
* Communication ZigBee
* Niveau RSSI ZigBee

Fixme
-

* Tâche GPS, droneState utilisé avant récupération.
