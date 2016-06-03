# Walking Machine follow_someone node

## Description
Node qui lance le service follow_someone qui attend de recevoir 1 pour suivre la personne la plus proche (pour l'instant) <br />
Écoute également le topic /people_tracker_measurements du package afin de voir les personnes détectées

## Dépendances
roslaunch wm_2dnav move_base.launch <br />
roslaunch wm_2dnav sara_configuration.launch <br />
roslaunch people leg_detector.launch

## Service /follow_someone
0 : Arrête de suivre la personne <br />
1 : Commence à suivre la personne
