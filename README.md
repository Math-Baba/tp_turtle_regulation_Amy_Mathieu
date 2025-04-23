# Turtle Regulation

Un TP ROS 2 pour déplacer une tortue dans `turtlesim` vers des waypoints

## Fonctionnalités

- Suivi de la position avec `/turtle1/pose`
- Publication de vitesse via `/turtle1/cmd_vel`
- Système de service pour définir des waypoints dynamiquement
- Client qui envoie un autre waypoint une fois la tortue immobile


## Lancer le projet

1. Lancer le `turtlesim` :
   ```bash
   ros2 run turtlesim turtlesim_node
   ```
   
2. Lancer ensuite cette commande dans un deuxième terminal :
   ```bash
   ros2 run turtle_regulation set_way_point
   ```

3. Dans un troisième terminal, lancer la commande :
   ```bash
   ros2 run turtle_regulation client
   ```



## Partie 1 : 6) Kp — Contrôle de la rotation
### Kp fort :

La tortue tourne très rapidement vers la cible.

Elle peut tourner trop brusquement si elle est très décalée par rapport au waypoint.

Cela peut entraîner des mouvements saccadés ou une instabilité dans l’alignement.

### Kp faible :

La tortue tourne lentement.

L’ajustement de la direction est plus progressif, mais elle risque de prendre trop de temps pour se réaligner.

Cela peut retarder l’arrivée à la cible ou provoquer des trajectoires moins directes.

 
 
## Partie 2 : 5) Kpl — Contrôle de la vitesse linéaire
### Kpl fort :

La tortue se déplace rapidement vers le waypoint.

Cela peut être efficace si l’angle est bien contrôlé.

Mais si Kp est trop faible, la tortue risque d'avancer dans la mauvaise direction ou de rater sa cible (sous-virage).

### Kpl faible :

Le déplacement est plus lent et plus précis.

Permet un contrôle plus fin, surtout utile pour des cibles proches ou des ajustements précis.

Packages
1. turtle_regulation
   
set_way_point.py : joue le rôle de serveur de service et publie les commandes de mouvement (cmd_vel).

client.py : agit comme client du service et définit la séquence de waypoints à envoyer.

### Fonctions clés :

* Publie des messages de type Twist pour contrôler le déplacement de la tortue.

* Utilise un service personnalisé pour recevoir un nouveau waypoint.

* Publie un Booléen (is_moving) pour indiquer si la tortue est encore en mouvement.

2. turtle_interfaces :

Il contient le fichier de service .srv : SetWayPoint.srv, qui définit les champs float64 x, float64 y en entrée, et bool res en réponse.

Ce service est importé à la fois dans publisher.py (serveur) et client.py (client) pour permettre l’échange de données de type waypoint.
