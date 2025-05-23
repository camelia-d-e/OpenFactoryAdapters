# OpenFactoryIntegration
Pour intégrer des appareils (senseurs, machines, etc) avec OpenFactory utilisant MTConnect, il faut passer par un adapteur si l'appareil que l'on veut surveiller n'est pas déjà fait pour communiquer avec MTConnect. 
![image](https://github.com/user-attachments/assets/1fd54c6e-5c0a-4098-bb16-2fcd3ed83264) 
OpenFactory agit à la fois comme l'agent et l'application. Un adapteur s'assure d'acheminer l'information nécessaire à l'agent dans le bon format. 

## Communication Adapteur/Agent
Pour simplifier, l'adapteur est un serveur qui attend la connection d'un client (agent). La communication entre l'adapteur et l'agent se fait par le protocole de communication SHDR. L'agent s'assure de la connexion à l'adapteur en lui envoyant périodiquement un `* PING`. L'adapteur répond avec un `* PONG HEARTBEAT_TIMEOUT` si la connexion est bien faite (HEARTBEAT_TIMEOUT réfère à la fréquence d'envoi du PING).
### Première connexion
Tout message acheminant de l'information à l'agent est formatté de la façon suivante :
`|nom_variable|valeur\n` ou `timestamp(utc)|nom_variable|valeur\n`.  Lors de la première connexion de l'agent à un adapteur, la premier message envoyé doit indiquer à l'agent si la connexion à l'appareil est fonctionnelle: `|avail|AVAILABLE\n` ou `|avail|UNAVAILBALE\n` (selon l'état de la connexion). Si c'est le cas un message spécifiant l'opérateur de l'appareil peut ensuite être envoyé : `|operator|nom_operateur\n`. Ensuite, pour envoyer divers métadonnées sur l'appareil ou l'adapteur (comme le serialNumber de l'appareil ou la version de l'adapteur, par exemple), l'adapteur doit les envoyer par l'usage de commandes dont le format est le suivant: `* nom_commande: valeur\n`.

Pour plus de détails sur le protocole de communication Adapteur/Agent MTConnect voir ici: [Adapter Agent Protocol](https://www.mtcup.org/Protocol).
