# routeur_le_professolaire

Sources de : https://sites.google.com/view/le-professolaire/routeur-professolaire

## routeur_le_professolaire_v8.03_mathieu

Adaptation qui comprend:

- Ajouts UI:
  * Bouton ON/OFF
  * Bouton Restart
  * Uptime
  * Température et humidité (DHT22)
- Support du MQTT
  * FIX: controle du Retain Flag (qui ne doit pas être à true par défaut, autrement une console de domotique peut lire des paramètres faux et dans le passé)
  * **En lecture**
    - `<base-topi>/enable <0|1>` routeur on (1) ou off (0)
    - `<base-topi>/uptime <number>` uptime (secondes)
    - `<base-topi>/surplus <number>` le surplus virtuel disponible pour un autre gros consommateur qui fonctionne sur le surplus (surplus réseau - routage)
  * **En écriture**
    - `<base-topi>/power2/set <number>` pour envoyer la puissance de surplus à la place d'une pince JSY (exemple: écouter directemnt sur le topic MQTT d'un Shelly EM) 
    - `<base-topi>/enable/set <0|1>` active/désactive le routeur
    - `<base-topi>/restart <1>` redémarre le routeur
  
Un patch est dispo pour voir ou appliquer le diff.
