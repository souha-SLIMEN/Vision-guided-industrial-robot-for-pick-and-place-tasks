### STRUCTURE GLOBALE

- Le dossier YOLO_server contient le code python "yolo_server.py" ainsi que le
 fichier "best.pt" qu'il utilise pour fournir l'orientation de la piece 

- tous les controllers des differents robots presents dans lenvironnement webots
 sont visibles dans le dossier "controllers" dans "LeProjetSemestriel"

- La conception Solidworks de notre piece est présente dans ce dossier



### LANCEMENT DE LA SIMULATION

1)Ouvrir le dossier "LeprojetSemestriel"

2)Ouvrir le dossier "worlds"

3)Lancer l'environnement "Brouillon_projetSemestriel"

4)Ouvrir Windows Powershell et exécuter les commandes suivantes:

>>cd "... (path de yolo_server)..."
>>python yolo_server.py

5)Lancer la simulation dans Webots




### STRUCTURE DU FICHIER ENTRAINEMENT

- Le dossier Entrainement contient 3 dossiers: Images, Dataset, et Modèles

- Images: il constitue la base de données constitués avec les images prises 
dans webots (un total de 216 images)

- Dataset: il contient les fichiers extraits après l'annotation sur Roboflow

- Modèles: il contient le fichier final d'entrainement "best.pt" qui
constitue notre modèle final contenant les resultats issus de l'entrainement
(en 2 phases) via ultralytics yolo v8 obb  


