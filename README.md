# TPS-PI09-repository

Projet Ingénieur 09 Acquisition d'un modèle 3D pour la planification de tâches robotiques en utilisant ROS2

## Requirements

- Ubuntu 22.04.3 LTS
- Distribution de ROS 2 : Humble Hawksbill
 
  ```
  https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
  ```
- Colcon (pour l'installer utiliser la commande ci-dessous)
  ```
  sudo apt install python3-colcon-common-extensions
  ```
- Modèle de caméra : Intel RealSense Depth Camera D435i 
(Les programmes peuvent fonctionner pour toutes les caméras de la série D400)

## Librairies Python nécessaires pour le fonctionnement du programme:

pyrealsense2
open3d
numpy

Les librairies peuvent être télécharger en téléchargeant le fichier requirements.txt puis en entrant la commande suivante dans un terminal : 
```
pip install -r requirements.txt
```

## Ce que le projet permet de faire

L'objectif du projet est de reconstruire une surface de forme convexe. Pour cela il faut utiliser une caméra dont le type est indiqué dans la section Requirements. La caméra filme la surface à reconstruire dans certaines conditions qui sont précisées dans la partie Installation de la caméra. Les différents programmes permettent d'accomplir les tâches suivantes : 
- Récupérer les données RGBD de la caméra à un instant donné et créer un fichier .ply contenant ces données.
- Reconstruire la surface filmée et créer un fichier .stl contenant la surface reconstruite.
Chacune de ces tâches peut être effectuée à l'aide de services ROS.

## Installation de la caméra

La surface à reconstruire doit être placée dans le champ de vue de la caméra :
- La surface doit se trouver à une distance comprise entre 30cm et 1m de la caméra.
- Les objectifs de la caméra doivent être orientés en direction de la surface.
La caméra doit être branchée au PC qui exécute le programme d'acquisition à l'aide du cable USB-C.

  
## Installation des services

- Le service d'acquisition permet de récupérer les données RGBD de la caméra à un instant donné dans un fichier .ply. Ce service prend en demande un entier et envoie comme réponse le même entier. Le fichier .ply est créé dans le dossier Data du répertoire TPS-PI09-repository.
- Le service de reconstuction permet de créer un fichier .stl contenant un modèle 3D créé à partir d'un fichier .ply. Ce service prend en demande un entier et envoie comme réponse le même entier. Pour exécuter ce service, le fichier .ply à reconstruire doit être présent dans le dossier Data du répertoire TPS-PI09-repository. Le fichier .stl est créé dans le dossier Data.
- Le service roboticam3d permet de réaliser une acquisition puis une reconstruction. Ce service prend en demande un entier et envoie comme réponse le même entier. Les fichiers .ply et .stl se trouvent dans le dossier Data.

Pour construire ces services, suivre les étapes suivantes :

- Télécharger le répertoire TPS-PI09-repository à partir de GitHub.
  ```
  git clone https://github.com/hugopgs/TPS-PI09-repository
  ```
- Ouvrir un terminal et se placer dans le répertoire TPS-PI09-repository.
  ```
  cd TPS-PI09-repository
  ```
- Construire les packages.
  ```
  colcon build && colcon build
  ```

## Calibrage de la caméra

Pour calibrer la caméra, vous pouvez exécuter le fichier pyhton rs-imu-calibration.py
```
python rs-imu-calibration.py
```
Suivez ensuite les étapes de ce pdf (4.3 Running the rs-imu-calibration.py): https://www.intelrealsense.com/wp-content/uploads/2020/07/IMU_Calibration_Tool_for_Intel_RealSense-Depth_Cameras_Whitepaper.pdf

## Utilisation des services

### Service d'acquisition

Pour appeler le service d'acquisition, suivre les étapes suivantes : 

- Ouvrir un terminal, se placer dans le répertoire TPS-PI09-repository et sourcer les fichier setup.
  ```
  cd TPS-PI09-repository
  source install/setup.bash
  ```
- Lancer le noeud de service d'acquisition.
  ```
  ros2 run py_acquisition service
  ```
- Ouvrir un autre terminal, se placer dans le répertoire TPS-PI09-repository et sourcer les fichiers setup.
  ```
  cd TPS-PI09-repository
  source install/setup.bash
  ```
- Lancer le noeud client.
  ```
  ros2 run py_acquisition client 1 
  ```
  Le 1 peut être n'importe quel entier.

### Service de reconstruction

Pour appeler le service de reconstruction, suivre les étapes suivantes : 

- Ouvrir un terminal, se placer dans le répertoire TPS-PI09-repository et sourcer les fichier setup.
  ```
  cd TPS-PI09-repository
  source install/setup.bash
  ```
- Lancer le noeud de service de reconstruction.
  ```
  ros2 run py_reconstruction service
  ```
- Ouvrir un autre terminal, se placer dans le répertoire TPS-PI09-repository et sourcer les fichiers setup.
  ```
  cd TPS-PI09-repository
  source install/setup.bash
  ```
- Lancer le noeud client.
  ```
  ros2 run py_reconstruction client 1 
  ```
  Le 1 peut être n'importe quel entier.
  
### Service d'acquisition et de reconstruction

Pour appeler le service d'acquisition et de reconstruction, suivre les étapes suivantes : 

- Ouvrir un terminal, se placer dans le répertoire TPS-PI09-repository et sourcer les fichier setup.
  ```
  cd TPS-PI09-repository
  source install/setup.bash
  ```
- Lancer le noeud de service.
  ```
  ros2 run py_roboticam3d service
  ```
- Ouvrir un autre terminal, se placer dans le répertoire TPS-PI09-repository et sourcer les fichiers setup.
  ```
  cd TPS-PI09-repository
  source install/setup.bash
  ```
- Lancer le noeud client.
  ```
  ros2 run py_roboticam3d client 1 
  ```
  Le 1 peut être n'importe quel entier.
  Le message suivant s'affiche dans le terminal utilisé pour appeler le service.
  ![image](https://github.com/hugopgs/TPS-PI09-repository/assets/148219749/b69153d0-237a-4eeb-bd81-b0e228965d74)

## Programme de comparaison

Le programme de comparaison prend en arguments deux fichiers stl et renvoie un pourcentage de points de correspondance entre ces fichiers. Il est possible d'ajouter une image de l'arrière plan dans lequel l'objet a été photographié comme troisième argument pour enlever le bruit lors de la comparaison.






