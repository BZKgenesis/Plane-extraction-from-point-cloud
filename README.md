# Visualisation et Simulation de Nuages de Points avec LiDAR

Ce projet permet de simuler et de visualiser des nuages de points générés par un capteur LiDAR. Il inclut des outils pour manipuler les points, simuler des scans, et afficher les données en temps réel à l'aide de Pygame.

## Fonctionnalités

- **Simulation de Nuages de Points** : Génération de points aléatoires pour simuler des données LiDAR.
- **Acquisition de Données Réelles** : Intégration avec un capteur LiDAR via la bibliothèque `RPyLIDAR` fait par Ryan Brazeal. Lien : [RPyLIDAR](https://github.com/ryan-brazeal-ufl/RPyLIDAR).
- **Visualisation en Temps Réel** : Affichage des nuages de points dans une fenêtre graphique Pygame.
- **Utilitaires** : Fonctions pour calculer le centre et la variance des points, interpoler des vecteurs, et plus encore.

## Structure du Projet

- `utils.py` : Contient des fonctions utilitaires pour la manipulation des points et la génération aléatoire.
- `extractPointScan.py` : Gère l'acquisition des données, soit par simulation, soit via un capteur LiDAR réel.
- `main.py` : Script principal pour visualiser les nuages de points.
- `README.md` : Documentation du projet.

## Prérequis

- Python 3.x
- Bibliothèques nécessaires : `pygame`, `pyautogui`, `RPyLIDAR`

Installez les dépendances avec pip :

```bash
pip install pygame pyautogui
```

Pour `RPyLIDAR`, consultez sa [documentation officielle](https://github.com/your-link-here) pour les instructions d'installation.

## Utilisation

1. Clonez le dépôt :

   ```bash
   git clone https://github.com/votre-lien-repo.git
   cd Plane-extraction-from-point-cloud
   ```

2. Lancez le script principal :

   ```bash
   python main.py
   ```

3. Une fenêtre Pygame s'ouvrira pour afficher les nuages de points. Appuyez sur `ESC` pour quitter.

## Personnalisation

- Modifiez la fonction `fakeScan` dans `extractPointScan.py` pour ajuster la simulation des nuages de points.
- Remplacez `fakeScan` par `oneScan` dans `main.py` pour utiliser des données réelles provenant d'un capteur LiDAR.

## Dépannage

- Assurez-vous que votre capteur LiDAR est correctement connecté et configuré si vous utilisez `oneScan`.
- Vérifiez les dépendances et installez les bibliothèques manquantes en cas d'erreurs.

## Utilisation de la Bibliothèque RPyLIDAR

Lien du dépôt : [RPyLIDAR](https://github.com/ryan-brazeal-ufl/RPyLIDAR)

Le script `RPyLIDAR.py` a été modifié pour `yield` les données de scan au lieu de les enregistrer dans un fichier. Cela permet de traiter les données en temps réel et d'afficher les nuages de points directement dans la fenêtre Pygame.
