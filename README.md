# Wiggler

App iPhone (15 Pro, iOS 17+) qui mesure en temps réel la rotation d'un objet filmé par un téléphone fixe :
tour de potier, chaise de bureau, n'importe quoi qui tourne autour d'un axe fixe. Elle infère l'axe, puis l'angle
de l'objet (0–360°) autour de cet axe, et affiche en réalité augmentée un cylindre sur l'axe et un rayon
(demi-plan) qui tourne avec la pièce.

Aucune hypothèse sur l'objet, la distance ou le point de vue. Une seule hypothèse : entre deux ré-évaluations
l'objet est à peu près rigide et l'axe à peu près fixe (« semi-statique »). Tous les capteurs sont utilisés en
permanence : image de la caméra, profondeur LiDAR, pose ARKit.

## Construire

```bash
brew install xcodegen
cd wiggler
xcodegen generate          # produit Wiggler.xcodeproj
open Wiggler.xcodeproj     # choisir votre équipe de signature, cible = votre iPhone
```

Les tests du cœur algorithmique tournent sans iPhone, directement sur le Mac :

```bash
cd WigglerCore
swift test
```

Ils vérifient l'algèbre (valeurs propres, transformations), le suivi de points, l'estimation d'axe, le suivi
d'angle, la détection de période d'aspect, et un test de bout en bout sur une scène synthétique (disque texturé
en rotation à vitesse variable, avec carte de profondeur et bruit).

> Ce code a été écrit sans compilateur Swift sous la main (les maths ont été validées en Python, la logique KLT
> aussi). Attendez-vous éventuellement à une ou deux erreurs de compilation triviales à corriger au premier build.

## Utilisation

1. Fixer le téléphone en portrait, l'objet dans le champ.
2. Toucher l'objet à l'écran : un repère (croix + cercle en pointillés) apparaît. Le cercle est la région
   d'intérêt ; le curseur « rayon » l'agrandit. Toucher ailleurs, ou glisser, déplace le repère. Chaque
   déplacement relance l'algorithme depuis zéro — c'est fait pour « jouer avec le point » et déboguer.
3. Faire tourner l'objet. État « calibration » : l'app collecte des cordes 3D jusqu'à trouver un axe bien
   conditionné, puis attend un tour complet (compteur 0→360°). État « verrouillé » : l'axe est affiché (ligne
   cyan, cylindre translucide, anneau à la base) et le rayon orange tourne avec la pièce.
4. Pendant le premier tour après verrouillage, l'app apprend l'aspect de la pièce tous les 10° (jauge
   « aspect »). Ensuite elle recale l'angle absolu en permanence sur cette bibliothèque et détecte la période
   d'aspect : 360° (pièce quelconque), 180°, 120°… ou « symétrique » (pièce de révolution : seul l'angle relatif
   est disponible, l'angle affiché dérive lentement).

Points affichés : vert = suivi cohérent avec la rotation, rouge = incohérent (mains, fond), jaune = pas de
profondeur, blanc = nouveau.

## Comment ça marche

Tout est dans `WigglerCore` (Swift pur, sans dépendance Apple, donc testable sur Mac). L'app (`Wiggler/`) ne fait
que convertir les frames ARKit, appeler le moteur et dessiner.

### 1. Suivi de points (image)

Image luma réduite à 480×360. Coins de Shi–Tomasi détectés dans le disque autour du repère, suivis d'une frame
à l'autre par un Lucas–Kanade pyramidal (4 niveaux, fenêtre 9×9) : à 100 tr/min et 60 fps un point du bord
bouge d'une quinzaine de pixels, ce que la pyramide absorbe. Les pistes sont tuées quand le résidu photométrique
monte (occlusion par la main), quand elles sortent de la région, ou quand elles s'avèrent statiques (fond).
Le stock de pistes est réapprovisionné en continu (~160 points).

### 2. Points 3D (LiDAR + pose)

Chaque point suivi est dé-projeté avec la profondeur LiDAR (médiane 3×3, rejet des bords de profondeur) et les
intrinsèques de la caméra, puis passé en coordonnées monde avec la pose ARKit — ce qui absorbe les petits
mouvements du téléphone. Résultat : des trajectoires 3D métriques, sans aucune connaissance de l'échelle ni de
la distance.

### 3. Axe : contraintes de cordes

Un point rigide qui tourne autour d'un axe (c, n) décrit un cercle dans un plan ⊥ n. Pour deux positions a, b
d'une même piste, la corde d = b − a vérifie exactement, quelle que soit l'amplitude de l'arc :

* d · n = 0 (la corde est dans le plan du cercle) ;
* (m − c) · d = 0 avec m = (a+b)/2 (la médiatrice d'une corde passe par le centre).

Le moteur accumule ces cordes (une par piste toutes les 3 frames, avec une longueur minimale adaptée au bruit :
2 cm puis 10 % du rayon de l'objet, base temporelle ≤ 0,75 s) dans une fenêtre glissante de 15 s, et résout :

* n = vecteur propre de la plus petite valeur propre de Σ w dᵀd ;
* c = moindres carrés linéaires de Σ w ((m − c)·d̂)², contraint dans le plan ⊥ n passant par le barycentre.

Le tout en IRLS avec pondération de Huber (3 cm) : mains, fond et profondeurs aberrantes tombent en
dehors. La qualité est lue sur les valeurs propres (planarité λ₀/λ₁, couverture angulaire λ₁/λ₂) et le taux
d'inliers. Validé en Python : 0,25° sur la direction et 0,7 mm sur la position avec 6 mm de bruit et 20 % de
points parasites.

### 4. Angle : offsets par piste

Pour chaque piste i, l'azimut φᵢ(t) autour de l'axe vaut θ(t) + oᵢ. L'offset oᵢ est ancré à la naissance de la
piste ; θ(t) est la moyenne circulaire robuste (Cauchy puis porte à 15°, poids ∝ min(r, r_cap)²) des φᵢ − oᵢ,
prédite par la vitesse précédente pour encaisser 10°/frame. Les pistes incohérentes trois frames de suite sont
ré-ancrées. Comme chaque piste « se souvient » de θ depuis sa naissance, la dérive ne vient que du renouvellement
des pistes, pas de chaque frame (≈ 2° RMS en simulation à 100 tr/min, 6 mm de bruit).

Quand l'axe est raffiné (toutes les 10 frames, fusion douce si cohérent), les offsets sont recalculés pour que
θ reste continu. Si les nouvelles estimations d'axe contredisent l'axe verrouillé trois fois de suite (objet
déplacé), tout est recalibré : c'est la ré-évaluation périodique « semi-statique ».

### 5. Relocalisation d'aspect (angle absolu, période, symétrie)

Après verrouillage, une vignette 32×32 de la région d'intérêt est enregistrée tous les 10° sur un tour. La
moyenne de ces 36 vignettes (le fond statique) est soustraite, puis chaque vignette est normalisée. En marche,
la vignette courante est comparée (corrélation normalisée) aux 36 : un pic net recale θ en douceur (gain 0,15) ;
un désaccord important mais stable pendant 6 frames (après une occlusion) fait sauter θ. L'autocorrélation de la
bibliothèque donne la période d'aspect (le plus petit décalage diviseur du tour dont la similarité rivalise avec
celle des voisins), et une similarité voisine trop faible signale une pièce de révolution — dans ce cas l'angle
absolu n'est pas prédit, seulement l'angle relatif intégré.

### 6. Affichage

SceneKit (`ARSCNView`) : repère local (e₁, n, −e₂) placé sur l'axe monde ; une rotation de θ autour de l'axe y
local correspond exactement à l'azimut mesuré par le moteur, donc le rayon orange est simplement
`eulerAngles.y = θ`. Le cylindre prend le rayon (80ᵉ percentile des distances à l'axe) et la hauteur
(5ᵉ–95ᵉ percentiles) des pistes cohérentes.

## Réglages utiles (`EngineConfig`)

| Paramètre | Défaut | Rôle |
|---|---|---|
| `targetTrackCount` | 160 | nombre de points suivis |
| `maxResidual` | 0.12 | rejet photométrique (0–1) |
| `chordMinMeters` | 0.02 | longueur minimale d'une corde avant axe |
| `chordMaxFrames` | 45 | base temporelle max d'une corde |
| `constraintWindowFrames` | 900 | fenêtre glissante des cordes (15 s) |
| `lockedDriftFrames` | 3 | estimations contradictoires avant recalibration |
| `relocGain` | 0.15 | force du recalage d'aspect |
| `keyframeBins` | 36 | vignettes par tour (10°) |

## Limites connues / pistes

* ARKit peut se laisser perturber par un grand objet en mouvement (le tour occupe l'image) ; si la pose
  devient fausse, poser le téléphone face à un fond fixe visible sur les bords, ou forcer la pose identité.
* Une pièce parfaitement lisse ET de révolution n'offre ni texture ni asymétrie : rien à suivre — par
  construction l'app ne prédit rien dans ce cas.
* Le LiDAR est bruité sur les bords (rejet par médiane) et sous ~20 cm ; entre 30 cm et 2 m tout va bien.

## Journal d'essais (14 sept. 2026, chaise de bureau + boîte)

* **Perf** : le package compilé sans `-O` prenait 60–400 ms/image ; avec `-O` forcé dans `Package.swift`, 2–5 ms.
* **Boîte au bord de l'assise** : échec — le LiDAR (256×192, lissé) attribue aux points de la boîte la profondeur
  du sol derrière. Ajout d'un rejet des sauts de profondeur par piste (> 8 %). Règle pratique : viser l'intérieur
  de la silhouette de profondeur.
* **Recalibrations intempestives** : l'axe ré-estimé bougeait de 3–8 cm (chaise qui roule) → tolérance élargie et
  *adoption* douce du nouvel axe au lieu d'un retour en calibration.
* **Objet tenu en main** (pas d'axe fixe) : jamais verrouillé, planarité ≈ 0,4 — comportement voulu.
* **Enregistrement `wiggler-20260914-154441`** : verrouillé après un tour (10 s), bibliothèque d'aspect complète à
  14 s, confiance 0,8–0,99, dispersion 1–3°, passage de la main et inversion de sens encaissés.
* ARKit livre ~20 fps après quelques minutes (contrainte thermique) ; le moteur n'est pas le goulot.

Outils : `tools/wigreader.py` (lecture des `.wig`), `tools/replay.py` (rejeu complet avec OpenCV, rapport PNG),
`tools/diag_tracks.py` (ajustement de cercles par piste).
