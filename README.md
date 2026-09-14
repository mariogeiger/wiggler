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

Un `Wiggler.xcodeproj` écrit à la main est aussi fourni (pas besoin de XcodeGen). Le bouton « Enregistrer » de
l'app écrit des fichiers `.wig` (image 480×360, profondeur, pose, sorties du moteur) partageables par AirDrop et
rejouables hors ligne avec `tools/replay.py`, et comparables entre stratégies de fusion avec
`tools/fusion.py` (qui sait aussi injecter une occlusion ou une bibliothèque périmée pour tester les deux
comportements qui comptent).

## Utilisation

1. Fixer le téléphone en portrait, l'objet dans le champ.
2. Toucher l'objet à l'écran : une croix apparaît ; les points sont cherchés dans un large disque autour d'elle.
   Toucher ailleurs, ou glisser, déplace le repère. Chaque déplacement relance l'algorithme depuis zéro — c'est
   fait pour « jouer avec le point » et déboguer.
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

### 4. Angle : offsets par piste (la « mesure relative »)

Pour chaque piste i, l'azimut φᵢ(t) autour de l'axe vaut θ(t) + oᵢ. L'offset oᵢ est ancré à la naissance de la
piste ; θ(t) est la moyenne circulaire robuste (Cauchy puis porte à 15°, poids ∝ min(r, r_cap)²) des φᵢ − oᵢ,
prédite par la vitesse précédente pour encaisser 10°/frame. Les pistes incohérentes trois frames de suite sont
ré-ancrées. Comme chaque piste « se souvient » de θ depuis sa naissance, la dérive ne vient que du renouvellement
des pistes, pas de chaque frame (≈ 2° RMS en simulation à 100 tr/min, 6 mm de bruit).

Le raffinement de l'axe est fait **après** la mesure d'angle, jamais avant : ré-ancrer les offsets d'abord rend
tous les candidats d'accord avec θ courant et détruit silencieusement un incrément toutes les
`axisUpdateInterval` frames (mesuré : 7 % des images gelées, soit ~7 % de rotation perdue en permanence).

### 4 bis. Recoupement indépendant

Les mêmes correspondances KLT donnent aussi une rotation 2D dans le plan image (Procrustes + IRLS,
`similarityRotation`). Elle n'utilise ni la profondeur ni l'axe, donc elle échoue dans d'autres situations que
l'azimut 3D. Le rapport entre les deux est appris en ligne (il ne dépend que de l'inclinaison de l'axe vis-à-vis
de la caméra, constante à téléphone fixe) ; quand les deux cessent de concorder trois frames de suite, c'est que
quelque chose d'autre s'est emparé des points suivis, et la géométrie n'est plus déclarée « saine » — quel que
soit le nombre d'inliers qu'elle annonce.

### 5. Aspect et fusion (la « mesure absolue »)

Une vignette 32×32 de la région d'intérêt est rangée dans un casier tous les 10°. La bibliothèque est utilisable
dès 6 casiers — attendre un tour complet la rendrait indisponible la plupart du temps (mesuré : 8 % → 94 % du
temps verrouillé). La moyenne des casiers remplis (le fond statique) est soustraite avant normalisation.

La fusion (`AngleFusion`) est un filtre complémentaire classique, l'incrément géométrique jouant le gyroscope et
l'aspect la boussole :

1. **Aucun téléport.** Une correction devient un débit borné (90°/s), donc l'angle affiché est toujours continu —
   et les tr/min, qui viennent du seul incrément géométrique, ne sont jamais pollués par les recalages.
2. **Une incertitude honnête.** σ croît en marche aléatoire (renouvellement des pistes) tant que la géométrie
   mesure, et **linéairement à la vitesse de rotation** quand elle ne mesure plus : un objet que personne ne
   regarde continue de tourner dans le même sens. C'est pour cela qu'une occlusion de 0,3 s à 100 tr/min ouvre le
   portail à 180° alors que 30 s de suivi sain le garde à quelques degrés.
3. **Portail et validation.** Un appariement n'est utilisé que dans un portail d'innovation à 3σ, et seulement si
   aucun appariement nettement meilleur ne se trouve en dehors. Le lobe à 120° d'une boîte hexagonale devient donc
   impossible à suivre après un fonctionnement sain, et une vraie ré-acquisition à 150° reste possible juste après
   une perte.

Quand un appariement **fort** contredit une géométrie **saine** pendant plus de 2 s, c'est la bibliothèque qui a
tort (objet déplacé, éclairage changé) : elle est reconstruite au lieu d'être combattue.

Mesuré sur `wiggler-20260914-160109` (90 s, manipulations à la main) : 11 discontinuités jusqu'à 177° avant,
**0 après**, et le résidu contre la rotation d'image indépendante passe de 39 à 19 °/s.

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
| `lockedDriftFrames` | 3 | estimations d'axe contradictoires avant adoption |
| `minHealthyInliers` | 20 | points en dessous desquels la géométrie n'est plus « saine » |
| `staleLibrarySeconds` | 2.0 | contradiction avant reconstruction de la bibliothèque d'aspect |
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
