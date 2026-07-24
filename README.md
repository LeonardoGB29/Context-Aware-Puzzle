# Context-Aware-Puzzle

Generador + solver de un rompecabezas de piezas irregulares de **bordes planos**
(sin curvas ni pestañas), en C++/OpenCV. Inspirado en los histogramas de textones
de Ruzić & Pižurica para *organizar* las piezas por textura antes de armarlas.

```
make            # compila generator y solver
make run        # menu: elige modo y grupo de imagenes (data/imagenN)
make clean      # borra binarios y salidas globales
```

`make run` pregunta dos cosas y despacha solo:

```
que quieres hacer?
  1) cortar y unir   (doy una imagen, la corta y la arma)
  2) unir directo    (ya tengo las piezas, solo las arma)
grupos disponibles:
  1) data/imagen1
  2) data/imagen2
```

Cada `data/imagenN/` es un ejemplo: adentro va su `imagen_referencia*` y su
carpeta `pieces/`. La opción **cortar** parte de la referencia; la opción
**unir directo** usa las piezas ya cortadas (tipo rompecabezas real). También se
puede llamar a mano: `./build/generator data/imagenN` y `./build/solver data/imagenN`.

---

## 1. Cómo corta las piezas (generador)

El corte es irregular pero sin piezas diminutas. Cuatro pasos:

1. **Semillas con Poisson-disk (Bridson)** — se siembran puntos al azar dentro de
   la imagen con la condición de que **ninguno quede a menos de `r` píxeles de otro**.
   Ese `r` es el que fija el tamaño mínimo de pieza (resuelve el problema de las
   piezas minúsculas).
2. **Diagrama de Voronoi** (`cv::Subdiv2D::getVoronoiFacetList`) sobre esas semillas.
   Cada semilla se convierte en una celda poligonal: aristas rectas, formas
   irregulares (triángulos, cuadriláteros, trapecios…), sin curvas.
3. **Recorte al marco** (Sutherland–Hodgman) — las celdas del borde se cortan
   contra el rectángulo de la imagen. Todos los polígonos quedan orientados CCW
   (el solver lo asume).
4. **Extracción + rotación** — cada celda se recorta de la imagen con canal alpha,
   se le aplica una **rotación aleatoria** y se guarda como PNG.

Salidas del generador (dentro de la carpeta del ejemplo `data/imagenN/`):

- `cut_preview.png` — la misma imagen con las líneas de corte dibujadas encima.
- `pieces/1_piezas_cortadas/` — las N piezas tal como se cortaron.
- `pieces/2_piezas_rotadas/` — las mismas piezas ya rotadas al azar
  (es la entrada del solver) + `pieces.yml` y `ground_truth.yml`.

---

## 2. Cómo une las piezas (solver)

El criterio principal para decidir si dos piezas encajan es la
**continuidad de color a lo largo de la costura** (edge matching): para cada
arista se muestrea una *tira* de colores justo por dentro del borde; dos aristas
casan cuando sus tiras (una en sentido invertido) coinciden, es decir cuando el
color **no da un salto** al cruzar la unión. Es una compatibilidad de bordes,
no el "coloreo" (el coloreo es solo la visualización de textones del punto 3).

Sobre ese criterio se montan tres refuerzos tomados de la idea abstracta del MRF
del paper, que son los que evitan cadenas de errores en zonas lisas:

- **Marco primero** — se ancla una esquina y se arma todo el borde antes que el
  interior. Cada arista de marco debe quedar alineada a los ejes y colineal con
  las demás (`frameOK`); recién luego se avanza de las esquinas hacia adentro.
- **Consenso multi-costura** — una pieza no se coloca por una sola arista buena:
  debe cuadrar con **todas** las costuras que genera con sus vecinas ya puestas.
  Si alguna costura es mala, se rechaza toda la colocación.
- **Aceptación mutua** — se prefiere el par de aristas en el que cada una es la
  mejor opción de la otra (best-buddy), antes de aceptar cualquier otro.
- **Relleno y reparación por energía global** (idea del MRF del paper) — al final,
  las piezas sueltas se prueban por fuerza bruta donde cierren 2+ costuras
  geométricas (el color solo desempata), y si el lugar está ocupado se intenta el
  desalojo: se expulsa al ocupante, se rellena, y el cambio se queda **solo si baja
  la energía global** del armado (costuras buenas restan, malas suman, sueltas
  castigan). Es la versión con derecho a arrepentirse del greedy.

La **textura (textones de Gabor)** entra como un descuento suave para *desempatar*
y organizar candidatos; nunca decide sola (medido: usada como penalización dura
empeora el armado).

---

## 3. Cómo queda la imagen

Se producen dos vistas del mismo armado:

- **Imagen unida por textones (`data/imagenN/pieces/3_analisis_coloreo/`)** — cada
  pieza pintada según sus texturas dominantes: zonas con textura parecida comparten
  color, así que se ven "manchas" o formas de textura, no la foto. Esto es lo que
  *guía y organiza* el emparejamiento; **no es el resultado final**.
- **Imagen unida normal (`data/imagenN/resultado.png`)** — la foto real reconstruida,
  con las piezas rotadas y colocadas en su sitio. Sale en el **mismo tamaño y
  orientación** que la imagen de entrada (alineación Kabsch robusta contra el
  ground truth). Si hay metadatos también reporta la precisión (piezas bien
  ubicadas / total).

---

## Limitación conocida (matching por color)

Como la decisión fina es la continuidad de color en la costura, dos piezas
**pueden no unirse cuando su color es ambiguo**: regiones planas o de color muy
uniforme dan tiras casi idénticas en muchos bordes, y ahí el color no distingue
la unión correcta. La textura ayuda a desambiguar, pero el criterio dominante
sigue siendo cromático. Si se busca que sea **más robusto que "compatible por
color"**, el camino es darle más peso a la textura y a la geometría global (que
hoy solo organizan/desempatan) frente al color puro.
