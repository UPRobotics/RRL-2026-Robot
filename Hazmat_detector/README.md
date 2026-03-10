# Detector de Materiales Peligrosos (HAZMAT)

Detección en tiempo real de materiales peligrosos usando webcam.  
Pipeline combinado: **YOLO** (detección de regiones) + **ResNet-18** (clasificación del tipo).

## Clases detectables

| # | Clase         |
|---|---------------|
| 1 | Combustible   |
| 2 | Corrosive     |
| 3 | Explosives    |
| 4 | Flammable     |
| 5 | Flammable_gas |
| 6 | Fuel          |
| 7 | Oxidizer      |
| 8 | Peroxide      |
| 9 | Poison        |
|10 | Radioactive   |

---

## Requisitos

- **Python 3.9+**
- Una **webcam** conectada

---

## Instalación rápida

### 1. Crear entorno virtual (recomendado)

```bash
python -m venv venv

# Windows
venv\Scripts\activate

# Linux / Mac
source venv/bin/activate
```

### 2. Instalar dependencias

```bash
pip install -r requirements.txt
```

> **Nota sobre PyTorch con GPU (opcional):**  
> Si tienes GPU NVIDIA y quieres aceleración CUDA, instala PyTorch con soporte CUDA **antes** de las demás dependencias:
> ```bash
> pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121
> pip install -r requirements.txt
> ```

---

## Uso

```bash
python hazmat_camera.py
```

- La ventana mostrará la cámara con las detecciones en tiempo real.
- Si detecta un material peligroso, dibuja el recuadro con la clase y confianza.
- Presiona **`q`** para salir.

### Cambiar cámara

Si tu webcam no es la cámara 1 (por ejemplo si tienes cámara integrada en laptop), abre `hazmat_camera.py` y cambia:

```python
cap = cv2.VideoCapture(1)   # <-- Cambia a 0 si solo tienes una cámara
```

---

## Archivos incluidos

| Archivo              | Descripción                                      |
|----------------------|--------------------------------------------------|
| `hazmat_camera.py`   | Script principal — ejecuta el detector en webcam  |
| `hazmat_best.pt`     | Modelo YOLO entrenado para detectar hazmat        |
| `best_resnet18.pth`  | Modelo ResNet-18 para clasificar tipo de material |
| `requirements.txt`   | Dependencias de Python                            |

---

## Solución de problemas

| Problema | Solución |
|----------|----------|
| `No se pudo abrir la camara` | Cambia `VideoCapture(1)` a `VideoCapture(0)` en el script |
| `ModuleNotFoundError` | Ejecuta `pip install -r requirements.txt` |
| Detección muy lenta | Instala PyTorch con CUDA (ver nota arriba) |
| `hazmat_best.pt no encontrado` | Asegúrate de que los archivos `.pt` y `.pth` estén en la misma carpeta que el script |
