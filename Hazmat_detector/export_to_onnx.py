#!/usr/bin/env python3
"""
Exportar modelos a ONNX para usar con el detector C++.
Ejecutar desde la carpeta Hazmat_detector/
"""

import torch
import torch.nn as nn
from torchvision import models
from ultralytics import YOLO

# --- Exportar YOLO a ONNX ---
print("Exportando YOLO a ONNX...")
yolo = YOLO("hazmat_best.pt")
yolo.export(format="onnx", imgsz=640, simplify=True)
print("hazmat_best.onnx creado")

# --- Exportar ResNet-18 a ONNX ---
print("Exportando ResNet-18 a ONNX...")
device = torch.device("cpu")
model = models.resnet18(weights=None)
model.fc = nn.Sequential(
    nn.Dropout(p=0.3),
    nn.Linear(512, 10),
)
ckpt = torch.load("best_resnet18.pth", map_location=device, weights_only=False)
state_dict = ckpt.get("model_state_dict", ckpt)
model.load_state_dict(state_dict)
model.eval()

dummy = torch.randn(1, 3, 224, 224)
torch.onnx.export(
    model, dummy, "best_resnet18.onnx",
    input_names=["input"],
    output_names=["output"],
    dynamic_axes={"input": {0: "batch"}, "output": {0: "batch"}},
    opset_version=17,
)
print("best_resnet18.onnx creado")
print("Listo! Ahora puedes compilar y ejecutar hazmat_detector.cpp")
