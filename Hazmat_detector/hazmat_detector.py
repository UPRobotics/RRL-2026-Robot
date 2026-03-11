#!/usr/bin/env python3
"""
Script para detección de materiales peligrosos con pipeline de dos etapas:
  1. YOLO (hazmat_best.pt) - Detección y localización de señales hazmat
  2. ResNet-18 (best_resnet18.pth) - Clasificación del tipo de material peligroso
"""

import cv2
import os
import torch
import torch.nn as nn
from torchvision import models, transforms
from PIL import Image
from ultralytics import YOLO
import numpy as np

# Constantes para normalización ImageNet
IMAGENET_MEAN = [0.485, 0.456, 0.406]
IMAGENET_STD = [0.229, 0.224, 0.225]

RESNET_CLASS_NAMES = [
    "Combustible", "Corrosive", "Explosives", "Flammable",
    "Flammable_gas", "Fuel", "Oxidizer", "Peroxide",
    "Poison", "Radioactive",
]

class HazmatDetector:
    def __init__(self, yolo_path='hazmat_best.pt', resnet_path='best_resnet18.pth'):
        """Inicializar detector de materiales peligrosos (YOLO + ResNet-18)"""
        # --- Cargar modelo YOLO (detección) ---
        if not os.path.exists(yolo_path):
            print(f"❌ Error: Modelo YOLO {yolo_path} no encontrado")
            print("🔍 Modelos disponibles:")
            for file in os.listdir('.'):
                if file.endswith('.pt') or file.endswith('.pth'):
                    print(f"   📁 {file}")
            return None
        
        print(f"🔄 Cargando modelo YOLO: {yolo_path}")
        self.model = YOLO(yolo_path)
        print(f"✅ Modelo YOLO cargado exitosamente")
        print(f"🎯 Clases detectables (YOLO): {list(self.model.names.values())}")
        
        # --- Cargar modelo ResNet-18 (clasificación) ---
        self.classifier = None
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        if os.path.exists(resnet_path):
            print(f"🔄 Cargando clasificador ResNet-18: {resnet_path}")
            self.classifier = models.resnet18(weights=None)
            self.classifier.fc = nn.Sequential(
                nn.Dropout(p=0.3),
                nn.Linear(512, len(RESNET_CLASS_NAMES)),
            )
            ckpt = torch.load(resnet_path, map_location=self.device, weights_only=False)
            state_dict = ckpt.get("model_state_dict", ckpt)
            self.classifier.load_state_dict(state_dict)
            self.classifier.to(self.device)
            self.classifier.eval()
            print(f"✅ Clasificador ResNet-18 cargado exitosamente")
            print(f"🎯 Clases clasificables: {RESNET_CLASS_NAMES}")
        else:
            print(f"⚠️ Clasificador {resnet_path} no encontrado — solo se usará YOLO")
        
        # Transforms para ResNet-18
        self.resnet_transforms = transforms.Compose([
            transforms.Resize(256),
            transforms.CenterCrop(224),
            transforms.ToTensor(),
            transforms.Normalize(mean=IMAGENET_MEAN, std=IMAGENET_STD),
        ])
    
    def _classify_crop(self, crop_bgr):
        """Clasificar un recorte BGR con ResNet-18. Retorna (clase, confianza)."""
        if self.classifier is None:
            return None, 0.0
        crop_rgb = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)
        pil_img = Image.fromarray(crop_rgb)
        tensor = self.resnet_transforms(pil_img).unsqueeze(0).to(self.device)
        with torch.no_grad():
            outputs = self.classifier(tensor)
            probs = torch.softmax(outputs, dim=1)
            pred_idx = outputs.argmax(1).item()
            conf = probs[0, pred_idx].item()
        return RESNET_CLASS_NAMES[pred_idx], conf
    
    def detect_hazmat_image(self, image_path, confidence=0.5, save_result=True):
        """Detectar materiales peligrosos en una imagen"""
        print(f"🔍 Analizando imagen para materiales peligrosos: {image_path}")
        
        if not os.path.exists(image_path):
            print(f"❌ Imagen no encontrada: {image_path}")
            return None
        
        # Realizar predicción con umbral de confianza
        results = self.model(image_path, conf=confidence)
        
        # Procesar resultados
        detections = results[0].boxes
        hazmat_found = False
        
        # Leer imagen para recortes de clasificación
        img_bgr = cv2.imread(image_path)
        
        if detections is not None and len(detections) > 0:
            print(f"⚠️ ¡MATERIALES PELIGROSOS DETECTADOS! ({len(detections)} objetos)")
            hazmat_found = True
            
            for i, box in enumerate(detections):
                class_id = int(box.cls[0])
                yolo_name = self.model.names[class_id]
                det_conf = float(box.conf[0])
                coords = box.xyxy[0].tolist()
                
                # Clasificar recorte con ResNet-18
                x1, y1, x2, y2 = map(int, coords)
                crop = img_bgr[y1:y2, x1:x2]
                resnet_name, resnet_conf = self._classify_crop(crop)
                
                print(f"   🚨 {i+1}. YOLO: {yolo_name} ({det_conf*100:.1f}%)")
                if resnet_name:
                    print(f"      🔬 ResNet-18: {resnet_name} ({resnet_conf*100:.1f}%)")
                print(f"      📍 Posición: ({coords[0]:.0f}, {coords[1]:.0f}) - ({coords[2]:.0f}, {coords[3]:.0f})")
        else:
            print("✅ No se detectaron materiales peligrosos")
        
        if save_result:
            annotated_frame = results[0].plot()
            
            # Dibujar clasificaciones ResNet-18 sobre la imagen
            if hazmat_found and self.classifier is not None:
                for box in detections:
                    coords = box.xyxy[0].tolist()
                    x1, y1, x2, y2 = map(int, coords)
                    crop = img_bgr[y1:y2, x1:x2]
                    resnet_name, resnet_conf = self._classify_crop(crop)
                    label = f"{resnet_name} {resnet_conf*100:.0f}%"
                    cv2.putText(annotated_frame, label,
                                (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, (0, 255, 255), 2)
            
            status = "HAZMAT_DETECTED" if hazmat_found else "SAFE"
            output_path = image_path.replace('.', f'_{status}.')
            cv2.imwrite(output_path, annotated_frame)
            print(f"💾 Resultado guardado: {output_path}")
        
        return results, hazmat_found
    
    def detect_hazmat_video(self, video_path, confidence=0.5, save_result=True):
        """Detectar materiales peligrosos en un video"""
        print(f"🎬 Procesando video para detección de materiales peligrosos: {video_path}")
        
        cap = cv2.VideoCapture(video_path)
        
        if save_result:
            # Configurar grabación del video resultado
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            fps = int(cap.get(cv2.CAP_PROP_FPS))
            width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            output_path = video_path.replace('.', '_hazmat_analysis.')
            out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))
        
        frame_count = 0
        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        hazmat_detections = []
        
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            frame_count += 1
            print(f"\r🔄 Procesando frame {frame_count}/{total_frames}", end="")
            
            # Realizar predicción
            results = self.model(frame, conf=confidence, verbose=False)
            annotated_frame = results[0].plot()
            
            # Registrar detecciones y clasificar con ResNet-18
            detections = results[0].boxes
            if detections is not None and len(detections) > 0:
                det_info = []
                for box in detections:
                    coords = box.xyxy[0].tolist()
                    x1, y1, x2, y2 = map(int, coords)
                    crop = frame[y1:y2, x1:x2]
                    resnet_name, resnet_conf = self._classify_crop(crop)
                    det_info.append(resnet_name)
                    if resnet_name and self.classifier is not None:
                        label = f"{resnet_name} {resnet_conf*100:.0f}%"
                        cv2.putText(annotated_frame, label,
                                    (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.6, (0, 255, 255), 2)
                hazmat_detections.append({
                    'frame': frame_count,
                    'time': frame_count / fps,
                    'detections': len(detections),
                    'classes': det_info,
                })
            
            if save_result:
                out.write(annotated_frame)
        
        cap.release()
        if save_result:
            out.release()
            print(f"\n💾 Video analizado guardado: {output_path}")
        
        # Mostrar resumen
        print(f"\n📊 RESUMEN DEL ANÁLISIS:")
        print(f"   🎬 Frames totales: {total_frames}")
        print(f"   ⚠️ Frames con materiales peligrosos: {len(hazmat_detections)}")
        if hazmat_detections:
            print(f"   🚨 ALERTA: Materiales peligrosos detectados en {len(hazmat_detections)} frames")
            print(f"   ⏱️ Primeras detecciones en: {hazmat_detections[0]['time']:.1f}s")
        else:
            print(f"   ✅ Video limpio - No hay materiales peligrosos")
    
    @staticmethod
    def list_cameras(max_index=5):
        """Listar cámaras disponibles."""
        available = []
        for i in range(max_index):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                available.append((i, w, h))
                cap.release()
        return available
    
    def detect_hazmat_webcam(self, confidence=0.5, camera_index=None):
        """Detectar materiales peligrosos desde la webcam en tiempo real"""
        if camera_index is None:
            cams = self.list_cameras()
            if not cams:
                print("❌ No se encontraron cámaras")
                return
            print("📹 Cámaras disponibles:")
            for idx, w, h in cams:
                print(f"   {idx}: {w}x{h}")
            try:
                camera_index = int(input("Selecciona el índice de cámara: ").strip())
            except ValueError:
                camera_index = 0
        
        print(f"📹 Usando cámara {camera_index}")
        print("🚨 MODO ALERTA ACTIVADO")
        print("Presiona 'q' para salir")
        
        cap = cv2.VideoCapture(camera_index)
        
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            # Realizar predicción
            results = self.model(frame, conf=confidence, verbose=False)
            annotated_frame = results[0].plot()
            
            # Verificar si hay detecciones y clasificar con ResNet-18
            detections = results[0].boxes
            if detections is not None and len(detections) > 0:
                for box in detections:
                    coords = box.xyxy[0].tolist()
                    x1, y1, x2, y2 = map(int, coords)
                    crop = frame[y1:y2, x1:x2]
                    resnet_name, resnet_conf = self._classify_crop(crop)
                    if resnet_name:
                        label = f"{resnet_name} {resnet_conf*100:.0f}%"
                        cv2.putText(annotated_frame, label,
                                    (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.6, (0, 255, 255), 2)
                
            
            # Mostrar frame
            cv2.imshow('HAZMAT DETECTOR - Detección en Tiempo Real', annotated_frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        cap.release()
        cv2.destroyAllWindows()

def main():
    """Función principal para usar el detector de materiales peligrosos"""
    print("🚨 DETECTOR DE MATERIALES PELIGROSOS")
    print("=" * 45)
    
    # Crear detector con pipeline de dos etapas
    detector = HazmatDetector('hazmat_best.pt', 'best_resnet18.pth')
    
    if detector is None:
        return
    
    # Menú interactivo
    while True:
        print("\n" + "="*45)
        print("🚨 ¿Qué tipo de análisis quieres hacer?")
        print("1. 📷 Analizar imagen")
        print("2. 🎬 Analizar video")
        print("3. 📹 Detección en tiempo real (webcam)")
        print("4. ⚙️ Cambiar umbral de confianza")
        print("5. ❌ Salir")
        
        choice = input("Selecciona una opción (1-5): ").strip()
        
        if choice == '1':
            image_path = input("📂 Ruta de la imagen: ").strip()
            if os.path.exists(image_path):
                confidence = float(input("📊 Umbral de confianza (0.1-0.9, default 0.5): ") or 0.5)
                detector.detect_hazmat_image(image_path, confidence)
            else:
                print("❌ Archivo no encontrado")
        
        elif choice == '2':
            video_path = input("📂 Ruta del video: ").strip()
            if os.path.exists(video_path):
                confidence = float(input("📊 Umbral de confianza (0.1-0.9, default 0.5): ") or 0.5)
                detector.detect_hazmat_video(video_path, confidence)
            else:
                print("❌ Archivo no encontrado")
        
        elif choice == '3':
            confidence = float(input("📊 Umbral de confianza (0.1-0.9, default 0.5): ") or 0.5)
            detector.detect_hazmat_webcam(confidence)  # te pedirá elegir cámara
        
        elif choice == '4':
            print("⚙️ El umbral de confianza se puede cambiar en cada análisis")
            print("💡 Valores recomendados:")
            print("   - 0.3: Más sensible (más detecciones, posibles falsos positivos)")
            print("   - 0.5: Balanceado (recomendado)")
            print("   - 0.7: Más conservador (solo detecciones muy seguras)")
        
        elif choice == '5':
            print("👋 ¡Mantente seguro!")
            break
        
        else:
            print("❌ Opción inválida")

if __name__ == "__main__":
    main()