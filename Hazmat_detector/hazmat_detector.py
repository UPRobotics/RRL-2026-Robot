#!/usr/bin/env python3
"""
Script para detección de materiales peligrosos usando el modelo personalizado YOLO
Usa el modelo hazmat_best.pt entrenado para detectar materiales peligrosos
"""

import cv2
import os
from ultralytics import YOLO
import numpy as np

class HazmatDetector:
    def __init__(self, model_path='hazmat_best.pt'):
        """Inicializar detector de materiales peligrosos"""
        if not os.path.exists(model_path):
            print(f"❌ Error: Modelo {model_path} no encontrado")
            print("🔍 Modelos disponibles:")
            for file in os.listdir('.'):
                if file.endswith('.pt'):
                    print(f"   📁 {file}")
            return None
        
        print(f"🔄 Cargando modelo personalizado: {model_path}")
        self.model = YOLO(model_path)
        print(f"✅ Modelo {model_path} cargado exitosamente")
        
        # Mostrar las clases que puede detectar
        print(f"🎯 Clases detectables: {list(self.model.names.values())}")
    
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
        
        if detections is not None and len(detections) > 0:
            print(f"⚠️ ¡MATERIALES PELIGROSOS DETECTADOS! ({len(detections)} objetos)")
            hazmat_found = True
            
            # Mostrar detalles de cada detección
            for i, box in enumerate(detections):
                class_id = int(box.cls[0])
                class_name = self.model.names[class_id]
                confidence = float(box.conf[0])
                coords = box.xyxy[0].tolist()  # [x1, y1, x2, y2]
                
                print(f"   🚨 {i+1}. {class_name}")
                print(f"      📊 Confianza: {confidence:.2f} ({confidence*100:.1f}%)")
                print(f"      📍 Posición: ({coords[0]:.0f}, {coords[1]:.0f}) - ({coords[2]:.0f}, {coords[3]:.0f})")
        else:
            print("✅ No se detectaron materiales peligrosos")
        
        if save_result:
            # Guardar imagen con anotaciones
            annotated_frame = results[0].plot()
            
            # Agregar estado al nombre del archivo
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
            
            # Registrar detecciones
            detections = results[0].boxes
            if detections is not None and len(detections) > 0:
                hazmat_detections.append({
                    'frame': frame_count,
                    'time': frame_count / fps,
                    'detections': len(detections)
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
    
    def detect_hazmat_webcam(self, confidence=0.5):
        """Detectar materiales peligrosos desde la webcam en tiempo real"""
        print("📹 Iniciando detección de materiales peligrosos desde webcam...")
        print("🚨 MODO ALERTA ACTIVADO")
        print("Presiona 'q' para salir")
        
        cap = cv2.VideoCapture(0)
        
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            # Realizar predicción
            results = self.model(frame, conf=confidence, verbose=False)
            annotated_frame = results[0].plot()
            
            # Verificar si hay detecciones
            detections = results[0].boxes
            if detections is not None and len(detections) > 0:
                # Agregar alerta visual
                cv2.putText(annotated_frame, "⚠️ MATERIALES PELIGROSOS DETECTADOS ⚠️", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
                
                # Parpadear la imagen (efecto de alerta)
                if int(cv2.getTickCount() / cv2.getTickFrequency()) % 2:
                    overlay = annotated_frame.copy()
                    overlay[:] = (0, 0, 255)  # Rojo
                    annotated_frame = cv2.addWeighted(annotated_frame, 0.8, overlay, 0.2, 0)
            
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
    
    # Crear detector
    detector = HazmatDetector('hazmat_best.pt')
    
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
            detector.detect_hazmat_webcam(confidence)
        
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