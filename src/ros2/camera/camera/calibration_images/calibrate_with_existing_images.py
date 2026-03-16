#!/usr/bin/env python3
import cv2
import numpy as np
import glob
import yaml
import os

def calibrate_with_markers():
    """
    Калибровка камеры по отдельным ArUco маркерам
    """
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    package_dir = os.path.dirname(script_dir)
    images_path = os.path.join(package_dir, 'calibration_images')
    output_config_path = os.path.join(package_dir, 'config', 'camera_calibration_config.yaml')
    
    print("=== КАЛИБРОВКА ПО ОТДЕЛЬНЫМ МАРКЕРАМ ===")
    print(f"Директория: {images_path}")
    print("=" * 50)
    
    # Загружаем изображения
    image_files = glob.glob(os.path.join(images_path, '*.jpg')) + \
                  glob.glob(os.path.join(images_path, '*.png'))
    
    print(f"Найдено {len(image_files)} изображений")
    
    # Используем словарь DICT_5X5_50
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
    detector = cv2.aruco.ArucoDetector(aruco_dict)
    
    # Реальный размер маркера в метрах
    marker_size = 0.046  # 46 мм
    
    # Собираем данные для калибровки
    all_object_points = []  # список массивов 3D точек
    all_image_points = []   # список массивов 2D точек
    
    image_size = None
    total_markers = 0
    
    for i, image_path in enumerate(image_files):
        print(f"\nОбработка {i+1}/{len(image_files)}: {os.path.basename(image_path)}")
        
        img = cv2.imread(image_path)
        if img is None:
            print(f"  Не удалось загрузить")
            continue
        
        if image_size is None:
            image_size = (img.shape[1], img.shape[0])
            print(f"  Размер: {image_size[0]}x{image_size[1]}")
        
        # Улучшаем изображение
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        enhanced = clahe.apply(gray)
        
        # Детектируем маркеры
        corners, ids, _ = detector.detectMarkers(enhanced)
        
        if ids is not None and len(ids) > 0:
            print(f"  Найдено маркеров: {len(ids)}")
            
            # Для этого изображения создаем списки точек
            obj_points = []
            img_points = []
            
            for j, (corner, id_) in enumerate(zip(corners, ids)):
                # 3D точки углов маркера (в системе координат маркера)
                obj_corner = np.array([
                    [-marker_size/2,  marker_size/2, 0],
                    [ marker_size/2,  marker_size/2, 0],
                    [ marker_size/2, -marker_size/2, 0],
                    [-marker_size/2, -marker_size/2, 0]
                ], dtype=np.float32)
                
                # 2D точки углов маркера (на изображении)
                img_corner = corner[0].astype(np.float32)
                
                obj_points.append(obj_corner)
                img_points.append(img_corner)
            
            # Добавляем данные этого изображения в общие списки
            all_object_points.append(np.vstack(obj_points))
            all_image_points.append(np.vstack(img_points))
            total_markers += len(ids)
            
            # Визуализация
            img_copy = img.copy()
            cv2.aruco.drawDetectedMarkers(img_copy, corners, ids)
            cv2.imshow('Detected Markers', cv2.resize(img_copy, (960, 540)))
            cv2.waitKey(100)
        else:
            print(f"  Маркеры не найдены")
    
    cv2.destroyAllWindows()
    
    if len(all_object_points) < 3:
        print(f"\nОшибка: недостаточно изображений с маркерами (нужно минимум 3)")
        return False
    
    print(f"\nУспешно обработано изображений: {len(all_object_points)}")
    print(f"Всего маркеров: {total_markers}")
    
    # Начальное приближение для матрицы камеры
    focal_length = max(image_size)  # приблизительно
    camera_matrix = np.array([
        [focal_length, 0, image_size[0]/2],
        [0, focal_length, image_size[1]/2],
        [0, 0, 1]
    ], dtype=np.float32)
    
    dist_coeffs = np.zeros(5)
    
    # Калибровка
    print("\nКалибровка камеры...")
    
    retval, camera_matrix, dist_coeffs, rvecs, tvecs = \
        cv2.calibrateCamera(
            all_object_points,
            all_image_points,
            image_size,
            camera_matrix,
            dist_coeffs,
            flags=cv2.CALIB_USE_INTRINSIC_GUESS
        )
    
    if not retval:
        print("Ошибка калибровки")
        return False
    
    # Вычисляем ошибку репроекции
    total_error = 0
    total_points = 0
    
    for i in range(len(all_object_points)):
        img_points2, _ = cv2.projectPoints(
            all_object_points[i],
            rvecs[i], tvecs[i],
            camera_matrix, dist_coeffs
        )
        
        # Исправляем: img_points2 имеет форму (N, 1, 2), нужно привести к (N, 2)
        img_points2 = img_points2.reshape(-1, 2)
        
        error = cv2.norm(all_image_points[i], img_points2, cv2.NORM_L2) / len(img_points2)
        total_error += error
        total_points += len(img_points2)
    
    mean_error = total_error / len(all_object_points)
    
    # Сохраняем результаты
    calibration_data = {
        'camera_matrix': camera_matrix.tolist(),
        'dist_coeff': dist_coeffs.tolist(),
        'image_width': image_size[0],
        'image_height': image_size[1],
        'reprojection_error': float(mean_error),
        'calibration_date': str(np.datetime64('now')),
        'method': 'individual_markers',
        'marker_size': marker_size,
        'dictionary': 'DICT_5X5_50'
    }
    
    os.makedirs(os.path.dirname(output_config_path), exist_ok=True)
    
    with open(output_config_path, 'w') as f:
        yaml.dump(calibration_data, f, default_flow_style=False)
    
    print(f"\n=== РЕЗУЛЬТАТЫ КАЛИБРОВКИ ===")
    print(f"Средняя ошибка репроекции: {mean_error:.4f} пикселей")
    print(f"\nМатрица камеры:")
    print(f"  fx = {camera_matrix[0,0]:.2f}")
    print(f"  fy = {camera_matrix[1,1]:.2f}")
    print(f"  cx = {camera_matrix[0,2]:.2f}")
    print(f"  cy = {camera_matrix[1,2]:.2f}")
    print(f"\nКоэффициенты дисторсии:")
    print(f"  k1 = {dist_coeffs[0]:.6f}")
    print(f"  k2 = {dist_coeffs[1]:.6f}")
    print(f"  p1 = {dist_coeffs[2]:.6f}")
    print(f"  p2 = {dist_coeffs[3]:.6f}")
    print(f"  k3 = {dist_coeffs[4]:.6f}")
    print(f"\nРезультаты сохранены в: {output_config_path}")
    
    # Проверка на тестовом изображении
    if len(image_files) > 0:
        test_img = cv2.imread(image_files[0])
        gray = cv2.cvtColor(test_img, cv2.COLOR_BGR2GRAY)
        enhanced = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8)).apply(gray)
        
        corners, ids, _ = detector.detectMarkers(enhanced)
        
        if ids is not None:
            # Рисуем маркеры
            test_img_copy = test_img.copy()
            cv2.aruco.drawDetectedMarkers(test_img_copy, corners, ids)
            
            # Для первого маркера рисуем оси
            if len(rvecs) > 0 and len(tvecs) > 0:
                cv2.drawFrameAxes(test_img_copy, camera_matrix, dist_coeffs, 
                                 rvecs[0], tvecs[0], 0.05)
            
            cv2.imshow('Calibration Test', cv2.resize(test_img_copy, (960, 540)))
            print("\nНажмите любую клавишу...")
            cv2.waitKey(0)
            cv2.destroyAllWindows()
    
    return True

if __name__ == "__main__":
    calibrate_with_markers()
