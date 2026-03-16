#!/usr/bin/env python3
import cv2
import numpy as np
import glob
import yaml
import os
from cv2 import aruro

def calibrate_with_existing_images():
    """
    Калибровка камеры с использованием существующих фотографий ChArUco доски
    """
    
    # Пути
    script_dir = os.path.dirname(os.path.abspath(__file__))
    package_dir = os.path.dirname(script_dir)
    images_path = os.path.join(package_dir, 'calibration_images')
    output_config_path = os.path.join(package_dir, 'config', 'camera_calibration_config.yaml')
    
    # Параметры ChArUco доски из image.png
    columns = 5  # Количество столбцов
    rows = 7     # Количество строк
    square_length = 0.25  # 250 мм в метрах
    marker_length = 0.175  # 175 мм (70% от клетки)
    
    print("=== Калибровка камеры по существующим фотографиям ===")
    print(f"Директория с изображениями: {images_path}")
    print(f"Параметры доски: {columns}x{rows} клеток, {square_length*1000} мм")
    print("=" * 50)
    
    # Проверяем наличие изображений
    if not os.path.exists(images_path):
        print(f"Ошибка: директория {images_path} не найдена")
        return False
    
    image_files = glob.glob(os.path.join(images_path, '*.jpg')) + \
                  glob.glob(os.path.join(images_path, '*.png'))
    
    if len(image_files) == 0:
        print(f"Ошибка: в директории {images_path} нет изображений")
        return False
    
    print(f"Найдено {len(image_files)} изображений")
    
    # Словарь маркеров
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
    
    # Создаем доску Charuco
    charuco_board = aruco.CharucoBoard(
        (columns, rows),
        square_length,
        marker_length,
        aruco_dict
    )
    
    # Параметры детектора
    charuco_detector = aruco.CharucoDetector(charuco_board)
    
    # Собираем точки для калибровки
    all_corners = []
    all_ids = []
    image_size = None
    
    successful_images = 0
    
    for i, image_path in enumerate(image_files):
        print(f"\nОбработка {i+1}/{len(image_files)}: {os.path.basename(image_path)}")
        
        # Читаем изображение
        img = cv2.imread(image_path)
        if img is None:
            print(f"  Не удалось загрузить")
            continue
            
        if image_size is None:
            image_size = (img.shape[1], img.shape[0])
            print(f"  Размер изображения: {image_size[0]}x{image_size[1]}")
        
        # Конвертируем в grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Детектируем доску
        charuco_corners, charuco_ids, marker_corners, marker_ids = \
            charuco_detector.detectBoard(gray)
        
        if charuco_corners is not None and len(charuco_corners) > 4:
            all_corners.append(charuco_corners)
            all_ids.append(charuco_ids)
            successful_images += 1
            print(f"  ✓ Найдено {len(charuco_corners)} углов ChArUco")
        else:
            print(f"  ✗ Не удалось обнаружить достаточно углов")
    
    print(f"\nУспешно обработано {successful_images} из {len(image_files)} изображений")
    
    if successful_images < 5:
        print(f"Ошибка: недостаточно изображений с доской (нужно минимум 5)")
        return False
    
    # Калибровка камеры
    print("\nВыполняется калибровка...")
    
    camera_matrix = np.eye(3)
    dist_coeffs = np.zeros(5)
    
    retval, camera_matrix, dist_coeffs, rvecs, tvecs = \
        aruco.calibrateCameraCharuco(
            charuco_corners=all_corners,
            charuco_ids=all_ids,
            board=charuco_board,
            imageSize=image_size,
            cameraMatrix=camera_matrix,
            distCoeffs=dist_coeffs,
            flags=cv2.CALIB_RATIONAL_MODEL
        )
    
    if not retval:
        print("Ошибка калибровки")
        return False
    
    # Вычисляем ошибку репроекции
    total_error = 0
    for i, corners in enumerate(all_corners):
        img_points, _ = cv2.projectPoints(
            charuco_board.getChessboardCorners()[all_ids[i].flatten()],
            rvecs[i], tvecs[i],
            camera_matrix, dist_coeffs
        )
        error = cv2.norm(corners, img_points, cv2.NORM_L2) / len(img_points)
        total_error += error
    
    mean_error = total_error / len(all_corners)
    
    # Сохраняем результаты
    calibration_data = {
        'camera_matrix': camera_matrix.tolist(),
        'dist_coeff': dist_coeffs.tolist(),
        'image_width': image_size[0],
        'image_height': image_size[1],
        'reprojection_error': float(mean_error),
        'calibration_date': str(np.datetime64('now')),
        'charuco_params': {
            'columns': columns,
            'rows': rows,
            'square_length': square_length,
            'marker_length': marker_length
        }
    }
    
    # Создаем директорию config если её нет
    os.makedirs(os.path.dirname(output_config_path), exist_ok=True)
    
    # Сохраняем в YAML файл
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
    print(f"  k1 = {dist_coeffs[0]:.4f}")
    print(f"  k2 = {dist_coeffs[1]:.4f}")
    print(f"  p1 = {dist_coeffs[2]:.4f}")
    print(f"  p2 = {dist_coeffs[3]:.4f}")
    print(f"  k3 = {dist_coeffs[4]:.4f}")
    print(f"\nРезультаты сохранены в: {output_config_path}")
    
    # Покажем одно из обработанных изображений
    if successful_images > 0:
        # Берем первое успешное изображение
        test_img = cv2.imread(image_files[0])
        gray = cv2.cvtColor(test_img, cv2.COLOR_BGR2GRAY)
        
        # Детектируем и рисуем оси
        charuco_corners, charuco_ids, marker_corners, marker_ids = \
            charuco_detector.detectBoard(gray)
        
        if charuco_corners is not None:
            # Найдем соответствующую трансформацию
            for i, corners in enumerate(all_corners):
                if len(corners) == len(charuco_corners):
                    cv2.drawFrameAxes(test_img, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.2)
                    break
            
            # Уменьшаем для отображения
            display = cv2.resize(test_img, (960, 540))
            cv2.imshow('Calibration Result', display)
            print("\nНажмите любую клавишу для закрытия...")
            cv2.waitKey(0)
            cv2.destroyAllWindows()
    
    return True

if __name__ == "__main__":
    calibrate_with_existing_images()
