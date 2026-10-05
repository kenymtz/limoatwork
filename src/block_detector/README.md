# block_detector

Nodo ROS 2 Humble en Python para detectar bloques de construccion con OpenCV,
HSV y profundidad RGB-D. No usa YOLO ni modelos de inteligencia artificial.

## Estructura

```text
block_detector/
  block_detector/block_detector_node.py
  config/block_detector_params.yaml
  launch/block_detector.launch.py
  package.xml
  setup.cfg
  setup.py
```

## Camara revisada en este workspace

Con la Orbbec DaBai activa se observaron estos topicos:

```bash
/camera/color/image_raw        sensor_msgs/msg/Image
/camera/depth/image_raw        sensor_msgs/msg/Image
/camera/color/camera_info      sensor_msgs/msg/CameraInfo
/camera/depth/camera_info      sensor_msgs/msg/CameraInfo
/camera/depth/points           sensor_msgs/msg/PointCloud2
```

Muestras vistas en ejecucion:

```text
color: encoding=rgb8, width=640, height=480, frame=camera_color_optical_frame
depth: encoding=16UC1, width=640, height=400, frame=camera_depth_optical_frame
color CameraInfo K=[487.28076172, 0.0, 323.65737915, 0.0, 487.28076172, 213.4556427, 0.0, 0.0, 1.0]
```

La profundidad actual no esta alineada con color porque difieren resolucion y
frame. El nodo detecta esta condicion, publica debug/detecciones, pero deja
`position_camera` en `null` y marca `depth_status` hasta que uses profundidad
registrada al frame optico de color o una configuracion equivalente.

## Compilar e instalar

Desde la raiz del workspace:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select block_detector --symlink-install
source install/setup.bash
```

## Ejecutar

```bash
ros2 launch block_detector block_detector.launch.py
```

Tambien puedes ejecutarlo directamente:

```bash
ros2 run block_detector block_detector --ros-args \
  --params-file src/block_detector/config/block_detector_params.yaml
```

## Visualizar

```bash
ros2 run rqt_image_view rqt_image_view /block_detector/debug_image
ros2 topic echo /block_detector/detections
```

Si activas `publish_masks: true`:

```bash
ros2 run rqt_image_view rqt_image_view /block_detector/masks/rojo
ros2 run rqt_image_view rqt_image_view /block_detector/masks/verde
```

## Calibrar HSV

1. Abre la imagen cruda y la imagen debug:

```bash
ros2 run rqt_image_view rqt_image_view /camera/color/image_raw
ros2 run rqt_image_view rqt_image_view /block_detector/debug_image
```

2. Activa mascaras en `config/block_detector_params.yaml`:

```yaml
publish_masks: true
```

3. Ajusta cada `hsv_ranges.<color>` con la escala de OpenCV:
   H entre 0 y 179, S/V entre 0 y 255.

4. Para rojo conserva dos rangos porque cruza el borde del espacio HSV:

```yaml
hsv_ranges.rojo: [0, 80, 50, 10, 255, 255, 170, 80, 50, 179, 255, 255]
```

5. Baja o sube `min_area_pixels`, `morphology_kernel_size`,
   `morphology_open_iterations` y `morphology_close_iterations` hasta que la
   mascara cubra cada bloque sin unirlo con ruido cercano.

## Reducir falsos positivos

El detector publica metricas de confiabilidad por deteccion:

```text
confidence, rectangularity, solidity, color_fill_ratio
```

Para hacerlo mas estricto, sube gradualmente:

```yaml
min_area_pixels: 900.0
min_rectangularity: 0.70
min_solidity: 0.90
min_color_fill_ratio: 0.70
```

Si los bloques aparecen siempre en una zona de trabajo, limita el ROI
normalizado. Por ejemplo, para mirar solo la zona central/inferior:

```yaml
roi_x_min_norm: 0.15
roi_y_min_norm: 0.25
roi_x_max_norm: 0.85
roi_y_max_norm: 0.95
```

Si algun color detecta mesa, sombras u objetos del entorno, primero revisa su
mascara en `rqt_image_view` y estrecha el rango HSV de ese color.

## Notas de profundidad

El nodo soporta `16UC1` en milimetros y `32FC1` en metros. Rechaza NaN,
infinitos, ceros y valores fuera de `min_depth_m`/`max_depth_m`. Para cada
bloque usa la mediana dentro de una mascara erosionada, no solo el pixel central.

La estimacion de `bloque_3cm`, `bloque_4cm`, `bloque_6cm` y `bloque_7_5cm` es
aproximada y depende de que color y profundidad esten alineados.
