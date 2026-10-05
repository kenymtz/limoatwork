# Guia de funcionamento: mapeamento, navegacao, missao e docking

Este guia registra o fluxo usado no workspace `limoatwork` para mapear, iniciar a navegacao autonoma e executar a missao com docking. O rastro principal veio do arquivo `comandos` na raiz do repo e foi cruzado com os launch files e nos atuais.

## Resumo dos launchers usados

| Etapa | Comando usado | Papel |
| --- | --- | --- |
| Base do LIMO | `ros2 launch limo_base limo_base.launch.py` | Sobe o driver da base e publica odometria/controle da plataforma. |
| LiDAR YDLidar | `ros2 launch ydlidar_ros2_driver ydlidar_launch.py` | Sobe o driver do YDLidar em `/scan` e a TF `base_link -> laser_frame`. |
| Mapeamento | `ros2 launch limo_bringup cartographer.launch.py` | Sobe Cartographer, publica o mapa/occupancy grid e abre o RViz em `/map`. |
| Navegacao | `ros2 launch limo_bringup navigation2.launch.py` | Sobe Nav2 com o mapa atual e RViz. |
| Missao | `ros2 run limo_mission limo_main` | Executa a fila de missoes e chama o docking apos cada meta configurada. |

## Fluxo usado para mapear

Abra um terminal por processo e sempre rode o source do workspace antes dos comandos:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

1. Subir a base:

```bash
ros2 launch limo_base limo_base.launch.py
```

2. Subir o LiDAR:

```bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```

O launch do YDLidar usa `src/limo_ros2/ydlidar_ros2/params/ydlidar.yaml`, com `port: /dev/ttyUSB0`, `frame_id: laser_frame`, `angle_min: -110.0`, `angle_max: 110.0` e topico `/scan`.

3. Subir o mapeamento:

```bash
ros2 launch limo_bringup cartographer.launch.py
```

Esse launch sobe `cartographer_ros/cartographer_node` com `src/limo_ros2/limo_bringup/config_files/limo_lds_2d.lua` e inclui `occupancy_grid.launch.py`. A configuracao usa `map`, `odom`, `base_link`, odometria ativa e um LaserScan em `/scan`.

Por padrao, o mesmo launch tambem abre o RViz com `src/limo_ros2/limo_bringup/config_files/demo_2d.rviz`, que ja vem com `Fixed Frame: map`, display `Map` no topico `/map`, `LaserScan` em `/scan` e `Submaps` do Cartographer. Para rodar sem RViz:

```bash
ros2 launch limo_bringup cartographer.launch.py open_rviz:=false
```

4. Dirigir o robo pelo ambiente para fechar o mapa:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

5. Salvar o mapa quando a geometria estiver boa:

```bash
ros2 run nav2_map_server map_saver_cli -f src/limo_ros2/limo_bringup/maps/map
```

Isso gera/atualiza `src/limo_ros2/limo_bringup/maps/map.yaml` e `src/limo_ros2/limo_bringup/maps/map.pgm`. Para preservar cenarios, salve com outro nome primeiro, por exemplo:

```bash
ros2 run nav2_map_server map_saver_cli -f src/limo_ros2/limo_bringup/maps/cenario_lab_01
```

Depois passe esse mapa explicitamente no launch de navegacao ou copie/renomeie para `map.yaml` e `map.pgm`.

## Alternativa preparada para mapeamento

Existe tambem:

```bash
ros2 launch limo_slam slam_toolbox.launch.py
```

Ele sobe `slam_toolbox/async_slam_toolbox_node` com `src/limo_slam/config/slam_toolbox.yaml`. A config esta em `mode: mapping`, usa `odom`, `map`, `base_link` e `scan_topic: /scan`. Pelo rastro em `comandos`, o fluxo usado foi Cartographer; o `slam_toolbox` fica como alternativa para comparar qualidade do mapa.

## Fluxo usado para navegacao

Com a base e o LiDAR rodando, suba o Nav2:

```bash
ros2 launch limo_bringup navigation2.launch.py
```

Esse launch carrega por padrao:

- mapa: `src/limo_ros2/limo_bringup/maps/map.yaml`
- parametros: `src/limo_ros2/limo_bringup/param/navigation2.yaml`
- RViz: `nav2_default_view.rviz`

O `navigation2.yaml` esta configurado para LIMO mecanum/omni:

- AMCL com `robot_model_type: nav2_amcl::OmniMotionModel`
- DWB com `holonomic: True`
- topico de scan em `/scan`
- frames principais `map`, `odom`, `base_link`

Observacao: existe `limo_start_navigation.launch.py`, que tenta compor localizacao + navegacao, mas no estado atual do arquivo ele monta o `LaunchDescription` e nao retorna `ld`. Ate corrigir isso, use `navigation2.launch.py`, que foi o comando registrado.

## Fluxo usado para executar a missao

Depois de Nav2 estar ativo, rode:

```bash
ros2 run limo_mission limo_main
```

Entrypoints equivalentes do pacote:

```bash
ros2 run limo_mission mission_ws
ros2 run limo_mission mission_manager
```

Hoje todos apontam para `limo_mission.main:main`.

O arquivo principal e `src/limo_mission/limo_mission/main.py`. Ele cria:

- `RobotNode`: publica `/initialpose`, publica `/cmd_vel`, assina `/scan` e usa a action `navigate_to_pose`.
- `Navigator`: envia metas para Nav2 (`nav2_msgs/action/NavigateToPose`).
- `DockingController`: executa a maquina de estados de docking usando o LaserScan frontal.
- `MissionManager`: orquestra a fila de missoes.

A fila padrao atual em `main.py` e:

```python
DEFAULT_ROUTE = ["WS01", "WS02", "WS03", "WS04", "WS05"]
```

Cada WS da rota e adicionada como `action='dock'`, entao o fluxo e: navegar ate a WS, fazer docking, sair um pouco da estacao quando ainda houver proxima WS, e seguir para a proxima.

As poses estao em `src/limo_mission/limo_mission/config/poses.py`:

- `WS01`
- `WS02`
- `WS03`
- `WS04`
- `WS05`

Para um novo cenario, esse e o principal arquivo a editar depois de salvar o mapa.

## Maquina de estados da missao

Arquivo: `src/limo_mission/limo_mission/mission_manager.py`

Estados definidos em `src/limo_mission/limo_mission/core/states.py`:

```text
INIT
SET_INITIAL_POSE
WAIT_LOCALIZATION
LOAD_NEXT_MISSION
NAVIGATING
DOCKING
UNDOCKING
FINISHED
ERROR
```

Fluxo:

1. Publica initial pose em `map`.
2. Espera 4 segundos para localizacao estabilizar.
3. Carrega a proxima meta da fila.
4. Envia `NavigateToPose` para Nav2.
5. Se a meta terminar com sucesso:
   - `goto_only`: carrega a proxima meta.
   - `dock`: entra no `DockingController`.
6. Se docking completar e ainda houver proxima WS, entra em `UNDOCKING` e anda um pouco para tras.
7. Carrega a proxima meta.
8. Se a fila acabar, entra em `FINISHED`.

## Maquina de estados do docking

Arquivo: `src/limo_mission/limo_mission/core/docking_controller.py`

Estados:

```text
IDLE
SEARCH
ALIGNING_COARSE
ALIGNING_FINE
RECENTER_FOR_DOCK
APPROACHING
DONE
FAILED
```

O docking atual nao e um launch separado. Ele roda dentro de `ros2 run limo_mission limo_main`.

Entradas:

- `/scan`: LaserScan usado para achar a face frontal da estacao.
- `/cmd_vel`: comando publicado para alinhar, recentralizar e aproximar.

Logica geral:

1. Recorta o setor frontal do LiDAR (`front_sector_deg = 55.0`).
2. Separa clusters por distancia entre pontos.
3. Escolhe um cluster compativel com face de docking.
4. Estima a face por PCA.
5. Calcula distancia frontal, offset lateral, yaw e beta.
6. Classifica se o robo esta pronto para dock ou escapou para esquerda/direita.
7. Controla:
   - yaw grosso em `ALIGNING_COARSE`
   - yaw fino em `ALIGNING_FINE`
   - movimento lateral `linear.y` em `RECENTER_FOR_DOCK`
   - aproximacao `linear.x` em `APPROACHING`

Parametros sensiveis para novo cenario:

- `target_distance`: distancia final da face de docking.
- `ready_offset_tol` e `escape_offset_tol`: tolerancias laterais.
- `ready_beta_deg` e `escape_beta_deg`: tolerancias angulares.
- `invert_y_axis`: inverter se o movimento lateral sair para o lado errado.
- `invert_yaw_sign`: inverter se o giro sair para o lado errado.
- `cal_beta_deg` e `cal_yaw_deg`: calibracao real usada para compensar yaw esperado por beta.

## Dock server alternativo

Existe tambem:

```bash
ros2 run limo_mission dock_server
```

Arquivo: `src/limo_mission/limo_mission/dock_server.py`

Ele cria uma action `/dock` usando `limo_mission_msgs/action/Dock.action`. Esse caminho e util para testar docking como action separada, mas o fluxo principal da missao atual nao chama esse servidor; ele usa o `DockingController` direto dentro do processo `limo_main`.

## Checklist para mapear outro cenario

1. Atualizar e compilar o workspace:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

2. Subir base e LiDAR:

```bash
ros2 launch limo_base limo_base.launch.py
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```

3. Confirmar topicos e TF:

```bash
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo base_link laser_frame
```

4. Subir Cartographer:

```bash
ros2 launch limo_bringup cartographer.launch.py
```

5. Teleoperar pelo ambiente e salvar o mapa com nome novo:

```bash
ros2 run nav2_map_server map_saver_cli -f src/limo_ros2/limo_bringup/maps/<nome_do_cenario>
```

6. Testar Nav2 com o mapa novo:

```bash
ros2 launch limo_bringup navigation2.launch.py map:=/home/limopro/limoatwork/src/limo_ros2/limo_bringup/maps/<nome_do_cenario>.yaml
```

7. No RViz, setar `2D Pose Estimate` e testar `Nav2 Goal`.

8. Capturar poses de aproximacao para docking/estacoes. Rode o capturador e depois envie um `Nav2 Goal` pelo RViz no ponto/orientacao desejado:

```bash
ros2 run limo_mission capture_goal_pose -- --name WS04
```

O utilitario escuta `/goal_pose` por padrao e imprime uma linha pronta para adicionar no `pose_dict` em `src/limo_mission/limo_mission/config/poses.py`. Ele nao grava arquivo automaticamente; por enquanto a captura fica no terminal para ser copiada manualmente.

Se o RViz usado estiver publicando no topico antigo, use:

```bash
ros2 run limo_mission capture_goal_pose -- --name WS04 --topic /move_base_simple/goal
```

Tambem da para conferir manualmente:

```bash
ros2 topic echo /goal_pose --once
```

9. Ajustar a fila em `src/limo_mission/limo_mission/main.py`:

```python
manager.add_mission(pose_dict["NOVA_WS"], action='dock')
```

10. Rodar a missao:

```bash
ros2 run limo_mission limo_main
```

Para rodar apenas um subconjunto ou mudar a ordem:

```bash
ros2 run limo_mission limo_main -- --route WS03 WS01 WS05
```

11. Durante os testes, observar os logs:

```text
STATE: ... -> ...
DOCK STATE: ... -> ...
state=... window=... front=... offset=... yaw_corr=... cmd=(...)
```

Esses logs sao o melhor sinal para calibrar `target_distance`, tolerancias laterais, sentido de `linear.y` e sentido do yaw.

## Arquivos principais para manter no Git

- `docs/guia_mapeamento_missao_docking.md`
- `src/limo_ros2/limo_bringup/maps/<cenario>.yaml`
- `src/limo_ros2/limo_bringup/maps/<cenario>.pgm`
- `src/limo_mission/limo_mission/config/poses.py`
- `src/limo_mission/limo_mission/main.py`
- `src/limo_mission/limo_mission/core/docking_controller.py`
- `src/limo_ros2/limo_bringup/param/navigation2.yaml`
- `src/limo_ros2/ydlidar_ros2/params/ydlidar.yaml`
