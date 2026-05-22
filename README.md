# Turtle Draw

Pega uma foto, isola o objeto escuro do fundo claro com **threshold**, detecta o contorno da silhueta com **filtro Sobel**, e a tartaruga do `turtlesim` percorre o contorno varrendo linha a linha.

## Estrutura

```
turtle_circle/
├── images/dog.png              ← imagem de entrada
├── output/                     ← gerado pela pipeline (contour.json + preview)
├── turtle_circle/
│   ├── pipeline.py             ← visão computacional (5 funções)
│   └── draw_node.py            ← nó ROS 2 que desenha
├── notebook_pipeline.ipynb     ← walkthrough da pipeline com gráficos
├── setup.py
└── package.xml
```

## Restrições atendidas

- `cv2` é usado **em uma única linha** (`cv2.imread` para carregar a imagem).
- Todo o resto é NumPy puro.

## Como rodar

### 1. Gerar o contour.json

```bash
cd ~/ros2_ws/src/turtle_circle
python3 -m turtle_circle.pipeline --imagem images/dog.png --saida output
```

Parâmetros opcionais: `--largura` (default 70), `--limiar-intensidade` (default 0.45), `--limiar-borda` (default 0.5).

### 2. Compilar e rodar

```bash
cd ~/ros2_ws && colcon build --packages-select turtle_circle && source install/setup.bash

# Terminal 1
ros2 run turtlesim turtlesim_node

# Terminal 2
ros2 run turtle_circle draw_node --ros-args -p arquivo_contorno:=$HOME/ros2_ws/src/turtle_circle/output/contour.json
```

## Pipeline (5 etapas)

| Função | O que faz |
| --- | --- |
| `carregar` | Lê a imagem do disco com `cv2.imread` |
| `para_cinza` | Média dos 3 canais (BGR) → tom de cinza em [0, 1] |
| `redimensionar` | Reduz a imagem (cada linha vira uma "passada" da impressora) |
| `binarizar_escuros` | Threshold de intensidade: pixel < limiar = cachorro |
| `sobel` | Filtro Sobel sobre a máscara binária: detecta a borda da silhueta |
| `varrer_linhas` | Para cada linha, extrai segmentos contíguos de borda |
| `mapear_turtlesim` | Converte (i, j) da imagem para o canvas 11×11 do turtlesim |

## Controle no `draw_node.py`

Para cada waypoint alvo, controle proporcional clássico de go-to-goal:

- `rho` = distância até o alvo
- `alpha` = diferença entre o ângulo desejado e o atual
- `v_linear = Kp_lin · rho` (zerado se `|alpha|` for grande — gira primeiro)
- `v_angular = Kp_ang · alpha`

Entre segmentos a caneta é levantada com `/turtle1/set_pen`.
