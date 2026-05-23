# Turtle Draw — Relatório Técnico

**Aluno:** Pietro Alkmin · **Disciplina:** Robótica e Visão Computacional · **Data:** 22/05/2026

## Visão geral

A pipeline é uma sequência simples de cinco operações em NumPy. A ideia central é **separar o problema em duas fases**: primeiro um threshold de intensidade isola o cachorro (escuro) do fundo (claro), eliminando o ruído de textura da parede e do pelo; em seguida o filtro Sobel é aplicado sobre essa máscara binária limpa para extrair a borda da silhueta. A tartaruga então varre a imagem linha a linha (estilo impressora matricial), traçando segmentos onde há pixels de borda.

Tudo está em dois arquivos: `pipeline.py` (visão) e `draw_node.py` (controle ROS 2).

## Pipeline de visão computacional

**1. Cinza por média dos canais.** Convertemos BGR → cinza pela média dos 3 canais (`img.mean(axis=2) / 255`). Para threshold de intensidade não precisamos de informação de cor — apenas de quão claro/escuro é cada pixel. A média é a forma mais simples e funciona bem para esta imagem.

**2. Redimensionamento.** A foto original tem 720×1280 (~900k pixels). Cada linha da imagem reduzida vira uma passada da "impressora", e a tartaruga é lenta. Reduzir para 70 px de largura (→ 39 linhas) mantém a silhueta reconhecível com quantidade gerenciável de pontos a percorrer. Para cada pixel da imagem reduzida, pego o pixel mais próximo da imagem original — sem interpolar.

**3. Binarização por threshold de intensidade.** Aplicamos `cinza < 0.45` gerando uma máscara onde os pixels marcados como `True` representam "cachorro". Como o cachorro é visivelmente mais escuro que o fundo, comparar com um limiar fixo já basta. **Esta etapa é importante porque limpa o problema antes da detecção de bordas:** aplicar Sobel direto na foto original gera centenas de bordas falsas (tijolos da parede, textura do pelo).

**4. Detecção de bordas com filtro Sobel.** Borda é uma mudança brusca de intensidade entre pixels vizinhos. O Sobel calcula a derivada da imagem em x e em y usando dois kernels 3×3:

$$
K_x = \begin{bmatrix}-1 & 0 & 1\\ -2 & 0 & 2 \\ -1 & 0 & 1\end{bmatrix},\quad
K_y = \begin{bmatrix}-1 & -2 & -1 \\ 0 & 0 & 0 \\ 1 & 2 & 1\end{bmatrix}
$$

A aplicação do filtro é manual: somo nove versões deslocadas da imagem multiplicadas pelos pesos dos kernels. A magnitude $\sqrt{G_x^2 + G_y^2}$ mede a "força" da borda em cada pixel. Como a entrada é a máscara binária (silhueta limpa), o resultado destaca **exatamente o contorno externo do cachorro** mais alguns detalhes internos (peito branco). Aplicamos `magnitude > 0.5` para gerar um mapa de bordas.

**5. Varredura linha a linha e mapeamento.** Para cada linha do mapa de bordas, percorremos os pixels da esquerda para a direita agrupando pixels contíguos de borda num segmento (início e fim). Trechos sem borda são pulados (a caneta sobe). Depois cada segmento é mapeado da coordenada da imagem para a coordenada do turtlesim, invertendo o eixo y (a imagem tem origem no topo; o turtlesim no chão) e centralizando no canvas 11×11.

## Controle ROS 2

O nó `draw_node` percorre quatro fases em sequência (`WAIT` → `TRAVEL` ↔ `DRAW` → `DONE`) usando **controle proporcional**:

$$
\rho = \sqrt{\Delta x^2 + \Delta y^2}, \quad
\alpha = \arctan2(\Delta y, \Delta x) - \theta
$$
$$
v_{lin} = K_p^{lin}\,\rho, \quad
v_{ang} = K_p^{ang}\,\alpha
$$

Onde $\rho$ é a distância até o próximo ponto e $\alpha$ é o erro de direção. Com $K_p^{lin}=1.5$, $K_p^{ang}=6.0$ e velocidades limitadas. Quando $|\alpha| > 0.25$ rad, $v_{lin}$ é zerado para a tartaruga girar primeiro — necessário aqui, porque entre o fim de uma linha e o início da próxima ela precisa virar quase 180°. Sem essa condição, ela desenharia curvas indesejadas.

Entre segmentos a caneta é levantada via `/turtle1/set_pen` (chamada sem aguardar resposta, pois bloquear dentro do laço de controle travaria o nó). O ângulo $\alpha$ é mantido no intervalo $(-\pi, \pi]$ usando `atan2(sin(x), cos(x))`, evitando que a tartaruga gire pelo caminho longo.


## Resultado

A pipeline gera 95 segmentos / 190 waypoints a partir da foto do bulldog. O desenho final no turtlesim reproduz claramente o contorno do cachorro: as duas orelhas, a cabeça, o tronco e as patas dianteiras são reconhecíveis.
