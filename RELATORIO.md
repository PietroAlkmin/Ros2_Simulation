# Turtle Draw — Relatório Técnico

**Aluno:** Pietro Alkmin · **Disciplina:** Robótica e Visão Computacional · **Data:** 22/05/2026

## Visão geral

A pipeline é uma sequência simples de cinco operações em NumPy. A ideia central é **separar o problema em duas fases**: primeiro um threshold de intensidade isola o cachorro (escuro) do fundo (claro), eliminando o ruído de textura da parede e do pelo; em seguida o filtro Sobel é aplicado sobre essa máscara binária limpa para extrair a borda da silhueta. A tartaruga então varre a imagem linha a linha (estilo impressora matricial), traçando segmentos onde há pixels de borda.

Tudo está em dois arquivos: `pipeline.py` (visão) e `draw_node.py` (controle ROS 2).

## Pipeline de visão computacional

**1. Cinza por média dos canais.** Convertemos BGR → cinza pela média dos 3 canais (`img.mean(axis=2) / 255`). Para threshold de intensidade não precisamos de informação de cor — apenas de quão claro/escuro é cada pixel. A média é a forma mais simples e funciona bem para esta imagem.

**2. Redimensionamento.** A foto original tem 720×1280 (~900k pixels). Cada linha da imagem reduzida vira uma passada da "impressora", e a tartaruga é lenta. Reduzir para 70 px de largura (→ 39 linhas) mantém a silhueta reconhecível com quantidade gerenciável de waypoints. Uso indexação por step (`np.arange(...) * h / nova_altura`), equivalente a *nearest neighbor* manual — sem interpolar.

**3. Binarização por threshold de intensidade.** Aplicamos `cinza < 0.45` gerando uma máscara booleana onde `True` representa "cachorro". Como o cachorro é visivelmente mais escuro que o fundo, o threshold global é suficiente — não precisamos de Otsu, adaptativo, ou métodos mais sofisticados. **Esta etapa é importante porque limpa o problema antes da detecção de bordas:** aplicar Sobel direto na foto original gera centenas de bordas espúrias (tijolos da parede, textura do pelo).

**4. Detecção de bordas com filtro Sobel.** Borda é uma mudança brusca de intensidade entre pixels vizinhos. O Sobel calcula a derivada da imagem em x e em y usando dois kernels 3×3:

$$
K_x = \begin{bmatrix}-1 & 0 & 1\\ -2 & 0 & 2 \\ -1 & 0 & 1\end{bmatrix},\quad
K_y = \begin{bmatrix}-1 & -2 & -1 \\ 0 & 0 & 0 \\ 1 & 2 & 1\end{bmatrix}
$$

A convolução é manual: faço `np.pad` na imagem e somo as 9 janelas deslocadas multiplicadas pelos pesos do kernel. A magnitude $\sqrt{G_x^2 + G_y^2}$ mede a "força" da borda em cada pixel. Como a entrada é a máscara binária (silhueta limpa), o resultado destaca **exatamente o contorno externo do cachorro** mais alguns detalhes internos (peito branco). Aplicamos `magnitude > 0.5` para gerar um mapa binário de bordas.

**5. Varredura linha a linha e mapeamento.** Para cada linha do mapa de bordas, percorremos os pixels da esquerda para a direita agrupando pixels contíguos de borda num segmento `[(i, j_início), (i, j_fim)]`. Trechos sem borda são pulados (a caneta sobe). Depois cada segmento é mapeado da coordenada (i, j) da imagem para (x, y) do turtlesim, invertendo o eixo y (a imagem tem origem no topo; o turtlesim no chão) e centralizando no canvas 11×11.

## Controle ROS 2

O nó `draw_node` implementa uma máquina de estados com 4 estados (`WAIT` → `TRAVEL` ↔ `DRAW` → `DONE`) e a lei de **controle proporcional** clássica de *go-to-goal*:

$$
\rho = \sqrt{\Delta x^2 + \Delta y^2}, \quad
\alpha = \mathrm{wrap}\big(\arctan2(\Delta y, \Delta x) - \theta\big)
$$
$$
v_{lin} = K_p^{lin}\,\rho, \quad
v_{ang} = K_p^{ang}\,\alpha
$$

Com $K_p^{lin}=1.5$, $K_p^{ang}=6.0$ e saturações. Quando $|\alpha| > 0.25$ rad, $v_{lin}$ é zerado para a tartaruga girar primeiro — fundamental aqui, porque entre o fim de uma linha e o início da próxima ela precisa virar quase 180°. Sem essa condição, ela desenharia arcos no carry-return.

Entre segmentos a caneta é levantada via `/turtle1/set_pen` (chamada *fire-and-forget* com `call_async`, sem aguardar — `spin_until_future_complete` dentro de um callback do timer trava o executor). A normalização de $\alpha$ para $(-\pi, \pi]$ usa `atan2(sin(x), cos(x))`, evitando giro pelo caminho longo.

## Dificuldades

1. **Sobel direto na foto não funciona.** Primeira tentativa: Sobel direto na imagem em cinza. Resultado: centenas de bordas espúrias da textura dos tijolos e do pelo do cachorro. Aumentar o limiar perdia o contorno; baixar enchia de ruído. **Solução: aplicar threshold antes do Sobel** — a máscara binária é limpa e o Sobel destaca só o contorno da silhueta.
2. **Carry-return da impressora.** Sem zerar $v_{lin}$ quando $|\alpha|$ é grande, a tartaruga descrevia arcos longos entre o fim de uma linha e o início da próxima. Com a condição `|α| > 0.25 → v_lin = 0`, ela gira no lugar e parte reto.
3. **Service call dentro de timer callback.** O ROS 2 não permite chamar `spin_until_future_complete` de dentro de um callback (o executor já está spinando). Mudei o `set_pen` para *fire-and-forget*.
4. **Wrap-around angular.** Sem `normalizar(α)`, a tartaruga ocasionalmente girava o caminho longo (>180°). `atan2(sin(x), cos(x))` resolve.

## Resultado

A pipeline gera 95 segmentos / 190 waypoints a partir da foto do bulldog. O desenho final no turtlesim reproduz claramente o contorno do cachorro: as duas orelhas, a cabeça, o tronco e as patas dianteiras são reconhecíveis.
