"""
Pipeline de visao computacional para o Turtle Draw.

Estrategia em duas fases:
  - Threshold isola o cachorro (mais escuro) do fundo (parede clara).
    Isso elimina ruido de textura (tijolos, pelo) que atrapalharia uma
    deteccao de bordas aplicada diretamente na foto.
  - Filtro Sobel sobre a mascara binaria detecta a borda da silhueta.
    Como a entrada e binaria e limpa, o Sobel destaca exatamente o
    contorno externo do cachorro.

Depois varremos linha a linha (estilo impressora) e a tartaruga traca
cada trecho de borda em cada linha. Resultado: contorno do cachorro.
"""

import argparse
import json
import os

import cv2  # apenas para carregar a imagem
import numpy as np


def carregar(caminho):
    """Le a imagem do disco. Retorna array BGR uint8."""
    img = cv2.imread(caminho)
    if img is None:
        raise FileNotFoundError(caminho)
    return img


def para_cinza(img_bgr):
    """Converte BGR -> cinza pela media dos 3 canais. Resultado em [0, 1]."""
    return img_bgr.mean(axis=2) / 255.0


def redimensionar(img, nova_largura):
    """
    Reduz a imagem para 'nova_largura' pixels, mantendo proporcao.

    Nearest-neighbor manual: para cada pixel da imagem reduzida,
    pegamos o pixel mais proximo na imagem original.
    """
    h, w = img.shape
    nova_altura = int(h * nova_largura / w)
    iy = (np.arange(nova_altura) * h / nova_altura).astype(int)
    ix = (np.arange(nova_largura) * w / nova_largura).astype(int)
    return img[np.ix_(iy, ix)]


def binarizar_escuros(img_cinza, limiar=0.45):
    """Retorna True onde o pixel e escuro (= parte do cachorro)."""
    return img_cinza < limiar


def sobel(img):
    """
    Filtro Sobel para detectar bordas.

    Borda e uma mudanca brusca de intensidade. O Sobel calcula a derivada
    da imagem em x e em y usando dois kernels 3x3, e a magnitude
    sqrt(Gx^2 + Gy^2) mede a 'forca' da borda em cada pixel.
    """
    # Kernels classicos do Sobel (Kx detecta bordas verticais, Ky horizontais)
    Kx = np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]], dtype=float)
    Ky = np.array([[-1, -2, -1], [0, 0, 0], [1, 2, 1]], dtype=float)

    img = img.astype(float)
    h, w = img.shape
    # Padding "edge" replica as bordas para a convolucao nao sair do array
    pad = np.pad(img, 1, mode='edge')
    gx = np.zeros_like(img)
    gy = np.zeros_like(img)
    # Convolucao manual: cada (u,v) e uma janela deslocada da imagem.
    # Em vez de iterar pixel por pixel, somamos as 9 janelas vetorizadas
    # multiplicadas pelos pesos do kernel - isso e equivalente a aplicar
    # o kernel em cada pixel, mas usando operacoes numpy.
    for u in range(3):
        for v in range(3):
            janela = pad[u:u + h, v:v + w]
            gx += Kx[u, v] * janela
            gy += Ky[u, v] * janela
    # Magnitude do gradiente: mede a "forca" da borda em cada pixel
    return np.sqrt(gx ** 2 + gy ** 2)


def varrer_linhas(bordas):
    """
    Varredura horizontal estilo impressora.

    Para cada linha, agrupa pixels de borda contiguos num segmento. Usa
    np.diff: a derivada da linha binaria marca onde comecam (+1) e
    terminam (-1) os trechos True.
    """
    segmentos = []
    for i, linha in enumerate(bordas):
        # Truque do np.diff: derivada de uma linha binaria marca +1 onde
        # comeca um trecho True e -1 onde termina. prepend/append=0
        # garante que detectamos trechos colados nas bordas da linha.
        d = np.diff(linha.astype(int), prepend=0, append=0)
        inicios = np.where(d == 1)[0]
        fins = np.where(d == -1)[0] - 1
        for ini, fim in zip(inicios, fins):
            segmentos.append([(i, int(ini)), (i, int(fim))])
    return segmentos


def mapear_turtlesim(pontos, h, w, tam=11.0, margem=0.5):
    """
    Converte (i, j) da imagem para (x, y) do turtlesim.

    A imagem tem origem no topo (y para baixo); o turtlesim no chao
    (y para cima), canvas ~11x11. Mantem proporcao e centraliza.
    """
    util = tam - 2 * margem
    # Mesma escala em x e y preserva a proporcao da imagem
    escala = min(util / w, util / h)
    # Offsets centralizam a silhueta no canvas
    off_x = (tam - w * escala) / 2
    off_y = (tam - h * escala) / 2
    # (h - i) inverte o eixo y: imagem tem origem no topo, turtlesim no chao
    return [(j * escala + off_x, (h - i) * escala + off_y) for i, j in pontos]


def executar(imagem, saida='output', largura=70,
             limiar_intensidade=0.45, limiar_borda=0.5):
    """Roda a pipeline inteira e salva os waypoints em contour.json."""
    os.makedirs(saida, exist_ok=True)

    # As 5 etapas da pipeline em sequencia
    cinza = para_cinza(carregar(imagem))               # 1. pre-processamento
    pequena = redimensionar(cinza, largura)            # 2. resize agressivo
    mascara = binarizar_escuros(pequena, limiar_intensidade)  # 3. silhueta
    bordas = sobel(mascara) > limiar_borda             # 4. deteccao de bordas

    # 5. planejamento de caminho: cada linha vira N segmentos para a tartaruga
    h, w = bordas.shape
    tracos = [mapear_turtlesim(seg, h, w) for seg in varrer_linhas(bordas)]

    caminho_json = os.path.join(saida, 'contour.json')
    with open(caminho_json, 'w') as f:
        json.dump({'tracos': tracos}, f)

    print(f'{len(tracos)} segmentos, {sum(len(t) for t in tracos)} waypoints')
    print(f'Salvo: {caminho_json}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--imagem', required=True)
    parser.add_argument('--saida', default='output')
    parser.add_argument('--largura', type=int, default=70)
    parser.add_argument('--limiar-intensidade', type=float, default=0.45)
    parser.add_argument('--limiar-borda', type=float, default=0.5)
    args = parser.parse_args()
    executar(args.imagem, args.saida, args.largura,
             args.limiar_intensidade, args.limiar_borda)


if __name__ == '__main__':
    main()
