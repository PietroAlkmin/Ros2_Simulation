import argparse
import json
import os

import cv2  # apenas para carregar a imagem
import numpy as np


def carregar(caminho):
    img = cv2.imread(caminho)
    if img is None:
        raise FileNotFoundError(caminho)
    return img


def para_cinza(img_bgr):
    # Media dos 3 canais (BGR) -> tom de cinza em [0, 1]
    return img_bgr.mean(axis=2) / 255.0


def redimensionar(img, nova_largura):
    # Reduz mantendo a proporcao, pegando o pixel mais proximo (sem interpolar)
    h, w = img.shape
    nova_altura = int(h * nova_largura / w)
    iy = (np.arange(nova_altura) * h / nova_altura).astype(int)
    ix = (np.arange(nova_largura) * w / nova_largura).astype(int)
    return img[np.ix_(iy, ix)]


def binarizar_escuros(img_cinza, limiar=0.45):
    # True onde o pixel e mais escuro que o limiar (= parte do cachorro)
    return img_cinza < limiar


def sobel(img):
    # Filtro Sobel: detecta mudancas bruscas de intensidade (bordas).
    # Kx pega variacao horizontal, Ky pega variacao vertical.
    Kx = np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]], dtype=float)
    Ky = np.array([[-1, -2, -1], [0, 0, 0], [1, 2, 1]], dtype=float)

    img = img.astype(float)
    h, w = img.shape
    # Padding "edge" replica as bordas para o filtro nao sair do array
    pad = np.pad(img, 1, mode='edge')
    gx = np.zeros_like(img)
    gy = np.zeros_like(img)
    # Soma das 9 janelas deslocadas multiplicadas pelos pesos do kernel
    for u in range(3):
        for v in range(3):
            janela = pad[u:u + h, v:v + w]
            gx += Kx[u, v] * janela
            gy += Ky[u, v] * janela
    # Magnitude: mede a "forca" da borda em cada pixel
    return np.sqrt(gx ** 2 + gy ** 2)


def varrer_linhas(bordas):
    # Varre cada linha da esquerda para a direita e agrupa pixels de borda
    # contiguos num segmento. np.diff marca onde comeca (+1) e termina (-1)
    # cada trecho True; prepend/append=0 capturam trechos colados nas bordas.
    segmentos = []
    for i, linha in enumerate(bordas):
        d = np.diff(linha.astype(int), prepend=0, append=0)
        inicios = np.where(d == 1)[0]
        fins = np.where(d == -1)[0] - 1
        for ini, fim in zip(inicios, fins):
            segmentos.append([(i, int(ini)), (i, int(fim))])
    return segmentos


def mapear_turtlesim(pontos, h, w, tam=11.0, margem=0.5):
    # Converte (i, j) da imagem para (x, y) do turtlesim
    util = tam - 2 * margem
    # Mesma escala em x e y preserva a proporcao da imagem
    escala = min(util / w, util / h)
    # Centraliza a silhueta no canvas
    off_x = (tam - w * escala) / 2
    off_y = (tam - h * escala) / 2
    # (h - i) inverte o eixo y: imagem tem origem no topo, turtlesim no chao
    return [(j * escala + off_x, (h - i) * escala + off_y) for i, j in pontos]


def executar(imagem, saida='output', largura=70,
             limiar_intensidade=0.45, limiar_borda=0.5):
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
