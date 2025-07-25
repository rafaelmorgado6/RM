# -*- coding: utf-8 -*-

'''
O algoritmo de Bresenham é uma forma eficiente de desenhar uma linha reta entre dois pontos
numa grelha de coordenadas inteiras (por exemplo, num mapa ou imagem digital). 
Em vez de usar cálculos com números reais ou funções trigonométricas (que são mais lentos),
o algoritmo usa apenas operações inteiras para determinar os pontos que mais se aproximam da linha ideal.

Determina quais células de uma grelha são atravessadas por uma linha, como um raio laser num mapeamento de ocupação.
'''

def bresenham(x0, y0, x1, y1):
    """Yield integer coordinates on the line from (x0, y0) to (x1, y1).

    Input coordinates should be integers.

    The result will contain both the start and the end point.
    """
    dx = x1 - x0
    dy = y1 - y0

    xsign = 1 if dx > 0 else -1
    ysign = 1 if dy > 0 else -1

    dx = abs(dx)
    dy = abs(dy)

    if dx > dy:
        xx, xy, yx, yy = xsign, 0, 0, ysign
    else:
        dx, dy = dy, dx
        xx, xy, yx, yy = 0, ysign, xsign, 0

    D = 2*dy - dx
    y = 0

    for x in range(dx + 1):
        yield x0 + x*xx + y*yx, y0 + x*xy + y*yy
        if D >= 0:
            y += 1
            D -= 2*dx
        D += 2*dy