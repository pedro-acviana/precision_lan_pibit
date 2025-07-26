import py_trees

from precision_landing.nodos.deteccao import deteccao
from precision_landing.nodos.registrar import registrar
from precision_landing.nodos.cor_detectada import cor_detectada
from precision_landing.nodos.cor_detectada import cor_detectada2
from precision_landing.nodos.rotaciona import rotaciona
from precision_landing.nodos.aproxima import aproxima
from precision_landing.nodos.centraliza import centraliza
from precision_landing.nodos.distancia_segura import distancia_segura
from precision_landing.nodos.ajustar_posicao import ajustar_posicao
from precision_landing.nodos.contornar import contornar


def missao_principal(commander, camera_detect):
    # Subcomponente da missão - código de detecção
    detectado = deteccao("Detecção", commander)

    # Comportamento principal: execução do loop de postes
    repeat_principal = py_trees.decorators.Repeat(
        name="LoopPostes",
        child=criar_loop_postes(commander, camera_detect),
        num_success= 4
    )

    
    return repeat_principal


def criar_loop_postes(commander, camera_detect):
    # Seletor: já detectou a cor?
    fallback_cor = py_trees.composites.Selector(name="Verifica Cor", memory=True)
    verifica_cor = cor_detectada("Cor Detectada?", commander, camera_detect)

    # Paralelo: rotaciona até detectar cor
    paralel_cor = py_trees.composites.Parallel(
        name="Detecta Cor (com Rotação)",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    loop_rotacao = py_trees.decorators.Repeat(
        name="Loop Rotação",
        child=rotaciona("Rotaciona", commander),
        num_success=-1  # Loop infinito até sucesso
    )
    cor_detectada_rotaciona = cor_detectada2("Cor Detectada", commander, camera_detect)
    paralel_cor.add_children([loop_rotacao, cor_detectada_rotaciona])
    fallback_cor.add_children([verifica_cor, paralel_cor])

    # Seletor: já está em distância segura?
    fallback_dist = py_trees.composites.Selector(name="Distância OK?", memory=True)
    verifica_dist = distancia_segura("Distância OK", commander)

    # Paralelo: ajusta posição até alcançar distância segura
    paralel_dist = py_trees.composites.Parallel(
        name="Ajuste até Distância OK",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    loop_ajuste = py_trees.decorators.Repeat(
        name="Loop Ajuste Posição",
        child=ajustar_posicao("Ajustar Posição", commander),
        num_success=-1
    )
    distancia_ok2 = distancia_segura("Distância OK Final", commander)
    paralel_dist.add_children([loop_ajuste, distancia_ok2])
    fallback_dist.add_children([verifica_dist, paralel_dist])

    # Sequência final do poste
    seq_poste = py_trees.composites.Sequence(name="Passar Poste", memory=True)
    seq_poste.add_children([
        fallback_cor,
        aproxima("Aproxima", commander),
        centraliza("Centraliza", commander),
        fallback_dist,
        contornar("Contornar", commander),
        registrar("Registra", commander)
    ])

    return seq_poste
