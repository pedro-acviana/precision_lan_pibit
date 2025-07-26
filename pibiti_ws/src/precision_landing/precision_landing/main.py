import time
import rclpy
import py_trees
import subprocess
import py_trees.display
import os

from precision_landing.nodos.achar_local_seguro import achar_local_seguro
from precision_landing.nodos.img2local import img2local  
from precision_landing.nodos.aproxima import aproxima
from precision_landing.nodos.achar_local_seguro import achar_local_seguro
from precision_landing.nodos.img2local import img2local
from precision_landing.nodos.aproxima import aproxima
from drone_behaviors.behavior_factory import BehaviorFactory
from drone_behaviors.commander.px4_commander import PX4Commander


def build_behavior_tree(commander):
    root = py_trees.composites.Sequence(name="Missao Precision Landing", memory=True)
    
    # Criar o nodo de detecção de câmera (achar_local_seguro)
    camera_detect = achar_local_seguro("Achar Local Seguro", commander)
    
    # Passe camera_detect para takeoff_seq
    takeoff_seq = BehaviorFactory.create_takeoff_sequence(commander, camera_detect)
    
    # Nova sequência da missão principal
    missao_seq = py_trees.composites.Sequence(name="Missao Principal", memory=True)
    
    # 2. Converter posição da imagem para coordenadas NED
    converter_coord = img2local("Converter Imagem->NED", commander)
    
    # 3. Aproximar do local usando comandos de velocidade
    aproximar = aproxima("Aproximar Local Seguro", commander)
    
    # Adiciona os nodos na sequência principal (sem o buscar_local pois já está no paralelo)
    missao_seq.add_children([
        takeoff_seq,
        converter_coord, 
        aproximar
    ])
    
    # Sequência de pouso
    landing_seq = BehaviorFactory.create_landing_sequence(commander)

    # Configuração em paralelo similar ao exemplo
    seq_missao = py_trees.composites.Parallel(
        name="Missao em Paralelo", 
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )

    seq_missao.add_children([
        camera_detect,  # Nodo de detecção rodando em paralelo
        missao_seq      # Sequência principal da missão
    ])

    # Árvore final: Missão em Paralelo -> Pouso
    root.add_children([
        seq_missao,
        landing_seq
    ])
    
    tree = py_trees.trees.BehaviourTree(root)
    tree.setup(timeout=15)
    return tree


def gerar_imagem_arvore(tree):
    """
    Gera imagem da árvore de comportamento no formato .png
    """
    # Criar diretório se não existir
    import os
    os.makedirs("img", exist_ok=True)
    
    # Gera o arquivo .dot
    py_trees.display.render_dot_tree(
        root=tree.root,
        name="arvore_precision_landing",
        target_directory="img",
        
    )

    # Converte o .dot para .png usando o Graphviz
    subprocess.run(["dot", "-Tpng", "img/arvore_precision_landing.dot", "-o", "img/arvore_precision_landing.png"])
    print("Imagem da árvore salva como 'img/arvore_precision_landing.png'")


def main(args=None):
    rclpy.init(args=args)
    blackboard = py_trees.blackboard.Blackboard()
    
    # Inicializa variáveis do blackboard se necessário
    blackboard.set("local_seguro_encontrado", False)

    commander = PX4Commander()
    tree = build_behavior_tree(commander)

    print("PID:", os.getpid())  # Print the process ID

    # Gera a imagem da árvore de comportamento
    # gerar_imagem_arvore(tree)  # Comentado para evitar dependência do Graphviz

    try:
        while rclpy.ok():
            rclpy.spin_once(commander, timeout_sec=0.1)
            tree.tick()
            print(py_trees.display.unicode_tree(tree.root, show_status=True))

            if tree.root.status == py_trees.common.Status.SUCCESS:
                commander.get_logger().info("Missão de precision landing completa.")
                break
            elif tree.root.status == py_trees.common.Status.FAILURE:
                commander.get_logger().error("Missão falhou.")
                break

            time.sleep(0.05)  # Tick rate ~20Hz
    except KeyboardInterrupt:
        commander.get_logger().info("Missão interrompida pelo usuário.")
    finally:
        commander.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
