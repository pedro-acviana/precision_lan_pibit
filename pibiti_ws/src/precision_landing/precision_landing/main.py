import time
import rclpy
import py_trees
import subprocess
import py_trees.display
import os

from precision_landing.nodos.achar_local_seguro import achar_local_seguro
from precision_landing.nodos.img2local import img2local  
from precision_landing.nodos.aproxima import aproxima
from drone_behaviors.behavior_factory import BehaviorFactory
from drone_behaviors.commander.px4_commander import PX4Commander


def build_behavior_tree(commander):
    root = py_trees.composites.Parallel(
        name="Missao Precision Landing", 
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    
    # Criar o nodo de detecção de câmera (achar_local_seguro) - roda em paralelo
    camera_detect = achar_local_seguro("Achar Local Seguro", commander)
    
    # Sequência principal da missão
    missao_seq = py_trees.composites.Sequence(name="Sequencia Principal", memory=True)
    
    # 1. Sequência de takeoff (deve completar primeiro)
    takeoff_seq = BehaviorFactory.create_takeoff_sequence(commander, camera_detect)
    
    # 2. Converter posição da imagem para coordenadas NED (aguarda automaticamente a estabilização)
    converter_coord = img2local("Converter Imagem->NED")
    
    # 3. Aproximar do local e realizar pouso automaticamente
    aproximar = aproxima("Aproximar Local Seguro", commander)
    
    # Adiciona todos os nodos na sequência principal
    missao_seq.add_children([
        takeoff_seq,        # 1º: Takeoff (deve completar antes de prosseguir)
        converter_coord,    # 2º: Aguarda estabilização e converte coordenadas
        aproximar          # 3º: Aproximar do local e realizar pouso completo
    ])
    
    # Adiciona tanto o detector de câmera quanto a sequência principal em paralelo
    root.add_children([
        camera_detect,      # Roda em paralelo durante toda a missão
        missao_seq         # Sequência principal da missão
    ])
    
    tree = py_trees.trees.BehaviourTree(root)
    tree.setup(timeout=15)
    
    # Configurar manualmente os nodos que precisam de setup específico
    def configure_custom_nodes(node):
        """Configura nodos que precisam de setup personalizado"""
        if isinstance(node, img2local):
            node.setup(node=commander)
        elif isinstance(node, aproxima):
            node.setup()
        
        # Recursivamente buscar em nós compostos
        if hasattr(node, 'children'):
            for child in node.children:
                configure_custom_nodes(child)
    
    configure_custom_nodes(root)
    
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

    # Variáveis para tracking de mudanças
    previous_tree_status = None
    previous_tree_display = None
    
    try:
        while rclpy.ok():
            rclpy.spin_once(commander, timeout_sec=0.1)
            tree.tick()
            
            # Verifica se houve mudança no status da árvore
            current_tree_status = tree.root.status
            current_tree_display = py_trees.display.unicode_tree(tree.root, show_status=True)
            
            # Só imprime se houver mudança no status ou na estrutura da árvore
            status_changed = (current_tree_status != previous_tree_status)
            display_changed = (current_tree_display != previous_tree_display)
            
            if status_changed or display_changed:
                print("\n" + "="*60)
                print(f"🌳 ÁRVORE ATUALIZADA - Status: {current_tree_status}")
                print("="*60)
                print(current_tree_display)
                print("="*60 + "\n")
                
                # Atualiza os valores anteriores
                previous_tree_status = current_tree_status
                previous_tree_display = current_tree_display

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
