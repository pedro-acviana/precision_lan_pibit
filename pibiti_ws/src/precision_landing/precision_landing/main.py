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
    root = py_trees.composites.Sequence(name="Missao Precision Landing", memory=True)
    
    # Criar o nodo de detecção de câmera (achar_local_seguro)
    camera_detect = achar_local_seguro("Achar Local Seguro", commander)
    
    # 1. Sequência de takeoff (deve completar primeiro)
    takeoff_seq = BehaviorFactory.create_takeoff_sequence(commander, camera_detect)
    
    # 2. Buscar local seguro (após takeoff completo)
    buscar_local = camera_detect
    
    # 3. Converter posição da imagem para coordenadas NED
    converter_coord = img2local("Converter Imagem->NED")
    
    # 4. Aproximar do local usando comandos de velocidade
    aproximar = aproxima("Aproximar Local Seguro", commander)
    
    # 5. Sequência de pouso
    landing_seq = BehaviorFactory.create_landing_sequence(commander)

    # Adiciona todos os nodos na sequência principal (execução sequencial)
    root.add_children([
        takeoff_seq,        # 1º: Takeoff (deve completar antes de prosseguir)
        buscar_local,       # 2º: Buscar local seguro (5s de estabilização)
        converter_coord,    # 3º: Converter pixel para NED
        aproximar,         # 4º: Aproximar do local
        landing_seq        # 5º: Pousar
    ])
    
    tree = py_trees.trees.BehaviourTree(root)
    tree.setup(timeout=15)
    
    # Configurar manualmente o img2local com o node
    # Buscar o nó img2local na árvore e configurá-lo
    def find_and_setup_img2local(node):
        if isinstance(node, img2local):
            node.setup(node=commander)
        # Recursivamente buscar em nós compostos
        if hasattr(node, 'children'):
            for child in node.children:
                find_and_setup_img2local(child)
    
    find_and_setup_img2local(root)
    
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
