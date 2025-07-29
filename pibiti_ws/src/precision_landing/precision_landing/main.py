import time
import rclpy
import py_trees
import subprocess
import py_trees.display
import os

from precision_landing.nodos.achar_local_seguro import achar_local_seguro
from precision_landing.nodos.img2local import img2local  
from precision_landing.nodos.aproxima import aproxima
from precision_landing.nodos.calibracao_dinamica import calibracao_dinamica
from precision_landing.nodos.camera_feed import camera_feed
from drone_behaviors.behavior_factory import BehaviorFactory
from drone_behaviors.commander.px4_commander import PX4Commander


def build_behavior_tree(commander):
    root = py_trees.composites.Sequence(name="Missao Precision Landing", memory=True)
    
    # Criar nós do sistema de precisão
    camera_feed_node = camera_feed("Camera Feed Continuo", commander)
    camera_detect = achar_local_seguro("Achar Local Seguro", commander)
    calibracao_node = calibracao_dinamica("Calibracao Dinamica", commander)
    converter_coord = img2local("Converter Imagem->NED")
    aproximar_node = aproxima("Aproximar Local Seguro", commander)
    
    # 1. Sequência de takeoff (deve completar primeiro)
    takeoff_seq = BehaviorFactory.create_takeoff_sequence(commander, camera_detect)
    
    # 2. Criar comportamento paralelo para camera feed (roda continuamente)
    parallel_camera = py_trees.composites.Parallel(
        name="Sistema Camera Paralelo",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()  # Sucesso quando um filho completa
    )
    
    # 3. Sequência principal da missão (após takeoff)
    mission_sequence = py_trees.composites.Sequence(name="Sequencia Missao Principal", memory=True)
    mission_sequence.add_children([
        # calibracao_node,    # 2º: Calibração dinâmica (temporariamente desabilitada)
        camera_detect,      # 2º: Buscar local seguro (5s de estabilização) 
        converter_coord,    # 3º: Converter pixel para NED (usa calibração se disponível)
        aproximar_node      # 4º: Aproximar do local e pousar
    ])
    
    # Camera feed roda em paralelo com a missão principal
    parallel_camera.add_children([
        camera_feed_node,   # Feed contínuo da câmera (ÚNICO acesso à câmera física)
        mission_sequence    # Sequência principal da missão
    ])
    
    # 2. Sequência principal da missão (após takeoff)
    root.add_children([
        takeoff_seq,        # 1º: Takeoff (deve completar antes de prosseguir)
        parallel_camera     # 2º: Camera feed + missão principal em paralelo
    ])
    
    tree = py_trees.trees.BehaviourTree(root)
    tree.setup(timeout=15)
    
    # Configurar manualmente os nós que precisam de setup
    def setup_custom_nodes(node):
        if isinstance(node, img2local):
            node.setup(node=commander)
        elif isinstance(node, aproxima):
            node.setup()
        # elif isinstance(node, calibracao_dinamica):
        #     node.setup()  # Temporariamente desabilitado
        elif isinstance(node, camera_feed):
            node.setup()
        elif isinstance(node, achar_local_seguro):
            node.setup()
        
        # Recursivamente buscar em nós compostos
        if hasattr(node, 'children'):
            for child in node.children:
                setup_custom_nodes(child)
    
    setup_custom_nodes(root)
    
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
    
    # Inicializa variáveis do blackboard
    blackboard.set("local_seguro_encontrado", False)
    blackboard.set("calibration_available", False)
    blackboard.set("camera_pronta", False)

    commander = PX4Commander()
    tree = build_behavior_tree(commander)

    print("PID:", os.getpid())  # Print the process ID

    # Gera a imagem da árvore de comportamento
    # gerar_imagem_arvore(tree)  # Comentado para evitar dependência do Graphviz

    # Variáveis para tracking de mudanças e monitoring
    previous_tree_status = None
    previous_tree_display = None
    tick_count = 0
    last_blackboard_info = None
    
    def print_mission_status():
        """Imprime status detalhado da missão"""
        blackboard = py_trees.blackboard.Blackboard()
        
        # Coleta informações do blackboard
        camera_pronta = blackboard.get("camera_pronta") if blackboard.exists("camera_pronta") else False
        calibration_available = blackboard.get("calibration_available") if blackboard.exists("calibration_available") else False
        local_seguro_encontrado = blackboard.get("local_seguro_encontrado") if blackboard.exists("local_seguro_encontrado") else False
        
        print("\n" + "🚁 " + "="*50 + " STATUS MISSÃO " + "="*50)
        print(f"📷 Camera Feed: {'✅ ATIVO' if camera_pronta else '❌ INATIVO'}")
        print(f"🎯 Calibração: {'✅ DISPONÍVEL' if calibration_available else '⏳ PENDENTE'}")
        print(f"📍 Local Seguro: {'✅ ENCONTRADO' if local_seguro_encontrado else '🔍 BUSCANDO'}")
        
        if blackboard.exists("local_seguro_pixel"):
            pixel_pos = blackboard.get("local_seguro_pixel")
            print(f"📌 Posição Pixel: {pixel_pos}")
            
        if blackboard.exists("target_ned_position"):
            ned_pos = blackboard.get("target_ned_position")
            print(f"🧭 Posição NED: N={ned_pos['x']:.2f}m, E={ned_pos['y']:.2f}m")
        
        print("="*112 + "\n")
    
    try:
        while rclpy.ok():
            rclpy.spin_once(commander, timeout_sec=0.1)
            tree.tick()
            tick_count += 1
            
            # Verifica se houve mudança no status da árvore
            current_tree_status = tree.root.status
            current_tree_display = py_trees.display.unicode_tree(tree.root, show_status=True)
            
            # Só imprime se houver mudança no status ou na estrutura da árvore
            status_changed = (current_tree_status != previous_tree_status)
            display_changed = (current_tree_display != previous_tree_display)
            
            if status_changed or display_changed:
                print("\n" + "="*60)
                print(f"🌳 ÁRVORE ATUALIZADA - Status: {current_tree_status} - Tick: {tick_count}")
                print("="*60)
                print(current_tree_display)
                print("="*60)
                
                # Imprime status detalhado da missão
                print_mission_status()
                
                # Atualiza os valores anteriores
                previous_tree_status = current_tree_status
                previous_tree_display = current_tree_display
            
            # Status de progresso a cada 100 ticks (aprox. 5s)
            elif tick_count % 100 == 0:
                blackboard = py_trees.blackboard.Blackboard()
                current_blackboard_info = {
                    'camera_pronta': blackboard.get("camera_pronta") if blackboard.exists("camera_pronta") else False,
                    'calibration_available': blackboard.get("calibration_available") if blackboard.exists("calibration_available") else False,
                    'local_seguro_encontrado': blackboard.get("local_seguro_encontrado") if blackboard.exists("local_seguro_encontrado") else False
                }
                
                # Só imprime se houve mudança no blackboard
                if current_blackboard_info != last_blackboard_info:
                    print(f"⏱️  Tick {tick_count} - Status: Camera={'✅' if current_blackboard_info['camera_pronta'] else '❌'}, "
                          f"Calibração={'✅' if current_blackboard_info['calibration_available'] else '⏳'}, "
                          f"Local={'✅' if current_blackboard_info['local_seguro_encontrado'] else '🔍'}")
                    last_blackboard_info = current_blackboard_info

            if tree.root.status == py_trees.common.Status.SUCCESS:
                commander.get_logger().info("🎉 Missão de precision landing COMPLETA!")
                print_mission_status()
                break
            elif tree.root.status == py_trees.common.Status.FAILURE:
                commander.get_logger().error("❌ Missão FALHOU!")
                print_mission_status()
                break

            time.sleep(0.05)  # Tick rate ~20Hz
    except KeyboardInterrupt:
        commander.get_logger().info("Missão interrompida pelo usuário.")
    finally:
        commander.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()