import pickle

class Calibracao:

    def __init__(self, caminho):
        try:
            with open(caminho, "rb") as arquivo:
                dados_calibracao = pickle.load(arquivo)

        except FileNotFoundError:
            print(f"ERRO: ARQUIVO NAO ENCONTRADO NO CAMINHO {caminho}")
            return None

        except Exception as e:
            print(f"ERRO AO CARREGAR O ARQUIVO: '{caminho}' --> {e}")
            return None
        
        if isinstance(dados_calibracao, dict):
            self.matriz_camera = dados_calibracao.get("camera_matrix")
            self.coeficiente_distorcao = dados_calibracao.get("distortion_coefficients")
            #self.vetores_rotacao = dados_calibracao.get("rotation_vectors")
            #self.vetores_trasnlacao = dados_calibracao.get("translation_vectors")
            #self.erro_de_projecao = dados_calibracao.get("reprojection_error")

    
    def Get_dados_calibracao(self):
        return self.matriz_camera, self.coeficiente_distorcao








