import csv
import networkx as nx
import matplotlib.pyplot as plt
from geopy.distance import geodesic

# Classe para implementar Kruskal
class Graph:
    def find(self, parent, node):
        if parent[node] == node:
            return node
        return self.find(parent, parent[node])

    def union(self, parent, rank, x, y):
        root_x = self.find(parent, x)
        root_y = self.find(parent, y)

        if rank[root_x] < rank[root_y]:
            parent[root_x] = root_y
        elif rank[root_x] > rank[root_y]:
            parent[root_y] = root_x
        else:
            parent[root_y] = root_x
            rank[root_x] += 1

    def aplicar_kruskal(self, grafo):
        arestas = []
        # Extrair as arestas do grafo
        for u in grafo:
            for v, peso in grafo[u].items():
                if v != 'pos':  # Ignorar a entrada de coordenadas
                    if (v, u, peso) not in arestas:  # Evitar duplicação
                        arestas.append((u, v, peso))

        # Ordenar as arestas pelo peso
        arestas.sort(key=lambda x: x[2])

        # Inicializar estruturas para Kruskal
        parent = {node: node for node in grafo}
        rank = {node: 0 for node in grafo}
        mst = {node: {'pos': grafo[node]['pos']} for node in grafo}  # Estrutura da MST

        for u, v, peso in arestas:
            root_u = self.find(parent, u)
            root_v = self.find(parent, v)

            if root_u != root_v:  # Não forma ciclo
                # Adicionar aresta na MST
                mst[u][v] = peso
                mst[v][u] = peso
                self.union(parent, rank, root_u, root_v)

        return mst


    # Função para extrair coordenadas do formato WKT
    def parse_wkt(self, wkt):
        wkt = wkt.replace("POINT (", "").replace(")", "")
        lon, lat = map(float, wkt.split())
        return lat, lon

    # Ler o arquivo CSV
    def read_wifi_data(self, file_path):
        wifi_points = []
        with open(file_path, 'r', encoding='utf-8') as file:
            reader = csv.DictReader(file)
            for row in reader:
                if row['WKT']:
                    lat_lon = self.parse_wkt(row['WKT'])
                    name = row['nome']
                    wifi_points.append({'nome': name, 'coordenadas': lat_lon})
        return wifi_points

    # Criar o grafo ponderado incluindo coordenadas no dicionário
    def create_wifi_graph(self, wifi_points):
        G = {}
        for i, point1 in enumerate(wifi_points):
            name1, coord1 = point1['nome'], point1['coordenadas']
            G[name1] = {'pos': coord1}  # Adicionar posição ao nó
            for j, point2 in enumerate(wifi_points):
                if i < j:
                    name2, coord2 = point2['nome'], point2['coordenadas']
                    distance = geodesic(coord1, coord2).meters
                    G[name1][name2] = distance
                    G.setdefault(name2, {'pos': coord2})[name1] = distance
        return G

    def desenhar_grafo(self, grafo, titulo="Grafo de Cobertura Wi-Fi"):
        # Preparar dados para o grafo
        pos = {}  # Coordenadas dos nós
        edges = []  # Arestas e pesos
    
        # Preencher os dados do grafo
        for u, vizinhos in grafo.items():
            if 'pos' in grafo[u]:
                pos[u] = grafo[u]['pos']
            for v, peso in vizinhos.items():
                if isinstance(peso, (int, float)):  # Verificar se é um peso válido
                    edges.append((u, v, peso))
    
        # Configurar o tamanho da figura
        plt.figure(figsize=(12, 10))
    
        # Desenhar os nós
        node_size = max(100, 2000 // len(pos))  # Ajustar tamanho com base no número de nós
        for node, (x, y) in pos.items():
            plt.scatter(x, y, c='dodgerblue', s=node_size, alpha=0.8, edgecolors='black')
            plt.text(x, y + 0.0001, node, fontsize=9, fontweight="bold", ha="center", va="center")
    
        # Desenhar as arestas
        for u, v, peso in edges:
            x1, y1 = pos[u]
            x2, y2 = pos[v]
            plt.plot([x1, x2], [y1, y2], color='gray', lw=0.5, alpha=0.6, linestyle='-')
            # Adicionar rótulo com peso da aresta
            mid_x, mid_y = (x1 + x2) / 2, (y1 + y2) / 2
            plt.text(mid_x, mid_y, f"{peso:.1f}m", fontsize=7, color="darkred", alpha=0.7, ha="center", va="center")
    
        # Melhorar o layout da visualização
        plt.title(titulo, fontsize=14)
        plt.xlabel("Longitude", fontsize=12)
        plt.ylabel("Latitude", fontsize=12)
        plt.grid(True, linestyle='--', alpha=0.3)  # Adicionar grade para facilitar a localização
        plt.tight_layout()  # Ajustar espaçamento
        plt.show()

# Função principal
def main():
    graph = Graph()
    file_path = 'wifi_points.csv'  # Substitua pelo caminho do seu arquivo CSV
    wifi_points = graph.read_wifi_data(file_path)
    
    # Criar o grafo a partir dos dados
    wifi_graph = graph.create_wifi_graph(wifi_points)
    
    # Desenhar o grafo completo
    graph.desenhar_grafo(wifi_graph, titulo="Grafo Completo de Cobertura Wi-Fi")

    # Aplicar o algoritmo de Kruskal manual
    mst = graph.aplicar_kruskal(wifi_graph)
    
    # Desenhar a árvore geradora mínima
    graph.desenhar_grafo(mst, titulo="Árvore Geradora Mínima (Algoritmo de Kruskal)")

if __name__ == '__main__':
    main()


