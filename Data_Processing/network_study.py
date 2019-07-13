import networkx as nx
import matplotlib.pyplot as plt

G = nx.read_multiline_adjlist('graph.multiline_adjlist')
nx.draw_networkx(G,arrows=True,node_size = 180,node_shape = "o")
# nx.draw_spectral(G,arrows=True,node_size = 200,node_shape = "8",node_color = "red")

# nx.write_gml(G,"test.gml")
plt.show()