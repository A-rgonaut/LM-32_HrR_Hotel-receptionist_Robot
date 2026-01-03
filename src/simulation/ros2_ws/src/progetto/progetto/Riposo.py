class Riposo:
    def __init__(self, nodo):
        self.nodo = nodo

    def esegui(self, testo=None, kb=None, llm=None):
        # In questa fase il robot potrebbe aggiungere
        # previsioni meteo, eventi locali, non so...
        self.nodo.parla("Sono a riposo. Seleziona uno scenario su Unity.")

    def reset(self, ospite=None):
        self.nodo.get_logger().info("Entrato in modalità Riposo.")
