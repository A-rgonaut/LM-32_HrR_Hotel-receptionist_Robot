class RicaricaBatteria:
    def __init__(self, nodo):
        self.nodo = nodo

    def esegui(self, testo=None, kb=None, llm=None):
        self.nodo.parla("Sto ricaricando la batteria. Imposta il livello di carica su Unity.")

    def reset(self, ospite=None):
        self.nodo.get_logger().info("Entrato in modalità RicaricaBatteria.")
