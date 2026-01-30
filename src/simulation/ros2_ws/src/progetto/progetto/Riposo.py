from math import pi

class Riposo:
    def __init__(self, nodo):
        self.nodo = nodo

    def esegui(self, testo=None):
        # In questa fase il robot potrebbe aggiungere
        # previsioni meteo, eventi locali, non so...
        self.nodo.parla("Sono a riposo. Seleziona uno scenario su Unity.")

    def reset(self, ospite=None):
        self.nodo.get_logger().info("Entrato in modalità Riposo.")
        # Se la destinazione è già quella di home, non resettare il flag
        nuovo_target = (10, 11, pi)
        if self.nodo.destinazione_target != nuovo_target:
            self.nodo.destinazione_target = nuovo_target
            self.nodo.raggiunta_destinazione = False
        self.nodo.comportamento_precedente = "Riposo"
        self.nodo.comportamenti["Naviga"].reset()