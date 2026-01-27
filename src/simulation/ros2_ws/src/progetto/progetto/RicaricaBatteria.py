class RicaricaBatteria:
    def __init__(self, nodo):
        self.nodo = nodo
        self.timer_simulazione = None

    def esegui(self, testo=None):
        pass

    def reset(self, ospite=None):
        self.nodo.get_logger().info("Entrato in RicaricaBatteria. Simulazione ricarica rapida (5s)...")
        self.nodo.parla("Ricarica rapida avviata. Torno al 100% tra 5 secondi.")
        if self.timer_simulazione is not None:
            self.timer_simulazione.destroy()
        self.timer_simulazione = self.nodo.create_timer(5.0, self.completa_ricarica)

    def completa_ricarica(self):
        self.nodo.livello_batteria = 100.0
        self.nodo.in_carica = False
        self.nodo.get_logger().info("Batteria ricaricata al 100%.")
        self.nodo.parla("Batteria carica.")
        if self.timer_simulazione:
            self.timer_simulazione.destroy()
            self.timer_simulazione = None
