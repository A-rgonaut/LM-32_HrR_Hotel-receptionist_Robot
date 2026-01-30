"""
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
"""
class RicaricaBatteria:
    def __init__(self, nodo):
        self.nodo = nodo

    def esegui(self, testo=None):
        # Una volta arrivati alla stazione, il robot monitora lo stato 
        # che arriva dai sensori di Unity tramite l'Arbitraggio.
        if self.nodo.raggiunta_destinazione:
            if self.nodo.in_carica:
                self.nodo.get_logger().info(f"In carica: {self.nodo.livello_batteria}%", throttle_duration_sec=10.0)
            else:
                self.nodo.parla("Sono in posizione. In attesa di collegamento elettrico.")
        else:
            self.nodo.get_logger().info("In viaggio verso la stazione di ricarica...")

    def reset(self, ospite=None):
        self.nodo.get_logger().info("Comportamento RicaricaBatteria attivato.")
        
        # 1. Definiamo le coordinate della stazione di ricarica (es. la base)
        target_stazione = (10, 11)
        
        # 2. Impostiamo il target solo se necessario per attivare il modulo Naviga
        if self.nodo.destinazione_target != target_stazione:
            self.nodo.destinazione_target = target_stazione
            self.nodo.raggiunta_destinazione = False
        
        # 3. Aggiorniamo i riferimenti per la navigazione
        self.nodo.comportamento_precedente = "RicaricaBatteria"
        self.nodo.comportamenti["Naviga"].reset()