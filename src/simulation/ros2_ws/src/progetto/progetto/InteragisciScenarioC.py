from progetto.InteragisciConOspite import InteragisciConOspite

class InteragisciScenarioC(InteragisciConOspite):
    def esegui(self, testo, kb, llm):
        self.nodo.get_logger().info(f"[ScenarioC] Stato: {self.stato}, Input: {testo}")
        if self.stato == "INIZIO":
            self.stato = "FINE"
        elif self.stato == "FINE":
            self.nodo.get_logger().info("TODO")
        else:
            self.nodo.get_logger().info("self.stato NON VALIDO.")
