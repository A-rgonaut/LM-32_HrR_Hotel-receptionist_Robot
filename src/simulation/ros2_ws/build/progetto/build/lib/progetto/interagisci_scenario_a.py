from progetto.InteragisciConOspite import InteragisciConOspite
from utils import Ospite

class InteragisciScenarioA(InteragisciConOspite):
    def rileva_lingua(self, testo):
        dizionario = {
            'IT': {'ciao', 'buongiorno', 'salve', 'buonasera', 'sono', 'chiamo'},
            'EN': {'hi', 'hello', 'hey', 'good morning', 'am', 'name'},
        }
        parole = set(testo.lower().split())
        for lang, parola in dizionario.items():
            if parole.intersection(parola):
                return lang
        return None
    
    def aggiorna_lingua(self, kb):
        query = """MATCH (o:Ospite {id: $id})
                   SET o.lingua = $lingua
                   RETURN o.lingua"""
        parametri = {
            "id":     self.contesto['ospite'].id,
            "lingua": self.contesto['ospite'].lingua
        }
        aggiornato = kb.interrogaGraphDatabase(query, parametri)
        return aggiornato[0]['o.lingua'] == self.contesto['ospite'].lingua

    def esegui(self, testo, kb, llm):
        self.nodo.get_logger().info(f"[ScenarioA] Stato: {self.stato}, Input: {testo}")
        if self.stato == "INIZIO":
            self.contesto['ospite'] = Ospite("id", "Peppe", "Rossi", 30, None)  # tornera da unity. puo servire un UnityManager?
            lingua = self.rileva_lingua(testo)
            if lingua is None:
                self.nodo.parla(super().dialogo_scriptato(tipo="errore_lingua"))
                return
            self.contesto['ospite'].lingua = lingua
            self.aggiorna_lingua(kb)