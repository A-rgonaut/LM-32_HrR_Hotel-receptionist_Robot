from progetto.InteragisciConOspite import InteragisciConOspite
from progetto.utils import Ospite

import string

class InteragisciScenarioA(InteragisciConOspite):
    def rileva_lingua(self, testo):
        testo_clean = testo.lower().translate(str.maketrans('', '', string.punctuation))
        parole = testo_clean.split()
        score_it = 0
        score_en = 0
        vocab = {
            'IT': {
                'ciao', 'buongiorno', 'buonasera', 'salve', 'arrivederci',
                'sono', 'mi', 'chiamo', 'io', 'vorrei', 'per', 'favore',
                'grazie', 'prego', 'il', 'lo', 'la', 'che', 'cosa',
                'tutto', 'bene', 'scusa', 'hotel', 'prenotazione', 'camera'
            },
            'EN': {
                'hello', 'hi', 'hey', 'good', 'morning', 'evening', 'bye',
                'am', 'my', 'name', 'is', 'i', 'would', 'like', 'please',
                'thanks', 'thank', 'you', 'the', 'what', 'how',
                'everything', 'ok', 'sorry', 'reservation', 'room'
            }
        }
        for parola in parole:
            if parola in vocab['IT']:
                score_it += 1
            if parola in vocab['EN']:
                score_en += 1
        if score_it > score_en:
            return 'IT'
        elif score_en > score_it:
            return 'EN'
        return None

    def aggiorna_lingua(self, sincro):
        query = "MATCH (o:Ospite) WHERE id(o) = $id SET o.lingua = $lingua RETURN o.lingua"
        parametri = {
            "id":     self.contesto['ospite'].id,
            "lingua": self.contesto['ospite'].lingua
        }
        aggiornato = sincro.interrogaGraphDatabase(query, parametri)
        return aggiornato[0]['o.lingua'] == self.contesto['ospite'].lingua

    def recupera_dati_prenotazione(self, sincro):
        query = """
            MATCH (o:Ospite)-[:EFFETTUA]->(p:Prenotazione)
            WHERE id(o) = $id
            WITH p
            ORDER BY p.data_inizio DESC LIMIT 1
            RETURN
                toString(p.data_inizio) AS checkin,
                toString(p.data_fine) AS checkout,
                duration.inDays(date(p.data_inizio), date(p.data_fine)).days AS notti_reali
        """
        risultati = sincro.interrogaGraphDatabase(query, {'id': self.contesto['ospite'].id})
        if risultati:
            self.contesto['checkin']   = risultati[0]['checkin']
            self.contesto['checkout']  = risultati[0]['checkout']
            self.contesto['num_notti'] = risultati[0]['notti_reali']
            self.nodo.get_logger().info(f"[ScenarioA] Prenotazione: {risultati[0]}")
            return True
        else:
            self.nodo.get_logger().warning("[ScenarioA] Nessuna prenotazione.")
            return False

    def rileva_interesse(self, testo, sincro):
        nome_interesse_raw = sincro.ask_llm(testo, scenario="A", tipo="estrazione_semantica")
        nome_classe_ufficiale = sincro.trova_classe_da_sinonimo(nome_interesse_raw, nome_radice="Interesse")
        if nome_classe_ufficiale:
            self.nodo.get_logger().info(f"Interesse rilevato: '{nome_interesse_raw}' -> Mapped to: '{nome_classe_ufficiale}'")
            return nome_classe_ufficiale
        else:
            self.nodo.get_logger().warning(f"Nessuna classe ontologica trovata per: '{nome_interesse_raw}'")
            return None

    def salva_interesse(self, sincro):
        query = """
        MATCH (o:Ospite)
        WHERE id(o) = $id
        MERGE (i:Interesse {nome: $nome_interesse})
        MERGE (o)-[:HA_INTERESSE]->(i)
        """
        sincro.interrogaGraphDatabase(query, {
            'id': self.contesto['ospite'].id,
            'nome_interesse': self.contesto['interesse']
        })
        self.nodo.get_logger().info(f"[DB] Salvato interesse '{self.contesto['interesse']}' per l'ospite.")

    def recupera_dati_per_suggerimento(self, sincro):
        query = """
            MATCH (o:Ospite)-[:EFFETTUA]->(p:Prenotazione)
            WHERE id(o) = $id
            OPTIONAL MATCH (o)-[:SOFFRE_DI]->(pat)
            OPTIONAL MATCH (o)-[:HA_INTERESSE]->(int)
            MATCH (o)-[:HA_INTERESSE]->(i:Interesse)
            MATCH (e)
            WHERE e.data_ora >= p.data_inizio AND e.data_ora <= p.data_fine
            AND any(lbl IN labels(e) WHERE toLower(lbl) CONTAINS toLower(i.nome))
            OPTIONAL MATCH (m:PrevisioneMeteo) WHERE m.data_ora = e.data_ora
            RETURN
                o.nome AS nome,
                collect(DISTINCT labels(pat)) AS patologie_labels,
                collect(DISTINCT labels(int)) AS interessi_labels,
                collect(DISTINCT {
                    tipo: labels(e),
                    nome: e.nome,
                    data: toString(e.data_ora),
                    meteo: m.condizione
                }) AS eventi_disponibili
        """
        risultati = sincro.interrogaGraphDatabase(query, {'id': self.contesto['ospite'].id})
        if risultati:
            record = risultati[0]
            self.contesto['dati_neo4j'] = {
                'nome_ospite': record['nome'],
                'patologie': [lbl[0] for lbl in record['patologie_labels'] if lbl],
                'interessi_profilo': [lbl[0] for lbl in record['interessi_labels'] if lbl],
                'eventi': []
            }
            for ev in record['eventi_disponibili']:
                tipo_evento = next((l for l in ev['tipo'] if "Evento" in l), "EventoGenerico")
                self.contesto['dati_neo4j']['eventi'].append({
                    'label': tipo_evento,
                    'nome': ev['nome'],
                    'data': ev['data'],
                    'meteo': ev.get('meteo')
                })
            self.nodo.get_logger().info(f"[ScenarioA] Eventi caricati da DB: {len(self.contesto['dati_neo4j']['eventi'])}")
            return True
        return False

    def suggerisci_evento_locale(self, sincro):
        self.nodo.parla("Un attimo, analizzo tutti gli eventi disponibili...")
        if not self.recupera_dati_per_suggerimento(sincro):
            self.nodo.parla("Errore nel recupero dati.")
            return
        dati_neo = self.contesto['dati_neo4j']
        ospite_info = {
            "nome": dati_neo['nome_ospite'],
            "eta": self.contesto['ospite'].eta,
            "patologie": dati_neo['patologie'],
            "interessi": dati_neo['interessi_profilo']
        }
        eventi_list = dati_neo['eventi']
        report_ragionamento = sincro.ragiona_su_eventi(ospite_info, eventi_list)
        if report_ragionamento:
            msg_input = f"Analisi per l'ospite {self.contesto['ospite'].nome}:\n"
            for item in report_ragionamento:
                msg_input += f"\nEVENTO: {item['evento']}\nESITO: {item['esito']}\nMOTIVAZIONE LOGICA: {item['dettagli']}\n----------------"
            self.nodo.get_logger().info(msg_input)
            risposta = sincro.ask_llm(msg_input, scenario="A", tipo="explainability")
            self.nodo.parla(risposta)
        else:
            self.nodo.parla("Ho analizzato gli eventi ma non ho trovato indicazioni specifiche (né positive né negative).")

    def salva_suggerimento(self, sincro):  # TODO
        # (idoneo = true lo mette il reasoner)
        query = ""
        parametri = None
        return False

    def recupera_eta(self, sincro):
        query = "MATCH (o:Ospite) WHERE id(o) = $id RETURN o.eta AS eta"
        rows = sincro.interrogaGraphDatabase(query, {'id': self.contesto['ospite'].id})
        if rows and rows[0].get('eta') is not None:
            return int(rows[0]['eta'])
        self.nodo.get_logger().error(f"Nessun ospite trovato con ID {self.contesto['ospite'].id}!")
        return -1

    def esegui(self, testo, sincro):
        self.nodo.get_logger().info(f"[ScenarioA] Stato: {self.stato}, Input: {testo}")
        if self.stato == "INIZIO":
            lingua = self.rileva_lingua(testo)
            if lingua is None:
                self.nodo.parla(self.dialogo_scriptato(tipo="errore_lingua"))
                return
            p = self.contesto['ospite']
            self.nodo.get_logger().info(f"{self.contesto['ospite']}")
            eta = self.recupera_eta(sincro)
            self.contesto['ospite'] = Ospite(p.id, p.nome, p.cognome, eta, lingua)
            self.nodo.get_logger().info(f"{self.contesto['ospite']}")
            self.nodo.get_logger().info(f"{self.aggiorna_lingua(sincro)}")
            if self.recupera_dati_prenotazione(sincro):
                self.nodo.parla(self.dialogo_scriptato(tipo="benvenuto"))
                self.stato = "RILEVA_INTERESSE"
            else:
                self.nodo.parla("Non trovo nessuna prenotazione attiva.")
                self.stato = "FINE"
        elif self.stato == "RILEVA_INTERESSE":
            interesse = self.rileva_interesse(testo, sincro)
            if interesse:
                self.nodo.get_logger().info(interesse)
                self.contesto['interesse'] = interesse
                self.nodo.parla(self.dialogo_scriptato(tipo="conferma_interesse"))
                self.stato = "CONFERMA"
        elif self.stato == "CONFERMA":
            if self.rileva_conferma(testo):
                if self.contesto['interesse']:
                    self.salva_interesse(sincro)
                    self.stato = "SUGGERISCI_EVENTO_LOCALE"
                    self.esegui("", sincro)  # Presidente?
            else:
                self.stato = "RILEVA_INTERESSE"
                self.esegui(testo, sincro)
        elif self.stato == "SUGGERISCI_EVENTO_LOCALE":
            self.suggerisci_evento_locale(sincro)
            self.stato = "FINE"
        elif self.stato == "FINE":
            self.nodo.parla(self.dialogo_scriptato(tipo="arrivederci"))
        else:
            self.nodo.get_logger().info("self.stato NON VALIDO.")
