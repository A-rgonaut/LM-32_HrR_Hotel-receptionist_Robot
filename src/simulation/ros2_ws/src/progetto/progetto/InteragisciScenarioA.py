from progetto.InteragisciConOspite import InteragisciConOspite
from progetto.utils import Ospite

import json
import os
import string

class InteragisciScenarioA(InteragisciConOspite):
    def __init__(self, nodo):
        super().__init__(nodo)
        self.vocab_lingua_it = self.carica_vocabolario("lingua", "ita.txt")
        self.vocab_lingua_en = self.carica_vocabolario("lingua", "eng.txt")

    def rileva_lingua(self, testo):
        testo_clean = testo.lower().translate(str.maketrans('', '', string.punctuation))
        parole = testo_clean.split()
        score_it = 0
        score_en = 0
        for parola in parole:
            if parola in self.vocab_lingua_it:
                score_it += 1
            if parola in self.vocab_lingua_en:
                score_en += 1
        if score_it > score_en:
            return 'IT'
        elif score_en > score_it:
            return 'EN'
        return None

    def aggiorna_lingua(self):
        query = "MATCH (o:Ospite) WHERE id(o) = $id SET o.lingua = $lingua RETURN o.lingua"
        parametri = {
            "id":     self.contesto['ospite'].id,
            "lingua": self.contesto['ospite'].lingua
        }
        aggiornato = self.sincro.interrogaGraphDatabase(query, parametri)
        if aggiornato:
            return aggiornato[0]['o.lingua'] == self.contesto['ospite'].lingua
        return False

    def recupera_dati_prenotazione(self):
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
        risultati = self.sincro.interrogaGraphDatabase(query, {'id': self.contesto['ospite'].id})
        if risultati:
            self.contesto['checkin']   = risultati[0]['checkin']
            self.contesto['checkout']  = risultati[0]['checkout']
            self.contesto['num_notti'] = risultati[0]['notti_reali']
            self.nodo.get_logger().info(f"[ScenarioA] Prenotazione: {risultati[0]}")
            return True
        else:
            self.nodo.get_logger().warning("[ScenarioA] Nessuna prenotazione.")
            return False

    def rileva_interesse(self, testo):
        #nome_interesse_raw = "trekking"  #
        nome_interesse_raw = self.sincro.ask_llm(testo, scenario="A", tipo="estrazione_semantica")
        risultati = self.sincro.trova_classe_da_sinonimo([nome_interesse_raw], nome_radice="Interesse")
        #Se la lista è vuota, assegna None (o un valore di default), altrimenti il primo elemento
        nome_classe_ufficiale = risultati[0] if risultati else None
        if nome_classe_ufficiale:
            self.nodo.get_logger().info(f"Interesse rilevato: '{nome_interesse_raw}' -> Mapped to: '{nome_classe_ufficiale}'")
            self.contesto['nome_interesse_raw'] = nome_interesse_raw
            return nome_classe_ufficiale
        else:
            self.nodo.get_logger().warning(f"Nessuna classe ontologica trovata per: '{nome_interesse_raw}'")
            return []

    def salva_interesse(self):
        query = f"""
        MATCH (o:Ospite)
        WHERE id(o) = $id
        MERGE (i:Interesse:{self.contesto['interesse']} {{nome_interesse: $nome_interesse}})
        MERGE (o)-[:HA_INTERESSE]->(i)
        """
        self.sincro.interrogaGraphDatabase(query, {
            'id': self.contesto['ospite'].id,
            'nome_interesse': self.contesto['nome_interesse_raw']
        })
        self.nodo.get_logger().info(f"[DB] Salvato interesse :{self.contesto['interesse']} ('{self.contesto['nome_interesse_raw']}') per l'ospite.")

    def recupera_dati_per_suggerimento(self):
        query = """MATCH (o:Ospite)-[:EFFETTUA]->(p:Prenotazione)
            WHERE id(o) = $id
            OPTIONAL MATCH (o)-[:SOFFRE_DI]->(pat)
            OPTIONAL MATCH (o)-[:HA_INTERESSE]->(i:Interesse)
            WITH o, p, collect(pat) as patologie, collect(i) as interessi
            OPTIONAL MATCH (e)
            WHERE e.data_ora_evento_locale >= p.data_inizio AND e.data_ora_evento_locale <= p.data_fine
            AND ANY(interest IN interessi WHERE ANY(lbl IN labels(interest)
            WHERE lbl <> 'Interesse' AND ANY(elbl IN labels(e) WHERE elbl CONTAINS lbl)))
            OPTIONAL MATCH (m:PrevisioneMeteo) WHERE m.data_ora_meteo = e.data_ora_evento_locale
            WITH o, p, patologie, interessi, collect(e) as eventi, collect(m) as meteo
            WITH [o, p] + patologie + interessi + eventi + meteo as nodi
            UNWIND nodi as n
            RETURN DISTINCT id(n) as node_id
        """
        risultati = self.sincro.interrogaGraphDatabase(query, {'id': self.contesto['ospite'].id})
        if risultati:
            self.contesto["ids"] = [record['node_id'] for record in risultati if record['node_id'] is not None]
            self.nodo.get_logger().info(f"[ScenarioA] Eventi caricati da DB: {len(self.contesto['ids'])}")
            return True
        return False

    def suggerisci_evento_locale(self):
        self.nodo.parla(self.dialogo_scriptato("attesa_analisi"))
        if not self.recupera_dati_per_suggerimento():
            self.nodo.parla(self.dialogo_scriptato("dati_insufficienti"))
            return
        self.sincro.crea_ontologia_istanze(self.contesto["ids"])
        assiomi = self.sincro.spiegami_tutto(parentClassName="Evento")
        self.nodo.get_logger().info(f"{assiomi}")
        self.nodo.get_logger().info(f"{json.loads(assiomi)}")
        """
        assiomi = json.dumps({
            "Evento_1_NonSo":     {"EventoConsigliabile": "Il reasoner NON deduce assiomi inerenti", "EventoNonConsigliabile": "Il reasoner NON deduce assiomi inerenti"},
            "Evento_2_SoloPro":   {"EventoConsigliabile": "Sconto 50%", "EventoNonConsigliabile": "Il reasoner NON deduce assiomi inerenti"},
            "Evento_3_SoloContro":{"EventoConsigliabile": "Il reasoner NON deduce assiomi inerenti", "EventoNonConsigliabile": "Zona Pericolosa"},
            "Evento_4_Conflitto": {"EventoConsigliabile": "Bel Panorama", "EventoNonConsigliabile": "Strada Rotta"}
        })
        """
        data = json.loads(assiomi)
        for evento_str, proprieta in data.items():
            dettagli_evento = json.loads(evento_str)
            nome_evento_locale = dettagli_evento['nome_evento_locale']
            data_ora_evento_locale = dettagli_evento.get('data_ora_evento_locale', 'N/A')
            proprieta_con_assiomi = [k for k, v in proprieta.items() if v != "Il reasoner NON deduce assiomi inerenti"]
            num_con_assiomi = len(proprieta_con_assiomi)
            self.nodo.parla(self.dialogo_scriptato("titolo_evento", nome_evento_locale=nome_evento_locale, data_ora_evento_locale=data_ora_evento_locale))
            if num_con_assiomi == 0:  # CASO 1: Nessuna informazione
                self.nodo.parla(self.dialogo_scriptato("evento_consiglio_base"))
            elif num_con_assiomi > 1:  # CASO 4: Conflitto
                valore_pro    = proprieta.get("EventoConsigliabile")
                valore_contro = proprieta.get("EventoNonConsigliabile")
                self.nodo.parla(self.dialogo_scriptato("evento_conflitto", valore_pro=valore_pro, valore_contro=valore_contro))
            else:
                chiave_attiva = proprieta_con_assiomi[0]
                valore_attivo = proprieta[chiave_attiva]
                if chiave_attiva == "EventoConsigliabile":  # CASO 2: Solo Incentivo
                    self.nodo.parla(self.dialogo_scriptato("evento_consigliabile_per", valore_attivo=valore_attivo))
                elif chiave_attiva == "EventoNonConsigliabile":  # CASO 3: Solo Divieto
                    self.nodo.parla(self.dialogo_scriptato("evento_sconsigliabile_per", valore_attivo=valore_attivo))
                else:
                    self.nodo.parla(self.dialogo_scriptato("evento_info_generica", chiave_attiva=chiave_attiva, valore_attivo=valore_attivo))
            self.nodo.parla("")
        # risposta = self.sincro.ask_llm(msg_input, scenario="A", tipo="explainability")
        # self.nodo.parla(risposta)
        # salvare il suggerimento come 'idoneo' in neo4j
        # self.salva_suggerimento(evento_scelto['id'])

    def salva_suggerimento(self):  # TODO
        # (idoneo = true lo mette il reasoner)
        query = ""
        parametri = None
        return False

    def recupera_eta(self):
        query = "MATCH (o:Ospite) WHERE id(o) = $id RETURN o.eta AS eta"
        rows = self.sincro.interrogaGraphDatabase(query, {'id': self.contesto['ospite'].id})
        if rows and rows[0].get('eta') is not None:
            return int(rows[0]['eta'])
        self.nodo.get_logger().error(f"Nessun ospite trovato con ID {self.contesto['ospite'].id}!")
        return -1

    def esegui(self, testo):
        self.nodo.get_logger().info(f"[ScenarioA] Stato: {self.stato}, Input: {testo}")
        if self.stato == "INIZIO":
            lingua = self.rileva_lingua(testo)
            if lingua is None:
                self.nodo.parla(self.dialogo_scriptato(tipo="errore_lingua"))
                self.stato = "INIZIO"
            p = self.contesto['ospite']
            self.nodo.get_logger().info(f"{self.contesto['ospite']}")
            eta = self.recupera_eta()
            self.contesto['ospite'] = Ospite(p.id, p.nome, p.cognome, eta, lingua)
            self.nodo.get_logger().info(f"{self.contesto['ospite']}")
            self.nodo.get_logger().info(f"{self.aggiorna_lingua()}")
            if self.recupera_dati_prenotazione():
                self.nodo.parla(self.dialogo_scriptato(tipo="benvenuto"))
                self.stato = "RILEVA_INTERESSE"
            else:
                self.nodo.parla(self.dialogo_scriptato(tipo="nessuna_prenotazione"))
                self.stato = "FINE"
        elif self.stato == "RILEVA_INTERESSE":
            interesse = self.rileva_interesse(testo)
            if interesse:
                self.nodo.get_logger().info(interesse)
                self.contesto['interesse'] = interesse
                self.nodo.parla(self.dialogo_scriptato(tipo="conferma_interesse"))
                self.stato = "CONFERMA"
            else:
                self.stato = "RILEVA_INTERESSE"
        elif self.stato == "CONFERMA":
            risposta = self.rileva_conferma(testo)
            if risposta is None:
                self.stato = "CONFERMA"
            elif risposta is False:
                self.stato = "RILEVA_INTERESSE"
            else:
                self.salva_interesse()
                self.stato = "SUGGERISCI_EVENTO_LOCALE"
                self.esegui("")
        elif self.stato == "SUGGERISCI_EVENTO_LOCALE":
            self.suggerisci_evento_locale()
            self.stato = "SALUTO_FINALE"
        elif self.stato == "SALUTO_FINALE":
            # Qualsiasi cosa l'utente abbia detto (testo), noi chiudiamo.
            # (Analizzare 'testo' per vedere se e' un insulto o un grazie,
            # per ora assumiamo sia un ringraziamento...).
            self.nodo.parla(self.dialogo_scriptato(tipo="arrivederci"))
            self.stato = "FINE"
        elif self.stato == "FINE":
            pass
        else:
            self.nodo.get_logger().error(f"[ScenarioA] Stato sconosciuto o non gestito: '{self.stato}'")
