from progetto.InteragisciConOspite import InteragisciConOspite
from progetto.utils import Ospite

import json
from math import pi

class InteragisciScenarioB(InteragisciConOspite):
    def __init__(self, nodo, specialista):
        super().__init__(nodo)
        self.specialista = specialista

        self.domande_diagnostica = {
            "setpoint_temperatura": "A che temperatura è impostata la stufa ?",
            "tempo_accensione": "Da quanti minuti è accesa la stufa?",
            "dom1": "temperatura_impostata",
            "dom2": "tempo_acceso",
        }
    
    def reset(self, ospite=None):
        super().reset(ospite)
        #self.nodo.destinazione_target = (-10, 11)
        self.nodo.destinazione_target = (6, -7.8,0)   # Stanza 3
        #self.nodo.destinazione_target = (-10, 7)  # Intra u divanu
        #self.nodo.destinazione_target = (10,10,pi)  # stanza 1
        #self.nodo.destinazione_target = (11, 11)  # narrè
        self.nodo.raggiunta_destinazione = False
        self.nodo.comportamento_precedente = "InteragisciScenarioB"
        self.stato = "ASPETTA_ROBOT"
        self.nodo.comportamenti["Naviga"].reset()
    
    def esegui(self, testo):
        self.nodo.get_logger().info(f"[ScenarioB] Stato: {self.stato}, Input: {testo}")
        if self.stato == "ASPETTA_ROBOT":
            if self.nodo.raggiunta_destinazione:
                self.nodo.parla(f"Buonasera, sono qui per assisterLa e nel risolvere la situazione nel modo più rapido possibile. Mi dica l'oggetto in questione danneggiato")
                p = self.contesto['ospite']
                self.contesto['ospite'] = Ospite(p.id, p.nome, p.cognome, "2000", "IT") 
                self.nodo.destinazione_target = None
                self.stato = "RILEVA_GUASTO"    
                #self.stato = "FINE"  
            else:
                self.nodo.parla("Arrivo subito, sto arrivando da te!")
        elif self.stato == "RILEVA_GUASTO":
            oggetto_guasto = self.rileva_guasto(testo) 
            if oggetto_guasto:
                #self.nodo.parla(oggetto_guasto) 
                self.contesto['oggetto_guasto'] = oggetto_guasto
                self.nodo.parla(f"Se ho capito bene, l'oggetto in questione è {oggetto_guasto}, giusto?")
                #self.nodo.parla(self.dialogo(tipo="conferma_oggetto_guasto")) # esempio self.nodo.get_logger().info(f"ES. Mi confermi che il problema è il condizionatore?")
                self.stato = "CONFERMA_GUASTO"
            else :
                self.nodo.parla("Non ho capito bene l'oggetto in questione, puoi ripetere?") 
                self.stato = "RILEVA_GUASTO"
        elif self.stato == "CONFERMA_GUASTO":
            if self.rileva_conferma(testo):
                self.nodo.parla(self.domande_diagnostica["setpoint_temperatura"]) 
                self.stato = "SUGGERISCI_SOLUZIONE_1" 
            else:
                self.nodo.parla("Allora non ho capito bene l'oggetto in questione, puoi ripetere?")
                # dire che non ha capito bene il robot o probabilmente non ha detto qualcosa che concerne la stanza
                self.stato = "RILEVA_GUASTO"
        elif self.stato == "SUGGERISCI_SOLUZIONE_1":
            dom1 = self.sincro.ask_llm(testo, scenario="B", tipo="dom")
            if dom1:
                self.contesto[self.domande_diagnostica["dom1"]]=dom1
                self.nodo.parla(self.domande_diagnostica["tempo_accensione"]) #TODO alla gabriele
                self.stato = "SUGGERISCI_SOLUZIONE_2"
            else:
                self.nodo.parla("Non ho ben capito il primo valore, puoi ripetere?")
                self.stato = "SUGGERISCI_SOLUZIONE_1"
        elif self.stato == "SUGGERISCI_SOLUZIONE_2":
            dom2 = self.sincro.ask_llm(testo, scenario="B", tipo="dom")
            if dom2:
                self.contesto[self.domande_diagnostica["dom2"]]=dom2
                self.aggiorna_stato_oggetto()
                spiegazione = self.suggerisci_specialista()
                if spiegazione:
                    self.nodo.get_logger().info(f"{spiegazione}")
                    spiegazione = self.sincro.ask_llm(spiegazione, scenario="B", tipo="explainability")
                    self.nodo.parla(f"{spiegazione}  ")
                    self.stato = "CONTATTA_SPECIALISTA"
                    self.esegui("")
                else:
                    self.nodo.parla("Dalle tue risposte non identifico il problema, vuoi che ti chiami comunque lo specialista? ")
                    self.stato = "SPECIALISTA_SI-NO"
            else:
                self.nodo.parla("Non ho ben capito il secondo valore, puoi ripetere?")
                self.stato = "SUGGERISCI_SOLUZIONE_2"                
        elif self.stato == "SPECIALISTA_SI-NO":
            risposta = self.rileva_conferma(testo)
            if risposta in [None, False]:
                self.stato = "FINE"
            else:
                #self.salva_guasto() # <--- Inserito qui
                self.stato = "CONTATTA_SPECIALISTA"
                self.esegui("") # queryTODO ,  salvo l'oggetto che è guasto e la chiamata allo specialista
        elif self.stato == "CONTATTA_SPECIALISTA":
            self.specialista.chiama(self.nodo,self.contesto['tipo_oggetto_guasto'], self.contesto['ospite'], self.contesto['spiegazioner']) 
            self.stato = "FINE"
        elif self.stato == "FINE":
            pass
        else:
            self.nodo.get_logger().error(f"[ScenarioB] Stato sconosciuto o non gestito: '{self.stato}'")
       
    def termostato(self):
        return 23  # random float tra 20.0 e 24.0

    def salva_guasto(self):  # TODO
        """
        Aggiorna esclusivamente il timestamp del guasto sull'oggetto nel database.
        """
        label_oggetto = self.contesto['tipo_oggetto_guasto']
        id_ospite = self.contesto['ospite'].id

        # Query rigorosa: identifica l'oggetto tramite la relazione con l'ospite e la stanza
        query = f"""
        MATCH (o:Ospite)-[:EFFETTUA]->(:Prenotazione)-[:ASSOCIATA_A]->(:Stanza)<-[:CONTENUTO_IN]-(obj:{label_oggetto})
        WHERE id(o) = $id_ospite
        SET obj.timestamp_guasto = datetime()
        RETURN obj
        """
    
        parametri = {"id_ospite": id_ospite}
        self.sincro.interrogaGraphDatabase(query, parametri)

        return False

    def rileva_guasto(self, testo):
        nome_interesse_raw = self.sincro.ask_llm(testo, scenario="B", tipo="estrazione_semantica")
        risultati = self.sincro.trova_classe_da_sinonimo([nome_interesse_raw], nome_radice="OggettoGuastabile")
        #Se la lista è vuota, assegna None (o un valore di default), altrimenti il primo elemento
        nome_classe_ufficiale = risultati[0] if risultati else None
        if nome_classe_ufficiale:
            self.nodo.get_logger().info(f"Interesse rilevato: '{nome_interesse_raw}' -> Mapped to: '{nome_classe_ufficiale}'")
            self.contesto['nome_interesse_raw'] = nome_interesse_raw
            return nome_classe_ufficiale
        else:
            self.nodo.get_logger().warning(f"Nessuna classe ontologica trovata per: '{nome_interesse_raw}'")
            return []
        
    def aggiorna_stato_oggetto(self):
        # Recupero la label dell'oggetto (es. "Stufa")
        label_oggetto = self.contesto['oggetto_guasto'] 
        # Recupero i valori salvati nel dizionario 'contesto' usando le chiavi definite in __init__
        # dom1 -> temperatura_impostata
        valore_dom1 = self.contesto.get(self.domande_diagnostica["dom1"]) 
        # dom2 -> temperatura_stanza (Nota: se la domanda era sul tempo, verificare la coerenza della chiave nel dizionario)
        valore_dom2 = self.contesto.get(self.domande_diagnostica["dom2"])
        if not label_oggetto:
            self.nodo.get_logger().error("Nessun oggetto_guasto trovato nel contesto.")
            return False
        # Costruzione della query. 
        # Utilizzo f-string per la Label (come in Scenario C) poiché le Label non sono parametrizzabili in Cypher.
        # Aggiorno le proprietà corrispondenti ai nodi definiti nel DB (Stufa).
        query = f"""
        MATCH (o:Ospite)-[:EFFETTUA]->(:Prenotazione)-[:ASSOCIATA_A]->(:Stanza)<-[:CONTENUTO_IN]-(n:{label_oggetto})
        WHERE id(o) = $id_ospite
        SET n.temperatura_impostata =  toInteger($val1),
            n.tempo_acceso =  toInteger($val2) 
        RETURN n
        """
        parametri = {
            "id_ospite": self.contesto['ospite'].id,
            "val1": valore_dom1,
            "val2": valore_dom2
        }
        risultati = self.sincro.interrogaGraphDatabase(query, parametri)
        if risultati:
            # Prendi il primo record (ce ne dovrebbe essere uno solo per quell'oggetto in quella stanza)
            nodo = risultati[0]['n']
            # Leggi la proprietà dal nodo Neo4j
            tipo_guasto = nodo.get('tipo_oggetto_guastabile')
            # Salvala nel dizionario contesto con la chiave che usi poi in CONTATTA_SPECIALISTA
            if tipo_guasto:
                self.contesto['tipo_oggetto_guasto'] = tipo_guasto
                self.nodo.get_logger().info(f"Tipo guasto recuperato: {tipo_guasto}")
        self.nodo.get_logger().info(f"Aggiornato oggetto {label_oggetto}: temp_imp={valore_dom1}, val2={valore_dom2}")
        return True
    
    def recupera_dati_per_suggerimento(self):
        query = """
                MATCH (o) WHERE id(o) = $id
                OPTIONAL MATCH (o)-[:EFFETTUA]->(:Prenotazione)-[:ASSOCIATA_A]->(stanza:Stanza)
                OPTIONAL MATCH (obj)-[:CONTENUTO_IN]->(stanza)
                WITH o,
                    stanza,
                    collect(DISTINCT obj) AS oggetti
                WITH [o] + [stanza] + oggetti AS nodi_totali
                UNWIND nodi_totali AS n
                WITH n WHERE n IS NOT NULL
                RETURN DISTINCT id(n) AS node_id
             """
        # Recupero l'ID dell'ospite dal contesto (assicurati che sia l'oggetto Ospite corretto)
        risultati = self.sincro.interrogaGraphDatabase(query, {'id': self.contesto['ospite'].id})
        if risultati: 
            self.contesto["ids"] = [record['node_id'] for record in risultati if record['node_id'] is not None]
            self.nodo.get_logger().info(f"[ScenarioC] Eventi caricati da DB: {len(self.contesto['ids'])}")
            return True
        return False

    def suggerisci_specialista(self):
        self.nodo.parla("Un attimo, analizzo le risposte e chiedo al sistema esperto...")
        if not self.recupera_dati_per_suggerimento():
            self.nodo.parla(self.dialogo("dati_insufficienti"))
            return
        self.sincro.crea_ontologia_istanze(self.contesto["ids"])
        assiomi = self.sincro.spiegami_tutto(parentClassName="Guasto")
        spiegazione = ""
        data = json.loads(assiomi)
        #self.nodo.parla(data)
        if data:
            for oggetto, proprieta in data.items():
                valori = list(proprieta.values())
                if valori:  # Controllo se la lista non è vuota
                    testo_spiegazione = valori[0]  # Prendo la stringa all'indice 0
                    if testo_spiegazione != "Il reasoner NON deduce assiomi inerenti":
                        spiegazione=testo_spiegazione
        self.contesto['spiegazioner']=spiegazione
        return spiegazione