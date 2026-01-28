from progetto.InteragisciConOspite import InteragisciConOspite
from progetto.utils import Ospite

import json

class InteragisciScenarioB(InteragisciConOspite):
    def __init__(self, nodo, specialista):
        super().__init__(nodo)
        self.specialista = specialista
        
        self.domande_diagnostica = {
            "setpoint_temperatura": "A che temperatura è impostata la stufa ?",
            "tempo_accensione": "Da quanto tempo è accesa la stufa?",
            "dom1": "temperatura_impostata",
            "dom2": "tempo_acceso",

        }
    
    def reset(self, ospite=None):
        super().reset(ospite)
        #self.nodo.destinazione_target = (-10, 11)
        #self.nodo.destinazione_target = (7, -7.8)  # Stanza 3
        #self.nodo.destinazione_target = (-10, 7)  # Intra u divanu
        self.nodo.destinazione_target = (10,10)  # stanza 1
        #self.nodo.destinazione_target = (11, 11)  # narrè
        self.nodo.raggiunta_destinazione = False
        self.nodo.comportamento_precedente = "InteragisciScenarioB"
        self.stato = "ASPETTA_ROBOT"
        self.nodo.comportamenti["Naviga"].reset()
    
    
   
    def esegui(self, testo):
        self.nodo.get_logger().info(f"[ScenarioB] Stato: {self.stato}, Input: {testo}")
        if self.stato == "ASPETTA_ROBOT":
            if self.nodo.raggiunta_destinazione:
                self.nodo.destinazione_target = None
            else:
                self.nodo.parla("Arrivo subito, sto arrivando da te!")
            p = self.contesto['ospite']
            self.contesto['ospite'] = Ospite(p.id, p.nome, p.cognome, "2000", "IT") 
            self.nodo.parla(f"Buonasera, sono qui per assisterLa e nel risolvere la situazione nel modo più rapido possibile. Mi dica l'oggetto  in questione danneggiato")
            self.stato = "RILEVA_GUASTO"    
        elif self.stato == "RILEVA_GUASTO":

            oggetto_guasto = self.rileva_guasto(testo) 
            if oggetto_guasto:
                #self.nodo.parla(oggetto_guasto) 
                self.contesto['oggetto_guasto'] = oggetto_guasto
                self.nodo.parla(f"Se ho capito bene , l'oggetto in questione è {oggetto_guasto} , giusto?")
                #self.nodo.parla(self.dialogo_scriptato(tipo="conferma_oggetto_guasto")) # esempio self.nodo.get_logger().info(f"ES. Mi confermi che il problema è il condizionatore?")
                self.stato = "CONFERMA_GUASTO"
            else :
                self.nodo.parla("non ho capito bene, puoi ripetere?") # esempio self.nodo.get_logger().info(f"ES. Mi confermi che il problema è il condizionatore?")
                self.stato = "RILEVA_GUASTO"
        elif self.stato == "CONFERMA_GUASTO":
            if self.rileva_conferma(testo):
                self.nodo.parla(self.domande_diagnostica["setpoint_temperatura"]) #TODO alla gabriele
                self.stato = "SUGGERISCI_SOLUZIONE_1" 
            else:
                self.nodo.parla("non ho capito allora bene, puoi ripetere il guasto?")
                # dire che non ha capito bene il robot o probabilmente non ha detto qualcosa che concerne la stanza
                self.stato = "RILEVA_GUASTO"

        elif self.stato == "SUGGERISCI_SOLUZIONE_1":
            dom1 = self.sincro.ask_llm(testo, scenario="B", tipo="dom")
            #self.nodo.parla(dom1)
            if dom1:
                self.contesto[self.domande_diagnostica["dom1"]]=dom1
                
                self.nodo.parla(self.domande_diagnostica["tempo_accensione"]) #TODO alla gabriele
                self.stato = "SUGGERISCI_SOLUZIONE_2"
            else:
                self.nodo.parla("non ho capito allora bene, puoi ripetere?")
                # print(non e' guasto perche non e' acceso da almeno xxx minuti...)
                self.stato = "SUGGERISCI_SOLUZIONE_1"


        elif self.stato == "SUGGERISCI_SOLUZIONE_2":
            dom2 = self.sincro.ask_llm(testo, scenario="B", tipo="dom")
            #self.nodo.parla(dom2)
            if dom2:
                
                self.contesto[self.domande_diagnostica["dom2"]]=dom2

                #metto su db dom1 e dom 2 TODO
                self.aggiorna_stato_oggetto()
                #chiamiamo lo spiegami tutto per vedere se ci sono oggetti in Guasto TODO





                spiegazione = self.suggerisci_specialista()
                if spiegazione:
                    self.nodo.parla(f"ti chiamo lo specialista perchè {spiegazione}  ")

                    self.stato = "CONTATTA_SPECIALISTA"
                    self.esegui("")
                else:
                    self.nodo.parla("non so che dirti, non ci sono assiomi che dicono che l'oggetto è rotto , vuoi che ti chiami cmq lo specialista ")
                    self.stato = "MEDICO_SI-NO"
            else:
                self.nodo.parla("non ho capito allora bene, puoi ripetere?")
                self.stato = "SUGGERISCI_SOLUZIONE_2"
                
        elif self.stato == "MEDICO_SI-NO":
            risposta = self.rileva_conferma(testo)
            if risposta in [None, False]:
                self.stato = "FINE"
            else:
                #self.motivo_chiamata = "I sintomi detti dal paziente non sono risultati compatibili con le sue malattie pregresse, però lui vuole chiamarti lo stesso."
                #fare query per dire che l'oggetto è guasto 
                self.stato = "CONTATTA_SPECIALISTA"
                self.esegui("")

        elif self.stato == "CONTATTA_SPECIALISTA":
            #TODO
            
            self.specialista.chiama(self.nodo,self.contesto['tipo_oggetto_guasto'], self.contesto['ospite'], self.contesto['oggetto_guasto'] ) 
            self.stato = "FINE"


        elif self.stato == "FINE":
            pass
        else:
            self.nodo.get_logger().error(f"[ScenarioB] Stato sconosciuto o non gestito: '{self.stato}'")
       

    
   

    def rileva_temperatura(self, testo):
        # testo = Assolutamente sì. La temperatura ambientale è ferma a 17°C e ho impostato il condizionatore su 26°C. Il condizionatore è acceso, ma la temperatura in stanza non è cambiata
        temperatura_impostata = 30  # llm.handle_request(scenario="B", tipo="estrazione_temperatura", msg=testo)
        self.contesto['temperatura_impostata'] = temperatura_impostata
        return None  # TODO: E SE NON VA A BUON FINE??

    def termostato(self):
        return 23  # random float tra 20.0 e 24.0

    def salva_guasto(self, kb):  # TODO
        # (guasto = true lo mette il reasoner)
        query = ""
        parametri = None
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
                MATCH (o)
                WHERE id(o) = $id
                
                // 1. Recupero la Stanza associata all'ospite tramite la Prenotazione
                OPTIONAL MATCH (o)-[:EFFETTUA]->(:Prenotazione)-[:ASSOCIATA_A]->(stanza:Stanza)
                
                // 2. Recupero tutti gli oggetti contenuti in quella stanza
                OPTIONAL MATCH (obj)-[:CONTENUTO_IN]->(stanza)

                WITH o,
                    stanza,
                    collect(DISTINCT obj) AS oggetti

                // 3. Unisco tutto in un'unica lista: [Ospite] + [Stanza] + [Lista Oggetti]
                // Nota: [stanza] crea una lista di un elemento, 'oggetti' è già una lista.
                WITH [o] + [stanza] + oggetti AS nodi_totali
                
                UNWIND nodi_totali AS n
                // Filtro per evitare nodi nulli (es. se la stanza non viene trovata)
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
        self.nodo.parla(self.dialogo_scriptato("attesa_analisi_medico"))
        if not self.recupera_dati_per_suggerimento():
            self.nodo.parla(self.dialogo_scriptato("dati_insufficienti"))
            return
        self.sincro.crea_ontologia_istanze(self.contesto["ids"])
        # nello spiegami tutto mi serve vedere la classe OspiteStatoChiamataSpecialista
        # se l'ospite è lì dentro e se ho info sul perchè, allora riporto spiegazione
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



        return spiegazione

    def contatta_specialista(self):
        #ci va perchè ha chiamato lo specialista
        # dato il problema è easy
        #stampiamo a schermo il numero e o il nome dello specialista
        # dobbiamo stampare la telefonata che il robot farebbe allo specialista
        # in cui spiega che ... SpiegamiTutto()
        return None




           

    def eseguid(self, testo):
        self.nodo.get_logger().info(f"[ScenarioB] Stato: {self.stato}, Input: {testo}")
        if self.stato == "INIZIO":
            p = self.contesto['ospite']
            self.contesto['ospite'] = Ospite(p.id, p.nome, p.cognome, "2000", "IT") 
            self.nodo.parla(f"Buonasera, sono qui per assisterLa e nel risolvere la situazione nel modo più rapido possibile. Mi dica l'oggetto  in questione danneggiato")
            self.stato = "RILEVA_GUASTO"
        elif self.stato == "RILEVA_GUASTO":
            oggetto_guasto = self.rileva_guasto(testo) 
            if oggetto_guasto:
                #self.nodo.parla(oggetto_guasto) 
                self.contesto['oggetto_guasto'] = oggetto_guasto
                self.nodo.parla(f"Se ho capito bene , l'oggetto in questione è {oggetto_guasto} , giusto?")
                #self.nodo.parla(self.dialogo_scriptato(tipo="conferma_oggetto_guasto")) # esempio self.nodo.get_logger().info(f"ES. Mi confermi che il problema è il condizionatore?")
                self.stato = "CONFERMA_GUASTO"
            else :
                self.nodo.parla("non ho capito bene, puoi ripetere?") # esempio self.nodo.get_logger().info(f"ES. Mi confermi che il problema è il condizionatore?")
                self.stato = "RILEVA_GUASTO"
        elif self.stato == "CONFERMA_GUASTO":
            if self.rileva_conferma(testo):
                self.nodo.parla(self.domande_diagnostica["setpoint_temperatura"]) #TODO alla gabriele
                self.stato = "SUGGERISCI_SOLUZIONE_1" 
            else:
                self.nodo.parla("non ho capito allora bene, puoi ripetere il guasto?")
                # dire che non ha capito bene il robot o probabilmente non ha detto qualcosa che concerne la stanza
                self.stato = "RILEVA_GUASTO"

        elif self.stato == "SUGGERISCI_SOLUZIONE_1":
            dom1 = self.sincro.ask_llm(testo, scenario="B", tipo="dom")
            #self.nodo.parla(dom1)
            if dom1:
                self.contesto[self.domande_diagnostica["dom1"]]=dom1
                
                self.nodo.parla(self.domande_diagnostica["tempo_accensione"]) #TODO alla gabriele
                self.stato = "SUGGERISCI_SOLUZIONE_2"
            else:
                self.nodo.parla("non ho capito allora bene, puoi ripetere?")
                # print(non e' guasto perche non e' acceso da almeno xxx minuti...)
                self.stato = "SUGGERISCI_SOLUZIONE_1"


        elif self.stato == "SUGGERISCI_SOLUZIONE_2":
            dom2 = self.sincro.ask_llm(testo, scenario="B", tipo="dom")
            #self.nodo.parla(dom2)
            if dom2:
                
                self.contesto[self.domande_diagnostica["dom2"]]=dom2

                #metto su db dom1 e dom 2 TODO
                self.aggiorna_stato_oggetto()
                #chiamiamo lo spiegami tutto per vedere se ci sono oggetti in Guasto TODO





                spiegazione = self.suggerisci_specialista()
                if spiegazione:
                    self.nodo.parla(f"ti chiamo lo specialista perchè {spiegazione}  ")

                    self.stato = "CONTATTA_SPECIALISTA"
                    self.esegui("")
                else:
                    self.nodo.parla("non so che dirti, non ci sono assiomi che dicono che l'oggetto è rotto , vuoi che ti chiami cmq lo specialista ")
                    self.stato = "MEDICO_SI-NO"
            else:
                self.nodo.parla("non ho capito allora bene, puoi ripetere?")
                self.stato = "SUGGERISCI_SOLUZIONE_2"
                
        elif self.stato == "MEDICO_SI-NO":
            risposta = self.rileva_conferma(testo)
            if risposta in [None, False]:
                self.stato = "FINE"
            else:
                #self.motivo_chiamata = "I sintomi detti dal paziente non sono risultati compatibili con le sue malattie pregresse, però lui vuole chiamarti lo stesso."
                #fare query per dire che l'oggetto è guasto 
                self.stato = "CONTATTA_SPECIALISTA"
                self.esegui("")

        elif self.stato == "CONTATTA_SPECIALISTA":
            #TODO
            
            self.specialista.chiama(self.nodo,self.contesto['tipo_oggetto_guasto'], self.contesto['ospite'], self.contesto['oggetto_guasto'] ) 
            self.stato = "FINE"


        elif self.stato == "FINE":
            pass
        else:
            self.nodo.get_logger().error(f"[ScenarioB] Stato sconosciuto o non gestito: '{self.stato}'")
 