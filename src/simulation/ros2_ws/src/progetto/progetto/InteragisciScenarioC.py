from progetto.InteragisciConOspite import InteragisciConOspite
from progetto.utils import Ospite
import ast
import json
import time

class InteragisciScenarioC(InteragisciConOspite):

    def __init__(self, nodo, specialista,spiegazione):  # , spiegazione): # da vedere dove viene chiamato e controllare il costruttore
        super().__init__(nodo)
        self.specialista = specialista
        self.motivo_chiamata = ""
        self.spiegazione=spiegazione
        #self.spiegazione="Sei qua perchè ho rilevato un'emergenza"# self.spiegazione = spiegazione # se viene dal rilevamento automatico allora spiegazione != null, altrimenti se vuoto veniamo dal bottone
        # in fase di testing, per adesso quando clicchiamo i lbottone dobbiamo provare con spiegazione e senza

    def rileva_sintomi(self, testo):
        lista_sintomi = self.sincro.ask_llm(testo, scenario="C", tipo="estrazione_semantica")
        lista_sintomi = ast.literal_eval(lista_sintomi)
        self.contesto['sintomi']=lista_sintomi
        #lista_sintomi=['cefalea', 'tosse']
        if not lista_sintomi:
            return []
        self.nodo.parla(f"Sintomi estratti: {lista_sintomi}, {type(lista_sintomi)}")
        nome_classe_ufficiale =self.sincro.trova_classe_da_sinonimo(lista_sintomi, nome_radice="Sintomo")# questo sarà per uno, penso che per qua va esteso

        self.nodo.parla(f"Sintomi nome_classe_ufficiale: {nome_classe_ufficiale}, {type(lista_sintomi)}")



        self.nodo.get_logger().info(f"Interesse rilevato: '{lista_sintomi}' -> Mapped to: '{nome_classe_ufficiale}'")
        if nome_classe_ufficiale:
            self.nodo.get_logger().info(f"Interesse rilevato: '{lista_sintomi}' -> Mapped to: '{nome_classe_ufficiale}'")
            return nome_classe_ufficiale
        else:
            self.nodo.get_logger().warning(f"Nessuna classe ontologica trovata per: '{nome_classe_ufficiale}'")
            return []

    def aggiorna_sintomi(self,lista_label):
        lista_nomi = [d for d, f in zip(self.contesto['sintomi'],lista_label) if f != ""]
        lista_label = [f for f in lista_label if f != ""]
        for nome, label in zip(lista_nomi,lista_label):
            
            query = f"""
            MATCH (o) WHERE id(o) = $id
            MERGE (s:Sintomo:{label} {{nome_sintomo: $nome}})
            MERGE (o)-[:HA_SINTOMO]->(s)
            """
            
            parametri = {
                "id": self.contesto['ospite'].id,
                "nome": nome
            }
            
            # Esecuzione singola
            self.sincro.interrogaGraphDatabase(query, parametri)
        return True

    def recupera_dati_per_suggerimento(self):
        query = """
                MATCH (o)
                WHERE id(o) = $id
                OPTIONAL MATCH (o)-[:SOFFRE_DI]->(pat)
                OPTIONAL MATCH (o)-[:HA_SINTOMO]->(s:Sintomo)
            
                WITH o, 
                    collect(DISTINCT pat) AS patologie, 
                    collect(DISTINCT s) AS sintomi
                   
                WITH [o] + patologie + sintomi AS nodi_totali
                UNWIND nodi_totali AS n
                RETURN DISTINCT id(n) AS node_id
             """
        risultati = self.sincro.interrogaGraphDatabase(query, {'id': self.contesto['ospite'].id})
        if risultati:
            self.contesto["ids"] = [record['node_id'] for record in risultati if record['node_id'] is not None]
            self.nodo.get_logger().info(f"[ScenarioC] Eventi caricati da DB: {len(self.contesto['ids'])}")
            return True
        return False
    
    def suggerisci_medico(self):
        self.nodo.parla("Un attimo, analizzo e chiedo al sistema esperto...")
        if not self.recupera_dati_per_suggerimento():
            self.nodo.parla("Non ho trovato dati sufficienti nel database.")
            return 
        self.sincro.crea_ontologia_istanze(self.contesto["ids"])
        # nello spiegami tutto mi serve vedere la classe OspiteStatoChiamataSpecialista, se  l'ospite è lì dentro e se ho info sul perchè , allora riporto spiegazione
        assiomi = self.sincro.spiegami_tutto(parentClassName="Ospite")
        spiegazione=""
        data = json.loads(assiomi)
        for evento_str, proprieta in data.items():
            if proprieta["OspiteStatoChiamataSpecialista"] == "Il reasoner NON deduce assiomi inerenti":                    
                    self.nodo.parla(f"Il reasoner NON deduce assiomi inerenti. i sintomi che hai detto non fanno suscitare in me alcun bisogno di chiamare il dottore. Se vuoi lo posso chiamare lo stesso.")
            else:
                    spiegazione=proprieta['OspiteStatoChiamataSpecialista']
                    # spiegazione = self.sincro.ask_llm(spiegazione, scenario="c", tipo="explainability") # da allineare il prompt
                    self.nodo.parla(spiegazione)
                    # aggiornato = self.cambia_stato()  # cambiare stato da quello che è a Specialista o Ospite a ChiamatoSpecialista
                    # salvare il time stamp che è avvenuto questo  cambiamento?? Il date time?
        return spiegazione
            
    def cambia_stato(self):
        #query = "MATCH (n) WHERE id(n) = $p.id REMOVE n:Ospite:OspiteInEmergenza SET n:OspiteChiamatoSpecialista,  n.aggiornato_il = datetime() RETURN n" per mettere il timestamp
        query = "MATCH (n) WHERE id(n) = $id REMOVE n:Ospite, n:OspiteInStatodiAllerta, n:OspiteStatoChiamataSpecialista SET n:OspiteStatoChiamataSpecialista RETURN n, datetime() AS momentoTransizione"
        parametri = {
            "id":     self.contesto['ospite'].id,
        }
        aggiornato = self.sincro.interrogaGraphDatabase(query, parametri)
        return aggiornato

    def esegui(self, testo):
        # questo esegui() va fatto partire dopo che il robot è davanti la persona.
        self.nodo.get_logger().info(f"[ScenarioC] Stato: {self.stato}, Input: {testo}")
        if self.stato == "INIZIO":
            #se c'è la spiegazione veniamo dal braccialetto manager (rilevamento automatico), altrimenti dal bottone
            if self.spiegazione :
                # stampare spiegazione a schermo
                self.nodo.parla(f"{self.spiegazione} Puoi dirmi se sei cosciente?")
                #vediamo se risponde prima di un certo tempo
                self.inizio_stato_di_coscienza = time.time() # vedere 
                self.stato = "COSCIENZA"
            else : #spiegazione vuota, quindi vengo dal braccialetto
                self.nodo.parla("Dal mio ultimo rilevamento non sei in stato di cattiva salute, mi hai chiamato dal braccialetto. Dimmi quali sintomi ti affliggono.")
                # metto la persona da Ospite a OspiteInStatodiAllerta  su neo 4j # da capire, forse si, così non lo rimonitoro , al momento no
                # vice dice di farlo quando  clicca il bottone, si va fatto quando clicca il bottone dello scenario C
                #gab boccia, dangerous perchè non c'è spiegazione e ci sono rischi arbitraggio
                self.stato = "DESCRIZIONE_SINTOMI"
        elif self.stato == "COSCIENZA":
            p = self.contesto['ospite']
            #va recuperato o un contesto completo oppure faccio una funzione che mi da eta e lingua e aggiungo dopo, questo serve in scenarioB
            self.contesto['ospite'] = Ospite(p.id, p.nome, p.cognome, "2000", "IT")
            self.nodo.get_logger().info(f"{p}")  #importante che su neo4j ci sia la lingua della persona registrata
            tempo_trascorso = time.time() - self.inizio_stato_di_coscienza # da vedere se funge
            self.nodo.get_logger().info(f"{tempo_trascorso}")
            risposta = self.rileva_conferma(testo)
            if not risposta or tempo_trascorso > 30:
                self.nodo.parla(tempo_trascorso)
                self.motivo_chiamata = " tempo di risposta superiore ai 30 secondi"
                self.cambia_stato()
                self.stato = "CHIAMATA_SPECIALISTA"
                self.esegui("")
            else:
                self.nodo.parla("Dimmi che sintomi hai per favore.")
                self.stato = "DESCRIZIONE_SINTOMI"
        elif self.stato == "CHIAMATA_SPECIALISTA":
            self.nodo.parla("Sto chiamando il dottore, stai tranq, faccio subito, pf")
            #TODO
            self.specialista.chiama(self.nodo, "medico", self.contesto['ospite'], self.motivo_chiamata ) # penso che questo preceda questo stato, da capire , da fare , da vedere
            self.stato = "FINE"
        elif self.stato == "DESCRIZIONE_SINTOMI":
            lista_sintomi = self.rileva_sintomi(testo)
            #self.contesto['sintomi'] = lista_sintomi
            self.nodo.parla(lista_sintomi)
            # ritornano i sintomi e si salvano i sintomi su neo4j
            self.nodo.get_logger().info(f"{self.aggiorna_sintomi(lista_sintomi)}")
            spiegazione=self.suggerisci_medico()
            if spiegazione :
                #self.nodo.parla(spiegazione)
                self.motivo_chiamata = spiegazione
                self.cambia_stato()
                self.stato = "CHIAMATA_SPECIALISTA"
                self.esegui("")
            else:
                self.nodo.parla("Non ho abbastanza assiomi per chiamare il dottore, vuoi comunque chiamarlo?")
                self.stato = "MEDICO_SI-NO"
        elif self.stato == "MEDICO_SI-NO":
            risposta = self.rileva_conferma(testo)
            if risposta in [None, False]:
                self.stato = "FINE"
            else:
                self.motivo_chiamata = " i sintomi detti dal paziente non sono risultati compatibili con le sue malattie pregresse, però lui vuole chiamarti lo stesso ."
                self.cambia_stato()
                self.stato = "CHIAMATA_SPECIALISTA"
                self.esegui("")
        elif self.stato == "FINE":
            pass
        else:
            self.nodo.get_logger().error(f"[ScenarioC] Stato sconosciuto o non gestito: '{self.stato}'")