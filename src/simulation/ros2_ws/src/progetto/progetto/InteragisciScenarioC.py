from progetto.InteragisciConOspite import InteragisciConOspite
from progetto.utils import Ospite
import ast
import json
import time

class InteragisciScenarioC(InteragisciConOspite):

    def __init__(self, nodo, specialista):  # , spiegazione): # da vedere dove viene chiamato e controllare il costruttore
        super().__init__(nodo)
        self.specialista = specialista
        #self.spiegazione=""
        self.spiegazione="Sei qua perchè ho rilevato un'emergenza"# self.spiegazione = spiegazione # se viene dal rilevamento automatico allora spiegazione != null, altrimenti se vuoto veniamo dal bottone
        # in fase di testing, per adesso quando clicchiamo i lbottone dobbiamo provare con spiegazione e senza


    #TODO
    def rileva_sintomi(self, testo):

    
        lista_sintomi = self.sincro.ask_llm(testo, scenario="C", tipo="estrazione_semantica")
        lista_sintomi = ast.literal_eval(lista_sintomi)
        #lista_sintomi=['cefalea', 'tosse']
        if not lista_sintomi:
            return []
        self.nodo.parla(f"Sintomi estratti: {lista_sintomi}, {type(lista_sintomi)}")
        #da sopra in realtà ritorna una lista, ma per adesso facciamolo ad uno
        #lista_sintomi = ['tosse', 'tachicardia']


        #gabriele unn'ava scassari a mi



        # o li controlliamo tutti o per adesso camminiamo con uno

        nome_classe_ufficiale =self.sincro.trova_classe_da_sinonimo(lista_sintomi, nome_radice="Sintomo")# questo sarà per uno, penso che per qua va esteso
        self.nodo.get_logger().info(f"Interesse rilevato: '{lista_sintomi}' -> Mapped to: '{nome_classe_ufficiale}'")
        if nome_classe_ufficiale:
            self.nodo.get_logger().info(f"Interesse rilevato: '{lista_sintomi}' -> Mapped to: '{nome_classe_ufficiale}'")
            return nome_classe_ufficiale
        else:
            self.nodo.get_logger().warning(f"Nessuna classe ontologica trovata per: '{nome_classe_ufficiale}'")
            return []


    def aggiorna_sintomi(self,lista_nomi):
        query= "MATCH (o:Ospite) WHERE id(o) = $id UNWIND  $lista_nomi AS nome MERGE (s:Sintomo {nome_sintomo: nome}) MERGE (o)-[r:HA_SINTOMO]->(s) RETURN o, r, s"
        parametri = {
            "id"         : self.contesto['ospite'].id,
            "lista_nomi" : lista_nomi
        }
        aggiornato = self.sincro.interrogaGraphDatabase(query, parametri)
        return aggiornato

    


    def cambia_stato(self):
        #query = "MATCH (n) WHERE id(n) = $p.id REMOVE n:Ospite:OspiteInEmergenza SET n:OspiteChiamatoSpecialista,  n.aggiornato_il = datetime() RETURN n" per mettere il timestamp
        query = "MATCH (n) WHERE id(n) = $id REMOVE n:Ospite:OspiteInStatodiAllerta SET n:OspiteStatoChiamataSpecialista RETURN n"
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

                self.nodo.parla(self.spiegazione)
                self.nodo.parla("Puoi dirmi se sei cosciente?")
                #vediamo se risponde prima di un certo tempo
                self.inizio_stato_di_coscienza = time.time() # vedere se metterlo static oppure da qualche altra parte in maniera che si vede nel codice

                self.stato = "COSCIENZA"

            else : #spiegazione vuota, quindi vengo dal braccialetto

                self.nodo.parla("Dal mio ultimo rilevamento non sei in stato di cattiva salute, mi hai chiamato dal braccialetto. Dimmi quali sintomi ti affliggono.")

                # metto la persona da Ospite a OspiteInStatodiAllerta  su neo 4j # da capire, forse si, così non lo rimonitoro , al momento no

                # vice dice di farlo quando  clicca il bottone, si va fatto quando clicca il bottone dello scenario C
                #gab boccia, dangerous perchè non c'è spiegazione e ci sono rischi arbitraggio
                
                self.stato = "DESCRIZIONE_SINTOMI"


        elif self.stato == "COSCIENZA":   # domandiamo se è cosciente
            #da capire se il tempo va  qua o meno
            self.nodo.parla("Fase coscienza")
            # ritorna il testo con si o no

            tempo_trascorso = time.time() - self.inizio_stato_di_coscienza # da vedere se funge
            self.nodo.get_logger().info(f"{tempo_trascorso}")
            
            p = self.contesto['ospite']
            #va recuperato o un contesto completo oppure faccio una funzione che mi da eta e lingua e aggiungo dopo, questo serve in scenarioB
            self.contesto['ospite'] = Ospite(p.id, p.nome, p.cognome, "2000", "IT")
            self.nodo.get_logger().info(f"{p}")
            #importante che su neo4j ci sia la lingua della persona registrata
            risposta = self.rileva_conferma(testo)

            if not risposta or tempo_trascorso > 30:
                self.nodo.parla(tempo_trascorso)
                self.stato = "CHIAMATA_SPECIALISTA"
            else:
                self.nodo.parla("Dimmi che sintomi hai per favore.")
                self.stato = "DESCRIZIONE_SINTOMI"


        elif self.stato == "CHIAMATA_SPECIALISTA":


            # si mette su neo4j che la persona che ha id passa dallo stato Ospite o OspiteInStatodiAllerta a OspiteStatoChiamataSpecialista
            self.nodo.get_logger().info(f"{self.cambia_stato()}")

            self.nodo.parla("Sto chiamando il dottore, stai tranq, faccio subito, pf")

            #TODO

            #self.specialista.chiama(self.nodo, "medico", "Peppe Rossi", "assiomi ritornati da spiegami tutto (es. sintomi da cardiopatia)") # penso che questo preceda questo stato, da capire , da fare , da vedere
            self.stato = "FINE"



        elif self.stato == "DESCRIZIONE_SINTOMI":




            lista_sintomi = self.rileva_sintomi(testo)  # vedere come fatto nello scen A
            # Soluzione robusta
            self.nodo.parla(lista_sintomi)
            #lista_sintomi = [s.strip() for s in self.rileva_sintomi(testo).split(",")]

            # ritornano i sintomi e si salvano i sintomi su neo4j
            self.nodo.get_logger().info(f"{self.aggiorna_sintomi(lista_sintomi)}")


            #TODO
            # da fare, da vedere self.sincro.crea_ontologia_istanze(self.contesto["ids"]) # da vedere da gab
            # creiamo l'ontologia con quello che ci serve



            # attiviamo il reasoner nell'ontologia che ci dice se dire che non sappiamo dire nulla di certo oppure se chiamare il dottore nel caso
            # in cui individuiamo i sintomi legati alla sua malattia pregressa
            # e quindi chiamiamo java spiegami tutto per dire in che classe è ,
            # mi ritornano le informazioni in maniera indiretta

            #qua ci va la regola dei punteggi
            #TODO
            assiomi =self.sincro.spiegami_tutto(parentClassName="Ospite")
            self.nodo.get_logger().info(f"{assiomi}")
            self.nodo.get_logger().info(f"{json.loads(assiomi)}")

            #TODO
            spiegazione=""# qua vedo cosa c'è dentro   OspiteStatoChiamataSpecialista
            # mi salvo la spiegazione che è notifica specialista, se esiste



            if spiegazione :

                self.nodo.parla(spiegazione)

                self.stato == "CHIAMATA_SPECIALISTA"

            else:

                self.nodo.parla("Non ho abbastanza assiomi per chiamare il dottore")

                # domanda comunque se vuole chiamare il medico e se dice si, lo classifichiamo OspiteStatoChiamataSpecialista .

                self.stato == "MEDICO_SI-NO"






        elif self.stato == "MEDICO_SI-NO":

            risposta = self.rileva_conferma(testo)

            if risposta is None or False:
                self.stato == "FINE"
            else:
                self.stato = "CHIAMATA_SPECIALISTA"


        elif self.stato == "FINE":
            pass
        else:
            self.nodo.get_logger().error(f"[ScenarioC] Stato sconosciuto o non gestito: '{self.stato}'")

'''
#query
#da fare
#da capire
'''
