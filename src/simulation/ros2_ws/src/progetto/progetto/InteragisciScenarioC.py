from progetto.InteragisciConOspite import InteragisciConOspite
from progetto.utils import Ospite

class InteragisciScenarioC(InteragisciConOspite):
    def __init__(self, nodo, specialista):
        super().__init__(nodo)
        self.specialista = specialista

    def esegui(self, testo):
        self.nodo.get_logger().info(f"[ScenarioC] Stato: {self.stato}, Input: {testo}")
        if self.stato == "INIZIO":
            self.specialista.chiama(self.nodo, "medico", "Peppe Rossi", "assiomi ritornati da spiegami tutto (es. sintomi da cardiopatia)")
            self.stato = "FINE"
        elif self.stato == "FINE":
            pass
        else:
            self.nodo.get_logger().error(f"[ScenarioC] Stato sconosciuto o non gestito: '{self.stato}'")

'''


from progetto.InteragisciConOspite import InteragisciConOspite
import time
def __init__(self,braccialeto,id_ppersona):
    braccialetto=braccialetto # ritorno dal braccialetrto manager se true allora perchè ammaccavo u buttune altrimenti dal reasoner
    #nel contesto potremmo vedere se lo scenario è attivato dal braccialetto

class InteragisciScenarioC(InteragisciConOspite): # se spiegazione diversa da null allora viene da arbitraggio, altrimenti viene dal bottone

    # Attributo di istanza

    def rileva_sintomi(self, testo, kb, llm):
        # lista_sintomi = llm.handle_request(scenario="C", tipo="estrazione_semantica", msg=testo)

        #da sopra in realtà ritorna una lista, ma per adesso facciamolo ad uno
        lista_sintomi = """vomito"""


        #gabriele unn'ava scassari a mi

        self.nodo.parla("DECOMMENTARE LA CHIAMATA LLM 1")

        # o li controlliamo tutti o per adesso camminiamo con uno
        """
        nome_classe_ufficiale = kb.trova_classe_da_sinonimo(lista_sintomi, nome_radice="Sintomo")
        if nome_classe_ufficiale:
            self.nodo.get_logger().info(f"Interesse rilevato: '{lista sintomi}' -> Mapped to: '{nome_classe_ufficiale}'")
            return nome_classe_ufficiale
        else:
            self.nodo.get_logger().warning(f"Nessuna classe ontologica trovata per: '{nome_interesse_raw}'")
            return None
        """
    def notifica_specialista():
        return None

    def esegui(self, testo, kb, llm):
        self.nodo.get_logger().info(f"[ScenarioC] Stato: {self.stato}, Input: {testo}")
        if self.stato == "INIZIO":
            #avendo l'self.contesto['ospite'].id vediamo l'id della persona(oppure salviamo lo stato di partenza sul context )

            #statoOntologiaOspite= chiamata java per vedere che classe è l'ospite #da vedere su python
            if statoOntologiaOspite == "OspiteInStatodiAllerta" :
                #reasoner

                # Perchè self.contesto['ospite'].id è classe  OspiteInStatodiAllerta    questo ci dobbiamo fare dire da spiegami tutto

                inizio_stato = time.time()
                self.stato = "COSCIENZA"

            elif statoOntologiaOspite == "Ospite":
                #vengo dal braccialetto
                # dal mio rilevamento  seii in stato di buona salute, mi hai chiamato dal braccialetto. Dimmi quali sintomi ti affliggono.


                # metto la persona da Ospite a OspiteInStatodiAllerta
                self.stato = "DESCRIZIONE_SINTOMI"


        elif self.stato == "COSCIENZA":
            # domandiamo se è cosciente


            # start time

            # ritorna il testo con si o no

            tempo_trascorso = time.time() - inizio_stato
            if super().rileva_conferma(testo)==True:

                self.stato = "DESCRIZIONE_SINTOMI"
            elif tempo_trascorso > 30 or super().rileva_conferma(testo):
                # non può rispondere no, se passa più di un certo tempo entriamo qua
                #e mostriamo a schermo sta cazzata
                #chiama dottore


                #forzare nell'ontologia da OspiteInStatodiAllerta a  NOTIFICA_SPECIALISTA  , mi piace , voto 10 /10
                self.stato = "NOTIFICA_SPECIALISTA"



        elif self.stato == "NOTIFICA_SPECIALISTA":
            self.notifica_specialista()# spiegami tutto
            self.nodo.get_logger().info("sto chiammando il dottore, stai tranq")
            self.stato = "FINE"



        elif self.stato == "DESCRIZIONE_SINTOMI":


            # lavoriamo qua , ava rinescere

            sintomi = self.rileva_sintomi(testo, kb, llm)

            #ritornano  i sintomi
            # si salvano i sintomi su neo4j
            # successivamente vanno messi nell'ontologia

            # attiviamo il reasoner nell'ontologia che ci dice se dobbiamo
            # dare consigli generici sui sintomi o chiamare il dottore se
            # indivifìduiamo i sintomi legati alla sua malattia pregressa

            chiamiamo java spiegami tutto per dire in che classe è , e lo spieghiamo se esiste una spiegazione, altrimenti none se è uguale allo stato precedente
            ritornano o gli assiomi o la spiegazione
            self.stato = "NOTIFICA_SPECIALISTA"
            #se la frase dice che non riporta assiomi allora

            else:
            #se rimane nella classe ospite in allerta significa che non ha superato la soglia e il robot da consigli generici sui sintomi
            #alla fine gli domanda comunque se vuole chiamare il medico e se dice di NOTIFICA_SPECIALISTA.

            # qua bisogna ritornare testo, quindi ci va un altro stato.

            self.stato == "MEDICOSI-NO"
            #else

            #se cambia a NOTIFICA_SPECIALISTA allora chiama il medico.


            #altrimenti ritorna  dallo spiegami tutto che non ha assiomi da spiegare.
            self.stato == "FINE"


        elif self.stato == "FINE":
            self.nodo.get_logger().info("Ciao ragazz*, vado alla hall")

        elif self.stato == "MEDICOSI-NO":
            if super().rileva_conferma(testo):
                # se lui è qua significa che la sua classe va cambiata in notifica specialista
                self.stato = "NOTIFICA_SPECIALISTA"
            else :
                self.stato == "FINE"
'''
