from progetto.InteragisciConOspite import InteragisciConOspite
from progetto.utils import Ospite

class InteragisciScenarioB(InteragisciConOspite):
    def __init__(self, nodo, specialista):
        super().__init__(nodo)
        self.specialista = specialista

    def reset(self, ospite=None):
        super().reset(ospite)
        #self.nodo.destinazione_target = (-10, 11)
        self.nodo.destinazione_target = (7, -7.8)  # Stanza 3
        #self.nodo.destinazione_target = (-10, 7)  # Intra u divanu
        #self.nodo.destinazione_target = (7 + (-0.28921), 4.2 + 2.5634)  # stanza 1
        #self.nodo.destinazione_target = (6, 4)  # stanza 1
        #self.nodo.destinazione_target = (11, 11)  # narrè
        self.nodo.raggiunta_destinazione = False
        self.nodo.comportamento_precedente = "InteragisciScenarioB"
        self.stato = "ASPETTA_ROBOT"
        self.nodo.comportamenti["Naviga"].reset()

    def esegui(self, testo):
        self.nodo.get_logger().info(f"[ScenarioB] Stato: {self.stato}, Input: {testo}")
        if self.stato == "ASPETTA_ROBOT":
            if self.nodo.raggiunta_destinazione:
                self.specialista.chiama(self.nodo, "elettricista", "camera 1", "assiomi ritornati da spiegami tutto (es. phon guasto)")
                self.stato = "FINE"
            else:
                self.nodo.parla("Arrivo subito, sto arrivando da te!")
        elif self.stato == "FINE":
            pass
        else:
            self.nodo.get_logger().error(f"[ScenarioB] Stato sconosciuto o non gestito: '{self.stato}'")

'''
    def recupera_eta_lingua(self, kb):
        query = "MATCH (o:Ospite) WHERE id(o) = $id RETURN o.eta, o.lingua"
        result = kb.interrogaGraphDatabase(query, {'id': self.contesto['ospite'].id})[0]
        return result['o.eta'], result['o.lingua']

    def rileva_guasto(self, testo):
        # testo = "Ciao Pippor, continuo a sentire molto freddo."
        nome_guasto_raw = "caldo"  # self.sincro.ask_llm(testo, scenario="B", tipo="estrazione_semantica")
        self.contesto['tipologia_guasto'] = nome_guasto_raw
        
        risultati = self.sincro.trova_classe_da_sinonimo([nome_guasto_raw], nome_radice="Guasto")
        #Se la lista è vuota, assegna None (o un valore di default), altrimenti il primo elemento
        nome_classe_ufficiale = risultati[0] if risultati else None

        nome_classe_ufficiale =self.sincro.trova_classe_da_sinonimo(nome_guasto_raw, nome_radice="Guasto")
        if nome_classe_ufficiale:  # es. condizionatore
            self.nodo.get_logger().info(f"Guasto rilevato: '{nome_guasto_raw}' -> Mapped to: '{nome_classe_ufficiale}'")
            return nome_classe_ufficiale  # es. ritorno che il problema è il condizionatore
        else:
            self.nodo.get_logger().warning(f"Nessuna classe ontologica trovata per: '{nome_guasto_raw}'")
            return []

    """
    def ritorna_soglia_regola(self, oggetto, modalita=None):
    from owlready2 import *

    onto = get_ontology("ontologiaAlbergo.rdf").load()

    def debug_profondo_regole(ontologia):
        soglie_modalita = {}
        for i, regola in enumerate(ontologia.rules()):
            # Analizziamo solo le regole che sembrano quelle del condizionatore
            if "Condizionatore" not in str(regola):
                continue
            print(f"\nAnalisi Regola #{i+1}: {regola}")
            modo_trovato = None
            nome_variabile_tempo = None
            soglia_tempo = None
            for atomo in regola.body:
                # Stampiamo COSA è questo atomo per capire perché falliva prima
                print(f"  [Atomo]: {atomo}")
                print(f"    -> Tipo Python: {type(atomo)}")
                # Controllo ARGOMENTI (spesso più sicuro che controllare le proprietà)
                if hasattr(atomo, 'arguments'):
                    args = atomo.arguments
                    print(f"    -> Argomenti: {args}")
                    # LOGICA DI ESTRAZIONE BASATA SUGLI ARGOMENTI
                    # Cerchiamo l'atomo che ha 'HEAT' o 'COOL' come secondo argomento
                    if len(args) > 1:
                        secondo_arg = args[1]
                        # 1. CERCA MODALITA' (controllando se il valore è HEAT/COOL)
                        if str(secondo_arg) in ["HEAT", "COOL", "'HEAT'", "'COOL'"]:
                            modo_trovato = str(secondo_arg).replace("'", "")
                            print(f"    >>> TROVATO MODO: {modo_trovato}")
                        # 2. CERCA VARIABILE TEMPO (Cerchiamo l'atomo 'tempo_acceso')
                        # Se non riusciamo a leggere .property.name, guardiamo la stringa dell'atomo
                        elif "tempo_acceso" in str(atomo):
                            if hasattr(secondo_arg, 'name'):
                                nome_variabile_tempo = secondo_arg.name
                                print(f"    >>> TROVATA VAR TEMPO: ?{nome_variabile_tempo}")
                # 3. CERCA SOGLIA (greaterThan)
                if hasattr(atomo, 'builtin'):
                    # Verifica lasca se è greaterThan
                    if "greaterThan" in str(atomo.builtin):
                        arg_var = atomo.arguments[0] # es. ?t
                        arg_val = atomo.arguments[1] # es. 20
                        print(f"    -> Controllo soglia su var: {arg_var}, val: {arg_val}")
                        if nome_variabile_tempo and hasattr(arg_var, 'name'):
                            if arg_var.name == nome_variabile_tempo:
                                if isinstance(arg_val, (int, float)):
                                    soglia_tempo = arg_val
                                    print(f"    >>> TROVATA SOGLIA: {soglia_tempo}")
            if modo_trovato and soglia_tempo is not None:
                soglie_modalita[modo_trovato] = soglia_tempo
                print(f"  *** REGOLA COMPLETATA: {modo_trovato} -> {soglia_tempo} ***")
        return soglie_modalita

    risultato = debug_profondo_regole(onto)
    print("\n--- RISULTATO FINALE ---")
    print(risultato)
    return risultato
    """

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

    def contatta_specialista(self):
        #ci va perchè ha chiamato lo specialista
        # dato il problema è easy
        #stampiamo a schermo il numero e o il nome dello specialista
        # dobbiamo stampare la telefonata che il robot farebbe allo specialista
        # in cui spiega che ... SpiegamiTutto()
        return None

    def esegui(self, testo):
        self.nodo.get_logger().info(f"[ScenarioB] Stato: {self.stato}, Input: {testo}")
        if self.stato == "INIZIO":
            self.nodo.parla(f"Buonasera, Sono qui per assisterLa e nel risolvere la situazione nel modo più rapido possibile. Mi dica l'oggetto  in questione danneggiato")
            self.stato = "RILEVA_GUASTO"
        elif self.stato == "RILEVA_GUASTO":
            oggetto_guasto = self.rileva_guasto(testo)
            if oggetto_guasto:
                self.nodo.get_logger().info(oggetto_guasto)
                self.contesto['oggetto_guasto'] = oggetto_guasto
                #self.nodo.parla(self.dialogo_scriptato(tipo="conferma_oggetto_guasto")) # esempio self.nodo.get_logger().info(f"ES. Mi confermi che il problema è il condizionatore?")
                self.stato = "CONFERMA_GUASTO"
        elif self.stato == "CONFERMA_GUASTO":
            if self.rileva_conferma(testo):
                self.stato = "SUGGERISCI_SOLUZIONE_1"
            else:
                # dire che non ha capito bene il robot o probabilmente non ha detto qualcosa che concerne la stanza
                self.stato = "RILEVA_GUASTO"
        elif self.stato == "SUGGERISCI_SOLUZIONE_1":
            # self.dialogo_scriptato_suggerisci_soluzioni()
            # recuperare 20 SE RISCALDARE O 40 SE RAFFREDARE attenzione che caldo->HEAT e freddo->COOL
            self.nodo.get_logger().info(f"Innanzitutto, Sig. Mario, il condizionatore è acceso da almeno (20|40) minuti ?")
            self.stato = "CONFERMA_SUGGERISCI_SOLUZIONE_1"
        elif self.stato == "CONFERMA_SUGGERISCI_SOLUZIONE_1":
            if self.rileva_conferma(testo):
                self.contesto['CONFERMA_1'] = True
                self.stato = "SUGGERISCI_SOLUZIONE_2"
            else:
                self.contesto['CONFERMA_1'] = False
                # print(non e' guasto perche non e' acceso da almeno xxx minuti...)
                self.stato = "FINE"
        elif self.stato == "SUGGERISCI_SOLUZIONE_2":
            self.nodo.get_logger().info(f"Molto chiaro. Può confermarmi che il dispositivo è impostato sulla modalità riscaldamento (Heat|Cool)?")
            self.stato = "CONFERMA_SUGGERISCI_SOLUZIONE_2"
        elif self.stato == "CONFERMA_SUGGERISCI_SOLUZIONE_2":
            if self.rileva_conferma(testo):
                self.contesto['CONFERMA_2'] = True
                self.stato = "SUGGERISCI_SOLUZIONE_3"
            else:
                self.contesto['CONFERMA_2'] = False
                # print(non e' guasto perche non e' impostato sulla modalita' ...)
                self.stato = "FINE"
        elif self.stato == "SUGGERISCI_SOLUZIONE_3":
            self.nodo.get_logger().info(f"La prego di rispondermi ad un'ultima domanda, a quanto ha impostato la temperatura desiderata?")
            self.stato = "CONFERMA_SUGGERISCI_SOLUZIONE_3"
        elif self.stato == "CONFERMA_SUGGERISCI_SOLUZIONE_3":
                self.rileva_temperatura(testo)
                temp_ambiente = 23  # self.termostato()
                # reasoner
                self.salva_guasto()
               
                self.nodo.get_logger().info("La ringrazio, per la Sua collaborazione. Dalle Sue risposte, deduco che il problema non derivi da un uso errato del dispositivo, bensì da un potenziale guasto tecnico all'unità. Non è un problema che posso risolvere autonomamente. Mi scuso per il disagio. Provvederò immediatamente a contattare e informare personalmente il tecnico di turno, richiedendo che intervenga nella Sua stanza nel più breve tempo possibile.")
                self.stato = "CONTATTA_SPECIALISTA"
        elif self.stato == "CONTATTA_SPECIALISTA":
            self.specialista.chiama(self.nodo, "tecnico idoneo", self.contesto['ospite'], self.motivo_chiamata )
            self.stato = "FINE"
        elif self.stato == "FINE":

        else:
            self.nodo.get_logger().info("self.stato NON VALIDO.")
'''
