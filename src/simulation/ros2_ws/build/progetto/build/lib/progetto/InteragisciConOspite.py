class InteragisciConOspite():
    def __init__(self, nodo):
        self.nodo = nodo
        self.reset()

    def reset(self):
        self.stato = "INIZIO"
        self.contesto = {}

    def dialogo_scriptato(self, tipo):
        lingua = self.contesto['lingua']
        if tipo == "benvenuto":
            nome      = self.contesto['ospite'].nome
            cognome   = self.contesto['ospite'].cognome
            num_notti = self.contesto['num_notti']
            if lingua == 'IT':
                return f"Benvenuto {nome} {cognome}! Piacere sono Pippor e sono qui per assisterti. Vedo che hai prenotato per {num_notti} notti. Posso chiederti il motivo del tuo viaggio? Hai interessi particolari?"
            elif lingua == 'EN':
                return f"Welcome {nome} {cognome}. I see {num_notti} nights. Any interests?"
        elif tipo == "conferma_interesse":
            interesse = self.contesto['interesse']
            if lingua == 'IT':
                return f"Ah confermi che il tuo interesse è {interesse}?"
            elif lingua == 'EN':
                return f"Do you confirm you like: {interesse}?"
        elif tipo == "arrivederci":
            if lingua == 'IT':
                nome = self.contesto['ospite'].nome
                return f"Grazie a te {nome}, rimango a disposizione!"
            elif lingua == 'EN':
                return f"Thanks {nome}, I am here for you."
        elif tipo == "errore_lingua":
            return "Non ho capito la lingua, puoi ripetere? (Prova 'Ciao'). I don't understand the language, could you repeat? (Try 'Hello')."

    def rileva_conferma(self, testo):
        lingua = self.contesto['lingua']
        t = testo.lower()
        if lingua == 'IT':
            return t in ["si", "sì", "certo", "ok"]
        elif lingua == 'EN':
            return t in ["yes", "yeah", "sure", "ok"]
        return False

    def esegui(self, testo):
        raise NotImplementedError("Ogni InteragisciScenario deve implementare la propria logica.")