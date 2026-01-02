import string

class InteragisciConOspite():
    def __init__(self, nodo):
        self.nodo = nodo
        self.stato = None
        self.contesto = {}

    def reset(self, ospite):
        self.stato = "INIZIO"
        self.contesto = {}
        self.contesto["ospite"] = ospite

    def dialogo_scriptato(self, tipo):
        lingua = self.contesto['ospite'].lingua
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
        # Ho capito che ti interessa '{nome_interesse_clean}', ma non è presente tra i servizi dell'albergo.

    def rileva_conferma(self, testo):
        lingua = self.contesto['ospite'].lingua
        testo_clean = testo.lower().translate(str.maketrans('', '', string.punctuation))
        parole = set(testo_clean.split())
        keywords = {
            'IT': {
                'POS': {
                    'si', 'sì', 'certo', 'ok', 'va bene', 'ovvio', 'sicuro',
                    'confermo', 'esatto', 'perfetto', 'ottimo', 'volentieri',
                    'assolutamente', 'procedi', 'dai', 'certamente'
                },
                'NEG': {
                    'no', 'negativo', 'sbagliato', 'ferma', 'annulla', 'non',
                    'basta', 'errore', 'aspetta', 'mai', 'neanche', 'per nulla'
                }
            },
            'EN': {
                'POS': {
                    'yes', 'yeah', 'yup', 'sure', 'ok', 'okay', 'fine', 'correct',
                    'confirm', 'perfect', 'great', 'absolutely', 'go ahead',
                    'right', 'yep', 'of course'
                },
                'NEG': {
                    'no', 'nope', 'nah', 'negative', 'wrong', 'stop', 'cancel',
                    'wait', 'never', 'not'
                }
            }
        }
        vocab_pos = keywords[lingua]['POS']
        vocab_neg = keywords[lingua]['NEG']
        has_pos = not parole.isdisjoint(vocab_pos)
        has_neg = not parole.isdisjoint(vocab_neg)
        if has_pos and not has_neg:
            return True
        if has_neg and not has_pos:
            return False
        return False

    def esegui(self, testo):
        raise NotImplementedError("Ogni InteragisciScenario deve implementare la propria logica.")
