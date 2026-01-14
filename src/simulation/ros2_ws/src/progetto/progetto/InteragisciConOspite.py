import json
import os
import string

from dotenv import load_dotenv
load_dotenv()

class InteragisciConOspite():
    def __init__(self, nodo):
        self.nodo = nodo
        self.stato = None
        self.sincro = nodo.sincro
        self.contesto = {}
        self.vocab_conferme = {
            'IT': {
                'POS': self.carica_vocabolario("conferme", "ita_pos.txt"),
                'NEG': self.carica_vocabolario("conferme", "ita_neg.txt")
            },
            'EN': {
                'POS': self.carica_vocabolario("conferme", "eng_pos.txt"),
                'NEG': self.carica_vocabolario("conferme", "eng_neg.txt")
            }
        }
        with open("src/progetto/progetto/vocabolari/dialoghi.json", "r", encoding="utf-8") as f:
            self.dialoghi = json.load(f)

    def reset(self, ospite):
        self.stato = "INIZIO"
        self.contesto = {}
        self.contesto["ospite"] = ospite

    def carica_vocabolario(self, cartella, file_nome):
        base_path = os.getenv("VOCABOLARI")
        full_path = os.path.join(base_path, cartella, file_nome)
        parole = set()
        try:
            if os.path.exists(full_path):
                with open(full_path, 'r', encoding='utf-8') as f:
                    for line in f:
                        p = line.strip().lower()
                        if p and not p.startswith("#"):
                            parole.add(p)
            else:
                self.nodo.get_logger().warning(f"Vocabolario mancante: {full_path}")
        except Exception as e:
            self.nodo.get_logger().error(f"Errore caricamento {file_nome}: {e}")
        return parole

    def dialogo_scriptato(self, tipo):
        if tipo == "errore_lingua":
            return self.dialoghi["errore_lingua"]["DEFAULT"]
        lingua = self.contesto['ospite'].lingua
        dati = {
            "nome": getattr(self.contesto['ospite'], 'nome', ''),
            "cognome": getattr(self.contesto['ospite'], 'cognome', ''),
            "num_notti": self.contesto.get('num_notti', ''),
            "interesse": self.contesto.get('interesse', '')
        }
        try:
            template = self.dialoghi[tipo].get(lingua, self.dialoghi[tipo].get("EN", ""))
            return template.format(**dati)
        except KeyError:
            return f"[Errore: Dialogo '{tipo}' o lingua '{lingua}' mancante]"

    def rileva_conferma(self, testo):
        lingua = self.contesto['ospite'].lingua
        testo_clean = testo.lower().translate(str.maketrans('', '', string.punctuation))
        parole = set(testo_clean.split())
        vocab_pos = self.vocab_conferme[lingua]['POS']
        vocab_neg = self.vocab_conferme[lingua]['NEG']
        has_pos = not parole.isdisjoint(vocab_pos)
        has_neg = not parole.isdisjoint(vocab_neg)
        if has_pos and not has_neg:
            return True
        if has_neg and not has_pos:
            return False
        return None

    def esegui(self, testo):
        raise NotImplementedError("Ogni InteragisciScenario deve implementare la propria logica.")
