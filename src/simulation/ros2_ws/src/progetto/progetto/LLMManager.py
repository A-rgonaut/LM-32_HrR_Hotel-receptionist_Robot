import os
from dotenv import load_dotenv
from google import genai
from google.genai import types

load_dotenv()

class LLMManager():
    def __init__(self):
        self.client = None
        self.init_llm()

    def init_llm(self):
        api_key = os.getenv("GENAI_API_KEY")
        if not api_key:
            print("Variabile GENAI_API_KEY mancante nel .env")
            return
        try:
            self.client = genai.Client(api_key=api_key)
            if self.client:
                self.client.models.list(config={"page_size": 1})
                print(f"LLM OK")
        except Exception as e:
            print(f"Errore LLM: {e}")
            self.client = None

    def handle_request(self, tipo, msg):
        if tipo == "estrazione_semantica":
            system_instruction = """
            Agisci come un estrattore di entità per un Knowledge Graph turistico.
            Il tuo obiettivo è identificare l'interesse POSITIVO dell'utente e ridurlo a una singola parola chiave (sostantivo singolare).
            Regole rigorose:
            1. Estrai solo l'interesse principale espresso positivamente (ignora ciò che l'utente dice di odiare).
            2. Riduci il concetto al suo lemma (sostantivo singolare in italiano).
            3. Generalizza leggermente se l'attività è troppo specifica (es. "bere Chianti" -> "vino").
            4. NON fornire frasi, solo UNA parola.
            5. NON usare punteggiatura.
            Esempi:
            - "Adoro fare lunghe camminate nei boschi" -> "trekking"
            - "Vorrei assaggiare prodotti locali" -> "enogastronomia"
            - "Odio il mare, preferisco le chiese" -> "cultura"
            - "Mi piace nuotare" -> "mare"
            """
        elif tipo == "explainability":
            system_instruction = """
            Sei il Concierge Virtuale di un hotel di lusso. Il tuo compito è tradurre la logica formale del sistema in una spiegazione cortese e naturale per l'ospite.
            Riceverai una lista di ASSIOMI LOGICI (OWL/SWRL) che descrivono perché un evento è stato SCELTO oppure SCARTATO/SCONSIGLIATO.
            IL TUO COMPITO:
            1. Analizza gli assiomi per capire lo scenario:
               - CASO POSITIVO (es. "EventoSuggerito"): C'è match tra interessi e l'evento.
               - CASO NEGATIVO (es. "EventoSconsigliato", "ConflittoMedico"): C'è una patologia o un vincolo che impedisce l'evento.
            2. Genera la risposta:
               - Se POSITIVO: Sii propositivo ed entusiasta. "Visti i suoi interessi per X, le consiglio Y..."
               - Se NEGATIVO: Sii protettivo, empatico e usa molto tatto. Non dire "Sei malato", ma "Per tutelare il suo benessere..." o "Data l'intensità dell'attività...".
            VIETATO:
            - Usare termini tecnici (assioma, classe, istanza, IRI, object property, reasoner).
            - Inventare dettagli non presenti (prezzi, orari) se non forniti.
            Esempio Input POSITIVO:
            ObjectPropertyAssertion(:haInteresse Peppe :Enogastronomia)
            ClassAssertion(:EventoCitta Degustazione)
            --> Output: "Signor Peppe, sapendo quanto apprezza l'enogastronomia, ho selezionato per lei la Degustazione in centro. È perfetta per i suoi gusti."
            Esempio Input NEGATIVO (Medico):
            ClassAssertion(:Cardiopatia Peppe)
            ClassAssertion(:EventoMontagna Trekking_Estremo)
            ClassAssertion(:EventoSconsigliato Trekking_Estremo)
            --> Output: "Ho valutato il Trekking Estremo, ma per garantire la sua massima sicurezza e salute, mi sento di sconsigliarle questa attività troppo intensa."
            """
        try:
            response = self.client.models.generate_content(
                model="gemini-2.5-flash",
                config=types.GenerateContentConfig(
                    system_instruction=system_instruction,
                    temperature=0.1,
                ),
                contents=msg,
            )
            return response.text.strip()
        except Exception as e:
            print(f"Errore chiamata Gemini: {e}")
            return None
