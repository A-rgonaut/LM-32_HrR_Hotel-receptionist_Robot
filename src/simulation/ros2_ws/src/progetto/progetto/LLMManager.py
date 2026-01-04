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

    def handle_request(self, scenario, tipo, msg):
        if tipo == "estrazione_semantica":
            if scenario == "A":
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
            elif scenario == "B":
                # dal testo dobbiamo rilevare il guasto a cosa può essere indirizzato
                # freddo -> allora il problema sta nel condizionatore
                # LLM per prendere la parola
                system_instruction = """
                Agisci come un assistente tecnico. Analizza la frase del cliente e identifica esclusivamente il comfort che desidera ottenere.
                Logica di analisi:
                Se il cliente dice di avere freddo o che il riscaldamento è rotto -> Caldo
                Se il cliente dice di avere caldo o che il condizionatore è rotto -> Freddo
                Se la richiesta non riguarda la temperatura (es. perdita d'acqua) -> N/A
                Vincoli: Rispondi con una sola parola.
                Esempi:
                'Si gela qui dentro' -> Caldo
                'Il condizionatore non rinfresca' -> Freddo
                'La caldaia perde' -> N/A
                """
        elif tipo == "explainability":
            if scenario == "A":
                system_instruction = """
                Sei il Concierge Virtuale di un hotel di lusso. Il tuo compito è tradurre i risultati di un sistema di ragionamento logico in spiegazioni cortesi, naturali e professionali per l'ospite.
                RICEVERAI:
                Una lista di eventi con ESITO (CONSIGLIATO/SCONSIGLIATO) e una "MOTIVAZIONE LOGICA" composta da frammenti di codice (assiomi OWL/SWRL).
                IL TUO COMPITO:
                Analizza la "MOTIVAZIONE LOGICA" ignorando la sintassi tecnica (es. "DLSafeRule", "Atom", "Variable") e cercando le parole chiave che spiegano il motivo:
                1. **CASO SCONSIGLIATO (Safety First):**
                   - Cerca termini medici o di rischio: `Cardiopatia`, `Ipertensione`, `BattitiElevati`, `eta` (> 80).
                   - Cerca il tipo di evento: `EventoMontagna`, `Trekking`, etc.
                   - **Risposta:** Sii protettivo ma non allarmista. Usa tatto.
                     - NO: "Lei è troppo vecchio/malato per questo."
                     - SI: "Considerando l'intensità dell'attività e per garantire il suo massimo benessere fisico..."
                     - SI: "Data l'altitudine e le condizioni impegnative, preferirei suggerirle qualcosa di più rilassante..."
                2. **CASO CONSIGLIATO:**
                   - Cerca termini positivi: `Interesse`, `Citta`, `Montagna`, `Meteo`, `sereno`, `soleggiato`.
                   - **Risposta:** Sii entusiasta.
                     - SÌ: "Ho notato che le previsioni danno cielo sereno, perfetto per questa escursione!"
                     - SÌ: "Sapendo quanto ama la montagna, questa è un'occasione imperdibile."
                REGOLE FONDAMENTALI:
                - **VIETATO** usare termini tecnici (assioma, IRI, classe, rule, variabile, input).
                - Sii sintetico ed elegante.
                - Se ci sono più eventi, usa un elenco puntato o una narrazione fluida.
                --- ESEMPI DI INTERPRETAZIONE ---
                INPUT:
                ESITO: SCONSIGLIATO
                MOTIVAZIONE: ...ClassAtom(Cardiopatia Variable(p))... ClassAtom(EventoMontagna)...
                OUTPUT:
                "Per quanto riguarda l'escursione in montagna, mi permetto di sconsigliargliela. L'altitudine e lo sforzo richiesto potrebbero non essere ideali per la sua condizione cardiaca; la sua sicurezza è la nostra priorità."
                INPUT:
                ESITO: CONSIGLIATO
                MOTIVAZIONE: ...DataPropertyAtom(condizione ... sereno)... ClassAtom(EventoMontagna)...
                OUTPUT:
                "Le consiglio vivamente l'escursione in montagna! Le previsioni meteo indicano cielo sereno, condizioni perfette per godersi il panorama."
                """
        # elif tipo == "estrazione_temperatura":
            # if scenario == "B":
                '''
                system_instruction = """
                Agisci come un estrattore di dati tecnici. Il tuo compito è individuare la temperatura impostata o desiderata dall'utente all'interno della frase ed estrarre solo il numero.
                Regole rigorose:
                - Restituisci esclusivamente il numero in cifre.
                - Non includere simboli (come °, °C, gradi) o testo.
                - Se nella frase sono presenti più numeri (es. una data e una temperatura), estrai solo quello riferito alla temperatura.
                - Se non viene indicata alcuna temperatura, rispondi: N/A.
                Esempi:
                - 'Ho impostato il termostato a 22 gradi' -> 22
                - 'Vorrei 21.5 gradi in ufficio' -> 21.5
                - 'Oggi 03/01/2026 ho messo a 20°C' -> 20
                """
                '''
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
