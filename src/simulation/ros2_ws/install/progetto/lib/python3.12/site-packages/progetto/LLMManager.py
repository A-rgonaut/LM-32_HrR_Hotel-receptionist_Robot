import os
from dotenv import load_dotenv
from google import genai
from google.genai import types

load_dotenv()

class LLMManager():
    def __init__(self):
        client = None
        self.init_llm()

    def init_llm(self):
        api_key = os.getenv("GENAI_API_KEY")
        if not api_key:
            print("Variabile GENAI_API_KEY mancante nel .env")
            return
        try:
            client = genai.Client(api_key=api_key)
            print(f"LLM OK")
        except Exception as e:
            print(f"Errore LLM: {e}")
            self.client = None

    def handle_request(self, tipo, msg):
        if tipo == "estrazione_semantica":
            system_instruction = """Sei un assistente che fa estrazione
                semantica. Restituisci solo una parola che rappresenta
                l'interesse principale."""
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