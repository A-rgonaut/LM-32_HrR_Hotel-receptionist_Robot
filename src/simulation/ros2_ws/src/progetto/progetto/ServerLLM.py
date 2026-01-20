import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
from dotenv import load_dotenv
from google import genai
from google.genai import types

load_dotenv()

class ServerLLM(Node):
    def __init__(self):
        super().__init__("ServerLLM")
        self.client = None
        self.init_llm()
        self.sub = self.create_subscription(String, "/llm/request", self.handle_ros_request, 10)
        self.pub = self.create_publisher(String, "/llm/response", 10)
        if self.client:
            self.get_logger().info("ServerLLM avviato.")
        else:
            self.get_logger().info("ServerLLM avviato SENZA chiave API valida!")

    def init_llm(self):
        api_key = os.getenv("GENAI_API_KEY")
        if not api_key:
            self.get_logger().info("ERRORE: Variabile GENAI_API_KEY mancante nel .env")
            return
        try:
            self.client = genai.Client(api_key=api_key)
            # self.client.models.list(config={"page_size": 1})
            self.get_logger().info("Connessione Gemini OK.")
        except Exception as e:
            self.get_logger().info(f"Errore connessione LLM: {e}")
            self.client = None

    def handle_ros_request(self, msg):
        try:
            req = json.loads(msg.data)
            req_id = req.get('id')
            data = req.get('payload', {})
            scenario = data.get('scenario')
            tipo = data.get('tipo')
            user_text = data.get('msg')
            response_text = ""
            success = True
            error_msg = ""
            if not self.client:
                success = False
                error_msg = "LLM Client non inizializzato (Manca API KEY?)"
            else:
                try:
                    response_text = self.process_llm_logic(scenario, tipo, user_text)
                except Exception as e:
                    success = False
                    error_msg = str(e)
                    self.get_logger().info(f"Errore generazione Gemini: {e}")
            res_packet = {
                "id": req_id,
                "success": success,
                "payload": response_text,
                "error": error_msg
            }
            out_msg = String()
            out_msg.data = json.dumps(res_packet)
            self.pub.publish(out_msg)
        except Exception as e:
            self.get_logger().info(f"Errore critico nel nodo LLM: {e}")

    def process_llm_logic(self, scenario, tipo, msg):
        base_folder = os.getenv("SYSTEM_INSTRUCTIONS")
        nome_file = None
        if tipo == "estrazione_semantica":
            if scenario == "A":
                nome_file = "estrazione_semantica_a.md"
            elif scenario == "B":
                # dal testo dobbiamo rilevare il guasto a cosa può essere indirizzato
                # freddo -> allora il problema sta nel condizionatore
                # LLM per prendere la parola
                nome_file = "estrazione_semantica_b.md"
            elif scenario == "C":
                # dal testo dobbiamo rilevare la lista dei sintomi
                nome_file = "estrazione_semantica_c.md"
        elif tipo == "explainability":
            if scenario == "A":
                nome_file = "explainability_a.md"
        elif tipo == "estrazione_temperatura":
            if scenario == "B":
                nome_file = "estrazione_temperatura.md"
        system_instruction = "Sei un assistente utile. Rispondi brevemente."
        if nome_file:
            full_path = os.path.join(base_folder, nome_file)
            try:
                with open(full_path, 'r', encoding='utf-8') as file:
                    content = file.read().strip()
                    if content:
                        system_instruction = content
            except FileNotFoundError:
                self.get_logger().info(f"Attenzione: Il file '{full_path}' non esiste. Uso il default.")
            except Exception as e:
                self.get_logger().info(f"Errore generico nella lettura del file: {e}")
        response = self.client.models.generate_content(
            model="gemini-2.5-flash",
            config=types.GenerateContentConfig(
                system_instruction=system_instruction,
                temperature=0.1,
            ),
            contents=msg,
        )
        # "503 UNAVAILABLE. {'error': {'code': 503, 'message': 'The model is overloaded. Please try again later.', 'status': 'UNAVAILABLE'}}":
        return response.text.strip()

def main(args=None):
    rclpy.init(args=args)
    llm = ServerLLM()
    rclpy.spin(llm)
    llm.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
