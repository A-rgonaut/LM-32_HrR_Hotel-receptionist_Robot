import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from progetto.InteragisciScenarioA import InteragisciScenarioA
# from progetto.InteragisciScenarioB import InteragisciScenarioB
# from progetto.InteragisciScenarioC import InteragisciScenarioC
from progetto.BaseConoscenzaManager import BaseConoscenzaManager
from progetto.LLMManager import LLMManager
from progetto.utils import Ospite
import json

class Arbitraggio(Node):
    def __init__(self):
        super().__init__('Arbitraggio')

        self.kb = BaseConoscenzaManager()
        self.llm = LLMManager()

        self.pub = self.create_publisher(
            String,
            'dialogo_robot',
            10
        )

        self.sub = self.create_subscription(
            String,
            'dialogo_umano',
            self.processa_input,
            10
        )

        self.bottone = self.create_subscription(
            String,
            'bottone',
            self.arbitra,
            10
        )

        self.comportamenti = {
            #"Riposo": Riposo(self),
            "InteragisciScenarioA": InteragisciScenarioA(self),
            #"InteragisciScenarioB": InteragisciScenarioB(self),
            #"InteragisciScenarioC": InteragisciScenarioC(self),
            #"InteragisciConPersonale": InteragisciConPersonale(self)
        }

        self.comportamento_attivo = "Riposo"

        self.get_logger().info("Arbitraggio avviato.")

    def processa_input(self, msg):
        testo = msg.data.strip()
        if not testo:
            return
        if self.comportamento_attivo == "Riposo":
            self.parla("Sono a riposo. Seleziona uno scenario su Unity.")
            return
        scenario = self.comportamenti.get(self.comportamento_attivo)
        if scenario:
            self.get_logger().info(f" {testo}")
            scenario.esegui(testo, self.kb, self.llm)

    def arbitra(self, msg):
        self.get_logger().info(msg.data)
        try:
            dati = json.loads(msg.data)
            ospite = Ospite(int(dati["id"]), dati["nome"], dati["cognome"], 30, None)
            bottone_premuto = dati["bottone"]
            mappa_bottoni = {
                "Scenario A": "InteragisciScenarioA",
                # "b": "InteragisciScenarioB",
                # "c": "InteragisciScenarioC",
            }
            scenario = mappa_bottoni.get(bottone_premuto)
            if not scenario:
                self.get_logger().warning(f"Bottone '{bottone_premuto}' non riconosciuto.")
                return
            if scenario == 'InteragisciScenarioC':
                if self.comportamento_attivo != "InteragisciScenarioC":
                    self.attiva_scenario("InteragisciScenarioC", ospite)
                return
            if self.comportamento_attivo == "InteragisciScenarioC":
                return
            if self.comportamento_attivo != scenario:
                self.attiva_scenario(scenario, ospite)
            else:
                self.get_logger().info(f"Scenario '{scenario}' gia attivo.")
        except json.JSONDecodeError:
            self.get_logger().error("Errore nel formato JSON ricevuto da Unity.")
        except Exception as e:
            self.get_logger().error(f"Errore in arbitra: {e}")

    def attiva_scenario(self, nome, ospite):
        self.comportamento_attivo = nome
        scenario = self.comportamenti[nome]
        scenario.reset(ospite)
        self.get_logger().info("Scenario resettato. In attesa che l'utente parli (Ciao/Hello)...")

    def parla(self, testo):
        msg = String()
        msg.data = testo
        self.pub.publish(msg)
        self.get_logger().info(f"Pippor: {testo}")

def main(args=None):
    rclpy.init(args=args)
    arbitraggio = Arbitraggio()
    try:
        rclpy.spin(arbitraggio)
    except KeyboardInterrupt:
        arbitraggio.get_logger().info("Interruzione Arbitraggio...")
    finally:
        if hasattr(arbitraggio, 'kb'):
            arbitraggio.kb.close()
        arbitraggio.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
