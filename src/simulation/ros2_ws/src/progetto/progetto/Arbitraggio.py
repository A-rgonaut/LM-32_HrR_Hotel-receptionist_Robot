import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

# --- COMPORTAMENTI ---
from progetto.InteragisciScenarioA import InteragisciScenarioA
# from progetto.InteragisciScenarioB import InteragisciScenarioB
from progetto.InteragisciScenarioC import InteragisciScenarioC
# from progetto.Localizza import Localizza
# from progetto.Pianifica import Pianifica
from progetto.RicaricaBatteria import RicaricaBatteria
from progetto.Riposo import Riposo
# --- ------------- ---

from progetto.SincronizzaManager import SincronizzaManager
from progetto.utils import Persona

import json
import traceback

class Arbitraggio(Node):
    def __init__(self):
        super().__init__('Arbitraggio')

        self.sincro = SincronizzaManager(self)

        self.cb_group = ReentrantCallbackGroup()
        self.pub = self.create_publisher(String, '/unity/dialogo_robot', 10)
        self.sub = self.create_subscription(String, '/unity/dialogo_umano',
            self.processa_input, 10, callback_group=self.cb_group)
        self.bottone = self.create_subscription(String, '/unity/bottone',
            self.arbitra, 10, callback_group=self.cb_group)

        self.comportamenti = {
            "InteragisciScenarioA": InteragisciScenarioA(self),
            #"InteragisciScenarioB": InteragisciScenarioB(self),
            "InteragisciScenarioC": InteragisciScenarioC(self),
            #"InteragisciConPersonale": InteragisciConPersonale(self),
            "Riposo": Riposo(self),
            "RicaricaBatteria": RicaricaBatteria(self)
        }

        self.comportamento_attivo = "Riposo"

        self.nome_robot = None

        self.get_logger().info("Arbitraggio avviato.")

    def processa_input(self, msg):
        testo = msg.data.strip()
        if not testo:
            return
        scenario = self.comportamenti.get(self.comportamento_attivo)
        if scenario:
            self.get_logger().info(f"Testo: {testo}")
            try:
                scenario.esegui(testo, self.sincro)
            except Exception as e:
                self.get_logger().error(
                    f"Eccezione in {self.comportamento_attivo}.esegui: {e}\n{traceback.format_exc()}"
                )
        else:
            self.get_logger().error(f"Comportamento '{self.comportamento_attivo}' non trovato!")

    def arbitra(self, msg):
        self.get_logger().info(msg.data)
        try:
            dati = json.loads(msg.data)
            ospite = Persona(int(dati["id"]), dati["nome"], dati["cognome"])
            bottone_premuto = dati["bottone"]
            mappa_bottoni = {
                "Scenario A": "InteragisciScenarioA",
                # "Scenario B": "InteragisciScenarioB",
                "Scenario C": "InteragisciScenarioC",
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
        if self.nome_robot is None:
            nomi = self.sincro.interrogaGraphDatabase("MATCH (r:Robot) RETURN TOUPPER(r.nome) AS nomeMaiuscolo")
            # Al momento abbiamo un solo robot:
            self.get_logger().info(f"Recuperato nome_robot: {nomi[0]['nomeMaiuscolo']}")
            self.nome_robot = nomi[0]['nomeMaiuscolo']
        msg.data = f"{self.nome_robot}: {testo}"
        self.pub.publish(msg)
        self.get_logger().info(msg.data)

def main(args=None):
    rclpy.init(args=args)
    arbitraggio = Arbitraggio()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(arbitraggio)
    try:
        executor.spin()
    except KeyboardInterrupt:
        arbitraggio.get_logger().info("Interruzione Arbitraggio...")
    finally:
        arbitraggio.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
