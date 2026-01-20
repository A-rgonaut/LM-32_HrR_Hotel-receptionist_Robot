import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

# --- COMPORTAMENTI ---
from progetto.InteragisciScenarioC import InteragisciScenarioC  # 0
from progetto.RicaricaBatteria import RicaricaBatteria          # 1
from progetto.InteragisciScenarioB import InteragisciScenarioB  # 2
from progetto.InteragisciScenarioA import InteragisciScenarioA  # 3
from progetto.Naviga import Naviga                              # 4
from progetto.Riposo import Riposo                              # 5
# --- ------------- ---

from progetto.SincronizzaManager import SincronizzaManager
from progetto.utils import Persona
from progetto.Specialista import Specialista

import json
import traceback

class Arbitraggio(Node):
    def __init__(self):
        super().__init__('Arbitraggio')

        self.sincro = SincronizzaManager(self)
        self.specialista = Specialista()

        self.cb_group = ReentrantCallbackGroup()

        self.pub = self.create_publisher(String, '/unity/dialogo_robot', 10)
        self.sub = self.create_subscription(String, '/unity/dialogo_umano',
                                            self.processa_input, 10,
                                            callback_group=self.cb_group)
        self.bottone = self.create_subscription(String, '/unity/bottone',
                                                self.gestisci_bottone, 10,
                                                callback_group=self.cb_group)
        self.stato = self.create_publisher(String, '/unity/stato', 10)
        self.batteria = self.create_subscription(Int32, '/unity/battery_state',
                                                 self.gestisci_batteria, 10,
                                                 callback_group=self.cb_group)
        
        self.nome_robot = None
        self.destinazione_target = None
        self.raggiunta_destinazione = True

        self.livello_batteria = 100.0
        self.in_carica = False
        self.bottone_premuto = None
        self.parametri_vitali_sballati = False
        self.ospite_corrente = None

        self.comportamenti = {
            "InteragisciScenarioC": InteragisciScenarioC(self, self.specialista),
            "RicaricaBatteria": RicaricaBatteria(self),
            "InteragisciScenarioB": InteragisciScenarioB(self, self.specialista),
            "InteragisciScenarioA": InteragisciScenarioA(self),
            "Naviga": Naviga(self),
            "Riposo": Riposo(self),
        }

        self.lista_priorita = [
            {
                "nome": "InteragisciScenarioC",
                "oggetto": self.comportamenti["InteragisciScenarioC"],
                "trigger": lambda: (
                    self.bottone_premuto == "InteragisciScenarioC" or
                    self.parametri_vitali_sballati or
                    self.is_running("InteragisciScenarioC")
                )
            },
            {
                "nome": "RicaricaBatteria",
                "oggetto": self.comportamenti["RicaricaBatteria"],
                "trigger": lambda: (
                    self.livello_batteria < 20.0 or
                    (self.in_carica and self.livello_batteria < 100.0)
                )
            },
            {
                "nome": "InteragisciScenarioB",
                "oggetto": self.comportamenti["InteragisciScenarioB"],
                "trigger": lambda: (
                    self.bottone_premuto == "InteragisciScenarioB" or
                    self.is_running("InteragisciScenarioB")
                )
            },
            {
                "nome": "InteragisciScenarioA",
                "oggetto": self.comportamenti["InteragisciScenarioA"],
                "trigger": lambda: (
                    self.bottone_premuto == "InteragisciScenarioA" or
                    self.is_running("InteragisciScenarioA")
                )
            },
            {
                "nome": "Naviga",
                "oggetto": self.comportamenti["Naviga"],
                "trigger": lambda: (
                    self.destinazione_target is not None and
                    not self.raggiunta_destinazione
                )
            },
            {
                "nome": "Riposo",
                "oggetto": self.comportamenti["Riposo"],
                "trigger": lambda: True
            }
        ]

        self.comportamento_attivo = "Riposo"
        self.get_logger().info("Arbitraggio avviato.")

        self.timer = self.create_timer(0.1, self.loop_arbitraggio)

    def loop_arbitraggio(self):
        nuovo_comportamento = None
        nuovo_obj = None
        for item in self.lista_priorita:
            if item["trigger"]():
                nuovo_comportamento = item["nome"]
                nuovo_obj = item["oggetto"]
                break
        if nuovo_comportamento != self.comportamento_attivo:
            if "Scenario" in self.comportamento_attivo:
                self.concludi_scenario_msg(self.comportamento_attivo)
            self.get_logger().info(f"Cambio comportamento: {self.comportamento_attivo} -> {nuovo_comportamento}")
            if "Scenario" in nuovo_comportamento and self.bottone_premuto == nuovo_comportamento:
                nuovo_obj.reset(self.ospite_corrente)
                self.bottone_premuto = None
            else:
                nuovo_obj.reset()
            self.comportamento_attivo = nuovo_comportamento
            msg = String()
            msg.data = json.dumps({"stato_attivo": self.comportamento_attivo})
            self.stato.publish(msg)

    def processa_input(self, msg):
        testo = msg.data.strip()
        if not testo:
            return
        scenario = self.comportamenti.get(self.comportamento_attivo)
        if scenario:
            self.get_logger().info(f"Testo per {self.comportamento_attivo}: {testo}")
            try:
                scenario.esegui(testo)
            except Exception as e:
                self.get_logger().error(f"Eccezione in esegui: {e}\n{traceback.format_exc()}")
        else:
            self.get_logger().error(f"Comportamento '{self.comportamento_attivo}' non trovato!")

    def gestisci_bottone(self, msg):
        self.get_logger().info(f"Ricevuto bottone: {msg.data}")
        try:
            dati = json.loads(msg.data)
            self.ospite_corrente = Persona(int(dati["id"]), dati["nome"], dati["cognome"])
            bottone_premuto_raw = dati["bottone"]
            mappa_bottoni = {
                "Scenario A": "InteragisciScenarioA",
                "Scenario B": "InteragisciScenarioB",
                "Scenario C": "InteragisciScenarioC",
            }
            scenario_target = mappa_bottoni.get(bottone_premuto_raw)
            if not scenario_target:
                self.get_logger().warning(f"Bottone '{bottone_premuto_raw}' non riconosciuto.")
                return
            if scenario_target == "InteragisciScenarioC":
                self.bottone_premuto = "InteragisciScenarioC"
            else:
                if self.comportamento_attivo != scenario_target:
                    self.bottone_premuto = scenario_target
                else:
                    self.get_logger().info(f"Scenario '{scenario_target}' già attivo.")
        except json.JSONDecodeError:
            self.get_logger().error("Errore nel formato JSON ricevuto da Unity.")
        except Exception as e:
            self.get_logger().error(f"Errore in gestisci_bottone: {e}")

    def is_running(self, nome_scenario):
        obj = self.comportamenti.get(nome_scenario)
        if obj and hasattr(obj, 'stato'):
            return obj.stato not in ["FINE", None]
        return False

    def concludi_scenario_msg(self, nome_scenario):
        msg = String()
        msg.data = json.dumps({"comando": "FINE_SCENARIO", "scenario": nome_scenario})
        self.stato.publish(msg)

    def parla(self, testo):
        msg = String()
        if self.nome_robot is None:
            try:
                nomi = self.sincro.interrogaGraphDatabase("MATCH (r:Robot) RETURN TOUPPER(r.nome_robot) AS nomeMaiuscolo")
                self.nome_robot = nomi[0]['nomeMaiuscolo'] if nomi else "ROBOT"
            except:
                self.nome_robot = "ROBOT"
        msg.data = f"{self.nome_robot}: {testo}"
        self.pub.publish(msg)
        self.get_logger().info(msg.data)
        
    def gestisci_batteria(self, msg):
        self.get_logger().info(f"Batteria = {msg.data}%")

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