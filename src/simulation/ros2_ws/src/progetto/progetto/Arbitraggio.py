import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

# --- COMPORTAMENTI ---
from progetto.InteragisciScenarioC import InteragisciScenarioC  # 0
from progetto.RicaricaBatteria import RicaricaBatteria          # 1
from progetto.InteragisciScenarioB import InteragisciScenarioB  # 2
from progetto.InteragisciScenarioA import InteragisciScenarioA  # 3
# from progetto.Naviga import Naviga                              # 4
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

        self.cb_group = ReentrantCallbackGroup()
        self.pub = self.create_publisher(String, '/unity/dialogo_robot', 10)
        self.sub = self.create_subscription(String, '/unity/dialogo_umano',
                                            self.processa_input, 10,
                                            callback_group=self.cb_group)
        self.bottone = self.create_subscription(String, '/unity/bottone',
                                                self.arbitra, 10,
                                                callback_group=self.cb_group)
        self.stato = self.create_publisher(String, '/unity/stato', 10)
        """
        self.bottone_premuto           = None
        self.parametri_vitali_sballati = False
            /unity/health_filtered

        self.comportamenti_objects = {}

        self.livello_batteria = 100.0
        self.in_carica        = False

        self.destinazione_target    = None
        self.raggiunta_destinazione = True

        self.comportamenti = [  # Assumiamo che ogni oggetto scenario abbia una proprietà interna `.is_running` che diventa `True` quando inizia e `False` quando finisce.
            {
                "nome": "InteragisciScenarioC",
                "oggetto": InteragisciScenarioC(self),
                "trigger": lambda: (
                    self.bottone_premuto == "C" or
                    self.parametri_vitali_sballati or
                    self.comportamenti_objects["InteragisciScenarioC"].is_running  # Scenario in corso (non finito).
                )
            },
            {
                "nome": "RicaricaBatteria",
                "oggetto": RicaricaBatteria(self),
                "trigger": lambda: (
                    self.livello_batteria < 20.0 or
                    (self.in_carica and self.livello_batteria < 100.0)  # Isteresi
                )
            },
            {
                "nome": "InteragisciScenarioB",
                "oggetto": InteragisciScenarioB(self),
                "trigger": lambda: (
                    self.bottone_premuto == "B" or
                    self.comportamenti_objects["InteragisciScenarioB"].is_running  # Include all'interno la logica di dire al livello navigazione "Vai in camera X".
                )
            },
    # Nota: Questi scenari NON guidano i motori per spostarsi tra le stanze.
    # Si limitano a impostare `self.destinazione_target` e gestire il dialogo.
            {
                "nome": "InteragisciScenarioA",
                "oggetto": InteragisciScenarioA(self),
                "trigger": lambda: (
                    self.bottone_premuto == "A" or
                    self.comportamenti_objects["InteragisciScenarioA"].is_running
                )
            },
            {
                "nome": "Navigazione",  # Include Pianifica e Localizza
                "oggetto": Navigazione(self),      # Questo layer si attiva se qualcuno sopra ha impostato una `self.destinazione_target`
                "trigger": lambda: (                      # ma il robot non è ancora arrivato.
                    self.destinazione_target is not None and
                    not self.raggiunta_destinazione
                )
            },
            # Qui dentro fai A*, SLAM locale e gestione ostacoli.
            # Se uno Scenario è attivo (es. B), esso setta la destinazione e poi "cede"
            # il controllo motorio a questo layer, ma siccome B è priorità più alta,
            # B deve essere progettato per dire "Sto aspettando di arrivare", lasciando passare
            # il controllo a questo layer,
            {
                "nome": "Riposo",
                "oggetto": Riposo(self),
                "trigger": lambda: True
            }
        ]
        """

        self.specialista = Specialista()

        self.comportamenti = {
            "InteragisciScenarioA": InteragisciScenarioA(self),
            "InteragisciScenarioB": InteragisciScenarioB(self, self.specialista),
            "RicaricaBatteria": RicaricaBatteria(self),
            # "Pianifica": Pianifica(self),
            # "Localizza": Localizza(self),
            "InteragisciScenarioC": InteragisciScenarioC(self, self.specialista),
            "Riposo": Riposo(self),
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
                scenario.esegui(testo)
                stato_corrente = getattr(scenario, "stato", None)
                if stato_corrente == "FINE":
                    self.concludi_scenario()
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
                "Scenario B": "InteragisciScenarioB",
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

    def concludi_scenario(self):
        self.get_logger().info(f"Scenario {self.comportamento_attivo} concluso.")
        msg = String()
        msg.data = json.dumps({"comando": "FINE_SCENARIO", "scenario": self.comportamento_attivo})
        self.stato.publish(msg)
        self.comportamento_attivo = "Riposo"

    def parla(self, testo):
        msg = String()
        if self.nome_robot is None:
            nomi = self.sincro.interrogaGraphDatabase("MATCH (r:Robot) RETURN TOUPPER(r.nome_robot) AS nomeMaiuscolo")
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
