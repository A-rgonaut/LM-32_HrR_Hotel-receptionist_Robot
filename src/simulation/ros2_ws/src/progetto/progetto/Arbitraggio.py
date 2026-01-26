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
        self.batteria = self.create_subscription(String, '/unity/battery_state',
                                                 self.gestisci_batteria, 10,
                                                 callback_group=self.cb_group)
        self.braccialetti = self.create_subscription(String, '/health_filtered',
                                                self.salva_dati_salute, 10,
                                                callback_group=self.cb_group)

        self.timer = self.create_timer(30.0, self.gestione_periodica_salute)
        self.dati_salute = None

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

    def salva_dati_salute(self, msg):
        self.get_logger().info("salva_dati_salute()")
        try:
            self.dati_salute = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("Errore nel formato JSON ricevuto da BraccialettiManager.")
        except Exception as e:
            self.get_logger().error(f"Errore in salva_dati_salute: {e}")

    def carica_dati(self):
        
        query = """
        UNWIND $batch AS row
        MATCH (o)
        
        // Casting esplicito: l'ID nel JSON è stringa, id(o) è intero
        WHERE id(o) = toInteger(row.id)

        SET o.bpm_attuale = toInteger(row.hr),
            o.pressione_min_attuale = toInteger(row.pmin),
            o.pressione_max_attuale = toInteger(row.pmax)

        // Ritorno i valori per debug
        RETURN id(o) as ID_Interno, o.nome, o.bpm_attuale, o.pressione_min_attuale, o.pressione_max_attuale
        """ 
        
        # Mappiamo la lista Python sul parametro Cypher $batch
        parametri = {
            "batch": self.dati_salute 
        }
                
        # Esecuzione
        self.sincro.interrogaGraphDatabase(query, parametri)
        
        return None
    

    def importa_dati(self):
        # Recuperiamo gli ID di tutti gli Ospiti, delle Soglie globali e delle loro Patologie specifiche.
        # Questo permette a crea_ontologia_istanze di scaricare i nodi completi e le relazioni.
        query = """
        MATCH (o:Ospite)
        RETURN id(o) as node_id
        """
        
        # Esecuzione query
        risultati = self.sincro.interrogaGraphDatabase(query, {})

        if risultati:
            # Creiamo la lista piatta di ID per crea_ontologia_istanze
            lista_ids = [record['node_id'] for record in risultati if record['node_id'] is not None]
            
            self.get_logger().info(f"[Monitoraggio] Recuperati {len(lista_ids)} nodi (Ospiti e Soglie).")
            return lista_ids

        self.get_logger().info("[Monitoraggio] Nessun dato trovato.")
        return []



    def gestione_periodica_salute(self):
        self.get_logger().info("gestione_periodica_salute()")
        # TODO:

        #self.get_logger().info(f"{self.dati_salute}")
        # - Ogni minuto scrivere dati aggiornati dei braccialetti output_data Neo4j!
        
        self.carica_dati()

        # da neo4 j mi devo far riornare le soglie di anomali e di allerta di ciascun Ospite ... sia per i cardiopatici che per i non 
        dati=self.importa_dati()
        if dati:
            self.get_logger().info(f"Iddu è  {dati}")
            self.sincro.crea_ontologia_istanze(dati)
            assiomi = self.sincro.spiegami_tutto(parentClassName="Ospite")
            #stampare sti assiommi
            data = json.loads(assiomi)
            self.get_logger().info(f"Iddu è assiomi  {data}")
            robotLibero=True
            for ospite_json_str, risultati in data.items():
                            # 1. Decodifica dati ospite
                            dati_ospite = json.loads(ospite_json_str)
                            nome_completo = f"{dati_ospite.get('nome', '')} {dati_ospite.get('cognome', '')}".strip()
                            self.get_logger().info(nome_completo)
                            # 2. Itera sui risultati
                            for tipo_assioma, messaggio in risultati.items():
                                
                                # Ci interessa agire solo in base all'assioma "OspiteInStatodiAllerta"
                                if tipo_assioma == "OspiteInStatodiAllerta":
                                    
                                    # Logica richiesta:
                                    # 1. "NON deduce" è presente nel messaggio (quindi non c'è allerta dedotta logicamente)
                                    non_ha_dedotto = "NON deduce" in messaggio
                                    #self.get_logger().info(f"not non_ha_dedotto and robotLibero:  {messaggio}")
                                    # 2. Condizione composta: Nessuna deduzione AND Robot Libero
                                    if not non_ha_dedotto and robotLibero:
                                        self.get_logger().info(f"OspiteInStatodiAllerta:  {messaggio}")
                                        robotLibero=False
                                        #self.ospite_corrente[id]=
                                        #il robot inizia lo scenario C
                                        #cambia tipo
                                    elif not non_ha_dedotto and not robotLibero:
                                        self.get_logger().info(f"not non_ha_dedotto and not robotLibero:{messaggio}")
                                        
                                        #chiama specialista per questa persona con id.. 
                                        #cambia tipo peer questa persona 

                                if tipo_assioma == "OspiteStatoChiamataSpecialista":
                                    non_ha_dedottoSpecialista = "NON deduce" in messaggio
                                    #self.get_logger().info(f"OspiteStatoChiamataSpecialista:  {messaggio}")
                                    if not non_ha_dedottoSpecialista:
                                        self.get_logger().info(f"not non_ha_dedottoSpecialista: {messaggio}")
                                        #chiama specialista per questa persona con id.. 
                                        #cambia tipo peer questa persona 

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
        self.get_logger().info(f"{msg.data}")
        bat = json.loads(msg.data)
        self.get_logger().info(f"{bat['level']}% - {bat['is_charging']}")
        # self.livello_batteria = bat['level']
        # self.in_carica = bat['is_charging'] == "true"

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
