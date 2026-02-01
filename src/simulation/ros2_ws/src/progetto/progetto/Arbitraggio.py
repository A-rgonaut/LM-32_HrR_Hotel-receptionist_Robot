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
from progetto.utils import Persona, Ospite
from progetto.Specialista import Specialista
from progetto.Emergenza import Emergenza

import json
import traceback

class Arbitraggio(Node):
    def __init__(self):
        super().__init__('Arbitraggio')

        self.sincro = SincronizzaManager(self)

        self.specialista = Specialista()
        self.emergenza = Emergenza(self)
        self.spiegazioneC = ""  # Inizializzalo come stringa vuota

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

        self.timer_salute = self.create_timer(60.0, self.gestione_periodica_salute, callback_group=self.cb_group)
        self.dati_salute = None

        self.nome_robot = None
        self.destinazione_target = None
        self.raggiunta_destinazione = True

        #self.livello_batteria = 100.0
        self.in_carica = False
        self.bottone_premuto = None
        self.parametri_vitali_sballati = False
        self.ospite_corrente = None

        self.comportamenti = {
            "InteragisciScenarioC": InteragisciScenarioC(self, self.specialista, self.spiegazioneC),
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
                    self.parametri_vitali_sballati
                )
            },
            {
                "nome": "RicaricaBatteria",
                "oggetto": self.comportamenti["RicaricaBatteria"],
                "trigger": lambda: (
                    self.livello_batteria < -10.0  and  self.comportamento_attivo in ["InteragisciScenarioA", "InteragisciScenarioB"]#or
                    #(self.in_carica and self.livello_batteria < 100.0)
                )
            },
            {
                "nome": "InteragisciScenarioB",
                "oggetto": self.comportamenti["InteragisciScenarioB"],
                "trigger": lambda: self.bottone_premuto == "InteragisciScenarioB"
            },
            {
                "nome": "InteragisciScenarioA",
                "oggetto": self.comportamenti["InteragisciScenarioA"],
                "trigger": lambda: self.bottone_premuto == "InteragisciScenarioA"
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

        self.timer_arbitraggio = self.create_timer(0.1, self.loop_arbitraggio, callback_group=self.cb_group)

    def loop_arbitraggio(self):
        attivo = self.comportamento_attivo
        obj_attivo = self.comportamenti.get(attivo)
        # SE LO SCENARIO HA FINITO, LIBERA IL ROBOT
        if obj_attivo and hasattr(obj_attivo, "stato"):
            if obj_attivo.stato == "FINE":
                self.get_logger().info(f"[Arbitraggio] Scenario {attivo} terminato, ritorno a Riposo")
                self.concludi_scenario_msg(attivo)
                self.comportamento_attivo = "Riposo"
                self.comportamenti["Riposo"].reset()
                return

        candidato = None
        obj = None

        # --- 1. SELEZIONE CANDIDATO TRAMITE lista_priorita (ESCLUSO RIPOSO) ---
        for item in self.lista_priorita:
            if item["nome"] == "Riposo":
                continue  # Riposo è fallback, non compete
            if item["trigger"]():
                candidato = item["nome"]
                obj = item["oggetto"]
                break

        # --- 2. FALLBACK ---
        if candidato is None:
            candidato = "Riposo"
            obj = self.comportamenti["Riposo"]

        """
        # --- 3. BLOCCO SCENARI (Solo C può interrompere A/B, ma C deve persistere) ---
        if attivo in ("InteragisciScenarioA", "InteragisciScenarioB", "InteragisciScenarioC"):
            # Se l'attuale è C, resta in C finché non finisce
            # Se l'attuale è A o B, resta lì a meno che non arrivi un C
            if attivo == "InteragisciScenarioC" or candidato != "InteragisciScenarioC":
                candidato = attivo
                obj = self.comportamenti[attivo]
                
                # Consuma bottoni pendenti per non triggerare cambi indesiderati dopo
                if self.bottone_premuto in ("InteragisciScenarioA", "InteragisciScenarioB", "InteragisciScenarioC"):
                    self.bottone_premuto = None
        """
        if attivo in ("InteragisciScenarioA", "InteragisciScenarioB", "InteragisciScenarioC"):
            # C è assolutamente prioritario e non interrompibile
            if attivo == "InteragisciScenarioC":
                candidato = attivo
                obj = self.comportamenti[attivo]
            
            # Se siamo in A o B, permettiamo l'interruzione SOLO se il candidato è C o RicaricaBatteria
            elif candidato not in ("InteragisciScenarioC", "RicaricaBatteria"):
                candidato = attivo
                obj = self.comportamenti[attivo]

            # Consumo dei bottoni pendenti per evitare trigger sporchi
            if self.bottone_premuto in ("InteragisciScenarioA", "InteragisciScenarioB", "InteragisciScenarioC"):
                self.bottone_premuto = None
                
        # --- 4. TRANSIZIONE ---
        if candidato != attivo:
            # chiusura scenario precedente
            if "Scenario" in attivo:
                self.concludi_scenario_msg(attivo)

            self.get_logger().info(
                f"Cambio comportamento: {attivo} -> {candidato}"
            )
            self.comportamento_attivo = candidato

            # notifica Unity
            msg = String()
            msg.data = json.dumps({"stato_attivo": candidato})
            self.stato.publish(msg)

            # reset comportamento
            if candidato == "InteragisciScenarioC":
                obj.spiegazione = self.spiegazioneC
                obj.reset(self.ospite_corrente)
                self.parametri_vitali_sballati = False

            elif "Scenario" in candidato:
                obj.reset(self.ospite_corrente)

            else:
                obj.reset()

            # bottone consumato SOLO se ha causato transizione
            self.bottone_premuto = None


    def salva_dati_salute(self, msg):
        # self.get_logger().info("salva_dati_salute()")
        try:
            self.dati_salute = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("Errore nel formato JSON ricevuto da BraccialettiManager.")
        except Exception as e:
            self.get_logger().error(f"Errore in salva_dati_salute: {e}")

    def gestione_periodica_salute(self):
        #self.get_logger().info("gestione_periodica_salute()")
        # self.get_logger().info(f"{self.dati_salute}")
        # - Ogni minuto scrivere dati aggiornati dei braccialetti output_data Neo4j!
        self.get_logger().info(f"{self.emergenza.carica_dati()}")
        # da neo4j mi devo far riornare le soglie di anomali e di allerta di ciascun Ospite ... sia per i cardiopatici che per i non
        dati=self.emergenza.importa_dati()
        if dati:
            self.get_logger().info(f"Dati dei soli ospiti :    {dati}")
            self.sincro.crea_ontologia_istanze(dati, braccialetti=True)
            assiomi = self.sincro.spiegami_tutto(parentClassName="Ospite", braccialetti=True)
            # stampare sti assiommi
            data = json.loads(assiomi)
            self.get_logger().info(f"assiomi per vedere in che stato sono le persone  {data}")
            """
            robotLibero = [x["nome"] for x in self.lista_priorita].index(self.comportamento_attivo) >= 2
            robotLibero = self.comportamento_attivo in ["InteragisciScenarioB", "InteragisciScenarioA", "Naviga", "Riposo"]
            """
            #robotLibero = [x["nome"] for x in self.lista_priorita].index(self.comportamento_attivo) >= 2
            for ospite_json_str, risultati in data.items():
                # 1. Decodifica dati ospite
                dati_ospite = json.loads(ospite_json_str)
                nome=dati_ospite.get('nome', '')
                cognome=dati_ospite.get('cognome', '')
                nome_completo = f"{dati_ospite.get('nome', '')} {dati_ospite.get('cognome', '')}".strip()
                ospite_da_monitorare = Ospite(dati_ospite.get('id'), nome, cognome, "2000", "IT")
                self.get_logger().info(f" Persone che sto analizzando {nome_completo}")
                # 2. Itera sui risultati
                for tipo_assioma, messaggio in risultati.items():
                    # Ci interessa agire solo in base all'assioma "OspiteInStatodiAllerta"
                    robotLibero = self.comportamento_attivo in ["InteragisciScenarioB", "InteragisciScenarioA", "Naviga", "Riposo"]
                    self.get_logger().info(f"robot:Libero  {robotLibero}   per  {tipo_assioma}")

                    if tipo_assioma == "OspiteInStatodiAllerta":
                        # Logica richiesta:
                        # 1. "NON deduce" è presente nel messaggio (quindi non c'è allerta dedotta logicamente)
                        non_ha_dedotto = "NON deduce" in messaggio
                        #self.get_logger().info(f"not non_ha_dedotto and robotLibero:  {messaggio}")
                        # 2. Condizione composta: Nessuna deduzione AND Robot Libero
                        if not non_ha_dedotto and robotLibero:
                            self.get_logger().info(f"OspiteInStatodiAllerta:  {messaggio}") # queryTODO ,  salvo il cambio label
                            #robotLibero=False
                            self.ospite_corrente=Ospite(self.emergenza.cambia_stato_allerta( nome,cognome), nome, cognome, "2000", "IT")
                            self.get_logger().info(f"ospite_corrente  { self.ospite_corrente}")
                            self.spiegazioneC = messaggio
                            self.parametri_vitali_sballati = True  # il robot inizia lo scenario C
                            
                        elif not non_ha_dedotto and not robotLibero:
                            self.get_logger().info(f"not non_ha_dedotto and not robotLibero:{messaggio}")
                            self.emergenza.cambia_stato_spe(nome,cognome)
                            self.spiegazioneC=messaggio
                            self.specialista.chiama(self, "medico",ospite_da_monitorare, " Il Robot è impegnato in un'emergenza e non può andare dall'ospite in Allerta") # queryTODO ,  salvo la chiamata allo specialista
                            
                    if tipo_assioma == "OspiteStatoChiamataSpecialista":
                        non_ha_dedottoSpecialista = "NON deduce" in messaggio
                        #self.get_logger().info(f"OspiteStatoChiamataSpecialista:  {messaggio}")
                        if not non_ha_dedottoSpecialista:
                            self.get_logger().info(f"not non_ha_dedottoSpecialista: {messaggio}")
                            #chiama specialista per questa persona con id..
                            self.emergenza.cambia_stato_spe(nome,cognome)
                            self.specialista.chiama(self, "medico",ospite_da_monitorare, " l'ospite è in chiamataSpecialista") # queryTODO ,  salvo la chiamata allo specialista

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

            # aggiorna ospite SEMPRE (serve a C e al reset iniziale)
            self.ospite_corrente = Persona(
                int(dati["id"]),
                dati["nome"],
                dati["cognome"]
            )

            mappa_bottoni = {
                "Scenario A": "InteragisciScenarioA",
                "Scenario B": "InteragisciScenarioB",
                "Scenario C": "InteragisciScenarioC",
            }

            scenario_target = mappa_bottoni.get(dati.get("bottone"))
            if not scenario_target:
                self.get_logger().warning("Bottone non riconosciuto.")
                return

            # REGOLA CHIAVE:
            # A/B non possono preemptare A/B
            if (
                scenario_target in ["InteragisciScenarioA", "InteragisciScenarioB"]
                and self.comportamento_attivo in ["InteragisciScenarioA", "InteragisciScenarioB"]
            ):
                self.get_logger().info(
                    f"Ignorato bottone {scenario_target}: "
                    f"{self.comportamento_attivo} già attivo"
                )
                return

            # C può sempre passare
            self.bottone_premuto = scenario_target

        except json.JSONDecodeError:
            self.get_logger().error("Errore JSON da Unity.")
        except Exception as e:
            self.get_logger().error(f"Errore in gestisci_bottone: {e}")

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
        try:
            bat = json.loads(msg.data)
            livello_unity = float(bat.get('level', 0.0))
            is_charging_unity=False
            is_charging_unity = (str(bat.get('is_charging')).lower() == "true")
            # FIX PER TESTING:
            # Se la mia simulazione interna dice che sono al 100%, ignoro Unity
            # se Unity mi dice che sono scarico (altrimenti tornerei subito in ricarica).
            #if self.livello_batteria == 100.0 and livello_unity < 90.0:
            #    return
            # Altrimenti aggiorno normalmente
            self.livello_batteria = livello_unity
            self.in_carica = is_charging_unity
        except Exception as e:
            self.get_logger().error(f"Errore lettura batteria: {e}")
    """

    def gestisci_batteria(self, msg):
        try:
            # Decodifica il JSON inviato da Unity
            bat = json.loads(msg.data)
            
            # 1. Aggiorna il livello reale della batteria
            self.livello_batteria = float(bat.get('level', 0.0))
            
            # 2. Aggiorna lo stato di connessione elettrica
            # Trasforma la stringa "true"/"false" in un booleano Python
            self.in_carica = (str(bat.get('is_charging')).lower() == "true")

        except Exception as e:
            self.get_logger().error(f"Errore lettura batteria da Unity: {e}")
    """

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
