import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
from dotenv import load_dotenv
from owlready2 import get_ontology, AnnotationProperty
from datetime import datetime
import subprocess

load_dotenv()

class ServerOntologia(Node):
    def __init__(self):
        super().__init__("ServerOntologia")
        self.ontology = None
        self.init_ontology()
        self.sub = self.create_subscription(String, "/onto/request", self.handle_request, 10)
        self.pub = self.create_publisher(String, "/onto/response", 10)
        self.get_logger().info("ServerOntologia avviato.")

    def init_ontology(self):
        path = os.getenv("PATH_ONTO")
        if not path:
            self.get_logger().info("ERRORE: PATH_ONTO mancante nel .env")
            return
        try:
            self.ontology = get_ontology(path).load()
            self.get_logger().info(f"Ontologia caricata: {path}")
            skos = self.ontology.get_namespace("http://www.w3.org/2004/02/skos/core#")
            with self.ontology:
                if not self.ontology["altLabel"]:
                    class altLabel(AnnotationProperty):
                        namespace = skos
        except Exception as e:
            self.get_logger().info(f"Errore ontologia: {e}")

    def handle_request(self, msg):
        try:
            req = json.loads(msg.data)
            req_id = req.get('id')
            action = req.get('payload', {}).get('action')
            data = req.get('payload')
            response_payload = None
            success = True
            error_msg = ""
            if action == "verifica_sottoclasse":
                response_payload = self.logic_verifica_sottoclasse(
                    data.get("nome_figlio"),
                    data.get("nome_genitore")
                )

            elif action == "trova_classe_da_sinonimo":
                response_payload = self.logic_trova_classe(
                    data.get("testo"),
                    data.get("nome_radice")
                )
            elif action == "ragiona_su_eventi":
                response_payload = self.logic_ragionamento_completo(
                    data.get("dati_ospite"),
                    data.get("eventi")
                )
            else:
                success = False
                error_msg = f"Azione sconosciuta: {action}"
            res = {
                "id": req_id,
                "success": success,
                "payload": response_payload,
                "error": error_msg
            }
            res_msg = String()
            res_msg.data = json.dumps(res)
            self.pub.publish(res_msg)
        except Exception as e:
            self.get_logger().info(f"Errore processing richiesta: {e}")

    def logic_verifica_sottoclasse(self, nome_figlio, nome_genitore):
        if not self.ontology:
            return False
        cls_figlio = self.ontology[nome_figlio]
        cls_genitore = self.ontology[nome_genitore]
        if cls_figlio and cls_genitore:
            return issubclass(cls_figlio, cls_genitore)
        return False

    def logic_trova_classe(self, testo, nome_radice):
        if not self.ontology:
            return None
        clean_text = str(testo).lower().strip()
        root_class = self.ontology[nome_radice]
        if not root_class:
            return None
        # Tentativo A: Match esatto
        candidate = self.ontology[clean_text]
        if candidate and self.logic_verifica_sottoclasse(candidate.name, nome_radice):
            return candidate.name
        # Tentativo B: Ricerca profonda
        for cls in root_class.descendants():
            if cls.name.lower() == clean_text:
                return cls.name
            sinonimi = [str(s).lower() for s in getattr(cls, 'altLabel', [])]
            labels = [str(l).lower() for l in cls.label]
            if clean_text in sinonimi or clean_text in labels:
                return cls.name
        return None

    def logic_ragionamento_completo(self, dati_ospite, eventi):
        if not self.ontology:
            return []
        onto = self.ontology
        mapping_nomi_eventi = []
        temp_path = os.path.abspath(os.getenv("PATH_TEMP_ONTO"))
        with onto:
            nome_ospite_safe = dati_ospite['nome'].replace(" ", "_")
            ospite = onto.Ospite(nome_ospite_safe)
            ospite.eta = [dati_ospite.get('eta')]
            if hasattr(onto, "SOFFRE_DI"):
                for pat_str in dati_ospite.get('patologie', []):
                    ClassePat = getattr(onto, pat_str, None)
                    if ClassePat:
                        pat_inst = ClassePat(f"patologia_{pat_str}")
                        ospite.SOFFRE_DI.append(pat_inst)
            lista_interessi = dati_ospite.get('interessi_profilo') or dati_ospite.get('interessi', [])
            if hasattr(onto, "HA_INTERESSE"):
                for int_str in lista_interessi:
                    ClasseInt = getattr(onto, int_str, None)
                    if ClasseInt:
                        int_inst = ClasseInt(f"interesse_{int_str}")
                        ospite.HA_INTERESSE.append(int_inst)
            for ev in eventi:
                nome_ev_safe = ev['nome'].replace(" ", "_").replace("'", "")
                tipo_ev = ev['label']
                data_ora_str = ev.get('data')
                meteo_reale = ev.get('meteo')
                ClasseEvento = getattr(onto, tipo_ev, onto.EventoLocale)
                evento_inst = ClasseEvento(nome_ev_safe)
                dt_obj = datetime.now()
                if hasattr(onto, "data_ora"):
                    try:
                        dt_obj = datetime.fromisoformat(data_ora_str)
                    except:
                        dt_obj = datetime.now()
                    evento_inst.data_ora = [dt_obj]
                meteo_inst = onto.PrevisioneMeteo(f"Meteo_per_{nome_ev_safe}")
                meteo_inst.condizione = [meteo_reale]
                if hasattr(onto, "data_ora"):
                    meteo_inst.data_ora = [dt_obj]
                sug_name = f"Suggerimento_per_{nome_ev_safe}"
                suggerimento_inst = onto.Suggerimento(sug_name)
                if hasattr(onto, "INVIATO_A"):
                    suggerimento_inst.INVIATO_A.append(ospite)
                if hasattr(onto, "RIFERITO_A"):
                    suggerimento_inst.RIFERITO_A.append(evento_inst)
                mapping_nomi_eventi.append({
                    'iri_suggerimento': suggerimento_inst.iri,
                    'real_name_evento': ev['nome']
                })
            onto.save(file=temp_path)
        report = []
        for m in mapping_nomi_eventi:
            iri = m['iri_suggerimento']
            nome_reale = m['real_name_evento']
            neg = self.call_java(temp_path, iri, "idoneo", "false")
            if neg:
                report.append({"evento": nome_reale, "esito": "SCONSIGLIATO", "dettagli": neg})
                continue
            pos = self.call_java(temp_path, iri, "idoneo", "true")
            if pos:
                report.append({"evento": nome_reale, "esito": "CONSIGLIATO", "dettagli": pos})
        return report

    def call_java(self, owl_path, individual_iri, property_name, value):
        base_java_path = os.getenv("PATH_JAVA_PROJECT")
        jar_name = os.getenv("PATH_JAVA_FILE")
        dep_name = os.getenv("PATH_JAVA_DEPENDENCIES")
        main_class = os.getenv("PATH_JAVA_MAIN")
        if not all([base_java_path, jar_name, dep_name, main_class]):
            self.get_logger().error("ERRORE: Variabili ambiente per JAVA incomplete nel .env")
            return None
        jar_full = os.path.join(base_java_path, jar_name)
        dep_full = os.path.join(base_java_path, dep_name)
        classpath = f"{jar_full}{os.pathsep}{dep_full}"
        cmd = ["java", "-cp", classpath, main_class, str(owl_path), str(individual_iri), str(property_name), str(value)]
        try:
            result = subprocess.run(cmd, capture_output=True, text=True)
            if result.returncode == 0 and result.stdout.strip():
                output = result.stdout.strip()
                marker_start = "--- START AXIOMS ---"
                marker_end   = "--- END AXIOMS ---"
                if marker_start in output and marker_end in output:
                    try:
                        part1 = output.split(marker_start)[1]
                        axioms = part1.split(marker_end)[0].strip()
                        return axioms
                    except IndexError:
                        self.get_logger().error("Errore nel parsing dei marker Java.")
            if result.returncode != 0:
                self.get_logger().warning(f"Java ha restituito errore (Exit {result.returncode}): {result.stderr}")
        except Exception as e:
            self.get_logger().error(f"Eccezione esecuzione Java: {e}")
        return None

def main(args=None):
    rclpy.init(args=args)
    onto = ServerOntologia()
    rclpy.spin(onto)
    onto.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
