import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import json
import os
import subprocess

from rdflib import Graph, Namespace, RDF, OWL, Literal, XSD

from progetto.SincronizzaManager import SincronizzaManager

from dotenv import load_dotenv
load_dotenv()

"""
MANTENERE NEO4J SEMPRE AGGIORNATO
1. logic_crea_ontologia_istanze -> I NODI DI INTERESSE (AI FINI DEL REASONER, NON TUTTI!) E LE RELAZIONI DI INTERESSE (AI FINI DEL REASONER, NON TUTTE!) DA NEO4J VENGONO "INIETTATI" NELL'ONTOLOGIA CON I NOMI GIA PRONTI PER LA FASE FINALE
2. logic_spiegami_tutto         -> IL REASONER RAGIONA SUL FILE APPENA CREATO
NON DIMENTICARE DI SALVARE SU NEO4J I RISULTATI RILEVANTI DEL REASONER!
"""
class SpiegamiTutto(Node):
    def __init__(self):
        super().__init__("SpiegamiTutto")

        self.cb_group = ReentrantCallbackGroup()
        self.sub = self.create_subscription(String, "/explain/request", self.handle_request,
                                            10, callback_group=self.cb_group)
        self.pub = self.create_publisher(String, "/explain/response", 10)

        self.sincro = SincronizzaManager(self)

        self.get_logger().info("SpiegamiTutto avviato.")

    def handle_request(self, msg):
        try:
            req = json.loads(msg.data)
            req_id = req.get('id')
            action = req.get('payload', {}).get('action')
            data   = req.get('payload')
            response_payload = None
            success = True
            error_msg = ""
            if action == "crea_ontologia_istanze":
                if bool(data.get("braccialetti")):
                    owl_path = os.getenv("PATH_TEMP_ONTO_BRACCIALETTI")
                else:
                    owl_path = os.getenv("PATH_TEMP_ONTO")
                try:
                    self.logic_crea_ontologia_istanze(data.get("ids", []), owl_path)
                    response_payload = "Ontologia con istanze creata con successo."
                except Exception as e:
                    success = False
                    error_msg = f"Errore creazione ontologia con istanze: {str(e)}"
            elif action == "spiegami_tutto":
                if bool(data.get("braccialetti")):
                    owl_path = os.getenv("PATH_TEMP_ONTO_BRACCIALETTI")
                else:
                    owl_path = os.getenv("PATH_TEMP_ONTO")
                # self.get_logger().info(f"Usando owl_path: {owl_path}")
                risultato = self.logic_spiegami_tutto(
                    owl_path=owl_path,
                    parentClassName=data.get("parentClassName"),
                    debug=data.get("debug")
                )
                if risultato:
                    response_payload = risultato
                else:
                    success = False
                    error_msg = "Errore SpiegamiTutto..."
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
            self.get_logger().error(f"Errore callback SpiegamiTutto: {e}")

    def literal(self, value):
        if isinstance(value, int):
            return Literal(value, datatype=XSD.integer)
        if isinstance(value, float):
            return Literal(value, datatype=XSD.double)
        if isinstance(value, str) and "T" in value and value[0].isdigit():
             return Literal(value, datatype=XSD.dateTime)
        if isinstance(value, str):
            return Literal(value, datatype=XSD.string)
        return Literal(value)

    def logic_crea_ontologia_istanze(self, ids, owl_path):
        PATH_ONTO = os.getenv("PATH_ONTO")
        g = Graph()
        g.parse(PATH_ONTO, format="xml")
        ontology_iri = list(g.subjects(RDF.type, OWL.Ontology))
        if ontology_iri:
            BASE_IRI = str(ontology_iri[0])
            if not BASE_IRI.endswith('#') and not BASE_IRI.endswith('/'):
                BASE_IRI += '#'
        else:
            BASE_IRI = os.getenv("BASE_IRI")
            if not BASE_IRI.endswith('#'):
                BASE_IRI += '#'
        # self.get_logger().info(f"Usando BASE_IRI: {BASE_IRI}")
        NS = Namespace(BASE_IRI)
        g.bind("ex",  NS)
        g.bind("owl", OWL)
        g.bind("rdf", RDF)
        g.bind("xsd", XSD)
        nodes = self.sincro.interrogaGraphDatabase("""
            MATCH (n) WHERE id(n) IN $ids RETURN id(n) AS id, labels(n) AS labels, properties(n) AS props""", {"ids": ids})
        for record in nodes:  # Export nodi:
            node_id = record["id"]
            labels  = record["labels"]
            props   = record["props"]
            class_name = labels[0] if labels else "Node"      # Classe principale
            instance_name = next((str(v).replace(" ", "_") for k, v in props.items() if k.startswith("nome")), f"{class_name}_{node_id}")
            node_iri = NS[instance_name]
            g.add((node_iri, RDF.type, OWL.NamedIndividual))  # Dichiara individuo OWL
            for label in labels:                              # Aggiungi tutte le label come rdf:type
                g.add((node_iri, RDF.type, NS[label]))
            for key, value in props.items():                  # Aggiungi proprietà
                g.add((node_iri, NS[key], self.literal(value)))
        rels = self.sincro.interrogaGraphDatabase("""
            MATCH (a)-[r]->(b) WHERE id(a) IN $ids AND id(b) IN $ids RETURN id(a) AS aid, labels(a) AS alabels, properties(a) AS aprops,
            type(r) AS rel, id(b) AS bid, labels(b) AS blabels, properties(b) AS bprops""", {"ids": ids})
        for record in rels:  # Export relazioni:
            # Classi principali
            a_class = record["alabels"][0] if record["alabels"] else "Node"
            b_class = record["blabels"][0] if record["blabels"] else "Node"
            # IRI individui coerenti
            subj_name = next((str(v).replace(" ", "_") for k, v in record["aprops"].items() if k.startswith("nome")), f"{a_class}_{record['aid']}")
            obj_name  = next((str(v).replace(" ", "_") for k, v in record["bprops"].items() if k.startswith("nome")), f"{b_class}_{record['bid']}")
            subj = NS[subj_name]
            obj  = NS[obj_name]
            pred = NS[record["rel"]]                           # ObjectProperty
            g.add((pred, RDF.type, OWL.ObjectProperty))        # Dichiara la proprieta' OWL
            g.add((subj, pred, obj))                           # Aggiungi la tripla
        g.serialize(destination=owl_path, format="xml")  # Salvataggio
        # self.get_logger().info(f"Export completato: {owl_path}")

    def logic_spiegami_tutto(self, owl_path, parentClassName, debug):
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
        cmd = ["java", "-cp", classpath, main_class, str(owl_path), str(parentClassName), str(debug)]
        try:
            result = subprocess.run(cmd, capture_output=True, text=True)
            if result.returncode == 0 and result.stdout.strip():
                return result.stdout.strip()
            if result.returncode != 0:
                self.get_logger().warning(f"Java ha restituito errore (Exit {result.returncode}): {result.stderr}")
        except Exception as e:
            self.get_logger().error(f"Eccezione esecuzione Java: {e}")
        return None

def main(args=None):
    rclpy.init(args=args)
    node = SpiegamiTutto()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
