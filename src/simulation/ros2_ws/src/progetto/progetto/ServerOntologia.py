import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import json
import os

from owlready2 import get_ontology, AnnotationProperty

from dotenv import load_dotenv
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

    def logic_trova_classe(self, lista_sintomi, nome_radice):
        self.get_logger().info(f"nome radice {nome_radice}  {lista_sintomi}")
        if not self.ontology or not isinstance(lista_sintomi, list):
            return []
        root_class = self.ontology[nome_radice]
        if not root_class:
            self.get_logger().info(f"  if not root_class:")
            return []
        risultati = set()
        for sintomo in lista_sintomi:
            clean_term = str(sintomo).lower().strip()
            match_found = None
            candidate = self.ontology[clean_term]
            if candidate and self.logic_verifica_sottoclasse(candidate.name, nome_radice):
                match_found = candidate.name
            if not match_found:
                for cls in root_class.descendants():
                    if cls.name.lower() == clean_term:
                        match_found = cls.name
                    else:
                        sinonimi = [str(s).lower() for s in getattr(cls, 'altLabel', [])]
                        labels = [str(l).lower() for l in cls.label]
                        if clean_term in sinonimi or clean_term in labels:
                            match_found = cls.name
            if match_found:
                risultati.add(match_found)
            else:
                risultati.add("")
        return list(risultati)

def main(args=None):
    rclpy.init(args=args)
    onto = ServerOntologia()
    rclpy.spin(onto)
    onto.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
