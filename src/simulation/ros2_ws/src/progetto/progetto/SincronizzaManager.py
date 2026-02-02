from progetto.SincronizzamiTutto import SincronizzamiTutto

class SincronizzaManager():
    def __init__(self, nodo):
        self.nodo = nodo

        self.neo4j_client = SincronizzamiTutto(self.nodo,
            "/neo4j/request", "/neo4j/response", timeout=120.0)
        self.onto_client = SincronizzamiTutto(self.nodo,
            "/onto/request", "/onto/response", timeout=120.0)
        self.llm_client = SincronizzamiTutto(self.nodo,
            "/llm/request", "/llm/response", timeout=120.0)
        self.explain_client = SincronizzamiTutto(self.nodo,
            "/explain/request", "explain/response", timeout=120.0)

        self.nodo.get_logger().info('SincronizzaManager avviato.')

    def interrogaGraphDatabase(self, query_cypher, parameters=None):
        payload = {
            "query": query_cypher,
            "params": parameters or {}
        }
        try:
            return self.neo4j_client.call(payload)
        except Exception as e:
            self.nodo.get_logger().error(f"Errore Neo4j: {e}")
            return []

    def verifica_sottoclasse(self, nome_figlio, nome_genitore):
        payload = {
            "action": "verifica_sottoclasse",
            "nome_figlio": nome_figlio,
            "nome_genitore": nome_genitore
        }
        try:
            return self.onto_client.call(payload)
        except Exception as e:
            self.nodo.get_logger().error(f"Errore onto (verifica_sottoclasse): {e}")
            return False

    def trova_classe_da_sinonimo(self, testo, nome_radice):
        payload = {
            "action": "trova_classe_da_sinonimo",
            "testo": testo,
            "nome_radice": nome_radice
        }
        try:
            return self.onto_client.call(payload)
        except Exception as e:
            self.nodo.get_logger().error(f"Errore onto (trova_classe): {e}")
            return None

    def ragiona_su_eventi(self, dati_ospite, lista_eventi):
        payload = {
            "action": "ragiona_su_eventi",
            "dati_ospite": dati_ospite,
            "eventi": lista_eventi
        }
        try:
            return self.onto_client.call(payload)
        except Exception as e:
            self.nodo.get_logger().error(f"Errore chiamata ragionamento onto: {e}")
            return []

    def ask_llm(self, testo_utente, scenario, tipo):
        payload = {
            "msg": testo_utente,
            "scenario": scenario,
            "tipo": tipo
        }
        try:
            return self.llm_client.call(payload)
        except Exception as e:
            self.nodo.get_logger().error(f"Errore chiamata LLM: {e}")
            return None

    def crea_ontologia_istanze(self, ids, braccialetti=False):
        payload = {
            "action": "crea_ontologia_istanze",
            "ids": ids,
            "braccialetti": braccialetti
        }
        try:
            return self.explain_client.call(payload)
        except Exception as e:
            self.nodo.get_logger().error(f"Errore richiesta ontologia istanze: {e}")
            return None

    def spiegami_tutto(self, parentClassName, debug="False", braccialetti=False):
        payload = {
            "action": "spiegami_tutto",
            "parentClassName": parentClassName,
            "debug": debug,
            "braccialetti": braccialetti
        }
        try:
            return self.explain_client.call(payload)
        except Exception as e:
            self.nodo.get_logger().error(f"Errore richiesta spiegazione: {e}")
            return None
