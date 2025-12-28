import os
from dotenv import load_dotenv
from neo4j import GraphDatabase
from owlready2 import get_ontology, default_world, sync_reasoner

load_dotenv()

class BaseConoscenzaManager:
    def __init__(self, use_ontology=True, use_neo4j=True):
        self.use_ontology = use_ontology
        if self.use_ontology:
            self.init_ontology()

        self.use_neo4j = use_neo4j
        if self.use_neo4j:
            self.init_neo4j()

        self.ontology = None
        self.neo4j_driver = None

    def init_ontology(self):
        path = os.getenv("PATH_ONTO")
        if not path:
            print("Variabile PATH_ONTO mancante nel .env")
            return
        try:
            self.ontology = get_ontology(path).load()
            print(f"Ontologia caricata: {path}")
        except Exception as e:
            print(f"Errore caricamento ontologia: {e}")
            self.ontology = None

    def init_neo4j(self):
        uri = os.getenv("NEO4J_URI")
        auth = (os.getenv("NEO4J_USER"), os.getenv("NEO4J_PASS"))
        if not uri or not auth[0]:
            print("Variabili Neo4j mancanti nel .env")
            return
        try:
            self.neo4j_driver = GraphDatabase.driver(uri, auth=auth)
            self.neo4j_driver.verify_connectivity()
            print("Neo4j connesso.")
        except Exception as e:
            print(f"Errore connessione Neo4j: {e}")
            self.neo4j_driver = None
    """
    def interrogaOntologia(self, query_sparql):
        if not self.use_ontology or not self.ontology:
            print("Richiesta ontologia ignorata: ontologia non caricata o disabilitata.")
            return []
        try:
            return list(default_world.sparql(query_sparql))
        except Exception as e:
            print(f"Errore query SPARQL: {e}")
            return []
    """
    def avvia_reasoner(self):
        if self.use_ontology and self.ontology:
            print("Avvio reasoner...")
            try:
                with self.ontology:
                    sync_reasoner()
                print("Ragionamento terminato.")
            except Exception as e:
                print(f"Errore reasoner: {e}")
    """
    def verifica_sottoclasse(self, nome_figlio, nome_genitore):
        if not self.use_ontology or not self.ontology:
            print("Ontologia non attiva.")
            return False
        cls_figlio = self.ontology[nome_figlio]
        cls_genitore = self.ontology[nome_genitore]
        if not cls_figlio or not cls_genitore:
            print(f"Errore: Classi {nome_figlio} o {nome_genitore} non trovate nell'ontologia.")
            return False
        return issubclass(cls_figlio, cls_genitore)
    """
    def interrogaGraphDatabase(self, query_cypher, parameters=None):
        if not self.use_neo4j or not self.neo4j_driver:
            print("Richiesta Neo4j ignorata: driver non attivo o disabilitato.")
            return []
        with self.neo4j_driver.session() as session:
            try:
                result = session.run(query_cypher, parameters)
                return [record.data() for record in result]
            except Exception as e:
                print(f"Errore query CYPHER: {e}")
                return []

    def close(self):
        if self.neo4j_driver:
            self.neo4j_driver.close()
            print("Connessione Neo4j chiusa.")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()