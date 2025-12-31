import os
from dotenv import load_dotenv
from neo4j import GraphDatabase
from owlready2 import get_ontology, default_world, sync_reasoner, AnnotationProperty

load_dotenv()

class BaseConoscenzaManager:
    def __init__(self, use_ontology=True, use_neo4j=True):
        self.ontology = None
        self.neo4j_driver = None

        self.use_ontology = use_ontology
        if self.use_ontology:
            self.init_ontology()

        self.use_neo4j = use_neo4j
        if self.use_neo4j:
            self.init_neo4j()

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
        if cls_figlio and cls_genitore:
            return issubclass(cls_figlio, cls_genitore)
        print(f"Errore: Classi {nome_figlio} o {nome_genitore} non trovate nell'ontologia.")
        return False

    def trova_classe_da_sinonimo(self, testo, nome_radice):
        if not self.ontology:
            return None
        onto = self.ontology
        clean_text = testo.lower().strip()
        # 1. Definizione dinamica SKOS (se non esiste già)
        skos = onto.get_namespace("http://www.w3.org/2004/02/skos/core#")
        with onto:
            # Controllo preventivo per non ridefinire se già presente
            if not onto["altLabel"]:
                class altLabel(AnnotationProperty):
                    namespace = skos
        # 2. Recupero Classe Radice
        root_class = onto[nome_radice]
        if not root_class:
            print(f"Classe radice '{nome_radice}' non trovata.")
            return None
        # 3. Tentativo A: Ricerca Esatta
        candidate = onto[clean_text]
        if candidate and self.verifica_sottoclasse(candidate.name, nome_radice):
            return candidate.name
        # 4. Tentativo B: Ricerca profonda (Label e Sinonimi)
        for cls in root_class.descendants():
            # Recupera label e altLabel in modo sicuro
            # getattr(cls, 'altLabel', []) evita crash se la proprietà manca
            if cls.name.lower() == clean_text:
                return cls.name
            sinonimi = [str(s).lower() for s in getattr(cls, 'altLabel', [])]
            labels = [str(l).lower() for l in cls.label]
            if clean_text in sinonimi or clean_text in labels:
                return cls.name
        return None

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
