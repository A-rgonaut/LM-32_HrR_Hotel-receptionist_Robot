import json

class Emergenza():
    def __init__(self, nodo):
        self.nodo = nodo

    def carica_dati(self):
        dati = self.nodo.dati_salute
        if dati is None:
            self.nodo.get_logger().info("[Emergenza] Nessun dato salute ricevuto dai braccialetti.")
            return None,None
        query = """
        UNWIND $batch AS row MATCH (o)
        // Casting esplicito: l'ID nel JSON è stringa, id(o) è intero
        WHERE id(o) = toInteger(row.id)
        SET o.bpm_attuale = toInteger(row.hr),
            o.pressione_min_attuale = toInteger(row.pmin),
            o.pressione_max_attuale = toInteger(row.pmax)
        RETURN row.id AS id_estratto, row.x AS x, row.z AS z
        """
        parametri = {  # Mappiamo la lista Python sul parametro Cypher $batch
            "batch": dati
        }
        risultati = self.nodo.sincro.interrogaGraphDatabase(query, parametri)
    
        # Se la query restituisce i valori direttamente da Cypher (scelta più efficiente)
        if risultati:
            # Esempio per il primo elemento del batch
            primo_risultato = risultati[0]
            return primo_risultato['x'], primo_risultato['z']
        
        return None, None

    def importa_dati(self):
        # Recuperiamo gli ID di tutti gli Ospiti, delle Soglie globali e delle loro Patologie specifiche.
        # Questo permette a crea_ontologia_istanze di scaricare i nodi completi e le relazioni.
        query = "MATCH (o:Ospite) RETURN id(o) as node_id"
        risultati = self.nodo.sincro.interrogaGraphDatabase(query, {})  # Esecuzione query
        if risultati:  # Creiamo la lista piatta di ID per crea_ontologia_istanze
            lista_ids = [record['node_id'] for record in risultati if record['node_id'] is not None]
            self.nodo.get_logger().info(f"[Monitoraggio] Recuperati {len(lista_ids)} nodi (Ospiti e Soglie).")
            return lista_ids
        self.nodo.get_logger().info("[Monitoraggio] Nessun dato trovato.")
        return []

    def cambia_stato_spe(self, nome, cognome):
        # query = "MATCH (n) WHERE id(n) = $p.id REMOVE n:Ospite:OspiteInEmergenza SET n:OspiteChiamatoSpecialista,  n.aggiornato_il = datetime() RETURN n" per mettere il timestamp
        query = "MATCH (n) WHERE n.nome = $nome AND n.cognome = $cognome REMOVE n:Ospite, n:OspiteInStatodiAllerta, n:OspiteStatoChiamataSpecialista SET n:OspiteStatoChiamataSpecialista RETURN id(n) AS id "
        parametri = {
            "nome":     nome,
            "cognome": cognome
        }
        aggiornato = self.nodo.sincro.interrogaGraphDatabase(query, parametri)
        return aggiornato[0]['id'] if aggiornato else None

    def cambia_stato_allerta(self, nome, cognome):
        # query = "MATCH (n) WHERE id(n) = $p.id REMOVE n:Ospite:OspiteInEmergenza SET n:OspiteChiamatoSpecialista,  n.aggiornato_il = datetime() RETURN n" per mettere il timestamp
        query = "MATCH (n) WHERE n.nome = $nome AND n.cognome = $cognome REMOVE n:Ospite, n:OspiteInStatodiAllerta, n:OspiteStatoChiamataSpecialista SET n:OspiteInStatodiAllerta RETURN id(n) AS id "
        parametri = {
            "nome":     nome,
            "cognome": cognome
        }
        aggiornato = self.nodo.sincro.interrogaGraphDatabase(query, parametri)
        return aggiornato[0]['id'] if aggiornato else None
