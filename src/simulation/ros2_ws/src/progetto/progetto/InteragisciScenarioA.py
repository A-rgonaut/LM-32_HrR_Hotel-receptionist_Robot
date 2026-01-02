from progetto.InteragisciConOspite import InteragisciConOspite
from progetto.utils import Ospite

import os
import subprocess
from datetime import datetime
import string

class InteragisciScenarioA(InteragisciConOspite):
    def rileva_lingua(self, testo):
        testo_clean = testo.lower().translate(str.maketrans('', '', string.punctuation))
        parole = testo_clean.split()
        score_it = 0
        score_en = 0
        vocab = {
            'IT': {
                'ciao', 'buongiorno', 'buonasera', 'salve', 'arrivederci',
                'sono', 'mi', 'chiamo', 'io', 'vorrei', 'per', 'favore',
                'grazie', 'prego', 'il', 'lo', 'la', 'che', 'cosa',
                'tutto', 'bene', 'scusa', 'hotel', 'prenotazione', 'camera'
            },
            'EN': {
                'hello', 'hi', 'hey', 'good', 'morning', 'evening', 'bye',
                'am', 'my', 'name', 'is', 'i', 'would', 'like', 'please',
                'thanks', 'thank', 'you', 'the', 'what', 'how',
                'everything', 'ok', 'sorry', 'reservation', 'room'
            }
        }
        for parola in parole:
            if parola in vocab['IT']:
                score_it += 1
            if parola in vocab['EN']:
                score_en += 1
        if score_it > score_en:
            return 'IT'
        elif score_en > score_it:
            return 'EN'
        return None

    def aggiorna_lingua(self, kb):
        query = "MATCH (o:Ospite) WHERE id(o) = $id SET o.lingua = $lingua RETURN o.lingua"
        parametri = {
            "id":     self.contesto['ospite'].id,
            "lingua": self.contesto['ospite'].lingua
        }
        aggiornato = kb.interrogaGraphDatabase(query, parametri)
        return aggiornato[0]['o.lingua'] == self.contesto['ospite'].lingua

    def recupera_dati_prenotazione(self, kb):
        query = """
            MATCH (o:Ospite)-[:EFFETTUA]->(p:Prenotazione)
            WHERE id(o) = $id
            WITH p
            ORDER BY p.data_inizio DESC LIMIT 1
            RETURN
                toString(p.data_inizio) AS checkin,
                toString(p.data_fine) AS checkout,
                duration.inDays(date(p.data_inizio), date(p.data_fine)).days AS notti_reali
        """
        risultati = kb.interrogaGraphDatabase(query, {'id': self.contesto['ospite'].id})
        if risultati:
            self.contesto['checkin']   = risultati[0]['checkin']
            self.contesto['checkout']  = risultati[0]['checkout']
            self.contesto['num_notti'] = risultati[0]['notti_reali']
            self.nodo.get_logger().info(f"[ScenarioA] Prenotazione: {risultati[0]}")
            return True
        else:
            self.nodo.get_logger().warning("[ScenarioA] Nessuna prenotazione.")
            return False

    def rileva_interesse(self, testo, kb, llm):
        # nome_interesse_raw = llm.handle_request(scenario="A", tipo="estrazione_semantica", msg=testo)
        nome_interesse_raw = "montagna"
        self.nodo.parla("DECOMMENTARE LA CHIAMATA LLM 1")
        nome_classe_ufficiale = kb.trova_classe_da_sinonimo(nome_interesse_raw, nome_radice="Interesse")
        if nome_classe_ufficiale:
            self.nodo.get_logger().info(f"Interesse rilevato: '{nome_interesse_raw}' -> Mapped to: '{nome_classe_ufficiale}'")
            return nome_classe_ufficiale
        else:
            self.nodo.get_logger().warning(f"Nessuna classe ontologica trovata per: '{nome_interesse_raw}'")
            return None

    def salva_interesse(self, kb):
        query = """
        MATCH (o:Ospite)
        WHERE id(o) = $id
        MERGE (i:Interesse {nome: $nome_interesse})
        MERGE (o)-[:HA_INTERESSE]->(i)
        """
        kb.interrogaGraphDatabase(query, {
            'id': self.contesto['ospite'].id,
            'nome_interesse': self.contesto['interesse']
        })
        self.nodo.get_logger().info(f"[DB] Salvato interesse '{self.contesto['interesse']}' per l'ospite.")

    def recupera_dati_per_suggerimento(self, kb):
        query = """
            MATCH (o:Ospite)-[:EFFETTUA]->(p:Prenotazione)
            WHERE id(o) = $id
            OPTIONAL MATCH (o)-[:SOFFRE_DI]->(pat)
            OPTIONAL MATCH (o)-[:HA_INTERESSE]->(int)
            MATCH (o)-[:HA_INTERESSE]->(i:Interesse)
            MATCH (e)
            WHERE e.data_ora >= p.data_inizio AND e.data_ora <= p.data_fine
            AND any(lbl IN labels(e) WHERE toLower(lbl) CONTAINS toLower(i.nome))
            OPTIONAL MATCH (m:PrevisioneMeteo) WHERE m.data_ora = e.data_ora
            RETURN
                o.nome AS nome,
                collect(DISTINCT labels(pat)) AS patologie_labels,
                collect(DISTINCT labels(int)) AS interessi_labels,
                collect(DISTINCT {
                    tipo: labels(e),
                    nome: e.nome,
                    data: toString(e.data_ora),
                    meteo: m.condizione
                }) AS eventi_disponibili
        """
        risultati = kb.interrogaGraphDatabase(query, {'id': self.contesto['ospite'].id})
        if risultati:
            record = risultati[0]
            self.contesto['dati_neo4j'] = {
                'nome_ospite': record['nome'],
                'patologie': [lbl[0] for lbl in record['patologie_labels'] if lbl],
                'interessi_profilo': [lbl[0] for lbl in record['interessi_labels'] if lbl],
                'eventi': []
            }
            for ev in record['eventi_disponibili']:
                tipo_evento = next((l for l in ev['tipo'] if "Evento" in l), "EventoGenerico")
                self.contesto['dati_neo4j']['eventi'].append({
                    'label': tipo_evento,
                    'nome': ev['nome'],
                    'data': ev['data'],
                    'meteo': ev.get('meteo')
                })
            self.nodo.get_logger().info(f"[ScenarioA] Eventi caricati da DB: {len(self.contesto['dati_neo4j']['eventi'])}")
            return True
        return False

    def costruisci_mondo_owl(self, kb):
        onto = kb.ontology
        dati = self.contesto['dati_neo4j']
        nome_ospite_safe = dati['nome_ospite'].replace(" ", "_")
        with onto:
            ospite = onto.Ospite(nome_ospite_safe)
            ospite.eta = [self.contesto['ospite'].eta]
            if hasattr(onto, "SOFFRE_DI"):
                for pat_str in dati['patologie']:
                    ClassePat = getattr(onto, pat_str, None)
                    if ClassePat:
                        pat_inst = ClassePat(f"patologia_{pat_str}")
                        ospite.SOFFRE_DI.append(pat_inst)
            if hasattr(onto, "HA_INTERESSE"):
                for int_str in dati['interessi_profilo']:
                    ClasseInt = getattr(onto, int_str, None)
                    if ClasseInt:
                        int_inst = ClasseInt(f"interesse_{int_str}")
                        ospite.HA_INTERESSE.append(int_inst)
            self.contesto['mapping_nomi_eventi'] = []
            for ev in dati['eventi']:
                nome_ev_safe = ev['nome'].replace(" ", "_").replace("'", "")
                tipo_ev = ev['label']
                data_ora_str = ev.get('data')
                meteo_reale = ev.get('meteo')
                ClasseEvento = getattr(onto, tipo_ev, onto.EventoLocale)
                evento_inst = ClasseEvento(nome_ev_safe)
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
                self.contesto['mapping_nomi_eventi'].append({
                    'iri_suggerimento': suggerimento_inst.iri,
                    'real_name_evento': ev['nome']
                })
            temp_file = os.path.abspath(os.getenv("PATH_TEMP_ONTO"))
            onto.save(file=temp_file)
            return temp_file

    def chiedi_spiegazione_a_java(self, owl_path, nome_evento_iri, proprieta, valore):
        from dotenv import load_dotenv
        load_dotenv()
        base_java_path = os.getenv("PATH_JAVA_PROJECT")
        jar_file = f'{base_java_path}/{os.getenv("PATH_JAVA_FILE")}'
        dependencies = f'{base_java_path}/{os.getenv("PATH_JAVA_DEPENDENCIES")}'
        classpath = f"{jar_file}:{dependencies}"
        main_class = os.getenv("PATH_JAVA_MAIN")
        cmd = ["java", "-cp", classpath, main_class, owl_path, nome_evento_iri, proprieta, valore]
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, check=False)
            output = result.stdout
            marker_start = "--- START AXIOMS ---"
            marker_end   = "--- END AXIOMS ---"
            if marker_start in output and marker_end in output:
                part1 = output.split(marker_start)[1]
                axioms = part1.split(marker_end)[0].strip()
                return axioms
            return None
        except Exception as e:
            self.nodo.get_logger().error(f"Errore chiamata Java: {e}")
            return None

    def suggerisci_evento_locale(self, kb, llm):
        self.nodo.parla("Un attimo, analizzo tutti gli eventi disponibili...")
        if not self.recupera_dati_per_suggerimento(kb):
            self.nodo.parla("Errore nel recupero dati.")
            return
        try:
            owl_path = self.costruisci_mondo_owl(kb)
        except Exception as e:
            self.nodo.get_logger().error(f"Errore OWL: {e}")
            return
        candidati = self.contesto.get('mapping_nomi_eventi', [])
        report_ragionamento = []
        for cand in candidati:
            iri_sugg = cand['iri_suggerimento']
            nome_evento = cand['real_name_evento']
            assiomi_neg = self.chiedi_spiegazione_a_java(owl_path, iri_sugg, "idoneo", "false")
            if assiomi_neg:
                self.nodo.get_logger().info(f"NEGATIVO: {nome_evento}")
                report_ragionamento.append({
                    "evento": nome_evento,
                    "esito": "SCONSIGLIATO",
                    "dettagli": assiomi_neg
                })
                continue
            assiomi_pos = self.chiedi_spiegazione_a_java(owl_path, iri_sugg, "idoneo", "true")
            if assiomi_pos:
                self.nodo.get_logger().info(f"POSITIVO: {nome_evento}")
                report_ragionamento.append({
                    "evento": nome_evento,
                    "esito": "CONSIGLIATO",
                    "dettagli": assiomi_pos
                })
            else:
                self.nodo.get_logger().info(f"NEUTRO/IGNOTO: {nome_evento}")
        if report_ragionamento:
            msg_input = f"Analisi per l'ospite {self.contesto['ospite'].nome}:\n"
            for item in report_ragionamento:
                msg_input += f"\nEVENTO: {item['evento']}\nESITO: {item['esito']}\nMOTIVAZIONE LOGICA: {item['dettagli']}\n----------------"
            self.nodo.get_logger().info(msg_input)
            # risposta = llm.handle_request(scenario="A", tipo="explainability", msg=msg_input)
            # self.nodo.parla(risposta)
            self.nodo.parla("DECOMMENTARE LA CHIAMATA LLM 2")
        else:
            self.nodo.parla("Ho analizzato gli eventi ma non ho trovato indicazioni specifiche (né positive né negative).")

    def esegui(self, testo, kb, llm):
        self.nodo.get_logger().info(f"[ScenarioA] Stato: {self.stato}, Input: {testo}")
        if self.stato == "INIZIO":
            lingua = self.rileva_lingua(testo)
            if lingua is None:
                self.nodo.parla(super().dialogo_scriptato(tipo="errore_lingua"))
                return
            self.contesto['ospite'].lingua = lingua
            print(self.aggiorna_lingua(kb))
            if self.recupera_dati_prenotazione(kb):
                self.nodo.parla(super().dialogo_scriptato(tipo="benvenuto"))
                self.stato = "RILEVA_INTERESSE"
            else:
                self.nodo.parla("Non trovo nessuna prenotazione attiva.")
                self.stato = "FINE"
        elif self.stato == "RILEVA_INTERESSE":
            interesse = self.rileva_interesse(testo, kb, llm)
            if interesse:
                self.nodo.get_logger().info(interesse)
                self.contesto['interesse'] = interesse
                self.nodo.parla(super().dialogo_scriptato(tipo="conferma_interesse"))
                self.stato = "CONFERMA"
        elif self.stato == "CONFERMA":
            if super().rileva_conferma(testo):
                if self.contesto['interesse']:
                    self.salva_interesse(kb)
                    self.stato = "SUGGERISCI_EVENTO_LOCALE"
                    self.esegui("", kb, llm)  # Presidente?
            else:
                self.stato = "RILEVA_INTERESSE"
                self.esegui(testo, kb, llm)
        elif self.stato == "SUGGERISCI_EVENTO_LOCALE":
            self.suggerisci_evento_locale(kb, llm)
            self.stato = "FINE"
        elif self.stato == "FINE":
            self.nodo.parla(super().dialogo_scriptato(tipo="arrivederci"))
        else:
            self.nodo.get_logger().info("self.stato NON VALIDO.")
