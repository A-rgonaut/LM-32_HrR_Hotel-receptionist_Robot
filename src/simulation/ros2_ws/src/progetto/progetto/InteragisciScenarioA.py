from progetto.InteragisciConOspite import InteragisciConOspite
from progetto.utils import Ospite

import os
import subprocess
from datetime import datetime

class InteragisciScenarioA(InteragisciConOspite):
    def rileva_lingua(self, testo):
        dizionario = {
            'IT': {'ciao', 'buongiorno', 'salve', 'buonasera', 'sono', 'chiamo'},
            'EN': {'hi', 'hello', 'hey', 'good morning', 'am', 'name'},
        }
        parole = set(testo.lower().split())
        for lang, parola in dizionario.items():
            if parole.intersection(parola):
                return lang
        return None

    def aggiorna_lingua(self, kb):
        query = "MATCH (o:Ospite) WHERE id(o) = $id SET o.lingua = $lingua RETURN o.lingua"
        parametri = {
            "id":     0,  # self.contesto['ospite'].id,
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
        risultati = kb.interrogaGraphDatabase(query, {'id': 0})  # self.contesto['ospite'].id})
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
        nome_interesse_raw = "montagna" # llm.handle_request(tipo="estrazione_semantica", msg=testo)
        self.nodo.parla("DECOMMENTARE LA CHIAMATA LLM 1")
        nome_classe_ufficiale = kb.trova_classe_da_sinonimo(nome_interesse_raw, nome_radice="Interesse")
        if nome_classe_ufficiale:
            self.nodo.get_logger().info(f"Interesse rilevato: '{nome_interesse_raw}' -> Mapped to: '{nome_classe_ufficiale}'")
            return nome_classe_ufficiale
        else:
            self.nodo.get_logger().warning(f"Nessuna classe ontologica trovata per: '{nome_interesse_raw}'")
            return None

    def salva_interesse_db(self, kb, nome_interesse):
        query = """
        MATCH (o:Ospite)
        WHERE id(o) = $id
        MERGE (i:Interesse {nome: $nome_interesse})
        MERGE (o)-[:HA_INTERESSE]->(i)
        """
        kb.interrogaGraphDatabase(query, {
            'id': 0,  # self.contesto['ospite'].id,
            'nome_interesse': nome_interesse
        })
        self.nodo.get_logger().info(f"[DB] Salvato interesse '{nome_interesse}' per l'ospite.")

    def recupera_dati_per_suggerimento(self, kb):
        query = """
            MATCH (o:Ospite)-[:EFFETTUA]->(p:Prenotazione)
            WHERE id(o) = $id
            OPTIONAL MATCH (o)-[:SOFFRE_DI]->(pat)
            OPTIONAL MATCH (o)-[:HA_INTERESSE]->(int)
            MATCH (e)
            WHERE (e:EventoCitta OR e:EventoMontagna OR e:EventoMare)
              AND e.data_ora >= p.data_inizio
              AND e.data_ora <= p.data_fine
            RETURN
                o.nome AS nome,
                collect(DISTINCT labels(pat)) AS patologie_labels,
                collect(DISTINCT labels(int)) AS interessi_labels,
                collect(DISTINCT {
                    tipo: labels(e),
                    nome: e.nome,
                    data: toString(e.data_ora)
                }) AS eventi_disponibili
        """
        risultati = kb.interrogaGraphDatabase(query, {'id': 0})  # self.contesto['ospite'].id})
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
                    'nome': ev['nome']
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
            ospite.eta = [30] # Default o prendilo da dati_neo4j se c'è

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
                data_ora_str = ev.get('data', "2026-01-14T10:00:00") # Data evento

                ClasseEvento = getattr(onto, tipo_ev, onto.EventoLocale)
                evento_inst = ClasseEvento(nome_ev_safe)

                # Assegna data_ora (necessaria per regola S3 Meteo)
                # owlready gestisce le date come datetime o stringhe a seconda del caricamento
                # Per sicurezza passiamo il valore esatto che si aspetta la regola (datetime)
                # Qui usiamo stringa/datetime a seconda di come è definita la property
                if hasattr(onto, "data_ora"):
                    # Mock: proviamo a passare la stringa ISO, owlready dovrebbe parsare
                    evento_inst.data_ora = [datetime.fromisoformat(data_ora_str)]

                # B. Creazione Meteo Favorevole (Trucco per attivare regola S3)
                # Creiamo una previsione meteo "sereno" per la stessa data dell'evento
                meteo_inst = onto.PrevisioneMeteo(f"Meteo_per_{nome_ev_safe}")
                # meteo_inst.condizione = ["sereno"]
                meteo_inst.condizione = ["diluvio"]
                if hasattr(onto, "data_ora"):
                    meteo_inst.data_ora = [datetime.fromisoformat(data_ora_str)]

                # C. Creazione Nodo SUGGERIMENTO (Il cuore delle tue regole)
                # Regole: INVIATO_A Ospite, RIFERITO_A Evento
                sug_name = f"Suggerimento_per_{nome_ev_safe}"
                suggerimento_inst = onto.Suggerimento(sug_name)

                if hasattr(onto, "INVIATO_A"):
                    suggerimento_inst.INVIATO_A.append(ospite)
                if hasattr(onto, "RIFERITO_A"):
                    suggerimento_inst.RIFERITO_A.append(evento_inst)

                # Salviamo il mapping puntando al SUGGERIMENTO, non all'evento!
                # Perché è il suggerimento che diventa "idoneo" o no.
                self.contesto['mapping_nomi_eventi'].append({
                    'iri_suggerimento': suggerimento_inst.iri,
                    'real_name_evento': ev['nome']
                })

            # --- 3. CLASSI MAGICHE (Bridge Boolean -> Class) ---
            # Definiamo classi dinamiche per permettere a Java di controllare 'idoneo'
            if hasattr(onto, "idoneo"):

                # Classe: SuggerimentoPositivo <-> Suggerimento and idoneo == true
                if not onto["SuggerimentoPositivo"]:
                    class SuggerimentoPositivo(onto.Suggerimento):
                        equivalent_to = [onto.Suggerimento & (onto.idoneo == True)]

                # Classe: SuggerimentoNegativo <-> Suggerimento and idoneo == false
                if not onto["SuggerimentoNegativo"]:
                    class SuggerimentoNegativo(onto.Suggerimento):
                        equivalent_to = [onto.Suggerimento & (onto.idoneo == False)]
            else:
                print("ATTENZIONE: La DataProperty 'idoneo' non è stata trovata nell'ontologia. Le classi magiche non sono state create.")

            temp_file = os.path.abspath("/tmp/temp_context.owl")
            onto.save(file=temp_file)
            return temp_file

    def chiedi_spiegazione_a_java(self, owl_path, nome_evento_iri, classe_target):
        from dotenv import load_dotenv
        load_dotenv()

        # Recupera il percorso base del progetto Java (es: /root/ros2_ws/src/SpiegamiTutto)
        # Ti consiglio di mettere nel .env: JAVA_PROJECT_PATH=/root/ros2_ws/src/SpiegamiTutto
        base_java_path = os.getenv("JAVA_PROJECT_PATH", "/root/ros2_ws/src/SpiegamiTutto")

        # 1. Costruiamo il Classpath (Attenzione ai percorsi assoluti per essere sicuri)
        # Include il tuo JAR compilato e tutte le dipendenze nella cartella dependency
        jar_file = f"{base_java_path}/target/demo-1.0-SNAPSHOT.jar"
        dependencies = f"{base_java_path}/target/dependency/*"
        classpath = f"{jar_file}:{dependencies}" # Nota: su Windows si usa ';' invece di ':'

        main_class = "com.example.l.SpiegamiTutto"

        print(f"--- DEBUG JAVA CALL (Classpath Mode) ---")
        print(f"CP: {classpath}")
        print(f"Main: {main_class}")
        print(f"Args: {owl_path}, {nome_evento_iri}, {classe_target}")

        cmd = ["java", "-cp", classpath, main_class, owl_path, nome_evento_iri, classe_target]
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, check=False)
            # if "--- START AXIOMS ---" in output:
                # start = output.find("--- START AXIOMS ---") + len("--- START AXIOMS ---")
                # end = output.find("--- END AXIOMS ---")
                # return output[start:end].strip()
            if result.stderr:
                print(f"🔴 JAVA STDERR:\n{result.stderr}")
            print(f"🟢 JAVA STDOUT: {result.stdout}")
            if result.returncode != 0:
                self.nodo.get_logger().error(f"Java terminato con codice errore: {result.returncode}")
                return None
            return result.stdout
        except Exception as e:
            self.nodo.get_logger().error(f"Errore subprocess Java: {e}")
            return None
    '''
        if evento_scelto:
            msg_input = f"""
            Nome Ospite: {self.contesto['ospite'].nome}
            Nome Evento Suggerito: {evento_scelto}
            --- LOGICA FORNITA DAL REASONER ---
            {assiomi_ragionamento}
            """
        if risultato:
            nome_ev, assiomi, contesto = risultato
            msg_input = f"""
            CONTESTO: {contesto}
            Nome Ospite: {self.contesto['ospite'].nome}
            Nome Evento: {nome_ev}
            --- ASSIOMI SWRL ---
            {assiomi}
            """
            # risposta = llm.handle_request(tipo="explainability", msg=msg_input)
    '''
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

            assiomi_pos = self.chiedi_spiegazione_a_java(owl_path, iri_sugg, "SuggerimentoPositivo")
            if assiomi_pos:
                self.nodo.get_logger().info(f"✅ POSITIVO: {nome_evento}")
                report_ragionamento.append({
                    "evento": nome_evento,
                    "esito": "CONSIGLIATO",
                    "dettagli": assiomi_pos
                })
                continue # Se è positivo, non controlliamo se è negativo, passiamo al prossimo evento

            assiomi_neg = self.chiedi_spiegazione_a_java(owl_path, iri_sugg, "SuggerimentoNegativo")
            if assiomi_neg:
                self.nodo.get_logger().info(f"❌ NEGATIVO: {nome_evento}")
                report_ragionamento.append({
                    "evento": nome_evento,
                    "esito": "SCONSIGLIATO",
                    "dettagli": assiomi_neg
                })
            else:
                # Caso NEUTRO (né esplicitamente True né False, o ragionamento fallito)
                self.nodo.get_logger().info(f"⚠️ NEUTRO/IGNOTO: {nome_evento}")

        if report_ragionamento:
            msg_input = f"Analisi per l'ospite {self.contesto['ospite'].nome}:\n"
            for item in report_ragionamento:
                msg_input += f"\nEVENTO: {item['evento']}\nESITO: {item['esito']}\nMOTIVAZIONE LOGICA: {item['dettagli']}\n----------------"

            # risposta = llm.handle_request(tipo="explainability", msg=msg_input)
            # self.nodo.parla(risposta)
            self.nodo.parla("DECOMMENTARE LA CHIAMATA LLM 2")
        else:
            self.nodo.parla("Ho analizzato gli eventi ma non ho trovato indicazioni specifiche (né positive né negative).")

    def esegui(self, testo, kb, llm):
        self.nodo.get_logger().info(f"[ScenarioA] Stato: {self.stato}, Input: {testo}")
        if self.stato == "INIZIO":
            self.contesto['ospite'] = Ospite("id", "Peppe", "Rossi", 30, None)  # tornera da unity. puo servire un UnityManager?
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
                interesse_da_salvare = self.contesto.get('interesse')
                if interesse_da_salvare:
                    self.salva_interesse_db(kb, interesse_da_salvare)
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
