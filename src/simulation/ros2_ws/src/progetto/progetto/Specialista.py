class Specialista():
    def chiama(self, nodo, chi_chiamare, riguardo_chi, perche):
        """
        Crea una segnalazione identificando l'ospite per nome e cognome invece che per ID.
        """
        # Sostituiamo WHERE id(o) = $id_ospite con il match diretto sulle proprietà
        query = """
            MATCH (s:Specialista {specialita: $tipo})
            MATCH (o {nome: $nome_o, cognome: $cognome_o})
            CREATE (seg:Segnalazione {
                timestamp: datetime(),
                stato: "aperta",
                motivo: $motivo
            })
            CREATE (s)-[:GESTISCE]->(seg)
            CREATE (seg)-[:RIFERITA_A]->(o)
            RETURN (s.nome + " " + s.cognome) AS nome_completo, s.numero_telefono AS tel
            """
        
        # Mappatura rigorosa dei parametri estratti dall'oggetto riguardo_chi
        parametri = {
            'tipo': chi_chiamare,
            'nome_o': riguardo_chi.nome,    # Preso da o:Ospite {nome: "Peppe", ...} 
            'cognome_o': riguardo_chi.cognome, # Preso da o:Ospite {cognome: "Rossi", ...} 
            'motivo': perche
        }

        risultati = nodo.sincro.interrogaGraphDatabase(query, parametri)

        if risultati:
            sp = risultati[0]
            nodo.get_logger().info(
                f"Segnalazione creata per {riguardo_chi.nome} {riguardo_chi.cognome}. "
                f"Specialista: {sp['nome_completo']}"
            )
            return sp
        return None