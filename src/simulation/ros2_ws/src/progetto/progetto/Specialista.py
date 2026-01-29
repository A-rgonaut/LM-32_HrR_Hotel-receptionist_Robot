class Specialista():
    def chiama(self, nodo, chi_chiamare, riguardo_chi, perche):
        
        query = """
            MATCH (s:Specialista)
            WHERE s.specialita = $tipo
            RETURN s
            """
        # Eseguo la query
        risultati = nodo.sincro.interrogaGraphDatabase(query, {'tipo': chi_chiamare})
        nodo.get_logger().info(f"{risultati} {riguardo_chi} {perche}")

        #response = nodo.sincro.interrogaGraphDatabase("MATCH (n) RETURN n LIMIT 1")  # INSERT della segnalazione
        # response = nodo.sincro.interrogaGraphDatabase("MATCH (n) RETURN n LIMIT 1")  # SELECT nome, cognome, numero di telefono dello specialista
        # nodo.sincro.ask_llm(msg_input, scenario="C", tipo="specialista") # implementare?
        # nodo.sincro.ask_llm(msg_input, scenario="A", tipo="explainability")
        #nodo.get_logger().info(f"{response}")
        return None
