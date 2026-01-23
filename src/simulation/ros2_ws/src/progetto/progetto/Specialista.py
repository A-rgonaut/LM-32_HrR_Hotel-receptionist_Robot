class Specialista():
    def chiama(self, nodo, chi_chiamare, riguardo_chi, perche):
        nodo.get_logger().info(f"{chi_chiamare} {riguardo_chi} {perche}")
        #response = nodo.sincro.interrogaGraphDatabase("MATCH (n) RETURN n LIMIT 1")  # INSERT della segnalazione
        # response = nodo.sincro.interrogaGraphDatabase("MATCH (n) RETURN n LIMIT 1")  # SELECT nome, cognome, numero di telefono dello specialista
        # nodo.sincro.ask_llm(msg_input, scenario="C", tipo="specialista") # implementare?
        # nodo.sincro.ask_llm(msg_input, scenario="A", tipo="explainability")
        #nodo.get_logger().info(f"{response}")
        return None
