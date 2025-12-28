class Persona():
    def __init__(self, id, nome, cognome):
        self.id = id
        self.nome = nome
        self.cognome = cognome

class Ospite(Persona):
    def __init__(self, id, nome, cognome, eta, lingua):
        super().__init__(id, nome, cognome)
        self.eta = eta
        self.lingua = lingua