import json

class Entity():
    def __init__(self, id):
        self.id = id

    def __str__(self):
        return json.dumps(self.__dict__, ensure_ascii=False)

    def __eq__(self, other):
        if isinstance(other, Entity):
            return self.id == other.id
        return False

    def __hash__(self):
        return hash(self.id)

    def __repr__(self):
        return f"Entity({self.id!r})"

class Persona(Entity):
    def __init__(self, id, nome, cognome):
        super().__init__(id)
        self.nome = nome
        self.cognome = cognome

    def __repr__(self):
        return f"Persona({self.id!r}, {self.nome!r}, {self.cognome!r})"

class Ospite(Persona):
    def __init__(self, id, nome, cognome, eta, lingua):
        super().__init__(id, nome, cognome)
        self.eta = eta
        self.lingua = lingua

    def __repr__(self):
        return f"Ospite({self.id!r}, {self.nome!r}, {self.cognome!r}, {self.eta!r}, {self.lingua!r})"
